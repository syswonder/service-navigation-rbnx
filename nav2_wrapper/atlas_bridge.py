#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""nav2_wrapper_rbnx — atlas bridge (driver-init lifecycle).

Wraps system-installed nav2_bringup. Owns service/navigation/*.

Spawn order:
  1. start.sh launches THIS process — no nav2 spawn yet.
  2. main() starts the Driver server plus Navigate/Status/Cancel gRPC
     servicers and MCP tools, then registers with atlas.
  3. rbnx boot calls Driver(CMD_INIT, config_json).
  4. Init handler: pick params_file from config, spawn `ros2 launch
     nav2_bringup navigation_launch.py …`, wait for the navigate_to_pose
     action server to come up, declare navigate/status/cancel on atlas.

NavigateToPose action client uses the existing /odom + /map + /tf the
rest of the stack provides. Goals are tracked in an internal dict so
status() / cancel() work even after the goal has terminated.

Config (passed via Driver(CMD_INIT, config_json)):
    params_profile   default "slam"     → config/nav2_params_<profile>.yml
                                          (slam | sim | default)
    params_file      unset = derive from params_profile (override w/ abs path)
    use_sim_time     default false
    action_wait_s    default 45.0       — nav2 lifecycle takes a while
"""
from __future__ import annotations

import logging
import math
import os
import queue
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import grpc

from nav2_wrapper.diagnostics import classify_nav2_line, format_result_detail

logging.basicConfig(level=os.environ.get("NAV2_LOG_LEVEL", "INFO").upper(),
                    format="[nav2_wrapper] %(message)s")
log = logging.getLogger("nav2_wrapper")


def _pump_output(stream, tag: str) -> None:
    """Forward a child process's merged stdout/stderr into scribe via the
    package logger — one unified log stream, no side-car *.log file."""
    for raw in iter(stream.readline, b""):
        line = raw.decode(errors="replace").rstrip()
        if line:
            log.info("[%s] %s", tag, line)
            if tag == "nav2":
                diagnostic = classify_nav2_line(line)
                if diagnostic:
                    with _state_lock:
                        _nav_diagnostics.append(diagnostic)


def _ensure_proto_gen() -> None:
    d = Path(__file__).resolve().parent
    while d.parent != d:
        codegen = d / "rbnx-build" / "codegen"
        pg = codegen / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            sys.path.insert(0, str(pg))
            mt = codegen / "robonix_mcp_types"
            if mt.is_dir():
                sys.path.insert(0, str(mt))
            return
        d = d.parent


_ensure_proto_gen()

import navigation_pb2  # noqa: E402
import robonix_contracts_pb2_grpc as contracts_grpc  # noqa: E402
import soma_pb2  # noqa: E402
from navigation_mcp import (  # noqa: E402
    Navigate_Request as McpNavigateRequest,
    Navigate_Response as McpNavigateResponse,
    GetNavigationStatus_Request as McpStatusRequest,
    GetNavigationStatus_Response as McpStatusResponse,
    CancelNavigation_Request as McpCancelRequest,
    CancelNavigation_Response as McpCancelResponse,
)

# Current Robonix provider API (same one mapping_rbnx uses). The Service
# class owns atlas registration, the Driver(CMD_INIT/SHUTDOWN) lifecycle
# server, and heartbeat — so this package no longer talks to the raw
# AtlasStub (its old RegisterCapability RPC no longer exists).
from robonix_api import Service, Ok, Err, Deferred, ATLAS  # noqa: E402

CAP_ID = os.environ.get("ROBONIX_CAPABILITY_ID", "nav2")
NAMESPACE = "robonix/service/navigation"

# The provider. on_init (below) does the nav2 bring-up; nav.run() serves
# the Driver lifecycle + registers + heartbeats.
nav = Service(id=CAP_ID, namespace=NAMESPACE)


# ── shared state ─────────────────────────────────────────────────────────────
_state_lock = threading.Lock()
_cap_id: str = CAP_ID
_pkg_root: Path = Path(__file__).resolve().parent.parent
_nav2_proc: subprocess.Popen | None = None
_velocity_guard_proc: subprocess.Popen | None = None
_scan_projector_proc: subprocess.Popen | None = None
_scan_deskew_proc: subprocess.Popen | None = None
_scan_filter_proc: subprocess.Popen | None = None
_initialized = False

# ROS2 client state (initialized inside Driver.Init after nav2 is alive)
_ros_node = None
_nav_action_client = None
_nav_action_ready = False
_NavigateToPose = None
_PoseStamped = None
_GoalStatus = None
_nav_queue: "queue.Queue[tuple[str, dict]]" = queue.Queue()
_goal_states: dict[str, dict] = {}
_goal_handles: dict[str, object] = {}
_nav_diagnostics: deque[str] = deque(maxlen=12)
_last_run_id = ""
# Whether nav2 (and therefore the TF tree it consumes) runs on /clock sim
# time. The wrapper's own rclpy node must match: it stamps goal poses with
# node.get_clock().now(), and if that clock is wall time while map->odom TF
# is published on sim time, every goal lookup hits a ~decades extrapolation
# error and the planner aborts. Set from cfg in init().
_USE_SIM_TIME = False


def _import_ros2() -> None:
    global _NavigateToPose, _PoseStamped, _GoalStatus
    from geometry_msgs.msg import PoseStamped as RosPoseStamped  # type: ignore
    from nav2_msgs.action import NavigateToPose  # type: ignore
    try:
        from action_msgs.msg import GoalStatus  # type: ignore
        _GoalStatus = GoalStatus
    except ImportError:
        _GoalStatus = None
    _PoseStamped = RosPoseStamped
    _NavigateToPose = NavigateToPose


# ── atlas-driven dependency discovery ────────────────────────────────────────
# nav2 needs a few upstream data streams. We DO NOT hardcode which package
# provides them — we ask atlas for each contract and remap the topic into
# nav2 at launch time. This keeps the wrapper coupled to contracts only;
# whoever publishes them on this deploy is irrelevant.
#
# (config_key, contract_id, default_remap_target) — config_key is the
# string we look up in `cfg["topic_remap"]` so an operator can override
# any individual binding without disabling discovery.
_REQUIRED_DEPS: tuple[tuple[str, str, str], ...] = (
    # robonix/service/map/occupancy_grid → nav2 expects /map for the
    # global costmap's StaticLayer.
    ("map",   "robonix/service/map/occupancy_grid",  "/map"),
    # Consume the deployment's canonical odometry provider directly. Mapping
    # also consumes this stream, but must not re-declare the same ROS topic as
    # a second capability owner. Ranger currently pins this to ranger_chassis;
    # a future Mid360 LIO package can replace it through provider_ids.odom.
    ("odom",  "robonix/primitive/chassis/odom",       "/odom"),
)

# Optional deps: if present on atlas, we wire them; if absent, nav2 still
# launches and just won't have that observation source. Useful when the
# deploy has e.g. a 3D lidar but nav2's costmap is configured around 2D
# scan — the operator may legitimately not provide one.
_OPTIONAL_DEPS: tuple[tuple[str, str, str], ...] = (
    # 2D scan for ObstacleLayer (some configs); 3D lidar for VoxelLayer.
    ("scan",        "robonix/primitive/lidar/lidar",   "/scan"),
    ("scan_cloud",  "robonix/primitive/lidar/lidar3d", "/scanner/cloud"),
)


def _resolve_dep(contract_id: str, provider_id: str = "") -> str | None:
    """Ask atlas which ROS2 topic backs `contract_id`; return it or None.

    Uses the same ATLAS.find_capability + connect_capability path mapping
    uses — we want the resolved topic string, and connecting also records
    nav2 as a consumer of that contract. The Channel is closed immediately."""
    recs = ATLAS.find_capability(
        contract_id=contract_id, transport="ros2", provider_id=provider_id
    )
    if not recs:
        return None
    rec = recs[0]
    try:
        ch = nav.connect_capability(rec, contract_id=contract_id, transport="ros2")
    except Exception as e:  # noqa: BLE001
        log.warning("connect %s/%s failed: %s", rec.provider_id, contract_id, e)
        return None
    endpoint = (ch.endpoint or "").strip()
    ch.close()
    return endpoint or None


def _build_remap_args(cfg: dict) -> tuple[list[str], list[str]]:
    """Return (remap_args, missing_required).
    remap_args is a list of `from:=to` strings ready to pass to ros2 launch.
    missing_required is a list of contract_ids that should have been there
    but weren't — caller decides whether to defer / degrade / fail."""
    overrides = dict(cfg.get("topic_remap", {}) or {})
    providers = dict(cfg.get("provider_ids", {}) or {})
    remap_args: list[str] = []
    missing: list[str] = []

    for key, contract_id, default_target in _REQUIRED_DEPS:
        if key in overrides:
            ep = str(overrides[key])
        else:
            ep = _resolve_dep(contract_id, str(providers.get(key) or "")) or ""
        if not ep:
            missing.append(contract_id)
            continue
        # ros2 launch syntax: pass remaps via the ros-args mechanism. The
        # nav2_bringup composable nodes pick them up via DeclareLaunchArgument.
        # Cleanest path: rewrite a temp params file with the resolved topic
        # name (the params YAML is where most nav2 nodes look for it).
        remap_args.append(f"{key}:={ep}")
        log.info("resolved %s → %s = %s", contract_id, default_target, ep)

    for key, contract_id, default_target in _OPTIONAL_DEPS:
        if key in overrides:
            ep = str(overrides[key])
        else:
            ep = _resolve_dep(contract_id, str(providers.get(key) or "")) or ""
        if ep:
            remap_args.append(f"{key}:={ep}")
            log.info("resolved (optional) %s → %s = %s", contract_id, default_target, ep)
        else:
            log.info("optional dep %s not on atlas — skipping", contract_id)

    return remap_args, missing


def _binding_value(bindings: list[str], key: str) -> str:
    """Return the resolved Atlas endpoint for a named dependency binding."""
    prefix = f"{key}:="
    for binding in bindings:
        if binding.startswith(prefix):
            return binding[len(prefix):]
    return ""


def _uses_projected_scan(cfg: dict) -> bool:
    """Whether this profile requires a LaserScan but may only have lidar3d."""
    return not cfg.get("params_file") and cfg.get("params_profile", "slam") == "ranger_mini_v3"


def _kill_scan_projector() -> None:
    global _scan_projector_proc, _scan_deskew_proc, _scan_filter_proc
    procs = (_scan_projector_proc, _scan_deskew_proc, _scan_filter_proc)
    _scan_projector_proc = None
    _scan_deskew_proc = None
    _scan_filter_proc = None
    for proc in procs:
        if proc is None:
            continue
        try:
            # ros2 run is a Python parent that spawns the actual ROS binary.
            # The parent may exit while its child still owns this session's
            # process group, so target the known PGID directly.
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            continue
        if proc.poll() is None:
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def _prepare_scan(cfg: dict, bindings: list[str]) -> list[str]:
    """Ensure the Ranger 2D ObstacleLayer has a LaserScan source.

    A native lidar contract wins when available. Ranger Mini otherwise exposes
    the standard lidar3d contract, so this package owns a pointcloud-to-scan
    child instead of pushing hardware-specific topic plumbing into the deploy.
    """
    global _scan_projector_proc, _scan_deskew_proc, _scan_filter_proc
    if not _uses_projected_scan(cfg) or _binding_value(bindings, "scan"):
        return bindings

    cloud_topic = _binding_value(bindings, "scan_cloud")
    if not cloud_topic:
        raise RuntimeError(
            "ranger_mini_v3 requires robonix/primitive/lidar/lidar or "
            "robonix/primitive/lidar/lidar3d"
        )
    if _scan_projector_proc is not None and _scan_projector_proc.poll() is None:
        return [*bindings, "scan:=/scanner/scan"]

    _, footprint_radius = _soma_footprint_info()
    range_min = footprint_radius + float(cfg.get("scan_self_filter_margin_m", 0.05))
    projector_cloud_topic = cloud_topic
    if bool(cfg.get("scan_deskewing", False)):
        projector_cloud_topic = f"{cloud_topic.rstrip('/')}/deskewed"
        deskew_args = [
            "ros2", "run", "rtabmap_util", "lidar_deskewing",
            "--ros-args",
            "-r", "__node:=robonix_nav_lidar_deskewing",
            "-r", f"input_cloud:={cloud_topic}",
            "-p", f"fixed_frame_id:={cfg.get('odom_frame', 'odom')}",
            "-p", "wait_for_transform:=0.2",
            "-p", "slerp:=true",
        ]
        try:
            _scan_deskew_proc = subprocess.Popen(
                deskew_args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except FileNotFoundError as exc:
            raise RuntimeError(
                "scan_deskewing requires ros-humble-rtabmap-util"
            ) from exc
        threading.Thread(
            target=_pump_output,
            args=(_scan_deskew_proc.stdout, "lidar_deskewing"),
            daemon=True,
        ).start()
        time.sleep(0.25)
        if _scan_deskew_proc.poll() is not None:
            _kill_scan_projector()
            raise RuntimeError("lidar_deskewing exited during startup")

    args = [
        "ros2", "run", "pointcloud_to_laserscan", "pointcloud_to_laserscan_node",
        "--ros-args",
        "-r", "__node:=robonix_pointcloud_to_laserscan",
        "-r", f"cloud_in:={projector_cloud_topic}",
        "-r", "scan:=/scanner/scan_raw",
        "-p", "target_frame:=base_link",
        "-p", "transform_tolerance:=0.15",
        "-p", "min_height:=0.30",
        "-p", "max_height:=1.40",
        "-p", f"range_min:={range_min:.3f}",
        "-p", "range_max:=12.0",
        "-p", "use_inf:=true",
    ]
    try:
        _scan_projector_proc = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except FileNotFoundError as exc:
        _kill_scan_projector()
        raise RuntimeError(
            "pointcloud_to_laserscan is required for ranger_mini_v3; install "
            "ros-humble-pointcloud-to-laserscan"
        ) from exc
    threading.Thread(
        target=_pump_output, args=(_scan_projector_proc.stdout, "pointcloud_to_laserscan"),
        daemon=True,
    ).start()
    time.sleep(0.25)
    if _scan_projector_proc.poll() is not None:
        _kill_scan_projector()
        raise RuntimeError("pointcloud_to_laserscan exited during startup")
    _scan_filter_proc = subprocess.Popen(
        [sys.executable, "-m", "nav2_wrapper.scan_filter"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(
        target=_pump_output, args=(_scan_filter_proc.stdout, "scan_filter"), daemon=True
    ).start()
    time.sleep(0.25)
    if _scan_filter_proc.poll() is not None:
        _kill_scan_projector()
        raise RuntimeError("scan speckle filter exited during startup")
    log.info(
        "projecting %s to /scanner/scan_raw then filtered /scanner/scan "
        "for Ranger Nav2 (deskew=%s, self-filter range_min=%.3fm)",
        projector_cloud_topic,
        bool(cfg.get("scan_deskewing", False)),
        range_min,
    )
    return [*bindings, "scan:=/scanner/scan"]


# ── nav2 subprocess management ───────────────────────────────────────────────
def _resolve_params_file(cfg: dict) -> str:
    explicit = cfg.get("params_file")
    if explicit:
        p = Path(explicit)
        if not p.is_absolute():
            p = _pkg_root / p
        if not p.is_file():
            raise FileNotFoundError(f"params_file not found: {p}")
        return str(p)
    profile = cfg.get("params_profile", "slam")
    candidates = {
        "slam":           _pkg_root / "config" / "nav2_params_slam.yml",
        "ranger_mini_v3": _pkg_root / "config" / "nav2_params_ranger_mini_v3.yml",
        "sim":            _pkg_root / "config" / "nav2_params_sim.yml",
        "default":        _pkg_root / "config" / "nav2_params.yml",
    }
    p = candidates.get(profile)
    if p is None:
        raise ValueError(f"unknown params_profile {profile!r}; "
                         f"options: {list(candidates)}")
    if not p.is_file():
        raise FileNotFoundError(f"params file for profile {profile!r} missing: {p}")
    return str(p)


_footprint_cache: tuple[str, float] | None = None


def _soma_footprint_info() -> tuple[str, float]:
    """Resolve the footprint polygon and circumscribed radius through Soma."""
    global _footprint_cache
    if _footprint_cache is not None:
        return _footprint_cache
    contract_id = "robonix/system/soma/footprint"
    records = ATLAS.find_capability(contract_id=contract_id, transport="grpc")
    if not records:
        raise RuntimeError(f"required capability unavailable: {contract_id}")

    connection = nav.connect_capability(
        records[0], contract_id=contract_id, transport="grpc"
    )
    endpoint = (connection.endpoint or "").strip()
    connection.close()
    if not endpoint:
        raise RuntimeError(f"{contract_id} resolved to an empty endpoint")

    channel = grpc.insecure_channel(endpoint)
    try:
        grpc.channel_ready_future(channel).result(timeout=10)
        response = contracts_grpc.RobonixSystemSomaFootprintStub(channel).GetFootprint(
            soma_pb2.GetFootprint_Request(), timeout=10
        )
    finally:
        channel.close()

    if not response.base_frame:
        raise ValueError("Soma footprint response has no base_frame")
    if len(response.points) < 3:
        raise ValueError("Soma footprint requires at least three polygon points")
    points = []
    radius = 0.0
    for point in response.points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            raise ValueError("Soma footprint contains non-finite point coordinates")
        points.append(f"[{point.x:.6f}, {point.y:.6f}]")
        radius = max(radius, math.hypot(point.x, point.y))
    value = "[ " + ", ".join(points) + " ]"
    log.info(
        "resolved Soma footprint base_frame=%s vertices=%d inscribed=%.3fm",
        response.base_frame,
        len(response.points),
        response.inscribed_radius_m,
    )
    _footprint_cache = (value, radius)
    return _footprint_cache


def _soma_footprint() -> str:
    return _soma_footprint_info()[0]


def _materialize_params(cfg: dict, bindings: list[str]) -> tuple[str, list[str]]:
    """Fill Atlas topic and Soma body tokens in target-specific profiles."""
    source = Path(_resolve_params_file(cfg))
    text = source.read_text(encoding="utf-8")
    if "__ROBONIX_" not in text:
        return str(source), bindings

    resolved = {}
    for item in bindings:
        key, sep, value = item.partition(":=")
        if sep:
            resolved[key] = value

    replacements = {
        "__ROBONIX_MAP_TOPIC__": resolved.get("map", ""),
        "__ROBONIX_ODOM_TOPIC__": resolved.get("odom", ""),
        "__ROBONIX_SCAN_TOPIC__": resolved.get("scan", ""),
        "__ROBONIX_SCAN_CLOUD_TOPIC__": resolved.get("scan_cloud", ""),
        "__ROBONIX_BT_XML__": str(
            _pkg_root / "config" / "ranger_mini_v3_navigate.xml"
        ),
    }
    if "__ROBONIX_FOOTPRINT__" in text:
        replacements["__ROBONIX_FOOTPRINT__"] = _soma_footprint()

    for token, value in replacements.items():
        if token in text:
            if not value:
                raise RuntimeError(f"cannot materialize {source.name}: {token} is unresolved")
            text = text.replace(token, value)

    unresolved = sorted(set(re.findall(r"__ROBONIX_[A-Z_]+__", text)))
    if unresolved:
        raise RuntimeError(
            f"cannot materialize {source.name}: unresolved tokens {unresolved}"
        )

    runtime_dir = _pkg_root / "rbnx-build" / "runtime"
    runtime_dir.mkdir(parents=True, exist_ok=True)
    target = runtime_dir / f"nav2_params_{_cap_id}.yaml"
    target.write_text(text, encoding="utf-8")
    log.info("materialized nav2 params %s -> %s", source, target)
    # Target-specific profiles consume Atlas bindings inside the generated
    # params file. They must not be passed as undeclared launch arguments.
    return str(target), []


def _materialize_guarded_launch() -> str:
    """Patch the distro launch so every Nav2 velocity crosses our final guard."""
    from ament_index_python.packages import get_package_share_directory  # type: ignore

    source = Path(get_package_share_directory("nav2_bringup")) / "launch" / "navigation_launch.py"
    text = source.read_text(encoding="utf-8")
    old_behavior = "remappings=remappings)"
    behavior_marker = "package='nav2_behaviors'"
    search_from = 0
    for _ in range(2):
        behavior_start = text.index(behavior_marker, search_from)
        behavior_end = text.index("package='nav2_bt_navigator'", behavior_start)
        behavior = text[behavior_start:behavior_end]
        if behavior.count(old_behavior) != 1:
            raise RuntimeError("unsupported nav2 behavior_server launch layout")
        behavior = behavior.replace(
            old_behavior,
            "remappings=remappings + [('cmd_vel', 'cmd_vel_guard_input')])",
        )
        text = text[:behavior_start] + behavior + text[behavior_end:]
        search_from = behavior_end
    old_smoother = "[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')])"
    new_smoother = "[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel_guard_input')])"
    if text.count(old_smoother) != 2:
        raise RuntimeError("unsupported nav2 velocity_smoother launch layout")
    text = text.replace(old_smoother, new_smoother)
    target = _pkg_root / "rbnx-build" / "runtime" / "guarded_navigation_launch.py"
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding="utf-8")
    return str(target)


def _spawn_velocity_guard(cfg: dict) -> None:
    global _velocity_guard_proc
    env = os.environ.copy()
    env.update({
        "ROBONIX_NAV_TRACE_DIR": str(
            cfg.get("trajectory_log_dir", _pkg_root / "rbnx-build" / "data" / "trajectories")
        ),
        "ROBONIX_GUARD_TERMINAL_XY_M": str(cfg.get("guard_terminal_xy_m", 0.45)),
        "ROBONIX_GUARD_TERMINAL_TIMEOUT_S": str(cfg.get("guard_terminal_timeout_s", 15.0)),
        "ROBONIX_GUARD_NO_PROGRESS_S": str(cfg.get("guard_no_progress_s", 3.0)),
        "ROBONIX_GUARD_GLOBAL_TIMEOUT_S": str(cfg.get("guard_global_spin_timeout_s", 25.0)),
        "ROBONIX_GUARD_GLOBAL_ROTATION_RAD": str(cfg.get("guard_global_spin_limit_rad", 6.783185307)),
    })
    _velocity_guard_proc = subprocess.Popen(
        [sys.executable, "-m", "nav2_wrapper.velocity_guard"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        env=env,
    )
    threading.Thread(
        target=_pump_output, args=(_velocity_guard_proc.stdout, "velocity_guard"), daemon=True
    ).start()


def _spawn_nav2(cfg: dict, remap_args: list[str]) -> None:
    global _nav2_proc
    params_file, launch_remaps = _materialize_params(cfg, remap_args)
    _spawn_velocity_guard(cfg)
    launch_file = _materialize_guarded_launch()
    use_sim_time = "true" if cfg.get("use_sim_time", False) else "false"
    args = [
        "ros2", "launch", launch_file,
        f"use_sim_time:={use_sim_time}",
        f"params_file:={params_file}",
    ]
    # Topic remaps from atlas resolution arrive as launch-arg-shaped
    # `<key>:=<resolved-topic>` pairs. The launch file translates them
    # into ros2 remap ops via `<set_remap>` blocks; for keys the launch
    # doesn't know about we still pass them — no-op if unused. (Future:
    # rewrite the params YAML with substitutions for nodes that read
    # topic names from params rather than via remap.)
    args.extend(launch_remaps)
    log.info("spawning nav2 (params=%s, remaps=%s)", params_file, launch_remaps)
    _nav2_proc = subprocess.Popen(
        args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    threading.Thread(target=_pump_output, args=(_nav2_proc.stdout, "nav2"),
                     daemon=True).start()


def _kill_nav2() -> None:
    global _velocity_guard_proc
    _kill_scan_projector()
    p = _nav2_proc
    if p is not None and p.poll() is None:
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            p.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    guard = _velocity_guard_proc
    _velocity_guard_proc = None
    if guard is not None and guard.poll() is None:
        try:
            os.killpg(os.getpgid(guard.pid), signal.SIGTERM)
            guard.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(guard.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass


# ── ROS2 wiring (started after nav2 is alive) ────────────────────────────────
def _start_ros2_thread() -> None:
    """Spin a rclpy node + ActionClient. Re-entrant: only acts once."""
    def _run():
        global _ros_node, _nav_action_client, _nav_action_ready
        import rclpy  # type: ignore
        from rclpy.executors import MultiThreadedExecutor  # type: ignore
        from rclpy.action import ActionClient  # type: ignore
        from rclpy.parameter import Parameter  # type: ignore

        rclpy.init(args=None)
        # use_sim_time must match nav2 / the TF tree (see _USE_SIM_TIME) so
        # node.get_clock() — which timestamps goal poses — is on the same
        # clock domain as the map->odom transform.
        node = rclpy.create_node(
            "nav2_wrapper_atlas_bridge",
            parameter_overrides=[
                Parameter("use_sim_time", Parameter.Type.BOOL, _USE_SIM_TIME)
            ],
        )
        _ros_node = node
        _import_ros2()
        _nav_action_client = ActionClient(node, _NavigateToPose, "navigate_to_pose")
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        log.info("rclpy node up; waiting on navigate_to_pose action server")
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            # Drain goals queued by Navigate gRPC handler.
            while True:
                try:
                    gid, payload = _nav_queue.get_nowait()
                except queue.Empty:
                    break
                _dispatch_goal(node, gid, payload)
    threading.Thread(target=_run, daemon=True).start()


def _wait_for_action(timeout_s: float) -> bool:
    """Block until `navigate_to_pose` is ready (post-Init nav2 lifecycle bring-up)."""
    global _nav_action_ready
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if _nav_action_client is not None and _nav_action_client.wait_for_server(timeout_sec=0.5):
            _nav_action_ready = True
            return True
        time.sleep(0.5)
    return False


def _make_pose(node, frame_id: str, x: float, y: float, yaw: float):
    g = _PoseStamped()
    g.header.frame_id = frame_id
    # Leave the goal stamp at 0 (the message default). A zero stamp tells
    # tf2 "use the LATEST available transform" instead of requiring the
    # frame_id->costmap transform at one exact instant. This is the
    # environment-agnostic fix for the "goal aborts with Extrapolation
    # Error" failure:
    #   - webots/sim: node clock and the map->odom TF can sit in different
    #     clock domains; a "now()" stamp lands decades away from the TF.
    #   - real robot: use_sim_time is false, but sensor/SLAM TF still lags
    #     wall-clock "now" by tens of ms, so a "now()" stamp can fall past
    #     the newest TF and extrapolate into the future.
    # Stamping 0 sidesteps both — the planner transforms against whatever
    # TF is currently buffered, which is exactly what "go to this pose"
    # means. (node is kept use_sim_time-consistent for action timing, but
    # the goal transform no longer depends on clock alignment at all.)
    g.pose.position.x = float(x)
    g.pose.position.y = float(y)
    g.pose.position.z = 0.0
    g.pose.orientation.z = math.sin(yaw / 2.0)
    g.pose.orientation.w = math.cos(yaw / 2.0)
    return g


def _goal_status_name(status: int) -> str:
    if _GoalStatus is None:
        return str(int(status))
    g = _GoalStatus
    m = {
        int(g.STATUS_UNKNOWN):  "UNKNOWN",
        int(g.STATUS_ACCEPTED): "ACCEPTED",
        int(g.STATUS_EXECUTING): "EXECUTING",
        int(g.STATUS_CANCELING): "CANCELING",
        int(g.STATUS_SUCCEEDED): "SUCCEEDED",
        int(g.STATUS_CANCELED): "CANCELED",
        int(g.STATUS_ABORTED):  "ABORTED",
    }
    return m.get(int(status), str(int(status)))


def _canonical_state(nav2_state: str) -> str:
    """Map Nav2 action result/status names to executor async state names."""
    return {
        "UNKNOWN": "RUNNING",
        "ACCEPTED": "RUNNING",
        "EXECUTING": "RUNNING",
        "CANCELING": "RUNNING",
        "SUCCEEDED": "SUCCEEDED",
        "CANCELED": "CANCELED",
        "ABORTED": "FAILED",
    }.get(nav2_state, "FAILED")


def _resolve_run_id(run_id: str) -> str:
    """Use the explicit run id, or fall back to the most recent navigation run."""
    return run_id or _last_run_id


def _goal_response_cb(fut, gid: str):
    try:
        gh = fut.result()
    except Exception as e:  # noqa: BLE001
        with _state_lock:
            _goal_states[gid] = {"state": "FAILED", "detail": str(e)}
        return
    if not gh.accepted:
        with _state_lock:
            _goal_states[gid] = {"state": "FAILED", "detail": "goal rejected"}
        return
    with _state_lock:
        _goal_handles[gid] = gh
        _goal_states[gid] = {"state": "RUNNING", "detail": "goal accepted"}
    res_fut = gh.get_result_async()
    res_fut.add_done_callback(lambda f: _result_cb(f, gid))


def _feedback_cb(message, gid: str) -> None:
    """Keep progress context because Humble NavigateToPose has an empty result."""
    feedback = message.feedback
    pose = feedback.current_pose.pose.position
    summary = {
        "distance_remaining": float(feedback.distance_remaining),
        "recoveries": int(feedback.number_of_recoveries),
        "x": float(pose.x),
        "y": float(pose.y),
    }
    with _state_lock:
        state = _goal_states.get(gid)
        if state is not None:
            state["feedback"] = summary


def _result_cb(fut, gid: str):
    try:
        res = fut.result()
        st_name = _goal_status_name(getattr(res, "status", -1))
        state = _canonical_state(st_name)
        with _state_lock:
            previous = _goal_states.get(gid, {})
            diagnostics = list(_nav_diagnostics) if state == "FAILED" else []
            _goal_states[gid] = {
                "state": state,
                "detail": format_result_detail(
                    st_name, previous.get("feedback"), diagnostics
                ),
            }
            _goal_handles.pop(gid, None)
    except Exception as e:  # noqa: BLE001
        with _state_lock:
            _goal_states[gid] = {"state": "FAILED", "detail": str(e)}
            _goal_handles.pop(gid, None)


def _dispatch_goal(node, gid: str, payload: dict):
    pose = _make_pose(node, payload["frame_id"], payload["x"], payload["y"], payload["yaw"])
    goal_msg = _NavigateToPose.Goal()
    goal_msg.pose = pose
    if _nav_action_client is None or not _nav_action_ready:
        with _state_lock:
            _goal_states[gid] = {"state": "FAILED",
                                 "detail": "nav action server not ready"}
        return
    with _state_lock:
        _nav_diagnostics.clear()
    send_future = _nav_action_client.send_goal_async(
        goal_msg, feedback_callback=lambda msg, g=gid: _feedback_cb(msg, g)
    )
    send_future.add_done_callback(lambda f, g=gid: _goal_response_cb(f, g))
    with _state_lock:
        _goal_states[gid] = {"state": "RUNNING", "detail": "goal sent"}


# ── lifecycle (Driver CMD_INIT / CMD_SHUTDOWN via robonix_api.Service) ────────
@nav.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. rbnx delivers the deploy config here via
    Driver(CMD_INIT, config_json) — the only sanctioned config path (never
    disk / env). Brings up nav2:

      1. Resolve upstream deps (map, odom, optional scan/cloud) from atlas
         by contract — never hardcoded topic names. Missing a REQUIRED dep
         → Deferred (rbnx retries once the upstream provider registers),
         so we never spawn a half-wired nav2.
      2. Spawn nav2_bringup with the resolved remaps + the params profile.
      3. Bring up the rclpy node + navigate_to_pose ActionClient and wait
         for nav2's lifecycle to advertise the action server.

    The navigate/status/cancel gRPC and MCP interfaces are hosted + declared by
    run() (see attach_grpc_servicer below); each guards on `_ros_node`, so
    a call landing before nav2 is ready returns a clean 'not initialized'."""
    global _initialized
    with _state_lock:
        if _initialized:
            return Ok()

    action_wait = float(cfg.get("action_wait_s", 45.0))

    remap_args, missing = _build_remap_args(cfg)
    if missing:
        return Deferred(
            f"missing required atlas contracts: {missing} "
            f"(awaiting upstream provider)"
        )

    global _USE_SIM_TIME
    _USE_SIM_TIME = bool(cfg.get("use_sim_time", False))

    try:
        _spawn_nav2(cfg, _prepare_scan(cfg, remap_args))
    except Exception as e:  # noqa: BLE001
        _kill_nav2()
        return Err(f"spawn nav2 failed: {e}")

    _start_ros2_thread()
    if not _wait_for_action(action_wait):
        # A failed Driver.Init must not orphan controller, scan, or guard
        # process groups after rbnx reports the package as failed.
        _kill_nav2()
        return Err(
            f"navigate_to_pose action server did not come up within {action_wait:.1f}s"
        )

    with _state_lock:
        _initialized = True
    log.info("init complete: nav2 alive, navigate/status/cancel serving")
    return Ok()


@nav.on_shutdown
def shutdown():
    """INACTIVE/ACTIVE → TERMINATED. Tear the nav2 subprocess down so it
    doesn't outlive the wrapper. Best-effort; never fails shutdown."""
    _kill_nav2()
    return Ok()


# ── navigation RPC/MCP shared implementation ─────────────────────────────────
def _quat_to_yaw(z: float, w: float) -> float:
    return 2.0 * math.atan2(z, w)


def _navigate_impl(goal) -> dict:
    """Queue a Nav2 goal and return the contract-level response fields."""
    global _last_run_id
    if _ros_node is None:
        return {"accepted": False, "run_id": "", "detail": "ROS2 not initialized"}
    run_id = str(uuid.uuid4())
    frame_id = goal.header.frame_id or "map"
    yaw = _quat_to_yaw(goal.pose.orientation.z, goal.pose.orientation.w)
    _nav_queue.put((run_id, {
        "frame_id": frame_id,
        "x": float(goal.pose.position.x),
        "y": float(goal.pose.position.y),
        "yaw": float(yaw),
    }))
    with _state_lock:
        _last_run_id = run_id
        _goal_states[run_id] = {"state": "PENDING", "detail": "queued"}
    return {"accepted": True, "run_id": run_id, "detail": "queued"}


def _status_impl(run_id: str) -> dict:
    """Return state/detail for an explicit run id, or the most recent run."""
    with _state_lock:
        resolved = _resolve_run_id(run_id)
        st = _goal_states.get(resolved)
    if st is None:
        return {"known": False, "state": "FAILED", "detail": "unknown run_id"}
    return {
        "known": True,
        "state": st.get("state", "FAILED"),
        "detail": st.get("detail", ""),
    }


def _cancel_impl(run_id: str) -> dict:
    """Cancel an explicit run id, or the most recent active navigation run."""
    with _state_lock:
        resolved = _resolve_run_id(run_id)
        gh = _goal_handles.get(resolved)
    if gh is None:
        return {"accepted": False, "detail": "no active goal handle"}
    try:
        gh.cancel_goal_async()  # type: ignore[union-attr]
    except Exception as e:  # noqa: BLE001
        return {"accepted": False, "detail": f"cancel failed: {e}"}
    return {"accepted": True, "detail": "cancel requested"}


# ── gRPC servicers ───────────────────────────────────────────────────────────
class _NavigateServicer(contracts_grpc.RobonixServiceNavigationNavigateServicer):
    def Navigate(self, request, context):
        out = _navigate_impl(request.goal)
        return navigation_pb2.Navigate_Response(**out)


class _StatusServicer(contracts_grpc.RobonixServiceNavigationNavigateStatusServicer):
    def GetNavigationStatus(self, request, context):
        out = _status_impl(request.run_id)
        return navigation_pb2.GetNavigationStatus_Response(**out)


class _CancelServicer(contracts_grpc.RobonixServiceNavigationNavigateCancelServicer):
    def CancelNavigation(self, request, context):
        out = _cancel_impl(request.run_id)
        return navigation_pb2.CancelNavigation_Response(**out)


# ── MCP tools ────────────────────────────────────────────────────────────────
@nav.mcp("robonix/service/navigation/navigate")
def navigate(req: McpNavigateRequest) -> McpNavigateResponse:
    """Start a Nav2 NavigateToPose run. Returns a run_id for status/cancel."""
    out = _navigate_impl(req.goal)
    if not out["accepted"]:
        raise RuntimeError(out["detail"])
    return McpNavigateResponse(**out)


@nav.mcp("robonix/service/navigation/navigate/status")
def status(req: McpStatusRequest) -> McpStatusResponse:
    """Poll a navigation run. Empty run_id means the most recent run."""
    out = _status_impl(req.run_id)
    if not out["known"]:
        raise RuntimeError(out["detail"])
    return McpStatusResponse(**out)


@nav.mcp("robonix/service/navigation/navigate/cancel")
def cancel(req: McpCancelRequest) -> McpCancelResponse:
    """Cancel a navigation run. Empty run_id means the most recent active run."""
    out = _cancel_impl(req.run_id)
    if not out["accepted"]:
        raise RuntimeError(out["detail"])
    return McpCancelResponse(**out)


# ── attach the navigate/status/cancel gRPC servicers ─────────────────────────
# run() hosts these on the same auto-allocated port as the Driver lifecycle
# and atlas-declares each by contract. They're live from bootstrap; each one
# guards on `_ros_node`, so a call before CMD_INIT finishes returns a clean
# 'not initialized' rather than crashing.
nav.attach_grpc_servicer("robonix/service/navigation/navigate", _NavigateServicer())
nav.attach_grpc_servicer("robonix/service/navigation/navigate/status", _StatusServicer())
nav.attach_grpc_servicer("robonix/service/navigation/navigate/cancel", _CancelServicer())


def main() -> int:
    """Blocking. Service.run() registers nav2 with atlas, serves the Driver
    lifecycle + navigate/status/cancel gRPC/MCP, heartbeats, and dispatches
    CMD_INIT/CMD_SHUTDOWN to the on_init / on_shutdown callbacks above."""
    nav.run()
    return 0


if __name__ == "__main__":
    main()
