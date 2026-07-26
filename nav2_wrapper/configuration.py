"""Deployment-owned configuration helpers for the Nav2 wrapper."""

from __future__ import annotations

import os
import logging
import re
from pathlib import Path
from typing import Mapping


log = logging.getLogger("nav2_wrapper")
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
LEGACY_ROOT = Path(__file__).resolve().parent / "legacy_config"
LEGACY_PROFILE_FILES = {
    "default": "nav2_params.yml",
    "slam": "nav2_params_slam.yml",
    "sim": "nav2_params_sim.yml",
    "ranger_mini_v3": "nav2_params_ranger_mini_v3.yml",
}

VELOCITY_OUTPUT_TOPIC_ENV = "ROBONIX_VELOCITY_OUTPUT_TOPIC"
DEFAULT_VELOCITY_OUTPUT_TOPIC = "/cmd_vel"
EXTERNAL_VELOCITY_GUARD_INPUT_TOPIC = "/cmd_vel_guard_input"


def resolve_use_composition(cfg: dict) -> bool:
    """Return the explicit composition choice without truthy coercion."""
    value = cfg.get("use_composition", False)
    if not isinstance(value, bool):
        raise ValueError("use_composition must be a boolean")
    return value


def resolve_external_velocity_guard(cfg: dict) -> bool:
    """Select a deployment-owned final guard without weakening defaults.

    The historical provider-owned velocity guard remains the default.  An
    external guard is opt-in and must consume the one exact private input
    topic; accepting an arbitrary destination here could bypass that guard.
    """
    value = cfg.get("external_velocity_guard", False)
    if not isinstance(value, bool):
        raise ValueError("external_velocity_guard must be a boolean")
    if (
        value
        and resolve_velocity_output_topic(cfg)
        != EXTERNAL_VELOCITY_GUARD_INPUT_TOPIC
    ):
        raise ValueError(
            "external_velocity_guard requires velocity_output_topic "
            f"{EXTERNAL_VELOCITY_GUARD_INPUT_TOPIC!r}"
        )
    return value


def render_python_expression_bool(value: bool) -> str:
    """Render a bool as a Python literal for ROS launch PythonExpression.

    Nav2 Humble evaluates ``use_composition`` in a ``PythonExpression``.  Its
    usual lower-case ROS boolean spelling would therefore be parsed as an
    undefined Python name instead of a boolean literal.
    """
    if not isinstance(value, bool):
        raise ValueError("launch PythonExpression value must be a boolean")
    return "True" if value else "False"


def deployment_root() -> Path:
    """Return the directory containing the active robot manifest."""
    raw = os.environ.get("RBNX_INVOCATION_CWD", "").strip()
    return Path(raw).expanduser().resolve() if raw else Path.cwd().resolve()


def resolve_deployment_file(value: object, field: str) -> Path:
    raw = str(value or "").strip()
    if not raw:
        raise ValueError(f"navigation config requires {field}")
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = deployment_root() / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"{field} not found: {path}")
    return path


def resolve_params_file(cfg: dict) -> Path:
    profile = str(cfg.get("params_profile") or "").strip()
    if profile:
        filename = LEGACY_PROFILE_FILES.get(profile)
        if filename is None:
            raise ValueError(
                f"unknown legacy params_profile {profile!r}; "
                f"known values: {sorted(LEGACY_PROFILE_FILES)}"
            )
        log.warning(
            "DEPRECATED config.params_profile=%s; copy %s into the robot "
            "deploy repository and use config.params_file instead",
            profile,
            filename,
        )
        if cfg.get("params_file"):
            log.warning("config.params_file overrides deprecated params_profile")
        else:
            return (LEGACY_ROOT / filename).resolve()
    return resolve_deployment_file(cfg.get("params_file"), "params_file")


def resolve_bt_xml_file(cfg: dict) -> Path | None:
    raw = cfg.get("bt_xml_file")
    if raw:
        return resolve_deployment_file(raw, "bt_xml_file")
    if str(cfg.get("params_profile") or "").strip() == "ranger_mini_v3":
        log.warning(
            "DEPRECATED ranger_mini_v3 profile is using its packaged BT XML; "
            "copy it into the deploy repository and set config.bt_xml_file"
        )
        return (LEGACY_ROOT / "ranger_mini_v3_navigate.xml").resolve()
    return None


def resolve_bt_through_poses_xml_file(cfg: dict) -> Path | None:
    """Resolve the optional deploy-owned NavigateThroughPoses tree.

    Nav2 Humble starts both navigator plugins during lifecycle configure. A
    deployment which only overrides ``default_nav_to_pose_bt_xml`` otherwise
    still loads the distro NavigateThroughPoses tree, including recovery
    actions which may intentionally be absent from the robot behavior server.
    """
    raw = cfg.get("bt_through_poses_xml_file")
    if raw:
        return resolve_deployment_file(raw, "bt_through_poses_xml_file")
    return None


def resolve_trajectory_log_dir(cfg: dict, runtime_directory: Path) -> Path:
    """Resolve the velocity guard's trace directory.

    Trace files are runtime output, not package build artifacts.  When the
    deployment does not request a persistent location, keep them below the
    provider process' private runtime directory so Docker/root and native/user
    launches cannot leave mutually unwritable files in the source tree.
    """
    configured = cfg.get("trajectory_log_dir")
    if configured is None:
        return runtime_directory / "trajectories"
    raw = str(configured).strip()
    if not raw:
        raise ValueError("trajectory_log_dir must not be empty")
    return Path(raw).expanduser()


def validate_absolute_ros_topic(value: object, field: str) -> str:
    """Return a safe fully-qualified ROS topic or reject it.

    The velocity guard is the final process in the motion path, so accepting a
    relative, private, substituted, or otherwise ambiguous name here could
    silently reconnect Nav2 to an unintended publisher.  Keep validation
    independent of rclpy so deployment configuration can be checked offline.
    """
    topic = str(value if value is not None else "").strip()
    if not topic:
        raise ValueError(f"{field} must not be empty")
    if not topic.startswith("/"):
        raise ValueError(f"{field} must be an absolute ROS topic: {topic!r}")
    token = r"[A-Za-z_][A-Za-z0-9_]*"
    if re.fullmatch(rf"/{token}(?:/{token})*", topic) is None:
        raise ValueError(f"{field} is not a valid absolute ROS topic: {topic!r}")
    return topic


def resolve_velocity_output_topic(
    cfg: dict,
    environ: Mapping[str, str] | None = None,
) -> str:
    """Resolve the guard's sole velocity output with config taking priority.

    An explicitly supplied empty config or environment value is an error.  A
    default is used only when neither source is present, preserving historical
    ``/cmd_vel`` behavior while allowing a deployment to select a deliberately
    non-motion topic during integration.
    """
    environment = os.environ if environ is None else environ
    if "velocity_output_topic" in cfg:
        raw = cfg["velocity_output_topic"]
        field = "velocity_output_topic"
    elif VELOCITY_OUTPUT_TOPIC_ENV in environment:
        raw = environment[VELOCITY_OUTPUT_TOPIC_ENV]
        field = VELOCITY_OUTPUT_TOPIC_ENV
    else:
        raw = DEFAULT_VELOCITY_OUTPUT_TOPIC
        field = "velocity_output_topic"
    return validate_absolute_ros_topic(raw, field)


def scan_projection_config(cfg: dict) -> dict[str, object]:
    raw = cfg.get("scan_projection")
    if raw is None:
        return {"enabled": False}
    if not isinstance(raw, dict):
        raise ValueError("scan_projection must be a mapping")
    allowed = {
        "enabled",
        "target_frame",
        "min_height_m",
        "max_height_m",
        "range_max_m",
        "self_filter_margin_m",
        "transform_tolerance_s",
        "deskewing",
        "deskew_fixed_frame",
        "deskew_wait_for_transform_s",
    }
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"unknown scan_projection field(s): {sorted(unknown)}")
    normalized: dict[str, object] = {
        "enabled": bool(raw.get("enabled", False)),
        "target_frame": str(raw.get("target_frame") or "").strip(),
        "min_height_m": float(raw.get("min_height_m", 0.0)),
        "max_height_m": float(raw.get("max_height_m", 2.0)),
        "range_max_m": float(raw.get("range_max_m", 30.0)),
        "self_filter_margin_m": float(raw.get("self_filter_margin_m", 0.05)),
        "transform_tolerance_s": float(raw.get("transform_tolerance_s", 0.15)),
        "deskewing": bool(raw.get("deskewing", False)),
        "deskew_fixed_frame": str(raw.get("deskew_fixed_frame") or "odom").strip(),
        "deskew_wait_for_transform_s": float(
            raw.get("deskew_wait_for_transform_s", 0.2)
        ),
    }
    if normalized["min_height_m"] >= normalized["max_height_m"]:
        raise ValueError("scan_projection min_height_m must be less than max_height_m")
    for key in (
        "range_max_m",
        "self_filter_margin_m",
        "transform_tolerance_s",
        "deskew_wait_for_transform_s",
    ):
        if normalized[key] < 0:
            raise ValueError(f"scan_projection {key} must be non-negative")
    return normalized
