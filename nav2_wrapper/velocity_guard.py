"""Final velocity guard and passive per-NavigateToPose trajectory recorder."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from nav2_wrapper.configuration import resolve_velocity_output_topic
from nav2_wrapper.rotation_guard_core import GuardLimits, RotationGuard, normalize_uuid_octets


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _uuid_text(raw) -> str:
    return bytes(raw).hex()


class VelocityGuardNode(Node):
    def __init__(self):
        # Validate before constructing the ROS node or creating any endpoint.
        # An invalid/empty/relative output must fail closed with no publisher.
        output_topic = resolve_velocity_output_topic({})
        super().__init__("robonix_velocity_guard")
        limits = GuardLimits(
            terminal_xy_m=float(os.getenv("ROBONIX_GUARD_TERMINAL_XY_M", "0.45")),
            terminal_timeout_s=float(os.getenv("ROBONIX_GUARD_TERMINAL_TIMEOUT_S", "15.0")),
            terminal_no_progress_s=float(os.getenv("ROBONIX_GUARD_NO_PROGRESS_S", "3.0")),
            global_spin_timeout_s=float(os.getenv("ROBONIX_GUARD_GLOBAL_TIMEOUT_S", "25.0")),
            global_spin_limit_rad=float(os.getenv("ROBONIX_GUARD_GLOBAL_ROTATION_RAD", "6.783185307")),
        )
        self.guard = RotationGuard(limits)
        self._lock = threading.Lock()
        self._active_uuid = ""
        self._active_uuid_raw = None
        self._robot_pose = None
        self._goal_pose = None
        self._last_cmd = (0.0, 0.0)
        self._cancel_sent = False
        self._trace = None
        self._trace_dir = Path(os.getenv("ROBONIX_NAV_TRACE_DIR", "/tmp/robonix-nav-traces"))
        self._trace_dir.mkdir(parents=True, exist_ok=True)
        self._scan_log = (self._trace_dir / "scan-anomalies.jsonl").open("a", encoding="utf-8")
        self._front_min_history = deque(maxlen=20)
        self._last_scan_summary_at = 0.0

        self._pub = self.create_publisher(Twist, output_topic, 10)
        self.create_subscription(Twist, "/cmd_vel_guard_input", self._on_cmd, 20)
        self.create_subscription(Odometry, "/odom", self._on_odom, 50)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(LaserScan, "/scanner/scan", self._on_scan, 20)
        self.create_subscription(
            GoalStatusArray, "/navigate_to_pose/_action/status", self._on_status, 10
        )
        self.create_subscription(
            NavigateToPose.Impl.FeedbackMessage,
            "/navigate_to_pose/_action/feedback",
            self._on_feedback,
            20,
        )
        self._cancel = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
        self.create_timer(0.05, self._publish_latched_zero)
        self.get_logger().info(
            f"guard active; output={output_topic}; traces={self._trace_dir}"
        )

    def _event(self, kind: str, **values) -> None:
        if self._trace is None:
            return
        record = {"t_wall": time.time(), "event": kind, **values}
        self._trace.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._trace.flush()

    def _begin_goal(self, goal_uuid: str, raw_uuid) -> None:
        if goal_uuid == self._active_uuid:
            return
        if self._trace is not None:
            self._event("trace_closed", reason="new_goal")
            self._trace.close()
        self._active_uuid = goal_uuid
        self._active_uuid_raw = normalize_uuid_octets(raw_uuid)
        self._cancel_sent = False
        self._robot_pose = None
        self._goal_pose = None
        self.guard.begin_goal(goal_uuid)
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self._trace = (self._trace_dir / f"{stamp}-{goal_uuid}.jsonl").open("a", encoding="utf-8")
        self._event("goal_active", uuid=goal_uuid)

    def _on_status(self, msg: GoalStatusArray) -> None:
        with self._lock:
            active = [
                item for item in msg.status_list
                if item.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                                   GoalStatus.STATUS_CANCELING)
            ]
            if active:
                item = active[-1]
                self._begin_goal(_uuid_text(item.goal_info.goal_id.uuid), item.goal_info.goal_id.uuid)
            self._event(
                "status",
                values=[{"uuid": _uuid_text(s.goal_info.goal_id.uuid), "status": int(s.status)}
                        for s in msg.status_list],
            )
            if not active and self._active_uuid:
                self._event("goal_inactive")
                self.guard.clear_if_idle()

    def _on_feedback(self, msg) -> None:
        with self._lock:
            uuid = _uuid_text(msg.goal_id.uuid)
            if uuid and uuid != self._active_uuid:
                self._begin_goal(uuid, msg.goal_id.uuid)
            pose = msg.feedback.current_pose.pose
            self._robot_pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
            self._event(
                "feedback", pose=self._robot_pose,
                distance_remaining=float(msg.feedback.distance_remaining),
                recoveries=int(msg.feedback.number_of_recoveries),
            )

    def _on_plan(self, msg: NavPath) -> None:
        if not msg.poses:
            return
        with self._lock:
            pose = msg.poses[-1].pose
            self._goal_pose = (pose.position.x, pose.position.y, _yaw(pose.orientation))
            self._event("plan", frame=msg.header.frame_id, points=len(msg.poses), goal=self._goal_pose)

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            pose = msg.pose.pose
            yaw = _yaw(pose.orientation)
            self.guard.observe_odom(yaw)
            self._event(
                "odom", pose=(pose.position.x, pose.position.y, yaw),
                twist=(msg.twist.twist.linear.x, msg.twist.twist.angular.z),
            )

    def _on_scan(self, msg: LaserScan) -> None:
        """Persist sudden/isolated close returns even when no goal is active."""
        front = []
        for index, value in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if (
                abs(angle) <= math.radians(60.0)
                and math.isfinite(value)
                and msg.range_min <= value <= msg.range_max
            ):
                front.append((index, angle, float(value)))
        if not front:
            return
        ordered = sorted(value for _, _, value in front)
        front_min = ordered[0]
        front_p05 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.05))]
        baseline = None
        if self._front_min_history:
            history = sorted(self._front_min_history)
            baseline = history[len(history) // 2]
        sudden = baseline is not None and baseline - front_min >= 0.40 and front_min <= 2.0

        isolated = []
        by_index = {index: value for index, _, value in front}
        for index, angle, value in front:
            if value > 2.0:
                continue
            neighbors = [by_index.get(index + offset) for offset in (-2, -1, 1, 2)]
            close_neighbors = sum(
                other is not None and abs(other - value) <= 0.20 for other in neighbors
            )
            if close_neighbors <= 1:
                isolated.append((round(angle, 5), round(value, 4)))
        now = time.monotonic()
        periodic = now - self._last_scan_summary_at >= 1.0
        if sudden or periodic:
            record = {
                "t_wall": time.time(),
                "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                "frame": msg.header.frame_id,
                "front_min": round(front_min, 4),
                "front_p05": round(front_p05, 4),
                "baseline_min": None if baseline is None else round(baseline, 4),
                "sudden": sudden,
                "isolated": isolated[:40],
                "front_under_2m": [
                    (round(angle, 5), round(value, 4))
                    for _, angle, value in front if value <= 2.0
                ] if sudden else [],
                "range_min": float(msg.range_min),
                "range_max": float(msg.range_max),
            }
            self._scan_log.write(json.dumps(record, separators=(",", ":")) + "\n")
            self._scan_log.flush()
            self._last_scan_summary_at = now
        self._front_min_history.append(front_min)

    def _on_cmd(self, msg: Twist) -> None:
        with self._lock:
            self._last_cmd = (float(msg.linear.x), float(msg.angular.z))
            reason = self.guard.evaluate(
                self.get_clock().now().nanoseconds / 1e9,
                self._last_cmd[0], self._last_cmd[1], self._robot_pose, self._goal_pose,
            )
            self._event(
                "cmd", linear=self._last_cmd[0], angular=self._last_cmd[1],
                terminal_rotation=self.guard.terminal_rotation,
                continuous_rotation=self.guard.spin_rotation,
                latched=bool(reason),
            )
            if reason:
                self._trip(reason)
                self._pub.publish(Twist())
            else:
                self._pub.publish(msg)

    def _trip(self, reason: str) -> None:
        if self._cancel_sent:
            return
        self._cancel_sent = True
        self.get_logger().error(f"ROTATION GUARD TRIPPED uuid={self._active_uuid}: {reason}")
        self._event("guard_trip", reason=reason)
        if self._active_uuid_raw is None or not self._cancel.service_is_ready():
            return
        request = CancelGoal.Request()
        request.goal_info.goal_id.uuid = self._active_uuid_raw
        self._cancel.call_async(request)
        self._event("cancel_requested", uuid=self._active_uuid)

    def _publish_latched_zero(self) -> None:
        with self._lock:
            if self.guard.latched_reason:
                self._pub.publish(Twist())


def main() -> None:
    rclpy.init()
    node = VelocityGuardNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
