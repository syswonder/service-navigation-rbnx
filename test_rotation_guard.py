import importlib.util
import math
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from nav2_wrapper.rotation_guard_core import GuardLimits, RotationGuard, normalize_uuid_octets


class VelocityGuardDepthTest(unittest.TestCase):
    @staticmethod
    def _twist():
        return SimpleNamespace(
            linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            angular=SimpleNamespace(x=0.0, y=0.0, z=0.0),
        )

    def setUp(self):
        node_api = Mock()
        node_api.get_clock.return_value.now.return_value.nanoseconds = 0
        node_api.create_client.return_value.service_is_ready.return_value = True
        node_type = type("Node", (), {
            "__init__": lambda _self, _name: None,
            **{name: getattr(node_api, name) for name in (
                "create_publisher", "create_subscription", "create_client",
                "create_timer", "get_clock", "get_logger",
            )},
        })
        self.status = SimpleNamespace(
            STATUS_ACCEPTED=1, STATUS_EXECUTING=2, STATUS_CANCELING=3,
        )
        fake_ros = {
            "rclpy": SimpleNamespace(),
            "rclpy.node": SimpleNamespace(Node=node_type),
            "rclpy.executors": SimpleNamespace(ExternalShutdownException=RuntimeError),
            "rclpy.qos": SimpleNamespace(qos_profile_sensor_data=object()),
            "action_msgs.msg": SimpleNamespace(GoalStatus=self.status, GoalStatusArray=object),
            "action_msgs.srv": SimpleNamespace(CancelGoal=SimpleNamespace(
                Request=lambda: SimpleNamespace(
                    goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=[])),
                ),
            )),
            "geometry_msgs.msg": SimpleNamespace(Twist=self._twist),
            "nav2_msgs.action": SimpleNamespace(NavigateToPose=SimpleNamespace(
                Impl=SimpleNamespace(FeedbackMessage=object),
            )),
            "nav2_msgs.msg": SimpleNamespace(SpeedLimit=object),
            "nav_msgs.msg": SimpleNamespace(Odometry=object, Path=object),
            "sensor_msgs.msg": SimpleNamespace(LaserScan=object, PointCloud2=object),
        }
        spec = importlib.util.spec_from_file_location(
            "velocity_guard_under_test",
            Path(__file__).parent / "nav2_wrapper" / "velocity_guard.py",
        )
        self.module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, fake_ros):
            spec.loader.exec_module(self.module)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.environment = {
            "ROBONIX_VELOCITY_OUTPUT_TOPIC": "/test/guarded_velocity",
            "ROBONIX_NAV_MAX_LINEAR_SPEED_MPS": "0.3",
            "ROBONIX_NAV_DEFAULT_SPEED_PERCENTAGE": "100",
            "ROBONIX_NAV_GUARD_SPEED_LIMIT_TOPIC": "/test/speed_limit",
            "ROBONIX_NAV_TRACE_DIR": directory.name,
            "ROBONIX_GUARD_DEPTH_TOPIC": "/test/depth_points",
        }
        environment_patch = patch.dict(os.environ, self.environment, clear=True)
        environment_patch.start()
        self.addCleanup(environment_patch.stop)
        self.now = 10.0
        clock_patch = patch.object(self.module.time, "monotonic", side_effect=lambda: self.now)
        clock_patch.start()
        self.addCleanup(clock_patch.stop)
        self.node_api = node_api

    def _node(self, **environment):
        with patch.dict(os.environ, environment):
            node = self.module.VelocityGuardNode()
        self.addCleanup(node._scan_log.close)
        self.addCleanup(lambda: node._trace.close() if node._trace else None)
        return node

    def _goal(self, node, number=1):
        raw_uuid = [number] * 16
        node._begin_goal(bytes(raw_uuid).hex(), raw_uuid)
        return raw_uuid

    @staticmethod
    def _depth(node, stamp=1):
        node._on_depth(SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=stamp, nanosec=0),
        )))

    def _command(self, node):
        command = self._twist()
        command.linear.x = 0.2
        command.angular.z = 0.1
        node._on_cmd(command)

    def _assert_zero(self, node):
        command = node._pub.publish.call_args.args[0]
        self.assertEqual(
            (command.linear.x, command.linear.y, command.linear.z,
             command.angular.x, command.angular.y, command.angular.z),
            (0.0,) * 6,
        )

    def test_missing_first_sample_blocks_and_cancels_exact_goal(self):
        node = self._node()
        raw_uuid = self._goal(node)
        self._command(node)
        self._assert_zero(node)
        self.assertIn("has not received", node.guard.latched_reason)
        request = node._cancel.call_async.call_args.args[0]
        self.assertEqual(request.goal_info.goal_id.uuid, raw_uuid)

    def test_timer_detects_expiry_without_another_command(self):
        node = self._node()
        self._depth(node)
        self._goal(node)
        self._command(node)
        self.assertEqual(node._pub.publish.call_args.args[0].linear.x, 0.2)
        self.now += 1.0
        node._publish_latched_zero()
        self._assert_zero(node)
        self.assertIn("timeout", node.guard.latched_reason)
        node._pub.publish.reset_mock()
        self.now += 1.0
        node._publish_latched_zero()
        self._assert_zero(node)
        node._cancel.call_async.assert_called_once()

    def test_command_path_detects_expiry_without_waiting_for_timer(self):
        node = self._node()
        self._depth(node)
        self._goal(node)
        self.now += 1.1
        self._command(node)
        self._assert_zero(node)
        node._cancel.call_async.assert_called_once()

    def test_advancing_samples_keep_the_same_goal_healthy(self):
        node = self._node()
        self._depth(node)
        self._goal(node)
        for stamp in range(2, 8):
            self.now += 0.5
            self._depth(node, stamp)
            node._publish_latched_zero()
            self._command(node)
            self.assertEqual(node.guard.latched_reason, "")
            self.assertEqual(node._pub.publish.call_args.args[0].linear.x, 0.2)
        node._cancel.call_async.assert_not_called()

    def test_repeated_older_and_zero_stamps_do_not_refresh_watchdog(self):
        for stamp in (0, 1, 2):
            with self.subTest(stamp=stamp):
                node = self._node()
                self._depth(node, 2)
                self._goal(node)
                received_at = node._depth_received_at
                self.now += 0.75
                self._depth(node, stamp)
                self.assertEqual(node._depth_received_at, received_at)
                self.now += 0.25
                node._publish_latched_zero()
                self._assert_zero(node)
                self.assertIn("timeout", node.guard.latched_reason)

    def test_recovery_and_terminal_status_do_not_release_same_goal(self):
        node = self._node()
        raw_uuid = self._goal(node)
        node._publish_latched_zero()
        reason = node.guard.latched_reason
        node._on_status(SimpleNamespace(status_list=[]))
        self._depth(node)
        node._begin_goal(bytes(raw_uuid).hex(), raw_uuid)
        self._command(node)
        self._assert_zero(node)
        self.assertEqual(node.guard.latched_reason, reason)
        node._cancel.call_async.assert_called_once()

    def test_new_goal_with_recovered_data_can_move(self):
        node = self._node()
        self._depth(node)
        self._goal(node)
        self.now += 1.1
        node._publish_latched_zero()
        self._depth(node, 2)
        self._goal(node, 2)
        self._command(node)
        self.assertEqual(node.guard.latched_reason, "")
        self.assertEqual(node._pub.publish.call_args.args[0].linear.x, 0.2)
        self.assertFalse(node._cancel_sent)

    def test_fresh_depth_preserves_upstream_rotation_idle_recovery(self):
        node = self._node()
        self._depth(node)
        self._goal(node)
        node.guard.latched_reason = "terminal rotation timeout"
        node._on_status(SimpleNamespace(status_list=[]))
        self.assertEqual(node.guard.latched_reason, "")
        self._command(node)
        self.assertEqual(node._pub.publish.call_args.args[0].linear.x, 0.2)
        node._cancel.call_async.assert_not_called()

    def test_new_goal_cannot_refresh_old_data(self):
        node = self._node()
        self._depth(node, 2)
        self._goal(node)
        self.now += 1.1
        node._publish_latched_zero()
        self._depth(node, 2)
        raw_uuid = self._goal(node, 2)
        self._command(node)
        self._assert_zero(node)
        self.assertIn("timeout", node.guard.latched_reason)
        request = node._cancel.call_async.call_args.args[0]
        self.assertEqual(request.goal_info.goal_id.uuid, raw_uuid)

    def test_cancel_unavailable_or_send_failure_retries_from_timer(self):
        node = self._node()
        self._goal(node)
        node._cancel.service_is_ready.return_value = False
        self._command(node)
        self.assertFalse(node._cancel_sent)
        node._cancel.call_async.assert_not_called()
        node._cancel.service_is_ready.return_value = True
        node._cancel.call_async.side_effect = [RuntimeError("unavailable"), Mock()]
        node._publish_latched_zero()
        self._assert_zero(node)
        self.assertFalse(node._cancel_sent)
        node._publish_latched_zero()
        self.assertTrue(node._cancel_sent)
        self.assertEqual(node._cancel.call_async.call_count, 2)

    def test_missing_or_zero_uuid_never_cancels_all_goals(self):
        for raw_uuid in (None, [0] * 16):
            with self.subTest(raw_uuid=raw_uuid):
                node = self._node()
                node._active_uuid_raw = raw_uuid
                node._publish_latched_zero()
                self._assert_zero(node)
                node._cancel.call_async.assert_not_called()

    def test_depth_disabled_preserves_command_and_idle_behavior(self):
        node = self._node(
            ROBONIX_GUARD_DEPTH_TOPIC="",
            ROBONIX_GUARD_DEPTH_TIMEOUT_S="nan",
        )
        self._goal(node)
        self.now += 100.0
        self._command(node)
        self.assertEqual(node._pub.publish.call_args.args[0].linear.x, 0.2)
        node._pub.publish.reset_mock()
        node._publish_latched_zero()
        node._pub.publish.assert_not_called()
        node._cancel.call_async.assert_not_called()
        self.assertFalse(any(
            call.args[1] == "/test/depth_points"
            for call in node.create_subscription.call_args_list
        ))
        node.guard.latched_reason = "rotation limit"
        node._on_status(SimpleNamespace(status_list=[]))
        self.assertEqual(node.guard.latched_reason, "")

    def test_depth_timeout_and_topic_are_validated_before_ros_endpoints(self):
        for timeout in ("0", "-1", "nan", "inf", "-inf", "bad"):
            with self.subTest(timeout=timeout):
                self.node_api.create_publisher.reset_mock()
                with self.assertRaises(ValueError):
                    self._node(ROBONIX_GUARD_DEPTH_TIMEOUT_S=timeout)
                self.node_api.create_publisher.assert_not_called()
        with self.assertRaises(ValueError):
            self._node(ROBONIX_GUARD_DEPTH_TOPIC="relative")

    def test_configured_timeout_allows_fresh_data_until_deadline(self):
        node = self._node(ROBONIX_GUARD_DEPTH_TIMEOUT_S="2.5")
        self._depth(node)
        self._goal(node)
        self.now += 2.0
        self._command(node)
        self.assertEqual(node.guard.latched_reason, "")
        self.assertEqual(node._pub.publish.call_args.args[0].linear.x, 0.2)
        self.now += 0.5
        node._publish_latched_zero()
        self._assert_zero(node)


class RotationGuardTest(unittest.TestCase):
    def test_ros_uuid_octets_are_plain_python_ints(self):
        class ByteLike:
            def __init__(self, value):
                self.value = value

            def __int__(self):
                return self.value

        result = normalize_uuid_octets(ByteLike(value) for value in range(16))
        self.assertEqual(result, list(range(16)))
        self.assertTrue(all(type(value) is int for value in result))

    def test_terminal_rotation_without_progress_latches(self):
        guard = RotationGuard(GuardLimits(terminal_no_progress_s=3.0))
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        self.assertEqual(guard.evaluate(0.0, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0)), "")
        guard.observe_odom(0.2)
        reason = guard.evaluate(3.1, 0.0, 0.2, (0, 0, 0.0), (0.1, 0, 1.0))
        self.assertEqual(reason, "terminal yaw made no progress")
        self.assertEqual(guard.evaluate(3.2, 0.0, 0.0, None, None), reason)

    def test_new_action_uuid_is_the_only_reset(self):
        guard = RotationGuard(GuardLimits(terminal_no_progress_s=1.0))
        guard.begin_goal("goal-a")
        guard.evaluate(0.0, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0))
        guard.evaluate(1.1, 0.0, 0.2, (0, 0, 0), (0.1, 0, 1.0))
        self.assertTrue(guard.latched_reason)
        guard.begin_goal("goal-a")
        self.assertTrue(guard.latched_reason)
        guard.begin_goal("goal-b")
        self.assertFalse(guard.latched_reason)

    def test_global_guard_catches_recovery_spin_away_from_goal(self):
        guard = RotationGuard(GuardLimits(global_spin_limit_rad=1.0))
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        guard.evaluate(0.0, 0.0, 0.2, None, None)
        guard.observe_odom(0.6)
        guard.observe_odom(1.2)
        reason = guard.evaluate(1.0, 0.0, 0.2, None, None)
        self.assertEqual(reason, "continuous stationary rotation limit")

    def test_global_timeout_means_no_measured_yaw_progress(self):
        guard = RotationGuard(
            GuardLimits(global_spin_timeout_s=2.0, global_spin_limit_rad=4.0)
        )
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        self.assertEqual(guard.evaluate(0.0, 0.0, 0.2, None, None), "")
        guard.observe_odom(0.02)
        self.assertEqual(guard.evaluate(1.0, 0.0, 0.2, None, None), "")
        self.assertEqual(
            guard.evaluate(2.1, 0.0, 0.2, None, None),
            "continuous stationary rotation timeout",
        )

    def test_measured_yaw_progress_extends_global_timeout(self):
        guard = RotationGuard(
            GuardLimits(
                global_spin_timeout_s=2.0,
                global_spin_limit_rad=4.0,
                global_min_progress_rad=0.04,
            )
        )
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        self.assertEqual(guard.evaluate(0.0, 0.0, 0.2, None, None), "")
        for now, yaw in ((1.5, 0.05), (3.0, 0.10), (4.5, 0.15)):
            guard.observe_odom(yaw)
            self.assertEqual(guard.evaluate(now, 0.0, 0.2, None, None), "")

    def test_legitimate_half_turn_is_below_global_limit(self):
        guard = RotationGuard()
        guard.begin_goal("goal-a")
        guard.observe_odom(0.0)
        guard.evaluate(0.0, 0.0, 0.2, None, None)
        for i in range(1, 17):
            guard.observe_odom(i * math.pi / 16.0)
            self.assertEqual(guard.evaluate(i * 0.5, 0.0, 0.2, None, None), "")


if __name__ == "__main__":
    unittest.main()
