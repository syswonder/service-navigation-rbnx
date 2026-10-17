import math
import unittest
import importlib.util
import os
import tempfile
from unittest.mock import Mock, patch

from nav2_wrapper.rotation_guard_core import GuardLimits, RotationGuard, normalize_uuid_octets


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


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "requires ROS runtime")
class ReferenceHelperTest(unittest.TestCase):
    def test_guard_pause_reset_and_depth_sampling(self):
        import rclpy
        import time
        import numpy as np
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import CameraInfo, Image
        from nav2_wrapper import velocity_guard, depth_cloud

        reading = [(0, 1, 1)]
        reference = Mock(read=lambda: reading[0])
        with tempfile.TemporaryDirectory() as trace_dir, patch.dict(os.environ, {
            "ROBONIX_VELOCITY_OUTPUT_TOPIC": "/chronos_test/guard_output",
            "ROBONIX_NAV_MAX_LINEAR_SPEED_MPS": "0.26",
            "ROBONIX_NAV_DEFAULT_SPEED_PERCENTAGE": "100",
            "ROBONIX_NAV_GUARD_SPEED_LIMIT_TOPIC": "/chronos_test/guard_speed",
            "ROBONIX_NAV_TRACE_DIR": trace_dir,
            "ROBONIX_DEPTH_CLOUD_OUT": "/chronos_test/depth_cloud",
            "ROBONIX_DEPTH_INFO_TOPIC": "/chronos_test/intrinsics",
            "ROBONIX_DEPTH_IMAGE_TOPIC": "/chronos_test/depth",
        }), patch.object(velocity_guard, "_ReferenceClock", return_value=reference), \
                patch.object(depth_cloud, "_ReferenceClock", return_value=reference):
            rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
            guard = velocity_guard.VelocityGuardNode()
            cloud = depth_cloud.DepthCloud()
            try:
                self.assertTrue(guard.get_parameter("use_sim_time").value)
                self.assertTrue(cloud.get_parameter("use_sim_time").value)
                guard._pub = Mock()
                guard._begin_goal("old", list(range(16)))
                command = Twist()
                command.angular.z = .2
                guard._on_cmd(command)
                time.sleep(.03)
                for _ in range(30):
                    guard._on_cmd(command)
                self.assertEqual(guard.guard.latched_reason, "")
                reading[0] = (0, 2, 1)
                guard._publish_latched_zero()
                self.assertIn("reference interrupted", guard.guard.latched_reason)
                guard._on_cmd(command)
                self.assertEqual(guard._pub.publish.call_args.args[0].angular.z, 0)

                cloud._pub = Mock()
                info = CameraInfo(width=1, height=1)
                info.k = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                cloud._on_info(info)
                frame = Image(width=1, height=1, encoding="32FC1", step=4,
                              data=np.array([1.], dtype=np.float32).tobytes())
                cloud._on_depth(frame)
                for _ in range(30):
                    cloud._on_depth(frame)
                self.assertEqual(cloud._pub.publish.call_count, 1)
                reading[0] = (200_000_000, 2, 1)
                time.sleep(.11)
                cloud._on_depth(frame)
                self.assertEqual(cloud._pub.publish.call_count, 2)
                reading[0] = (0, 3, 1)
                cloud._on_depth(frame)
                self.assertEqual(cloud._pub.publish.call_count, 3)
            finally:
                cloud.destroy_node()
                guard.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
