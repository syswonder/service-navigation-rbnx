import ast
from pathlib import Path
import unittest

from nav2_wrapper.guarded_launch import (
    EXTERNAL_BEHAVIOR_SINK,
    patch_navigation_launch,
)


HUMBLE_LAUNCH = Path(
    "/opt/ros/humble/share/nav2_bringup/launch/navigation_launch.py"
)


class GuardedLaunchTest(unittest.TestCase):
    @staticmethod
    def _package_blocks(source: str, marker: str) -> list[str]:
        starts = []
        offset = 0
        while True:
            start = source.find(marker, offset)
            if start < 0:
                break
            starts.append(start)
            offset = start + len(marker)
        blocks = []
        for start in starts:
            end = source.find("package='", start + len(marker))
            blocks.append(source[start : len(source) if end < 0 else end])
        return blocks

    def test_installed_humble_launch_is_guarded_in_both_branches(self):
        if not HUMBLE_LAUNCH.is_file():
            self.skipTest("ROS Humble nav2_bringup launch is not installed")
        result = patch_navigation_launch(HUMBLE_LAUNCH.read_text(encoding="utf-8"))
        ast.parse(result)

        self.assertEqual(
            result.count("('cmd_vel', 'cmd_vel_guard_input')"),
            2,
        )
        self.assertEqual(
            result.count("('cmd_vel_smoothed', 'cmd_vel_guard_input')"),
            2,
        )
        self.assertNotIn("('cmd_vel_smoothed', 'cmd_vel')", result)
        self.assertEqual(result.count("component_container_isolated"), 1)
        self.assertEqual(result.count("ld.add_action(robonix_nav2_container)"), 1)
        self.assertLess(
            result.index("ld.add_action(robonix_nav2_container)"),
            result.index("ld.add_action(load_composable_nodes)"),
        )
        self.assertIn("target_container=container_name_full", result)
        self.assertIn("name=container_name", result)
        self.assertIn("namespace=namespace", result)

        controllers = self._package_blocks(result, "package='nav2_controller'")
        behaviors = self._package_blocks(result, "package='nav2_behaviors'")
        smoothers = self._package_blocks(
            result, "package='nav2_velocity_smoother'"
        )
        self.assertEqual(len(controllers), 2)
        self.assertEqual(len(behaviors), 2)
        self.assertEqual(len(smoothers), 2)
        for block in controllers:
            self.assertIn("('cmd_vel', 'cmd_vel_nav')", block)
            self.assertNotIn("cmd_vel_guard_input", block)
        for block in behaviors:
            self.assertIn("('cmd_vel', 'cmd_vel_guard_input')", block)
        for block in smoothers:
            self.assertIn("('cmd_vel', 'cmd_vel_nav')", block)
            self.assertIn(
                "('cmd_vel_smoothed', 'cmd_vel_guard_input')", block
            )

    def test_layout_drift_fails_closed(self):
        with self.assertRaisesRegex(RuntimeError, "unsupported nav2"):
            patch_navigation_launch("def generate_launch_description():\n    pass\n")

    def test_external_guard_has_one_input_producer_and_behavior_sink(self):
        if not HUMBLE_LAUNCH.is_file():
            self.skipTest("ROS Humble nav2_bringup launch is not installed")
        result = patch_navigation_launch(
            HUMBLE_LAUNCH.read_text(encoding="utf-8"),
            external_velocity_guard=True,
        )
        ast.parse(result)
        self.assertEqual(result.count("cmd_vel_guard_input"), 2)
        self.assertEqual(
            result.count(
                "('cmd_vel_smoothed', 'cmd_vel_guard_input')"
            ),
            2,
        )
        self.assertNotIn(
            "('cmd_vel', 'cmd_vel_guard_input')",
            result,
        )
        self.assertEqual(result.count(EXTERNAL_BEHAVIOR_SINK), 2)
        behaviors = self._package_blocks(
            result, "package='nav2_behaviors'"
        )
        smoothers = self._package_blocks(
            result, "package='nav2_velocity_smoother'"
        )
        for block in behaviors:
            self.assertIn(
                f"('cmd_vel', '{EXTERNAL_BEHAVIOR_SINK}')", block
            )
            self.assertNotIn("cmd_vel_guard_input", block)
        for block in smoothers:
            self.assertIn(
                "('cmd_vel_smoothed', 'cmd_vel_guard_input')", block
            )

    def test_external_guard_selector_is_strict_boolean(self):
        if not HUMBLE_LAUNCH.is_file():
            self.skipTest("ROS Humble nav2_bringup launch is not installed")
        with self.assertRaisesRegex(RuntimeError, "must be a boolean"):
            patch_navigation_launch(
                HUMBLE_LAUNCH.read_text(encoding="utf-8"),
                external_velocity_guard="true",
            )

    def test_third_behavior_branch_fails_closed(self):
        if not HUMBLE_LAUNCH.is_file():
            self.skipTest("ROS Humble nav2_bringup launch is not installed")
        source = HUMBLE_LAUNCH.read_text(encoding="utf-8")
        drifted = source.replace(
            "package='nav2_planner'", "package='nav2_behaviors'", 1
        )
        with self.assertRaisesRegex(RuntimeError, "exactly 2 nav2_behaviors"):
            patch_navigation_launch(drifted)

    def test_controller_canonical_output_drift_fails_closed(self):
        if not HUMBLE_LAUNCH.is_file():
            self.skipTest("ROS Humble nav2_bringup launch is not installed")
        source = HUMBLE_LAUNCH.read_text(encoding="utf-8")
        drifted = source.replace(
            "remappings=remappings + [('cmd_vel', 'cmd_vel_nav')])",
            "remappings=remappings + [('cmd_vel', 'cmd_vel')])",
            1,
        )
        with self.assertRaisesRegex(RuntimeError, "controller velocity remap"):
            patch_navigation_launch(drifted)

    def test_unknown_producer_canonical_output_fails_closed(self):
        if not HUMBLE_LAUNCH.is_file():
            self.skipTest("ROS Humble nav2_bringup launch is not installed")
        source = HUMBLE_LAUNCH.read_text(encoding="utf-8")
        marker = "    # Create the launch description and populate\n"
        unknown_producer = """    unknown_velocity_source = Node(
        package='new_velocity_source',
        executable='producer',
        remappings=[('new_velocity', 'cmd_vel')])

"""
        drifted = source.replace(marker, unknown_producer + marker)
        with self.assertRaisesRegex(RuntimeError, "canonical cmd_vel output"):
            patch_navigation_launch(drifted)


if __name__ == "__main__":
    unittest.main()
