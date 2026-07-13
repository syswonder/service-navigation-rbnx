import ast
import os
import signal
import subprocess
import time
import unittest
import xml.etree.ElementTree as ET
import importlib.util
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent


class RangerProfileTest(unittest.TestCase):
    def setUp(self):
        self.params = yaml.safe_load(
            (ROOT / "config" / "nav2_params_ranger_mini_v3.yml").read_text()
        )
        self.controller = self.params["controller_server"]["ros__parameters"]

    def test_terminal_latches_survive_replanning(self):
        follow = self.controller["FollowPath"]
        goal = self.controller["general_goal_checker"]
        self.assertEqual(follow["plugin"], "dwb_core::DWBLocalPlanner")
        self.assertIn("PersistentRotateToGoal", follow["critics"])
        self.assertNotIn("RotateToGoal", follow["critics"])
        self.assertNotIn("primary_controller", follow)
        self.assertEqual(
            goal["plugin"], "robonix_nav2_terminal::PersistentGoalChecker"
        )
        self.assertEqual(goal["xy_enter_tolerance"], 0.30)
        self.assertEqual(goal["xy_exit_tolerance"], 0.45)
        self.assertEqual(
            follow["PersistentRotateToGoal.max_terminal_angular_velocity"], 0.30
        )
        self.assertEqual(
            follow["PersistentRotateToGoal.max_terminal_duration"], 15.0
        )

    def test_packaged_recovery_tree_does_not_command_a_spin(self):
        navigator = self.params["bt_navigator"]["ros__parameters"]
        self.assertEqual(navigator["default_bt_xml_filename"], "__ROBONIX_BT_XML__")
        self.assertEqual(
            navigator["default_nav_to_pose_bt_xml"], "__ROBONIX_BT_XML__"
        )
        tree = ET.parse(ROOT / "config" / "ranger_mini_v3_navigate.xml")
        root_recovery = tree.find(".//BehaviorTree/RecoveryNode")
        self.assertEqual(root_recovery.attrib["number_of_retries"], "1")
        rate = tree.find(".//RateController")
        self.assertEqual(rate.attrib["hz"], "5.0")
        self.assertIsNone(tree.find(".//Spin"))
        self.assertIsNone(tree.find(".//BackUp"))
        follow_path = tree.find(".//FollowPath")
        self.assertIsNotNone(follow_path)
        self.assertNotIn("path_topic", follow_path.attrib)

    def test_bt_loop_allows_action_callbacks_under_slam_load(self):
        navigator = self.params["bt_navigator"]["ros__parameters"]
        self.assertEqual(navigator["bt_loop_duration"], 50)
        self.assertEqual(navigator["default_server_timeout"], 1000)

    def test_global_inflation_matches_local_ranger_clearance(self):
        global_params = self.params["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        local_params = self.params["local_costmap"]["local_costmap"][
            "ros__parameters"
        ]
        global_inflation = global_params["inflation_layer"]
        local_inflation = local_params["inflation_layer"]
        self.assertEqual(global_inflation["inflation_radius"], 0.60)
        self.assertEqual(global_inflation["cost_scaling_factor"], 2.2)
        self.assertEqual(
            global_inflation["inflation_radius"],
            local_inflation["inflation_radius"],
        )

    def test_action_uuid_owns_goal_identity_after_first_status(self):
        goal_checker = (
            ROOT / "terminal_controller" / "src" / "persistent_goal_checker.cpp"
        ).read_text()
        critic = (
            ROOT
            / "terminal_controller"
            / "src"
            / "persistent_rotate_to_goal_critic.cpp"
        ).read_text()
        for source, geometry_check in (
            (goal_checker, "goalChanged(goal_pose)"),
            (critic, "goalChanged(goal)"),
        ):
            self.assertIn("shouldStartGoal", source)
            self.assertIn("if (epoch != 0)", source)
            self.assertIn(geometry_check, source)

        identity = (
            ROOT
            / "terminal_controller"
            / "include"
            / "robonix_nav2_terminal"
            / "navigate_goal_epoch.hpp"
        ).read_text()
        self.assertIn("observed_epoch != 0", identity)
        self.assertIn("observed_epoch != seen_epoch", identity)

    def test_yaw_latch_brakes_instead_of_rejecting_every_nonzero_sample(self):
        critic = (
            ROOT
            / "terminal_controller"
            / "src"
            / "persistent_rotate_to_goal_critic.cpp"
        ).read_text()
        self.assertNotIn("nonzero command after terminal yaw latch", critic)
        self.assertIn("angular_speed * angular_speed", critic)

    def test_humble_abort_detail_keeps_feedback_and_root_signal(self):
        path = ROOT / "nav2_wrapper" / "diagnostics.py"
        spec = importlib.util.spec_from_file_location("nav_diagnostics", path)
        diagnostics = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(diagnostics)
        line = diagnostics.classify_nav2_line(
            "[controller_server] Rotation Shim Controller detected Collision Ahead!"
        )
        detail = diagnostics.format_result_detail(
            "ABORTED",
            {"distance_remaining": 1.25, "recoveries": 2, "x": 3.0, "y": -1.0},
            [line],
        )
        self.assertIn("distance_remaining=1.250m", detail)
        self.assertIn("recoveries=2", detail)
        self.assertIn("Collision Ahead", detail)

    def test_ranger_scan_pipeline_can_deskew_before_projection(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn('"rtabmap_util", "lidar_deskewing"', source)
        self.assertIn("projector_cloud_topic = f\"{cloud_topic.rstrip('/')}/deskewed\"", source)
        self.assertIn("fixed_frame_id:=", source)
        self.assertIn("os.killpg(proc.pid, signal.SIGTERM)", source)

    def test_nav_consumes_provider_pinned_canonical_odom(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn('"odom",  "robonix/primitive/chassis/odom"', source)
        self.assertIn('providers = dict(cfg.get("provider_ids", {}) or {})', source)
        self.assertIn('provider_id=provider_id', source)
        self.assertNotIn(
            '("odom",  "robonix/service/map/odom"', source
        )

    def test_scan_cleanup_kills_child_after_ros2_parent_exits(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        tree = ast.parse(source)
        function = next(
            node for node in tree.body
            if isinstance(node, ast.FunctionDef) and node.name == "_kill_scan_projector"
        )
        namespace = {
            "os": os,
            "signal": signal,
            "subprocess": subprocess,
            "_scan_projector_proc": None,
            "_scan_deskew_proc": None,
            "_scan_filter_proc": None,
        }
        exec(compile(ast.Module(body=[function], type_ignores=[]), "cleanup", "exec"), namespace)

        parent = subprocess.Popen(
            ["bash", "-c", "sleep 30 & echo $!"],
            start_new_session=True,
            stdout=subprocess.PIPE,
            text=True,
        )
        child_pid = int(parent.stdout.readline().strip())
        parent.wait(timeout=2.0)
        namespace["_scan_projector_proc"] = parent
        namespace["_kill_scan_projector"]()

        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                os.kill(child_pid, 0)
            except ProcessLookupError:
                break
            state = subprocess.run(
                ["ps", "-o", "stat=", "-p", str(child_pid)],
                capture_output=True,
                text=True,
                check=False,
            ).stdout.strip()
            if not state or state.startswith("Z"):
                break
            time.sleep(0.05)
        else:
            os.kill(child_pid, signal.SIGKILL)
            self.fail("scan child process group survived cleanup")

    def test_final_velocity_guard_owns_cmd_vel(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn("('cmd_vel', 'cmd_vel_guard_input')", source)
        self.assertIn("('cmd_vel_smoothed', 'cmd_vel_guard_input')", source)
        self.assertIn('"-m", "nav2_wrapper.velocity_guard"', source)
        guard = (ROOT / "nav2_wrapper" / "velocity_guard.py").read_text()
        self.assertIn('create_publisher(Twist, "/cmd_vel"', guard)
        self.assertIn('CancelGoal, "/navigate_to_pose/_action/cancel_goal"', guard)
        self.assertIn('LaserScan, "/scanner/scan"', guard)
        self.assertIn('"scan-anomalies.jsonl"', guard)
        self.assertIn('baseline - front_min >= 0.40', guard)
        bridge = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn('"scan:=/scanner/scan_raw"', bridge)
        self.assertIn('"-m", "nav2_wrapper.scan_filter"', bridge)

    def test_failed_init_cleans_up_all_nav2_children(self):
        source = (ROOT / "nav2_wrapper" / "atlas_bridge.py").read_text()
        self.assertIn(
            'except Exception as e:  # noqa: BLE001\n        _kill_nav2()\n'
            '        return Err(f"spawn nav2 failed: {e}")',
            source,
        )
        self.assertIn(
            'if not _wait_for_action(action_wait):\n'
            '        # A failed Driver.Init must not orphan controller, scan, or guard',
            source,
        )


if __name__ == "__main__":
    unittest.main()
