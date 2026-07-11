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
        configured = self.params["bt_navigator"]["ros__parameters"][
            "default_bt_xml_filename"
        ]
        self.assertEqual(configured, "__ROBONIX_BT_XML__")
        tree = ET.parse(ROOT / "config" / "ranger_mini_v3_navigate.xml")
        root_recovery = tree.find(".//BehaviorTree/RecoveryNode")
        self.assertEqual(root_recovery.attrib["number_of_retries"], "1")
        rate = tree.find(".//RateController")
        self.assertEqual(rate.attrib["hz"], "5.0")
        spin = tree.find(".//Spin")
        self.assertEqual(spin.attrib["spin_dist"], "0.0")
        self.assertIsNone(tree.find(".//BackUp"))

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


if __name__ == "__main__":
    unittest.main()
