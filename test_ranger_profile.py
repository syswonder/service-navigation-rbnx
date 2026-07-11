import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parent


class RangerProfileTest(unittest.TestCase):
    def setUp(self):
        self.params = yaml.safe_load(
            (ROOT / "config" / "nav2_params_ranger_mini_v3.yml").read_text()
        )
        self.controller = self.params["controller_server"]["ros__parameters"]

    def test_v01_controller_owns_final_heading(self):
        follow = self.controller["FollowPath"]
        self.assertEqual(follow["plugin"], "dwb_core::DWBLocalPlanner")
        self.assertIn("RotateToGoal", follow["critics"])
        self.assertNotIn("primary_controller", follow)

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


if __name__ == "__main__":
    unittest.main()
