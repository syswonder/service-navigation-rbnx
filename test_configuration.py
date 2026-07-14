import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from nav2_wrapper.configuration import (
    resolve_bt_xml_file,
    resolve_params_file,
    scan_projection_config,
)


class DeploymentConfigurationTest(unittest.TestCase):
    def test_relative_files_resolve_from_manifest_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            params = root / "config" / "nav2_params.yaml"
            bt = root / "config" / "navigate.xml"
            params.parent.mkdir()
            params.write_text("controller_server: {}\n")
            bt.write_text("<root/>\n")
            with patch.dict(os.environ, {"RBNX_INVOCATION_CWD": directory}):
                self.assertEqual(
                    resolve_params_file({"params_file": "config/nav2_params.yaml"}),
                    params.resolve(),
                )
                self.assertEqual(
                    resolve_bt_xml_file({"bt_xml_file": "config/navigate.xml"}),
                    bt.resolve(),
                )

    def test_params_file_is_required(self):
        with self.assertRaisesRegex(ValueError, "requires params_file"):
            resolve_params_file({})

    def test_legacy_profile_still_resolves_with_migration_warning(self):
        with self.assertLogs("nav2_wrapper", level="WARNING") as logs:
            resolved = resolve_params_file({"params_profile": "sim"})
        self.assertEqual(resolved.name, "nav2_params_sim.yml")
        self.assertIn("DEPRECATED", "\n".join(logs.output))

    def test_unknown_legacy_profile_fails(self):
        with self.assertRaisesRegex(ValueError, "unknown legacy params_profile"):
            resolve_params_file({"params_profile": "other_robot"})

    def test_scan_projection_is_explicit_and_validated(self):
        self.assertFalse(scan_projection_config({})["enabled"])
        values = scan_projection_config(
            {
                "scan_projection": {
                    "enabled": True,
                    "min_height_m": 0.3,
                    "max_height_m": 1.4,
                    "range_max_m": 12.0,
                }
            }
        )
        self.assertTrue(values["enabled"])
        self.assertEqual(values["min_height_m"], 0.3)
        self.assertEqual(values["range_max_m"], 12.0)

    def test_scan_projection_rejects_unknown_fields(self):
        with self.assertRaisesRegex(ValueError, "unknown scan_projection"):
            scan_projection_config({"scan_projection": {"height": 1.0}})


if __name__ == "__main__":
    unittest.main()
