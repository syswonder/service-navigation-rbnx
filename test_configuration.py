import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from nav2_wrapper.configuration import (
    DEFAULT_VELOCITY_OUTPUT_TOPIC,
    EXTERNAL_VELOCITY_GUARD_INPUT_TOPIC,
    VELOCITY_OUTPUT_TOPIC_ENV,
    render_python_expression_bool,
    resolve_controller_velocity_output_topic,
    resolve_external_velocity_guard,
    resolve_bt_xml_file,
    resolve_bt_through_poses_xml_file,
    resolve_params_file,
    resolve_trajectory_log_dir,
    resolve_use_composition,
    resolve_velocity_output_topic,
    scan_projection_config,
)


class DeploymentConfigurationTest(unittest.TestCase):
    def test_relative_files_resolve_from_manifest_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            params = root / "config" / "nav2_params.yaml"
            bt = root / "config" / "navigate.xml"
            bt_through = root / "config" / "navigate_through_poses.xml"
            params.parent.mkdir()
            params.write_text("controller_server: {}\n")
            bt.write_text("<root/>\n")
            bt_through.write_text("<root/>\n")
            with patch.dict(os.environ, {"RBNX_INVOCATION_CWD": directory}):
                self.assertEqual(
                    resolve_params_file({"params_file": "config/nav2_params.yaml"}),
                    params.resolve(),
                )
                self.assertEqual(
                    resolve_bt_xml_file({"bt_xml_file": "config/navigate.xml"}),
                    bt.resolve(),
                )
                self.assertEqual(
                    resolve_bt_through_poses_xml_file(
                        {
                            "bt_through_poses_xml_file":
                                "config/navigate_through_poses.xml"
                        }
                    ),
                    bt_through.resolve(),
                )

    def test_through_poses_tree_is_optional(self):
        self.assertIsNone(resolve_bt_through_poses_xml_file({}))

    def test_params_file_is_required(self):
        with self.assertRaisesRegex(ValueError, "requires params_file"):
            resolve_params_file({})

    def test_trajectory_log_defaults_to_private_runtime_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory) / "provider-runtime"
            self.assertEqual(
                resolve_trajectory_log_dir({}, runtime),
                runtime / "trajectories",
            )

            persistent = Path(directory) / "persistent-traces"
            self.assertEqual(
                resolve_trajectory_log_dir(
                    {"trajectory_log_dir": str(persistent)}, runtime
                ),
                persistent,
            )

    def test_trajectory_log_rejects_explicit_empty_path(self):
        with self.assertRaisesRegex(ValueError, "must not be empty"):
            resolve_trajectory_log_dir({"trajectory_log_dir": "  "}, Path("/tmp"))

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

    def test_velocity_output_topic_defaults_to_cmd_vel(self):
        self.assertEqual(
            resolve_velocity_output_topic({}, {}),
            DEFAULT_VELOCITY_OUTPUT_TOPIC,
        )

    def test_controller_velocity_output_topic_is_strict_opt_in(self):
        self.assertIsNone(resolve_controller_velocity_output_topic({}))
        self.assertEqual(
            resolve_controller_velocity_output_topic(
                {
                    "controller_velocity_output_topic":
                        "/go2/robottrack/nav_cmd_vel_raw"
                }
            ),
            "/go2/robottrack/nav_cmd_vel_raw",
        )
        for topic in (
            "",
            "cmd_vel_raw",
            "/",
            "/go2/robottrack/",
            "/go2//nav_cmd_vel_raw",
        ):
            with self.subTest(topic=topic), self.assertRaises(ValueError):
                resolve_controller_velocity_output_topic(
                    {"controller_velocity_output_topic": topic}
                )

    def test_use_composition_is_strict_and_defaults_off(self):
        self.assertFalse(resolve_use_composition({}))
        self.assertTrue(resolve_use_composition({"use_composition": True}))
        self.assertFalse(resolve_use_composition({"use_composition": False}))
        for value in ("true", "false", 0, 1, None, [], {}):
            with self.subTest(value=value), self.assertRaisesRegex(
                ValueError, "must be a boolean"
            ):
                resolve_use_composition({"use_composition": value})

    def test_external_velocity_guard_is_strict_and_exactly_routed(self):
        self.assertFalse(resolve_external_velocity_guard({}))
        self.assertFalse(
            resolve_external_velocity_guard(
                {
                    "external_velocity_guard": False,
                    "velocity_output_topic": "/cmd_vel",
                }
            )
        )
        self.assertTrue(
            resolve_external_velocity_guard(
                {
                    "external_velocity_guard": True,
                    "velocity_output_topic": (
                        EXTERNAL_VELOCITY_GUARD_INPUT_TOPIC
                    ),
                }
            )
        )
        for value in ("true", "false", 0, 1, None, [], {}):
            with self.subTest(value=value), self.assertRaisesRegex(
                ValueError, "must be a boolean"
            ):
                resolve_external_velocity_guard(
                    {"external_velocity_guard": value}
                )
        for topic in ("/cmd_vel", "/go2/staged_nav2/cmd_vel"):
            with self.subTest(topic=topic), self.assertRaisesRegex(
                ValueError, "requires velocity_output_topic"
            ):
                resolve_external_velocity_guard(
                    {
                        "external_velocity_guard": True,
                        "velocity_output_topic": topic,
                    }
                )

    def test_python_expression_bool_uses_exact_python_literal_spelling(self):
        self.assertEqual(render_python_expression_bool(True), "True")
        self.assertEqual(render_python_expression_bool(False), "False")
        self.assertNotEqual(render_python_expression_bool(True), "true")
        self.assertNotEqual(render_python_expression_bool(False), "false")
        for value in ("true", "false", 0, 1, None):
            with self.subTest(value=value), self.assertRaisesRegex(
                ValueError, "must be a boolean"
            ):
                render_python_expression_bool(value)

    def test_velocity_output_topic_supports_env_and_config_override(self):
        environment = {
            VELOCITY_OUTPUT_TOPIC_ENV: "/robonix/nomotion/cmd_vel",
        }
        self.assertEqual(
            resolve_velocity_output_topic({}, environment),
            "/robonix/nomotion/cmd_vel",
        )
        self.assertEqual(
            resolve_velocity_output_topic(
                {"velocity_output_topic": "/cmd_vel"}, environment
            ),
            "/cmd_vel",
        )

    def test_velocity_output_topic_fails_closed_on_empty_or_relative_values(self):
        invalid = ("", "cmd_vel", "/", "/cmd_vel/", "/cmd//vel", "/9cmd_vel")
        for topic in invalid:
            with self.subTest(topic=topic):
                with self.assertRaises(ValueError):
                    resolve_velocity_output_topic(
                        {"velocity_output_topic": topic},
                        {},
                    )

        with self.assertRaisesRegex(ValueError, VELOCITY_OUTPUT_TOPIC_ENV):
            resolve_velocity_output_topic({}, {VELOCITY_OUTPUT_TOPIC_ENV: ""})


if __name__ == "__main__":
    unittest.main()
