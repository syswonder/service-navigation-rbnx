import math
import unittest

from nav2_wrapper.speed_control import (
    SpeedSettings,
    decide_speed,
    speed_settings,
)


class SpeedSettingsTest(unittest.TestCase):
    def test_defaults_leave_headroom_for_faster_and_slower(self):
        settings = speed_settings({})
        self.assertEqual(settings.default_percentage, 80.0)
        self.assertEqual(settings.step_percentage, 20.0)
        self.assertEqual(settings.min_percentage, 20.0)
        self.assertEqual(settings.topic, "/speed_limit")

    def test_deploy_can_select_headroom_and_relative_step(self):
        settings = speed_settings(
            {
                "dynamic_speed": {
                    "default_percentage": 70,
                    "step_percentage": 15,
                    "min_percentage": 25,
                    "topic": "/robot/navigation_speed_limit",
                }
            }
        )
        self.assertEqual(settings, SpeedSettings(70.0, 15.0, 25.0, "/robot/navigation_speed_limit"))

    def test_rejects_invalid_policy(self):
        invalid = (
            {"dynamic_speed": []},
            {"dynamic_speed": {"unknown": 1}},
            {"dynamic_speed": {"default_percentage": 101}},
            {"dynamic_speed": {"min_percentage": 0}},
            {"dynamic_speed": {"step_percentage": math.inf}},
            {"dynamic_speed": {"topic": "speed_limit"}},
        )
        for cfg in invalid:
            with self.subTest(cfg=cfg), self.assertRaises(ValueError):
                speed_settings(cfg)


class SpeedDecisionTest(unittest.TestCase):
    def setUp(self):
        self.settings = SpeedSettings(
            default_percentage=70.0,
            step_percentage=20.0,
            min_percentage=20.0,
        )

    def test_relative_commands_change_an_active_limit_without_restart(self):
        slower = decide_speed(70.0, "slower", 0.0, self.settings)
        faster = decide_speed(slower.percentage, "faster", 0.0, self.settings)
        self.assertEqual(slower.percentage, 50.0)
        self.assertEqual(faster.percentage, 70.0)

    def test_relative_commands_are_safely_bounded(self):
        self.assertEqual(
            decide_speed(95.0, "faster", 0.0, self.settings).percentage,
            100.0,
        )
        decision = decide_speed(20.0, "slower", 0.0, self.settings)
        self.assertEqual(decision.percentage, 20.0)
        self.assertFalse(decision.changed)

    def test_set_and_reset(self):
        self.assertEqual(
            decide_speed(70.0, "set", 42.5, self.settings).percentage,
            42.5,
        )
        self.assertEqual(
            decide_speed(42.5, "reset", 0.0, self.settings).percentage,
            70.0,
        )

    def test_rejects_unknown_or_unsafe_explicit_values(self):
        for operation, value in (
            ("", 0.0),
            ("fastest", 0.0),
            ("set", 19.9),
            ("set", 100.1),
            ("set", math.nan),
        ):
            with self.subTest(operation=operation, value=value), self.assertRaises(ValueError):
                decide_speed(70.0, operation, value, self.settings)


if __name__ == "__main__":
    unittest.main()
