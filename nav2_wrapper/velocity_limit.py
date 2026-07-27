"""ROS-independent helpers for enforcing navigation velocity ceilings."""

from __future__ import annotations

import math


def bounded_linear_velocity(
    x: float,
    y: float,
    max_linear_speed_mps: float,
) -> tuple[float, float, bool]:
    """Scale planar velocity to the configured cap while preserving direction."""
    speed = math.hypot(x, y)
    if speed <= max_linear_speed_mps or speed == 0.0:
        return float(x), float(y), False
    scale = max_linear_speed_mps / speed
    return float(x) * scale, float(y) * scale, True


def bounded_angular_velocity(
    z: float,
    max_angular_speed_radps: float,
) -> tuple[float, bool]:
    """Clamp yaw rate symmetrically while preserving rotation direction."""
    value = float(z)
    bounded = min(max(value, -max_angular_speed_radps), max_angular_speed_radps)
    return bounded, not math.isclose(
        bounded,
        value,
        rel_tol=0.0,
        abs_tol=1e-12,
    )
