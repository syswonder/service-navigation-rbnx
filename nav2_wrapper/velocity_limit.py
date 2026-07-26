"""ROS-independent helpers for enforcing a planar navigation speed cap."""

from __future__ import annotations

import math


def bounded_linear_velocity(
    x: float,
    y: float,
    max_speed_xy_mps: float,
) -> tuple[float, float, bool]:
    """Scale planar velocity to the configured cap while preserving direction."""
    speed = math.hypot(x, y)
    if speed <= max_speed_xy_mps or speed == 0.0:
        return float(x), float(y), False
    scale = max_speed_xy_mps / speed
    return float(x) * scale, float(y) * scale, True
