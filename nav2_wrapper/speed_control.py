"""Pure policy for runtime navigation speed-limit commands."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class SpeedSettings:
    """Validated provider policy expressed as percentages of Nav2's maximum."""

    default_percentage: float = 80.0
    step_percentage: float = 20.0
    min_percentage: float = 20.0
    topic: str = "/speed_limit"


@dataclass(frozen=True)
class SpeedDecision:
    """Result of one relative, explicit, or reset speed command."""

    percentage: float
    changed: bool
    detail: str


def speed_settings(cfg: dict) -> SpeedSettings:
    """Validate the deploy-owned policy without importing ROS dependencies."""
    raw = cfg.get("dynamic_speed", {})
    if raw is None:
        raw = {}
    if not isinstance(raw, dict):
        raise ValueError("dynamic_speed must be a mapping")
    allowed = {
        "default_percentage",
        "step_percentage",
        "min_percentage",
        "topic",
    }
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"unknown dynamic_speed field(s): {sorted(unknown)}")

    settings = SpeedSettings(
        default_percentage=float(raw.get("default_percentage", 80.0)),
        step_percentage=float(raw.get("step_percentage", 20.0)),
        min_percentage=float(raw.get("min_percentage", 20.0)),
        topic=str(raw.get("topic", "/speed_limit")).strip(),
    )
    values = (
        settings.default_percentage,
        settings.step_percentage,
        settings.min_percentage,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("dynamic_speed percentages must be finite")
    if not 0.0 < settings.min_percentage <= settings.default_percentage <= 100.0:
        raise ValueError(
            "dynamic_speed requires 0 < min_percentage <= "
            "default_percentage <= 100"
        )
    if not 0.0 < settings.step_percentage <= 100.0:
        raise ValueError("dynamic_speed step_percentage must be in (0, 100]")
    if not settings.topic.startswith("/"):
        raise ValueError("dynamic_speed topic must be an absolute ROS topic")
    return settings


def decide_speed(
    current_percentage: float,
    operation: str,
    requested_percentage: float,
    settings: SpeedSettings,
) -> SpeedDecision:
    """Resolve a dynamic command while preserving the configured safety bounds."""
    command = operation.strip().lower()
    if command == "faster":
        target = min(100.0, current_percentage + settings.step_percentage)
    elif command == "slower":
        target = max(
            settings.min_percentage,
            current_percentage - settings.step_percentage,
        )
    elif command == "set":
        if not math.isfinite(requested_percentage):
            raise ValueError("percentage must be finite")
        if not settings.min_percentage <= requested_percentage <= 100.0:
            raise ValueError(
                "percentage must be between "
                f"{settings.min_percentage:g} and 100"
            )
        target = requested_percentage
    elif command == "reset":
        target = settings.default_percentage
    else:
        raise ValueError("operation must be faster, slower, set, or reset")

    target = round(float(target), 6)
    changed = not math.isclose(
        target, current_percentage, rel_tol=0.0, abs_tol=1e-9
    )
    if changed:
        detail = f"navigation speed changed to {target:g}%"
    else:
        detail = f"navigation speed already at {target:g}%"
    return SpeedDecision(target, changed, detail)
