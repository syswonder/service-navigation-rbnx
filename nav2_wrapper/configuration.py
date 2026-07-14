"""Deployment-owned configuration helpers for the Nav2 wrapper."""

from __future__ import annotations

import os
import logging
from pathlib import Path


log = logging.getLogger("nav2_wrapper")
PACKAGE_ROOT = Path(__file__).resolve().parent.parent
LEGACY_ROOT = Path(__file__).resolve().parent / "legacy_config"
LEGACY_PROFILE_FILES = {
    "default": "nav2_params.yml",
    "slam": "nav2_params_slam.yml",
    "sim": "nav2_params_sim.yml",
    "ranger_mini_v3": "nav2_params_ranger_mini_v3.yml",
}


def deployment_root() -> Path:
    """Return the directory containing the active robot manifest."""
    raw = os.environ.get("RBNX_INVOCATION_CWD", "").strip()
    return Path(raw).expanduser().resolve() if raw else Path.cwd().resolve()


def resolve_deployment_file(value: object, field: str) -> Path:
    raw = str(value or "").strip()
    if not raw:
        raise ValueError(f"navigation config requires {field}")
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = deployment_root() / path
    path = path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"{field} not found: {path}")
    return path


def resolve_params_file(cfg: dict) -> Path:
    profile = str(cfg.get("params_profile") or "").strip()
    if profile:
        filename = LEGACY_PROFILE_FILES.get(profile)
        if filename is None:
            raise ValueError(
                f"unknown legacy params_profile {profile!r}; "
                f"known values: {sorted(LEGACY_PROFILE_FILES)}"
            )
        log.warning(
            "DEPRECATED config.params_profile=%s; copy %s into the robot "
            "deploy repository and use config.params_file instead",
            profile,
            filename,
        )
        if cfg.get("params_file"):
            log.warning("config.params_file overrides deprecated params_profile")
        else:
            return (LEGACY_ROOT / filename).resolve()
    return resolve_deployment_file(cfg.get("params_file"), "params_file")


def resolve_bt_xml_file(cfg: dict) -> Path | None:
    raw = cfg.get("bt_xml_file")
    if raw:
        return resolve_deployment_file(raw, "bt_xml_file")
    if str(cfg.get("params_profile") or "").strip() == "ranger_mini_v3":
        log.warning(
            "DEPRECATED ranger_mini_v3 profile is using its packaged BT XML; "
            "copy it into the deploy repository and set config.bt_xml_file"
        )
        return (LEGACY_ROOT / "ranger_mini_v3_navigate.xml").resolve()
    return None


def scan_projection_config(cfg: dict) -> dict[str, object]:
    raw = cfg.get("scan_projection")
    if raw is None:
        return {"enabled": False}
    if not isinstance(raw, dict):
        raise ValueError("scan_projection must be a mapping")
    allowed = {
        "enabled",
        "target_frame",
        "min_height_m",
        "max_height_m",
        "range_max_m",
        "self_filter_margin_m",
        "transform_tolerance_s",
        "deskewing",
        "deskew_fixed_frame",
        "deskew_wait_for_transform_s",
    }
    unknown = set(raw) - allowed
    if unknown:
        raise ValueError(f"unknown scan_projection field(s): {sorted(unknown)}")
    normalized: dict[str, object] = {
        "enabled": bool(raw.get("enabled", False)),
        "target_frame": str(raw.get("target_frame") or "").strip(),
        "min_height_m": float(raw.get("min_height_m", 0.0)),
        "max_height_m": float(raw.get("max_height_m", 2.0)),
        "range_max_m": float(raw.get("range_max_m", 30.0)),
        "self_filter_margin_m": float(raw.get("self_filter_margin_m", 0.05)),
        "transform_tolerance_s": float(raw.get("transform_tolerance_s", 0.15)),
        "deskewing": bool(raw.get("deskewing", False)),
        "deskew_fixed_frame": str(raw.get("deskew_fixed_frame") or "odom").strip(),
        "deskew_wait_for_transform_s": float(
            raw.get("deskew_wait_for_transform_s", 0.2)
        ),
    }
    if normalized["min_height_m"] >= normalized["max_height_m"]:
        raise ValueError("scan_projection min_height_m must be less than max_height_m")
    for key in (
        "range_max_m",
        "self_filter_margin_m",
        "transform_tolerance_s",
        "deskew_wait_for_transform_s",
    ):
        if normalized[key] < 0:
            raise ValueError(f"scan_projection {key} must be non-negative")
    return normalized
