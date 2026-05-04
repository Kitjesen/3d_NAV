"""Gateway helpers for interpreting safety state snapshots."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import asdict, is_dataclass
from numbers import Real
from typing import Any


SAFETY_STOP_BLOCKER = "safety_stop"

_SAFE_TEXT = {"", "0", "ok", "safe", "clear", "normal"}
_WARN_TEXT = {"1", "warn", "warning", "degraded"}
_STOP_TEXT = {"2", "stop", "stopped", "unsafe", "estop", "e_stop", "emergency_stop"}


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    if is_dataclass(value):
        try:
            return asdict(value)
        except Exception:
            return {}
    if hasattr(value, "model_dump"):
        try:
            data = value.model_dump()
            return dict(data) if isinstance(data, Mapping) else {}
        except Exception:
            return {}
    return {}


def normalized_safety_level(safety: Any) -> int | str | None:
    """Return a stable safety level from dict/dataclass/string-like inputs."""
    raw = _mapping(safety)
    level = raw.get("level", safety if safety is not None and not raw else None)
    if level is None:
        return None
    if hasattr(level, "value"):
        level = level.value
    if isinstance(level, bool):
        return 0 if level else 2
    if isinstance(level, int):
        return level
    if isinstance(level, Real):
        return int(level)
    token = str(level).strip().lower()
    if token in _SAFE_TEXT:
        return 0
    if token in _WARN_TEXT:
        return 1
    if token in _STOP_TEXT:
        return 2
    try:
        return int(float(token))
    except (TypeError, ValueError):
        return token


def safety_stop_active(safety: Any) -> bool:
    """Return True only for safety states that must block motion commands."""
    level = normalized_safety_level(safety)
    if isinstance(level, int):
        return level >= 2
    return str(level).strip().lower() in _STOP_TEXT


def safety_clear_for_motion(safety: Any) -> bool:
    return not safety_stop_active(safety)


def safety_summary(safety: Any) -> dict[str, Any]:
    raw = _mapping(safety)
    level = normalized_safety_level(safety)
    stop_active = safety_stop_active(safety)
    return {
        "level": level,
        "ok": level in (None, 0),
        "stop_active": stop_active,
        "motion_allowed": not stop_active,
        "raw": raw,
    }
