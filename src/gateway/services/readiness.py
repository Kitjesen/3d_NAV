"""Readiness snapshot helpers for GatewayModule."""

from __future__ import annotations

import time
import math
from dataclasses import asdict, is_dataclass
from collections.abc import Mapping
from numbers import Real
from typing import Any


READINESS_SCHEMA_VERSION = 1

_LOCALIZATION_BLOCKING_STATES = {
    "no_odometry",
    "initializing",
    "degraded",
    "lost",
    "relocalizing",
}


def _json_safe(value: Any) -> Any:
    """Return a JSON-compliant value for readiness diagnostics."""
    if value is None or isinstance(value, (bool, str)):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Real):
        number = float(value)
        return number if math.isfinite(number) else None
    if is_dataclass(value):
        return _json_safe(asdict(value))
    if hasattr(value, "model_dump"):
        try:
            return _json_safe(value.model_dump())
        except Exception:
            return str(value)
    if isinstance(value, dict):
        return {str(_json_safe(key)): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item) for item in value]
    return str(value)


def _requires_runtime_readiness(gw: Any, modules: Mapping[str, Any]) -> bool:
    """Return True when /ready should include robot runtime readiness.

    Stub/dev stacks may intentionally have no SLAM or odometry. Real robot
    stacks load SLAM/localizer/navigation modules or expose localization
    evidence, so module liveness alone is too optimistic there.
    """
    names = " ".join(str(name).lower() for name in modules)
    if any(
        token in names
        for token in (
            "slam",
            "localizer",
            "navigation",
            "cmdvelmux",
            "cmd_vel_mux",
        )
    ):
        return True

    session_mode = str(getattr(gw, "_session_mode", "") or "").lower()
    if session_mode in {"mapping", "navigating", "exploring"}:
        return True

    lock = getattr(gw, "_state_lock", None)
    if lock is None:
        return False
    try:
        with lock:
            return (
                getattr(gw, "_odom", None) is not None
                or getattr(gw, "_localization_status", None) is not None
                or getattr(gw, "_mission", None) is not None
            )
    except Exception:
        return False


def _runtime_readiness_reasons(gw: Any) -> tuple[list[str], dict[str, Any]]:
    from gateway.services.runtime_status import (
        build_localization_status,
        build_navigation_status,
    )

    runtime: dict[str, Any] = {}
    reasons: list[str] = []
    try:
        localization = build_localization_status(gw)
        runtime["localization"] = {
            "state": localization.get("state"),
            "ready": localization.get("ready"),
            "pose_fresh": localization.get("pose_fresh"),
            "pose_freshness": localization.get("pose_freshness"),
            "algorithm_healthy": localization.get("algorithm_healthy"),
            "reasons": localization.get("reasons", []),
        }
        state = str(localization.get("state") or "unknown").lower()
        if state in _LOCALIZATION_BLOCKING_STATES:
            reasons.append(f"localization:{state}")
        if localization.get("pose_fresh") is False:
            reasons.append("localization:pose_stale")
    except Exception as exc:
        runtime["localization"] = {"error": str(exc)}
        reasons.append("localization:status_error")

    try:
        navigation = build_navigation_status(gw)
        readiness = navigation.get("readiness", {})
        blockers = list(readiness.get("blockers") or [])
        runtime["navigation"] = {
            "state": navigation.get("state"),
            "can_accept_goal": navigation.get("can_accept_goal"),
            "blockers": blockers,
            "advisories": list(readiness.get("advisories") or []),
        }
        reasons.extend(f"navigation_blocked:{blocker}" for blocker in blockers)
    except Exception as exc:
        runtime["navigation"] = {"error": str(exc)}
        reasons.append("navigation:status_error")

    return list(dict.fromkeys(reasons)), runtime


def build_readiness_snapshot(
    gw: Any,
    now: float | None = None,
    *,
    include_details: bool = True,
) -> tuple[dict[str, Any], int]:
    """Return a stable /ready payload plus the HTTP status code."""
    ts = time.time() if now is None else now
    modules = getattr(gw, "_all_modules", None) or {}
    if not modules:
        return (
            {
                "schema_version": READINESS_SCHEMA_VERSION,
                "status": "not_started",
                "ready": False,
                "modules": {},
                "module_count": 0,
                "failed_modules": [],
                "reasons": ["no_modules_loaded"],
                "ts": ts,
            },
            503,
        )

    module_health: dict[str, Any] = {}
    failed_modules: list[str] = []

    for name, mod in modules.items():
        try:
            module_health[name] = {"ok": True}
            if include_details:
                detail = mod.health() if hasattr(mod, "health") else {}
                module_health[name]["detail"] = _json_safe(detail)
        except Exception as exc:
            failed_modules.append(str(name))
            module_health[name] = {"ok": False, "error": str(exc)}

    reasons = [
        f"module_failed:{name}"
        for name in failed_modules
    ]
    runtime: dict[str, Any] = {}
    if _requires_runtime_readiness(gw, modules):
        runtime_reasons, runtime = _runtime_readiness_reasons(gw)
        reasons.extend(runtime_reasons)

    ready = not failed_modules and not reasons
    payload = {
        "schema_version": READINESS_SCHEMA_VERSION,
        "status": "ready" if ready else "degraded",
        "ready": ready,
        "modules": module_health,
        "module_count": len(modules),
        "failed_modules": failed_modules,
        "reasons": reasons,
        "runtime": runtime,
        "ts": ts,
    }
    return payload, 200 if ready else 503
