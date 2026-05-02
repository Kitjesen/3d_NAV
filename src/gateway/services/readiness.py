"""Readiness snapshot helpers for GatewayModule."""

from __future__ import annotations

import time
import math
from dataclasses import asdict, is_dataclass
from numbers import Real
from typing import Any


READINESS_SCHEMA_VERSION = 1


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


def build_readiness_snapshot(gw: Any, now: float | None = None) -> tuple[dict[str, Any], int]:
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
            detail = mod.health() if hasattr(mod, "health") else {}
            module_health[name] = {"ok": True, "detail": _json_safe(detail)}
        except Exception as exc:
            failed_modules.append(str(name))
            module_health[name] = {"ok": False, "error": str(exc)}

    ready = not failed_modules
    payload = {
        "schema_version": READINESS_SCHEMA_VERSION,
        "status": "ready" if ready else "degraded",
        "ready": ready,
        "modules": module_health,
        "module_count": len(modules),
        "failed_modules": failed_modules,
        "reasons": [
            f"module_failed:{name}"
            for name in failed_modules
        ],
        "ts": ts,
    }
    return payload, 200 if ready else 503
