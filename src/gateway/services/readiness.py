"""Readiness snapshot helpers for GatewayModule."""

from __future__ import annotations

import time
from typing import Any


READINESS_SCHEMA_VERSION = 1


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
            module_health[name] = {"ok": True, "detail": detail}
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
