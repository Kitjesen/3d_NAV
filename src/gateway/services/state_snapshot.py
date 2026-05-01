"""Stable state snapshot helpers for GatewayModule."""

from __future__ import annotations

import time
from typing import Any

from gateway.services.app_bootstrap import (
    CLIENT_LINKS,
    _localization_summary,
    _map_summary,
    _safe_lease,
    _safe_session,
)


STATE_SNAPSHOT_SCHEMA_VERSION = 1


def build_state_snapshot(gw: Any) -> dict[str, Any]:
    """Return /api/v1/state while preserving legacy top-level fields."""
    with gw._state_lock:
        odometry = gw._odom
        safety = gw._safety
        mission = gw._mission
        eval_state = gw._eval
        dialogue = gw._dialogue
        mode = gw._mode
        teleop_active = gw._teleop_active
        teleop_clients = gw._teleop_clients
        scene_graph_json = gw._sg_json
        path_len = len(gw._last_path)
        localization_status = getattr(gw, "_localization_status", None)

    session = _safe_session(gw)
    localization = _localization_summary(
        odometry,
        session,
        float(getattr(gw, "_icp_quality", 0.0)),
        localization_status,
    )

    return {
        "schema_version": STATE_SNAPSHOT_SCHEMA_VERSION,
        "server": {
            "api_version": "v1",
            "time": time.time(),
        },
        # Legacy fields kept stable for existing dashboards/scripts.
        "odometry": odometry,
        "safety": safety,
        "mission": mission,
        "eval": eval_state,
        "dialogue": dialogue,
        "mode": mode,
        "lease": _safe_lease(gw),
        "teleop": {
            "active": bool(teleop_active),
            "clients": int(teleop_clients),
        },
        # New app/web-friendly summaries.
        "session": session,
        "localization": localization,
        "map": _map_summary(gw, session),
        "scene": {
            "available": bool(scene_graph_json) and scene_graph_json != "{}",
            "endpoint": CLIENT_LINKS["scene_graph"],
        },
        "path": {
            "points": path_len,
            "endpoint": CLIENT_LINKS["path"],
        },
        "links": {
            "bootstrap": CLIENT_LINKS["bootstrap"],
            "capabilities": CLIENT_LINKS["capabilities"],
            "events": CLIENT_LINKS["events"],
        },
    }
