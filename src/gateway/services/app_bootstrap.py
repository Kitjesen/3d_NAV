"""Lightweight App/Web bootstrap contract for GatewayModule."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any


APP_BOOTSTRAP_SCHEMA_VERSION = 1
APP_CAPABILITIES_SCHEMA_VERSION = 1

CLIENT_LINKS: dict[str, str] = {
    "bootstrap": "/api/v1/app/bootstrap",
    "capabilities": "/api/v1/app/capabilities",
    "state": "/api/v1/state",
    "scene_graph": "/api/v1/scene_graph",
    "locations": "/api/v1/locations",
    "path": "/api/v1/path",
    "devices": "/api/v1/devices",
    "health": "/api/v1/health",
    "session": "/api/v1/session",
    "events": "/api/v1/events",
    "teleop_ws": "/ws/teleop",
    "cloud_ws": "/ws/cloud",
    "camera_snapshot": "/api/v1/camera/snapshot",
    "webrtc_stats": "/api/v1/webrtc/stats",
    "webrtc_offer": "/api/v1/webrtc/offer",
    "webrtc_bitrate": "/api/v1/webrtc/bitrate",
    "webrtc_whep": "/api/v1/webrtc/whep",
    "go2rtc_status": "/api/v1/webrtc/go2rtc/status",
    "goal": "/api/v1/goal",
    "navigate_click": "/api/v1/navigate/click",
    "stop": "/api/v1/stop",
    "instruction": "/api/v1/instruction",
    "mode": "/api/v1/mode",
    "lease": "/api/v1/lease",
    "maps": "/api/v1/slam/maps",
    "map_lifecycle": "/api/v1/maps",
    "map_points": "/api/v1/map/points",
    "saved_map_points": "/api/v1/maps/{name}/points",
    "explore_status": "/api/v1/explore/status",
    "explore_start": "/api/v1/explore/start",
    "explore_stop": "/api/v1/explore/stop",
    "slam_status": "/api/v1/slam/status",
    "slam_switch": "/api/v1/slam/switch",
    "slam_auto_relocalize": "/api/v1/slam/auto_relocalize",
    "slam_relocalize": "/api/v1/slam/relocalize",
    "bag_start": "/api/v1/bag/start",
    "bag_stop": "/api/v1/bag/stop",
    "bag_status": "/api/v1/bag/status",
    "memory_temporal": "/api/v1/memory/temporal",
    "memory_temporal_semantic": "/api/v1/memory/temporal/semantic",
    "diagnostic_pack": "/api/v1/diagnostic_pack",
}

CLIENT_ENDPOINTS: dict[str, dict[str, dict[str, str]]] = {
    "app": {
        "bootstrap": {"method": "GET", "path": CLIENT_LINKS["bootstrap"]},
        "capabilities": {"method": "GET", "path": CLIENT_LINKS["capabilities"]},
    },
    "state": {
        "snapshot": {"method": "GET", "path": CLIENT_LINKS["state"]},
        "scene_graph": {"method": "GET", "path": CLIENT_LINKS["scene_graph"]},
        "locations": {"method": "GET", "path": CLIENT_LINKS["locations"]},
        "path": {"method": "GET", "path": CLIENT_LINKS["path"]},
        "devices": {"method": "GET", "path": CLIENT_LINKS["devices"]},
        "health": {"method": "GET", "path": CLIENT_LINKS["health"]},
    },
    "realtime": {
        "events": {"method": "SSE", "path": CLIENT_LINKS["events"]},
        "teleop": {"method": "WS", "path": CLIENT_LINKS["teleop_ws"]},
        "cloud": {"method": "WS", "path": CLIENT_LINKS["cloud_ws"]},
    },
    "control": {
        "goal": {"method": "POST", "path": CLIENT_LINKS["goal"]},
        "navigate_click": {"method": "POST", "path": CLIENT_LINKS["navigate_click"]},
        "stop": {"method": "POST", "path": CLIENT_LINKS["stop"]},
        "instruction": {"method": "POST", "path": CLIENT_LINKS["instruction"]},
        "mode": {"method": "POST", "path": CLIENT_LINKS["mode"]},
        "lease": {"method": "POST", "path": CLIENT_LINKS["lease"]},
    },
    "media": {
        "camera_snapshot": {"method": "GET", "path": CLIENT_LINKS["camera_snapshot"]},
        "webrtc_stats": {"method": "GET", "path": CLIENT_LINKS["webrtc_stats"]},
        "webrtc_offer": {"method": "POST", "path": CLIENT_LINKS["webrtc_offer"]},
        "webrtc_bitrate": {"method": "POST", "path": CLIENT_LINKS["webrtc_bitrate"]},
        "webrtc_whep": {"method": "POST", "path": CLIENT_LINKS["webrtc_whep"]},
        "go2rtc_status": {"method": "GET", "path": CLIENT_LINKS["go2rtc_status"]},
    },
    "map": {
        "maps": {"method": "GET", "path": CLIENT_LINKS["maps"]},
        "map_lifecycle": {"method": "POST", "path": CLIENT_LINKS["map_lifecycle"]},
        "map_points": {"method": "GET", "path": CLIENT_LINKS["map_points"]},
        "saved_map_points": {"method": "GET", "path": CLIENT_LINKS["saved_map_points"]},
        "session": {"method": "GET", "path": CLIENT_LINKS["session"]},
        "explore_status": {"method": "GET", "path": CLIENT_LINKS["explore_status"]},
        "explore_start": {"method": "POST", "path": CLIENT_LINKS["explore_start"]},
        "explore_stop": {"method": "POST", "path": CLIENT_LINKS["explore_stop"]},
    },
    "ops": {
        "slam_status": {"method": "GET", "path": CLIENT_LINKS["slam_status"]},
        "slam_switch": {"method": "POST", "path": CLIENT_LINKS["slam_switch"]},
        "slam_auto_relocalize": {"method": "POST", "path": CLIENT_LINKS["slam_auto_relocalize"]},
        "slam_relocalize": {"method": "POST", "path": CLIENT_LINKS["slam_relocalize"]},
        "bag_start": {"method": "POST", "path": CLIENT_LINKS["bag_start"]},
        "bag_stop": {"method": "POST", "path": CLIENT_LINKS["bag_stop"]},
        "bag_status": {"method": "GET", "path": CLIENT_LINKS["bag_status"]},
        "memory_temporal": {"method": "GET", "path": CLIENT_LINKS["memory_temporal"]},
        "memory_temporal_semantic": {"method": "POST", "path": CLIENT_LINKS["memory_temporal_semantic"]},
        "diagnostic_pack": {"method": "GET", "path": CLIENT_LINKS["diagnostic_pack"]},
    },
}

PROBE_ENDPOINTS: dict[str, dict[str, str]] = {
    "liveness": {"method": "GET", "path": "/health"},
    "readiness": {"method": "GET", "path": "/ready"},
}


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    return {}


def _safe_session(gw: Any) -> dict[str, Any]:
    try:
        snapshot = gw._session_snapshot()
        if isinstance(snapshot, Mapping):
            return dict(snapshot)
    except Exception:
        pass
    return {
        "mode": getattr(gw, "_session_mode", "unknown"),
        "active_map": getattr(gw, "_session_map", None),
        "pending": bool(getattr(gw, "_session_pending", False)),
        "error": "session_snapshot_unavailable",
        "can_start_mapping": False,
        "can_start_navigating": False,
        "can_start_exploring": False,
        "can_end": False,
    }


def _safe_lease(gw: Any) -> dict[str, Any]:
    lease = getattr(gw, "_lease", None)
    if hasattr(lease, "to_dict"):
        try:
            data = lease.to_dict()
            if isinstance(data, Mapping):
                return dict(data)
        except Exception:
            pass
    return {}


def _mission_summary(mission: Any) -> dict[str, Any]:
    raw = _mapping(mission)
    return {
        "state": raw.get("state", raw.get("status", "idle")),
        "raw": raw,
    }


def _safety_summary(safety: Any) -> dict[str, Any]:
    raw = _mapping(safety)
    level = raw.get("level")
    return {
        "level": level,
        "ok": level in (None, 0, "ok", "safe"),
        "raw": raw,
    }


def _localization_summary(
    odometry: Any,
    session: Mapping[str, Any],
    icp_quality: float,
    status: Any,
) -> dict[str, Any]:
    diagnostics = _mapping(status)
    ready = bool(session.get("localizer_ready", False))
    mode = session.get("mode")
    has_odom = odometry is not None
    if not has_odom:
        state = "unknown"
    elif ready:
        state = "ready"
    elif mode == "navigating":
        state = "initializing" if icp_quality <= 0.0 else "degraded"
    else:
        state = "tracking"
    return {
        "state": state,
        "ready": ready,
        "has_odometry": has_odom,
        "icp_quality": icp_quality,
        "reported_state": diagnostics.get("state"),
        "confidence": diagnostics.get("confidence"),
        "degeneracy": diagnostics.get("degeneracy"),
        "ts": diagnostics.get("ts"),
    }


def _map_summary(gw: Any, session: Mapping[str, Any]) -> dict[str, Any]:
    lock = getattr(gw, "_map_cloud_lock", None)
    if lock is None:
        points = getattr(gw, "_map_points", None)
    else:
        with lock:
            points = getattr(gw, "_map_points", None)
    try:
        live_points = len(points) if points is not None else 0
    except TypeError:
        live_points = 0
    return {
        "active": session.get("active_map"),
        "has_live_cloud": live_points > 0,
        "live_points": live_points,
        "live_cloud_frames": int(getattr(gw, "_map_cloud_count", 0)),
        "has_manager": getattr(gw, "_map_mgr", None) is not None,
        "map_has_pcd": bool(session.get("map_has_pcd", False)),
        "map_has_tomogram": bool(session.get("map_has_tomogram", False)),
    }


def _traffic_summary(gw: Any) -> dict[str, Any]:
    try:
        snapshot = gw._traffic_stats_snapshot()
        if isinstance(snapshot, Mapping):
            return dict(snapshot)
    except Exception:
        pass
    return {
        "sse": {
            "clients": 0,
            "queue_maxsize": None,
            "drop_policy": "drop_oldest",
        },
        "cloud": {
            "clients": 0,
            "queue_maxsize": None,
            "drop_policy": "drop_oldest",
        },
        "recommended_client_rates_hz": {},
    }


def _command_policy(gw: Any) -> dict[str, Any]:
    try:
        snapshot = gw._command_stats_snapshot()
        if isinstance(snapshot, Mapping):
            return dict(snapshot)
    except Exception:
        pass
    return {
        "idempotency_supported": False,
        "request_id_field": "request_id",
        "client_id_field": "client_id",
        "rate_policy_hz": {},
        "rate_policy_enforcement": "unavailable",
    }


def _auth_summary() -> dict[str, Any]:
    try:
        from gateway.auth import _get_configured_key

        enabled = _get_configured_key() is not None
    except Exception:
        enabled = False
    return {
        "enabled": enabled,
        "scheme": "api_key" if enabled else "none",
        "header": "X-API-Key",
        "query_param": "api_key",
        "cookie": "lingtu_api_key",
        "public_endpoints": [
            "/",
            "/api/v1/auth/login",
            "/api/v1/auth/check",
            "/docs",
            "/redoc",
            "/openapi.json",
        ],
    }


def _feature_flags(gw: Any) -> dict[str, bool]:
    return {
        "state": True,
        "health": True,
        "devices": True,
        "mapping": True,
        "navigation": True,
        "exploration": getattr(gw, "_frontier_explorer", None) is not None,
        "teleop": True,
        "camera_snapshot": True,
        "webrtc": getattr(gw, "_webrtc", None) is not None,
        "scene_graph": True,
        "locations": True,
        "sessions": True,
    }


def _schema_name(schema: Any) -> str | None:
    raw = _mapping(schema)
    ref = raw.get("$ref")
    if isinstance(ref, str):
        return ref.rsplit("/", 1)[-1]
    for key in ("anyOf", "oneOf"):
        variants = raw.get(key)
        if isinstance(variants, list):
            names = [_schema_name(item) for item in variants]
            joined = "|".join(name for name in names if name)
            if joined:
                return joined
    typ = raw.get("type")
    return str(typ) if typ else None


def _operation_contracts(gw: Any) -> dict[tuple[str, str], dict[str, Any]]:
    app = getattr(gw, "_app", None)
    if app is None or not hasattr(app, "openapi"):
        return {}
    try:
        openapi = app.openapi()
    except Exception:
        return {}

    contracts: dict[tuple[str, str], dict[str, Any]] = {}
    for path, methods in _mapping(openapi.get("paths")).items():
        if not isinstance(methods, Mapping):
            continue
        for method, operation in methods.items():
            http_method = str(method).upper()
            if http_method not in {"GET", "POST", "PUT", "PATCH", "DELETE"}:
                continue
            op = _mapping(operation)
            request_content = (
                _mapping(op.get("requestBody")).get("content", {})
                if isinstance(op.get("requestBody"), Mapping)
                else {}
            )
            request_json = _mapping(_mapping(request_content).get("application/json"))
            responses = _mapping(op.get("responses"))
            response_200 = _mapping(responses.get("200"))
            response_content = _mapping(response_200.get("content"))
            response_json = _mapping(response_content.get("application/json"))
            contracts[(http_method, str(path))] = {
                "operation_id": op.get("operationId"),
                "request_schema": _schema_name(request_json.get("schema")),
                "response_schema": _schema_name(response_json.get("schema")),
                "response_content_types": sorted(response_content.keys()),
                "status_codes": sorted(str(code) for code in responses.keys()),
            }
    return contracts


def _enrich_endpoint_specs(
    gw: Any,
    groups: Mapping[str, Mapping[str, Mapping[str, str]]],
) -> dict[str, dict[str, dict[str, Any]]]:
    contracts = _operation_contracts(gw)
    enriched: dict[str, dict[str, dict[str, Any]]] = {}
    for group, endpoints in groups.items():
        enriched[group] = {}
        for name, spec in endpoints.items():
            item: dict[str, Any] = dict(spec)
            method = str(item.get("method", "")).upper()
            lookup_method = "GET" if method == "SSE" else method
            contract = contracts.get((lookup_method, str(item.get("path", ""))))
            if contract:
                item.update({k: v for k, v in contract.items() if v not in (None, [])})
            enriched[group][name] = item
    return enriched


def build_app_capabilities(gw: Any) -> dict[str, Any]:
    """Return stable client-facing API capabilities and traffic policies."""
    traffic = _traffic_summary(gw)
    return {
        "schema_version": APP_CAPABILITIES_SCHEMA_VERSION,
        "server": {
            "api_version": "v1",
            "time": time.time(),
        },
        "auth": _auth_summary(),
        "features": _feature_flags(gw),
        "endpoints": _enrich_endpoint_specs(gw, CLIENT_ENDPOINTS),
        "probes": _enrich_endpoint_specs(gw, {"probes": PROBE_ENDPOINTS})["probes"],
        "realtime": {
            "events": {
                "path": CLIENT_LINKS["events"],
                "transport": "sse",
                "initial_snapshot": True,
                "heartbeat_s": 1.0,
                "drop_policy": traffic.get("sse", {}).get("drop_policy"),
            },
            "teleop": {
                "path": CLIENT_LINKS["teleop_ws"],
                "transport": "websocket",
                "binary_camera_frames": True,
            },
            "cloud": {
                "path": CLIENT_LINKS["cloud_ws"],
                "transport": "websocket",
                "binary_point_cloud_frames": True,
                "drop_policy": traffic.get("cloud", {}).get("drop_policy"),
            },
        },
        "client_policy": {
            "poll_rates_hz": traffic.get("recommended_client_rates_hz", {}),
            "commands": _command_policy(gw),
            "retry_safe_when_request_id_present": True,
        },
        "links": dict(CLIENT_LINKS),
    }


def build_app_bootstrap(gw: Any) -> dict[str, Any]:
    """Return the first payload a mobile app or web client needs after login."""
    with gw._state_lock:
        odometry = gw._odom
        mission = gw._mission
        safety = gw._safety
        mode = gw._mode
        scene_graph_json = gw._sg_json
        path_len = len(gw._last_path)
        teleop_active = gw._teleop_active
        teleop_clients = gw._teleop_clients
        localization_status = getattr(gw, "_localization_status", None)

    session = _safe_session(gw)
    icp_quality = float(getattr(gw, "_icp_quality", 0.0))
    localization = _localization_summary(
        odometry,
        session,
        icp_quality,
        localization_status,
    )
    webrtc_available = getattr(gw, "_webrtc", None) is not None

    return {
        "schema_version": APP_BOOTSTRAP_SCHEMA_VERSION,
        "server": {
            "api_version": "v1",
            "time": time.time(),
        },
        "robot": {
            "online": True,
            "has_odometry": odometry is not None,
        },
        "session": session,
        "mission": _mission_summary(mission),
        "safety": _safety_summary(safety),
        "localization": localization,
        "control": {
            "mode": mode,
            "lease": _safe_lease(gw),
            "teleop": {
                "active": bool(teleop_active),
                "clients": int(teleop_clients),
            },
            "estop_clear": mode != "estop",
            "can_send_commands": mode != "estop" and not bool(session.get("pending")),
            "command_policy": _command_policy(gw),
        },
        "map": _map_summary(gw, session),
        "scene": {
            "available": bool(scene_graph_json) and scene_graph_json != "{}",
            "endpoint": CLIENT_LINKS["scene_graph"],
        },
        "path": {
            "points": path_len,
            "endpoint": CLIENT_LINKS["path"],
        },
        "media": {
            "events": CLIENT_LINKS["events"],
            "teleop_ws": CLIENT_LINKS["teleop_ws"],
            "cloud_ws": CLIENT_LINKS["cloud_ws"],
            "camera_snapshot": CLIENT_LINKS["camera_snapshot"],
            "webrtc_available": webrtc_available,
            "webrtc_offer": CLIENT_LINKS["webrtc_offer"],
        },
        "traffic": _traffic_summary(gw),
        "capabilities": _feature_flags(gw),
        "capabilities_endpoint": CLIENT_LINKS["capabilities"],
        "links": dict(CLIENT_LINKS),
    }
