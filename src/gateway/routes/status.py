"""Status, telemetry, and health routes for GatewayModule."""

from __future__ import annotations

import asyncio
import time
from typing import Annotated, Any

from fastapi import Query
from fastapi.responses import JSONResponse, StreamingResponse

from gateway.schemas import (
    DevicesResponse,
    HealthResponse,
    LivenessResponse,
    LocalizationStatusResponse,
    LocationsResponse,
    NavigationStatusResponse,
    PathResponse,
    ReadinessResponse,
    SSEEventEnvelope,
    SceneGraphResponse,
    StateResponse,
)
from gateway.services.readiness import build_readiness_snapshot
from gateway.services.runtime_status import (
    build_localization_status,
    build_navigation_status,
)
from gateway.services.state_snapshot import build_state_snapshot
from gateway.services.traffic import (
    SSE_RETRY_MS,
    format_sse_message,
    normalize_sse_event,
)
from gateway.services.telemetry_normalizers import (
    build_locations_response,
    build_path_response,
    build_scene_graph_response,
)


def _probe_brainstem() -> dict[str, Any]:
    import brainstem_api as bapi
    import grpc

    ch = grpc.insecure_channel("127.0.0.1:13145")
    try:
        stub = bapi.RobotControlStub(ch)
        state = stub.GetCmsState(bapi.Empty(), timeout=1.0)
        fsm_map = {
            0: "ZERO",
            1: "GROUNDED",
            2: "STANDING",
            3: "WALKING",
            4: "TRANSITIONING",
        }
        info: dict[str, Any] = {
            "status": "connected",
            "host": "127.0.0.1:13145",
            "fsm": fsm_map.get(state.kind, str(state.kind)),
        }
        try:
            v = stub.GetVoltage(bapi.Empty(), timeout=1.0)
            if v.values:
                info["voltage_avg"] = round(sum(v.values) / len(v.values), 1)
        except Exception:
            pass
        return info
    finally:
        ch.close()


async def _brainstem_health(gw) -> dict[str, Any]:
    now = time.monotonic()
    ttl = float(getattr(gw, "_brainstem_health_cache_ttl_s", 0.0) or 0.0)
    lock = getattr(gw, "_brainstem_health_lock", None)
    if ttl > 0.0 and lock is not None:
        with lock:
            cached = getattr(gw, "_brainstem_health_cache", None)
            cache_ts = float(getattr(gw, "_brainstem_health_cache_ts", 0.0) or 0.0)
            age = now - cache_ts
            if cached is not None and age <= ttl:
                info = dict(cached)
                info["cached"] = True
                info["cache_age_s"] = round(max(0.0, age), 3)
                return info

    try:
        loop = asyncio.get_running_loop()
        info = await loop.run_in_executor(None, _probe_brainstem)
    except ImportError:
        info = {
            "status": "unavailable",
            "reason": "brainstem_api not installed",
        }
    except Exception as e:
        info = {
            "status": "unreachable",
            "host": "127.0.0.1:13145",
            "error": str(e)[:120],
        }

    info = dict(info)
    info["cached"] = False
    if ttl > 0.0 and lock is not None:
        with lock:
            gw._brainstem_health_cache = dict(info)
            gw._brainstem_health_cache_ts = time.monotonic()
    return info


def _health_module_needs_detail(name: str) -> bool:
    lowered = name.lower()
    return any(
        token in lowered
        for token in (
            "lidarmodule",
            "camera",
            "slambridge",
            "slammodule",
            "navigation",
        )
    )


def register_status_routes(app, gw) -> None:
    @app.get(
        "/api/v1/events",
        summary="SSE event stream",
        response_class=StreamingResponse,
        responses={
            200: {
                "content": {
                    "text/event-stream": {"schema": SSEEventEnvelope.model_json_schema()}
                }
            }
        },
    )
    async def sse_events():
        q, snapshot_event_id = gw._sse_subscribe_with_event_id()

        async def _stream():
            try:
                with gw._state_lock:
                    snapshot = {
                        "type": "snapshot",
                        "data": {
                            "odometry": gw._odom,
                            "safety": gw._safety,
                            "mission": gw._mission,
                            "mode": gw._mode,
                            "session": gw._session_snapshot(),
                        },
                    }
                yield format_sse_message(
                    normalize_sse_event(snapshot, event_id=snapshot_event_id),
                    retry_ms=SSE_RETRY_MS,
                )

                while True:
                    try:
                        event = await asyncio.wait_for(q.get(), timeout=1.0)
                    except asyncio.TimeoutError:
                        yield format_sse_message(
                            normalize_sse_event(
                                {"type": "ping"},
                                now=time.time(),
                            )
                        )
                        continue
                    yield format_sse_message(event)
                    await asyncio.sleep(0)
            finally:
                gw._sse_unsubscribe(q)

        return StreamingResponse(
            _stream(),
            media_type="text/event-stream",
            headers={"Cache-Control": "no-cache", "X-Accel-Buffering": "no"},
        )

    @app.get(
        "/api/v1/state",
        summary="Full robot state snapshot",
        response_model=StateResponse,
    )
    async def get_state():
        return build_state_snapshot(gw)

    @app.get(
        "/api/v1/scene_graph",
        summary="Current scene graph",
        response_model=SceneGraphResponse,
    )
    async def get_scene_graph():
        with gw._state_lock:
            sg = gw._sg_json
        return build_scene_graph_response(sg)

    @app.get(
        "/api/v1/locations",
        summary="List tagged navigation locations",
        response_model=LocationsResponse,
    )
    async def get_locations():
        tlm = gw._tagged_loc_module
        if tlm is None:
            return build_locations_response([])
        try:
            entries = list(tlm.store._store.values())
        except Exception:
            entries = []
        return build_locations_response(entries)

    @app.get(
        "/api/v1/path",
        summary="Latest planned path",
        response_model=PathResponse,
    )
    async def get_path():
        with gw._state_lock:
            path = gw._last_path
            robot = gw._odom
        return build_path_response(path, robot)

    @app.get(
        "/api/v1/localization/status",
        summary="Localization status for app and web clients",
        response_model=LocalizationStatusResponse,
    )
    async def get_localization_status():
        return build_localization_status(gw)

    @app.get(
        "/api/v1/navigation/status",
        summary="Navigation mission and control status",
        response_model=NavigationStatusResponse,
    )
    async def get_navigation_status():
        return build_navigation_status(gw)

    @app.get(
        "/api/v1/devices",
        summary="Hardware device registry status",
        response_model=DevicesResponse,
    )
    async def get_devices():
        mgr = gw._all_modules.get("DeviceManager") if gw._all_modules else None
        if mgr is None:
            return {"devices": [], "manager": "not_loaded"}
        try:
            health = mgr.health()
            return {
                "manager": "ok",
                "spec_count": health.get("spec_count", 0),
                "opened_count": health.get("opened_count", 0),
                "devices": health.get("devices", []),
            }
        except Exception as e:
            return {"devices": [], "manager": "error", "error": str(e)}

    @app.get(
        "/api/v1/health",
        summary="System health overview",
        response_model=HealthResponse,
    )
    async def get_health(
        details: Annotated[bool, Query(
            description="Probe every module health detail; default app polling path only probes displayed sensors.",
        )] = False
    ):
        traffic = gw._traffic_stats_snapshot()
        commands = gw._command_stats_snapshot()
        n_sse = traffic["sse"]["clients"]
        with gw._map_cloud_lock:
            map_pts = len(gw._map_points) if gw._map_points is not None else 0

        sensors: dict[str, Any] = {}
        modules_ok = 0
        modules_fail = 0
        module_summary: dict[str, str] = {}

        if gw._all_modules:
            for name, mod in gw._all_modules.items():
                probe_module = details or _health_module_needs_detail(str(name))
                if not probe_module:
                    module_summary[name] = "ok"
                    modules_ok += 1
                    continue
                try:
                    h = mod.health() if hasattr(mod, "health") else {}
                    module_summary[name] = "ok"
                    modules_ok += 1

                    if "LidarModule" in name:
                        lidar_h = h.get("lidar", {})
                        sensors["lidar"] = {
                            "status": lidar_h.get("state", "unknown"),
                            "ip": lidar_h.get("ip", "?"),
                            "cloud_hz": round(
                                h.get("ports_out", {})
                                .get("scan", {})
                                .get("rate_hz", 0),
                                1,
                            ),
                        }
                    elif "CameraBridge" in name:
                        color_out = h.get("ports_out", {}).get("color_image", {})
                        sensors["camera"] = {
                            "status": (
                                "streaming"
                                if color_out.get("msg_count", 0) > 0
                                else "idle"
                            ),
                            "fps": round(color_out.get("rate_hz", 0), 1),
                            "frames": color_out.get("msg_count", 0),
                        }
                    elif "SlamBridge" in name or "SLAMModule" in name:
                        odom_out = h.get("ports_out", {}).get("odometry", {})
                        slam_rate = round(odom_out.get("rate_hz", 0), 1)
                        sensors["slam"] = {
                            "status": (
                                "active"
                                if slam_rate > 0.0
                                else "inactive"
                            ),
                            "hz": slam_rate,
                            "messages": odom_out.get("msg_count", 0),
                        }
                    elif "Navigation" in name:
                        nav = h.get("navigation", h)
                        sensors["navigation"] = {
                            "state": nav.get(
                                "state",
                                h.get("mission_state", "idle"),
                            ),
                            "replan_count": nav.get(
                                "replan_count",
                                h.get("replan_count", 0),
                            ),
                        }
                except Exception:
                    module_summary[name] = "error"
                    modules_fail += 1

        slam_hz = float(sensors.get("slam", {}).get("hz") or 0.0)
        if slam_hz <= 0.0:
            slam_hz = gw._get_slam_hz_cached()

        brainstem_info = await _brainstem_health(gw)

        return {
            "status": "ok" if modules_fail == 0 else "degraded",
            "modules_ok": modules_ok,
            "modules_fail": modules_fail,
            "gateway": {
                "port": gw._port,
                "mode": gw._mode,
                "sse_clients": n_sse,
                "traffic": traffic,
                "commands": commands,
                "diagnostic_details": details,
            },
            "teleop": {
                "active": gw._teleop_active,
                "clients": gw._teleop_client_count(),
            },
            "sensors": sensors,
            "slam_hz": round(slam_hz, 1),
            "map_points": map_pts,
            "has_odom": gw._odom is not None,
            "modules": module_summary,
            "brainstem": brainstem_info,
        }

    @app.get(
        "/health",
        summary="Liveness probe",
        response_model=LivenessResponse,
    )
    async def liveness_health():
        return {"status": "ok", "ts": time.time()}

    @app.get(
        "/ready",
        summary="Readiness probe",
        response_model=ReadinessResponse,
        responses={503: {"model": ReadinessResponse}},
    )
    async def readiness_ready(
        details: Annotated[bool, Query(
            description="Include per-module health details; default probe payload is summary-only.",
        )] = False
    ):
        payload, status_code = build_readiness_snapshot(gw, include_details=details)
        return JSONResponse(payload, status_code=status_code)
