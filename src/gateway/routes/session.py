"""Session lifecycle routes for GatewayModule."""

from __future__ import annotations

import logging
import os
import time

from fastapi.responses import JSONResponse

from gateway.schemas import SessionResponse, SessionTransitionResponse
from gateway.services.map_safety import safe_map_name


logger = logging.getLogger(__name__)


def register_session_routes(app, gw) -> None:
    @app.get(
        "/api/v1/session",
        summary="Current session state + capabilities",
        response_model=SessionResponse,
    )
    async def session_get():
        inferred_mode, inferred_map = gw._session_detect_current_mode()
        if inferred_mode != gw._session_mode:
            gw._session_mode = inferred_mode
            gw._session_map = inferred_map
            gw._session_since = time.time()
        return gw._session_snapshot()

    @app.post(
        "/api/v1/session/start",
        summary="Enter mapping or navigating mode",
        response_model=SessionTransitionResponse,
        responses={
            400: {"model": SessionTransitionResponse},
            409: {"model": SessionTransitionResponse},
            500: {"model": SessionTransitionResponse},
            503: {"model": SessionTransitionResponse},
        },
    )
    async def session_start(body: dict):
        mode = (body.get("mode") or "").strip().lower()
        map_name = body.get("map_name") or body.get("map") or ""
        slam_profile = (
            body.get("slam_profile") or body.get("slam_backend") or ""
        ).strip().lower()
        if mode not in ("mapping", "navigating", "exploring"):
            return JSONResponse(
                {
                    "success": False,
                    "message": (
                        f"Unknown mode: {mode!r}. "
                        "Use 'mapping' | 'navigating' | 'exploring'."
                    ),
                },
                status_code=400,
            )
        if slam_profile and slam_profile not in ("fastlio2", "localizer", "super_lio"):
            return JSONResponse(
                {
                    "success": False,
                    "message": (
                        f"Unknown slam_profile: {slam_profile!r}. "
                        "Use 'fastlio2' | 'localizer' | 'super_lio'."
                    ),
                },
                status_code=400,
            )
        if map_name:
            err = safe_map_name(map_name)
            if err is not None:
                return JSONResponse(
                    {"success": False, "message": err},
                    status_code=400,
                )
        if mode == "exploring" and gw._frontier_explorer is None:
            return JSONResponse(
                {
                    "success": False,
                    "message": (
                        "FrontierExplorer module not running - start lingtu "
                        "with 'explore' profile."
                    ),
                },
                status_code=503,
            )
        if gw._session_mode != "idle":
            return JSONResponse(
                {
                    "success": False,
                    "message": (
                        f"Already in {gw._session_mode}. "
                        "Call /session/end first."
                    ),
                },
                status_code=409,
            )
        if gw._session_pending:
            return JSONResponse(
                {"success": False, "message": "Another transition in progress"},
                status_code=409,
            )

        if mode == "navigating":
            if not map_name:
                return JSONResponse(
                    {
                        "success": False,
                        "message": "map_name is required for navigating",
                    },
                    status_code=400,
                )

            base = os.path.expanduser(f"~/data/nova/maps/{map_name}")
            if not os.path.isfile(os.path.join(base, "map.pcd")):
                return JSONResponse(
                    {
                        "success": False,
                        "message": f"Map '{map_name}' has no map.pcd",
                    },
                    status_code=400,
                )
            if not os.path.isfile(os.path.join(base, "tomogram.pickle")):
                return JSONResponse(
                    {
                        "success": False,
                        "message": (
                            f"Map '{map_name}' has no tomogram - build it "
                            f"first (REPL: map build {map_name})"
                        ),
                    },
                    status_code=400,
                )
            active = os.path.expanduser("~/data/nova/maps/active")
            try:
                if os.path.islink(active) or os.path.exists(active):
                    os.remove(active)
                os.symlink(map_name, active)
            except OSError as e:
                return JSONResponse(
                    {
                        "success": False,
                        "message": f"Failed to activate map: {e}",
                    },
                    status_code=500,
                )

        gw._session_pending = True
        gw._session_error = ""
        try:
            from core.service_manager import get_service_manager

            svc = get_service_manager()
            backend = slam_profile or (
                "localizer" if mode == "navigating" else "fastlio2"
            )
            if backend == "super_lio":
                svc.stop("slam", "slam_pgo", "localizer")
                svc.ensure("lidar", "super_lio")
                ok = svc.wait_ready("lidar", "super_lio", timeout=10.0)
                if mode == "mapping" or mode == "exploring":
                    with gw._map_cloud_lock:
                        gw._map_points = None
                        gw._voxel_hits.clear()
            elif mode == "mapping" or mode == "exploring":
                svc.stop("localizer", "super_lio")
                svc.ensure("slam", "slam_pgo")
                ok = svc.wait_ready("slam", "slam_pgo", timeout=10.0)
                with gw._map_cloud_lock:
                    gw._map_points = None
                    gw._voxel_hits.clear()
            else:
                svc.stop("slam_pgo", "super_lio")
                svc.ensure("slam", "localizer")
                ok = svc.wait_ready("slam", "localizer", timeout=10.0)
            if not ok:
                gw._session_error = "Services not ready after 10s"
                return JSONResponse(
                    {"success": False, "message": gw._session_error},
                    status_code=500,
                )

            if mode == "navigating" and map_name and backend != "super_lio":
                gw._spawn_auto_relocalize(map_name)

            if mode == "exploring":
                try:
                    gw._frontier_explorer.begin_exploration()
                    gw._exploring = True
                except Exception as e:
                    gw._session_error = f"Explorer start failed: {e}"
                    return JSONResponse(
                        {"success": False, "message": gw._session_error},
                        status_code=500,
                    )

            gw._session_mode = mode
            gw._session_map = map_name if mode == "navigating" else None
            gw._session_slam_profile = backend
            gw._cached_slam_profile = backend
            gw._slam_profile_ts = time.time()
            gw._session_since = time.time()
            gw.push_event({"type": "session", "data": gw._session_snapshot()})
            return {"success": True, "session": gw._session_snapshot()}
        except Exception as e:
            gw._session_error = str(e)
            return JSONResponse(
                {"success": False, "message": str(e)},
                status_code=500,
            )
        finally:
            gw._session_pending = False

    @app.post(
        "/api/v1/session/end",
        summary="Exit current mode and return to idle",
        response_model=SessionTransitionResponse,
        responses={
            409: {"model": SessionTransitionResponse},
            500: {"model": SessionTransitionResponse},
        },
    )
    async def session_end():
        if gw._session_mode == "idle":
            return {"success": True, "session": gw._session_snapshot()}
        if gw._session_pending:
            return JSONResponse(
                {"success": False, "message": "Transition in progress"},
                status_code=409,
            )
        gw._session_pending = True
        try:
            if gw._exploring and gw._frontier_explorer is not None:
                try:
                    gw._frontier_explorer.end_exploration()
                except Exception as e:
                    logger.warning("session/end: end_exploration failed: %s", e)
                gw._exploring = False
                gw.push_event({"type": "exploring", "active": False})
            from core.service_manager import get_service_manager

            svc = get_service_manager()
            svc.stop("super_lio", "slam_pgo", "localizer", "slam")
            gw._session_mode = "idle"
            gw._session_map = None
            gw._session_slam_profile = "stopped"
            gw._cached_slam_profile = "stopped"
            gw._slam_profile_ts = time.time()
            gw._session_since = time.time()
            gw._session_error = ""
            gw.push_event({"type": "session", "data": gw._session_snapshot()})
            return {"success": True, "session": gw._session_snapshot()}
        except Exception as e:
            gw._session_error = str(e)
            return JSONResponse(
                {"success": False, "message": str(e)},
                status_code=500,
            )
        finally:
            gw._session_pending = False
