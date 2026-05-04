"""Session lifecycle routes for GatewayModule."""

from __future__ import annotations

import logging
import os
import time
from typing import Any

from fastapi.responses import JSONResponse

from gateway.schemas import (
    SessionResponse,
    SessionStartRequest,
    SessionTransitionResponse,
)
from gateway.services.map_paths import nav_map_root
from gateway.services.map_safety import safe_map_name


logger = logging.getLogger(__name__)

_SLAM_PROFILE_ALIASES = {
    "super-lio": "super_lio",
    "superlio": "super_lio",
    "super_lio_reloc": "super_lio_relocation",
    "super-lio-reloc": "super_lio_relocation",
    "superlio-reloc": "super_lio_relocation",
    "super_lio_relocation": "super_lio_relocation",
    "super-lio-relocation": "super_lio_relocation",
    "superlio-relocation": "super_lio_relocation",
    "relocation": "super_lio_relocation",
}


def _transition_payload(
    success: bool,
    *,
    session: dict[str, Any] | None = None,
    message: str | None = None,
) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "schema_version": 1,
        "ok": bool(success),
        "success": bool(success),
        "ts": time.time(),
    }
    if session is not None:
        payload["session"] = session
    if message is not None:
        payload["message"] = message
    return payload


def _transition_response(
    success: bool,
    *,
    status_code: int,
    session: dict[str, Any] | None = None,
    message: str | None = None,
) -> JSONResponse:
    return JSONResponse(
        _transition_payload(success, session=session, message=message),
        status_code=status_code,
    )


def _body_mapping(body: Any) -> dict[str, Any]:
    if hasattr(body, "model_dump"):
        return body.model_dump(exclude_none=True)
    if isinstance(body, dict):
        return body
    return {}


def _normalize_slam_profile(profile: str) -> str:
    raw = str(profile or "").strip().lower()
    return _SLAM_PROFILE_ALIASES.get(raw, raw)


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
    async def session_start(body: SessionStartRequest):
        payload = _body_mapping(body)
        mode = (payload.get("mode") or "").strip().lower()
        map_name = payload.get("map_name") or payload.get("map") or ""
        slam_profile = (
            payload.get("slam_profile") or payload.get("slam_backend") or ""
        ).strip().lower()
        slam_profile = _normalize_slam_profile(slam_profile)
        if mode not in ("mapping", "navigating", "exploring"):
            return _transition_response(
                False,
                status_code=400,
                message=(
                    f"Unknown mode: {mode!r}. "
                    "Use 'mapping' | 'navigating' | 'exploring'."
                ),
            )
        if slam_profile and slam_profile not in (
            "fastlio2",
            "localizer",
            "super_lio",
            "super_lio_relocation",
        ):
            return _transition_response(
                False,
                status_code=400,
                message=(
                    f"Unknown slam_profile: {slam_profile!r}. "
                    "Use 'fastlio2' | 'localizer' | 'super_lio' | "
                    "'super_lio_relocation'."
                ),
            )
        if map_name:
            err = safe_map_name(map_name)
            if err is not None:
                return _transition_response(False, status_code=400, message=err)
        if slam_profile == "super_lio_relocation" and mode != "navigating":
            return _transition_response(
                False,
                status_code=400,
                message="super_lio_relocation requires navigating with map_name",
            )
        if mode == "exploring" and not gw._explorer_available():
            return _transition_response(
                False,
                status_code=503,
                message=(
                    "Exploration backend not running - start lingtu with "
                    "'explore' or 'tare_explore' profile."
                ),
            )
        if gw._session_mode != "idle":
            return _transition_response(
                False,
                status_code=409,
                message=(
                    f"Already in {gw._session_mode}. "
                    "Call /session/end first."
                ),
            )
        if gw._session_pending:
            return _transition_response(
                False,
                status_code=409,
                message="Another transition in progress",
            )

        if mode == "navigating":
            if not map_name:
                return _transition_response(
                    False,
                    status_code=400,
                    message="map_name is required for navigating",
                )

            map_root = nav_map_root()
            base = (map_root / map_name).resolve()
            try:
                base.relative_to(map_root)
            except ValueError:
                return _transition_response(
                    False,
                    status_code=400,
                    message="map_name escapes NAV_MAP_DIR",
                )
            if not (base / "map.pcd").is_file():
                return _transition_response(
                    False,
                    status_code=400,
                    message=f"Map '{map_name}' has no map.pcd",
                )
            if not (base / "tomogram.pickle").is_file():
                return _transition_response(
                    False,
                    status_code=400,
                    message=(
                        f"Map '{map_name}' has no tomogram - build it "
                        f"first (REPL: map build {map_name})"
                    ),
                )
            active = map_root / "active"
            try:
                if active.is_symlink() or active.exists():
                    active.unlink()
                os.symlink(map_name, str(active))
            except OSError as e:
                return _transition_response(
                    False,
                    status_code=500,
                    message=f"Failed to activate map: {e}",
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
                svc.stop("slam", "slam_pgo", "localizer", "super_lio_relocation")
                svc.ensure("lidar", "super_lio")
                ok = svc.wait_ready("lidar", "super_lio", timeout=10.0)
                if mode == "mapping" or mode == "exploring":
                    with gw._map_cloud_lock:
                        gw._map_points = None
                        gw._voxel_hits.clear()
            elif backend == "super_lio_relocation":
                svc.stop("slam", "slam_pgo", "localizer", "super_lio")
                svc.ensure("lidar", "super_lio_relocation")
                ok = svc.wait_ready("lidar", "super_lio_relocation", timeout=10.0)
            elif mode == "mapping" or mode == "exploring":
                svc.stop("localizer", "super_lio", "super_lio_relocation")
                svc.ensure("slam", "slam_pgo")
                ok = svc.wait_ready("slam", "slam_pgo", timeout=10.0)
                with gw._map_cloud_lock:
                    gw._map_points = None
                    gw._voxel_hits.clear()
            else:
                svc.stop("slam_pgo", "super_lio", "super_lio_relocation")
                svc.ensure("slam", "localizer")
                ok = svc.wait_ready("slam", "localizer", timeout=10.0)
            if not ok:
                gw._session_error = "Services not ready after 10s"
                return _transition_response(
                    False,
                    status_code=500,
                    message=gw._session_error,
                )

            if (
                mode == "navigating"
                and map_name
                and backend not in {"super_lio", "super_lio_relocation"}
            ):
                gw._spawn_auto_relocalize(map_name)

            if mode == "exploring":
                try:
                    gw._begin_exploration()
                    gw._exploring = True
                except Exception as e:
                    gw._session_error = f"Explorer start failed: {e}"
                    return _transition_response(
                        False,
                        status_code=500,
                        message=gw._session_error,
                    )

            gw._session_mode = mode
            gw._session_map = map_name if mode == "navigating" else None
            gw._session_slam_profile = backend
            gw._cached_slam_profile = backend
            gw._slam_profile_ts = time.time()
            gw._session_since = time.time()
            gw.push_event({"type": "session", "data": gw._session_snapshot()})
            return _transition_payload(True, session=gw._session_snapshot())
        except Exception as e:
            gw._session_error = str(e)
            return _transition_response(
                False,
                status_code=500,
                message=str(e),
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
            return _transition_payload(True, session=gw._session_snapshot())
        if gw._session_pending:
            return _transition_response(
                False,
                status_code=409,
                message="Transition in progress",
            )
        gw._session_pending = True
        try:
            if gw._exploring and gw._explorer_available():
                try:
                    gw._end_exploration()
                except Exception as e:
                    logger.warning("session/end: end_exploration failed: %s", e)
                gw._exploring = False
                gw.push_event({"type": "exploring", "active": False})
            from core.service_manager import get_service_manager

            svc = get_service_manager()
            svc.stop(
                "super_lio_relocation",
                "super_lio",
                "slam_pgo",
                "localizer",
                "slam",
            )
            gw._session_mode = "idle"
            gw._session_map = None
            gw._session_slam_profile = "stopped"
            gw._cached_slam_profile = "stopped"
            gw._slam_profile_ts = time.time()
            gw._session_since = time.time()
            gw._session_error = ""
            gw.push_event({"type": "session", "data": gw._session_snapshot()})
            return _transition_payload(True, session=gw._session_snapshot())
        except Exception as e:
            gw._session_error = str(e)
            return _transition_response(
                False,
                status_code=500,
                message=str(e),
            )
        finally:
            gw._session_pending = False
