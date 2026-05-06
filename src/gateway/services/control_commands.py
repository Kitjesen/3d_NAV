"""Control command gating and receipts for Gateway routes."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

from fastapi.responses import JSONResponse

from gateway.schemas import PlanPreviewRequest
from gateway.services.safety_status import safety_stop_active, safety_summary


class ControlCommandService:
    """Keep motion-command safety gates out of HTTP route handlers."""

    def __init__(self, gateway: Any) -> None:
        self._gw = gateway

    def rejected_response(
        self,
        command: str,
        body: Any,
        *,
        error: str,
        message: str,
        detail: dict[str, Any],
        status_code: int = 409,
    ) -> JSONResponse:
        content = {
            "schema_version": 1,
            "ok": False,
            "error": error,
            "message": message,
            "command": {
                "name": command,
                "request_id": getattr(body, "request_id", None),
                "client_id": getattr(body, "client_id", "unknown"),
                "accepted": False,
                "replay": False,
                "ts": time.time(),
            },
            "detail": detail,
        }
        if hasattr(self._gw, "_publish_command_ack"):
            self._gw._publish_command_ack(content, status_code=status_code)
        return JSONResponse(status_code=status_code, content=content)

    def motion_safety_rejection(
        self,
        command: str,
        body: Any,
    ) -> JSONResponse | None:
        try:
            with self._gw._state_lock:
                safety = getattr(self._gw, "_safety", None)
        except Exception:
            safety = None

        if not safety_stop_active(safety):
            return None
        return self.rejected_response(
            command,
            body,
            error="safety_stop",
            message="Safety STOP is active; motion commands are not accepted.",
            detail={
                "safety": safety_summary(safety),
                "path": "/api/v1/state",
            },
        )

    def preview_navigation_plan(self, body: PlanPreviewRequest) -> dict[str, Any]:
        from gateway.services.runtime_status import build_navigation_status

        status = build_navigation_status(self._gw)
        readiness = status.get("readiness", {})
        blockers = list(readiness.get("blockers") or [])
        if blockers or not bool(status.get("has_odometry", False)):
            reasons = ["navigation_not_ready", *blockers]
            if not bool(status.get("has_odometry", False)):
                reasons.append("odometry_missing")
            return self._plan_preview_unavailable(
                body,
                reasons=reasons,
                source="gateway_readiness",
            )

        nav = (getattr(self._gw, "_all_modules", {}) or {}).get("NavigationModule")
        if nav is None or not hasattr(nav, "preview_plan"):
            return self._plan_preview_unavailable(
                body,
                reasons=["navigation_module_unavailable"],
                source="gateway_modules",
            )

        try:
            return nav.preview_plan(body.x, body.y, body.z)
        except Exception as exc:
            return self._plan_preview_unavailable(
                body,
                reasons=["planning_preview_failed"],
                source="gateway_exception",
                error=str(exc),
            )

    def run_planned_goal_command(
        self,
        command: str,
        body: Any,
        action: Callable[[], dict[str, Any]],
    ) -> dict[str, Any] | JSONResponse:
        request_id = getattr(body, "request_id", None) if body is not None else None
        client_id = getattr(body, "client_id", None) if body is not None else None
        rejection = self.motion_safety_rejection(command, body)
        if rejection is not None:
            return rejection
        replay = self._gw._command_journal.replay(command, request_id)
        if replay is not None:
            if hasattr(self._gw, "_publish_command_ack"):
                self._gw._publish_command_ack(replay, status_code=200)
            return replay

        rejection = self._goal_readiness_rejection(command, body)
        if rejection is not None:
            return rejection
        rejection = self._goal_plan_preview_rejection(command, body)
        if rejection is not None:
            return rejection

        response = self._gw._command_journal.accept(
            command,
            request_id,
            client_id,
            action(),
        )
        if hasattr(self._gw, "_publish_command_ack"):
            self._gw._publish_command_ack(response, status_code=200)
        return response

    def run_motion_guarded_command(
        self,
        command: str,
        body: Any,
        action: Callable[[], dict[str, Any]],
    ) -> dict[str, Any] | JSONResponse:
        rejection = self.motion_safety_rejection(command, body)
        if rejection is not None:
            return rejection
        return self._gw._run_control_command(command, body, action)

    def _goal_readiness_rejection(
        self,
        command: str,
        body: Any,
    ) -> JSONResponse | None:
        from gateway.services.runtime_status import build_navigation_status

        status = build_navigation_status(self._gw)
        readiness = status.get("readiness", {})
        blockers = list(readiness.get("blockers") or [])
        if bool(status.get("can_accept_goal", False)) and not blockers:
            return None
        return self.rejected_response(
            command,
            body,
            error="navigation_not_ready",
            message="Navigation cannot accept a goal in the current state.",
            detail={
                "state": status.get("state"),
                "has_odometry": status.get("has_odometry"),
                "session_mode": getattr(self._gw, "_session_mode", None),
                "blockers": blockers,
                "advisories": list(readiness.get("advisories") or []),
                "localization": status.get("localization", {}),
                "path": "/api/v1/navigation/status",
            },
        )

    def _goal_plan_preview_rejection(
        self,
        command: str,
        body: Any,
    ) -> JSONResponse | None:
        try:
            preview_body = PlanPreviewRequest(
                x=body.x,
                y=body.y,
                z=body.z,
                frame_id=getattr(body, "frame_id", "map"),
                client_id=getattr(body, "client_id", "unknown"),
            )
        except Exception as exc:
            return self.rejected_response(
                command,
                body,
                error="navigation_plan_invalid",
                message="Navigation goal cannot be previewed.",
                detail={"error": str(exc)},
            )

        preview = self.preview_navigation_plan(preview_body)
        if bool(preview.get("feasible", False)):
            return None
        return self.rejected_response(
            command,
            body,
            error="navigation_plan_infeasible",
            message="Navigation plan preview is not feasible.",
            detail={
                "preview": preview,
                "path": "/api/v1/navigation/plan",
            },
        )

    def _plan_preview_unavailable(
        self,
        body: PlanPreviewRequest,
        *,
        reasons: list[str],
        source: str,
        error: str | None = None,
    ) -> dict[str, Any]:
        ts = time.time()
        return {
            "schema_version": 1,
            "ok": error is None,
            "feasible": False,
            "frame_id": body.frame_id,
            "start": self._current_start_payload(ts=ts),
            "goal": _point_payload(
                body.x,
                body.y,
                body.z,
                ts=ts,
                frame_id=body.frame_id,
            ),
            "adjusted_goal": None,
            "path": [],
            "count": 0,
            "distance_m": None,
            "plan_ms": None,
            "planner": None,
            "source": source,
            "reasons": list(dict.fromkeys(reasons)),
            "error": error,
            "ts": ts,
        }

    def _current_start_payload(self, *, ts: float) -> dict[str, Any] | None:
        with self._gw._state_lock:
            odom = dict(self._gw._odom) if self._gw._odom else None
        if not odom:
            return None
        try:
            x = float(odom.get("x", 0.0))
            y = float(odom.get("y", 0.0))
            z = float(odom.get("z", 0.0))
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (x, y, z)):
            return None
        frame_id = str(odom.get("frame_id") or odom.get("frame") or "").strip()
        if not frame_id:
            header = odom.get("header")
            if isinstance(header, dict):
                frame_id = str(
                    header.get("frame_id") or header.get("frame") or ""
                ).strip()
        return _point_payload(x, y, z, ts=ts, frame_id=frame_id or "map")


def _point_payload(
    x: float,
    y: float,
    z: float,
    *,
    ts: float,
    frame_id: str = "map",
) -> dict[str, Any]:
    return {
        "x": float(x),
        "y": float(y),
        "z": float(z),
        "frame_id": frame_id,
        "ts": ts,
        "metadata": {},
    }
