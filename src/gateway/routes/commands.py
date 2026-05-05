"""Control command routes for GatewayModule."""

from __future__ import annotations

import math
import time
from typing import Any

from fastapi.responses import JSONResponse

from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from gateway.schemas import (
    CancelRequest,
    ClickNavRequest,
    CmdVelRequest,
    ControlCommandResponse,
    GatewayErrorResponse,
    GoalRequest,
    InstructionRequest,
    LeaseRequest,
    LeaseResponse,
    ModeRequest,
    PlanPreviewRequest,
    PlanPreviewResponse,
    StopRequest,
)
from gateway.services.safety_status import safety_stop_active, safety_summary


CONTROL_COMMAND_ERROR_RESPONSES = {
    409: {"model": GatewayErrorResponse},
}

LEASE_ERROR_RESPONSES = {
    403: {"model": GatewayErrorResponse},
    409: {"model": GatewayErrorResponse},
}


def _command_rejected_response(
    command: str,
    body: Any,
    *,
    error: str,
    message: str,
    detail: dict[str, Any],
    status_code: int = 409,
    gw: Any | None = None,
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
    if gw is not None and hasattr(gw, "_publish_command_ack"):
        gw._publish_command_ack(content, status_code=status_code)
    return JSONResponse(
        status_code=status_code,
        content=content,
    )


def _goal_readiness_rejection(gw, command: str, body: Any) -> JSONResponse | None:
    from gateway.services.runtime_status import build_navigation_status

    status = build_navigation_status(gw)
    readiness = status.get("readiness", {})
    blockers = list(readiness.get("blockers") or [])
    if bool(status.get("can_accept_goal", False)) and not blockers:
        return None
    return _command_rejected_response(
        command,
        body,
        error="navigation_not_ready",
        message="Navigation cannot accept a goal in the current state.",
        detail={
            "state": status.get("state"),
            "has_odometry": status.get("has_odometry"),
            "session_mode": getattr(gw, "_session_mode", None),
            "blockers": blockers,
            "advisories": list(readiness.get("advisories") or []),
            "localization": status.get("localization", {}),
            "path": "/api/v1/navigation/status",
        },
        gw=gw,
    )


def _motion_safety_rejection(gw, command: str, body: Any) -> JSONResponse | None:
    try:
        with gw._state_lock:
            safety = getattr(gw, "_safety", None)
    except Exception:
        safety = None

    if not safety_stop_active(safety):
        return None
    return _command_rejected_response(
        command,
        body,
        error="safety_stop",
        message="Safety STOP is active; motion commands are not accepted.",
        detail={
            "safety": safety_summary(safety),
            "path": "/api/v1/state",
        },
        gw=gw,
    )


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


def _current_start_payload(gw, *, ts: float) -> dict[str, Any] | None:
    with gw._state_lock:
        odom = dict(gw._odom) if gw._odom else None
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
    return _point_payload(
        x,
        y,
        z,
        ts=ts,
    )


def _plan_preview_unavailable(
    gw,
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
        "start": _current_start_payload(gw, ts=ts),
        "goal": _point_payload(body.x, body.y, body.z, ts=ts, frame_id=body.frame_id),
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


def _preview_navigation_plan(gw, body: PlanPreviewRequest) -> dict[str, Any]:
    from gateway.services.runtime_status import build_navigation_status

    status = build_navigation_status(gw)
    readiness = status.get("readiness", {})
    blockers = list(readiness.get("blockers") or [])
    if blockers or not bool(status.get("has_odometry", False)):
        reasons = ["navigation_not_ready", *blockers]
        if not bool(status.get("has_odometry", False)):
            reasons.append("odometry_missing")
        return _plan_preview_unavailable(
            gw,
            body,
            reasons=reasons,
            source="gateway_readiness",
        )

    nav = (getattr(gw, "_all_modules", {}) or {}).get("NavigationModule")
    if nav is None or not hasattr(nav, "preview_plan"):
        return _plan_preview_unavailable(
            gw,
            body,
            reasons=["navigation_module_unavailable"],
            source="gateway_modules",
        )

    try:
        return nav.preview_plan(body.x, body.y, body.z)
    except Exception as exc:
        return _plan_preview_unavailable(
            gw,
            body,
            reasons=["planning_preview_failed"],
            source="gateway_exception",
            error=str(exc),
        )


def _goal_plan_preview_rejection(gw, command: str, body: Any) -> JSONResponse | None:
    try:
        preview_body = PlanPreviewRequest(
            x=body.x,
            y=body.y,
            z=body.z,
            frame_id=getattr(body, "frame_id", "map"),
            client_id=getattr(body, "client_id", "unknown"),
        )
    except Exception as exc:
        return _command_rejected_response(
            command,
            body,
            error="navigation_plan_invalid",
            message="Navigation goal cannot be previewed.",
            detail={"error": str(exc)},
            gw=gw,
        )

    preview = _preview_navigation_plan(gw, preview_body)
    if bool(preview.get("feasible", False)):
        return None
    return _command_rejected_response(
        command,
        body,
        error="navigation_plan_infeasible",
        message="Navigation plan preview is not feasible.",
        detail={
            "preview": preview,
            "path": "/api/v1/navigation/plan",
        },
        gw=gw,
    )


def _run_planned_goal_command(
    gw,
    command: str,
    body: Any,
    action,
) -> dict[str, Any] | JSONResponse:
    request_id = getattr(body, "request_id", None) if body is not None else None
    client_id = getattr(body, "client_id", None) if body is not None else None
    rejection = _motion_safety_rejection(gw, command, body)
    if rejection is not None:
        return rejection
    replay = gw._command_journal.replay(command, request_id)
    if replay is not None:
        if hasattr(gw, "_publish_command_ack"):
            gw._publish_command_ack(replay, status_code=200)
        return replay

    rejection = _goal_readiness_rejection(gw, command, body)
    if rejection is not None:
        return rejection
    rejection = _goal_plan_preview_rejection(gw, command, body)
    if rejection is not None:
        return rejection

    response = gw._command_journal.accept(command, request_id, client_id, action())
    if hasattr(gw, "_publish_command_ack"):
        gw._publish_command_ack(response, status_code=200)
    return response


def register_command_routes(app, gw) -> None:
    @app.post(
        "/api/v1/navigation/plan",
        summary="Preview navigation plan without publishing a goal",
        response_model=PlanPreviewResponse,
    )
    async def post_navigation_plan(body: PlanPreviewRequest):
        return _preview_navigation_plan(gw, body)

    @app.post(
        "/api/v1/goal",
        summary="Send navigation goal",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_goal(body: GoalRequest):
        def _publish() -> dict[str, Any]:
            gw.goal_pose.publish(
                PoseStamped(
                    pose=Pose(
                        position=Vector3(body.x, body.y, body.z),
                        orientation=Quaternion.from_yaw(body.yaw),
                    ),
                    frame_id=body.frame_id,
                    ts=time.time(),
                )
            )
            if body.instruction:
                gw.instruction.publish(body.instruction)
            return {
                "status": "ok",
                "goal": [body.x, body.y, body.z],
                "yaw": body.yaw,
                "frame_id": body.frame_id,
            }

        return _run_planned_goal_command(gw, "goal", body, _publish)

    @app.post(
        "/api/v1/navigate/click",
        summary="Navigate to map-viewer click point",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigate_click(body: ClickNavRequest):
        def _publish() -> dict[str, Any]:
            gw.goal_pose.publish(
                PoseStamped(
                    pose=Pose(
                        position=Vector3(body.x, body.y, body.z),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id=body.frame_id,
                    ts=time.time(),
                )
            )
            return {
                "status": "ok",
                "goal": [body.x, body.y, body.z],
                "frame_id": body.frame_id,
            }

        return _run_planned_goal_command(gw, "navigate_click", body, _publish)

    @app.post(
        "/api/v1/cmd_vel",
        summary="Direct velocity command",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_cmd_vel(body: CmdVelRequest):
        rejection = _motion_safety_rejection(gw, "cmd_vel", body)
        if rejection is not None:
            return rejection

        def _publish() -> dict[str, Any]:
            gw.cmd_vel.publish(
                Twist(
                    linear=Vector3(body.vx, body.vy, 0),
                    angular=Vector3(0, 0, body.wz),
                )
            )
            return {"status": "ok"}

        return gw._run_control_command("cmd_vel", body, _publish)

    @app.post(
        "/api/v1/stop",
        summary="Emergency stop",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_stop(body: StopRequest | None = None):
        def _publish() -> dict[str, Any]:
            gw.stop_cmd.publish(2)
            gw.cmd_vel.publish(Twist())
            return {"status": "stopped"}

        return gw._run_control_command("stop", body, _publish)

    @app.post(
        "/api/v1/navigation/cancel",
        summary="Gracefully cancel current navigation mission",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_navigation_cancel(body: CancelRequest):
        def _publish() -> dict[str, Any]:
            gw.cancel.publish(body.reason)
            return {"status": "cancelled", "reason": body.reason}

        return gw._run_control_command("navigation_cancel", body, _publish)

    @app.post(
        "/api/v1/instruction",
        summary="Natural language navigation instruction",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_instruction(body: InstructionRequest):
        rejection = _motion_safety_rejection(gw, "instruction", body)
        if rejection is not None:
            return rejection

        def _publish() -> dict[str, Any]:
            gw.instruction.publish(body.text)
            return {"status": "ok", "instruction": body.text}

        return gw._run_control_command("instruction", body, _publish)

    @app.post(
        "/api/v1/mode",
        summary="Switch operating mode",
        response_model=ControlCommandResponse,
        responses=CONTROL_COMMAND_ERROR_RESPONSES,
    )
    async def post_mode(body: ModeRequest):
        def _publish() -> dict[str, Any]:
            with gw._state_lock:
                gw._mode = body.mode
            gw.mode_cmd.publish(body.mode)
            if body.mode == "estop":
                gw.stop_cmd.publish(2)
                gw.cmd_vel.publish(Twist())
            return {"status": "ok", "mode": body.mode}

        return gw._run_control_command("mode", body, _publish)

    @app.post(
        "/api/v1/lease",
        summary="Acquire/release/renew control lease",
        response_model=LeaseResponse,
        responses=LEASE_ERROR_RESPONSES,
    )
    async def post_lease(body: LeaseRequest):
        def _publish_lease_event(payload: dict[str, Any]) -> None:
            if hasattr(gw, "push_event"):
                gw.push_event({"type": "lease", "data": payload})

        def _apply() -> dict[str, Any]:
            if body.action == "acquire":
                ok = gw._lease.acquire(body.client_id, body.ttl)
                if not ok:
                    raise PermissionError("lease_conflict")
                result = {"status": "acquired", **gw._lease.to_dict()}
                _publish_lease_event(result)
                return result

            if body.action == "release":
                gw._lease.release(body.client_id)
                result = {"status": "released", **gw._lease.to_dict()}
                _publish_lease_event(result)
                return result

            ok = gw._lease.renew(body.client_id, body.ttl)
            if not ok:
                raise PermissionError("not_lease_holder")
            result = {"status": "renewed", **gw._lease.to_dict()}
            _publish_lease_event(result)
            return result

        try:
            return gw._run_control_command("lease", body, _apply)
        except PermissionError as exc:
            error = str(exc)
            status_code = 409 if error == "lease_conflict" else 403
            message = (
                "control lease is held by another client"
                if error == "lease_conflict"
                else "client does not hold the active control lease"
            )
            content = {
                "schema_version": 1,
                "ok": False,
                "error": error,
                "message": message,
                "command": {
                    "name": "lease",
                    "request_id": body.request_id,
                    "client_id": body.client_id,
                    "accepted": False,
                    "replay": False,
                    "ts": time.time(),
                },
                "detail": gw._lease.to_dict(),
            }
            if hasattr(gw, "_publish_command_ack"):
                gw._publish_command_ack(content, status_code=status_code)
            _publish_lease_event({
                "status": "rejected",
                "error": error,
                **gw._lease.to_dict(),
            })
            return JSONResponse(
                status_code=status_code,
                content=content,
            )
