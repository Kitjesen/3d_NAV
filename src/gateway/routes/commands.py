"""Control command routes for GatewayModule."""

from __future__ import annotations

import time
from typing import Any

from fastapi.responses import JSONResponse

from core.msgs.geometry import Pose, PoseStamped, Quaternion, Twist, Vector3
from gateway.schemas import (
    ClickNavRequest,
    CmdVelRequest,
    ControlCommandResponse,
    GatewayErrorResponse,
    GoalRequest,
    InstructionRequest,
    LeaseRequest,
    LeaseResponse,
    ModeRequest,
    StopRequest,
)


def _command_rejected_response(
    command: str,
    body: Any,
    *,
    error: str,
    message: str,
    detail: dict[str, Any],
    status_code: int = 409,
) -> JSONResponse:
    return JSONResponse(
        status_code=status_code,
        content={
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
        },
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
    )


def register_command_routes(app, gw) -> None:
    @app.post(
        "/api/v1/goal",
        summary="Send navigation goal",
        response_model=ControlCommandResponse,
    )
    async def post_goal(body: GoalRequest):
        rejection = _goal_readiness_rejection(gw, "goal", body)
        if rejection is not None:
            return rejection

        def _publish() -> dict[str, Any]:
            gw.goal_pose.publish(
                PoseStamped(
                    pose=Pose(
                        position=Vector3(body.x, body.y, body.z),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id="map",
                    ts=time.time(),
                )
            )
            if body.instruction:
                gw.instruction.publish(body.instruction)
            return {"status": "ok", "goal": [body.x, body.y, body.z]}

        return gw._run_control_command("goal", body, _publish)

    @app.post(
        "/api/v1/navigate/click",
        summary="Navigate to map-viewer click point",
        response_model=ControlCommandResponse,
    )
    async def post_navigate_click(body: ClickNavRequest):
        rejection = _goal_readiness_rejection(gw, "navigate_click", body)
        if rejection is not None:
            return rejection

        def _publish() -> dict[str, Any]:
            gw.goal_pose.publish(
                PoseStamped(
                    pose=Pose(
                        position=Vector3(body.x, body.y, body.z),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id="map",
                    ts=time.time(),
                )
            )
            return {"status": "ok", "goal": [body.x, body.y, body.z]}

        return gw._run_control_command("navigate_click", body, _publish)

    @app.post(
        "/api/v1/cmd_vel",
        summary="Direct velocity command",
        response_model=ControlCommandResponse,
    )
    async def post_cmd_vel(body: CmdVelRequest):
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
    )
    async def post_stop(body: StopRequest | None = None):
        def _publish() -> dict[str, Any]:
            gw.stop_cmd.publish(2)
            gw.cmd_vel.publish(Twist())
            return {"status": "stopped"}

        return gw._run_control_command("stop", body, _publish)

    @app.post(
        "/api/v1/instruction",
        summary="Natural language navigation instruction",
        response_model=ControlCommandResponse,
    )
    async def post_instruction(body: InstructionRequest):
        def _publish() -> dict[str, Any]:
            gw.instruction.publish(body.text)
            return {"status": "ok", "instruction": body.text}

        return gw._run_control_command("instruction", body, _publish)

    @app.post(
        "/api/v1/mode",
        summary="Switch operating mode",
        response_model=ControlCommandResponse,
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
        responses={
            403: {"model": GatewayErrorResponse},
            409: {"model": GatewayErrorResponse},
        },
    )
    async def post_lease(body: LeaseRequest):
        def _apply() -> dict[str, Any]:
            if body.action == "acquire":
                ok = gw._lease.acquire(body.client_id, body.ttl)
                if not ok:
                    raise PermissionError("lease_conflict")
                return {"status": "acquired", **gw._lease.to_dict()}

            if body.action == "release":
                gw._lease.release(body.client_id)
                return {"status": "released", **gw._lease.to_dict()}

            ok = gw._lease.renew(body.client_id, body.ttl)
            if not ok:
                raise PermissionError("not_lease_holder")
            return {"status": "renewed", **gw._lease.to_dict()}

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
            return JSONResponse(
                status_code=status_code,
                content={
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
                },
            )
