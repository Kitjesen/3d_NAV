from __future__ import annotations

import asyncio
import json
import math
import time

import pytest
from pydantic import ValidationError


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


def _mark_navigation_ready(gateway) -> None:
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0, "ts": time.time()}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "odom_age_ms": 100.0,
            "localizer_health": "RECOVERED",
        }


class _FakePlanPreviewNav:
    def __init__(
        self,
        *,
        feasible: bool = True,
        reasons: list[str] | None = None,
    ) -> None:
        self.calls: list[tuple[float, float, float]] = []
        self.feasible = feasible
        self.reasons = list(reasons or [])

    def preview_plan(self, x: float, y: float, z: float) -> dict:
        self.calls.append((x, y, z))
        ts = time.time()
        if not self.feasible:
            return {
                "schema_version": 1,
                "ok": True,
                "feasible": False,
                "frame_id": "map",
                "start": {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
                "goal": {"x": x, "y": y, "z": z, "frame_id": "map", "ts": ts},
                "adjusted_goal": None,
                "path": [],
                "count": 0,
                "distance_m": None,
                "plan_ms": 0.5,
                "planner": "fake",
                "source": "navigation_preview",
                "reasons": self.reasons or ["blocked_by_costmap"],
                "error": None,
                "ts": ts,
            }
        return {
            "schema_version": 1,
            "ok": True,
            "feasible": True,
            "frame_id": "map",
            "start": {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
            "goal": {"x": x, "y": y, "z": z, "frame_id": "map", "ts": ts},
            "adjusted_goal": None,
            "path": [
                {"x": 0.0, "y": 0.0, "z": 0.0, "frame_id": "map", "ts": ts},
                {"x": x, "y": y, "z": z, "frame_id": "map", "ts": ts},
            ],
            "count": 2,
            "distance_m": 1.0,
            "plan_ms": 0.5,
            "planner": "fake",
            "source": "navigation_preview",
            "reasons": [],
            "error": None,
            "ts": ts,
        }


def test_navigation_plan_preview_is_non_motion_and_typed():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    result = asyncio.run(
        post_plan(
            PlanPreviewRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                client_id="web",
            )
        )
    )
    model = PlanPreviewResponse.model_validate(result)

    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert model.schema_version == 1
    assert model.ok is True
    assert model.feasible is True
    assert model.path[-1].x == 1.0
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0


def test_navigation_plan_preview_does_not_publish_any_control_outputs():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    asyncio.run(
        post_plan(
            PlanPreviewRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                client_id="web",
            )
        )
    )

    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.instruction.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0
    assert gateway.cancel.msg_count == 0
    assert gateway.mode_cmd.msg_count == 0


def test_navigation_goal_requests_are_map_frame_only():
    from gateway.schemas import ClickNavRequest, GoalRequest, PlanPreviewRequest

    assert GoalRequest(x=1.0, y=2.0).frame_id == "map"
    assert ClickNavRequest(x=1.0, y=2.0).frame_id == "map"
    assert PlanPreviewRequest(x=1.0, y=2.0).frame_id == "map"
    with pytest.raises(ValidationError):
        GoalRequest(x=1.0, y=2.0, frame_id="odom")
    with pytest.raises(ValidationError):
        ClickNavRequest(x=1.0, y=2.0, frame_id="odom")
    with pytest.raises(ValidationError):
        PlanPreviewRequest(x=1.0, y=2.0, frame_id="odom")


def test_navigation_plan_preview_degrades_without_odometry_and_does_not_plan():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    result = asyncio.run(post_plan(PlanPreviewRequest(x=1.0, y=2.0)))
    model = PlanPreviewResponse.model_validate(result)

    assert nav.calls == []
    assert model.ok is True
    assert model.feasible is False
    assert "odometry_missing" in model.reasons
    assert model.goal.x == 1.0
    assert model.path == []
    assert gateway.goal_pose.msg_count == 0


def test_navigation_plan_preview_omits_invalid_start_when_unavailable():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    gateway._mode = "estop"
    with gateway._state_lock:
        gateway._odom = {"x": "bad", "y": 0.0, "z": 0.0, "ts": time.time()}
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    result = asyncio.run(post_plan(PlanPreviewRequest(x=1.0, y=2.0)))
    model = PlanPreviewResponse.model_validate(result)

    assert nav.calls == []
    assert model.feasible is False
    assert model.start is None
    assert "estop_active" in model.reasons
    assert gateway.goal_pose.msg_count == 0


def test_navigation_plan_preview_preserves_non_map_start_frame_when_blocked():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import PlanPreviewRequest, PlanPreviewResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    gateway._mode = "estop"
    with gateway._state_lock:
        gateway._odom = {
            "x": 1.0,
            "y": 2.0,
            "z": 0.0,
            "frame_id": "odom",
            "ts": time.time(),
        }
    post_plan = _endpoint(gateway, "/api/v1/navigation/plan")

    result = asyncio.run(post_plan(PlanPreviewRequest(x=3.0, y=4.0)))
    model = PlanPreviewResponse.model_validate(result)

    assert nav.calls == []
    assert model.feasible is False
    assert model.frame_id == "map"
    assert model.start is not None
    assert model.start.frame_id == "odom"
    assert "estop_active" in model.reasons
    assert gateway.goal_pose.msg_count == 0


def test_command_journal_replays_duplicate_request_id_without_republish():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    body = GoalRequest(
        x=1.0,
        y=2.0,
        z=0.0,
        instruction="dock",
        request_id="goal-001",
        client_id="web",
    )

    first = asyncio.run(post_goal(body))
    second = asyncio.run(post_goal(body))
    model = ControlCommandResponse.model_validate(first)

    assert gateway.goal_pose.msg_count == 1
    assert gateway.instruction.msg_count == 1
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert model.schema_version == 1
    assert model.ok is True
    assert model.status == "ok"
    assert model.goal == [1.0, 2.0, 0.0]
    assert model.command.name == "goal"
    assert model.command.request_id == "goal-001"
    assert model.command.client_id == "web"
    assert first["command"]["accepted"] is True
    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is True
    assert second["goal"] == [1.0, 2.0, 0.0]
    assert second["command"]["request_id"] == "goal-001"


def test_control_commands_publish_command_ack_events():
    from gateway.gateway_module import GatewayModule, GoalRequest

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")
    queue = gateway._sse_subscribe()

    try:
        body = GoalRequest(
            x=1.0,
            y=2.0,
            z=0.0,
            request_id="goal-ack-001",
            client_id="web",
        )
        first = asyncio.run(post_goal(body))
        second = asyncio.run(post_goal(body))

        first_event = queue.get_nowait()
        second_event = queue.get_nowait()
    finally:
        gateway._sse_unsubscribe(queue)

    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is True
    assert first_event["type"] == "command_ack"
    assert first_event["data"]["ok"] is True
    assert first_event["data"]["status"] == "ok"
    assert first_event["data"]["status_code"] == 200
    assert first_event["data"]["command"]["name"] == "goal"
    assert first_event["data"]["command"]["request_id"] == "goal-ack-001"
    assert first_event["data"]["command"]["replay"] is False
    assert second_event["type"] == "command_ack"
    assert second_event["data"]["command"]["replay"] is True


def test_goal_request_yaw_is_published_as_pose_orientation():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    result = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                yaw=math.pi / 2,
                client_id="script",
            )
        )
    )
    model = ControlCommandResponse.model_validate(result)

    assert gateway.goal_pose.msg_count == 1
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert len(sent_goals) == 1
    assert sent_goals[0].pose.orientation.yaw == pytest.approx(math.pi / 2)
    assert model.goal == [1.0, 2.0, 0.0]
    assert model.yaw == pytest.approx(math.pi / 2)
    assert model.frame_id == "map"
    assert result["yaw"] == pytest.approx(math.pi / 2)
    assert result["frame_id"] == "map"


def test_goal_route_rejects_infeasible_plan_preview_without_publishing():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav(feasible=False, reasons=["blocked_by_costmap"])
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_goal = _endpoint(gateway, "/api/v1/goal")
    queue = gateway._sse_subscribe()

    try:
        response = asyncio.run(
            post_goal(
                GoalRequest(
                    x=1.0,
                    y=2.0,
                    z=0.0,
                    request_id="blocked-goal",
                    client_id="web",
                )
            )
        )
        event = queue.get_nowait()
    finally:
        gateway._sse_unsubscribe(queue)
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.ok is False
    assert model.error == "navigation_plan_infeasible"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.request_id == "blocked-goal"
    assert model.command.accepted is False
    assert model.detail["path"] == "/api/v1/navigation/plan"
    assert model.detail["preview"]["reasons"] == ["blocked_by_costmap"]
    assert event["type"] == "command_ack"
    assert event["data"]["ok"] is False
    assert event["data"]["error"] == "navigation_plan_infeasible"
    assert event["data"]["command"]["accepted"] is False
    assert event["data"]["command"]["request_id"] == "blocked-goal"
    assert event["data"]["detail"]["path"] == "/api/v1/navigation/plan"
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert sent_goals == []
    assert gateway.goal_pose.msg_count == 0


def test_goal_route_rejects_infeasible_preview_with_instruction_without_any_publish():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav(feasible=False, reasons=["blocked_by_costmap"])
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                instruction="dock",
                request_id="blocked-goal-with-instruction",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_plan_infeasible"
    assert model.detail["preview"]["reasons"] == ["blocked_by_costmap"]
    assert nav.calls == [(1.0, 2.0, 0.0)]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_goal_route_rejects_safety_stop_without_planning_or_publishing():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                request_id="safety-stop-goal",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "safety_stop"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert model.detail["safety"]["stop_active"] is True
    assert nav.calls == []
    assert gateway.goal_pose.msg_count == 0


def test_goal_route_rejects_inactive_navigation_session_without_planning_or_publishing():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    gateway._session_mode = "idle"
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                request_id="inactive-session-goal",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_not_ready"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert "navigation_session_inactive" in model.detail["blockers"]
    assert nav.calls == []
    assert gateway.goal_pose.msg_count == 0


def test_click_navigation_rejects_infeasible_plan_preview_without_publishing():
    from gateway.gateway_module import ClickNavRequest, GatewayModule
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav(feasible=False, reasons=["blocked_by_costmap"])
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_click = _endpoint(gateway, "/api/v1/navigate/click")

    response = asyncio.run(
        post_click(
            ClickNavRequest(
                x=3.0,
                y=4.0,
                z=0.0,
                request_id="blocked-click",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_plan_infeasible"
    assert model.command is not None
    assert model.command.name == "navigate_click"
    assert model.command.request_id == "blocked-click"
    assert model.command.accepted is False
    assert model.detail["preview"]["reasons"] == ["blocked_by_costmap"]
    assert nav.calls == [(3.0, 4.0, 0.0)]
    assert sent_goals == []
    assert gateway.goal_pose.msg_count == 0


def test_click_navigation_rejects_safety_stop_without_planning_or_publishing():
    from gateway.gateway_module import ClickNavRequest, GatewayModule
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_click = _endpoint(gateway, "/api/v1/navigate/click")

    response = asyncio.run(
        post_click(
            ClickNavRequest(
                x=3.0,
                y=4.0,
                z=0.0,
                request_id="safety-stop-click",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "safety_stop"
    assert model.command is not None
    assert model.command.name == "navigate_click"
    assert nav.calls == []
    assert gateway.goal_pose.msg_count == 0


def test_click_navigation_previews_publishes_and_replays_request_id_once():
    from gateway.gateway_module import ClickNavRequest, GatewayModule
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    nav = _FakePlanPreviewNav()
    gateway.on_system_modules({"NavigationModule": nav})
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_click = _endpoint(gateway, "/api/v1/navigate/click")
    body = ClickNavRequest(
        x=3.0,
        y=4.0,
        z=0.0,
        request_id="click-001",
        client_id="web",
    )

    first = asyncio.run(post_click(body))
    second = asyncio.run(post_click(body))
    model = ControlCommandResponse.model_validate(first)

    assert model.ok is True
    assert model.command.name == "navigate_click"
    assert model.command.accepted is True
    assert model.command.replay is False
    assert model.goal == [3.0, 4.0, 0.0]
    assert first["command"]["replay"] is False
    assert second["command"]["replay"] is True
    assert second["goal"] == [3.0, 4.0, 0.0]
    assert nav.calls == [(3.0, 4.0, 0.0)]
    assert gateway.goal_pose.msg_count == 1
    assert len(sent_goals) == 1
    assert sent_goals[0].pose.position.x == pytest.approx(3.0)
    assert sent_goals[0].pose.position.y == pytest.approx(4.0)
    assert sent_goals[0].frame_id == "map"
    assert model.frame_id == "map"


def test_goal_route_rejects_missing_plan_preview_without_publishing():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    _mark_navigation_ready(gateway)
    sent_goals = []
    gateway.goal_pose._add_callback(sent_goals.append)
    post_goal = _endpoint(gateway, "/api/v1/goal")

    response = asyncio.run(
        post_goal(
            GoalRequest(
                x=1.0,
                y=2.0,
                z=0.0,
                request_id="missing-preview",
                client_id="web",
            )
        )
    )
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "navigation_plan_infeasible"
    assert model.command is not None
    assert model.command.name == "goal"
    assert model.command.accepted is False
    assert model.detail["preview"]["source"] == "gateway_modules"
    assert model.detail["preview"]["reasons"] == ["navigation_module_unavailable"]
    assert sent_goals == []
    assert gateway.goal_pose.msg_count == 0


def test_direct_motion_commands_reject_safety_stop_without_publishing():
    from gateway.gateway_module import CmdVelRequest, GatewayModule, InstructionRequest
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")
    post_instruction = _endpoint(gateway, "/api/v1/instruction")

    cmd_response = asyncio.run(
        post_cmd_vel(
            CmdVelRequest(
                vx=0.2,
                wz=0.1,
                request_id="safety-stop-cmd",
                client_id="web",
            )
        )
    )
    instruction_response = asyncio.run(
        post_instruction(
            InstructionRequest(
                text="go to dock",
                request_id="safety-stop-instruction",
                client_id="web",
            )
        )
    )
    cmd_model = GatewayErrorResponse.model_validate(_payload(cmd_response))
    instruction_model = GatewayErrorResponse.model_validate(
        _payload(instruction_response)
    )

    assert cmd_response.status_code == 409
    assert cmd_model.error == "safety_stop"
    assert cmd_model.command is not None
    assert cmd_model.command.name == "cmd_vel"
    assert instruction_response.status_code == 409
    assert instruction_model.error == "safety_stop"
    assert instruction_model.command is not None
    assert instruction_model.command.name == "instruction"
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.instruction.msg_count == 0


def test_cmd_vel_rejects_safety_stop_without_publishing_and_emits_rejected_ack():
    from gateway.gateway_module import CmdVelRequest, GatewayModule
    from gateway.schemas import GatewayErrorResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")
    queue = gateway._sse_subscribe()

    try:
        response = asyncio.run(
            post_cmd_vel(
                CmdVelRequest(
                    vx=0.2,
                    wz=0.1,
                    request_id="safety-stop-cmd-ack",
                    client_id="web",
                )
            )
        )
        event = queue.get_nowait()
    finally:
        gateway._sse_unsubscribe(queue)
    model = GatewayErrorResponse.model_validate(_payload(response))

    assert response.status_code == 409
    assert model.error == "safety_stop"
    assert model.command is not None
    assert model.command.accepted is False
    assert gateway.cmd_vel.msg_count == 0
    assert event["type"] == "command_ack"
    assert event["data"]["status_code"] == 409
    assert event["data"]["command"]["name"] == "cmd_vel"
    assert event["data"]["command"]["accepted"] is False


def test_cmd_vel_replays_duplicate_request_id_without_republish():
    from gateway.gateway_module import CmdVelRequest, GatewayModule
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    post_cmd_vel = _endpoint(gateway, "/api/v1/cmd_vel")
    body = CmdVelRequest(
        vx=0.2,
        vy=0.0,
        wz=0.1,
        request_id="cmd-001",
        client_id="web",
    )

    first = asyncio.run(post_cmd_vel(body))
    second = asyncio.run(post_cmd_vel(body))
    first_model = ControlCommandResponse.model_validate(first)
    second_model = ControlCommandResponse.model_validate(second)

    assert first_model.ok is True
    assert first_model.command.name == "cmd_vel"
    assert first_model.command.replay is False
    assert second_model.command.replay is True
    assert gateway.cmd_vel.msg_count == 1


def test_cmd_vel_rejects_non_finite_vy():
    from gateway.gateway_module import CmdVelRequest

    with pytest.raises(ValueError):
        CmdVelRequest(vx=0.0, vy=float("nan"), wz=0.0)


def test_stop_command_remains_available_when_safety_stop_is_active():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._safety = {"level": 2}
    post_stop = _endpoint(gateway, "/api/v1/stop")

    result = asyncio.run(post_stop())
    model = ControlCommandResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "stopped"
    assert gateway.stop_cmd.msg_count == 1
    assert gateway.cmd_vel.msg_count == 1


def test_navigation_cancel_publishes_cancel_without_motion_outputs():
    from gateway.gateway_module import CancelRequest, GatewayModule
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    cancel_msgs: list[str] = []
    gateway.cancel.subscribe(cancel_msgs.append)
    post_cancel = _endpoint(gateway, "/api/v1/navigation/cancel")

    result = asyncio.run(
        post_cancel(
            CancelRequest(
                reason="operator_cancel",
                request_id="cancel-001",
                client_id="web",
            )
        )
    )
    model = ControlCommandResponse.model_validate(result)

    assert model.ok is True
    assert model.status == "cancelled"
    assert model.reason == "operator_cancel"
    assert model.command.name == "navigation_cancel"
    assert model.command.request_id == "cancel-001"
    assert gateway.cancel.msg_count == 1
    assert cancel_msgs == ["operator_cancel"]
    assert gateway.goal_pose.msg_count == 0
    assert gateway.cmd_vel.msg_count == 0
    assert gateway.stop_cmd.msg_count == 0


def test_commands_without_request_id_preserve_existing_execute_every_time_behavior():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
    post_stop = _endpoint(gateway, "/api/v1/stop")

    first = asyncio.run(post_stop())
    second = asyncio.run(post_stop())
    model = ControlCommandResponse.model_validate(first)

    assert gateway.stop_cmd.msg_count == 2
    assert gateway.cmd_vel.msg_count == 2
    assert model.schema_version == 1
    assert model.ok is True
    assert model.status == "stopped"
    assert first["status"] == "stopped"
    assert first["command"]["request_id"] is None
    assert second["command"]["replay"] is False


def test_lease_command_uses_receipt_and_replays_duplicate_request_id():
    from gateway.gateway_module import GatewayModule, LeaseRequest
    from gateway.schemas import LeaseResponse

    gateway = GatewayModule()
    gateway.setup()
    post_lease = _endpoint(gateway, "/api/v1/lease")
    queue = gateway._sse_subscribe()

    try:
        acquire = LeaseRequest(
            action="acquire",
            client_id="web",
            ttl=30.0,
            request_id="lease-001",
        )
        first = asyncio.run(post_lease(acquire))
        second = asyncio.run(post_lease(acquire))
        release = asyncio.run(
            post_lease(
                LeaseRequest(
                    action="release",
                    client_id="web",
                    ttl=30.0,
                    request_id="lease-release-001",
                )
            )
        )
        events = []
        while not queue.empty():
            events.append(queue.get_nowait())
    finally:
        gateway._sse_unsubscribe(queue)

    acquired = LeaseResponse.model_validate(first)
    replayed = LeaseResponse.model_validate(second)
    released = LeaseResponse.model_validate(release)
    command_stats = gateway._command_journal.snapshot()

    assert acquired.schema_version == 1
    assert acquired.ok is True
    assert acquired.status == "acquired"
    assert acquired.holder == "web"
    assert acquired.active is True
    assert acquired.command.name == "lease"
    assert acquired.command.request_id == "lease-001"
    assert acquired.command.client_id == "web"
    assert acquired.command.replay is False
    assert replayed.command.replay is True
    assert replayed.holder == "web"
    assert released.status == "released"
    assert released.active is False
    assert released.command.request_id == "lease-release-001"
    assert command_stats["accepted_commands"] == 2
    assert command_stats["replayed_commands"] == 1
    lease_events = [event for event in events if event["type"] == "lease"]
    ack_events = [event for event in events if event["type"] == "command_ack"]
    assert [event["data"]["status"] for event in lease_events] == ["acquired", "released"]
    assert ack_events[0]["data"]["command"]["name"] == "lease"


def test_lease_conflict_emits_rejected_ack_and_lease_event():
    from gateway.gateway_module import GatewayModule, LeaseRequest

    gateway = GatewayModule()
    gateway.setup()
    post_lease = _endpoint(gateway, "/api/v1/lease")
    queue = gateway._sse_subscribe()

    try:
        first = asyncio.run(
            post_lease(
                LeaseRequest(
                    action="acquire",
                    client_id="web",
                    ttl=30.0,
                    request_id="lease-web",
                )
            )
        )
        conflict = _payload(
            asyncio.run(
                post_lease(
                    LeaseRequest(
                        action="acquire",
                        client_id="mobile",
                        ttl=30.0,
                        request_id="lease-mobile",
                    )
                )
            )
        )
        events = []
        while not queue.empty():
            events.append(queue.get_nowait())
    finally:
        gateway._sse_unsubscribe(queue)

    lease_events = [event for event in events if event["type"] == "lease"]
    ack_events = [event for event in events if event["type"] == "command_ack"]

    assert first["ok"] is True
    assert conflict["ok"] is False
    assert conflict["error"] == "lease_conflict"
    assert conflict["command"]["accepted"] is False
    assert [event["data"]["status"] for event in lease_events] == [
        "acquired",
        "rejected",
    ]
    assert ack_events[-1]["data"]["ok"] is False
    assert ack_events[-1]["data"]["status_code"] == 409
    assert ack_events[-1]["data"]["command"]["client_id"] == "mobile"


def test_bootstrap_and_health_expose_command_policy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    gateway = GatewayModule()

    bootstrap = build_app_bootstrap(gateway)
    health = gateway.health()

    policy = bootstrap["control"]["command_policy"]
    assert policy["idempotency_supported"] is True
    assert policy["request_id_field"] == "request_id"
    assert policy["client_id_field"] == "client_id"
    assert policy["rate_policy_hz"]["cmd_vel"] == 20.0
    assert health["gateway"]["commands"]["rate_policy_enforcement"] == "advisory"
