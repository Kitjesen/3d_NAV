from __future__ import annotations

import asyncio
import time

import pytest


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _mark_navigation_ready(gateway) -> None:
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
    def __init__(self) -> None:
        self.calls: list[tuple[float, float, float]] = []

    def preview_plan(self, x: float, y: float, z: float) -> dict:
        self.calls.append((x, y, z))
        ts = time.time()
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


def test_command_journal_replays_duplicate_request_id_without_republish():
    from gateway.gateway_module import GatewayModule, GoalRequest
    from gateway.schemas import ControlCommandResponse

    gateway = GatewayModule()
    gateway.setup()
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
