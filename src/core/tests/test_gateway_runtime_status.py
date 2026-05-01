from __future__ import annotations

import asyncio

import pytest


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    gateway.setup()
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def test_localization_status_covers_product_states():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_localization_status

    gateway = GatewayModule()

    payload = build_localization_status(gateway)
    assert payload["state"] == "no_odometry"
    assert payload["has_odometry"] is False
    assert payload["can_relocalize"] is False

    with gateway._state_lock:
        gateway._odom = {"x": 1.0}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "tracking"
    assert payload["reported_state"] == "TRACKING"

    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.2
    with gateway._state_lock:
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    payload = build_localization_status(gateway)
    assert payload["state"] == "ready"
    assert payload["ready"] is True

    with gateway._state_lock:
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.4,
            "degeneracy": "MILD",
        }
    payload = build_localization_status(gateway)
    assert payload["state"] == "degraded"
    assert payload["can_relocalize"] is True
    assert "low_confidence" in payload["reasons"]

    with gateway._state_lock:
        gateway._localization_status = {"state": "LOST", "confidence": 0.0}
    payload = build_localization_status(gateway)
    assert payload["state"] == "lost"
    assert payload["can_relocalize"] is True


def test_localization_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.8}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/localization/status")())

    assert payload["schema_version"] == 1
    assert payload["state"] == "tracking"
    assert payload["has_odometry"] is True
    assert payload["reported_state"] == "TRACKING"


def test_navigation_status_reports_mission_path_and_control_source():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "path_follower",
                "sources": {
                    "path_follower": {"active": True, "priority": 40},
                },
            }

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "wp_index": 2,
            "wp_total": 5,
            "replan_count": 1,
            "speed_scale": 0.5,
            "failure_reason": "",
            "ts": 123.0,
        }
        gateway._last_path = [{"x": 0.0}, {"x": 1.0}, {"x": 2.0}]
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["state"] == "EXECUTING"
    assert payload["can_accept_goal"] is True
    assert payload["wp_index"] == 2
    assert payload["wp_total"] == 5
    assert payload["replan_count"] == 1
    assert payload["speed_scale"] == 0.5
    assert payload["path"]["points"] == 3
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert payload["control"]["mode"] == "autonomous"
    assert payload["control"]["active_cmd_source"] == "path_follower"
    assert payload["control"]["command_owner"] == "navigation"
    assert payload["control"]["source_category"] == "autonomy"
    assert payload["control"]["manual_override"] is False
    assert payload["control"]["preempting_autonomy"] is False
    assert payload["progress"]["fraction"] == 0.4
    assert payload["progress"]["active"] is True
    assert payload["readiness"]["can_execute_autonomy"] is True
    assert payload["reason_codes"] == []
    assert payload["localization"]["degraded"] is False


def test_navigation_status_handles_failed_mission_and_missing_mux():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {
            "state": "STUCK",
            "failure_reason": "blocked",
            "speed_scale": 0.25,
        }
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.2,
        }

    payload = build_navigation_status(gateway)

    assert payload["state"] == "STUCK"
    assert payload["failure_reason"] == "blocked"
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert payload["control"]["cmd_vel_mux"]["available"] is False
    assert payload["localization"]["degraded"] is True
    assert "mission_stuck" in payload["reason_codes"]
    assert "failure_blocked" in payload["reason_codes"]
    assert "localization_degraded" in payload["reason_codes"]
    assert "cmd_vel_mux_unavailable" in payload["reason_codes"]
    assert payload["diagnostics"]["failure_reason"] == "blocked"


def test_navigation_status_reports_teleop_preemption_for_active_mission():
    from gateway.gateway_module import GatewayModule
    from gateway.services.runtime_status import build_navigation_status

    class FakeMux:
        def health(self):
            return {
                "active_source": "teleop",
                "sources": {
                    "teleop": {
                        "active": True,
                        "priority": 100,
                        "age_ms": 20,
                    },
                    "path_follower": {
                        "active": True,
                        "priority": 40,
                        "age_ms": 25,
                    },
                },
            }

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0}
        gateway._mode = "autonomous"
        gateway._mission = {
            "state": "EXECUTING",
            "wp_index": 1,
            "wp_total": 4,
            "speed_scale": 1.0,
        }
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.9}
    gateway._all_modules = {"CmdVelMux": FakeMux()}

    payload = build_navigation_status(gateway)

    assert payload["control"]["active_cmd_source"] == "teleop"
    assert payload["control"]["command_owner"] == "teleop"
    assert payload["control"]["source_category"] == "manual"
    assert payload["control"]["manual_override"] is True
    assert payload["control"]["preempting_autonomy"] is True
    assert payload["control"]["active_source"]["priority"] == 100
    assert "control_preempted_by_teleop" in payload["reason_codes"]
    assert "control_preempted_by_teleop" in payload["readiness"]["advisories"]


def test_navigation_status_route_returns_stable_schema():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._mission = {"state": "IDLE"}

    payload = asyncio.run(_endpoint(gateway, "/api/v1/navigation/status")())

    assert payload["schema_version"] == 1
    assert payload["state"] == "IDLE"
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert "odometry_missing" in payload["reason_codes"]
    assert payload["readiness"]["blockers"] == ["odometry_missing"]
