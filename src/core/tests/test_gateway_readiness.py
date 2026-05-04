from __future__ import annotations

import json
import math
import asyncio
from types import SimpleNamespace

import pytest


pytest.importorskip("fastapi")


class _HealthyModule:
    def health(self):
        return {"state": "ok"}


class _BrokenModule:
    def health(self):
        raise RuntimeError("boom")


class _NonFiniteModule:
    def health(self):
        return {
            "state": "diagnostic",
            "nan": math.nan,
            "pos_inf": math.inf,
            "nested": {"neg_inf": -math.inf},
            "values": [1.0, math.nan],
        }


class _SlowHealthModule:
    def health(self):
        raise AssertionError("summary readiness must not call module health")


def test_readiness_snapshot_reports_not_started_without_modules():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()

    payload, status_code = build_readiness_snapshot(gateway, now=123.0)

    assert status_code == 503
    assert payload["schema_version"] == 1
    assert payload["status"] == "not_started"
    assert payload["ready"] is False
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert payload["modules"] == {}
    assert payload["module_count"] == 0
    assert payload["reasons"] == ["no_modules_loaded"]
    assert payload["ts"] == 123.0


def test_readiness_snapshot_reports_ready_when_all_modules_are_healthy():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _HealthyModule(), "B": SimpleNamespace()}

    payload, status_code = build_readiness_snapshot(gateway, now=124.0)

    assert status_code == 200
    assert payload["status"] == "ready"
    assert payload["ready"] is True
    assert payload["data_ready"] is True
    assert payload["motion_ready"] is True
    assert payload["non_motion_safe"] is True
    assert payload["module_count"] == 2
    assert payload["failed_modules"] == []
    assert payload["modules"]["A"]["detail"] == {"state": "ok"}
    assert payload["modules"]["B"]["detail"] == {}


def test_readiness_snapshot_can_omit_module_details_for_probe_payload():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _SlowHealthModule()}

    payload, status_code = build_readiness_snapshot(
        gateway,
        now=124.5,
        include_details=False,
    )

    assert status_code == 200
    assert payload["ready"] is True
    assert payload["modules"]["A"] == {"ok": True}


def test_ready_route_defaults_to_summary_and_supports_details_query():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    gateway._all_modules = {"A": _HealthyModule()}
    endpoint = next(route.endpoint for route in gateway._app.routes if route.path == "/ready")

    summary_response = asyncio.run(endpoint())
    summary_payload = json.loads(summary_response.body)
    detail_response = asyncio.run(endpoint(details=True))
    detail_payload = json.loads(detail_response.body)

    assert summary_payload["modules"]["A"] == {"ok": True}
    assert detail_payload["modules"]["A"]["detail"] == {"state": "ok"}


def test_readiness_snapshot_reports_failed_modules_and_keeps_legacy_fields():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _HealthyModule(), "Bad": _BrokenModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=125.0)

    assert status_code == 503
    assert payload["status"] == "degraded"
    assert payload["ready"] is False
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert payload["failed_modules"] == ["Bad"]
    assert payload["reasons"] == ["module_failed:Bad"]
    assert payload["modules"]["Bad"]["ok"] is False
    assert payload["modules"]["Bad"]["error"] == "boom"


def test_readiness_snapshot_sanitizes_non_finite_health_values():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _NonFiniteModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=126.0)

    assert status_code == 200
    detail = payload["modules"]["A"]["detail"]
    assert detail["nan"] is None
    assert detail["pos_inf"] is None
    assert detail["nested"]["neg_inf"] is None
    assert detail["values"] == [1.0, None]
    json.dumps(payload, allow_nan=False)


def test_readiness_snapshot_blocks_lost_robot_localization():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"SlamBridgeModule": _HealthyModule()}
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._localization_status = {
            "state": "LOST",
            "confidence": 0.0,
        }

    payload, status_code = build_readiness_snapshot(gateway, now=127.0)

    assert status_code == 503
    assert payload["status"] == "degraded"
    assert payload["ready"] is False
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert payload["failed_modules"] == []
    assert "localization:lost" in payload["reasons"]
    assert payload["runtime"]["localization"]["state"] == "lost"


def test_readiness_snapshot_includes_navigation_blockers():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"NavigationModule": _HealthyModule()}
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 2500.0,
            "localizer_health": "RECOVERED",
        }

    payload, status_code = build_readiness_snapshot(gateway, now=128.0)

    assert status_code == 503
    assert "localization:degraded" in payload["reasons"]
    assert "navigation_blocked:pose_stale" in payload["reasons"]
    assert payload["data_ready"] is False
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert payload["runtime"]["navigation"]["blockers"] == ["pose_stale"]
    assert payload["runtime"]["summary"]["data_blockers"] == [
        "localization:degraded",
        "localization:pose_stale",
        "navigation_blocked:pose_stale",
    ]


def test_readiness_snapshot_blocks_motion_but_not_data_when_safety_stop_active():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"NavigationModule": _HealthyModule()}
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.03
    with gateway._state_lock:
        gateway._odom = {"x": 0.0, "y": 0.0, "z": 0.0}
        gateway._mission = {"state": "IDLE"}
        gateway._safety = {"level": 2}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.028,
            "odom_age_ms": 100.0,
            "localizer_health": "RECOVERED",
        }

    payload, status_code = build_readiness_snapshot(gateway, now=128.5)

    assert status_code == 503
    assert payload["ready"] is False
    assert payload["data_ready"] is True
    assert payload["motion_ready"] is False
    assert payload["non_motion_safe"] is True
    assert "navigation_blocked:safety_stop" in payload["reasons"]
    assert "safety:stop" in payload["reasons"]
    assert payload["runtime"]["safety"]["stop_active"] is True
    assert payload["runtime"]["summary"]["data_blockers"] == []


def test_readiness_snapshot_flags_active_command_source_as_not_non_motion_safe():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    class FakeMux:
        def health(self):
            return {
                "active_source": "teleop",
                "sources": {
                    "teleop": {"active": True, "priority": 100},
                },
            }

    gateway = GatewayModule()
    gateway._all_modules = {
        "NavigationModule": _HealthyModule(),
        "CmdVelMux": FakeMux(),
    }
    gateway._session_mode = "navigating"
    with gateway._state_lock:
        gateway._odom = {"x": 0.0}
        gateway._mission = {"state": "EXECUTING"}
        gateway._localization_status = {
            "state": "TRACKING",
            "confidence": 0.9,
            "degeneracy": "NONE",
            "icp_fitness": 0.03,
            "localizer_health": "RECOVERED",
            "odom_age_ms": 100.0,
        }
        gateway._icp_quality = 0.03

    payload, status_code = build_readiness_snapshot(gateway, now=129.0)

    assert status_code == 200
    assert payload["ready"] is True
    assert payload["data_ready"] is True
    assert payload["motion_ready"] is True
    assert payload["non_motion_safe"] is False
    assert payload["runtime"]["navigation"]["active_cmd_source"] == "teleop"
    assert payload["runtime"]["summary"]["mission_state"] == "EXECUTING"
