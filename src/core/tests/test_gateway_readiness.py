from __future__ import annotations

import json
import math
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


def test_readiness_snapshot_reports_not_started_without_modules():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()

    payload, status_code = build_readiness_snapshot(gateway, now=123.0)

    assert status_code == 503
    assert payload["schema_version"] == 1
    assert payload["status"] == "not_started"
    assert payload["ready"] is False
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
    assert payload["module_count"] == 2
    assert payload["failed_modules"] == []
    assert payload["modules"]["A"]["detail"] == {"state": "ok"}
    assert payload["modules"]["B"]["detail"] == {}


def test_readiness_snapshot_reports_failed_modules_and_keeps_legacy_fields():
    from gateway.gateway_module import GatewayModule
    from gateway.services.readiness import build_readiness_snapshot

    gateway = GatewayModule()
    gateway._all_modules = {"A": _HealthyModule(), "Bad": _BrokenModule()}

    payload, status_code = build_readiness_snapshot(gateway, now=125.0)

    assert status_code == 503
    assert payload["status"] == "degraded"
    assert payload["ready"] is False
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
    assert payload["runtime"]["navigation"]["blockers"] == ["pose_stale"]
