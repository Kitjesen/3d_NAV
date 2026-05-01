from __future__ import annotations

from types import SimpleNamespace

import pytest


pytest.importorskip("fastapi")


class _HealthyModule:
    def health(self):
        return {"state": "ok"}


class _BrokenModule:
    def health(self):
        raise RuntimeError("boom")


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
