from __future__ import annotations

import asyncio

import pytest


pytest.importorskip("fastapi")


def test_state_snapshot_preserves_legacy_fields_and_adds_client_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.services.state_snapshot import build_state_snapshot

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0}
        gateway._safety = {"level": 0}
        gateway._mission = {"state": "running"}
        gateway._eval = {"score": 0.9}
        gateway._dialogue = {"last": "ok"}
        gateway._mode = "autonomous"
        gateway._teleop_clients = 2
        gateway._sg_json = '{"nodes":[]}'
        gateway._last_path = [{"x": 0.0}, {"x": 1.0}]
        gateway._localization_status = {"state": "TRACKING", "confidence": 0.8}
    gateway._teleop_module = type("TeleopStub", (), {"_active": True})()

    payload = build_state_snapshot(gateway)

    assert payload["schema_version"] == 1
    assert payload["ts"] > 0
    assert payload["server"]["time"] == payload["ts"]
    assert payload["server"]["api_version"] == "v1"
    assert payload["odometry"] == {"x": 1.0, "y": 2.0}
    assert payload["safety"] == {"level": 0}
    assert payload["mission"] == {"state": "running"}
    assert payload["eval"] == {"score": 0.9}
    assert payload["dialogue"] == {"last": "ok"}
    assert payload["mode"] == "autonomous"
    assert payload["lease"]["holder"] is None
    assert payload["teleop"] == {"active": True, "clients": 2}
    assert payload["localization"]["reported_state"] == "TRACKING"
    assert payload["localization"]["confidence"] == 0.8
    assert payload["navigation"]["state"] == "running"
    assert payload["navigation"]["path"]["points"] == 2
    assert payload["scene"]["available"] is True
    assert payload["path"]["points"] == 2
    assert payload["links"]["capabilities"] == "/api/v1/app/capabilities"
    assert payload["links"]["localization_status"] == "/api/v1/localization/status"
    assert payload["links"]["navigation_status"] == "/api/v1/navigation/status"


def test_state_route_returns_stable_snapshot():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    route = next(route for route in gateway._app.routes if route.path == "/api/v1/state")
    payload = asyncio.run(route.endpoint())

    assert payload["schema_version"] == 1
    assert payload["ts"] > 0
    assert "odometry" in payload
    assert "teleop" in payload
    assert "navigation" in payload
    assert payload["links"]["events"] == "/api/v1/events"
