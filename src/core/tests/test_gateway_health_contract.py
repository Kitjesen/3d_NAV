from __future__ import annotations

import asyncio

import pytest


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def test_liveness_and_devices_routes_validate_response_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import DevicesResponse, LivenessResponse

    gateway = GatewayModule()
    gateway.setup()

    liveness = asyncio.run(_endpoint(gateway, "/health")())
    devices = asyncio.run(_endpoint(gateway, "/api/v1/devices")())

    live_model = LivenessResponse.model_validate(liveness)
    devices_model = DevicesResponse.model_validate(devices)

    assert live_model.status == "ok"
    assert live_model.ts > 0
    assert devices_model.manager == "not_loaded"
    assert devices_model.devices == []


def test_health_schema_keeps_top_level_app_contract_flexible():
    from gateway.schemas import HealthResponse

    model = HealthResponse.model_validate(
        {
            "status": "ok",
            "modules_ok": 3,
            "modules_fail": 0,
            "gateway": {
                "port": 5050,
                "mode": "manual",
                "traffic": {"sse": {"clients": 0}},
                "commands": {"idempotency_supported": True},
            },
            "teleop": {"active": False, "clients": 0},
            "sensors": {"camera": {"status": "idle"}},
            "slam_hz": 0.0,
            "map_points": 0,
            "has_odom": False,
            "modules": {"GatewayModule": "ok"},
            "brainstem": {"status": "unavailable"},
            "future_field": "preserved",
        }
    )

    assert model.status == "ok"
    assert model.gateway["port"] == 5050
    assert model.teleop.clients == 0
    assert model.model_extra["future_field"] == "preserved"
