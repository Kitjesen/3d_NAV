from __future__ import annotations

import asyncio

import pytest


pytest.importorskip("fastapi")


def test_app_bootstrap_service_returns_client_contract():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap, build_app_capabilities

    gateway = GatewayModule()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "yaw": 0.1}
        gateway._mission = {"state": "running", "goal": "dock"}
        gateway._safety = {"level": 0}
        gateway._mode = "autonomous"
        gateway._last_path = [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}]
        gateway._localization_status = {
            "state": "DEGRADED",
            "confidence": 0.4,
            "degeneracy": "MILD",
            "ts": 123.0,
        }
    gateway._session_mode = "navigating"
    gateway._icp_quality = 0.2
    gateway._frontier_explorer = object()
    gateway._webrtc = object()

    payload = build_app_bootstrap(gateway)

    assert payload["schema_version"] == 1
    assert payload["server"]["api_version"] == "v1"
    assert payload["robot"]["has_odometry"] is True
    assert payload["mission"]["state"] == "running"
    assert payload["safety"]["ok"] is True
    assert payload["localization"]["state"] == "degraded"
    assert payload["localization"]["reported_state"] == "DEGRADED"
    assert payload["localization"]["confidence"] == 0.4
    assert payload["navigation"]["state"] == "running"
    assert payload["navigation"]["path"]["points"] == 2
    assert payload["control"]["mode"] == "autonomous"
    assert payload["control"]["active_cmd_source"] == "unknown"
    assert payload["control"]["command_owner"] == "unknown"
    assert payload["control"]["estop_clear"] is True
    assert payload["path"]["points"] == 2
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert "path" not in payload["path"]
    assert payload["scene"]["endpoint"] == "/api/v1/scene_graph"
    assert "scene_graph" not in payload["scene"]
    assert payload["traffic"]["client_policy"]["usage"] == "cold_start_only"
    assert payload["traffic"]["client_policy"]["events_endpoint"] == "/api/v1/events"
    assert payload["capabilities"]["exploration"] is True
    assert payload["media"]["webrtc_available"] is True
    assert payload["capabilities_endpoint"] == "/api/v1/app/capabilities"
    assert payload["links"]["state"] == "/api/v1/state"
    assert payload["links"]["localization_status"] == "/api/v1/localization/status"
    assert payload["links"]["navigation_status"] == "/api/v1/navigation/status"
    assert payload["links"]["teleop_ws"] == "/ws/teleop"

    capabilities = build_app_capabilities(gateway)

    assert capabilities["schema_version"] == 1
    assert capabilities["features"]["exploration"] is True
    assert capabilities["features"]["localization"] is True
    assert capabilities["features"]["webrtc"] is True
    assert capabilities["endpoints"]["app"]["bootstrap"]["path"] == "/api/v1/app/bootstrap"
    assert (
        capabilities["endpoints"]["state"]["localization_status"]["path"]
        == "/api/v1/localization/status"
    )
    assert (
        capabilities["endpoints"]["state"]["navigation_status"]["path"]
        == "/api/v1/navigation/status"
    )
    assert capabilities["endpoints"]["control"]["goal"]["method"] == "POST"
    assert capabilities["endpoints"]["map"]["maps"]["path"] == "/api/v1/slam/maps"
    assert capabilities["endpoints"]["map"]["map_lifecycle"]["method"] == "POST"
    assert capabilities["probes"]["readiness"]["path"] == "/ready"
    assert capabilities["realtime"]["events"]["transport"] == "sse"
    assert capabilities["client_policy"]["retry_safe_when_request_id_present"] is True
    assert capabilities["client_policy"]["commands"]["idempotency_supported"] is True


def test_app_bootstrap_falls_back_when_session_snapshot_fails():
    from gateway.gateway_module import GatewayModule
    from gateway.services.app_bootstrap import build_app_bootstrap

    gateway = GatewayModule()

    def broken_session_snapshot():
        raise RuntimeError("boom")

    gateway._session_snapshot = broken_session_snapshot

    payload = build_app_bootstrap(gateway)

    assert payload["session"]["error"] == "session_snapshot_unavailable"
    assert payload["localization"]["state"] == "no_odometry"


def test_app_bootstrap_route_endpoint_returns_payload():
    from fastapi import FastAPI

    from gateway.gateway_module import GatewayModule
    from gateway.routes.app import register_app_routes

    gateway = GatewayModule()
    app = FastAPI()
    register_app_routes(app, gateway)

    route = next(route for route in app.routes if route.path == "/api/v1/app/bootstrap")
    payload = asyncio.run(route.endpoint())

    assert payload["schema_version"] == 1
    assert payload["links"]["health"] == "/api/v1/health"

    cap_route = next(route for route in app.routes if route.path == "/api/v1/app/capabilities")
    capabilities = asyncio.run(cap_route.endpoint())

    assert capabilities["schema_version"] == 1
    assert capabilities["endpoints"]["state"]["snapshot"]["path"] == "/api/v1/state"
    assert (
        capabilities["endpoints"]["state"]["navigation_status"]["path"]
        == "/api/v1/navigation/status"
    )


def test_app_capabilities_enriches_specs_from_openapi():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    route = next(
        route
        for route in gateway._app.routes
        if route.path == "/api/v1/app/capabilities"
    )
    capabilities = asyncio.run(route.endpoint())

    state = capabilities["endpoints"]["state"]["snapshot"]
    scene_graph = capabilities["endpoints"]["state"]["scene_graph"]
    locations = capabilities["endpoints"]["state"]["locations"]
    path = capabilities["endpoints"]["state"]["path"]
    localization = capabilities["endpoints"]["state"]["localization_status"]
    navigation = capabilities["endpoints"]["state"]["navigation_status"]
    goal = capabilities["endpoints"]["control"]["goal"]
    map_list = capabilities["endpoints"]["map"]["maps"]
    camera = capabilities["endpoints"]["media"]["camera_snapshot"]

    assert state["response_schema"] == "StateResponse"
    assert scene_graph["response_schema"] == "SceneGraphResponse"
    assert locations["response_schema"] == "LocationsResponse"
    assert path["response_schema"] == "PathResponse"
    assert localization["response_schema"] == "LocalizationStatusResponse"
    assert navigation["response_schema"] == "NavigationStatusResponse"
    assert goal["request_schema"] == "GoalRequest"
    assert goal["response_schema"] == "ControlCommandResponse"
    assert map_list["path"] == "/api/v1/slam/maps"
    assert map_list["response_schema"] == "MapListResponse"
    assert "image/jpeg" in camera["response_content_types"]
