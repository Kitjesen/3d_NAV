from __future__ import annotations

import asyncio
import json

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
    assert isinstance(payload["ts"], float)
    assert payload["server"]["time"] == payload["ts"]
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
    assert payload["control"]["can_send_goal"] is True
    assert payload["control"]["goal_blockers"] == []
    assert payload["path"]["points"] == 2
    assert payload["path"]["endpoint"] == "/api/v1/path"
    assert "path" not in payload["path"]
    assert payload["scene"]["endpoint"] == "/api/v1/scene_graph"
    assert "scene_graph" not in payload["scene"]
    assert payload["traffic"]["client_policy"]["usage"] == "cold_start_only"
    assert payload["traffic"]["client_policy"]["events_endpoint"] == "/api/v1/events"
    assert payload["traffic"]["client_policy"]["large_event_policy"]["slope_grid_payload"] == "metadata_sse"
    assert payload["capabilities"]["exploration"] is True
    assert payload["media"]["webrtc_available"] is True
    assert payload["media"]["camera_ws"] == "/ws/camera"
    assert payload["capabilities_endpoint"] == "/api/v1/app/capabilities"
    assert payload["links"]["state"] == "/api/v1/state"
    assert payload["links"]["localization_status"] == "/api/v1/localization/status"
    assert payload["links"]["navigation_status"] == "/api/v1/navigation/status"
    assert payload["links"]["navigation_plan"] == "/api/v1/navigation/plan"
    assert payload["links"]["teleop_ws"] == "/ws/teleop"
    assert payload["links"]["camera_ws"] == "/ws/camera"
    assert payload["links"]["session_start"] == "/api/v1/session/start"
    assert payload["links"]["session_end"] == "/api/v1/session/end"

    capabilities = build_app_capabilities(gateway)

    assert capabilities["schema_version"] == 1
    assert isinstance(capabilities["ts"], float)
    assert capabilities["server"]["time"] == capabilities["ts"]
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
    assert capabilities["endpoints"]["control"]["navigation_plan"]["method"] == "POST"
    assert capabilities["endpoints"]["map"]["maps"]["path"] == "/api/v1/slam/maps"
    assert capabilities["endpoints"]["map"]["map_lifecycle"]["method"] == "POST"
    assert capabilities["endpoints"]["map"]["map_activate"]["path"] == "/api/v1/map/activate"
    assert capabilities["endpoints"]["map"]["map_rename"]["path"] == "/api/v1/map/rename"
    assert capabilities["endpoints"]["map"]["map_save"]["path"] == "/api/v1/map/save"
    assert (
        capabilities["endpoints"]["map"]["map_restore_predufo"]["path"]
        == "/api/v1/map/restore_predufo"
    )
    assert capabilities["probes"]["readiness"]["path"] == "/ready"
    assert capabilities["realtime"]["events"]["transport"] == "sse"
    assert capabilities["realtime"]["events"]["event_schema"] == "SSEEventEnvelope"
    assert capabilities["realtime"]["events"]["event_id_field"] == "event_id"
    assert capabilities["realtime"]["events"]["retry_ms"] == 3000
    assert capabilities["realtime"]["events"]["replay_supported"] is False
    assert capabilities["realtime"]["events"]["last_event_id_header"] == "Last-Event-ID"
    assert capabilities["realtime"]["events"]["named_events"] is False
    assert capabilities["realtime"]["events"]["browser_handler"] == "onmessage"
    assert {"snapshot", "ping", "slam_diag", "gnss_fusion", "slam_drift"} <= set(
        capabilities["realtime"]["events"]["event_types"]
    )
    assert {"slam_diag", "gnss_fusion", "slam_drift"} <= set(
        capabilities["realtime"]["events"]["diagnostic_event_types"]
    )
    assert "heartbeat" in capabilities["realtime"]["events"]["legacy_event_types"]
    assert capabilities["realtime"]["events"]["large_event_policy"]["point_cloud_payload"] == "binary_websocket"
    assert capabilities["realtime"]["teleop"]["binary_camera_frames"] is False
    assert capabilities["realtime"]["teleop"]["legacy_camera_query"] == "?video=1"
    assert capabilities["realtime"]["camera"]["path"] == "/ws/camera"
    assert capabilities["realtime"]["camera"]["binary_camera_frames"] is True
    assert capabilities["endpoints"]["realtime"]["camera"]["method"] == "WS"
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
    assert payload["ts"] > 0
    assert payload["links"]["health"] == "/api/v1/health"

    cap_route = next(route for route in app.routes if route.path == "/api/v1/app/capabilities")
    capabilities = asyncio.run(cap_route.endpoint())

    assert capabilities["schema_version"] == 1
    assert capabilities["ts"] > 0
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
    control = capabilities["endpoints"]["control"]
    navigation_plan = capabilities["endpoints"]["control"]["navigation_plan"]
    goal = capabilities["endpoints"]["control"]["goal"]
    map_list = capabilities["endpoints"]["map"]["maps"]
    session_start = capabilities["endpoints"]["map"]["session_start"]
    map_activate = capabilities["endpoints"]["map"]["map_activate"]
    map_rename = capabilities["endpoints"]["map"]["map_rename"]
    map_save = capabilities["endpoints"]["map"]["map_save"]
    slam_switch = capabilities["endpoints"]["ops"]["slam_switch"]
    slam_relocalize = capabilities["endpoints"]["ops"]["slam_relocalize"]
    bag_start = capabilities["endpoints"]["ops"]["bag_start"]
    memory_semantic = capabilities["endpoints"]["ops"]["memory_temporal_semantic"]
    webrtc_offer = capabilities["endpoints"]["media"]["webrtc_offer"]
    events = capabilities["endpoints"]["realtime"]["events"]
    camera = capabilities["endpoints"]["media"]["camera_snapshot"]

    assert state["response_schema"] == "StateResponse"
    assert scene_graph["response_schema"] == "SceneGraphResponse"
    assert locations["response_schema"] == "LocationsResponse"
    assert path["response_schema"] == "PathResponse"
    assert localization["response_schema"] == "LocalizationStatusResponse"
    assert navigation["response_schema"] == "NavigationStatusResponse"
    assert navigation_plan["request_schema"] == "PlanPreviewRequest"
    assert navigation_plan["response_schema"] == "PlanPreviewResponse"
    assert goal["request_schema"] == "GoalRequest"
    assert goal["response_schema"] == "ControlCommandResponse"
    for name in ("goal", "navigate_click", "stop", "instruction", "mode", "lease"):
        assert "409" in control[name]["status_codes"]
    assert "403" in control["lease"]["status_codes"]
    assert map_list["path"] == "/api/v1/slam/maps"
    assert map_list["response_schema"] == "MapListResponse"
    assert session_start["request_schema"] == "SessionStartRequest"
    assert session_start["response_schema"] == "SessionTransitionResponse"
    assert map_activate["request_schema"] == "MapNameRequest"
    assert map_activate["response_schema"] == "MapLifecycleResponse"
    assert map_rename["request_schema"] == "MapRenameRequest"
    assert map_rename["response_schema"] == "MapLifecycleResponse"
    assert map_save["request_schema"] == "MapSaveRequest"
    assert map_save["response_schema"] == "MapLifecycleResponse"
    assert slam_switch["request_schema"] == "SlamSwitchRequest"
    assert slam_relocalize["request_schema"] == "SlamRelocalizeRequest"
    assert bag_start["request_schema"] == "BagStartRequest"
    assert memory_semantic["request_schema"] == "TemporalSemanticRequest"
    assert webrtc_offer["request_schema"] == "WebRTCOfferRequest"
    assert events["response_schema"] == "SSEEventEnvelope"
    assert events["response_content_types"] == ["text/event-stream"]
    assert "image/jpeg" in camera["response_content_types"]


def test_app_web_cold_start_routes_return_stable_client_shapes():
    from fastapi.testclient import TestClient

    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._odom = {
            "x": 1.25,
            "y": -0.5,
            "z": 0.0,
            "yaw": 0.2,
            "frame_id": "map",
            "ts": 101.0,
        }
        gateway._mission = {"state": "idle"}
        gateway._safety = {"level": 0}
        gateway._mode = "manual"
        gateway._last_path = [
            {"x": 1.25, "y": -0.5, "z": 0.0},
            {"x": 2.0, "y": 0.0, "z": 0.0},
        ]
        gateway._sg_json = json.dumps(
            {
                "frame_id": "map",
                "ts": 102.0,
                "objects": [
                    {
                        "id": "obj-1",
                        "label": "dock",
                        "x": 2.0,
                        "y": 0.0,
                        "confidence": 0.9,
                    }
                ],
            }
        )

    client = TestClient(gateway._app)

    bootstrap = client.get("/api/v1/app/bootstrap")
    state = client.get("/api/v1/state")
    scene_graph = client.get("/api/v1/scene_graph")
    locations = client.get("/api/v1/locations")
    path = client.get("/api/v1/path")
    capabilities = client.get("/api/v1/app/capabilities")

    for response in (
        bootstrap,
        state,
        scene_graph,
        locations,
        path,
        capabilities,
    ):
        assert response.status_code == 200

    bootstrap_payload = bootstrap.json()
    state_payload = state.json()
    scene_graph_payload = scene_graph.json()
    locations_payload = locations.json()
    path_payload = path.json()
    capabilities_payload = capabilities.json()

    assert bootstrap_payload["schema_version"] == 1
    assert bootstrap_payload["ts"] > 0
    assert bootstrap_payload["server"]["time"] == bootstrap_payload["ts"]
    assert bootstrap_payload["traffic"]["client_policy"]["usage"] == "cold_start_only"
    assert bootstrap_payload["links"]["state"] == "/api/v1/state"
    assert bootstrap_payload["links"]["events"] == "/api/v1/events"
    assert "scene_graph" not in bootstrap_payload["scene"]
    assert "path" not in bootstrap_payload["path"]
    assert "can_send_goal" in bootstrap_payload["control"]
    assert isinstance(bootstrap_payload["control"]["goal_blockers"], list)

    assert state_payload["schema_version"] == 1
    assert state_payload["ts"] > 0
    assert state_payload["server"]["time"] == state_payload["ts"]
    assert state_payload["links"]["state"] == "/api/v1/state"
    assert state_payload["links"]["events"] == "/api/v1/events"
    assert state_payload["links"]["scene_graph"] == "/api/v1/scene_graph"
    assert state_payload["links"]["locations"] == "/api/v1/locations"
    assert state_payload["links"]["path"] == "/api/v1/path"
    assert state_payload["links"]["camera_ws"] == "/ws/camera"
    assert state_payload["links"]["cloud_ws"] == "/ws/cloud"
    assert state_payload["links"]["health"] == "/api/v1/health"
    assert state_payload["links"]["goal"] == "/api/v1/goal"
    assert state_payload["links"]["stop"] == "/api/v1/stop"
    assert state_payload["path"]["points"] == 2

    assert scene_graph_payload["schema_version"] == 1
    assert scene_graph_payload["frame_id"] == "map"
    assert scene_graph_payload["count"] == 1
    assert scene_graph_payload["objects"][0]["label"] == "dock"
    assert scene_graph_payload["scene_graph"] is not None

    assert locations_payload["schema_version"] == 1
    assert locations_payload["locations"] == []
    assert locations_payload["count"] == 0

    assert path_payload["schema_version"] == 1
    assert path_payload["count"] == 2
    assert path_payload["robot"]["x"] == 1.25
    assert path_payload["path"][1]["x"] == 2.0

    assert capabilities_payload["schema_version"] == 1
    assert capabilities_payload["ts"] > 0
    assert capabilities_payload["server"]["time"] == capabilities_payload["ts"]
    assert (
        capabilities_payload["endpoints"]["app"]["bootstrap"]["response_schema"]
        == "AppBootstrapResponse"
    )
    assert (
        capabilities_payload["endpoints"]["state"]["snapshot"]["response_schema"]
        == "StateResponse"
    )
    assert capabilities_payload["links"]["events"] == "/api/v1/events"
    assert capabilities_payload["links"]["map_activate"] == "/api/v1/map/activate"
    assert capabilities_payload["links"]["map_rename"] == "/api/v1/map/rename"
    assert capabilities_payload["links"]["map_save"] == "/api/v1/map/save"


def test_app_web_events_stream_starts_with_snapshot_contract():
    from gateway.gateway_module import GatewayModule

    async def read_first_event(gateway):
        route = next(
            route
            for route in gateway._app.routes
            if route.path == "/api/v1/events"
        )
        response = await route.endpoint()
        iterator = response.body_iterator
        try:
            chunk = await iterator.__anext__()
        finally:
            close = getattr(iterator, "aclose", None)
            if close is not None:
                await close()
        return response, chunk

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "yaw": 0.25, "ts": 201.0}
        gateway._mission = {"state": "idle"}
        gateway._safety = {"level": 0}
        gateway._mode = "manual"

    response, chunk = asyncio.run(read_first_event(gateway))
    line = chunk.decode("utf-8") if isinstance(chunk, bytes) else chunk
    data_line = next(item for item in line.splitlines() if item.startswith("data: "))
    payload = json.loads(data_line.removeprefix("data: "))

    assert response.media_type == "text/event-stream"
    assert payload["type"] == "snapshot"
    assert payload["data"]["odometry"]["x"] == 1.0
    assert payload["schema_version"] == 1
    assert payload["event_id"] == 1
    assert payload["ts"] > 0
    assert payload["data"]["mission"]["state"] == "idle"
    assert payload["data"]["mode"] == "manual"
    assert payload["data"]["session"]["mode"] in {"idle", "navigating", "mapping"}
    assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0


def test_app_web_events_stream_uses_sse_ids_without_named_event_type():
    from gateway.gateway_module import GatewayModule

    async def read_first_event(gateway):
        route = next(
            route
            for route in gateway._app.routes
            if route.path == "/api/v1/events"
        )
        response = await route.endpoint()
        iterator = response.body_iterator
        try:
            chunk = await iterator.__anext__()
        finally:
            close = getattr(iterator, "aclose", None)
            if close is not None:
                await close()
        return chunk

    gateway = GatewayModule()
    gateway.setup()

    chunk = asyncio.run(read_first_event(gateway))
    text = chunk.decode("utf-8") if isinstance(chunk, bytes) else chunk

    assert text.startswith("retry: 3000\nid: 1\ndata: ")
    assert "\nevent:" not in text
    data_line = next(line for line in text.splitlines() if line.startswith("data: "))
    payload = json.loads(data_line.removeprefix("data: "))

    assert payload["type"] == "snapshot"
    assert payload["schema_version"] == 1
    assert payload["event_id"] == 1


def test_app_web_events_reconnect_snapshot_uses_latest_state_and_monotonic_id():
    from gateway.gateway_module import GatewayModule

    async def read_first_payload(gateway):
        route = next(
            route
            for route in gateway._app.routes
            if route.path == "/api/v1/events"
        )
        response = await route.endpoint()
        iterator = response.body_iterator
        try:
            chunk = await iterator.__anext__()
        finally:
            close = getattr(iterator, "aclose", None)
            if close is not None:
                await close()
        text = chunk.decode("utf-8") if isinstance(chunk, bytes) else chunk
        data_line = next(line for line in text.splitlines() if line.startswith("data: "))
        return json.loads(data_line.removeprefix("data: "))

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._odom = {"x": 1.0, "y": 2.0, "yaw": 0.25}
        gateway._mission = {"state": "idle"}

    first = asyncio.run(read_first_payload(gateway))

    with gateway._state_lock:
        gateway._odom = {"x": 3.0, "y": 4.0, "yaw": 0.5}
        gateway._mission = {"state": "running", "goal": "dock"}

    second = asyncio.run(read_first_payload(gateway))

    assert first["type"] == "snapshot"
    assert second["type"] == "snapshot"
    assert second["event_id"] > first["event_id"]
    assert second["data"]["odometry"]["x"] == 3.0
    assert second["data"]["mission"]["state"] == "running"
    assert second["data"]["mission"]["goal"] == "dock"
    assert gateway._traffic_stats_snapshot()["sse"]["clients"] == 0
