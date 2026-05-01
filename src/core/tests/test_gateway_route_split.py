from __future__ import annotations

import asyncio
import io
import json
import shutil
import tarfile
import uuid
from collections import Counter
from pathlib import Path
from types import SimpleNamespace

import pytest


fastapi = pytest.importorskip("fastapi")


def test_auth_routes_register_expected_paths():
    from fastapi import FastAPI
    from fastapi.exceptions import RequestValidationError

    from gateway.routes.auth import register_auth_routes

    app = FastAPI()
    register_auth_routes(app)

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/api/v1/auth/login" in paths
    assert "/api/v1/auth/check" in paths
    assert RequestValidationError in app.exception_handlers


def test_realtime_routes_register_expected_websockets():
    from fastapi import FastAPI

    from gateway.routes.realtime import register_realtime_routes

    app = FastAPI()
    gw = SimpleNamespace()
    register_realtime_routes(app, gw)

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/ws/teleop" in paths
    assert "/ws/cloud" in paths


def test_map_routes_register_expected_paths():
    from fastapi import FastAPI

    from gateway.routes.maps import register_map_routes

    app = FastAPI()
    register_map_routes(app, SimpleNamespace())

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/api/v1/slam/maps" in paths
    assert "/api/v1/maps/{name}/pcd" in paths
    assert "/api/v1/maps/{name}/points" in paths
    assert "/api/v1/map/points" in paths
    assert "/api/v1/map_cloud/reset" in paths
    assert "/map/viewer" in paths
    assert "/robot/meshes/{filename}" in paths
    assert "/api/v1/map/restore_predufo" in paths
    assert "/api/v1/map/activate" in paths
    assert "/api/v1/map/rename" in paths
    assert "/api/v1/map/save" in paths


def test_status_routes_register_expected_paths():
    from fastapi import FastAPI

    from gateway.routes.status import register_status_routes

    app = FastAPI()
    register_status_routes(app, SimpleNamespace())

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/api/v1/events" in paths
    assert "/api/v1/state" in paths
    assert "/api/v1/scene_graph" in paths
    assert "/api/v1/locations" in paths
    assert "/api/v1/path" in paths
    assert "/api/v1/localization/status" in paths
    assert "/api/v1/navigation/status" in paths
    assert "/api/v1/devices" in paths
    assert "/api/v1/health" in paths
    assert "/health" in paths
    assert "/ready" in paths


def test_session_routes_register_expected_paths():
    from fastapi import FastAPI

    from gateway.routes.session import register_session_routes

    app = FastAPI()
    gw = SimpleNamespace()
    register_session_routes(app, gw)

    paths = {getattr(route, "path", "") for route in app.routes}
    assert "/api/v1/session" in paths
    assert "/api/v1/session/start" in paths
    assert "/api/v1/session/end" in paths


def test_camera_routes_register_expected_snapshot_path():
    from fastapi import FastAPI

    from gateway.routes.camera import register_camera_routes

    app = FastAPI()
    register_camera_routes(app)

    route = next(route for route in app.routes if route.path == "/api/v1/camera/snapshot")
    assert route.endpoint.__module__ == "gateway.routes.camera"


def test_command_routes_register_expected_paths():
    from fastapi import FastAPI

    from gateway.routes.commands import register_command_routes

    app = FastAPI()
    register_command_routes(app, SimpleNamespace())

    routes = {getattr(route, "path", ""): route for route in app.routes}
    assert "/api/v1/goal" in routes
    assert "/api/v1/navigate/click" in routes
    assert "/api/v1/cmd_vel" in routes
    assert "/api/v1/stop" in routes
    assert "/api/v1/instruction" in routes
    assert "/api/v1/mode" in routes
    assert "/api/v1/lease" in routes
    assert routes["/api/v1/goal"].endpoint.__module__ == "gateway.routes.commands"


def test_map_route_file_resolution_rejects_path_escape(monkeypatch):
    from fastapi import HTTPException

    from gateway.routes.maps import _safe_map_file

    temp_root = Path.cwd() / ".tmp" / "test_gateway_route_split" / uuid.uuid4().hex
    map_root = temp_root / "maps"
    map_root.mkdir(parents=True, exist_ok=False)
    monkeypatch.setenv("NAV_MAP_DIR", str(map_root))

    try:
        assert _safe_map_file("demo", "map.pcd").name == "map.pcd"
        with pytest.raises(HTTPException) as exc_info:
            _safe_map_file("..", "map.pcd")
        assert exc_info.value.status_code == 403
    finally:
        shutil.rmtree(temp_root, ignore_errors=True)


def test_diagnostic_routes_export_tarball(monkeypatch):
    from fastapi import FastAPI
    from fastapi.testclient import TestClient

    from gateway.routes import diagnostics
    from gateway.routes.diagnostics import register_diagnostic_routes

    temp_root = Path.cwd() / ".tmp" / "test_gateway_route_split" / uuid.uuid4().hex
    temp_root.mkdir(parents=True, exist_ok=False)
    monkeypatch.setattr(diagnostics.tempfile, "gettempdir", lambda: str(temp_root))

    class HealthyModule:
        def health(self):
            return {"ok": True}

    try:
        app = FastAPI()
        register_diagnostic_routes(
            app,
            SimpleNamespace(_all_modules={"healthy": HealthyModule()}),
        )

        response = TestClient(app).get("/api/v1/diagnostic_pack")
        assert response.status_code == 200
        assert response.headers["content-type"].startswith("application/gzip")

        with tarfile.open(fileobj=io.BytesIO(response.content), mode="r:gz") as tar:
            names = set(tar.getnames())
            assert "diag/modules.json" in names
            assert "diag/health.json" in names
            modules_file = tar.extractfile("diag/modules.json")
            assert modules_file is not None
            modules = json.loads(modules_file.read().decode("utf-8"))
            assert modules["healthy"] == {"ok": True}
        assert list(temp_root.glob("diag_*.tar.gz")) == []
    finally:
        shutil.rmtree(temp_root, ignore_errors=True)


def test_gateway_module_builds_split_routes_once():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    counts = Counter(getattr(route, "path", "") for route in gateway._app.routes)
    assert counts["/api/v1/app/bootstrap"] == 1
    assert counts["/api/v1/app/capabilities"] == 1
    assert counts["/api/v1/auth/login"] == 1
    assert counts["/api/v1/auth/check"] == 1
    assert counts["/api/v1/diagnostic_pack"] == 1
    assert counts["/api/v1/events"] == 1
    assert counts["/api/v1/state"] == 1
    assert counts["/api/v1/localization/status"] == 1
    assert counts["/api/v1/navigation/status"] == 1
    assert counts["/api/v1/health"] == 1
    assert counts["/health"] == 1
    assert counts["/ready"] == 1
    assert counts["/api/v1/session"] == 1
    assert counts["/api/v1/session/start"] == 1
    assert counts["/api/v1/session/end"] == 1
    assert counts["/api/v1/slam/maps"] == 1
    assert counts["/api/v1/map/points"] == 1
    assert counts["/api/v1/map_cloud/reset"] == 1
    assert counts["/api/v1/goal"] == 1
    assert counts["/api/v1/navigate/click"] == 1
    assert counts["/api/v1/cmd_vel"] == 1
    assert counts["/api/v1/stop"] == 1
    assert counts["/api/v1/instruction"] == 1
    assert counts["/api/v1/mode"] == 1
    assert counts["/api/v1/lease"] == 1
    assert counts["/ws/teleop"] == 1
    assert counts["/ws/cloud"] == 1


def test_gateway_module_keeps_client_route_inventory():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    paths = {getattr(route, "path", "") for route in gateway._app.routes}
    expected = {
        "/api/v1/app/bootstrap",
        "/api/v1/app/capabilities",
        "/api/v1/state",
        "/api/v1/scene_graph",
        "/api/v1/locations",
        "/api/v1/path",
        "/api/v1/localization/status",
        "/api/v1/navigation/status",
        "/api/v1/devices",
        "/api/v1/health",
        "/health",
        "/ready",
        "/api/v1/session",
        "/api/v1/diagnostic_pack",
        "/api/v1/events",
        "/api/v1/maps",
        "/api/v1/slam/maps",
        "/api/v1/map/points",
        "/api/v1/maps/{name}/points",
        "/api/v1/map_cloud/reset",
        "/api/v1/goal",
        "/api/v1/navigate/click",
        "/api/v1/cmd_vel",
        "/api/v1/stop",
        "/api/v1/instruction",
        "/api/v1/mode",
        "/api/v1/lease",
        "/api/v1/camera/snapshot",
        "/ws/teleop",
        "/ws/cloud",
    }
    assert expected <= paths


def test_capabilities_manifest_paths_exist_in_gateway_routes():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    route_paths = {getattr(route, "path", "") for route in gateway._app.routes}
    route = next(
        route
        for route in gateway._app.routes
        if route.path == "/api/v1/app/capabilities"
    )
    capabilities = asyncio.run(route.endpoint())

    manifest_paths = set()
    for group in capabilities["endpoints"].values():
        for spec in group.values():
            manifest_paths.add(spec["path"])
    for spec in capabilities["probes"].values():
        manifest_paths.add(spec["path"])

    assert manifest_paths <= route_paths


def _schema_ref_for(
    openapi: dict,
    path: str,
    status: str = "200",
    method: str = "get",
) -> str:
    return openapi["paths"][path][method]["responses"][status]["content"][
        "application/json"
    ]["schema"]["$ref"]


def _content_for(
    openapi: dict,
    path: str,
    method: str = "get",
    status: str = "200",
) -> dict:
    return openapi["paths"][path][method]["responses"][status]["content"]


def test_openapi_exposes_client_response_models():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    openapi = gateway._app.openapi()
    schemas = openapi["components"]["schemas"]

    assert "StateResponse" in schemas
    assert "ReadinessResponse" in schemas
    assert "AppBootstrapResponse" in schemas
    assert "AppCapabilitiesResponse" in schemas
    assert "HealthResponse" in schemas
    assert "DevicesResponse" in schemas
    assert "LivenessResponse" in schemas
    assert "ControlCommandResponse" in schemas
    assert "GatewayErrorResponse" in schemas
    assert "CommandReceipt" in schemas
    assert "SceneGraphResponse" in schemas
    assert "SceneGraphObject" in schemas
    assert "SceneGraphRelation" in schemas
    assert "SceneGraphRegion" in schemas
    assert "LocationsResponse" in schemas
    assert "LocationEntry" in schemas
    assert "PathResponse" in schemas
    assert "PathPoint" in schemas
    assert "RobotPoseSummary" in schemas
    assert "LocalizationStatusResponse" in schemas
    assert "NavigationStatusResponse" in schemas
    assert "NavigationControlSummary" in schemas
    assert "NavigationReadinessSummary" in schemas
    assert "NavigationProgressSummary" in schemas
    assert "NavigationDiagnosticsSummary" in schemas
    assert "AuthLoginResponse" in schemas
    assert "AuthCheckResponse" in schemas
    assert "LeaseResponse" in schemas
    assert "SessionResponse" in schemas
    assert "SessionTransitionResponse" in schemas
    assert "MapLifecycleResponse" in schemas
    assert "MapListResponse" in schemas
    assert "MapPointsResponse" in schemas
    assert "TemporalMemoryResponse" in schemas
    assert "ExplorationCommandResponse" in schemas
    assert "ExplorationStatusResponse" in schemas
    assert "SlamStatusResponse" in schemas
    assert "SlamOperationResponse" in schemas
    assert "BagOperationResponse" in schemas
    assert "BagStatusResponse" in schemas
    assert "WebRTCStatsResponse" in schemas
    assert "Go2RTCStatusResponse" in schemas
    assert "WebRTCControlResponse" in schemas
    assert _schema_ref_for(openapi, "/api/v1/state").endswith("/StateResponse")
    assert _schema_ref_for(openapi, "/ready").endswith("/ReadinessResponse")
    assert _schema_ref_for(openapi, "/ready", "503").endswith("/ReadinessResponse")
    assert _schema_ref_for(openapi, "/api/v1/health").endswith("/HealthResponse")
    assert _schema_ref_for(openapi, "/api/v1/devices").endswith("/DevicesResponse")
    assert _schema_ref_for(openapi, "/health").endswith("/LivenessResponse")
    assert _schema_ref_for(openapi, "/api/v1/scene_graph").endswith(
        "/SceneGraphResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/locations").endswith(
        "/LocationsResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/path").endswith("/PathResponse")
    assert _schema_ref_for(openapi, "/api/v1/localization/status").endswith(
        "/LocalizationStatusResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/navigation/status").endswith(
        "/NavigationStatusResponse"
    )
    assert _schema_ref_for(
        openapi, "/api/v1/goal", method="post"
    ).endswith("/ControlCommandResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/cmd_vel", method="post"
    ).endswith("/ControlCommandResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/stop", method="post"
    ).endswith("/ControlCommandResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/instruction", method="post"
    ).endswith("/ControlCommandResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/mode", method="post"
    ).endswith("/ControlCommandResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/auth/login", method="post"
    ).endswith("/AuthLoginResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/auth/check"
    ).endswith("/AuthCheckResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/lease", method="post"
    ).endswith("/LeaseResponse")
    assert _schema_ref_for(openapi, "/api/v1/session").endswith("/SessionResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/session/start", method="post"
    ).endswith("/SessionTransitionResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/session/end", method="post"
    ).endswith("/SessionTransitionResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/maps", method="post"
    ).endswith("/MapLifecycleResponse")
    assert _schema_ref_for(openapi, "/api/v1/slam/maps").endswith(
        "/MapListResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/map/points").endswith(
        "/MapPointsResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/maps/{name}/points").endswith(
        "/MapPointsResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/memory/temporal").endswith(
        "/TemporalMemoryResponse"
    )
    assert _schema_ref_for(
        openapi, "/api/v1/memory/temporal/semantic", method="post"
    ).endswith("/TemporalMemoryResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/explore/start", method="post"
    ).endswith("/ExplorationCommandResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/explore/stop", method="post"
    ).endswith("/ExplorationCommandResponse")
    assert _schema_ref_for(openapi, "/api/v1/explore/status").endswith(
        "/ExplorationStatusResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/slam/status").endswith(
        "/SlamStatusResponse"
    )
    assert _schema_ref_for(
        openapi, "/api/v1/slam/switch", method="post"
    ).endswith("/SlamOperationResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/slam/auto_relocalize", method="post"
    ).endswith("/SlamOperationResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/slam/relocalize", method="post"
    ).endswith("/SlamOperationResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/bag/start", method="post"
    ).endswith("/BagOperationResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/bag/stop", method="post"
    ).endswith("/BagOperationResponse")
    assert _schema_ref_for(openapi, "/api/v1/bag/status").endswith(
        "/BagStatusResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/webrtc/stats").endswith(
        "/WebRTCStatsResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/webrtc/go2rtc/status").endswith(
        "/Go2RTCStatusResponse"
    )
    assert _schema_ref_for(
        openapi, "/api/v1/webrtc/offer", method="post"
    ).endswith("/WebRTCControlResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/webrtc/bitrate", method="post"
    ).endswith("/WebRTCControlResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/map_cloud/reset", method="post"
    ).endswith("/MapLifecycleResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/map/activate", method="post"
    ).endswith("/MapLifecycleResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/map/rename", method="post"
    ).endswith("/MapLifecycleResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/map/save", method="post"
    ).endswith("/MapLifecycleResponse")
    assert _schema_ref_for(
        openapi, "/api/v1/map/restore_predufo", method="post"
    ).endswith("/MapLifecycleResponse")
    assert _schema_ref_for(openapi, "/api/v1/app/bootstrap").endswith(
        "/AppBootstrapResponse"
    )
    assert _schema_ref_for(openapi, "/api/v1/app/capabilities").endswith(
        "/AppCapabilitiesResponse"
    )

    assert schemas["SceneGraphResponse"]["properties"]["objects"]["items"][
        "$ref"
    ].endswith("/SceneGraphObject")
    assert schemas["SceneGraphResponse"]["properties"]["relations"]["items"][
        "$ref"
    ].endswith("/SceneGraphRelation")
    assert schemas["SceneGraphResponse"]["properties"]["regions"]["items"][
        "$ref"
    ].endswith("/SceneGraphRegion")
    assert schemas["LocationsResponse"]["properties"]["locations"]["items"][
        "$ref"
    ].endswith("/LocationEntry")
    assert schemas["PathResponse"]["properties"]["path"]["items"]["$ref"].endswith(
        "/PathPoint"
    )
    assert schemas["NavigationStatusResponse"]["properties"]["control"]["$ref"].endswith(
        "/NavigationControlSummary"
    )
    assert schemas["NavigationStatusResponse"]["properties"]["readiness"][
        "$ref"
    ].endswith("/NavigationReadinessSummary")
    assert schemas["NavigationStatusResponse"]["properties"]["progress"][
        "$ref"
    ].endswith("/NavigationProgressSummary")
    assert "schema_version" in schemas["ControlCommandResponse"]["properties"]
    assert "ok" in schemas["ControlCommandResponse"]["properties"]
    assert schemas["ControlCommandResponse"]["properties"]["command"]["$ref"].endswith(
        "/CommandReceipt"
    )
    assert "schema_version" in schemas["GatewayErrorResponse"]["properties"]
    assert "ok" in schemas["GatewayErrorResponse"]["properties"]
    assert schemas["GatewayErrorResponse"]["properties"]["command"]["anyOf"][0][
        "$ref"
    ].endswith("/CommandReceipt")
    assert "request_id" in schemas["LeaseRequest"]["properties"]
    assert schemas["LeaseResponse"]["properties"]["command"]["$ref"].endswith(
        "/CommandReceipt"
    )
    assert "command" in schemas["LeaseResponse"]["required"]

    assert "application/sdp" in _content_for(
        openapi, "/api/v1/webrtc/whep", method="post"
    )
    assert "text/event-stream" in _content_for(openapi, "/api/v1/events")
    assert "application/gzip" in _content_for(openapi, "/api/v1/diagnostic_pack")
    assert "application/octet-stream" in _content_for(
        openapi, "/api/v1/maps/{name}/pcd"
    )
    assert "image/jpeg" in _content_for(openapi, "/api/v1/camera/snapshot")


def test_capabilities_manifest_http_paths_exist_in_openapi():
    from gateway.gateway_module import GatewayModule

    gateway = GatewayModule()
    gateway.setup()

    route = next(
        route
        for route in gateway._app.routes
        if route.path == "/api/v1/app/capabilities"
    )
    capabilities = asyncio.run(route.endpoint())
    openapi_paths = set(gateway._app.openapi()["paths"])

    manifest_paths = set()
    for group in capabilities["endpoints"].values():
        for spec in group.values():
            if spec["method"] != "WS":
                manifest_paths.add(spec["path"])
    for spec in capabilities["probes"].values():
        manifest_paths.add(spec["path"])

    assert manifest_paths <= openapi_paths

    key_specs = [
        capabilities["endpoints"]["state"]["snapshot"],
        capabilities["endpoints"]["state"]["scene_graph"],
        capabilities["endpoints"]["state"]["locations"],
        capabilities["endpoints"]["state"]["path"],
        capabilities["endpoints"]["state"]["localization_status"],
        capabilities["endpoints"]["state"]["navigation_status"],
        capabilities["endpoints"]["state"]["devices"],
        capabilities["endpoints"]["state"]["health"],
        capabilities["endpoints"]["app"]["bootstrap"],
        capabilities["endpoints"]["app"]["capabilities"],
    ]
    for spec in key_specs:
        assert spec["response_schema"]
        assert "application/json" in spec["response_content_types"]
