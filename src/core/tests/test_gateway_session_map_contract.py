from __future__ import annotations

import asyncio
import json
import shutil
import struct
import uuid
from pathlib import Path

import pytest


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def _payload(response_or_payload):
    if hasattr(response_or_payload, "body"):
        return json.loads(response_or_payload.body)
    return response_or_payload


class _JsonRequest:
    def __init__(self, payload: dict):
        self._payload = payload

    async def json(self):
        return self._payload


class _FakeMapResponse:
    def __init__(self):
        self._callbacks = []

    def _add_callback(self, callback):
        self._callbacks.append(callback)


class _FakeMapCommand:
    def __init__(self, response: _FakeMapResponse):
        self.response = response
        self.delivered = []

    def _deliver(self, raw: str):
        command = json.loads(raw)
        self.delivered.append(command)
        for callback in list(self.response._callbacks):
            callback(
                {
                    "action": command["action"],
                    "success": True,
                    "name": command.get("name"),
                }
            )


class _FakeMapManager:
    def __init__(self):
        self.map_response = _FakeMapResponse()
        self.map_command = _FakeMapCommand(self.map_response)


def _write_binary_xyz_pcd(path: Path) -> None:
    header = (
        b"# .PCD v0.7\n"
        b"VERSION 0.7\n"
        b"FIELDS x y z\n"
        b"SIZE 4 4 4\n"
        b"TYPE F F F\n"
        b"COUNT 1 1 1\n"
        b"WIDTH 2\n"
        b"HEIGHT 1\n"
        b"POINTS 2\n"
        b"DATA binary\n"
    )
    path.write_bytes(header + struct.pack("<ffffff", 1, 2, 3, 4, 5, 6))


def test_auth_routes_validate_response_contracts(monkeypatch):
    from gateway import auth
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import AuthCheckResponse, AuthLoginResponse

    monkeypatch.setattr(auth, "_get_configured_key", lambda: None)

    gateway = GatewayModule()
    gateway.setup()

    check_payload = asyncio.run(_endpoint(gateway, "/api/v1/auth/check")())
    login_response = asyncio.run(
        _endpoint(gateway, "/api/v1/auth/login")(_JsonRequest({"key": ""}))
    )

    check = AuthCheckResponse.model_validate(check_payload)
    login = AuthLoginResponse.model_validate(_payload(login_response))

    assert check.auth_required is False
    assert login.ok is True
    assert login.message == "\u8ba4\u8bc1\u672a\u542f\u7528"


def test_auth_login_invalid_key_preserves_legacy_message(monkeypatch):
    from gateway import auth
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import AuthLoginResponse

    monkeypatch.setattr(auth, "_get_configured_key", lambda: "secret")

    gateway = GatewayModule()
    gateway.setup()

    login_response = asyncio.run(
        _endpoint(gateway, "/api/v1/auth/login")(_JsonRequest({"key": "bad"}))
    )
    login = AuthLoginResponse.model_validate(_payload(login_response))

    assert login_response.status_code == 403
    assert login.ok is False
    assert login.message == "Key \u65e0\u6548"


def test_lease_route_validates_success_and_conflict_payloads():
    from gateway.gateway_module import GatewayModule, LeaseRequest
    from gateway.schemas import GatewayErrorResponse, LeaseResponse

    gateway = GatewayModule()
    gateway.setup()
    post_lease = _endpoint(gateway, "/api/v1/lease")

    acquired_payload = asyncio.run(
        post_lease(LeaseRequest(action="acquire", client_id="web", ttl=30.0))
    )
    conflict_response = asyncio.run(
        post_lease(LeaseRequest(action="acquire", client_id="phone", ttl=30.0))
    )
    released_payload = asyncio.run(
        post_lease(LeaseRequest(action="release", client_id="web", ttl=30.0))
    )

    acquired = LeaseResponse.model_validate(acquired_payload)
    conflict = GatewayErrorResponse.model_validate(_payload(conflict_response))
    released = LeaseResponse.model_validate(released_payload)

    assert acquired.status == "acquired"
    assert acquired.holder == "web"
    assert acquired.active is True
    assert conflict.error == "lease_conflict"
    assert released.status == "released"


def test_session_routes_validate_idle_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SessionResponse, SessionTransitionResponse

    gateway = GatewayModule()
    gateway.setup()

    session_payload = asyncio.run(_endpoint(gateway, "/api/v1/session")())
    end_payload = asyncio.run(_endpoint(gateway, "/api/v1/session/end")())

    session = SessionResponse.model_validate(session_payload)
    ended = SessionTransitionResponse.model_validate(end_payload)

    assert session.mode == "idle"
    assert ended.success is True
    assert ended.session is not None
    assert ended.session.mode == "idle"


def test_map_routes_validate_json_contracts(monkeypatch):
    from gateway.gateway_module import GatewayModule, MapRequest
    from gateway.schemas import (
        GatewayErrorResponse,
        MapLifecycleResponse,
        MapListResponse,
        MapPointsResponse,
    )

    root = Path.cwd() / ".tmp_gateway_tests" / uuid.uuid4().hex
    try:
        map_dir = root / "maps"
        demo = map_dir / "demo"
        demo.mkdir(parents=True)
        _write_binary_xyz_pcd(demo / "map.pcd")
        (demo / "tomogram.pickle").write_bytes(b"tomogram")
        monkeypatch.setenv("NAV_MAP_DIR", str(map_dir))

        gateway = GatewayModule()
        gateway.setup()

        maps_payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/maps")())
        live_points_payload = asyncio.run(_endpoint(gateway, "/api/v1/map/points")())
        saved_points_payload = asyncio.run(
            _endpoint(gateway, "/api/v1/maps/{name}/points")("demo")
        )
        reset_payload = asyncio.run(_endpoint(gateway, "/api/v1/map_cloud/reset")())
        missing_manager_response = asyncio.run(
            _endpoint(gateway, "/api/v1/maps")(MapRequest(action="list"))
        )

        maps = MapListResponse.model_validate(maps_payload)
        live_points = MapPointsResponse.model_validate(live_points_payload)
        saved_points = MapPointsResponse.model_validate(saved_points_payload)
        reset = MapLifecycleResponse.model_validate(reset_payload)
        missing_manager = GatewayErrorResponse.model_validate(
            _payload(missing_manager_response)
        )

        assert [item.name for item in maps.maps] == ["demo"]
        assert maps.maps[0].has_pcd is True
        assert maps.maps[0].has_tomogram is True
        assert live_points.count == 0
        assert live_points.layout == "xyz_rows"
        assert live_points.points == []
        assert saved_points.count == 2
        assert saved_points.layout == "flat_xyz"
        assert saved_points.points == [1, 2, 3, 4, 5, 6]
        assert reset.success is True
        assert missing_manager.error == "MapManagerModule not running"
    finally:
        shutil.rmtree(root, ignore_errors=True)


def test_maps_route_accepts_legacy_and_canonical_actions():
    from gateway.gateway_module import GatewayModule, MapRequest
    from gateway.schemas import MapLifecycleResponse

    gateway = GatewayModule()
    gateway.setup()
    manager = _FakeMapManager()
    gateway._map_mgr = manager
    post_maps = _endpoint(gateway, "/api/v1/maps")

    use_payload = asyncio.run(post_maps(MapRequest(action="use", name="demo")))
    build_payload = asyncio.run(post_maps(MapRequest(action="build", name="demo")))
    canonical_payload = asyncio.run(
        post_maps(MapRequest(action="build_tomogram", name="demo"))
    )

    assert MapLifecycleResponse.model_validate(use_payload).success is True
    assert MapLifecycleResponse.model_validate(build_payload).success is True
    assert MapLifecycleResponse.model_validate(canonical_payload).success is True
    assert manager.map_command.delivered == [
        {"action": "set_active", "name": "demo"},
        {"action": "build_tomogram", "name": "demo"},
        {"action": "build_tomogram", "name": "demo"},
    ]


def test_operational_routes_validate_idle_json_contracts():
    from gateway.gateway_module import BitrateRequest, GatewayModule
    from gateway.schemas import (
        BagOperationResponse,
        BagStatusResponse,
        ExplorationStatusResponse,
        GatewayErrorResponse,
        SlamOperationResponse,
        SlamStatusResponse,
        TemporalMemoryResponse,
        WebRTCControlResponse,
        WebRTCStatsResponse,
    )

    gateway = GatewayModule()
    gateway.setup()

    class _TemporalStore:
        def query(self, **_kwargs):
            return [{"label": "door", "confidence": 0.9}]

        def query_semantic(self, *_args, **_kwargs):
            return [{"label": "door", "score": 0.98}]

    gateway._temporal_store = _TemporalStore()

    temporal_payload = asyncio.run(_endpoint(gateway, "/api/v1/memory/temporal")())
    semantic_payload = asyncio.run(
        _endpoint(gateway, "/api/v1/memory/temporal/semantic")(
            {"embedding": [0.1, 0.2], "top_k": 1}
        )
    )
    semantic_response = asyncio.run(
        _endpoint(gateway, "/api/v1/memory/temporal/semantic")({})
    )
    explore_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/explore/status")())
    explore_start_response = asyncio.run(_endpoint(gateway, "/api/v1/explore/start")())
    slam_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/slam/status")())
    slam_switch_response = asyncio.run(
        _endpoint(gateway, "/api/v1/slam/switch")({"profile": "bad"})
    )
    bag_status_payload = asyncio.run(_endpoint(gateway, "/api/v1/bag/status")())
    bag_stop_response = asyncio.run(_endpoint(gateway, "/api/v1/bag/stop")())
    webrtc_stats_payload = asyncio.run(_endpoint(gateway, "/api/v1/webrtc/stats")())
    webrtc_bitrate_response = asyncio.run(
        _endpoint(gateway, "/api/v1/webrtc/bitrate")(BitrateRequest(bps=1_000_000))
    )

    temporal = TemporalMemoryResponse.model_validate(temporal_payload)
    semantic_ok = TemporalMemoryResponse.model_validate(semantic_payload)
    semantic = GatewayErrorResponse.model_validate(_payload(semantic_response))
    explore_status = ExplorationStatusResponse.model_validate(explore_status_payload)
    explore_start = GatewayErrorResponse.model_validate(_payload(explore_start_response))
    slam_status = SlamStatusResponse.model_validate(slam_status_payload)
    slam_switch = SlamOperationResponse.model_validate(_payload(slam_switch_response))
    bag_status = BagStatusResponse.model_validate(bag_status_payload)
    bag_stop = BagOperationResponse.model_validate(_payload(bag_stop_response))
    webrtc_stats = WebRTCStatsResponse.model_validate(webrtc_stats_payload)
    webrtc_bitrate = WebRTCControlResponse.model_validate(
        _payload(webrtc_bitrate_response)
    )

    assert temporal.count == 1
    assert temporal.observations[0]["label"] == "door"
    assert semantic_ok.count == 1
    assert semantic.error == "embedding required"
    assert explore_status.available is False
    assert explore_start.error == "WavefrontFrontierExplorer not running"
    assert slam_status.mode in {"fastlio2", "localizer", "slam_only", "stopped"}
    assert slam_switch.success is False
    assert bag_status.recording is False
    assert bag_stop.error == "not_recording"
    assert webrtc_stats.enabled is False
    assert webrtc_bitrate.error == "webrtc_unavailable"


def test_temporal_memory_response_accepts_observation_rows():
    from gateway.schemas import TemporalMemoryResponse

    payload = {
        "observations": [{"label": "door", "score": 0.92}],
        "count": 1,
    }

    response = TemporalMemoryResponse.model_validate(payload)

    assert response.count == 1
    assert response.observations[0]["label"] == "door"
