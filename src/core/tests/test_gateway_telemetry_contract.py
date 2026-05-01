from __future__ import annotations

import asyncio
from types import SimpleNamespace

import pytest


pytest.importorskip("fastapi")


def _endpoint(gateway, path: str):
    return next(route.endpoint for route in gateway._app.routes if route.path == path)


def test_scene_graph_path_and_locations_routes_validate_response_contracts():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationsResponse, PathResponse, SceneGraphResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._sg_json = (
            '{"frame_id":"map","ts":123.0,'
            '"objects":[{"id":"obj-1","label":"dock","x":1.0,"y":2.0,'
            '"confidence":0.9}],'
            '"relations":[{"source":"obj-1","target":"region-1",'
            '"relation":"inside"}],'
            '"regions":[{"id":"region-1","name":"yard"}]}'
        )
        gateway._last_path = [
            {"x": 0.0, "y": 0.0, "frame_id": "map"},
            (1.0, 1.0, 0.1),
        ]
        gateway._odom = {"x": 0.5, "y": 0.25, "yaw": 0.2, "vx": 0.1}
    gateway._tagged_loc_module = SimpleNamespace(
        store=SimpleNamespace(
            _store={
                "dock": {
                    "name": "dock",
                    "x": 1.23456,
                    "y": 2.34567,
                    "z": 0.0,
                    "yaw": 0.4,
                    "tags": ["charger", "home"],
                    "source": "test",
                    "ts": 456.0,
                },
                "bad": {"x": 9.0, "y": 9.0},
            }
        )
    )

    scene_payload = asyncio.run(_endpoint(gateway, "/api/v1/scene_graph")())
    path_payload = asyncio.run(_endpoint(gateway, "/api/v1/path")())
    locations_payload = asyncio.run(_endpoint(gateway, "/api/v1/locations")())

    scene = SceneGraphResponse.model_validate(scene_payload)
    path = PathResponse.model_validate(path_payload)
    locations = LocationsResponse.model_validate(locations_payload)

    assert scene.schema_version == 1
    assert scene.frame_id == "map"
    assert scene.ts == 123.0
    assert scene.count == 1
    assert scene.objects[0].id == "obj-1"
    assert scene.objects[0].label == "dock"
    assert scene.objects[0].confidence == 0.9
    assert scene.relations[0].relation == "inside"
    assert scene.regions[0].name == "yard"
    assert scene.scene_graph == gateway._sg_json
    assert path.schema_version == 1
    assert len(path.path) == 2
    assert path.count == 2
    assert path.path[0].frame_id == "map"
    assert path.path[1].z == 0.1
    assert path.robot is not None
    assert path.robot.x == 0.5
    assert path.robot.y == 0.25
    assert path.robot.yaw == 0.2
    assert path.source == "gateway_cache"
    assert locations.schema_version == 1
    assert locations.count == 1
    assert len(locations.locations) == 1
    assert locations.locations[0].name == "dock"
    assert locations.locations[0].x == 1.235
    assert locations.locations[0].y == 2.346
    assert locations.locations[0].yaw == 0.4
    assert locations.locations[0].tags == ["charger", "home"]
    assert locations.locations[0].source == "test"


def test_scene_graph_route_degrades_invalid_json_to_empty_structured_payload():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import SceneGraphResponse

    gateway = GatewayModule()
    gateway.setup()
    with gateway._state_lock:
        gateway._sg_json = "{not-json"

    scene_payload = asyncio.run(_endpoint(gateway, "/api/v1/scene_graph")())
    scene = SceneGraphResponse.model_validate(scene_payload)

    assert scene.scene_graph == "{not-json"
    assert scene.objects == []
    assert scene.relations == []
    assert scene.regions == []
    assert scene.count == 0


def test_locations_route_is_empty_without_tagged_location_module():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationsResponse

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/locations")())
    locations = LocationsResponse.model_validate(payload)

    assert locations.locations == []
    assert locations.count == 0
