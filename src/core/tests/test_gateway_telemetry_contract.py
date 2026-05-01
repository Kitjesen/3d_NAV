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
        gateway._sg_json = '{"nodes":[]}'
        gateway._last_path = [{"x": 0.0, "y": 0.0}, {"x": 1.0, "y": 1.0}]
        gateway._odom = {"x": 0.5, "y": 0.25}
    gateway._tagged_loc_module = SimpleNamespace(
        store=SimpleNamespace(
            _store={
                "dock": {"name": "dock", "x": 1.23456, "y": 2.34567, "z": 0.0},
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

    assert scene.scene_graph == '{"nodes":[]}'
    assert len(path.path) == 2
    assert path.robot == {"x": 0.5, "y": 0.25}
    assert len(locations.locations) == 1
    assert locations.locations[0].name == "dock"
    assert locations.locations[0].x == 1.235
    assert locations.locations[0].y == 2.346


def test_locations_route_is_empty_without_tagged_location_module():
    from gateway.gateway_module import GatewayModule
    from gateway.schemas import LocationsResponse

    gateway = GatewayModule()
    gateway.setup()

    payload = asyncio.run(_endpoint(gateway, "/api/v1/locations")())
    locations = LocationsResponse.model_validate(payload)

    assert locations.locations == []
