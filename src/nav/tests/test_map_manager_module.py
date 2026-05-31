"""Tests for MapManagerModule -- map CRUD, POI operations, command dispatch.

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from core.msgs.sensor import PointCloud2
from nav.services.nav_services.map_manager_module import MapManagerModule


# -- fixtures ------------------------------------------------------------------


@pytest.fixture
def map_manager(tmp_path):
    """MapManagerModule with temp directories."""
    map_dir = tmp_path / "maps"
    data_dir = tmp_path / "data"
    mod = MapManagerModule(map_dir=str(map_dir), data_dir=str(data_dir))
    mod.setup()
    # Collect published responses
    responses: list[dict] = []
    mod.map_response.subscribe(lambda r: responses.append(r))
    mod._test_responses = responses
    return mod


def _cmd(mod, cmd: dict) -> dict:
    """Send a JSON command and return the last published response."""
    mod._test_responses.clear()
    mod._on_command(json.dumps(cmd))
    assert len(mod._test_responses) > 0, "no response published"
    return mod._test_responses[-1]


def _write_minimal_pcd(map_dir: Path) -> None:
    """Write a minimal valid PCD file into an existing map dir."""
    (map_dir / "map.pcd").write_text(
        "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        "COUNT 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA ascii\n0.0 0.0 0.0\n",
        encoding="utf-8",
    )


# -- tests ---------------------------------------------------------------------


class TestMapManagerModule:
    """MapManagerModule instantiation, port types, command dispatch."""

    def test_instantiation(self, map_manager):
        """Module creates map_dir and data_dir on init."""
        assert os.path.isdir(map_manager._map_dir)
        assert os.path.isdir(map_manager._data_dir)

    def test_port_types(self, map_manager):
        """Verify In/Out port registration and types via port_summary."""
        s = map_manager.port_summary()
        assert "map_command" in s["ports_in"]
        assert s["ports_in"]["map_command"]["type"] == "str"
        assert "map_response" in s["ports_out"]
        assert s["ports_out"]["map_response"]["type"] == "dict"
        assert "map_cloud" in s["ports_in"]
        assert s["ports_in"]["map_cloud"]["type"] == "PointCloud2"

    def test_list_empty(self, map_manager):
        """list returns empty when no maps exist."""
        resp = _cmd(map_manager, {"action": "list"})
        assert resp["success"] is True
        assert resp["maps"] == []

    def test_delete_nonexistent(self, map_manager):
        """delete on a non-existent map returns failure."""
        resp = _cmd(map_manager, {"action": "delete", "name": "no_such_map"})
        assert resp["success"] is False
        assert "not found" in resp["message"]

    def test_rename_creates_and_renames(self, map_manager):
        """rename a map directory and verify files move."""
        src = Path(map_manager._map_dir) / "alpha"
        src.mkdir()
        (src / "map.pcd").touch()
        resp = _cmd(map_manager, {"action": "rename", "name": "alpha", "new_name": "beta"})
        assert resp["success"] is True
        assert not src.exists()
        assert (Path(map_manager._map_dir) / "beta" / "map.pcd").exists()

    def test_set_active_validates_artifacts(self, map_manager):
        """set_active rejects a map without metadata artifacts."""
        d = Path(map_manager._map_dir) / "badmap"
        d.mkdir()
        (d / "map.pcd").write_text("VERSION 0.7\nPOINTS 0\nDATA ascii\n")
        (d / "tomogram.pickle").write_bytes(b"bad")
        resp = _cmd(map_manager, {"action": "set_active", "name": "badmap"})
        assert resp["success"] is False
        assert "saved map artifact gate failed" in resp["message"]

    def test_poi_set_list_delete(self, map_manager):
        """POI CRUD: set, list, then delete."""
        resp = _cmd(map_manager, {"action": "poi_set", "name": "home", "x": 1.0, "y": 2.0})
        assert resp["success"] is True

        resp = _cmd(map_manager, {"action": "poi_list"})
        assert "home" in resp["pois"]

        resp = _cmd(map_manager, {"action": "poi_delete", "name": "home"})
        assert resp["success"] is True

        resp = _cmd(map_manager, {"action": "poi_list"})
        assert "home" not in resp["pois"]

    def test_poi_delete_nonexistent(self, map_manager):
        """delete a POI that does not exist."""
        resp = _cmd(map_manager, {"action": "poi_delete", "name": "nope"})
        assert resp["success"] is False

    def test_unknown_action(self, map_manager):
        """unknown action returns error."""
        resp = _cmd(map_manager, {"action": "foobar"})
        assert resp["success"] is False
        assert "unknown" in resp.get("message", "")

    def test_invalid_json(self, map_manager):
        """malformed JSON is handled gracefully."""
        map_manager._test_responses.clear()
        map_manager._on_command("not json{{{")
        assert len(map_manager._test_responses) > 0
        resp = map_manager._test_responses[-1]
        assert resp["success"] is False

    def test_on_map_cloud_stores_finite_points(self, map_manager):
        """map_cloud is filtered to finite XYZ points."""
        import numpy as np

        cloud = PointCloud2.from_numpy(
            np.array([[1.0, 2.0, 0.1], [3.0, 4.0, 0.2], [np.inf, np.nan, 0.3]], dtype=np.float32)
        )
        map_manager._on_map_cloud(cloud)
        with map_manager._map_cloud_lock:
            stored = map_manager._latest_map_points
        assert stored is not None
        assert stored.shape == (2, 3)

    def test_get_active_tomogram_none(self, map_manager):
        """get_active_tomogram returns None when no active map."""
        result = map_manager.get_active_tomogram()
        assert result is None

    def test_map_list_reports_has_occupancy(self, map_manager):
        """_map_list reports has_occupancy when occupancy.npz exists."""
        d = Path(map_manager._map_dir) / "mapwithall"
        d.mkdir()
        (d / "map.pcd").touch()
        (d / "tomogram.pickle").touch()
        (d / "occupancy.npz").touch()

        resp = _cmd(map_manager, {"action": "list"})
        entry = next(m for m in resp["maps"] if m["name"] == "mapwithall")
        assert entry["has_occupancy"] is True
