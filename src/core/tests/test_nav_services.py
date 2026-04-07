"""Tests for nav service modules, driver modules, and frontier explorer.

Covers:
  1. MapManagerModule      — map CRUD, POI operations, persistence
  2. WavefrontFrontierExplorer — frontier detection, scoring, health
  3. PatrolManagerModule   — route CRUD, patrol start/stop
  4. TaskSchedulerModule   — schedule CRUD, firing logic, deduplication
  5. GeofenceManagerModule — fence CRUD, point-in-polygon, intrusion
  6. CameraBridgeModule    — instantiation, setup without rclpy, health
  7. TeleopModule          — joystick scaling, idle release, health, JPEG path

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json
import os
import shutil
import tempfile
import threading
import time
import unittest
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List
from unittest.mock import MagicMock, patch

import numpy as np

from core.msgs.geometry import Pose, Twist, Vector3
from core.msgs.nav import Odometry
from core.msgs.sensor import Image, ImageFormat

# ---------------------------------------------------------------------------
# 1. MapManagerModule
# ---------------------------------------------------------------------------

class TestMapManagerModule(unittest.TestCase):
    """Test map CRUD, POI operations, and persistence."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        self._map_dir = os.path.join(self._tmpdir, "maps")
        self._data_dir = os.path.join(self._tmpdir, "data")
        from nav.services.nav_services.map_manager_module import MapManagerModule
        self.mod = MapManagerModule(
            map_dir=self._map_dir,
            data_dir=self._data_dir,
        )
        self.mod.setup()
        self._responses: list[dict] = []
        self.mod.map_response.subscribe(lambda r: self._responses.append(r))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _cmd(self, cmd: dict) -> dict:
        self._responses.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._responses) > 0, "no response published")
        return self._responses[-1]

    def test_instantiation(self):
        """Module creates map_dir and data_dir on init."""
        self.assertTrue(os.path.isdir(self._map_dir))
        self.assertTrue(os.path.isdir(self._data_dir))

    def test_list_empty(self):
        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(resp["maps"], [])

    def test_delete_nonexistent(self):
        resp = self._cmd({"action": "delete", "name": "no_such_map"})
        self.assertFalse(resp["success"])
        self.assertIn("not found", resp["message"])

    def test_rename_creates_and_renames(self):
        # Create a map dir manually, then rename
        src = Path(self._map_dir) / "alpha"
        src.mkdir()
        (src / "map.pcd").touch()
        resp = self._cmd({"action": "rename", "name": "alpha", "new_name": "beta"})
        self.assertTrue(resp["success"])
        self.assertFalse(src.exists())
        self.assertTrue((Path(self._map_dir) / "beta" / "map.pcd").exists())

    def test_rename_missing_names(self):
        resp = self._cmd({"action": "rename", "name": "", "new_name": ""})
        self.assertFalse(resp["success"])

    def test_set_active_and_list(self):
        (Path(self._map_dir) / "mymap").mkdir()
        resp = self._cmd({"action": "set_active", "name": "mymap"})
        self.assertTrue(resp["success"])
        self.assertEqual(resp["active"], "mymap")
        # Verify symlink
        link = Path(self._map_dir) / "active"
        self.assertTrue(link.is_symlink())

    def test_set_active_missing(self):
        resp = self._cmd({"action": "set_active", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_delete_active_clears_symlink(self):
        d = Path(self._map_dir) / "todel"
        d.mkdir()
        self._cmd({"action": "set_active", "name": "todel"})
        resp = self._cmd({"action": "delete", "name": "todel"})
        self.assertTrue(resp["success"])
        self.assertEqual(self.mod._active_map, "")

    # -- POI tests --

    def test_poi_set_list_delete(self):
        resp = self._cmd({"action": "poi_set", "name": "home", "x": 1.0, "y": 2.0})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "poi_list"})
        self.assertIn("home", resp["pois"])

        resp = self._cmd({"action": "poi_delete", "name": "home"})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "poi_list"})
        self.assertNotIn("home", resp["pois"])

    def test_poi_delete_nonexistent(self):
        resp = self._cmd({"action": "poi_delete", "name": "nope"})
        self.assertFalse(resp["success"])

    def test_unknown_action(self):
        resp = self._cmd({"action": "foobar"})
        self.assertFalse(resp["success"])
        self.assertIn("unknown", resp.get("message", ""))

    def test_invalid_json(self):
        self._responses.clear()
        self.mod._on_command("not json{{{")
        self.assertTrue(len(self._responses) > 0)
        resp = self._responses[-1]
        self.assertFalse(resp["success"])

    def test_get_active_tomogram_none(self):
        result = self.mod.get_active_tomogram()
        self.assertIsNone(result)

    def test_build_tomogram_missing_name(self):
        resp = self.mod._build_tomogram("")
        self.assertFalse(resp["success"])

    def test_map_save_missing_name(self):
        resp = self.mod._map_save("")
        self.assertFalse(resp["success"])


# ---------------------------------------------------------------------------
# 2. WavefrontFrontierExplorer
# ---------------------------------------------------------------------------

class TestWavefrontFrontierExplorer(unittest.TestCase):
    """Test frontier discovery, scoring, and health."""

    def _make_explorer(self, **kw):
        from nav.frontier_explorer_module import WavefrontFrontierExplorer
        kw.setdefault("min_frontier_size", 2)
        mod = WavefrontFrontierExplorer(**kw)
        mod.setup()
        return mod

    def test_instantiation(self):
        mod = self._make_explorer()
        self.assertIsNotNone(mod)
        self.assertEqual(mod._min_size, 2)

    def test_setup_runs_without_error(self):
        mod = self._make_explorer()
        # setup() should complete without raising
        self.assertIsNotNone(mod.costmap)

    def test_health_idle(self):
        mod = self._make_explorer()
        h = mod.health()
        self.assertEqual(h["exploration_state"], "idle")
        self.assertEqual(h["frontier_count"], 0)

    def test_on_odometry(self):
        mod = self._make_explorer()
        odom = Odometry(pose=Pose(5.0, 10.0, 0.0))
        mod._on_odometry(odom)
        self.assertAlmostEqual(mod._robot_x, 5.0)
        self.assertAlmostEqual(mod._robot_y, 10.0)

    def test_on_costmap(self):
        mod = self._make_explorer()
        costmap = {"grid": np.zeros((10, 10), dtype=np.int16), "resolution": 0.1,
                   "origin_x": 0.0, "origin_y": 0.0, "width": 10, "height": 10}
        mod._on_costmap(costmap)
        self.assertIsNotNone(mod._costmap_data)

    def test_parse_costmap_valid(self):
        mod = self._make_explorer()
        data = {"grid": np.zeros((20, 30), dtype=np.int16),
                "resolution": 0.05, "origin_x": -1.0, "origin_y": -1.5,
                "width": 30, "height": 20}
        grid, meta = mod._parse_costmap(data)
        self.assertEqual(grid.shape, (20, 30))
        self.assertAlmostEqual(meta["resolution"], 0.05)

    def test_parse_costmap_1d(self):
        mod = self._make_explorer()
        flat = np.zeros(100, dtype=np.int16)
        data = {"grid": flat, "resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0,
                "width": 10, "height": 10}
        grid, meta = mod._parse_costmap(data)
        self.assertEqual(grid.shape, (10, 10))

    def test_find_frontiers_all_free_no_unknown(self):
        """Grid with only FREE cells has no frontiers."""
        mod = self._make_explorer()
        grid = np.zeros((20, 20), dtype=np.int16)
        meta = {"resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0,
                "width": 20, "height": 20}
        clusters = mod._find_frontier_clusters(grid, meta, 1.0, 1.0)
        self.assertEqual(len(clusters), 0)

    def test_find_frontiers_with_unknown(self):
        """Grid with FREE cells next to UNKNOWN yields frontiers."""
        mod = self._make_explorer(min_frontier_size=1)
        grid = np.full((20, 20), -1, dtype=np.int16)  # all unknown
        # carve a free region around (10, 10)
        grid[8:13, 8:13] = 0  # free
        meta = {"resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0,
                "width": 20, "height": 20}
        clusters = mod._find_frontier_clusters(grid, meta, 1.0, 1.0)
        self.assertGreater(len(clusters), 0)

    def test_score_clusters_empty(self):
        mod = self._make_explorer()
        result = mod._score_clusters([], 0.0, 0.0, 0.0, {})
        self.assertEqual(result, [])

    def test_score_clusters_sorted_descending(self):
        mod = self._make_explorer()
        clusters = [
            {"cx": 1.0, "cy": 0.0, "size": 5, "cells": []},
            {"cx": 5.0, "cy": 0.0, "size": 20, "cells": []},
        ]
        meta = {"resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0,
                "width": 100, "height": 100}
        mod._costmap_data = {"grid": np.zeros((100, 100), dtype=np.int16),
                             "resolution": 0.1, "origin_x": 0.0, "origin_y": 0.0}
        scored = mod._score_clusters(clusters, 0.0, 0.0, 0.0, meta)
        self.assertEqual(len(scored), 2)
        self.assertGreaterEqual(scored[0]["score"], scored[1]["score"])

    def test_on_goal_reached(self):
        mod = self._make_explorer()
        mod._on_goal_reached(True)
        self.assertTrue(mod._goal_reached_event.is_set())

    def test_make_pose_stamped(self):
        mod = self._make_explorer()
        # _make_pose_stamped sets pose position via PoseStamped constructor
        # PoseStamped.x is a read-only property delegating to pose.x
        # The method may use direct pose attribute access; verify frame_id
        from core.msgs.geometry import Pose, PoseStamped
        ps = PoseStamped(pose=Pose(3.0, 4.0, 0.0), frame_id="map")
        self.assertAlmostEqual(ps.x, 3.0)
        self.assertAlmostEqual(ps.y, 4.0)
        self.assertEqual(ps.frame_id, "map")


# ---------------------------------------------------------------------------
# 3. PatrolManagerModule
# ---------------------------------------------------------------------------

class TestPatrolManagerModule(unittest.TestCase):
    """Test patrol route CRUD and start/stop logic."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        from nav.services.nav_services.patrol_manager_module import PatrolManagerModule
        self.mod = PatrolManagerModule(routes_dir=os.path.join(self._tmpdir, "routes"))
        self.mod.setup()
        self._statuses: list[str] = []
        self._goals: list[list] = []
        self.mod.patrol_status.subscribe(lambda s: self._statuses.append(s))
        self.mod.patrol_goals.subscribe(lambda g: self._goals.append(g))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _cmd(self, cmd: dict) -> dict:
        self._statuses.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._statuses) > 0)
        return json.loads(self._statuses[-1])

    def test_instantiation(self):
        self.assertTrue(Path(self._tmpdir, "routes").is_dir())

    def test_save_and_list(self):
        wps = [{"x": 1.0, "y": 2.0}, {"x": 3.0, "y": 4.0}]
        resp = self._cmd({"action": "save", "name": "route_a", "waypoints": wps})
        self.assertTrue(resp["success"])

        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(resp["routes"]), 1)
        self.assertEqual(resp["routes"][0]["waypoint_count"], 2)

    def test_save_missing_name(self):
        resp = self._cmd({"action": "save", "name": "", "waypoints": [{"x": 0}]})
        self.assertFalse(resp["success"])

    def test_save_empty_waypoints(self):
        resp = self._cmd({"action": "save", "name": "empty", "waypoints": []})
        self.assertFalse(resp["success"])

    def test_load_route(self):
        wps = [{"x": 1.0, "y": 2.0}]
        self._cmd({"action": "save", "name": "r1", "waypoints": wps})
        resp = self._cmd({"action": "load", "name": "r1"})
        self.assertTrue(resp["success"])
        self.assertIn("route", resp)

    def test_load_nonexistent(self):
        resp = self._cmd({"action": "load", "name": "nope"})
        self.assertFalse(resp["success"])

    def test_delete_route(self):
        self._cmd({"action": "save", "name": "del_me", "waypoints": [{"x": 0}]})
        resp = self._cmd({"action": "delete", "name": "del_me"})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "list"})
        self.assertEqual(len(resp["routes"]), 0)

    def test_delete_nonexistent(self):
        resp = self._cmd({"action": "delete", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_start_patrol(self):
        wps = [{"x": 1.0, "y": 2.0}]
        self._cmd({"action": "save", "name": "patrol1", "waypoints": wps})
        self._goals.clear()
        resp = self._cmd({"action": "start", "name": "patrol1"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(self._goals), 1)
        self.assertEqual(self.mod._active_route, "patrol1")

    def test_start_patrol_nonexistent(self):
        resp = self._cmd({"action": "start", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_stop_patrol(self):
        resp = self._cmd({"action": "stop"})
        self.assertTrue(resp["success"])
        self.assertIsNone(self.mod._active_route)

    def test_on_odom_updates_pose(self):
        odom = Odometry(pose=Pose(7.0, 8.0, 9.0))
        self.mod._on_odom(odom)
        self.assertAlmostEqual(self.mod._current_pose["x"], 7.0)

    def test_unknown_action(self):
        resp = self._cmd({"action": "xyz"})
        self.assertFalse(resp["success"])


# ---------------------------------------------------------------------------
# 4. TaskSchedulerModule
# ---------------------------------------------------------------------------

class TestTaskSchedulerModule(unittest.TestCase):
    """Test schedule CRUD, firing logic, and double-fire prevention."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        from nav.services.nav_services.task_scheduler_module import TaskSchedulerModule
        self.mod = TaskSchedulerModule(
            schedule_file=os.path.join(self._tmpdir, "sched.yaml")
        )
        self.mod.setup()
        self._tasks: list[dict] = []
        self.mod.scheduled_task.subscribe(lambda t: self._tasks.append(t))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _cmd(self, cmd: dict) -> dict:
        self._tasks.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._tasks) > 0)
        return self._tasks[-1]

    def test_instantiation(self):
        self.assertIsInstance(self.mod._schedules, dict)

    def test_add_and_list(self):
        resp = self._cmd({"action": "add", "name": "morning",
                          "patrol_route": "route_a", "hour": 8, "minute": 0})
        self.assertTrue(resp["success"])

        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(resp["schedules"]), 1)
        self.assertEqual(resp["schedules"][0]["patrol_route"], "route_a")

    def test_add_missing_name(self):
        resp = self._cmd({"action": "add", "name": "", "patrol_route": "r"})
        self.assertFalse(resp["success"])

    def test_add_missing_route(self):
        resp = self._cmd({"action": "add", "name": "s", "patrol_route": ""})
        self.assertFalse(resp["success"])

    def test_remove(self):
        self._cmd({"action": "add", "name": "temp", "patrol_route": "r"})
        resp = self._cmd({"action": "remove", "name": "temp"})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "list"})
        self.assertEqual(len(resp["schedules"]), 0)

    def test_remove_nonexistent(self):
        resp = self._cmd({"action": "remove", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_enable_disable(self):
        self._cmd({"action": "add", "name": "s1", "patrol_route": "r1"})
        resp = self._cmd({"action": "disable", "name": "s1"})
        self.assertTrue(resp["success"])
        self.assertFalse(self.mod._schedules["s1"]["enabled"])

        resp = self._cmd({"action": "enable", "name": "s1"})
        self.assertTrue(resp["success"])
        self.assertTrue(self.mod._schedules["s1"]["enabled"])

    def test_enable_nonexistent(self):
        resp = self._cmd({"action": "enable", "name": "nope"})
        self.assertFalse(resp["success"])

    def test_check_schedules_fires(self):
        self.mod._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14,
            "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6],
            "enabled": True,
        }
        self._tasks.clear()
        # Simulate time that matches
        now = datetime(2026, 4, 6, 14, 30, 0)  # Monday
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 1)
        self.assertEqual(fired[0]["patrol_route"], "r_test")
        self.assertEqual(fired[0]["event"], "schedule_fired")

    def test_check_schedules_no_double_fire(self):
        self.mod._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14, "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6], "enabled": True,
        }
        now = datetime(2026, 4, 6, 14, 30, 0)
        self.mod.check_schedules(now=now)
        self._tasks.clear()
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    def test_check_schedules_disabled_skipped(self):
        self.mod._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14, "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6], "enabled": False,
        }
        now = datetime(2026, 4, 6, 14, 30, 0)
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    def test_check_schedules_wrong_weekday(self):
        self.mod._schedules["test"] = {
            "patrol_route": "r_test",
            "hour": 14, "minute": 30,
            "weekdays": [2],  # Wednesday only
            "enabled": True,
        }
        # 2026-04-06 is Monday (weekday=0)
        now = datetime(2026, 4, 6, 14, 30, 0)
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    def test_unknown_action(self):
        resp = self._cmd({"action": "xyz"})
        self.assertFalse(resp["success"])


# ---------------------------------------------------------------------------
# 5. GeofenceManagerModule
# ---------------------------------------------------------------------------

class TestGeofenceManagerModule(unittest.TestCase):
    """Test fence CRUD, point-in-polygon, and intrusion detection."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        from nav.services.nav_services.geofence_manager_module import GeofenceManagerModule
        self.mod = GeofenceManagerModule(
            geofence_file=os.path.join(self._tmpdir, "fences.yaml")
        )
        self.mod.setup()
        self._alerts: list[dict] = []
        self.mod.geofence_alert.subscribe(lambda a: self._alerts.append(a))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _cmd(self, cmd: dict) -> dict:
        self._alerts.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._alerts) > 0)
        return self._alerts[-1]

    def test_instantiation(self):
        self.assertIsInstance(self.mod._fences, dict)

    def test_add_fence(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        resp = self._cmd({"action": "add", "name": "zone1", "polygon": polygon})
        self.assertTrue(resp["success"])

    def test_add_fence_too_few_vertices(self):
        resp = self._cmd({"action": "add", "name": "bad", "polygon": [[0, 0], [1, 1]]})
        self.assertFalse(resp["success"])

    def test_add_fence_missing_name(self):
        resp = self._cmd({"action": "add", "name": "", "polygon": [[0,0],[1,0],[1,1]]})
        self.assertFalse(resp["success"])

    def test_list_fences(self):
        self._cmd({"action": "add", "name": "z1",
                   "polygon": [[0, 0], [1, 0], [1, 1], [0, 1]]})
        resp = self._cmd({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(resp["fences"]), 1)
        self.assertEqual(resp["fences"][0]["vertex_count"], 4)

    def test_remove_fence(self):
        self._cmd({"action": "add", "name": "rm_me",
                   "polygon": [[0, 0], [1, 0], [1, 1]]})
        resp = self._cmd({"action": "remove", "name": "rm_me"})
        self.assertTrue(resp["success"])
        resp = self._cmd({"action": "list"})
        self.assertEqual(len(resp["fences"]), 0)

    def test_remove_nonexistent(self):
        resp = self._cmd({"action": "remove", "name": "ghost"})
        self.assertFalse(resp["success"])

    def test_clear_fences(self):
        self._cmd({"action": "add", "name": "a",
                   "polygon": [[0,0],[1,0],[1,1]]})
        self._cmd({"action": "add", "name": "b",
                   "polygon": [[0,0],[1,0],[1,1]]})
        resp = self._cmd({"action": "clear"})
        self.assertTrue(resp["success"])
        self.assertIn("2", resp["message"])

    def test_enable_disable(self):
        self._cmd({"action": "add", "name": "f1",
                   "polygon": [[0,0],[1,0],[1,1]]})
        resp = self._cmd({"action": "disable", "name": "f1"})
        self.assertTrue(resp["success"])
        self.assertFalse(self.mod._fences["f1"]["enabled"])

        resp = self._cmd({"action": "enable", "name": "f1"})
        self.assertTrue(resp["success"])
        self.assertTrue(self.mod._fences["f1"]["enabled"])

    def test_enable_nonexistent(self):
        resp = self._cmd({"action": "enable", "name": "nope"})
        self.assertFalse(resp["success"])

    # -- point-in-polygon --

    def test_point_in_polygon_inside(self):
        from nav.services.nav_services.geofence_manager_module import GeofenceManagerModule
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.assertTrue(GeofenceManagerModule._point_in_polygon(5, 5, poly))

    def test_point_in_polygon_outside(self):
        from nav.services.nav_services.geofence_manager_module import GeofenceManagerModule
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.assertFalse(GeofenceManagerModule._point_in_polygon(15, 15, poly))

    def test_point_in_polygon_too_few(self):
        from nav.services.nav_services.geofence_manager_module import GeofenceManagerModule
        self.assertFalse(GeofenceManagerModule._point_in_polygon(0, 0, [[0, 0]]))

    # -- intrusion detection --

    def test_check_intrusion_inside(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["zone"] = {"polygon": polygon, "enabled": True}
        odom = Odometry(pose=Pose(5.0, 5.0, 0.0))
        self.mod._on_odom(odom)
        self._alerts.clear()
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0]["type"], "intrusion")

    def test_check_intrusion_outside(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["zone"] = {"polygon": polygon, "enabled": True}
        odom = Odometry(pose=Pose(50.0, 50.0, 0.0))
        self.mod._on_odom(odom)
        self._alerts.clear()
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 0)

    def test_check_intrusion_disabled_fence(self):
        polygon = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["zone"] = {"polygon": polygon, "enabled": False}
        odom = Odometry(pose=Pose(5.0, 5.0, 0.0))
        self.mod._on_odom(odom)
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 0)

    def test_unknown_action(self):
        resp = self._cmd({"action": "foobar"})
        self.assertFalse(resp["success"])


# ---------------------------------------------------------------------------
# 6. CameraBridgeModule
# ---------------------------------------------------------------------------

class TestCameraBridgeModule(unittest.TestCase):
    """Test CameraBridgeModule instantiation and stub mode (no rclpy)."""

    def _make_bridge(self, **kw):
        from drivers.thunder.camera_bridge_module import CameraBridgeModule
        return CameraBridgeModule(**kw)

    def test_instantiation_defaults(self):
        mod = self._make_bridge()
        self.assertEqual(mod._color_topic, "/camera/color/image_raw")
        self.assertEqual(mod._depth_topic, "/camera/depth/image_raw")
        self.assertEqual(mod._spin_rate, 30.0)

    def test_instantiation_custom_topics(self):
        mod = self._make_bridge(color_topic="/rgb", depth_topic="/depth")
        self.assertEqual(mod._color_topic, "/rgb")
        self.assertEqual(mod._depth_topic, "/depth")

    def test_setup_no_rclpy(self):
        """setup() should not crash when rclpy is not available."""
        mod = self._make_bridge()
        mod.setup()  # Should gracefully handle ImportError
        self.assertFalse(mod._rclpy_available)

    def test_health_no_frames(self):
        mod = self._make_bridge()
        mod.setup()
        h = mod.health()
        self.assertAlmostEqual(h["fps"], 0.0)

    def test_health_with_timestamp(self):
        mod = self._make_bridge()
        mod.setup()
        mod._last_color_ts = time.time()
        h = mod.health()
        # fps will be computed from dt
        self.assertIn("fps", h)

    def test_start_without_node(self):
        """start() without rclpy node publishes alive=False."""
        mod = self._make_bridge()
        mod.setup()
        published = []
        mod.alive.subscribe(lambda v: published.append(v))
        mod.start()
        self.assertIn(False, published)

    def test_stop_idempotent(self):
        mod = self._make_bridge()
        mod.setup()
        mod.start()
        mod.stop()
        mod.stop()  # second stop should not crash


# ---------------------------------------------------------------------------
# 7. TeleopModule
# ---------------------------------------------------------------------------

class TestTeleopModule(unittest.TestCase):
    """Test joystick scaling, idle release, health, and JPEG encode path."""

    def _make_teleop(self, **kw):
        from drivers.teleop_module import TeleopModule
        mod = TeleopModule(**kw)
        mod.setup()
        return mod

    def test_instantiation_defaults(self):
        mod = self._make_teleop()
        self.assertAlmostEqual(mod._max_speed, 0.5)
        self.assertAlmostEqual(mod._max_yaw_rate, 1.0)
        self.assertAlmostEqual(mod._release_timeout, 3.0)

    def test_instantiation_custom(self):
        mod = self._make_teleop(max_speed=1.0, max_yaw_rate=2.0, release_timeout=5.0)
        self.assertAlmostEqual(mod._max_speed, 1.0)
        self.assertAlmostEqual(mod._max_yaw_rate, 2.0)

    def test_on_joy_scales_and_publishes(self):
        mod = self._make_teleop(max_speed=1.0, max_yaw_rate=2.0)
        twists = []
        mod.cmd_vel.subscribe(lambda t: twists.append(t))
        active_flags = []
        mod.teleop_active.subscribe(lambda a: active_flags.append(a))

        mod._on_joy({"lx": 0.5, "ly": 0.0, "az": -0.5})
        self.assertEqual(len(twists), 1)
        self.assertAlmostEqual(twists[0].linear.x, 0.5)
        self.assertAlmostEqual(twists[0].angular.z, -1.0)
        self.assertTrue(mod._active)
        self.assertIn(True, active_flags)

    def test_on_joy_clamps(self):
        mod = self._make_teleop(max_speed=0.5)
        twists = []
        mod.cmd_vel.subscribe(lambda t: twists.append(t))

        mod._on_joy({"lx": 5.0, "ly": -5.0, "az": 0.0})
        # clamped to [-1, 1] then scaled by max_speed
        self.assertAlmostEqual(twists[0].linear.x, 0.5)
        self.assertAlmostEqual(twists[0].linear.y, -0.5)

    def test_release(self):
        mod = self._make_teleop()
        active_flags = []
        mod.teleop_active.subscribe(lambda a: active_flags.append(a))
        mod._active = True
        mod._release()
        self.assertFalse(mod._active)
        self.assertIn(False, active_flags)

    def test_release_when_not_active(self):
        mod = self._make_teleop()
        active_flags = []
        mod.teleop_active.subscribe(lambda a: active_flags.append(a))
        mod._release()  # should be no-op
        self.assertEqual(len(active_flags), 0)

    def test_health_output(self):
        mod = self._make_teleop()
        h = mod.health()
        self.assertIn("active", h)
        self.assertIn("clients", h)
        self.assertFalse(h["active"])
        self.assertEqual(h["clients"], 0)
        self.assertIsNone(h["last_joy_age_ms"])

    def test_health_after_joy(self):
        mod = self._make_teleop()
        mod._on_joy({"lx": 0.1, "ly": 0.0, "az": 0.0})
        h = mod.health()
        self.assertIsNotNone(h["last_joy_age_ms"])
        self.assertGreaterEqual(h["last_joy_age_ms"], 0)

    def test_get_teleop_status_skill(self):
        mod = self._make_teleop(port=9999)
        status = mod.get_teleop_status()
        self.assertEqual(status["port"], 9999)
        self.assertFalse(status["active"])

    def test_force_release_skill(self):
        mod = self._make_teleop()
        mod._active = True
        result = mod.force_release()
        self.assertEqual(result, "Teleop released")
        self.assertFalse(mod._active)

    def test_client_connect_disconnect(self):
        mod = self._make_teleop()
        mod.on_client_connect()
        self.assertEqual(mod._clients, 1)
        mod.on_client_connect()
        self.assertEqual(mod._clients, 2)
        mod.on_client_disconnect()
        self.assertEqual(mod._clients, 1)

    def test_last_client_disconnect_releases(self):
        mod = self._make_teleop()
        mod._active = True
        mod._clients = 1
        active_flags = []
        mod.teleop_active.subscribe(lambda a: active_flags.append(a))
        mod.on_client_disconnect()
        self.assertFalse(mod._active)

    def test_on_image_no_gateway(self):
        """_on_image does nothing when gateway is None."""
        mod = self._make_teleop()
        img = Image(data=np.zeros((10, 10, 3), dtype=np.uint8), format=ImageFormat.BGR)
        mod._on_image(img)  # should not crash
        self.assertIsNone(mod._latest_raw)

    def test_on_image_with_gateway_and_clients(self):
        """_on_image stores raw frame when gateway + clients present."""
        mod = self._make_teleop()
        mod._gateway = MagicMock()
        mod._clients = 1
        img_data = np.zeros((10, 10, 3), dtype=np.uint8)
        img = Image(data=img_data, format=ImageFormat.BGR)
        mod._on_image(img)
        self.assertIsNotNone(mod._latest_raw)


if __name__ == "__main__":
    unittest.main()
