"""Tests for nav service modules and driver modules.

MapManagerModule, FrontierExplorerModule, PatrolManagerModule,
TaskSchedulerModule, GeofenceManagerModule, CameraBridgeModule, CmdVelMux.

All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import json
import os
import shutil
import tempfile
import time
import unittest
from datetime import datetime
from pathlib import Path
from unittest.mock import MagicMock, patch

import numpy as np

from core.msgs.geometry import Twist, Vector3
from core.msgs.nav import Odometry


# ---------------------------------------------------------------------------
# 1. MapManagerModule
# ---------------------------------------------------------------------------

from nav.services.nav_services.map_manager_module import MapManagerModule


class TestMapManagerModule(unittest.TestCase):
    """Tests for MapManagerModule — map CRUD, active map, POI, health."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        self._map_dir = os.path.join(self._tmpdir, "maps")
        self._data_dir = os.path.join(self._tmpdir, "data")
        self.mod = MapManagerModule(
            map_dir=self._map_dir, data_dir=self._data_dir,
        )
        self.mod.setup()
        self._responses = []
        self.mod.map_response.subscribe(lambda r: self._responses.append(r))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _send(self, cmd: dict) -> dict:
        self._responses.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._responses) > 0, "no response published")
        return self._responses[-1]

    # -- map list --

    def test_list_empty(self):
        """List maps on empty directory returns empty list."""
        resp = self._send({"action": "list"})
        self.assertTrue(resp["success"])
        self.assertEqual(resp["maps"], [])

    def test_list_maps_shows_directories(self):
        """Map directories appear in list result."""
        (Path(self._map_dir) / "warehouse").mkdir(parents=True)
        (Path(self._map_dir) / "warehouse" / "map.pcd").touch()
        resp = self._send({"action": "list"})
        self.assertTrue(resp["success"])
        names = [m["name"] for m in resp["maps"]]
        self.assertIn("warehouse", names)
        entry = [m for m in resp["maps"] if m["name"] == "warehouse"][0]
        self.assertTrue(entry["has_pcd"])
        self.assertFalse(entry["has_tomogram"])

    # -- set_active --

    def test_set_active_creates_symlink(self):
        """set_active creates an 'active' symlink and records the name."""
        (Path(self._map_dir) / "building").mkdir(parents=True)
        resp = self._send({"action": "set_active", "name": "building"})
        self.assertTrue(resp["success"])
        self.assertEqual(resp["active"], "building")
        self.assertTrue((Path(self._map_dir) / "active").is_symlink())

    def test_set_active_missing_map(self):
        """set_active on non-existent map fails gracefully."""
        resp = self._send({"action": "set_active", "name": "no_such"})
        self.assertFalse(resp["success"])

    # -- delete --

    def test_delete_map(self):
        """Delete removes map directory."""
        (Path(self._map_dir) / "tmp_map").mkdir(parents=True)
        resp = self._send({"action": "delete", "name": "tmp_map"})
        self.assertTrue(resp["success"])
        self.assertFalse((Path(self._map_dir) / "tmp_map").exists())

    def test_delete_active_map_clears_active(self):
        """Deleting the active map clears the active symlink."""
        (Path(self._map_dir) / "active_map").mkdir(parents=True)
        self._send({"action": "set_active", "name": "active_map"})
        resp = self._send({"action": "delete", "name": "active_map"})
        self.assertTrue(resp["success"])
        self.assertEqual(self.mod._active_map, "")

    # -- rename --

    def test_rename_map(self):
        """Rename moves directory and updates active if needed."""
        (Path(self._map_dir) / "old_name").mkdir(parents=True)
        resp = self._send({"action": "rename", "name": "old_name", "new_name": "new_name"})
        self.assertTrue(resp["success"])
        self.assertFalse((Path(self._map_dir) / "old_name").exists())
        self.assertTrue((Path(self._map_dir) / "new_name").exists())

    # -- POI --

    def test_poi_crud(self):
        """POI set, list, delete cycle works."""
        self._send({"action": "poi_set", "name": "dock", "x": 1.0, "y": 2.0})
        resp = self._send({"action": "poi_list"})
        self.assertTrue(resp["success"])
        self.assertIn("dock", resp["pois"])

        resp = self._send({"action": "poi_delete", "name": "dock"})
        self.assertTrue(resp["success"])

        resp = self._send({"action": "poi_list"})
        self.assertNotIn("dock", resp["pois"])

    # -- save (mocked subprocess) --

    def test_save_map_no_ros2(self):
        """Save map fails gracefully when ROS2 not available."""
        resp = self._send({"action": "save", "name": "test_save"})
        self.assertFalse(resp["success"])
        self.assertIn("ROS2", resp.get("message", ""))

    # -- port_summary (health proxy) --

    def test_port_summary(self):
        """port_summary returns module info dict."""
        info = self.mod.port_summary()
        self.assertEqual(info["module"], "MapManagerModule")

    # -- unknown action --

    def test_unknown_action(self):
        """Unknown action returns failure."""
        resp = self._send({"action": "bogus"})
        self.assertFalse(resp["success"])


# ---------------------------------------------------------------------------
# 2. WavefrontFrontierExplorer (FrontierExplorerModule)
# ---------------------------------------------------------------------------

from nav.frontier_explorer_module import WavefrontFrontierExplorer, FREE, OCCUPIED, UNKNOWN


class TestFrontierExplorer(unittest.TestCase):
    """Tests for WavefrontFrontierExplorer — costmap parsing, frontiers, health."""

    def _make_explorer(self, **kw):
        defaults = dict(min_frontier_size=2, explore_rate=1.0)
        defaults.update(kw)
        mod = WavefrontFrontierExplorer(**defaults)
        mod.setup()
        return mod

    def _make_costmap(self, grid, resolution=0.5, origin_x=0.0, origin_y=0.0):
        """Build a costmap dict from a 2D numpy grid."""
        h, w = grid.shape
        return {
            "grid": grid,
            "resolution": resolution,
            "origin_x": origin_x,
            "origin_y": origin_y,
            "width": w,
            "height": h,
        }

    # -- costmap parsing --

    def test_parse_costmap_valid(self):
        """Valid costmap dict is parsed into grid + meta."""
        mod = self._make_explorer()
        grid = np.zeros((10, 10), dtype=np.int16)
        costmap = self._make_costmap(grid)
        parsed_grid, meta = mod._parse_costmap(costmap)
        self.assertIsNotNone(parsed_grid)
        self.assertEqual(meta["width"], 10)
        self.assertEqual(meta["height"], 10)
        self.assertAlmostEqual(meta["resolution"], 0.5)

    def test_parse_costmap_1d_array(self):
        """Flat 1D array is reshaped using height/width."""
        mod = self._make_explorer()
        grid = np.zeros(100, dtype=np.int16)
        costmap = {"grid": grid, "resolution": 0.1, "origin_x": 0, "origin_y": 0,
                    "width": 10, "height": 10}
        parsed_grid, meta = mod._parse_costmap(costmap)
        self.assertIsNotNone(parsed_grid)
        self.assertEqual(parsed_grid.shape, (10, 10))

    def test_parse_costmap_bad_data(self):
        """Malformed costmap returns (None, None)."""
        mod = self._make_explorer()
        g, m = mod._parse_costmap({"grid": "not_an_array"})
        self.assertIsNone(g)

    # -- frontier discovery --

    def test_find_frontiers_simple(self):
        """Free cells adjacent to unknown cells are detected as frontiers."""
        mod = self._make_explorer()
        # 10x10 grid: left half free, right half unknown
        grid = np.full((10, 10), FREE, dtype=np.int16)
        grid[:, 5:] = UNKNOWN
        costmap = self._make_costmap(grid, resolution=1.0)
        mod._costmap_data = costmap

        meta = {"resolution": 1.0, "origin_x": 0.0, "origin_y": 0.0,
                "width": 10, "height": 10}
        clusters = mod._find_frontier_clusters(grid, meta, robot_x=2.0, robot_y=5.0)
        self.assertGreater(len(clusters), 0)
        # All frontier centroids should be near column 4 (boundary)
        for c in clusters:
            self.assertGreater(c["size"], 0)

    def test_no_frontiers_all_free(self):
        """Fully explored (all free) grid yields no frontiers."""
        mod = self._make_explorer()
        grid = np.full((10, 10), FREE, dtype=np.int16)
        meta = {"resolution": 1.0, "origin_x": 0.0, "origin_y": 0.0,
                "width": 10, "height": 10}
        clusters = mod._find_frontier_clusters(grid, meta, robot_x=5.0, robot_y=5.0)
        self.assertEqual(len(clusters), 0)

    def test_robot_outside_grid(self):
        """Robot position outside grid bounds returns empty frontier list."""
        mod = self._make_explorer()
        grid = np.full((10, 10), FREE, dtype=np.int16)
        grid[:, 8:] = UNKNOWN
        meta = {"resolution": 1.0, "origin_x": 0.0, "origin_y": 0.0,
                "width": 10, "height": 10}
        clusters = mod._find_frontier_clusters(grid, meta, robot_x=100.0, robot_y=100.0)
        self.assertEqual(len(clusters), 0)

    # -- health --

    def test_health_idle(self):
        """Health reports idle state when not exploring."""
        mod = self._make_explorer()
        h = mod.health()
        self.assertEqual(h["exploration_state"], "idle")
        self.assertEqual(h["frontier_count"], 0)

    # -- scoring --

    def test_score_clusters_sorted(self):
        """Scored clusters are sorted descending by score."""
        mod = self._make_explorer()
        mod._costmap_data = self._make_costmap(
            np.full((20, 20), FREE, dtype=np.int16), resolution=1.0)
        clusters = [
            {"cx": 3.0, "cy": 3.0, "size": 10, "cells": []},
            {"cx": 8.0, "cy": 8.0, "size": 50, "cells": []},
        ]
        meta = {"resolution": 1.0, "origin_x": 0.0, "origin_y": 0.0,
                "width": 20, "height": 20}
        scored = mod._score_clusters(clusters, 0.0, 0.0, 0.0, meta)
        self.assertEqual(len(scored), 2)
        self.assertGreaterEqual(scored[0]["score"], scored[1]["score"])


# ---------------------------------------------------------------------------
# 3. PatrolManagerModule
# ---------------------------------------------------------------------------

from nav.services.nav_services.patrol_manager_module import PatrolManagerModule


class TestPatrolManagerModule(unittest.TestCase):
    """Tests for PatrolManagerModule — route CRUD, patrol start/stop."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        self._routes_dir = os.path.join(self._tmpdir, "routes")
        self.mod = PatrolManagerModule(routes_dir=self._routes_dir)
        self.mod.setup()
        self._statuses = []
        self.mod.patrol_status.subscribe(lambda s: self._statuses.append(s))
        self._goals = []
        self.mod.patrol_goals.subscribe(lambda g: self._goals.append(g))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _send(self, cmd: dict) -> dict:
        self._statuses.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._statuses) > 0)
        return json.loads(self._statuses[-1])

    # -- save / list / load / delete --

    def test_save_and_list(self):
        """Save a route then verify it appears in list."""
        wps = [{"x": 1, "y": 2}, {"x": 3, "y": 4}]
        resp = self._send({"action": "save", "name": "route_a", "waypoints": wps})
        self.assertTrue(resp["success"])

        resp = self._send({"action": "list"})
        self.assertTrue(resp["success"])
        names = [r["name"] for r in resp["routes"]]
        self.assertIn("route_a", names)

    def test_load_route(self):
        """Load returns full route data."""
        wps = [{"x": 0, "y": 0}]
        self._send({"action": "save", "name": "rte", "waypoints": wps})
        resp = self._send({"action": "load", "name": "rte"})
        self.assertTrue(resp["success"])
        self.assertEqual(resp["route"]["waypoints"], wps)

    def test_load_missing_route(self):
        """Loading non-existent route fails."""
        resp = self._send({"action": "load", "name": "no_such"})
        self.assertFalse(resp["success"])

    def test_delete_route(self):
        """Delete removes route file."""
        wps = [{"x": 0, "y": 0}]
        self._send({"action": "save", "name": "del_me", "waypoints": wps})
        resp = self._send({"action": "delete", "name": "del_me"})
        self.assertTrue(resp["success"])
        resp = self._send({"action": "load", "name": "del_me"})
        self.assertFalse(resp["success"])

    def test_save_empty_waypoints_fails(self):
        """Saving a route with no waypoints fails."""
        resp = self._send({"action": "save", "name": "empty", "waypoints": []})
        self.assertFalse(resp["success"])

    # -- patrol start / stop --

    def test_start_patrol_publishes_goals(self):
        """Starting patrol publishes waypoints to patrol_goals port."""
        wps = [{"x": 1, "y": 2}, {"x": 5, "y": 6}]
        self._send({"action": "save", "name": "go", "waypoints": wps})
        self._goals.clear()
        resp = self._send({"action": "start", "name": "go"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(self._goals), 1)
        self.assertEqual(self._goals[0], wps)

    def test_start_missing_route_fails(self):
        """Starting patrol on non-existent route fails."""
        resp = self._send({"action": "start", "name": "missing"})
        self.assertFalse(resp["success"])

    def test_stop_patrol(self):
        """Stop patrol returns success and clears active route."""
        wps = [{"x": 0, "y": 0}]
        self._send({"action": "save", "name": "r", "waypoints": wps})
        self._send({"action": "start", "name": "r"})
        resp = self._send({"action": "stop"})
        self.assertTrue(resp["success"])
        self.assertIsNone(self.mod._active_route)

    # -- odometry --

    def test_odometry_updates_pose(self):
        """Odometry callback stores current pose."""
        from core.msgs.geometry import Pose
        odom = Odometry(pose=Pose(3.0, 4.0, 0.0))
        self.mod._on_odom(odom)
        self.assertEqual(self.mod._current_pose["x"], 3.0)

    # -- port_summary --

    def test_port_summary(self):
        info = self.mod.port_summary()
        self.assertEqual(info["module"], "PatrolManagerModule")


# ---------------------------------------------------------------------------
# 4. TaskSchedulerModule
# ---------------------------------------------------------------------------

from nav.services.nav_services.task_scheduler_module import TaskSchedulerModule


class TestTaskSchedulerModule(unittest.TestCase):
    """Tests for TaskSchedulerModule — schedule CRUD, time-based firing."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        self._sched_file = os.path.join(self._tmpdir, "schedules.yaml")
        self.mod = TaskSchedulerModule(schedule_file=self._sched_file)
        self.mod.setup()
        self._tasks = []
        self.mod.scheduled_task.subscribe(lambda t: self._tasks.append(t))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _send(self, cmd: dict) -> dict:
        self._tasks.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._tasks) > 0)
        return self._tasks[-1]

    # -- add / list / remove --

    def test_add_and_list(self):
        """Add a schedule then verify it appears in list."""
        resp = self._send({
            "action": "add", "name": "morning",
            "patrol_route": "route_a", "hour": 8, "minute": 0,
        })
        self.assertTrue(resp["success"])

        resp = self._send({"action": "list"})
        self.assertTrue(resp["success"])
        names = [s["name"] for s in resp["schedules"]]
        self.assertIn("morning", names)

    def test_remove_schedule(self):
        """Remove deletes the schedule."""
        self._send({"action": "add", "name": "tmp", "patrol_route": "r"})
        resp = self._send({"action": "remove", "name": "tmp"})
        self.assertTrue(resp["success"])

        resp = self._send({"action": "list"})
        names = [s["name"] for s in resp["schedules"]]
        self.assertNotIn("tmp", names)

    def test_remove_missing_fails(self):
        resp = self._send({"action": "remove", "name": "nope"})
        self.assertFalse(resp["success"])

    # -- enable / disable --

    def test_enable_disable(self):
        """Enable and disable toggles schedule state."""
        self._send({"action": "add", "name": "s1", "patrol_route": "r"})
        resp = self._send({"action": "disable", "name": "s1"})
        self.assertTrue(resp["success"])
        self.assertFalse(self.mod._schedules["s1"]["enabled"])

        resp = self._send({"action": "enable", "name": "s1"})
        self.assertTrue(resp["success"])
        self.assertTrue(self.mod._schedules["s1"]["enabled"])

    # -- check_schedules firing --

    def test_check_schedules_fires_due_task(self):
        """check_schedules fires when weekday/hour/minute match."""
        self.mod._schedules["morning"] = {
            "patrol_route": "lobby",
            "hour": 9,
            "minute": 30,
            "weekdays": [0, 1, 2, 3, 4, 5, 6],
            "enabled": True,
        }
        # Monday 09:30
        now = datetime(2026, 4, 6, 9, 30, 0)  # 2026-04-06 is Monday (weekday 0)
        self._tasks.clear()
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 1)
        self.assertEqual(fired[0]["patrol_route"], "lobby")

    def test_check_schedules_no_double_fire(self):
        """Same minute key does not fire twice."""
        self.mod._schedules["s"] = {
            "patrol_route": "r", "hour": 10, "minute": 0,
            "weekdays": [0, 1, 2, 3, 4, 5, 6], "enabled": True,
        }
        now = datetime(2026, 4, 6, 10, 0, 0)
        self.mod.check_schedules(now=now)
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    def test_check_schedules_disabled_not_fired(self):
        """Disabled schedules are skipped."""
        self.mod._schedules["off"] = {
            "patrol_route": "r", "hour": 10, "minute": 0,
            "weekdays": [0, 1, 2, 3, 4, 5, 6], "enabled": False,
        }
        now = datetime(2026, 4, 6, 10, 0, 0)
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    def test_check_schedules_wrong_weekday(self):
        """Schedule that does not include current weekday is skipped."""
        self.mod._schedules["weekdays_only"] = {
            "patrol_route": "r", "hour": 10, "minute": 0,
            "weekdays": [1, 2, 3, 4],  # Tue-Fri only
            "enabled": True,
        }
        # Monday = weekday 0
        now = datetime(2026, 4, 6, 10, 0, 0)
        fired = self.mod.check_schedules(now=now)
        self.assertEqual(len(fired), 0)

    # -- port_summary --

    def test_port_summary(self):
        info = self.mod.port_summary()
        self.assertEqual(info["module"], "TaskSchedulerModule")


# ---------------------------------------------------------------------------
# 5. GeofenceManagerModule
# ---------------------------------------------------------------------------

from nav.services.nav_services.geofence_manager_module import GeofenceManagerModule


class TestGeofenceManagerModule(unittest.TestCase):
    """Tests for GeofenceManagerModule — fence CRUD, intrusion detection."""

    def setUp(self):
        self._tmpdir = tempfile.mkdtemp()
        self._fence_file = os.path.join(self._tmpdir, "geofences.yaml")
        self.mod = GeofenceManagerModule(geofence_file=self._fence_file)
        self.mod.setup()
        self._alerts = []
        self.mod.geofence_alert.subscribe(lambda a: self._alerts.append(a))

    def tearDown(self):
        shutil.rmtree(self._tmpdir, ignore_errors=True)

    def _send(self, cmd: dict) -> dict:
        self._alerts.clear()
        self.mod._on_command(json.dumps(cmd))
        self.assertTrue(len(self._alerts) > 0)
        return self._alerts[-1]

    # -- add / list / remove / clear --

    def test_add_fence(self):
        """Add a polygon fence."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        resp = self._send({"action": "add", "name": "zone_a", "polygon": poly})
        self.assertTrue(resp["success"])

    def test_add_fence_too_few_vertices(self):
        """Polygon with < 3 vertices is rejected."""
        resp = self._send({"action": "add", "name": "bad", "polygon": [[0, 0], [1, 1]]})
        self.assertFalse(resp["success"])

    def test_list_fences(self):
        """List shows added fences."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self._send({"action": "add", "name": "z", "polygon": poly})
        resp = self._send({"action": "list"})
        self.assertTrue(resp["success"])
        names = [f["name"] for f in resp["fences"]]
        self.assertIn("z", names)

    def test_remove_fence(self):
        """Remove deletes a fence."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self._send({"action": "add", "name": "rm", "polygon": poly})
        resp = self._send({"action": "remove", "name": "rm"})
        self.assertTrue(resp["success"])
        self.assertNotIn("rm", self.mod._fences)

    def test_remove_missing_fence(self):
        resp = self._send({"action": "remove", "name": "nope"})
        self.assertFalse(resp["success"])

    def test_clear_fences(self):
        """Clear removes all fences."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self._send({"action": "add", "name": "a", "polygon": poly})
        self._send({"action": "add", "name": "b", "polygon": poly})
        resp = self._send({"action": "clear"})
        self.assertTrue(resp["success"])
        self.assertEqual(len(self.mod._fences), 0)

    # -- enable / disable --

    def test_enable_disable(self):
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self._send({"action": "add", "name": "f", "polygon": poly})
        resp = self._send({"action": "disable", "name": "f"})
        self.assertTrue(resp["success"])
        self.assertFalse(self.mod._fences["f"]["enabled"])

        resp = self._send({"action": "enable", "name": "f"})
        self.assertTrue(self.mod._fences["f"]["enabled"])

    # -- intrusion detection --

    def test_intrusion_inside_polygon(self):
        """Robot inside a fence polygon triggers intrusion alert."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["restricted"] = {"polygon": poly, "enabled": True}
        self.mod._robot_x = 5.0
        self.mod._robot_y = 5.0
        self._alerts.clear()
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 1)
        self.assertEqual(alerts[0]["fence"], "restricted")

    def test_no_intrusion_outside_polygon(self):
        """Robot outside a fence polygon does not trigger alert."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["restricted"] = {"polygon": poly, "enabled": True}
        self.mod._robot_x = 15.0
        self.mod._robot_y = 15.0
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 0)

    def test_disabled_fence_no_intrusion(self):
        """Disabled fence does not trigger intrusion even if robot is inside."""
        poly = [[0, 0], [10, 0], [10, 10], [0, 10]]
        self.mod._fences["off"] = {"polygon": poly, "enabled": False}
        self.mod._robot_x = 5.0
        self.mod._robot_y = 5.0
        alerts = self.mod.check_intrusion()
        self.assertEqual(len(alerts), 0)

    # -- point_in_polygon edge cases --

    def test_point_in_polygon_on_vertex(self):
        """Point on vertex — implementation-defined but should not crash."""
        result = GeofenceManagerModule._point_in_polygon(0.0, 0.0,
                                                          [[0, 0], [10, 0], [10, 10]])
        self.assertIsInstance(result, bool)

    def test_point_in_polygon_triangle(self):
        """Point clearly inside triangle returns True."""
        inside = GeofenceManagerModule._point_in_polygon(
            3.0, 2.0, [[0, 0], [10, 0], [5, 10]])
        self.assertTrue(inside)

    # -- odometry update --

    def test_odometry_updates_position(self):
        from core.msgs.geometry import Pose
        odom = Odometry(pose=Pose(7.0, 8.0, 0.0))
        self.mod._on_odom(odom)
        self.assertAlmostEqual(self.mod._robot_x, 7.0)
        self.assertAlmostEqual(self.mod._robot_y, 8.0)

    # -- port_summary --

    def test_port_summary(self):
        info = self.mod.port_summary()
        self.assertEqual(info["module"], "GeofenceManagerModule")


# ---------------------------------------------------------------------------
# 6. CameraBridgeModule
# ---------------------------------------------------------------------------

from drivers.thunder.camera_bridge_module import CameraBridgeModule


class TestCameraBridgeModule(unittest.TestCase):
    """Tests for CameraBridgeModule — instantiation, ports, health (no ROS2)."""

    def test_instantiation_without_rclpy(self):
        """Module can be instantiated without rclpy available."""
        mod = CameraBridgeModule()
        self.assertIsNotNone(mod)

    def test_port_declarations(self):
        """Output ports are declared on the class."""
        self.assertTrue(hasattr(CameraBridgeModule, "color_image"))
        self.assertTrue(hasattr(CameraBridgeModule, "depth_image"))
        self.assertTrue(hasattr(CameraBridgeModule, "camera_info"))
        self.assertTrue(hasattr(CameraBridgeModule, "alive"))

    def test_setup_without_rclpy(self):
        """Setup does not crash when rclpy is unavailable."""
        mod = CameraBridgeModule()
        # _create_ros2_node will fail gracefully without rclpy
        mod.setup()
        self.assertFalse(mod._rclpy_available)
        self.assertIsNone(mod._node)

    def test_start_without_node_publishes_alive_false(self):
        """start() publishes alive=False when no ROS2 node exists."""
        mod = CameraBridgeModule()
        mod.setup()
        alive_vals = []
        mod.alive.subscribe(lambda v: alive_vals.append(v))
        mod.start()
        self.assertIn(False, alive_vals)

    def test_health_returns_dict(self):
        """health() returns a dict with fps and frame_count."""
        mod = CameraBridgeModule()
        mod.setup()
        h = mod.health()
        self.assertIsInstance(h, dict)
        self.assertIn("fps", h)
        self.assertEqual(h["fps"], 0.0)
        self.assertEqual(h["frame_count"], 0)

    def test_custom_topics(self):
        """Custom topic names are stored correctly."""
        mod = CameraBridgeModule(
            color_topic="/my/rgb",
            depth_topic="/my/depth",
            info_topic="/my/info",
        )
        self.assertEqual(mod._color_topic, "/my/rgb")
        self.assertEqual(mod._depth_topic, "/my/depth")
        self.assertEqual(mod._info_topic, "/my/info")

    def test_stop_is_safe_without_start(self):
        """stop() does not crash even if start() was never called."""
        mod = CameraBridgeModule()
        mod.setup()
        mod.stop()  # should not raise


# ---------------------------------------------------------------------------
# 7. CmdVelMux — additional health() tests
# ---------------------------------------------------------------------------

from nav.cmd_vel_mux_module import CmdVelMux


def _twist(vx: float = 0.0, wz: float = 0.0) -> Twist:
    return Twist(linear=Vector3(x=vx, y=0.0, z=0.0),
                 angular=Vector3(x=0.0, y=0.0, z=wz))


class TestCmdVelMuxHealth(unittest.TestCase):
    """Additional health() tests for CmdVelMux (complements test_cmd_vel_mux.py)."""

    def _make_mux(self, timeout: float = 1.0) -> CmdVelMux:
        mux = CmdVelMux(source_timeout=timeout)
        mux.setup()
        return mux

    def test_health_no_active_source(self):
        """Health with no active source shows empty active_source."""
        mux = self._make_mux()
        h = mux.health()
        self.assertEqual(h["active_source"], "")
        self.assertIn("sources", h)
        for name in ("teleop", "visual_servo", "recovery", "path_follower"):
            self.assertIn(name, h["sources"])

    def test_health_active_source_shown(self):
        """Health reflects the currently active source."""
        mux = self._make_mux()
        mux._on_source("visual_servo", _twist(0.1, 0.2))
        h = mux.health()
        self.assertEqual(h["active_source"], "visual_servo")
        self.assertTrue(h["sources"]["visual_servo"]["active"])

    def test_health_source_ages(self):
        """Source age_ms increases after source goes idle."""
        mux = self._make_mux(timeout=0.1)
        mux._on_source("path_follower", _twist(0.3))
        time.sleep(0.15)
        h = mux.health()
        age = h["sources"]["path_follower"]["age_ms"]
        self.assertGreater(age, 100)
        self.assertFalse(h["sources"]["path_follower"]["active"])

    def test_health_priorities_reported(self):
        """Health shows priority for each source."""
        mux = self._make_mux()
        h = mux.health()
        self.assertEqual(h["sources"]["teleop"]["priority"], 100)
        self.assertEqual(h["sources"]["visual_servo"]["priority"], 80)
        self.assertEqual(h["sources"]["recovery"]["priority"], 60)
        self.assertEqual(h["sources"]["path_follower"]["priority"], 40)

    def test_health_after_timeout_no_active(self):
        """After all sources time out, active_source goes empty."""
        mux = self._make_mux(timeout=0.05)
        mux._on_source("teleop", _twist(0.5))
        time.sleep(0.1)
        # Trigger select_active via another source event that also expires
        mux._on_source("path_follower", _twist(0.1))
        # path_follower just published so it should be active
        self.assertEqual(mux._active, "path_follower")


if __name__ == "__main__":
    unittest.main()
