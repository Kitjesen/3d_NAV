"""Tests for 7 new unified modules — NavigationModule, SafetyRingModule,
AutonomyModule, GatewayModule, MCPServerModule, RerunModule, SemanticPlannerModule.

All external deps mocked. No GPU/FastAPI/Rerun/C++ required.
"""

import math
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from core import Module, In, Out
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion, Twist
from core.msgs.nav import Odometry, Path
from core.msgs.semantic import SafetyState, SceneGraph


# ============================================================================
# NavigationModule
# ============================================================================

class TestNavigationModule(unittest.TestCase):

    def _make(self, **kw):
        from nav.navigation_module import NavigationModule
        return NavigationModule(planner="astar", **kw)

    def test_ports_in(self):
        m = self._make()
        self.assertIn("goal_pose", m.ports_in)
        self.assertIn("odometry", m.ports_in)
        self.assertIn("instruction", m.ports_in)
        self.assertIn("stop_signal", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("waypoint", m.ports_out)
        self.assertIn("global_path", m.ports_out)
        self.assertIn("planner_status", m.ports_out)
        self.assertIn("mission_status", m.ports_out)

    def test_set_state_publishes(self):
        m = self._make()
        statuses = []
        m.planner_status._add_callback(statuses.append)
        m._set_state("PLANNING")
        self.assertEqual(statuses[-1], "PLANNING")

    def test_on_stop_clears(self):
        m = self._make()
        # Path now lives in the WaypointTracker service
        m._tracker._path = [np.array([1, 2, 0])]
        m._on_stop(2)
        self.assertEqual(m._tracker.path_length, 0)
        self.assertEqual(m._state, "IDLE")

    def test_downsample(self):
        m = self._make(downsample_dist=2.0)
        path = [(0, 0, 0), (0.5, 0, 0), (1, 0, 0), (3, 0, 0), (5, 0, 0)]
        goal = np.array(path[-1])
        # Downsample now lives in GlobalPlannerService
        result = m._planner_svc._downsample(path, goal)
        self.assertLess(len(result), len(path))

    def test_on_odom_updates_pos(self):
        m = self._make()
        odom = Odometry(pose=Pose(position=Vector3(1, 2, 0)), ts=time.time())
        m._on_odom(odom)
        self.assertAlmostEqual(m._robot_pos[0], 1.0)
        self.assertAlmostEqual(m._robot_pos[1], 2.0)

    def test_layer(self):
        m = self._make()
        self.assertEqual(m.layer, 5)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("navigation", h)
        self.assertEqual(h["navigation"]["planner"], "astar")

    def test_no_map_uses_direct_goal_fallback(self):
        m = self._make(enable_ros2_bridge=False)

        class _NoMapBackend:
            _grid = None

            def plan(self, start, goal):
                return []

        m._planner_svc._backend = _NoMapBackend()
        m._robot_pos = np.array([0.0, 0.0, 0.0])
        waypoints = []
        statuses = []
        m.waypoint._add_callback(waypoints.append)
        m.adapter_status._add_callback(statuses.append)

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        ))

        self.assertEqual(m._state, "EXECUTING")
        self.assertEqual(len(waypoints), 1)
        self.assertAlmostEqual(waypoints[0].pose.position.x, 4.0)
        self.assertAlmostEqual(waypoints[0].pose.position.y, 2.0)
        self.assertTrue(any(s.get("event") == "direct_goal_fallback" for s in statuses))

    def test_empty_path_with_map_still_fails(self):
        m = self._make(enable_ros2_bridge=False)

        class _MappedBackend:
            _grid = np.zeros((10, 10), dtype=np.float32)

            def plan(self, start, goal):
                return []

        m._planner_svc._backend = _MappedBackend()
        m._robot_pos = np.array([0.0, 0.0, 0.0])

        m._on_goal(PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        ))

        self.assertEqual(m._state, "FAILED")
        self.assertIn("empty path", m._failure_reason)

    def test_duplicate_goal_update_is_ignored_while_executing(self):
        m = self._make(enable_ros2_bridge=False)

        class _NoMapBackend:
            _grid = None

            def plan(self, start, goal):
                return []

        m._planner_svc._backend = _NoMapBackend()
        m._robot_pos = np.array([0.0, 0.0, 0.0])
        events = []
        m.adapter_status._add_callback(events.append)

        pose = PoseStamped(
            pose=Pose(position=Vector3(4.0, 2.0, 0.0), orientation=Quaternion()),
            frame_id="map", ts=time.time(),
        )
        m._on_goal(pose)
        first_path_len = m._tracker.path_length
        first_state = m._state
        m._on_goal(pose)

        self.assertEqual(first_state, "EXECUTING")
        self.assertEqual(m._tracker.path_length, first_path_len)
        self.assertTrue(any(e.get("event") == "goal_update_ignored" for e in events))


# ============================================================================
# SafetyRingModule
# ============================================================================

class TestSafetyRingModule(unittest.TestCase):

    def _make(self):
        from nav.safety_ring_module import SafetyRingModule
        return SafetyRingModule()

    def test_ports_in(self):
        m = self._make()
        self.assertIn("odometry", m.ports_in)
        self.assertIn("path", m.ports_in)
        self.assertIn("cmd_vel", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("stop_cmd", m.ports_out)
        self.assertIn("safety_state", m.ports_out)
        self.assertIn("execution_eval", m.ports_out)
        self.assertIn("dialogue_state", m.ports_out)

    def test_cross_track_no_path(self):
        m = self._make()
        self.assertAlmostEqual(m._cross_track_error(), 0.0)

    def test_distance_to_goal_no_goal(self):
        m = self._make()
        self.assertEqual(m._distance_to_goal(), float("inf"))

    def test_on_odom_updates_xy(self):
        m = self._make()
        m.setup()
        odom = Odometry(pose=Pose(position=Vector3(3, 4, 0)), ts=time.time())
        m._on_odom(odom)
        self.assertAlmostEqual(m._robot_xy[0], 3.0)

    def test_layer_is_0(self):
        m = self._make()
        self.assertEqual(m.layer, 0)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("safety_ring", h)


# ============================================================================
# TerrainModule / LocalPlannerModule / PathFollowerModule
# ============================================================================

class TestTerrainModule(unittest.TestCase):

    def test_ports(self):
        from base_autonomy.modules.terrain_module import TerrainModule
        m = TerrainModule(backend="simple")
        self.assertIn("odometry", m.ports_in)
        self.assertIn("map_cloud", m.ports_in)
        self.assertIn("terrain_map", m.ports_out)
        self.assertIn("traversability", m.ports_out)
        self.assertIn("alive", m.ports_out)

    def test_layer(self):
        from base_autonomy.modules.terrain_module import TerrainModule
        m = TerrainModule(backend="simple")
        self.assertEqual(m.layer, 2)

    def test_health(self):
        from base_autonomy.modules.terrain_module import TerrainModule
        m = TerrainModule(backend="simple")
        h = m.health()
        self.assertIn("terrain", h)
        self.assertEqual(h["terrain"]["backend"], "simple")


class TestLocalPlannerModule(unittest.TestCase):

    def test_ports(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule
        m = LocalPlannerModule(backend="simple")
        self.assertIn("odometry", m.ports_in)
        self.assertIn("terrain_map", m.ports_in)
        self.assertIn("waypoint", m.ports_in)
        self.assertIn("local_path", m.ports_out)

    def test_layer(self):
        from base_autonomy.modules.local_planner_module import LocalPlannerModule
        m = LocalPlannerModule(backend="simple")
        self.assertEqual(m.layer, 2)


class TestPathFollowerModule(unittest.TestCase):

    def test_ports(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule
        m = PathFollowerModule(backend="pid")
        self.assertIn("odometry", m.ports_in)
        self.assertIn("local_path", m.ports_in)
        self.assertIn("cmd_vel", m.ports_out)

    def test_layer(self):
        from base_autonomy.modules.path_follower_module import PathFollowerModule
        m = PathFollowerModule(backend="pid")
        self.assertEqual(m.layer, 2)


# ============================================================================
# GatewayModule
# ============================================================================

class TestGatewayModule(unittest.TestCase):

    def _make(self):
        from gateway.gateway_module import GatewayModule
        return GatewayModule(port=5050)

    def test_ports_in(self):
        m = self._make()
        self.assertEqual(len(m.ports_in), 6)
        self.assertIn("odometry", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("safety_state", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertEqual(len(m.ports_out), 5)
        self.assertIn("goal_pose", m.ports_out)
        self.assertIn("cmd_vel", m.ports_out)
        self.assertIn("stop_cmd", m.ports_out)

    def test_on_odometry_caches(self):
        m = self._make()
        odom = Odometry(pose=Pose(position=Vector3(1, 2, 0)), ts=time.time())
        m._on_odometry(odom)
        self.assertIsNotNone(m._odom)
        self.assertAlmostEqual(m._odom["x"], 1.0)

    def test_check_lease_open(self):
        m = self._make()
        self.assertTrue(m._lease.check("anyone"))

    def test_check_lease_held(self):
        m = self._make()
        m._lease.acquire("client_a", ttl=30.0)
        self.assertTrue(m._lease.check("client_a"))
        self.assertFalse(m._lease.check("client_b"))

    def test_layer(self):
        m = self._make()
        self.assertEqual(m.layer, 6)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("gateway", h)


# ============================================================================
# MCPServerModule
# ============================================================================

class TestMCPServerModule(unittest.TestCase):

    def _make(self):
        from gateway.mcp_server import MCPServerModule
        return MCPServerModule(port=8090)

    def test_ports_in(self):
        m = self._make()
        self.assertIn("odometry", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("safety_state", m.ports_in)
        self.assertIn("mission_status", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("goal_pose", m.ports_out)
        self.assertIn("cmd_vel", m.ports_out)
        self.assertIn("stop_cmd", m.ports_out)
        self.assertIn("instruction", m.ports_out)
        self.assertIn("mode_cmd", m.ports_out)

    def test_tools_discovered_via_skill(self):
        """@skill methods on MCPServerModule itself appear in _tool_registry after on_system_modules."""
        import json
        from core.module import Module, skill

        class FakeNav(Module, layer=5):
            @skill
            def navigate_to(self, x: float, y: float) -> str:
                """Go to map coordinates (x, y)."""
                return json.dumps({"status": "navigating", "goal": [x, y]})

        m = self._make()
        m.on_system_modules({"MCPServerModule": m, "NavigationModule": FakeNav()})
        # Built-in MCP tools + navigation skills when NavigationModule is present
        self.assertGreaterEqual(len(m._tool_list), 12)
        names = [t["name"] for t in m._tool_list]
        self.assertIn("stop", names)
        self.assertIn("navigate_to", names)
        self.assertIn("get_health", names)
        self.assertIn("get_config", names)

    def test_stop_via_skill(self):
        import json
        m = self._make()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        result = json.loads(m.stop())
        self.assertEqual(result["status"], "stopped")
        self.assertEqual(stops, [2])

    def test_get_position_no_odom(self):
        import json
        m = self._make()
        result = json.loads(m.get_robot_position())
        self.assertIn("error", result)

    def test_get_position_with_odom(self):
        import json
        m = self._make()
        m._odom = {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}
        result = json.loads(m.get_robot_position())
        self.assertAlmostEqual(result["x"], 1.0)

    def test_tag_location_via_skill(self):
        import json
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        m = self._make()
        m._odom = {"x": 5.0, "y": 3.0, "z": 0.0}
        tl = TaggedLocationsModule()
        m._tagged_locations_mod = tl
        result = json.loads(m.tag_location("office"))
        self.assertEqual(result["tagged"], "office")
        self.assertIsNotNone(tl.store.query("office"))

    def test_navigate_to_resolves_to_navigation_named_module(self):
        import json
        from core.module import Module, skill

        class FakeNav(Module, layer=5):
            def __init__(self):
                super().__init__()
                self.calls = []

            @skill
            def navigate_to(self, x: float, y: float, yaw: float = 0.0) -> str:
                """Navigate to (x, y) in map frame."""
                self.calls.append((x, y, yaw))
                return json.dumps({"status": "navigating", "goal": [x, y], "yaw": yaw})

        m = self._make()
        nav = FakeNav()
        m.on_system_modules({"MCPServerModule": m, "NavigationModule": nav})
        fn = m._tool_registry.get("navigate_to")
        self.assertIsNotNone(fn)
        self.assertEqual(getattr(fn, "__self__", None), nav)
        result = json.loads(fn(5.0, 3.0))
        self.assertEqual(result["status"], "navigating")
        self.assertEqual(nav.calls, [(5.0, 3.0, 0.0)])

    def test_set_mode_via_skill(self):
        import json
        m = self._make()
        result = json.loads(m.set_mode("autonomous"))
        self.assertEqual(result["mode"], "autonomous")

    def test_set_mode_invalid(self):
        import json
        m = self._make()
        result = json.loads(m.set_mode("unknown"))
        self.assertIn("error", result)

    def test_get_health_no_handle(self):
        import json
        m = self._make()
        result = json.loads(m.get_health())
        self.assertIn("error", result)

    def test_health(self):
        m = self._make()
        m.on_system_modules({"MCPServerModule": m})
        h = m.health()
        self.assertIn("mcp", h)
        self.assertEqual(h["mcp"]["tools"], len(m._tool_list))
        self.assertGreaterEqual(h["mcp"]["tools"], 13)


# ============================================================================
# RerunModule
# ============================================================================

class TestRerunModule(unittest.TestCase):

    def _make(self):
        from core.rerun_module import RerunModule
        return RerunModule(web_port=9090)

    def test_ports_in(self):
        m = self._make()
        self.assertIn("odometry", m.ports_in)
        self.assertIn("global_path", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("safety_state", m.ports_in)
        self.assertIn("waypoint", m.ports_in)
        self.assertIn("cmd_vel", m.ports_in)

    def test_trajectory_empty_initially(self):
        m = self._make()
        self.assertEqual(len(m._trajectory), 0)

    def test_layer(self):
        m = self._make()
        self.assertEqual(m.layer, 6)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("rerun", h)
        self.assertEqual(h["rerun"]["trajectory_len"], 0)


# ============================================================================
# SLAMModule
# ============================================================================

class TestSLAMModule(unittest.TestCase):

    def test_ports(self):
        from slam.slam_module import SLAMModule
        m = SLAMModule(backend="fastlio2")
        self.assertIn("alive", m.ports_out)
        self.assertNotIn("odometry", m.ports_out)
        self.assertNotIn("map_cloud", m.ports_out)

    def test_layer(self):
        from slam.slam_module import SLAMModule
        m = SLAMModule(backend="fastlio2")
        self.assertEqual(m.layer, 1)

    def test_backends_registered(self):
        from slam.slam_module import SLAMModule  # trigger @register
        from core.registry import list_plugins
        backends = list_plugins("slam")
        self.assertIn("fastlio2", backends)
        self.assertIn("pointlio", backends)

    def test_health(self):
        from slam.slam_module import SLAMModule
        m = SLAMModule(backend="pointlio")
        h = m.health()
        self.assertIn("slam", h)
        self.assertEqual(h["slam"]["backend"], "pointlio")


# ============================================================================
# SemanticPlannerModule
# ============================================================================

class TestSemanticPlannerModule(unittest.TestCase):

    def _make(self):
        import sys, os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "semantic", "planner"))
        from semantic.planner.semantic_planner.semantic_planner_module import SemanticPlannerModule
        return SemanticPlannerModule()

    def test_ports_in(self):
        m = self._make()
        self.assertIn("instruction", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("odometry", m.ports_in)
        self.assertIn("detections", m.ports_in)
        self.assertIn("mission_status", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("goal_pose", m.ports_out)
        self.assertIn("planner_status", m.ports_out)
        self.assertIn("task_plan", m.ports_out)
        self.assertIn("cancel", m.ports_out)

    def test_decompose_fallback(self):
        m = self._make()
        result = m._decompose("go to the kitchen")
        self.assertIn("subtasks", result)

    def test_layer(self):
        m = self._make()
        self.assertEqual(m.layer, 4)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("semantic_planner", h)
        self.assertIn("lera_triggers", h["semantic_planner"])

    # -- LERa integration tests -----------------------------------------------

    def test_lera_ignored_when_executing(self):
        """Non-terminal nav states must not trigger LERa."""
        m = self._make()
        statuses = []
        m.planner_status._add_callback(statuses.append)
        m._on_mission_status({"state": "EXECUTING"})
        self.assertNotIn("RECOVERING", statuses)

    def test_lera_triggers_on_stuck(self):
        """STUCK state must publish RECOVERING and increment lera_count."""
        import time as _time
        m = self._make()
        statuses = []
        m.planner_status._add_callback(statuses.append)
        m._lera_cooldown = 0  # disable cooldown for test
        m._on_mission_status({"state": "STUCK"})
        # RECOVERING is published synchronously before thread launch.
        self.assertIn("RECOVERING", statuses)
        self.assertEqual(m._lera_count, 1)
        # Wait briefly for background thread to finish and clear _lera_running.
        for _ in range(20):
            with m._lera_lock:
                if not m._lera_running:
                    break
            _time.sleep(0.05)

    def test_lera_cooldown_prevents_storm(self):
        """Second STUCK within cooldown window must be silently dropped."""
        m = self._make()
        counts = []
        m.planner_status._add_callback(lambda s: counts.append(s) if s == "RECOVERING" else None)
        m._lera_cooldown = 999  # very long cooldown
        m._last_nav_state = ""  # first trigger allowed
        m._on_mission_status({"state": "STUCK"})
        m._last_nav_state = ""  # reset so state-change check passes
        m._on_mission_status({"state": "STUCK"})  # must be dropped by cooldown
        self.assertEqual(len(counts), 1)

    def test_lera_abort_publishes_cancel(self):
        """abort strategy must publish 'lera_abort' on cancel port."""
        m = self._make()
        cancels = []
        m.cancel._add_callback(cancels.append)
        m._dispatch_recovery("abort")
        self.assertEqual(cancels, ["lera_abort"])

    def test_lera_retry_republishes_goal(self):
        """retry_different_path must republish the cached goal_pose."""
        m = self._make()
        goals = []
        m.goal_pose._add_callback(goals.append)
        pose = PoseStamped(
            pose=Pose(position=Vector3(3, 4, 0), orientation=Quaternion(0, 0, 0, 1)),
            frame_id="map", ts=0.0,
        )
        m._current_goal_pose = pose
        m._dispatch_recovery("retry_different_path")
        self.assertEqual(len(goals), 1)
        self.assertAlmostEqual(goals[0].pose.position.x, 3.0)

    def test_lera_abort_clears_state(self):
        """abort must reset failure_count and current_instruction."""
        m = self._make()
        m._failure_count = 3
        m._current_instruction = "go to kitchen"
        m._dispatch_recovery("abort")
        self.assertEqual(m._failure_count, 0)
        self.assertEqual(m._current_instruction, "")

    def test_lera_no_duplicate_trigger_same_state(self):
        """Consecutive publishes of the same state must only trigger LERa once."""
        m = self._make()
        m._lera_cooldown = 0
        m._on_mission_status({"state": "STUCK"})
        count_after_first = m._lera_count
        m._on_mission_status({"state": "STUCK"})  # same state, must be dropped
        self.assertEqual(m._lera_count, count_after_first)

    def test_fast_resolve_uses_target_xyz_when_position_missing(self):
        m = self._make()

        class _Result:
            confidence = 0.9
            target_x = 16.25
            target_y = 2.9
            target_z = 0.937
            frame_id = 'map'

        class _Resolver:
            def maybe_reload_kg(self):
                return None

            def fast_resolve(self, instruction, sg_json):
                return _Result()

        m._goal_resolver = _Resolver()
        goals = []
        m.goal_pose._add_callback(goals.append)

        m._try_resolve('find the stairs', '{"objects": []}')

        self.assertEqual(len(goals), 1)
        self.assertAlmostEqual(goals[0].pose.position.x, 16.25)
        self.assertAlmostEqual(goals[0].pose.position.y, 2.9)
        self.assertAlmostEqual(goals[0].pose.position.z, 0.937)

    def test_scene_graph_stored_as_object(self):
        """_on_scene_graph must persist the SceneGraph object, not just JSON."""
        from core.msgs.semantic import SceneGraph, Detection3D
        from core.msgs.geometry import Vector3
        m = self._make()
        sg = SceneGraph(objects=[Detection3D(label="chair", confidence=0.9)])
        m._on_scene_graph(sg)
        self.assertIsNotNone(m._current_scene_graph)
        self.assertEqual(m._current_scene_graph.objects[0].label, "chair")


# ============================================================================
# MissionLoggerModule
# ============================================================================

class TestMissionLoggerModule(unittest.TestCase):

    def _make(self, tmp_path=None):
        import tempfile
        from memory.modules.mission_logger_module import MissionLoggerModule
        log_dir = tmp_path or tempfile.mkdtemp(prefix="lingtu_test_missions_")
        return MissionLoggerModule(log_dir=log_dir)

    def test_ports(self):
        m = self._make()
        self.assertIn("mission_status", m.ports_in)
        self.assertIn("odometry", m.ports_in)
        self.assertEqual(len(m.ports_out), 0)

    def test_empty_list(self):
        m = self._make()
        self.assertEqual(m.list_missions(), [])

    def test_empty_stats(self):
        m = self._make()
        s = m.get_stats()
        self.assertEqual(s["total"], 0)
        self.assertEqual(s["success"], 0)
        self.assertEqual(s["failed"], 0)

    def test_mission_lifecycle_saved(self):
        import tempfile, os
        tmp = tempfile.mkdtemp(prefix="lingtu_test_missions_")
        m = self._make(tmp_path=tmp)

        m._on_mission_status({"state": "PLANNING", "goal": "kitchen"})
        self.assertIsNotNone(m._current)
        self.assertEqual(m._current["goal"], "kitchen")

        from core.msgs.geometry import Pose, Vector3, Quaternion
        def _odom(x, y):
            return Odometry(pose=Pose(position=Vector3(x=x, y=y, z=0.0)))
        m._on_odom(_odom(1.0, 2.0))
        odom2 = _odom(4.0, 6.0)
        m._on_odom(odom2)

        m._on_mission_status({"state": "SUCCESS"})
        self.assertIsNone(m._current)

        from pathlib import Path as PPath
        files = list(PPath(tmp).glob("*.json"))
        self.assertEqual(len(files), 1)
        import json
        data = json.loads(files[0].read_text())
        self.assertEqual(data["result"], "SUCCESS")
        self.assertEqual(data["goal"], "kitchen")
        self.assertGreater(data["distance_m"], 0)

    def test_list_and_stats_after_missions(self):
        import tempfile
        tmp = tempfile.mkdtemp(prefix="lingtu_test_missions_")
        m = self._make(tmp_path=tmp)

        for result in ("SUCCESS", "FAILED", "SUCCESS"):
            m._on_mission_status({"state": "PLANNING", "goal": "test"})
            m._on_mission_status({"state": result})

        missions = m.list_missions()
        self.assertEqual(len(missions), 3)
        s = m.get_stats()
        self.assertEqual(s["total"], 3)
        self.assertEqual(s["success"], 2)
        self.assertEqual(s["failed"], 1)

    def test_replan_count_tracked(self):
        import tempfile
        tmp = tempfile.mkdtemp(prefix="lingtu_test_missions_")
        m = self._make(tmp_path=tmp)

        m._on_mission_status({"state": "PLANNING", "goal": "go far"})
        m._on_mission_status({"state": "REPLANNING"})
        m._on_mission_status({"state": "REPLANNING"})
        m._on_mission_status({"state": "SUCCESS"})

        missions = m.list_missions()
        self.assertEqual(missions[0]["replan_count"], 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
