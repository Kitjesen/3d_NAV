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
        m._path = [np.array([1, 2, 0])]
        m._on_stop(2)
        self.assertEqual(len(m._path), 0)
        self.assertEqual(m._state, "IDLE")

    def test_downsample(self):
        m = self._make(downsample_dist=2.0)
        path = [(0, 0, 0), (0.5, 0, 0), (1, 0, 0), (3, 0, 0), (5, 0, 0)]
        result = m._downsample(path)
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
        self.assertIsNotNone(m._latest_odom)
        self.assertAlmostEqual(m._latest_odom["x"], 1.0)

    def test_check_lease_open(self):
        m = self._make()
        self.assertTrue(m._check_lease("anyone"))

    def test_check_lease_held(self):
        m = self._make()
        m._lease_holder = "client_a"
        m._lease_expiry = time.time() + 30
        self.assertTrue(m._check_lease("client_a"))
        self.assertFalse(m._check_lease("client_b"))

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

    def test_tools_count(self):
        from gateway.mcp_server import TOOLS
        self.assertEqual(len(TOOLS), 16)

    def test_execute_stop(self):
        m = self._make()
        stops = []
        m.stop_cmd._add_callback(stops.append)
        m._execute_tool("stop", {})
        self.assertEqual(stops, [2])

    def test_execute_get_position_no_odom(self):
        import json
        m = self._make()
        result = json.loads(m._execute_tool("get_robot_position", {}))
        self.assertIn("error", result)

    def test_execute_get_position_with_odom(self):
        import json
        m = self._make()
        m._odom = {"x": 1.0, "y": 2.0, "z": 0.0, "yaw": 0.5}
        result = json.loads(m._execute_tool("get_robot_position", {}))
        self.assertAlmostEqual(result["x"], 1.0)

    def test_execute_tag_location(self):
        import json
        m = self._make()
        m._odom = {"x": 5.0, "y": 3.0, "z": 0.0}
        result = json.loads(m._execute_tool("tag_location", {"name": "office"}))
        self.assertEqual(result["tagged"], "office")
        self.assertIn("office", m._tagged_locations)

    def test_execute_navigate_to(self):
        import json
        m = self._make()
        goals = []
        m.goal_pose._add_callback(goals.append)
        result = json.loads(m._execute_tool("navigate_to", {"x": 5, "y": 3}))
        self.assertEqual(result["status"], "navigating")
        self.assertEqual(len(goals), 1)

    def test_execute_set_mode(self):
        import json
        m = self._make()
        result = json.loads(m._execute_tool("set_mode", {"mode": "autonomous"}))
        self.assertEqual(result["mode"], "autonomous")

    def test_execute_unknown_tool(self):
        import json
        m = self._make()
        result = json.loads(m._execute_tool("nonexistent", {}))
        self.assertIn("error", result)

    def test_health(self):
        m = self._make()
        h = m.health()
        self.assertIn("mcp", h)
        self.assertEqual(h["mcp"]["tools"], 16)


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
        self.assertIn("odometry", m.ports_out)
        self.assertIn("map_cloud", m.ports_out)
        self.assertIn("alive", m.ports_out)

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
        from semantic_planner.semantic_planner_module import SemanticPlannerModule
        return SemanticPlannerModule()

    def test_ports_in(self):
        m = self._make()
        self.assertIn("instruction", m.ports_in)
        self.assertIn("scene_graph", m.ports_in)
        self.assertIn("odometry", m.ports_in)
        self.assertIn("detections", m.ports_in)

    def test_ports_out(self):
        m = self._make()
        self.assertIn("resolved_goal", m.ports_out)
        self.assertIn("planner_status", m.ports_out)
        self.assertIn("task_plan", m.ports_out)

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
