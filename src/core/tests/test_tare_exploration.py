"""Tests for the TARE exploration bridge and the exploration stack factory.

Runs without any C++ binary or DDS transport — relies on the module's
stub-mode when cyclonedds and rclpy are both absent, and exercises the
contracts (ports, skills, waypoint → PoseStamped conversion).
"""

from __future__ import annotations

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from core.msgs.geometry import PoseStamped

# ─── Bridge module ───────────────────────────────────────────────────────────

class TestTAREExplorerModulePorts(unittest.TestCase):

    def _make(self, **kw):
        from exploration.tare_explorer_module import TAREExplorerModule
        return TAREExplorerModule(**kw)

    def test_ports_declared(self):
        m = self._make()
        self.assertIn("exploration_goal", m.ports_out)
        self.assertIn("exploration_path", m.ports_out)
        self.assertIn("exploring", m.ports_out)
        self.assertIn("tare_stats", m.ports_out)
        self.assertIn("alive", m.ports_out)

    def test_exploration_goal_port_mirrors_wavefront(self):
        """Drop-in: port name + msg type must match the wavefront module
        so autoconnect can wire either backend to NavigationModule."""
        from exploration.tare_explorer_module import TAREExplorerModule
        from nav.frontier_explorer_module import WavefrontFrontierExplorer
        tare = TAREExplorerModule()
        wave = WavefrontFrontierExplorer()
        tare_port = tare.ports_out["exploration_goal"]
        wave_port = wave.ports_out["exploration_goal"]
        self.assertEqual(tare_port.msg_type, wave_port.msg_type)

    def test_stub_mode_no_crash(self):
        """No DDS, no rclpy — module should still setup/start/stop cleanly."""
        m = self._make(auto_start=False)
        m.setup()
        m.start()
        time.sleep(0.05)  # let the watchdog tick once
        m.stop()


class TestTAREWaypointEmission(unittest.TestCase):

    def test_emit_waypoint_publishes_pose_stamped(self):
        from exploration.tare_explorer_module import TAREExplorerModule
        m = TAREExplorerModule(auto_start=False)
        received: list[PoseStamped] = []
        m.exploration_goal._add_callback(received.append)

        m._emit_waypoint(3.0, 4.0, 0.0, frame_id="map")

        self.assertEqual(len(received), 1)
        pose = received[0]
        self.assertIsInstance(pose, PoseStamped)
        self.assertAlmostEqual(pose.pose.position.x, 3.0)
        self.assertAlmostEqual(pose.pose.position.y, 4.0)
        self.assertAlmostEqual(pose.pose.position.z, 0.0)
        self.assertEqual(pose.frame_id, "map")

    def test_waypoint_count_increments(self):
        from exploration.tare_explorer_module import TAREExplorerModule
        m = TAREExplorerModule(auto_start=False)
        self.assertEqual(m._waypoint_count, 0)
        m._emit_waypoint(1.0, 1.0, 0.0)
        m._emit_waypoint(2.0, 2.0, 0.0)
        self.assertEqual(m._waypoint_count, 2)
        self.assertGreater(m._last_waypoint_ts, 0.0)


class TestTAREStatsSnapshot(unittest.TestCase):

    def test_stats_schema(self):
        from exploration.tare_explorer_module import TAREExplorerModule
        m = TAREExplorerModule(auto_start=False)
        received: list[dict] = []
        m.tare_stats._add_callback(received.append)
        m._publish_stats()
        self.assertEqual(len(received), 1)
        snap = received[-1]
        for key in ("alive", "started", "healthy", "waypoint_count",
                    "waypoint_age_s", "last_runtime_ms", "finished"):
            self.assertIn(key, snap)

    def test_stats_report_unhealthy_before_first_waypoint(self):
        from exploration.tare_explorer_module import TAREExplorerModule
        m = TAREExplorerModule(auto_start=False, way_point_timeout_s=1.0)
        received: list[dict] = []
        m.tare_stats._add_callback(received.append)
        m._publish_stats()
        self.assertFalse(received[-1]["healthy"])
        # inf encoded as large number in dict form
        self.assertEqual(received[-1]["waypoint_count"], 0)


class TestTAREskills(unittest.TestCase):

    def test_skills_discoverable(self):
        from exploration.tare_explorer_module import TAREExplorerModule
        m = TAREExplorerModule(auto_start=False)
        names = {info.func_name for info in m.get_skill_infos()}
        self.assertIn("start_tare_exploration", names)
        self.assertIn("stop_tare_exploration", names)
        self.assertIn("get_tare_status", names)

    def test_start_stop_toggles_exploring_flag(self):
        from exploration.tare_explorer_module import TAREExplorerModule
        m = TAREExplorerModule(auto_start=False)
        received: list[bool] = []
        m.exploring._add_callback(received.append)
        m.start_tare_exploration()
        m.stop_tare_exploration()
        self.assertIn(True, received)
        self.assertIn(False, received)

    def test_get_tare_status_returns_json(self):
        import json

        from exploration.tare_explorer_module import TAREExplorerModule
        m = TAREExplorerModule(auto_start=False)
        parsed = json.loads(m.get_tare_status())
        self.assertIn("alive", parsed)
        self.assertIn("waypoint_count", parsed)


# ─── Stack factory ───────────────────────────────────────────────────────────

def _module_names(bp) -> set[str]:
    """Return the set of registered module names/aliases in a Blueprint."""
    return {e.name for e in bp._entries}


class TestExplorationStackFactory(unittest.TestCase):

    def test_none_returns_empty_blueprint(self):
        from core.blueprints.stacks.exploration import exploration
        bp = exploration(backend="none")
        self.assertEqual(len(bp._entries), 0)

    def test_empty_backend_returns_empty(self):
        from core.blueprints.stacks.exploration import exploration
        bp = exploration(backend="")
        self.assertEqual(len(bp._entries), 0)

    def test_unknown_backend_raises(self):
        from core.blueprints.stacks.exploration import exploration
        with self.assertRaises(ValueError):
            exploration(backend="bogus")

    def test_wavefront_adds_frontier_explorer(self):
        from core.blueprints.stacks.exploration import exploration
        bp = exploration(backend="wavefront")
        self.assertIn("WavefrontFrontierExplorer", _module_names(bp))

    def test_tare_falls_back_to_wavefront_when_binary_missing(self):
        """No TARE binary on dev machine — stack must fall back silently."""
        from core.blueprints.stacks.exploration import exploration
        bp = exploration(backend="tare")
        # TARE NativeModule must NOT be present; fallback wavefront may be
        self.assertNotIn("tare_explorer", _module_names(bp))


# ─── NativeModule factory ────────────────────────────────────────────────────

class TestTareNativeFactory(unittest.TestCase):

    def test_factory_produces_native_module(self):
        """Factory shape check — does not require the binary to exist."""
        from exploration.native_factories import tare_explorer
        m = tare_explorer(scenario="forest")
        self.assertIsNotNone(m)
        # NativeModule name matches what SlamBridge-style code expects
        self.assertEqual(m._native_config.name, "tare_explorer")
        # Essential topic remappings are in place
        remaps = m._native_config.remappings
        self.assertEqual(remaps.get("/state_estimation"), "/nav/odometry")
        self.assertEqual(remaps.get("/registered_scan"), "/nav/registered_cloud")
        self.assertEqual(remaps.get("/way_point"), "/exploration/way_point")


if __name__ == "__main__":
    unittest.main()
