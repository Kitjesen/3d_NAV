"""Tests for localization health monitoring (P0 safety).

Tests cover:
  - SlamBridgeModule watchdog: UNINIT → TRACKING → LOST → recovery
  - SafetyRingModule: localization status → stop_cmd
  - NavigationModule: pause/resume on localization loss
"""

import os
import sys
import time
import unittest

# Ensure src/ is on path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from core.msgs.geometry import Pose, Quaternion, Vector3
from core.msgs.nav import Odometry

# ──────────────────────────────────────────────────────────────────────────────
# SlamBridgeModule watchdog tests
# ──────────────────────────────────────────────────────────────────────────────

class TestSlamBridgeWatchdog(unittest.TestCase):

    def _make(self, **kw):
        from slam.slam_bridge_module import SlamBridgeModule
        defaults = {"odom_timeout": 0.1, "cloud_timeout": 0.2, "watchdog_hz": 20}
        defaults.update(kw)
        return SlamBridgeModule(**defaults)

    def test_initial_state_is_uninit(self):
        m = self._make()
        self.assertEqual(m._loc_state, "UNINIT")

    def test_ports_declared(self):
        m = self._make()
        self.assertIn("localization_status", m.ports_out)
        self.assertIn("odometry", m.ports_out)
        self.assertIn("map_cloud", m.ports_out)

    def test_watchdog_publishes_uninit_initially(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        time.sleep(0.15)
        m.stop()
        self.assertTrue(len(received) > 0)
        self.assertEqual(received[0]["state"], "UNINIT")
        self.assertAlmostEqual(received[0]["confidence"], 0.0)

    def test_odom_arrival_sets_tracking(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        # Simulate odom + cloud arrival
        m._last_odom_time = time.time()
        m._last_cloud_time = time.time()
        time.sleep(0.15)
        m.stop()
        # Should have transitioned to TRACKING
        states = [r["state"] for r in received]
        self.assertIn("TRACKING", states)

    def test_odom_timeout_sets_lost(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        # Simulate odom arrival then timeout
        m._last_odom_time = time.time()
        m._last_cloud_time = time.time()
        time.sleep(0.05)  # Within timeout — should be TRACKING
        # Now let odom age past timeout (0.1s)
        m._last_odom_time = time.time() - 0.2  # Artificially stale
        time.sleep(0.15)
        m.stop()
        states = [r["state"] for r in received]
        self.assertIn("LOST", states)

    def test_cloud_timeout_sets_degraded(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        # odom fresh, cloud stale
        m._last_odom_time = time.time()
        m._last_cloud_time = time.time() - 0.5  # Past cloud_timeout (0.2s)
        time.sleep(0.15)
        m.stop()
        states = [r["state"] for r in received]
        self.assertIn("DEGRADED", states)

    def test_recovery_from_lost(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        # Go to LOST
        m._last_odom_time = time.time() - 0.5
        m._last_cloud_time = time.time() - 0.5
        time.sleep(0.1)
        # Recover
        m._last_odom_time = time.time()
        m._last_cloud_time = time.time()
        time.sleep(0.1)
        m.stop()
        states = [r["state"] for r in received]
        self.assertIn("LOST", states)
        # After recovery, should see TRACKING
        lost_idx = max(i for i, s in enumerate(states) if s == "LOST")
        remaining = states[lost_idx + 1:]
        self.assertIn("TRACKING", remaining)

    def test_health_report_includes_localization(self):
        m = self._make()
        h = m.health()
        self.assertIn("localization", h)
        self.assertEqual(h["localization"]["state"], "UNINIT")

    def test_stop_is_idempotent(self):
        m = self._make()
        m.start()
        m.stop()
        m.stop()  # Should not raise


# ──────────────────────────────────────────────────────────────────────────────
# SafetyRingModule localization tests
# ──────────────────────────────────────────────────────────────────────────────

class TestSafetyRingLocalization(unittest.TestCase):

    def _make(self):
        from nav.safety_ring_module import SafetyRingModule
        return SafetyRingModule()

    def test_localization_status_port_exists(self):
        m = self._make()
        self.assertIn("localization_status", m.ports_in)

    def test_lost_triggers_stop(self):
        m = self._make()
        m.setup()
        # Prime odom so odom timeout doesn't interfere
        m._last_odom_time = time.time()
        m._last_cmdvel_time = time.time()

        stop_cmds = []
        m.stop_cmd._add_callback(stop_cmds.append)

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        # Should have published stop_cmd = 2
        self.assertTrue(any(cmd == 2 for cmd in stop_cmds),
                        f"Expected stop_cmd=2 in {stop_cmds}")

    def test_degraded_triggers_warn(self):
        m = self._make()
        m.setup()
        m._last_odom_time = time.time()
        m._last_cmdvel_time = time.time()

        stop_cmds = []
        m.stop_cmd._add_callback(stop_cmds.append)

        m._on_localization_status({"state": "DEGRADED", "confidence": 0.3})
        self.assertTrue(any(cmd == 1 for cmd in stop_cmds),
                        f"Expected stop_cmd=1 in {stop_cmds}")

    def test_tracking_clears_stop(self):
        m = self._make()
        m.setup()
        m._last_odom_time = time.time()
        m._last_cmdvel_time = time.time()

        stop_cmds = []
        m.stop_cmd._add_callback(stop_cmds.append)

        # Go LOST then recover
        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})
        self.assertTrue(any(cmd == 0 for cmd in stop_cmds),
                        f"Expected stop_cmd=0 in {stop_cmds}")

    def test_health_includes_localization(self):
        m = self._make()
        m._on_localization_status({"state": "TRACKING", "confidence": 0.9})
        h = m.health()
        self.assertEqual(h["safety_ring"]["localization_state"], "TRACKING")
        self.assertAlmostEqual(h["safety_ring"]["localization_confidence"], 0.9)

    def test_dialogue_includes_localization(self):
        m = self._make()
        m.setup()
        m._last_odom_time = time.time()

        dialogue_msgs = []
        m.dialogue_state._add_callback(dialogue_msgs.append)

        odom = Odometry(pose=Pose(position=Vector3(1, 2, 0)), ts=time.time())
        m._on_odom(odom)
        self.assertTrue(len(dialogue_msgs) > 0)
        self.assertIn("localization", dialogue_msgs[-1])


# ──────────────────────────────────────────────────────────────────────────────
# NavigationModule pause/resume tests
# ──────────────────────────────────────────────────────────────────────────────

class TestNavigationPauseResume(unittest.TestCase):

    def _make(self):
        from nav.navigation_module import MissionState, NavigationModule
        m = NavigationModule(planner="astar")
        m.setup()
        return m, MissionState

    def test_localization_status_port_exists(self):
        from nav.navigation_module import NavigationModule
        m = NavigationModule(planner="astar")
        self.assertIn("localization_status", m.ports_in)

    def test_lost_pauses_executing(self):
        m, MS = self._make()
        m._state = MS.EXECUTING
        m._goal = [1.0, 2.0, 0.0]

        m._on_localization_status({"state": "LOST", "confidence": 0.0})

        self.assertTrue(m._paused_for_localization)
        self.assertEqual(m._state, MS.IDLE)
        self.assertEqual(m._pre_pause_state, MS.EXECUTING)

    def test_tracking_resumes_after_pause(self):
        m, MS = self._make()
        m._state = MS.EXECUTING
        m._goal = [1.0, 2.0, 0.0]

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        self.assertEqual(m._state, MS.IDLE)

        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})
        self.assertFalse(m._paused_for_localization)
        self.assertEqual(m._state, MS.EXECUTING)

    def test_idle_not_affected_by_lost(self):
        m, MS = self._make()
        m._state = MS.IDLE

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        self.assertEqual(m._state, MS.IDLE)
        self.assertFalse(m._paused_for_localization)

    def test_odom_skipped_while_paused(self):
        m, MS = self._make()
        import numpy as np
        m._state = MS.EXECUTING
        m._goal = np.array([10.0, 10.0, 0.0])

        # Pause
        m._on_localization_status({"state": "LOST", "confidence": 0.0})

        # Send odom — should be skipped (position updates but tracker not called)
        initial_status_count = m.mission_status.msg_count
        odom = Odometry(pose=Pose(position=Vector3(5, 5, 0)), ts=time.time())
        m._on_odom(odom)
        # mission_status should not change (no tracker event)
        self.assertEqual(m._state, MS.IDLE)

    def test_goal_preserved_during_pause(self):
        m, MS = self._make()
        import numpy as np
        m._state = MS.EXECUTING
        m._goal = np.array([10.0, 20.0, 0.0])

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        # Goal should still be set
        self.assertIsNotNone(m._goal)
        self.assertAlmostEqual(m._goal[0], 10.0)
        self.assertAlmostEqual(m._goal[1], 20.0)

    def test_patrolling_pause_resume(self):
        m, MS = self._make()
        m._state = MS.PATROLLING
        m._goal = [5.0, 5.0, 0.0]

        m._on_localization_status({"state": "LOST", "confidence": 0.0})
        self.assertEqual(m._pre_pause_state, MS.PATROLLING)

        m._on_localization_status({"state": "TRACKING", "confidence": 1.0})
        self.assertEqual(m._state, MS.PATROLLING)


# ──────────────────────────────────────────────────────────────────────────────
# WaypointTracker pause tests
# ──────────────────────────────────────────────────────────────────────────────

class TestWaypointTrackerPause(unittest.TestCase):

    def test_pause_resets_stuck_timer(self):
        import numpy as np

        from nav.waypoint_tracker import WaypointTracker
        t = WaypointTracker(threshold=1.0, stuck_timeout=5.0)
        t.reset([np.array([10, 0, 0])], np.array([0, 0, 0]))
        # Simulate some time passing
        t._last_progress_time = time.time() - 100  # Very stale
        t._stuck_warn_sent = True
        t._stuck_sent = True
        # Pause should reset stuck state
        t.pause()
        self.assertFalse(t._stuck_warn_sent)
        self.assertFalse(t._stuck_sent)
        # last_progress_time should be recent
        self.assertAlmostEqual(t._last_progress_time, time.time(), delta=1.0)

    def test_pause_preserves_path(self):
        import numpy as np

        from nav.waypoint_tracker import WaypointTracker
        t = WaypointTracker(threshold=1.0)
        path = [np.array([5, 0, 0]), np.array([10, 0, 0])]
        t.reset(path, np.array([0, 0, 0]))
        t.pause()
        self.assertEqual(len(t._path), 2)
        self.assertEqual(t._wp_index, 0)


# ──────────────────────────────────────────────────────────────────────────────
# SLAM degeneracy detection tests
# ──────────────────────────────────────────────────────────────────────────────

class TestSlamDegeneracyDetection(unittest.TestCase):

    def _make(self, **kw):
        from slam.slam_bridge_module import SlamBridgeModule
        defaults = {"odom_timeout": 0.1, "cloud_timeout": 0.2, "watchdog_hz": 20}
        defaults.update(kw)
        return SlamBridgeModule(**defaults)

    def test_initial_degeneracy_is_none(self):
        from slam.slam_bridge_module import DEGEN_NONE
        m = self._make()
        self.assertEqual(m._degen_level, DEGEN_NONE)

    def test_high_icp_fitness_triggers_critical(self):
        from slam.slam_bridge_module import DEGEN_CRITICAL
        m = self._make()
        m._icp_fitness = 0.5  # Above fitness_critical (0.3)
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_CRITICAL)

    def test_moderate_icp_fitness_triggers_severe(self):
        from slam.slam_bridge_module import DEGEN_SEVERE
        m = self._make()
        m._icp_fitness = 0.2  # Above fitness_warn (0.15)
        m._effective_ratio = 0.5  # Not critical
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_SEVERE)

    def test_low_feature_ratio_triggers_critical(self):
        from slam.slam_bridge_module import DEGEN_CRITICAL
        m = self._make()
        m._icp_fitness = 0.0
        m._effective_ratio = 0.05  # Below feature_ratio_critical (0.1)
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_CRITICAL)

    def test_good_metrics_clears_degeneracy(self):
        from slam.slam_bridge_module import DEGEN_CRITICAL, DEGEN_NONE
        m = self._make()
        m._icp_fitness = 0.5
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_CRITICAL)
        # Now restore good metrics
        m._icp_fitness = 0.0
        m._effective_ratio = 1.0
        m._update_degeneracy_level()
        self.assertEqual(m._degen_level, DEGEN_NONE)

    def test_degeneracy_affects_watchdog_confidence(self):
        from slam.slam_bridge_module import DEGEN_CRITICAL
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        # Fresh odom + cloud, but critical degeneracy
        m._last_odom_time = time.time()
        m._last_cloud_time = time.time()
        m._degen_level = DEGEN_CRITICAL
        time.sleep(0.15)
        m.stop()
        # Should get DEGRADED with low confidence
        degraded = [r for r in received if r["state"] == "DEGRADED"]
        self.assertTrue(len(degraded) > 0, f"Expected DEGRADED, got states: {[r['state'] for r in received]}")
        self.assertLessEqual(degraded[0]["confidence"], 0.2)

    def test_localization_status_includes_degeneracy_fields(self):
        m = self._make()
        received = []
        m.localization_status._add_callback(received.append)
        m.start()
        time.sleep(0.1)
        m.stop()
        self.assertTrue(len(received) > 0)
        self.assertIn("degeneracy", received[0])
        self.assertIn("icp_fitness", received[0])
        self.assertIn("effective_ratio", received[0])

    def test_health_includes_degeneracy(self):
        m = self._make()
        m._icp_fitness = 0.2
        m._effective_ratio = 0.5
        h = m.health()
        self.assertEqual(h["localization"]["icp_fitness"], 0.2)
        self.assertEqual(h["localization"]["effective_ratio"], 0.5)


# ──────────────────────────────────────────────────────────────────────────────
# Navigation degeneracy speed limiting tests
# ──────────────────────────────────────────────────────────────────────────────

class TestNavigationDegeneracyResponse(unittest.TestCase):

    def _make(self):
        from nav.navigation_module import MissionState, NavigationModule
        m = NavigationModule(planner="astar")
        m.setup()
        return m, MissionState

    def test_severe_degeneracy_limits_speed(self):
        m, MS = self._make()
        m._on_localization_status({
            "state": "TRACKING", "confidence": 0.4,
            "degeneracy": "SEVERE",
        })
        self.assertAlmostEqual(m._speed_scale, 0.4)

    def test_mild_degeneracy_limits_speed(self):
        m, MS = self._make()
        m._on_localization_status({
            "state": "TRACKING", "confidence": 0.7,
            "degeneracy": "MILD",
        })
        self.assertAlmostEqual(m._speed_scale, 0.7)

    def test_no_degeneracy_full_speed(self):
        m, MS = self._make()
        m._on_localization_status({
            "state": "TRACKING", "confidence": 1.0,
            "degeneracy": "NONE",
        })
        self.assertAlmostEqual(m._speed_scale, 1.0)


if __name__ == "__main__":
    unittest.main()
