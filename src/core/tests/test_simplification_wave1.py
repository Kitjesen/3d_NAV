"""Wave 1 simplification-audit fixes — behavioural tests.

Each test targets a specific Wave 1 change from
``C:\\Users\\99563\\.claude\\plans\\streamed-moseying-patterson.md``.
The goal is to lock in the hard-fail / no-silent-fallback behaviour so later
refactors can't regress to the old "looks-fine-but-wrong" state.
"""
from __future__ import annotations

import asyncio
import math
import unittest
from unittest import mock

import numpy as np

from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3

# ---------------------------------------------------------------------------
# W1-1  LocalPlannerModule — no silent fallback to straight-line paths
# ---------------------------------------------------------------------------

class TestW1LocalPlannerNoFallback(unittest.TestCase):
    def test_straight_line_fallback_method_removed(self):
        """`_publish_straight_line` must not exist as an attribute anymore."""
        from base_autonomy.modules.local_planner_module import LocalPlannerModule
        self.assertFalse(
            hasattr(LocalPlannerModule, "_publish_straight_line"),
            "The straight-line fallback method must not be re-introduced — "
            "it caused the robot to publish obstacle-ignoring paths when the "
            "C++ backend was unavailable.",
        )

    def test_nanobind_backend_raises_when_nav_core_missing(self):
        """setup() must hard-fail (not fall back) when _nav_core.so is absent."""
        from base_autonomy.modules import local_planner_module as mod
        planner = mod.LocalPlannerModule(backend="nanobind")
        with mock.patch.object(mod, "try_import_nav_core", return_value=None):
            with self.assertRaises(RuntimeError) as ctx:
                planner._setup_nanobind()
            self.assertIn("_nav_core", str(ctx.exception))


# ---------------------------------------------------------------------------
# W1-2  TerrainModule — no silent fallback when _nav_core missing
# ---------------------------------------------------------------------------

class TestW1TerrainNoFallback(unittest.TestCase):
    def test_nanobind_backend_raises_when_nav_core_missing(self):
        from base_autonomy.modules import terrain_module as mod
        terrain = mod.TerrainModule(backend="nanobind")
        with mock.patch.object(mod, "try_import_nav_core", return_value=None):
            with self.assertRaises(RuntimeError) as ctx:
                terrain._setup_nanobind()
            self.assertIn("_nav_core", str(ctx.exception))

    def test_simple_backend_still_allowed_explicitly(self):
        """`backend="simple"` remains a valid passthrough choice for tests."""
        from base_autonomy.modules.terrain_module import TerrainModule
        t = TerrainModule(backend="simple")
        # setup() must not raise — simple backend = explicit passthrough
        t.setup()


# ---------------------------------------------------------------------------
# W1-3  NavigationModule — direct-goal fallback OFF by default
# ---------------------------------------------------------------------------

class TestW1NavigationDefaultSafe(unittest.TestCase):
    def test_allow_direct_goal_fallback_defaults_to_false(self):
        """Default must be safe: no planner → fail, not ‘publish raw goal’."""
        from nav.navigation_module import NavigationModule
        m = NavigationModule(planner="astar")
        self.assertFalse(m._allow_direct_goal_fallback)

    def test_explicit_opt_in_still_works(self):
        """Callers can still enable the legacy behaviour deliberately."""
        from nav.navigation_module import NavigationModule
        m = NavigationModule(planner="astar", allow_direct_goal_fallback=True)
        self.assertTrue(m._allow_direct_goal_fallback)


# ---------------------------------------------------------------------------
# W1-4 / W1-5  OccupancyGrid / ESDF — scipy is a hard dependency now
# ---------------------------------------------------------------------------

_scipy_available = True
try:
    import scipy.ndimage  # noqa: F401
except ImportError:
    _scipy_available = False


@unittest.skipUnless(_scipy_available, "scipy not installed in this environment")
class TestW1ScipyHardDependency(unittest.TestCase):
    def test_occupancy_setup_succeeds_with_scipy(self):
        """If scipy is installed the module sets up cleanly."""
        from nav.occupancy_grid_module import OccupancyGridModule
        og = OccupancyGridModule()
        og.setup()  # must not raise

    def test_esdf_setup_succeeds_with_scipy(self):
        from nav.esdf_module import ESDFModule
        e = ESDFModule()
        e.setup()

    def test_occupancy_setup_raises_without_scipy(self):
        from nav.occupancy_grid_module import OccupancyGridModule
        og = OccupancyGridModule()
        with mock.patch.dict("sys.modules", {"scipy.ndimage": None}):
            # mock.patch.dict with value=None makes `import` raise ImportError
            with self.assertRaises(RuntimeError) as ctx:
                og.setup()
            self.assertIn("scipy", str(ctx.exception))


# ---------------------------------------------------------------------------
# W1-6  WaypointTracker — yaw-aware stuck detection
# ---------------------------------------------------------------------------

class TestW1WaypointTrackerYaw(unittest.TestCase):
    def _mk(self):
        from nav.waypoint_tracker import WaypointTracker
        return WaypointTracker(
            threshold=0.5,
            stuck_timeout=0.2,
            stuck_dist=0.15,
            stuck_yaw_rad=0.35,
        )

    def test_distance_only_legacy_behaviour_preserved(self):
        """Callers that don't supply yaw see the original distance-only logic."""
        import time as _t

        from nav.waypoint_tracker import EV_STUCK, EV_STUCK_WARN
        tracker = self._mk()
        path = [np.array([5.0, 0.0, 0.0])]
        tracker.reset(path, np.array([0.0, 0.0, 0.0]))
        _t.sleep(0.25)
        # First update past timeout fires EV_STUCK_WARN (by design, warn-then-stuck).
        status1 = tracker.update(np.array([0.10, 0.0, 0.0]))  # under stuck_dist
        self.assertEqual(status1.event, EV_STUCK_WARN)
        # Next tick (after warn already sent) fires EV_STUCK.
        status2 = tracker.update(np.array([0.10, 0.0, 0.0]))
        self.assertEqual(status2.event, EV_STUCK)

    def test_yaw_progress_resets_stuck_timer(self):
        """When yaw is tracked, rotation alone counts as progress."""
        import time as _t
        tracker = self._mk()
        path = [np.array([5.0, 0.0, 0.0])]
        tracker.reset(path, np.array([0.0, 0.0, 0.0]), robot_yaw=0.0)
        _t.sleep(0.12)
        # Rotate past threshold (~0.35 rad ≈ 20°) without translating
        status = tracker.update(
            np.array([0.02, 0.0, 0.0]),  # below stuck_dist
            robot_yaw=0.6,               # well above stuck_yaw
        )
        # Must be non-stuck — yaw rotation counts as progress
        self.assertIsNone(status.event)

    def test_tiny_spin_with_no_distance_eventually_triggers_stuck(self):
        """Sub-threshold yaw + sub-threshold dist = still stuck after warn."""
        import time as _t

        from nav.waypoint_tracker import EV_STUCK, EV_STUCK_WARN
        tracker = self._mk()
        path = [np.array([5.0, 0.0, 0.0])]
        tracker.reset(path, np.array([0.0, 0.0, 0.0]), robot_yaw=0.0)
        _t.sleep(0.25)
        # First update past timeout fires EV_STUCK_WARN.
        status1 = tracker.update(
            np.array([0.02, 0.0, 0.0]),  # < stuck_dist
            robot_yaw=0.05,              # < stuck_yaw (0.35 rad)
        )
        self.assertEqual(status1.event, EV_STUCK_WARN)
        # Second update after warn fires EV_STUCK.
        status2 = tracker.update(
            np.array([0.02, 0.0, 0.0]),
            robot_yaw=0.05,
        )
        self.assertEqual(status2.event, EV_STUCK)


# ---------------------------------------------------------------------------
# W1-7  GNSS rtcm_age_s — pipe from driver, don't hardcode 99.9
# ---------------------------------------------------------------------------

class TestW1GnssRtcmAge(unittest.TestCase):
    def test_parse_gga_extracts_rtcm_age_when_present(self):
        from slam.gnss_serial_driver import parse_gga
        # GGA with age=1.2 (field 13) and refID=0000
        fields = [
            "$GNGGA", "000000.00",
            "3113.43", "N", "12126.88", "E",
            "4", "12", "0.8",
            "20.3", "M", "0.0", "M",
            "1.2", "0000",
        ]
        parsed = parse_gga(fields)
        self.assertIsNotNone(parsed)
        self.assertEqual(parsed["rtcm_age_s"], 1.2)

    def test_parse_gga_returns_none_when_age_field_empty(self):
        from slam.gnss_serial_driver import parse_gga
        fields = [
            "$GNGGA", "000000.00",
            "3113.43", "N", "12126.88", "E",
            "1", "8", "1.2",
            "20.3", "M", "0.0", "M",
            "", "",  # age empty
        ]
        parsed = parse_gga(fields)
        self.assertIsNotNone(parsed)
        self.assertIsNone(parsed["rtcm_age_s"])


# ---------------------------------------------------------------------------
# W1-8  MobileCLIPEncoder — fake "try Apple MobileCLIP" path removed
# ---------------------------------------------------------------------------

class TestW1MobileCLIPDeadCodeRemoved(unittest.TestCase):
    def test_try_load_mobileclip_method_removed(self):
        from semantic.perception.semantic_perception.mobileclip_encoder import (
            MobileCLIPEncoder,
        )
        self.assertFalse(
            hasattr(MobileCLIPEncoder, "_try_load_mobileclip"),
            "Dead path that tried to load non-existent /mobileclip_s2.pt "
            "must stay removed — it was never reachable in practice.",
        )


# ---------------------------------------------------------------------------
# W1-9  CLIPEncoder — degraded flag is exposed
# ---------------------------------------------------------------------------

class TestW1ClipDegradedFlag(unittest.TestCase):
    def test_is_degraded_property_exists(self):
        from semantic.perception.semantic_perception.clip_encoder import CLIPEncoder
        enc = CLIPEncoder.__new__(CLIPEncoder)  # avoid loading the model
        enc._degraded = False
        enc._degraded_reason = ""
        self.assertFalse(enc.is_degraded)
        self.assertEqual(enc.degraded_reason, "")

        enc._degraded = True
        enc._degraded_reason = "GPU OOM"
        self.assertTrue(enc.is_degraded)
        self.assertEqual(enc.degraded_reason, "GPU OOM")


# ---------------------------------------------------------------------------
# W1-10  MockLLMClient — confidence=0 + mock flag
# ---------------------------------------------------------------------------

class TestW1MockLLMSignals(unittest.TestCase):
    def test_mock_response_is_flagged_and_confidence_zero(self):
        from semantic.planner.semantic_planner.llm_client import (
            LLMConfig,
            MockLLMClient,
        )
        client = MockLLMClient(LLMConfig(backend="mock"))
        raw = asyncio.get_event_loop().run_until_complete(
            client.chat([{"role": "user", "content": "find the kitchen table"}])
        )
        import json as _json
        # MockLLMClient.chat returns a JSON string — parse it
        # (the response dict is serialised somewhere downstream)
        try:
            payload = _json.loads(raw)
        except (_json.JSONDecodeError, TypeError):
            self.fail(f"MockLLM did not return JSON: {raw!r}")
        self.assertTrue(payload.get("mock"),
                        "Mock responses must be tagged with mock=True")
        self.assertEqual(payload.get("confidence"), 0.0,
                         "Mock must report confidence=0 so downstream never "
                         "trusts its output as real inference.")


if __name__ == "__main__":
    unittest.main()
