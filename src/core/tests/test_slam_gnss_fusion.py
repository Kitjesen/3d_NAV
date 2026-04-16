"""Tests for SLAM + GNSS global position anchoring in SlamBridgeModule.

Covers the ``_fuse_gnss_position`` stage of ``_fuse_odometry``:
  - port wiring
  - quality / freshness / covariance gating
  - one-shot alignment (map↔ENU offset lock)
  - per-frame weighted blend (healthy vs. degraded SLAM)
  - RTK_FLOAT weight discount
  - preservation of Z and orientation
  - fusion disable switch

These tests do not start watchdog threads or DDS; they drive the fusion
path directly the way ``test_degeneracy_fusion.py`` does.
"""

from __future__ import annotations

import os
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from core.msgs.geometry import Pose, Quaternion, Vector3
from core.msgs.gnss import GnssFixType, GnssOdom
from core.msgs.nav import Odometry


def _make_slam_odom(x: float = 0.0, y: float = 0.0, z: float = 0.0,
                    qw: float = 1.0) -> Odometry:
    return Odometry(
        pose=Pose(
            position=Vector3(x=x, y=y, z=z),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=qw),
        ),
        ts=time.time(),
    )


def _make_gnss_odom(e: float = 0.0, n: float = 0.0, u: float = 0.0,
                    fix_type: GnssFixType = GnssFixType.RTK_FIXED,
                    cov_e: float = 0.01, cov_n: float = 0.01) -> GnssOdom:
    return GnssOdom(
        east=e, north=n, up=u,
        cov_e=cov_e, cov_n=cov_n, cov_u=0.04,
        fix_type=fix_type,
        ts=time.time(),
    )


def _make_bridge(**kw):
    from slam.slam_bridge_module import SlamBridgeModule
    defaults = {"odom_timeout": 0.1, "cloud_timeout": 0.2, "watchdog_hz": 20}
    defaults.update(kw)
    return SlamBridgeModule(**defaults)


def _prime_healthy(m) -> None:
    """Put the bridge in the state where GNSS alignment is allowed to lock."""
    from slam.slam_bridge_module import DEGEN_NONE, LOC_TRACKING
    m._loc_state = LOC_TRACKING
    m._degen_level = DEGEN_NONE


# ──────────────────────────────────────────────────────────────────────────────
# Port / config surface
# ──────────────────────────────────────────────────────────────────────────────

class TestGnssPortSurface(unittest.TestCase):

    def test_gnss_odom_port_exists(self):
        m = _make_bridge()
        self.assertIn("gnss_odom", m.ports_in)

    def test_gnss_fusion_defaults(self):
        m = _make_bridge()
        self.assertTrue(m._gnss_fusion)
        self.assertIsNone(m._last_gnss_odom)
        self.assertIsNone(m._gnss_map_offset)
        self.assertEqual(m._gnss_fused_count, 0)

    def test_gnss_fusion_disabled_switch(self):
        m = _make_bridge(gnss_fusion=False)
        self.assertFalse(m._gnss_fusion)


# ──────────────────────────────────────────────────────────────────────────────
# Quality / freshness gating
# ──────────────────────────────────────────────────────────────────────────────

class TestGnssGating(unittest.TestCase):

    def test_no_gnss_passthrough(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        slam = _make_slam_odom(x=3.0, y=4.0, z=0.5)
        fused = m._fuse_odometry(slam)
        self.assertAlmostEqual(fused.pose.position.x, 3.0)
        self.assertAlmostEqual(fused.pose.position.y, 4.0)
        self.assertAlmostEqual(fused.pose.position.z, 0.5)
        self.assertIsNone(m._gnss_map_offset)

    def test_single_fix_ignored(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        m._on_gnss_odom(_make_gnss_odom(e=100.0, n=200.0,
                                         fix_type=GnssFixType.SINGLE))
        fused = m._fuse_odometry(_make_slam_odom(x=1.0, y=2.0))
        # No alignment, no blend
        self.assertIsNone(m._gnss_map_offset)
        self.assertAlmostEqual(fused.pose.position.x, 1.0)
        self.assertAlmostEqual(fused.pose.position.y, 2.0)

    def test_dgps_ignored(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        m._on_gnss_odom(_make_gnss_odom(fix_type=GnssFixType.DGPS))
        fused = m._fuse_odometry(_make_slam_odom(x=1.0))
        self.assertIsNone(m._gnss_map_offset)
        self.assertAlmostEqual(fused.pose.position.x, 1.0)

    def test_stale_gnss_ignored(self):
        m = _make_bridge(gnss_max_age_s=0.5)
        m.setup()
        _prime_healthy(m)

        m._on_gnss_odom(_make_gnss_odom(e=50.0, n=60.0))
        # Force last receive timestamp into the past
        m._last_gnss_rx_ts = time.time() - 5.0

        fused = m._fuse_odometry(_make_slam_odom(x=1.0, y=2.0))
        self.assertIsNone(m._gnss_map_offset)
        self.assertAlmostEqual(fused.pose.position.x, 1.0)

    def test_high_covariance_ignored(self):
        m = _make_bridge(gnss_max_std_m=0.5)
        m.setup()
        _prime_healthy(m)

        # cov_e + cov_n = 2.0 → h_std = 1.0, exceeds 0.5 limit
        m._on_gnss_odom(_make_gnss_odom(e=10.0, n=10.0,
                                         cov_e=1.0, cov_n=1.0))
        fused = m._fuse_odometry(_make_slam_odom(x=1.0, y=2.0))
        self.assertIsNone(m._gnss_map_offset)
        self.assertAlmostEqual(fused.pose.position.x, 1.0)


# ──────────────────────────────────────────────────────────────────────────────
# Alignment (one-shot offset lock)
# ──────────────────────────────────────────────────────────────────────────────

class TestGnssAlignment(unittest.TestCase):

    def test_first_rtk_fixed_locks_offset(self):
        m = _make_bridge()
        m.setup()
        _prime_healthy(m)

        # SLAM at (10, 20), GNSS ENU at (3, 4) → offset = (7, 16)
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        fused = m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))

        self.assertIsNotNone(m._gnss_map_offset)
        self.assertAlmostEqual(m._gnss_map_offset[0], 7.0)
        self.assertAlmostEqual(m._gnss_map_offset[1], 16.0)
        # First frame: pose not modified — only locks
        self.assertAlmostEqual(fused.pose.position.x, 10.0)
        self.assertAlmostEqual(fused.pose.position.y, 20.0)

    def test_no_alignment_when_slam_unhealthy(self):
        """Uninitialised / lost SLAM must not lock bad offset."""
        from slam.slam_bridge_module import LOC_UNINIT
        m = _make_bridge()
        m.setup()
        m._loc_state = LOC_UNINIT

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))
        self.assertIsNone(m._gnss_map_offset)

    def test_no_alignment_when_slam_degenerate(self):
        from slam.slam_bridge_module import DEGEN_CRITICAL, LOC_TRACKING
        m = _make_bridge()
        m.setup()
        m._loc_state = LOC_TRACKING
        m._degen_level = DEGEN_CRITICAL

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))
        self.assertIsNone(m._gnss_map_offset)


# ──────────────────────────────────────────────────────────────────────────────
# Weighted blend
# ──────────────────────────────────────────────────────────────────────────────

class TestGnssBlend(unittest.TestCase):

    def _lock_and_drift(self, m, lock_slam_xy=(10.0, 20.0),
                        lock_gnss_en=(3.0, 4.0)):
        """Lock alignment at healthy SLAM pose, return True when done."""
        _prime_healthy(m)
        m._on_gnss_odom(_make_gnss_odom(e=lock_gnss_en[0], n=lock_gnss_en[1]))
        m._fuse_odometry(_make_slam_odom(x=lock_slam_xy[0], y=lock_slam_xy[1]))
        self.assertIsNotNone(m._gnss_map_offset)

    def test_healthy_blend_small_alpha(self):
        m = _make_bridge(gnss_alpha_healthy=0.05)
        m.setup()
        self._lock_and_drift(m)  # offset = (7, 16)

        # Now SLAM drifts to (20, 30); GNSS still at (3, 4) → in map (10, 20)
        # Expected fused XY: (20*0.95 + 10*0.05, 30*0.95 + 20*0.05)
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        fused = m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0))

        self.assertAlmostEqual(fused.pose.position.x, 20.0 * 0.95 + 10.0 * 0.05)
        self.assertAlmostEqual(fused.pose.position.y, 30.0 * 0.95 + 20.0 * 0.05)

    def test_degraded_blend_large_alpha(self):
        from slam.slam_bridge_module import DEGEN_CRITICAL
        m = _make_bridge(gnss_alpha_healthy=0.05, gnss_alpha_degraded=0.5)
        m.setup()
        self._lock_and_drift(m)

        # Degrade SLAM state
        m._degen_level = DEGEN_CRITICAL

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        fused = m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0))

        # With α=0.5, fused = slam*0.5 + gnss_in_map*0.5
        self.assertAlmostEqual(fused.pose.position.x, 0.5 * 20.0 + 0.5 * 10.0)
        self.assertAlmostEqual(fused.pose.position.y, 0.5 * 30.0 + 0.5 * 20.0)

    def test_rtk_float_weight_discount(self):
        m = _make_bridge(gnss_alpha_healthy=0.1, gnss_rtk_float_scale=0.3)
        m.setup()
        self._lock_and_drift(m)

        # RTK_FLOAT sample → effective α = 0.1 * 0.3 = 0.03
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0,
                                         fix_type=GnssFixType.RTK_FLOAT))
        fused = m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0))

        eff = 0.03
        self.assertAlmostEqual(fused.pose.position.x,
                                20.0 * (1 - eff) + 10.0 * eff)

    def test_z_untouched(self):
        m = _make_bridge()
        m.setup()
        self._lock_and_drift(m)

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0, u=999.0))
        fused = m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0, z=1.5))
        # Z must stay with SLAM, GNSS up=999 must NOT leak in
        self.assertAlmostEqual(fused.pose.position.z, 1.5)

    def test_orientation_untouched(self):
        m = _make_bridge()
        m.setup()
        self._lock_and_drift(m)

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        slam = Odometry(
            pose=Pose(
                position=Vector3(x=20.0, y=30.0, z=0.0),
                orientation=Quaternion(x=0.1, y=0.2, z=0.3, w=0.9),
            ),
            ts=time.time(),
        )
        fused = m._fuse_odometry(slam)

        self.assertAlmostEqual(fused.pose.orientation.x, 0.1)
        self.assertAlmostEqual(fused.pose.orientation.y, 0.2)
        self.assertAlmostEqual(fused.pose.orientation.z, 0.3)
        self.assertAlmostEqual(fused.pose.orientation.w, 0.9)

    def test_fused_count_increments(self):
        m = _make_bridge()
        m.setup()
        self._lock_and_drift(m)
        self.assertEqual(m._gnss_fused_count, 0)

        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        m._fuse_odometry(_make_slam_odom(x=20.0, y=30.0))
        self.assertEqual(m._gnss_fused_count, 1)

        m._on_gnss_odom(_make_gnss_odom(e=3.5, n=4.5))
        m._fuse_odometry(_make_slam_odom(x=21.0, y=31.0))
        self.assertEqual(m._gnss_fused_count, 2)


# ──────────────────────────────────────────────────────────────────────────────
# Disable switch
# ──────────────────────────────────────────────────────────────────────────────

class TestGnssDisabled(unittest.TestCase):

    def test_disabled_bypasses_all_gnss_logic(self):
        m = _make_bridge(gnss_fusion=False)
        m.setup()
        _prime_healthy(m)

        # Even with perfectly valid RTK data, nothing should happen
        m._on_gnss_odom(_make_gnss_odom(e=3.0, n=4.0))
        fused = m._fuse_odometry(_make_slam_odom(x=10.0, y=20.0))

        self.assertIsNone(m._gnss_map_offset)
        self.assertEqual(m._gnss_fused_count, 0)
        self.assertAlmostEqual(fused.pose.position.x, 10.0)
        self.assertAlmostEqual(fused.pose.position.y, 20.0)


if __name__ == "__main__":
    unittest.main()
