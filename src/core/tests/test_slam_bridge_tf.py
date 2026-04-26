"""Regression tests for SlamBridge map↔odom TF transforms.

Context: over the past 500 commits, `map→odom` TF handling has been fixed
FIVE times (see b666379/d79c409/8ab392d/788de37/19ad11f/d62f5bd). The root
cause was contract ambiguity about "who transforms". The contract is now:
SlamBridge is the ONE place. These tests pin that contract down so future
refactors don't re-drift.

We test the three pure-Python TF methods on a bare-instance (no Module
setup / no ROS2 subscribe) by binding the unbound methods onto a minimal
namespace holding only `_T_map_odom`.
"""
from __future__ import annotations

import math

import numpy as np
import pytest

from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from slam.slam_bridge_module import SlamBridgeModule

# ── Harness: bind unbound methods to a minimal holder ─────────────────────

class _TFHolder:
    """Minimal stand-in for SlamBridgeModule — owns `_T_map_odom` only."""
    _T_map_odom: np.ndarray | None = None


def _make_holder() -> _TFHolder:
    """Return an object that responds to the three TF methods."""
    import types as _t
    h = _TFHolder()
    h._cache_map_odom_tf = _t.MethodType(SlamBridgeModule._cache_map_odom_tf, h)
    h._apply_map_odom_to_points = _t.MethodType(
        SlamBridgeModule._apply_map_odom_to_points, h)
    h._apply_map_odom_to_odometry = _t.MethodType(
        SlamBridgeModule._apply_map_odom_to_odometry, h)
    return h


def _yaw_quat(yaw_rad: float) -> tuple[float, float, float, float]:
    """Return (qx, qy, qz, qw) for a pure-yaw rotation."""
    return (0.0, 0.0, math.sin(yaw_rad / 2), math.cos(yaw_rad / 2))


# ── TF cache: identity + rigid ───────────────────────────────────────────

def test_cache_identity_tf():
    h = _make_holder()
    h._cache_map_odom_tf(0, 0, 0, 0, 0, 0, 1)  # identity quat
    T = h._T_map_odom
    np.testing.assert_allclose(T, np.eye(4), atol=1e-12)


def test_cache_translation_only():
    h = _make_holder()
    h._cache_map_odom_tf(1.0, 2.0, 3.0, 0, 0, 0, 1)
    assert h._T_map_odom[:3, 3].tolist() == [1.0, 2.0, 3.0]
    np.testing.assert_allclose(h._T_map_odom[:3, :3], np.eye(3), atol=1e-12)


def test_cache_rotation_matrix_is_orthogonal():
    """T[:3,:3] must be SO(3) for any input quat — det=1, R @ R.T = I."""
    h = _make_holder()
    qx, qy, qz, qw = _yaw_quat(math.radians(37.5))
    h._cache_map_odom_tf(0, 0, 0, qx, qy, qz, qw)
    R = h._T_map_odom[:3, :3]
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-10)
    assert abs(np.linalg.det(R) - 1.0) < 1e-10


# ── Points transform: identity, translation, rotation, huge cloud ────────

def test_points_noop_when_tf_missing():
    h = _make_holder()
    pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
    out = h._apply_map_odom_to_points(pts)
    assert out is pts  # same object — explicit no-op


def test_points_empty_array_early_return():
    h = _make_holder()
    h._cache_map_odom_tf(10, 20, 30, 0, 0, 0, 1)
    empty = np.zeros((0, 3), dtype=np.float32)
    out = h._apply_map_odom_to_points(empty)
    assert out is empty


def test_points_translation_only():
    h = _make_holder()
    h._cache_map_odom_tf(10.0, -5.0, 2.0, 0, 0, 0, 1)
    pts = np.array([[0, 0, 0], [1, 1, 1]], dtype=np.float32)
    out = h._apply_map_odom_to_points(pts)
    np.testing.assert_allclose(out, [[10, -5, 2], [11, -4, 3]], atol=1e-5)


def test_points_90deg_yaw():
    """90° yaw rotates +X into +Y."""
    h = _make_holder()
    qx, qy, qz, qw = _yaw_quat(math.pi / 2)
    h._cache_map_odom_tf(0, 0, 0, qx, qy, qz, qw)
    pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    out = h._apply_map_odom_to_points(pts)
    np.testing.assert_allclose(out[0], [0.0, 1.0, 0.0], atol=1e-5)


def test_points_vectorized_large_cloud():
    """Verify the @ R.T + t path works for 10k points in one shot."""
    h = _make_holder()
    h._cache_map_odom_tf(5.0, 5.0, 0, *_yaw_quat(math.radians(45)))
    rng = np.random.default_rng(42)
    pts = rng.uniform(-10, 10, size=(10_000, 3)).astype(np.float32)
    out = h._apply_map_odom_to_points(pts)
    assert out.shape == pts.shape
    assert out.dtype == np.float32
    # First point should differ from input (not accidentally passthrough)
    assert not np.allclose(out[0], pts[0])


# ── Odometry transform: position + quaternion ────────────────────────────

def _make_odom(x: float, y: float, z: float,
               qx: float = 0, qy: float = 0, qz: float = 0, qw: float = 1) -> Odometry:
    return Odometry(
        pose=Pose(
            position=Vector3(x, y, z),
            orientation=Quaternion(qx, qy, qz, qw),
        ),
        twist=Twist(),
    )


def test_odom_noop_when_tf_missing():
    h = _make_holder()
    odom = _make_odom(1, 2, 3)
    out = h._apply_map_odom_to_odometry(odom)
    assert out is odom
    assert odom.pose.position.x == 1
    assert odom.pose.position.y == 2


def test_odom_translation_applies():
    h = _make_holder()
    h._cache_map_odom_tf(100, 200, 300, 0, 0, 0, 1)
    odom = _make_odom(1, 2, 3)
    h._apply_map_odom_to_odometry(odom)
    assert odom.pose.position.x == pytest.approx(101)
    assert odom.pose.position.y == pytest.approx(202)
    assert odom.pose.position.z == pytest.approx(303)


def test_odom_yaw_composition():
    """If TF is 90° yaw, a body already at 0° becomes 90° yaw in map frame.

    This was specifically broken in commit 788de37 — BBS3D align produces
    ~-104° yaw and we were leaving map.yaw = odom.yaw = 0, which made the
    URDF point the wrong way in Web.
    """
    h = _make_holder()
    tf_yaw = math.pi / 2
    h._cache_map_odom_tf(0, 0, 0, *_yaw_quat(tf_yaw))
    odom = _make_odom(1, 0, 0, *_yaw_quat(0))  # body at origin, yaw=0
    h._apply_map_odom_to_odometry(odom)
    # Position: TF rotates +X into +Y → map pos = (0, 1, 0)
    assert odom.pose.position.x == pytest.approx(0, abs=1e-6)
    assert odom.pose.position.y == pytest.approx(1, abs=1e-6)
    # Yaw: 0 + 90° = 90° → z-component of quat = sin(45°), w = cos(45°)
    assert odom.pose.orientation.z == pytest.approx(math.sin(math.pi / 4), abs=1e-6)
    assert odom.pose.orientation.w == pytest.approx(math.cos(math.pi / 4), abs=1e-6)


def test_odom_quat_stays_unit_after_compose():
    """|quat| must equal 1 after any composition — Shepperd stability check."""
    h = _make_holder()
    # Use an arbitrary TF: 30° yaw + translation
    h._cache_map_odom_tf(1.5, -2.3, 0.5, *_yaw_quat(math.radians(30)))
    # Body at arbitrary orientation (roll+pitch+yaw via roll rotation):
    half = math.radians(25) / 2
    odom = _make_odom(0, 0, 0, math.sin(half), 0, 0, math.cos(half))
    h._apply_map_odom_to_odometry(odom)
    q = odom.pose.orientation
    norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
    assert norm == pytest.approx(1.0, abs=1e-10)


def test_odom_identity_tf_is_noop_on_values():
    h = _make_holder()
    h._cache_map_odom_tf(0, 0, 0, 0, 0, 0, 1)
    orig = _make_odom(3.14, 2.71, 1.41, *_yaw_quat(math.radians(-104)))
    # Copy values out BEFORE transform mutates the object
    px, py, pz = orig.pose.position.x, orig.pose.position.y, orig.pose.position.z
    qx, qy, qz, qw = (orig.pose.orientation.x, orig.pose.orientation.y,
                      orig.pose.orientation.z, orig.pose.orientation.w)
    h._apply_map_odom_to_odometry(orig)
    # Identity TF should not change anything
    assert orig.pose.position.x == pytest.approx(px)
    assert orig.pose.position.y == pytest.approx(py)
    assert orig.pose.position.z == pytest.approx(pz)
    assert orig.pose.orientation.x == pytest.approx(qx, abs=1e-10)
    assert orig.pose.orientation.y == pytest.approx(qy, abs=1e-10)
    assert orig.pose.orientation.z == pytest.approx(qz, abs=1e-10)
    assert orig.pose.orientation.w == pytest.approx(qw, abs=1e-10)


# ── Round-trip: transform and inverse-transform gives back original ──────

def test_odom_tf_then_inverse_tf_roundtrips():
    """Apply TF then inverse TF → back to original pose. Catches sign/order bugs."""
    h_fwd = _make_holder()
    h_inv = _make_holder()
    tf_yaw = math.radians(47)
    tx, ty, tz = 3.0, -1.5, 0.2
    qx, qy, qz, qw = _yaw_quat(tf_yaw)
    h_fwd._cache_map_odom_tf(tx, ty, tz, qx, qy, qz, qw)

    # Inverse of translation+yaw: rotate -yaw, then translate by -R^(-1)·t
    inv_yaw = -tf_yaw
    iqx, iqy, iqz, iqw = _yaw_quat(inv_yaw)
    # Rotate -t by inverse rotation to get inverse translation
    c, s = math.cos(inv_yaw), math.sin(inv_yaw)
    itx = -(c * tx - s * ty)
    ity = -(s * tx + c * ty)
    itz = -tz
    h_inv._cache_map_odom_tf(itx, ity, itz, iqx, iqy, iqz, iqw)

    orig = _make_odom(1.0, 2.0, 0.5, *_yaw_quat(math.radians(15)))
    h_fwd._apply_map_odom_to_odometry(orig)
    h_inv._apply_map_odom_to_odometry(orig)
    assert orig.pose.position.x == pytest.approx(1.0, abs=1e-5)
    assert orig.pose.position.y == pytest.approx(2.0, abs=1e-5)
    assert orig.pose.position.z == pytest.approx(0.5, abs=1e-5)


def test_points_and_odom_consistent():
    """Same TF applied to a point via the two methods must agree."""
    h = _make_holder()
    h._cache_map_odom_tf(2.0, 3.0, 1.0, *_yaw_quat(math.radians(60)))

    # Via point-cloud transform
    pts = np.array([[1.0, 0.0, 0.0]], dtype=np.float32)
    pt_out = h._apply_map_odom_to_points(pts)[0]

    # Via odometry transform (position only)
    odom = _make_odom(1.0, 0.0, 0.0)
    h._apply_map_odom_to_odometry(odom)

    assert odom.pose.position.x == pytest.approx(float(pt_out[0]), abs=1e-5)
    assert odom.pose.position.y == pytest.approx(float(pt_out[1]), abs=1e-5)
    assert odom.pose.position.z == pytest.approx(float(pt_out[2]), abs=1e-5)
