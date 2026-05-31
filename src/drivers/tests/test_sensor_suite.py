"""Tests for drivers.sensor_suite — SensorMount, SensorSuite, transforms, validation."""

from __future__ import annotations

import numpy as np
import pytest

from drivers.sensor_suite import (
    SensorMount,
    SensorSuite,
    SensorType,
    _euler_to_rot,
)

# ============================================================================
# SensorType enum
# ============================================================================

class TestSensorType:
    def test_values(self):
        assert SensorType.LIDAR.value == "lidar"
        assert SensorType.CAMERA.value == "camera"
        assert SensorType.DEPTH_CAMERA.value == "depth_camera"
        assert SensorType.IMU.value == "imu"
        assert SensorType.GNSS.value == "gnss"

    def test_is_str_enum(self):
        assert isinstance(SensorType.LIDAR, str)


# ============================================================================
# SensorMount
# ============================================================================

class TestSensorMount:
    def test_default_position(self):
        m = SensorMount(sensor_id="test", sensor_type=SensorType.LIDAR, frame_id="f")
        assert m.x == 0.0 and m.y == 0.0 and m.z == 0.0
        assert m.roll == 0.0 and m.pitch == 0.0 and m.yaw == 0.0

    def test_position_args(self):
        m = SensorMount(
            sensor_id="lidar", sensor_type=SensorType.LIDAR, frame_id="livox_frame",
            x=0.1, y=0.2, z=0.3, roll=0.01, pitch=0.02, yaw=0.03,
        )
        assert m.x == 0.1
        assert m.yaw == 0.03
        assert m.model is None
        assert m.fps is None

    def test_optional_fields(self):
        m = SensorMount(
            sensor_id="cam", sensor_type=SensorType.CAMERA, frame_id="cam",
            fps=30.0, resolution=(640, 480), model="TestCam",
            extra={"fov": 90},
        )
        assert m.fps == 30.0
        assert m.resolution == (640, 480)
        assert m.model == "TestCam"
        assert m.extra["fov"] == 90


# ============================================================================
# SensorSuite construction & accessors
# ============================================================================

class TestSensorSuiteAccessors:
    def make_sensors(self):
        return [
            SensorMount("lidar", SensorType.LIDAR, "livox_frame",
                        x=-0.011, y=-0.02329, z=0.04412),
            SensorMount("camera_front", SensorType.CAMERA, "camera_frame",
                        x=0.15, y=0.0, z=0.45),
            SensorMount("gnss", SensorType.GNSS, "gnss_frame",
                        z=0.45),
        ]

    def test_count(self):
        suite = SensorSuite(self.make_sensors())
        assert suite.count == 3

    def test_get_by_id(self):
        suite = SensorSuite(self.make_sensors())
        s = suite.get("lidar")
        assert s.sensor_id == "lidar"
        assert s.sensor_type == SensorType.LIDAR

    def test_get_unknown_raises(self):
        suite = SensorSuite(self.make_sensors())
        with pytest.raises(KeyError, match="not found"):
            suite.get("nonexistent")

    def test_by_type(self):
        suite = SensorSuite(self.make_sensors())
        lidars = suite.by_type(SensorType.LIDAR)
        assert len(lidars) == 1
        assert lidars[0].sensor_id == "lidar"

        cameras = suite.by_type(SensorType.CAMERA)
        assert len(cameras) == 1

        # No IMU registered
        assert suite.by_type(SensorType.IMU) == []

    def test_sensors_property_is_copy(self):
        suite = SensorSuite(self.make_sensors())
        sensors_copy = suite.sensors
        assert len(sensors_copy) == 3
        sensors_copy.clear()
        assert suite.count == 3  # original unchanged

    def test_frame_ids(self):
        suite = SensorSuite(self.make_sensors())
        fids = suite.frame_ids()
        assert fids == {
            "lidar": "livox_frame",
            "camera_front": "camera_frame",
            "gnss": "gnss_frame",
        }


# ============================================================================
# T_body_sensor transform
# ============================================================================

class TestBodySensorTransform:
    def test_identity_transform(self):
        """Sensor at origin with no rotation → identity matrix."""
        m = SensorMount("s", SensorType.LIDAR, "f")
        suite = SensorSuite([m])
        T = suite.T_body_sensor("s")
        assert T.shape == (4, 4)
        np.testing.assert_allclose(T, np.eye(4), atol=1e-12)

    def test_translation_only(self):
        """Sensor at (1, 2, 3) with identity rotation."""
        m = SensorMount("s", SensorType.LIDAR, "f", x=1.0, y=2.0, z=3.0)
        suite = SensorSuite([m])
        T = suite.T_body_sensor("s")
        expected = np.eye(4)
        expected[:3, 3] = [-1.0, -2.0, -3.0]  # -R @ t with R=I
        np.testing.assert_allclose(T, expected, atol=1e-12)

    def test_rotation_only(self):
        """90 deg yaw → rotation submatrix is correct."""
        import math
        m = SensorMount("s", SensorType.LIDAR, "f", yaw=math.pi / 2)
        suite = SensorSuite([m])
        T = suite.T_body_sensor("s")
        # yaw=90° → rotation about Z
        expected = np.array([
            [0, -1, 0, 0],
            [1,  0, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1],
        ], dtype=np.float64)
        np.testing.assert_allclose(T, expected, atol=1e-12)

    def test_rotation_matrix_orthogonal(self):
        """Rotation submatrix is orthogonal (R^T @ R = I)."""
        import math
        m = SensorMount("s", SensorType.LIDAR, "f",
                        roll=0.1, pitch=-0.2, yaw=math.pi / 3)
        suite = SensorSuite([m])
        T = suite.T_body_sensor("s")
        R = T[:3, :3]
        np.testing.assert_allclose(R.T @ R, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(np.linalg.det(R), 1.0, atol=1e-12)


# ============================================================================
# T_sensor1_sensor2
# ============================================================================

class TestSensorSensorTransform:
    def test_same_sensor_is_identity(self):
        """Transform from sensor to itself = identity."""
        sensors = [
            SensorMount("a", SensorType.LIDAR, "f", x=1.0),
        ]
        suite = SensorSuite(sensors)
        T = suite.T_sensor1_sensor2("a", "a")
        np.testing.assert_allclose(T, np.eye(4), atol=1e-12)

    def test_two_sensors_different_position(self):
        """Two sensors at different positions on the body."""
        sensors = [
            SensorMount("front", SensorType.LIDAR, "f", x=0.5, z=0.3),
            SensorMount("rear", SensorType.LIDAR, "r", x=-0.5, z=0.3),
        ]
        suite = SensorSuite(sensors)
        T = suite.T_sensor1_sensor2("front", "rear")
        # front→rear = inv(T_body_front) @ T_body_rear
        # With R=I: T_body_s = [I | -t]; inv = [I | t]
        # So T_front_rear = [I | t_front] @ [I | -t_rear] = [I | t_front - t_rear]
        expected = np.eye(4)
        expected[:3, 3] = [0.5 - (-0.5), 0, 0.3 - 0.3]  # [1.0, 0, 0]
        np.testing.assert_allclose(T, expected, atol=1e-12)


# ============================================================================
# validate
# ============================================================================

class TestValidate:
    def test_valid_returns_empty(self):
        sensors = [
            SensorMount("a", SensorType.LIDAR, "frame_a"),
            SensorMount("b", SensorType.CAMERA, "frame_b"),
        ]
        suite = SensorSuite(sensors)
        assert suite.validate() == []

    def test_empty_sensor_id_detected(self):
        sensors = [
            SensorMount("", SensorType.LIDAR, "f"),
        ]
        suite = SensorSuite(sensors)
        issues = suite.validate()
        assert any("empty sensor_id" in issue for issue in issues)

    def test_duplicate_sensor_id_detected(self):
        sensors = [
            SensorMount("dup", SensorType.LIDAR, "f1"),
            SensorMount("dup", SensorType.CAMERA, "f2"),
        ]
        suite = SensorSuite(sensors)
        issues = suite.validate()
        assert any("duplicate sensor_id" in issue for issue in issues)

    def test_empty_frame_id_detected(self):
        sensors = [
            SensorMount("s1", SensorType.LIDAR, ""),
        ]
        suite = SensorSuite(sensors)
        issues = suite.validate()
        assert any("empty frame_id" in issue for issue in issues)

    def test_multiple_issues(self):
        sensors = [
            SensorMount("s1", SensorType.LIDAR, ""),
            SensorMount("dup", SensorType.CAMERA, "f"),
            SensorMount("dup", SensorType.GNSS, ""),
        ]
        suite = SensorSuite(sensors)
        issues = suite.validate()
        assert len(issues) >= 2


# ============================================================================
# _euler_to_rot helper
# ============================================================================

class TestEulerToRot:
    def test_identity(self):
        R = _euler_to_rot(0, 0, 0)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_yaw_only(self):
        import math
        R = _euler_to_rot(0, 0, math.pi / 2)
        expected = np.array([
            [0, -1, 0],
            [1,  0, 0],
            [0,  0, 1],
        ])
        np.testing.assert_allclose(R, expected, atol=1e-12)

    def test_roll_only(self):
        import math
        R = _euler_to_rot(math.pi / 2, 0, 0)
        expected = np.array([
            [1,  0, 0],
            [0,  0, -1],
            [0,  1,  0],
        ])
        np.testing.assert_allclose(R, expected, atol=1e-12)
