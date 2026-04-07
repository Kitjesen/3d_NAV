"""Tests for lingtu.core.msgs.sensor — Image, CameraIntrinsics, PointCloud2, Imu."""

import numpy as np
import pytest

from core.msgs.geometry import Quaternion, Vector3
from core.msgs.sensor import (
    CameraIntrinsics,
    Image,
    ImageFormat,
    Imu,
    PointCloud2,
)

# ---------------------------------------------------------------------------
# Image
# ---------------------------------------------------------------------------

class TestImage:
    def test_from_numpy_properties(self):
        arr = np.zeros((480, 640, 3), dtype=np.uint8)
        img = Image.from_numpy(arr, ImageFormat.RGB)
        assert img.height == 480
        assert img.width == 640
        assert img.channels == 3
        assert img.format is ImageFormat.RGB

    def test_rgb_to_bgr_roundtrip(self):
        arr = np.random.randint(0, 255, (4, 6, 3), dtype=np.uint8)
        img = Image.from_numpy(arr, ImageFormat.RGB)
        bgr = img.to_bgr()
        assert bgr.format is ImageFormat.BGR
        back = bgr.to_rgb()
        np.testing.assert_array_equal(back.data, arr)

    def test_to_grayscale(self):
        arr = np.random.randint(0, 255, (10, 10, 3), dtype=np.uint8)
        gray = Image.from_numpy(arr, ImageFormat.RGB).to_grayscale()
        assert gray.format is ImageFormat.GRAY
        assert gray.channels == 1
        assert gray.data.shape == (10, 10)

    def test_crop(self):
        arr = np.arange(100).reshape(10, 10).astype(np.uint8)
        img = Image.from_numpy(arr, ImageFormat.GRAY)
        cropped = img.crop(2, 3, 4, 5)
        assert cropped.width == 4
        assert cropped.height == 5

    def test_encode_decode_roundtrip(self):
        arr = np.random.randint(0, 255, (8, 12, 3), dtype=np.uint8)
        img = Image(data=arr, format=ImageFormat.BGR, ts=1234.5, frame_id="cam0")
        raw = img.encode()
        img2 = Image.decode(raw)
        assert img2.format is ImageFormat.BGR
        assert img2.frame_id == "cam0"
        assert abs(img2.ts - 1234.5) < 1e-9
        np.testing.assert_array_equal(img2.data, arr)


# ---------------------------------------------------------------------------
# CameraIntrinsics
# ---------------------------------------------------------------------------

class TestCameraIntrinsics:
    def test_k_matrix(self):
        ci = CameraIntrinsics(fx=500, fy=500, cx=320, cy=240, width=640, height=480)
        K = ci.K_matrix
        assert K.shape == (3, 3)
        assert K[0, 0] == 500.0
        assert K[1, 1] == 500.0
        assert K[0, 2] == 320.0
        assert K[1, 2] == 240.0
        assert K[2, 2] == 1.0

    def test_project(self):
        ci = CameraIntrinsics(fx=500, fy=500, cx=320, cy=240, width=640, height=480)
        # Project the principal point at 2m depth → should be (0, 0, 2)
        pt = ci.project(320, 240, 2.0)
        assert abs(pt.x) < 1e-9
        assert abs(pt.y) < 1e-9
        assert abs(pt.z - 2.0) < 1e-9

    def test_encode_decode_roundtrip(self):
        ci = CameraIntrinsics(fx=525.0, fy=525.0, cx=319.5, cy=239.5,
                              width=640, height=480, depth_scale=0.001)
        raw = ci.encode()
        ci2 = CameraIntrinsics.decode(raw)
        assert ci2.fx == ci.fx
        assert ci2.fy == ci.fy
        assert ci2.width == ci.width
        assert ci2.height == ci.height
        assert abs(ci2.depth_scale - 0.001) < 1e-12

    def test_to_from_dict(self):
        ci = CameraIntrinsics(fx=500, fy=500, cx=320, cy=240, width=640, height=480)
        d = ci.to_dict()
        ci2 = CameraIntrinsics.from_dict(d)
        assert ci2.fx == ci.fx and ci2.width == ci.width


# ---------------------------------------------------------------------------
# PointCloud2
# ---------------------------------------------------------------------------

class TestPointCloud2:
    def test_from_numpy(self):
        pts = np.random.rand(100, 3).astype(np.float32)
        pc = PointCloud2.from_numpy(pts)
        assert pc.num_points == 100
        assert not pc.is_empty

    def test_empty_cloud(self):
        pc = PointCloud2()
        assert pc.is_empty
        assert pc.num_points == 0

    def test_transform_identity(self):
        pts = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
        pc = PointCloud2.from_numpy(pts)
        pc2 = pc.transform(np.eye(4))
        np.testing.assert_allclose(pc2.points, pts, atol=1e-5)

    def test_transform_translation(self):
        pts = np.array([[0, 0, 0]], dtype=np.float32)
        T = np.eye(4)
        T[:3, 3] = [1, 2, 3]
        pc2 = PointCloud2.from_numpy(pts).transform(T)
        np.testing.assert_allclose(pc2.points[0, :3], [1, 2, 3], atol=1e-5)

    def test_voxel_downsample(self):
        # 1000 points in a 1m cube → voxel_size=0.5 should give ≤8 voxels
        pts = np.random.rand(1000, 3).astype(np.float32)
        pc = PointCloud2.from_numpy(pts).voxel_downsample(0.5)
        assert pc.num_points <= 8
        assert pc.num_points > 0

    def test_encode_decode_roundtrip(self):
        pts = np.random.rand(50, 4).astype(np.float32)  # with intensity
        pc = PointCloud2(points=pts, ts=99.0, frame_id="lidar")
        raw = pc.encode()
        pc2 = PointCloud2.decode(raw)
        assert pc2.frame_id == "lidar"
        assert abs(pc2.ts - 99.0) < 1e-9
        np.testing.assert_array_equal(pc2.points, pts)

    def test_to_dict_bounds(self):
        pts = np.array([[0, 0, 0], [1, 2, 3]], dtype=np.float32)
        d = PointCloud2.from_numpy(pts).to_dict()
        assert d["num_points"] == 2
        np.testing.assert_allclose(d["bounds"]["min"], [0, 0, 0])
        np.testing.assert_allclose(d["bounds"]["max"], [1, 2, 3])

    def test_ros2_metadata_defaults(self):
        pts = np.random.rand(5, 4).astype(np.float32)
        pc = PointCloud2(points=pts, frame_id="lidar")
        assert pc.height == 1
        assert pc.width == 5
        assert pc.point_step == 16
        assert pc.row_step == 80
        assert pc.is_dense
        assert [f.name for f in pc.fields] == ["x", "y", "z", "intensity"]


# ---------------------------------------------------------------------------
# Imu
# ---------------------------------------------------------------------------

class TestImu:
    def test_encode_decode_roundtrip(self):
        imu = Imu(
            orientation=Quaternion(0.0, 0.0, 0.1, 0.995),
            angular_velocity=Vector3(0.01, 0.02, 0.03),
            linear_acceleration=Vector3(0.0, 0.0, 9.81),
            ts=1000.0,
            frame_id="imu0",
        )
        raw = imu.encode()
        imu2 = Imu.decode(raw)
        assert imu2.frame_id == "imu0"
        assert abs(imu2.ts - 1000.0) < 1e-9
        assert abs(imu2.orientation.w - 0.995) < 1e-9
        assert abs(imu2.linear_acceleration.z - 9.81) < 1e-9
