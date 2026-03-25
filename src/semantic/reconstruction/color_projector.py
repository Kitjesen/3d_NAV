"""
color_projector.py — RGB-D 帧 → 世界坐标系彩色点云

原理:
  每帧 RGB-D: 深度图反投影 → 相机坐标系点云 → 变换到世界坐标系 → 存入体素颜色表
  发布时: 查询体素颜色表 → 输出彩色点云

体素颜色表: dict[(ix, iy, iz)] -> (r, g, b)
  分辨率 VOXEL_SIZE (默认 5cm)，越新的观测覆盖越旧的
"""

import threading
from typing import Optional, Tuple

import numpy as np


VOXEL_SIZE = 0.05          # 体素边长 (m)
SUBSAMPLE_STEP = 4         # 像素降采样步长 (每 4 像素取一个，降低 CPU)
MIN_DEPTH = 0.3            # 最小有效深度 (m)
MAX_DEPTH = 6.0            # 最大有效深度 (m)


class ColorProjector:
    """维护一个全局体素颜色表，将 RGB-D 帧投影到世界坐标系。"""

    def __init__(self, voxel_size: float = VOXEL_SIZE):
        self._voxel_size = voxel_size
        # (ix, iy, iz) -> np.array([r, g, b], uint8)
        self._voxel_colors: dict = {}
        self._lock = threading.Lock()

    # ── 公开接口 ────────────────────────────────────────────────

    def update_from_frame(
        self,
        color_bgr: np.ndarray,      # (H, W, 3) BGR uint8
        depth_mm: np.ndarray,       # (H, W) uint16，单位 mm
        fx: float, fy: float, cx: float, cy: float,
        camera_to_world: np.ndarray,  # 4×4 float64 变换矩阵
    ) -> int:
        """用一帧 RGB-D 数据更新体素颜色表，返回新增/更新的体素数。"""
        h, w = depth_mm.shape

        # 降采样像素索引
        rows = np.arange(0, h, SUBSAMPLE_STEP)
        cols = np.arange(0, w, SUBSAMPLE_STEP)
        cc, rr = np.meshgrid(cols, rows)
        cc = cc.ravel()
        rr = rr.ravel()

        # 深度过滤
        depths = depth_mm[rr, cc].astype(np.float32) * 0.001  # mm → m
        valid = (depths > MIN_DEPTH) & (depths < MAX_DEPTH)
        cc, rr, depths = cc[valid], rr[valid], depths[valid]
        if len(depths) == 0:
            return 0

        # 反投影到相机坐标系
        x_cam = (cc - cx) * depths / fx
        y_cam = (rr - cy) * depths / fy
        z_cam = depths

        # 变换到世界坐标系
        ones = np.ones_like(z_cam)
        pts_cam = np.stack([x_cam, y_cam, z_cam, ones], axis=1)  # Nx4
        pts_world = (camera_to_world @ pts_cam.T).T[:, :3]        # Nx3

        # BGR → RGB 颜色
        colors_rgb = color_bgr[rr, cc][:, ::-1].copy()  # Nx3 uint8

        # 写入体素表（最新帧覆盖旧值）
        idx = (pts_world / self._voxel_size).astype(np.int32)
        keys = [tuple(idx[i]) for i in range(len(idx))]

        with self._lock:
            for k, c in zip(keys, colors_rgb):
                self._voxel_colors[k] = c

        return len(keys)

    def get_colored_cloud(self) -> np.ndarray:
        """返回所有体素的彩色点云，shape (N, 6)，列为 [x, y, z, r, g, b]。"""
        with self._lock:
            if not self._voxel_colors:
                return np.empty((0, 6), dtype=np.float32)
            keys = np.array(list(self._voxel_colors.keys()), dtype=np.float32)
            colors = np.array(list(self._voxel_colors.values()), dtype=np.float32)

        positions = keys * self._voxel_size
        return np.hstack([positions, colors])

    def colorize_slam_cloud(
        self, points_world: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        用体素颜色表为 SLAM 点云上色。

        Returns:
            xyzrgb: (N, 6) float32
            has_color: (N,) bool — 是否找到颜色
        """
        if len(points_world) == 0:
            return np.empty((0, 6), dtype=np.float32), np.array([], dtype=bool)

        idx = (points_world / self._voxel_size).astype(np.int32)
        colors = np.zeros((len(points_world), 3), dtype=np.float32)
        has_color = np.zeros(len(points_world), dtype=bool)

        with self._lock:
            for i in range(len(idx)):
                key = (idx[i, 0], idx[i, 1], idx[i, 2])
                c = self._voxel_colors.get(key)
                if c is not None:
                    colors[i] = c
                    has_color[i] = True

        xyzrgb = np.hstack([points_world.astype(np.float32), colors])
        return xyzrgb, has_color

    @property
    def voxel_count(self) -> int:
        with self._lock:
            return len(self._voxel_colors)

    def clear(self):
        with self._lock:
            self._voxel_colors.clear()
