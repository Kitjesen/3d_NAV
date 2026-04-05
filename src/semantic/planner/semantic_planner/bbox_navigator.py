# Adapted from DimOS DetectionNavigation, Apache 2.0 License
# Original: dimos/navigation/visual_servoing/detection_navigation.py
# Original: dimos/navigation/bbox_navigation.py
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
BBox 视觉伺服导航 — 看到目标就追

借鉴 DimOS BBoxNavigationModule + DetectionNavigation

链路:
  1. 收到目标 bbox [x1,y1,x2,y2] (来自 VLM 或检测器)
  2. bbox 中心 + 深度图 → 3D 世界坐标
  3. PD 控制器: 角度误差 → angular_z, 距离误差 → linear_x
  4. 持续跟踪: 每帧更新 bbox → 更新 3D → 更新 cmd_vel
  5. 到达/丢失判定

不依赖 DimOS 框架，纯 Python + numpy，输出 TwistStamped 兼容 dict。
"""

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from core.config import get_config

logger = logging.getLogger(__name__)

# ── 状态常量 ──────────────────────────────────────────────────────────────────
STATE_IDLE = "idle"
STATE_TRACKING = "tracking"
STATE_ARRIVED = "arrived"
STATE_LOST = "lost"


@dataclass
class BBoxNavConfig:
    """视觉伺服导航参数。所有默认值与 DimOS DetectionNavigation 一致。"""
    target_distance: float = 1.5      # 目标保持距离 (m)
    min_distance: float = 0.8         # 太近→后退距离 (m)
    max_linear_speed: float = 0.5     # 最大线速度 (m/s)
    max_angular_speed: float = 0.8    # 最大角速度 (rad/s)
    linear_gain: float = 0.8          # 线速度 P 增益
    angular_gain: float = 1.5         # 角速度 P 增益
    lost_timeout: float = 5.0         # 丢失超时 (s)
    arrived_threshold: float = 0.3    # 到达判定距离容差 (m)
    depth_roi_ratio: float = 0.2      # 深度取样区域占 bbox 面积比例 (边长比)
    servo_takeover_distance: float = 3.0  # 视觉伺服接管距离 (m), 大于此值用规划栈


class BBoxNavigator:
    """
    通用视觉伺服导航器 — 任何物体都能追。

    使用方式:
      1. nav = BBoxNavigator()
      2. 每帧调用 nav.update(bbox, depth_image, camera_intrinsics, robot_pose)
      3. 使用返回的 linear_x / angular_z 发布 TwistStamped

    或低级调用:
      3d = nav.compute_3d_from_bbox(...)
      (lx, az) = nav.compute_cmd_vel(3d, robot_pose)
    """

    def __init__(self, config: Optional[BBoxNavConfig] = None) -> None:
        self._cfg = config if config is not None else BBoxNavConfig()
        self._state: str = STATE_IDLE
        self._last_bbox_time: float = 0.0
        self._target_bbox: Optional[list] = None
        self._target_3d: Optional[np.ndarray] = None   # [x, y, z] 世界坐标
        self._lock = threading.Lock()
        # Camera→body rotation from factory calibration
        cam_cfg = get_config().camera
        self._R_body_camera = cam_cfg.T_body_camera[:3, :3]

    # ── 公开接口 ─────────────────────────────────────────────────────────────

    def set_target_bbox(
        self, bbox: list, frame_timestamp: Optional[float] = None
    ) -> None:
        """
        设置/更新目标 bbox [x1, y1, x2, y2]。

        可选 frame_timestamp 用于精确丢失计时；
        不传则用调用时刻。
        """
        ts = frame_timestamp if frame_timestamp is not None else time.time()
        with self._lock:
            self._target_bbox = list(bbox)
            self._last_bbox_time = ts
            if self._state in (STATE_IDLE, STATE_LOST):
                self._state = STATE_TRACKING

    def compute_3d_from_bbox(
        self,
        bbox: list,
        depth_image: np.ndarray,
        fx: float,
        fy: float,
        cx: float,
        cy: float,
        robot_pose: Optional[tuple] = None,  # (x, y, yaw) 世界坐标系
    ) -> Optional[np.ndarray]:
        """
        bbox 中心 + 深度图 → 3D 坐标。

        步骤:
          1. 计算 bbox 中心像素 (u, v)
          2. 在中心区域 (bbox 边长 × depth_roi_ratio) 取中值深度，鲁棒去噪
          3. (u, v, depth) → 相机坐标系 3D 点 (X_c, Y_c, Z_c)
          4. 若提供 robot_pose，旋转到世界坐标系 (假设相机朝前，光轴 = 机体 x 轴)

        Returns:
            np.ndarray shape (3,) 或 None (深度无效时)
        """
        x1, y1, x2, y2 = float(bbox[0]), float(bbox[1]), float(bbox[2]), float(bbox[3])

        # bbox 中心像素
        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0

        # 中心采样区域 (bbox 边长 × roi_ratio)
        bw = x2 - x1
        bh = y2 - y1
        roi_ratio = self._cfg.depth_roi_ratio
        half_rw = max(bw * roi_ratio / 2.0, 2.0)
        half_rh = max(bh * roi_ratio / 2.0, 2.0)

        h_img, w_img = depth_image.shape[:2]
        r_x1 = int(max(u - half_rw, 0))
        r_y1 = int(max(v - half_rh, 0))
        r_x2 = int(min(u + half_rw, w_img - 1))
        r_y2 = int(min(v + half_rh, h_img - 1))

        # 中值深度 (鲁棒: 忽略 0 / NaN)
        roi = depth_image[r_y1:r_y2 + 1, r_x1:r_x2 + 1].astype(np.float32)
        valid = roi[(roi > 0) & np.isfinite(roi)]
        if valid.size < 3:
            # ROI 无有效深度，降级到单点
            d = float(depth_image[int(v), int(u)]) if (
                0 <= int(v) < h_img and 0 <= int(u) < w_img
            ) else 0.0
            if d <= 0 or not np.isfinite(d):
                return None
            depth_m = d
        else:
            depth_m = float(np.median(valid))

        # 深度单位归一化: Orbbec Gemini 335 以毫米输出
        # 若 depth_image 最大值 > 100，推断为毫米，转换为米
        if depth_m > 100.0:
            depth_m /= 1000.0

        if depth_m <= 0.0:
            return None

        # 相机坐标系 3D (光学坐标: X 右, Y 下, Z 前)
        X_c = (u - cx) / fx * depth_m
        Y_c = (v - cy) / fy * depth_m
        Z_c = depth_m

        if robot_pose is None:
            # 返回相机坐标系 (调用方自行转换)
            return np.array([X_c, Y_c, Z_c], dtype=float)

        # 转换到世界坐标系: camera 3D → body frame → world frame
        rx, ry, yaw = robot_pose
        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)

        # Camera optical → body frame via calibrated rotation
        cam_pt = np.array([X_c, Y_c, Z_c])
        body_pt = self._R_body_camera @ cam_pt

        wx = rx + cos_y * body_pt[0] - sin_y * body_pt[1]
        wy = ry + sin_y * body_pt[0] + cos_y * body_pt[1]
        wz = body_pt[2]

        return np.array([wx, wy, wz], dtype=float)

    def compute_cmd_vel(
        self,
        target_3d: np.ndarray,
        robot_pose: tuple,   # (x, y, yaw)
    ) -> tuple:
        """
        PD 控制器 — 从 DimOS DetectionNavigation._compute_twist_from_3d 移植。

        参考 detection_navigation.py:153-208:
          - 角度误差 → angular_z (P 控制，归一化到 [-π, π])
          - 距离误差 → linear_x (P 控制，转弯时线速度衰减)
          - 太近 → 后退

        Returns:
            (linear_x: float, angular_z: float)
        """
        cfg = self._cfg
        rx, ry, yaw = robot_pose

        dx = float(target_3d[0]) - rx
        dy = float(target_3d[1]) - ry
        distance = float(np.sqrt(dx * dx + dy * dy))

        # 角度误差，归一化到 [-π, π]
        angle_to_target = np.arctan2(dy, dx)
        angle_error = angle_to_target - yaw
        while angle_error > np.pi:
            angle_error -= 2.0 * np.pi
        while angle_error < -np.pi:
            angle_error += 2.0 * np.pi

        # 角速度
        angular_z = float(np.clip(
            angle_error * cfg.angular_gain,
            -cfg.max_angular_speed,
            cfg.max_angular_speed,
        ))

        # 线速度
        if distance < cfg.min_distance:
            # 太近，后退
            linear_x = -cfg.max_linear_speed * 0.6
        else:
            distance_error = distance - cfg.target_distance
            # 转弯时减速 (turn_factor: 0.3 ~ 1.0)
            turn_factor = 1.0 - min(abs(angle_error) / np.pi, 0.7)
            linear_x = float(np.clip(
                distance_error * cfg.linear_gain * turn_factor,
                -cfg.max_linear_speed,
                cfg.max_linear_speed,
            ))

        return linear_x, angular_z

    def update(
        self,
        bbox: list,
        depth_image: np.ndarray,
        camera_intrinsics: tuple,  # (fx, fy, cx, cy)
        robot_pose: tuple,         # (x, y, yaw)
    ) -> dict:
        """
        主更新方法 — 每帧调用一次。

        Args:
            bbox: [x1, y1, x2, y2] 像素坐标
            depth_image: 与 RGB 对齐的深度图 (H×W, uint16 mm 或 float32 m)
            camera_intrinsics: (fx, fy, cx, cy)
            robot_pose: (x, y, yaw) 世界坐标系

        Returns:
            {
                "state":     "tracking" | "arrived" | "lost" | "idle",
                "linear_x":  float,
                "angular_z": float,
                "target_3d": [x, y, z] or None,
                "distance":  float,
            }
        """
        now = time.time()
        fx, fy, cx, cy = camera_intrinsics

        # 更新 bbox 时间戳
        self.set_target_bbox(bbox, frame_timestamp=now)

        # 丢失检测 (在本帧有 bbox 时不会触发，但防止外部不调用 set_target_bbox)
        with self._lock:
            elapsed = now - self._last_bbox_time
            if elapsed > self._cfg.lost_timeout and self._state == STATE_TRACKING:
                self._state = STATE_LOST
                self._target_3d = None

        # 3D 投影
        target_3d = self.compute_3d_from_bbox(
            bbox, depth_image, fx, fy, cx, cy, robot_pose
        )

        if target_3d is None:
            with self._lock:
                current_state = self._state
            return {
                "state": current_state,
                "linear_x": 0.0,
                "angular_z": 0.0,
                "target_3d": None,
                "distance": 0.0,
            }

        with self._lock:
            self._target_3d = target_3d

        # 距离计算 (XY 平面)
        rx, ry, _ = robot_pose
        dx = float(target_3d[0]) - rx
        dy = float(target_3d[1]) - ry
        distance = float(np.sqrt(dx * dx + dy * dy))

        # 到达判定
        if distance <= self._cfg.arrived_threshold:
            with self._lock:
                self._state = STATE_ARRIVED
            return {
                "state": STATE_ARRIVED,
                "linear_x": 0.0,
                "angular_z": 0.0,
                "target_3d": target_3d.tolist(),
                "distance": distance,
            }

        # 控制量计算
        linear_x, angular_z = self.compute_cmd_vel(target_3d, robot_pose)

        with self._lock:
            self._state = STATE_TRACKING

        return {
            "state": STATE_TRACKING,
            "linear_x": linear_x,
            "angular_z": angular_z,
            "target_3d": target_3d.tolist(),
            "distance": distance,
        }

    def stop(self) -> None:
        """停止跟踪，清空状态。"""
        with self._lock:
            self._state = STATE_IDLE
            self._target_bbox = None
            self._target_3d = None
            self._last_bbox_time = 0.0

    def tick_lost_check(self) -> None:
        """
        外部定时调用 (可选)，在没有新 bbox 时推进丢失判定。

        如果 ROS2 节点在检测失败时不调用 update()，
        可每 0.5s 调用一次此方法。
        """
        with self._lock:
            if self._state == STATE_TRACKING:
                elapsed = time.time() - self._last_bbox_time
                if elapsed > self._cfg.lost_timeout:
                    logger.info(
                        "BBoxNavigator: target lost (no bbox for %.1fs)", elapsed
                    )
                    self._state = STATE_LOST
                    self._target_3d = None

    # ── 属性 ─────────────────────────────────────────────────────────────────

    @property
    def is_active(self) -> bool:
        """True 当且仅当正在跟踪目标（tracking 状态）。"""
        with self._lock:
            return self._state == STATE_TRACKING

    @property
    def state(self) -> str:
        """当前状态字符串: idle | tracking | arrived | lost。"""
        with self._lock:
            return self._state

    @property
    def target_3d(self) -> Optional[np.ndarray]:
        """最近一帧计算的目标 3D 坐标（世界坐标系），无则 None。"""
        with self._lock:
            return self._target_3d.copy() if self._target_3d is not None else None
