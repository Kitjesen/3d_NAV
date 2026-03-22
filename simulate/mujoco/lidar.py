"""MuJoCo LiDAR — 封装 sim/sensors/livox_mid360.py
# Extracted from sim/bridge/nova_nav_bridge.py (scan_lidar function)
# Wraps sim/sensors/livox_mid360.py (不重写，直接复用)

优先使用 mujoco_ray_caster plugin（方案 A），
fallback 到 Python mj_multiRay（方案 B）。
"""
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from simulate.core.sensor import LidarConfig

# 将 sim/sensors/ 加入路径，以便 import livox_mid360
_SIM_SENSORS = Path(__file__).resolve().parent.parent.parent / "sim" / "sensors"
if str(_SIM_SENSORS) not in sys.path:
    sys.path.insert(0, str(_SIM_SENSORS))


class MuJoCoLidar:
    """MuJoCo LiDAR 传感器封装.

    直接复用 sim/sensors/livox_mid360.py 中的 LidarSensor 类，
    提供与 SimEngine 接口对齐的 scan() 方法，输出 (N, 4) XYZI。
    """

    def __init__(self, model, data, config: LidarConfig) -> None:
        """初始化 LiDAR 传感器.

        Args:
            model: mujoco.MjModel 实例
            data: mujoco.MjData 实例
            config: LiDAR 配置
        """
        self._model = model
        self._data = data
        self._config = config

        # 从 sim/sensors/livox_mid360.py 导入统一接口
        try:
            from livox_mid360 import LidarSensor
            self._sensor = LidarSensor(
                model, data,
                body_name=config.body_name,
                sensor_name=config.sensor_name,
            )
            self._use_livox_module = True
        except ImportError:
            # fallback：内联实现（避免 sim/sensors 路径问题）
            self._use_livox_module = False
            self._sensor = None
            self._init_fallback(model, data, config)

        # Geom 过滤掩码（只检测环境 geom group=1，跳过机器人 group=0）
        import mujoco
        self._geomgroup = np.zeros(6, dtype=np.uint8)
        self._geomgroup[config.geom_group] = 1

        # 预计算 fallback 射线方向（golden angle 螺旋）
        if not self._use_livox_module:
            self._ray_dirs_local = self._build_golden_spiral(config.n_rays)
            self._frame_idx = 0

        print(f'[MuJoCoLidar] Initialized: body={config.body_name}, '
              f'sensor={config.sensor_name}')

    def _init_fallback(self, model, data, config: LidarConfig) -> None:
        """内联 fallback 初始化（当 livox_mid360 模块不可用时）."""
        import mujoco
        self._body_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, config.body_name
        )
        if self._body_id < 0:
            self._body_id = 0
            print(f'[MuJoCoLidar] body "{config.body_name}" not found, using world origin')
        self._rng = np.random.default_rng(0)

    @staticmethod
    def _build_golden_spiral(n: int,
                              vfov_min: float = np.deg2rad(-7.0),
                              vfov_max: float = np.deg2rad(52.0)) -> np.ndarray:
        """构建黄金角螺旋射线方向（Livox 非重复花瓣模式近似）."""
        golden_ang = np.pi * (3 - np.sqrt(5))
        i = np.arange(n, dtype=np.float64)
        ha = (i * golden_ang) % (2 * np.pi)
        va = vfov_min + i / n * (vfov_max - vfov_min)
        cv = np.cos(va)
        return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])

    def scan(self) -> np.ndarray:
        """执行一次 LiDAR 扫描.

        Returns:
            (N, 4) float32 XYZI 点云（世界坐标系），intensity=100。
            若无有效点返回 (0, 4) 空数组。
        """
        if self._use_livox_module and self._sensor is not None:
            pts_xyz = self._sensor.scan()  # (N, 3) float32
        else:
            pts_xyz = self._scan_fallback()

        if len(pts_xyz) == 0:
            return np.zeros((0, 4), dtype=np.float32)

        intensity = np.full((len(pts_xyz), 1), 100.0, dtype=np.float32)
        return np.hstack([pts_xyz.astype(np.float32), intensity])  # (N, 4)

    def scan_xyz(self) -> np.ndarray:
        """仅返回 XYZ 点云，不含 intensity.

        Returns:
            (N, 3) float32 世界坐标系
        """
        if self._use_livox_module and self._sensor is not None:
            return self._sensor.scan()
        return self._scan_fallback()

    def _scan_fallback(self) -> np.ndarray:
        """内联 fallback 扫描（mj_multiRay + golden angle）.

        # Extracted from sim/bridge/nova_nav_bridge.py — scan_lidar()
        """
        import mujoco

        body_id = getattr(self, '_body_id', 0)
        pos = self._data.xpos[body_id].copy()
        rmat = self._data.xmat[body_id].reshape(3, 3).copy()

        # 每帧旋转偏移（模拟非重复覆盖）
        ang = self._frame_idx * 0.628
        self._frame_idx += 1
        c, s = np.cos(ang), np.sin(ang)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
        dirs_local = self._ray_dirs_local @ Rz.T     # (N, 3) sensor frame
        dirs_world = dirs_local @ rmat.T              # sensor → world frame

        n_rays = len(dirs_world)
        dist_out = np.full(n_rays, -1.0, dtype=np.float64)
        geomid_out = np.full(n_rays, -1, dtype=np.int32)

        mujoco.mj_multiRay(
            self._model, self._data,
            pos, dirs_world.flatten(),
            self._geomgroup, 1, -1,
            geomid_out, dist_out, None,
            n_rays, self._config.range_max
        )

        mask = dist_out > self._config.range_min
        if not mask.any():
            return np.zeros((0, 3), dtype=np.float32)

        pts = (pos + dirs_world[mask] * dist_out[mask, None]).astype(np.float32)
        if self._config.add_noise:
            pts += self._rng.normal(
                0, self._config.noise_std, pts.shape
            ).astype(np.float32)
        return pts

    def update_data(self, data) -> None:
        """更新 MjData 引用（仿真 reset 后调用）."""
        self._data = data
        if self._use_livox_module and self._sensor is not None:
            self._sensor.data = data
        if hasattr(self, '_frame_idx'):
            self._frame_idx = 0

    @property
    def config(self) -> LidarConfig:
        return self._config
