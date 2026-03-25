"""MuJoCo LiDAR wrapper — wraps sim/sensors/livox_mid360.py
# Extracted from sim/bridge/nova_nav_bridge.py (scan_lidar function)
# Wraps sim/sensors/livox_mid360.py (reuse as-is, no reimplementation)

Prefers mujoco_ray_caster plugin (method A),
falls back to Python mj_multiRay (method B).
"""
import sys
from pathlib import Path
from typing import Optional

import numpy as np

from sim.engine.core.sensor import LidarConfig

# Add sim/sensors/ to path so livox_mid360 can be imported
_SIM_SENSORS = Path(__file__).resolve().parent.parent.parent / "sim" / "sensors"
if str(_SIM_SENSORS) not in sys.path:
    sys.path.insert(0, str(_SIM_SENSORS))


class MuJoCoLidar:
    """MuJoCo LiDAR sensor wrapper.

    Reuses sim/sensors/livox_mid360.py LidarSensor class directly,
    providing a scan() method aligned with the SimEngine interface,
    returning (N, 4) XYZI point cloud.
    """

    def __init__(self, model, data, config: LidarConfig) -> None:
        """Initialize LiDAR sensor.

        Args:
            model: mujoco.MjModel instance
            data: mujoco.MjData instance
            config: LiDAR configuration
        """
        self._model = model
        self._data = data
        self._config = config

        # Import unified interface from sim/sensors/livox_mid360.py
        try:
            from livox_mid360 import LidarSensor
            self._sensor = LidarSensor(
                model, data,
                body_name=config.body_name,
                sensor_name=config.sensor_name,
            )
            self._use_livox_module = True
        except ImportError:
            # Fallback: inline implementation (avoids sim/sensors path issues)
            self._use_livox_module = False
            self._sensor = None
            self._init_fallback(model, data, config)

        # Geom filter mask (only detect environment geom group=1, skip robot group=0)
        import mujoco
        self._geomgroup = np.zeros(6, dtype=np.uint8)
        self._geomgroup[config.geom_group] = 1

        # Pre-compute fallback ray directions (golden angle spiral)
        if not self._use_livox_module:
            self._ray_dirs_local = self._build_golden_spiral(config.n_rays)
            self._frame_idx = 0

        print(f'[MuJoCoLidar] Initialized: body={config.body_name}, '
              f'sensor={config.sensor_name}')

    def _init_fallback(self, model, data, config: LidarConfig) -> None:
        """Inline fallback initialization (when livox_mid360 module is unavailable)."""
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
        """Build golden-angle spiral ray directions (approximates Livox non-repetitive petal pattern)."""
        golden_ang = np.pi * (3 - np.sqrt(5))
        i = np.arange(n, dtype=np.float64)
        ha = (i * golden_ang) % (2 * np.pi)
        va = vfov_min + i / n * (vfov_max - vfov_min)
        cv = np.cos(va)
        return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])

    def scan(self) -> np.ndarray:
        """Perform one LiDAR scan.

        Returns:
            (N, 4) float32 XYZI point cloud (world frame), intensity=100.
            Returns (0, 4) empty array if no valid points.
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
        """Return XYZ-only point cloud without intensity.

        Returns:
            (N, 3) float32 world frame
        """
        if self._use_livox_module and self._sensor is not None:
            return self._sensor.scan()
        return self._scan_fallback()

    def _scan_fallback(self) -> np.ndarray:
        """Inline fallback scan (mj_multiRay + golden angle spiral).

        # Extracted from sim/bridge/nova_nav_bridge.py scan_lidar()
        """
        import mujoco

        body_id = getattr(self, '_body_id', 0)
        pos = self._data.xpos[body_id].copy()
        rmat = self._data.xmat[body_id].reshape(3, 3).copy()

        # Per-frame rotation offset (simulates non-repetitive coverage)
        ang = self._frame_idx * 0.628
        self._frame_idx += 1
        c, s = np.cos(ang), np.sin(ang)
        Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
        dirs_local = self._ray_dirs_local @ Rz.T     # (N, 3) sensor frame
        dirs_world = dirs_local @ rmat.T              # sensor -> world frame

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
        """Update MjData reference (call after simulation reset)."""
        self._data = data
        if self._use_livox_module and self._sensor is not None:
            self._sensor.data = data
        if hasattr(self, '_frame_idx'):
            self._frame_idx = 0

    @property
    def config(self) -> LidarConfig:
        return self._config
