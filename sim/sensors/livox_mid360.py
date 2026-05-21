"""Livox MID-360 LiDAR simulation helpers for MuJoCo.

The preferred path is a ``mujoco_ray_caster`` plugin sensor when present.
When the plugin is unavailable, this module falls back to ``mj_multiRay``
using a deterministic Livox-like non-repetitive scan pattern.
"""

from __future__ import annotations

from typing import Optional

import numpy as np


def read_plugin_lidar(model, data, sensor_name: str = "lidar_mid360") -> Optional[np.ndarray]:
    """Read point cloud data from a ray-caster plugin sensor.

    Returns world-frame XYZ points as ``float32`` or ``None`` when the plugin
    sensor is not available.
    """
    try:
        import mujoco

        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
        if sensor_id < 0:
            return None

        plugin_id = model.sensor_plugin[sensor_id]
        if plugin_id < 0:
            return None

        state_idx = model.plugin_stateadr[plugin_id]
        h_rays = int(data.plugin_state[state_idx])
        v_rays = int(data.plugin_state[state_idx + 1])
        n_pts = h_rays * v_rays
        if n_pts <= 0:
            return None

        adr = model.sensor_adr[sensor_id]
        raw = data.sensordata[adr : adr + n_pts * 3]
        pts = raw.reshape(-1, 3).astype(np.float32)
        valid = np.any(pts != 0, axis=1) & ~np.any(np.isnan(pts), axis=1)
        return pts[valid]
    except Exception:
        return None


class LivoxMid360Fallback:
    """Pure-Python MID-360 fallback backed by MuJoCo ``mj_multiRay``."""

    HFOV = 2 * np.pi
    VFOV_MIN = np.deg2rad(-7.0)
    VFOV_MAX = np.deg2rad(52.0)
    RANGE_MIN = 0.10
    RANGE_MAX = 70.0
    NOISE_STD = 0.02
    N_RAYS = 6400
    GOLDEN_ANG = np.pi * (3 - np.sqrt(5))
    ORIGIN_FORWARD_OFFSET_M = 0.20

    def __init__(
        self,
        model,
        data,
        body_name: str = "lidar_link",
        add_noise: bool = True,
        seed: int = 0,
    ):
        self.model = model
        self.data = data
        self.add_noise = add_noise
        self.rng = np.random.default_rng(seed)
        self._frame = 0

        import mujoco

        self._body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if self._body_id < 0:
            self._body_id = 0
            print(f'[LivoxFallback] body "{body_name}" not found, using world origin')

        self._geomgroup = np.ones(6, dtype=np.uint8)
        self._dirs_local = self._build_pattern(self.N_RAYS)
        print(
            "[LivoxFallback] MID-360 pattern: "
            f"{self.N_RAYS} rays, VFOV=[{np.degrees(self.VFOV_MIN):.0f},"
            f"{np.degrees(self.VFOV_MAX):.0f}] deg"
        )

    def _build_pattern(self, n: int) -> np.ndarray:
        i = np.arange(n, dtype=np.float64)
        ha = (i * self.GOLDEN_ANG) % (2 * np.pi)
        va = self.VFOV_MIN + i / n * (self.VFOV_MAX - self.VFOV_MIN)
        cv = np.cos(va)
        return np.column_stack([cv * np.cos(ha), cv * np.sin(ha), np.sin(va)])

    def scan(self) -> np.ndarray:
        import mujoco

        rmat = self.data.xmat[self._body_id].reshape(3, 3).copy()
        pos = self.data.xpos[self._body_id].copy()
        pos = pos + rmat[:, 0] * self.ORIGIN_FORWARD_OFFSET_M

        ang = self._frame * 0.628
        self._frame += 1
        c, s = np.cos(ang), np.sin(ang)
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)
        dirs = self._dirs_local @ rz.T @ rmat.T

        dist_out = np.full(self.N_RAYS, -1.0, dtype=np.float64)
        geomid_out = np.full(self.N_RAYS, -1, dtype=np.int32)

        mujoco.mj_multiRay(
            self.model,
            self.data,
            pos,
            dirs.flatten(),
            self._geomgroup,
            1,
            self._body_id,
            geomid_out,
            dist_out,
            None,
            self.N_RAYS,
            self.RANGE_MAX,
        )

        valid = (dist_out >= self.RANGE_MIN) & (dist_out <= self.RANGE_MAX)
        if not valid.any():
            return np.zeros((0, 3), dtype=np.float32)

        pts = (pos + dirs[valid] * dist_out[valid, None]).astype(np.float32)
        if self.add_noise:
            pts += self.rng.normal(0, self.NOISE_STD, pts.shape).astype(np.float32)
        return pts


class LidarSensor:
    """Unified LiDAR interface: plugin first, ``mj_multiRay`` fallback second."""

    def __init__(
        self,
        model,
        data,
        body_name: str = "lidar_link",
        sensor_name: str = "lidar_mid360",
    ):
        self.model = model
        self.data = data
        self.sensor_name = sensor_name
        self._fallback = None

        try:
            import mujoco

            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
            if sid >= 0 and model.sensor_plugin[sid] >= 0:
                self._use_plugin = True
                print(f"[LidarSensor] Using mujoco_ray_caster plugin ({sensor_name})")
                return
        except Exception:
            pass

        self._use_plugin = False
        self._fallback = LivoxMid360Fallback(model, data, body_name=body_name)
        print("[LidarSensor] Plugin not found, using Python fallback (mj_multiRay)")

    def scan(self) -> np.ndarray:
        """Return world-frame XYZ points as ``float32`` with shape ``(N, 3)``."""
        if self._use_plugin:
            pts = read_plugin_lidar(self.model, self.data, self.sensor_name)
            if pts is not None and len(pts) > 0:
                return pts
        if self._fallback is not None:
            return self._fallback.scan()
        return np.zeros((0, 3), dtype=np.float32)
