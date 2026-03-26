"""KinematicSimModule — deterministic robot simulation with MuJoCo raycasting.

Robot moves exactly according to cmd_vel (no RL policy, no physics instability).
LiDAR uses MuJoCo raycasting for realistic obstacle detection.
Best of both worlds: precise motion + realistic sensing.

This is what dimos effectively does — the robot SDK guarantees velocity tracking,
sim only needs to provide sensor data.

Usage::

    bp.add(KinematicSimModule, world="building_scene", obstacles=[...])
"""
from __future__ import annotations

import logging
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import Twist, Vector3, Pose, Quaternion
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud
from core.registry import register

logger = logging.getLogger(__name__)

_SIM_ROOT = Path(__file__).resolve().parents[3] / "sim"


@register("driver", "kinematic_sim", description="Kinematic sim with MuJoCo LiDAR")
class KinematicSimModule(Module, layer=1):
    """Deterministic kinematic simulation + MuJoCo raycasting for LiDAR.

    Robot: cmd_vel → exact position integration (no physics, no RL policy)
    LiDAR: MuJoCo mj_multiRay on scene geometry (realistic obstacle detection)
    """

    # Inputs
    cmd_vel: In[Twist]
    stop_signal: In[int]

    # Outputs
    odometry: Out[Odometry]
    lidar_cloud: Out[PointCloud]
    alive: Out[bool]

    def __init__(
        self,
        world: str = "building_scene",
        obstacles: list = None,
        start_pos: tuple = (2.0, 3.0, 0.35),
        sim_rate: float = 50.0,
        lidar_rays: int = 360,
        lidar_range: float = 20.0,
        lidar_rate: float = 10.0,
        **kw,
    ):
        super().__init__(**kw)
        self._world = world
        self._obstacles = obstacles or []
        self._start_pos = np.array(start_pos, dtype=np.float64)
        self._sim_rate = sim_rate
        self._lidar_rays = lidar_rays
        self._lidar_range = lidar_range
        self._lidar_rate = lidar_rate

        # Robot state (kinematic)
        self._pos = self._start_pos.copy()
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._stopped = False

        # MuJoCo (for raycasting only, no physics)
        self._mj_model = None
        self._mj_data = None

        self._sim_thread: Optional[threading.Thread] = None
        self._running = False

    def setup(self):
        self.cmd_vel.subscribe(self._on_cmd_vel)
        self.stop_signal.subscribe(self._on_stop)
        self._setup_mujoco_scene()

    def _setup_mujoco_scene(self):
        """Load MuJoCo scene for raycasting only (no robot physics)."""
        try:
            import mujoco

            # Build a simple scene XML with obstacles for raycasting
            scene_xml = self._generate_scene_xml()

            import tempfile
            tmp = tempfile.NamedTemporaryFile(
                suffix=".xml", delete=False, mode="w", encoding="utf-8")
            tmp.write(scene_xml)
            tmp.close()
            try:
                self._mj_model = mujoco.MjModel.from_xml_path(tmp.name)
                self._mj_data = mujoco.MjData(self._mj_model)
                mujoco.mj_forward(self._mj_model, self._mj_data)
                logger.info("KinematicSimModule: MuJoCo scene loaded (%d geoms)",
                            self._mj_model.ngeom)
            finally:
                Path(tmp.name).unlink(missing_ok=True)

        except ImportError:
            logger.warning("KinematicSimModule: MuJoCo not available, no LiDAR")
        except Exception as e:
            logger.error("KinematicSimModule: scene load failed: %s", e)

    def _generate_scene_xml(self) -> str:
        """Generate MuJoCo XML with floor + obstacles (no robot, no world file)."""
        obs_xml = ""
        for o in self._obstacles:
            if isinstance(o, dict):
                from sim.engine.core.world import ObstacleConfig
                o = ObstacleConfig(**o)
            size_str = " ".join(str(s) for s in o.size)
            pos_str = " ".join(str(p) for p in o.position)
            rgba_str = " ".join(str(c) for c in o.rgba)
            obs_xml += (
                f'    <geom name="{o.name}" type="{o.shape}" '
                f'size="{size_str}" pos="{pos_str}" rgba="{rgba_str}"/>\n'
            )

        return f"""<mujoco model="kinematic_sim">
  <worldbody>
    <geom name="floor" type="plane" size="50 50 0.1" rgba="0.3 0.3 0.3 1"/>
{obs_xml}  </worldbody>
</mujoco>"""

    def start(self):
        super().start()
        self._pos = self._start_pos.copy()
        self._yaw = 0.0
        self._running = True
        self._sim_thread = threading.Thread(
            target=self._sim_loop, name="kinematic_sim", daemon=True)
        self._sim_thread.start()
        self.alive.publish(True)

    def stop(self):
        self._running = False
        if self._sim_thread:
            self._sim_thread.join(timeout=2.0)
        self.alive.publish(False)
        super().stop()

    def _on_cmd_vel(self, twist: Twist):
        if self._stopped:
            return
        self._vx = twist.linear.x if hasattr(twist.linear, 'x') else 0.0
        self._vy = twist.linear.y if hasattr(twist.linear, 'y') else 0.0
        self._wz = twist.angular.z if hasattr(twist.angular, 'z') else 0.0

    def _on_stop(self, level: int):
        if level >= 1:
            self._vx = self._vy = self._wz = 0.0
            self._stopped = True

    def _sim_loop(self):
        """Main loop: integrate kinematics + publish odom + lidar."""
        dt = 1.0 / self._sim_rate
        lidar_interval = 1.0 / self._lidar_rate
        last_lidar = 0.0
        step = 0

        while self._running:
            t0 = time.monotonic()
            ts = time.time()

            # Kinematic integration (exact, deterministic)
            self._yaw += self._wz * dt
            # Normalize yaw
            while self._yaw > math.pi:
                self._yaw -= 2 * math.pi
            while self._yaw < -math.pi:
                self._yaw += 2 * math.pi

            # Body-frame velocity → world-frame displacement
            cos_y = math.cos(self._yaw)
            sin_y = math.sin(self._yaw)
            dx = (self._vx * cos_y - self._vy * sin_y) * dt
            dy = (self._vx * sin_y + self._vy * cos_y) * dt
            self._pos[0] += dx
            self._pos[1] += dy

            # Publish odometry
            qz = math.sin(self._yaw / 2)
            qw = math.cos(self._yaw / 2)
            self.odometry.publish(Odometry(
                pose=Pose(
                    position=Vector3(self._pos[0], self._pos[1], self._pos[2]),
                    orientation=Quaternion(0, 0, qz, qw)),
                twist=Twist(
                    linear=Vector3(self._vx, self._vy, 0),
                    angular=Vector3(0, 0, self._wz)),
                ts=ts,
            ))

            # LiDAR scan (at lidar_rate)
            if ts - last_lidar >= lidar_interval:
                pts = self._scan_lidar()
                if pts is not None and len(pts) > 0:
                    self.lidar_cloud.publish(PointCloud(
                        points=pts, frame_id="lidar", ts=ts))
                last_lidar = ts

            step += 1
            elapsed = time.monotonic() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    def _scan_lidar(self) -> Optional[np.ndarray]:
        """Raycast LiDAR using MuJoCo scene."""
        if self._mj_model is None:
            return None

        import mujoco

        pos = self._pos.copy()
        n = self._lidar_rays

        # Generate ray directions (horizontal fan)
        angles = np.linspace(0, 2 * math.pi, n, endpoint=False)
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)

        dirs = np.zeros((n, 3), dtype=np.float64)
        for i, a in enumerate(angles):
            # Local frame ray → world frame
            lx = math.cos(a)
            ly = math.sin(a)
            dirs[i, 0] = lx * cos_y - ly * sin_y
            dirs[i, 1] = lx * sin_y + ly * cos_y
            dirs[i, 2] = 0.0  # horizontal

        dist_out = np.full(n, -1.0, dtype=np.float64)
        geomid_out = np.full(n, -1, dtype=np.int32)
        geomgroup = np.ones(6, dtype=np.uint8)

        mujoco.mj_multiRay(
            self._mj_model, self._mj_data,
            pos, dirs.flatten(),
            geomgroup, 1, -1,
            geomid_out, dist_out, None,
            n, self._lidar_range,
        )

        mask = (dist_out > 0.1) & (dist_out < self._lidar_range)
        if not mask.any():
            return np.zeros((0, 3), dtype=np.float32)

        pts = (pos + dirs[mask] * dist_out[mask, None]).astype(np.float32)

        # Add slight noise
        pts += np.random.normal(0, 0.02, pts.shape).astype(np.float32)

        return pts

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["kinematic_sim"] = {
            "world": self._world,
            "pos": [round(self._pos[0], 2), round(self._pos[1], 2)],
            "yaw_deg": round(math.degrees(self._yaw), 1),
            "has_mujoco": self._mj_model is not None,
            "n_geoms": self._mj_model.ngeom if self._mj_model else 0,
        }
        return info
