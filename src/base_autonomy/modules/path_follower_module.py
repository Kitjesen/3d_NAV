"""PathFollowerModule — path tracking as a pluggable Module.

Takes local path + odometry, produces cmd_vel to follow the path.

Backends:
  "nav_core"     — C++ compute_control via nanobind (Pure Pursuit, adaptive lookahead,
                   two-way drive, omni-dir near-goal, acceleration limiting) [DEFAULT]
  "pure_pursuit" — C++ path_follower (NativeModule, Pure Pursuit)
  "pid"          — Python PID controller (testing/simple fallback)

Usage::

    bp.add(PathFollowerModule, backend="nav_core")
"""

from __future__ import annotations

import logging
import math
import time
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import Twist, Vector3
from core.registry import register

logger = logging.getLogger(__name__)


@register("path_follower", "nav_core",
          description="C++ compute_control via nanobind (Pure Pursuit + adaptive lookahead)")
class PathFollowerModule(Module, layer=2):
    """Path following — tracks local path and outputs cmd_vel.

    "nav_core"     backend: C++ compute_control via nanobind [default]
    "pure_pursuit" backend: C++ path_follower NativeModule
    "pid"          backend: Python PID controller (testing/fallback)
    """

    # -- Inputs --
    odometry: In[Odometry]
    local_path: In[Path]

    # -- Outputs --
    cmd_vel: Out[Twist]
    alive: Out[bool]

    def __init__(self, backend: str = "nav_core",
                 max_speed: float = 0.4, lookahead: float = 1.5, **kw):
        super().__init__(**kw)
        self._backend = backend
        self._max_speed = max_speed
        self._lookahead = lookahead
        self._node = None

        # Current robot pose (odom frame)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        # Path state for pid backend
        self._path_points = None

        # Output smoothing (shared across backends)
        self._smooth_vx = 0.0
        self._smooth_wz = 0.0

        # nav_core backend state
        self._nc = None           # _nav_core module reference
        self._nc_params = None    # PathFollowerParams
        self._nc_state = None     # PathFollowerState (persistent across calls)
        self._nc_path: list = []  # list of _nav_core.Vec3 (world frame)

        # Recorded pose at path receipt time (for vehicleRel transform)
        self._x_rec = 0.0
        self._y_rec = 0.0
        self._yaw_rec = 0.0
        self._cos_yaw_rec = 1.0
        self._sin_yaw_rec = 0.0

    def setup(self):
        self.odometry.subscribe(self._on_odom)
        self.local_path.subscribe(self._on_path)

        if self._backend == "nav_core":
            self._setup_nav_core()
        elif self._backend == "pure_pursuit":
            self._setup_native()
        else:
            logger.info("PathFollowerModule: pid backend")

    # ── nav_core backend setup ─────────────────────────────────────────────

    def _setup_nav_core(self):
        """Import _nav_core and create PathFollowerParams/State."""
        try:
            import _nav_core
            self._nc = _nav_core

            params = _nav_core.PathFollowerParams()
            params.max_speed = self._max_speed
            params.base_look_ahead_dis = self._lookahead * 0.2   # scale from legacy lookahead
            params.min_look_ahead_dis = 0.2
            params.max_look_ahead_dis = min(self._lookahead, 2.0)
            params.look_ahead_ratio = 0.5
            params.yaw_rate_gain = 7.5
            params.stop_yaw_rate_gain = 7.5
            params.max_yaw_rate = 45.0   # degrees
            params.max_accel = 1.0
            params.switch_time_thre = 1.0
            params.dir_diff_thre = 0.1   # ~5.7 deg
            params.omni_dir_goal_thre = 1.0
            params.omni_dir_diff_thre = 1.5  # ~86 deg
            params.stop_dis_thre = 0.2
            params.slow_dwn_dis_thre = 1.0
            params.two_way_drive = True
            params.no_rot_at_goal = True

            self._nc_params = params
            self._nc_state = _nav_core.PathFollowerState()

            logger.info("PathFollowerModule [nav_core]: C++ compute_control loaded")
        except ImportError as e:
            logger.warning(
                "PathFollowerModule: _nav_core not available (%s), falling back to pid", e
            )
            self._backend = "pid"
            self._nc = None

    # ── NativeModule (pure_pursuit) backend setup ──────────────────────────

    def _setup_native(self):
        try:
            from core.config import get_config
            from core.native_factories import path_follower

            cfg = get_config()
            self._node = path_follower(cfg)
            try:
                self._node.setup()
            except (FileNotFoundError, PermissionError) as e:
                logger.warning("PathFollowerModule: setup failed: %s", e)
                self._node = None
        except ImportError as e:
            logger.warning("PathFollowerModule: C++ backend not available: %s", e)

    # ── Module lifecycle ───────────────────────────────────────────────────

    def start(self):
        super().start()
        if self._node:
            try:
                self._node.start()
                logger.info("PathFollowerModule [pure_pursuit]: C++ node started")
            except Exception as e:
                logger.error("PathFollowerModule: start failed: %s", e)
        self.alive.publish(True)

    def stop(self):
        if self._node:
            try:
                self._node.stop()
            except Exception:
                pass
            self._node = None
        self.alive.publish(False)
        super().stop()

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_odom(self, odom: Odometry):
        prev_x, prev_y = self._robot_x, self._robot_y
        self._robot_x = odom.pose.position.x
        self._robot_y = odom.pose.position.y

        # Extract yaw from quaternion (reliable, works at all speeds)
        self._robot_yaw = odom.yaw

        if self._backend == "nav_core" and self._nc_path:
            self._nav_core_step(odom.ts)
        elif self._backend == "pid" and self._path_points is not None:
            self._pid_step()

    def _on_path(self, path: Path):
        if self._backend == "nav_core":
            # Record robot pose at path receipt — defines the reference frame
            self._x_rec = self._robot_x
            self._y_rec = self._robot_y
            self._yaw_rec = self._robot_yaw
            self._cos_yaw_rec = math.cos(self._yaw_rec)
            self._sin_yaw_rec = math.sin(self._yaw_rec)

            # Convert path to list of Vec3 (world/odom frame, as-is from C++ pathFollower)
            nc = self._nc
            pts = []
            for ps in path.poses:
                v = nc.Vec3(
                    ps.pose.position.x,
                    ps.pose.position.y,
                    ps.pose.position.z,
                )
                pts.append(v)
            self._nc_path = pts
            # Note: do NOT reset nc_state here — compute_control needs
            # continuous state to ramp up speed (vehicleSpeed persists)

        elif self._backend == "pid":
            pts = []
            for ps in path.poses:
                pts.append([ps.pose.position.x, ps.pose.position.y])
            self._path_points = np.array(pts) if len(pts) >= 2 else None

    # ── nav_core control step ──────────────────────────────────────────────

    def _nav_core_step(self, current_time: float):
        """Compute cmd_vel using C++ compute_control."""
        if not self._nc_path:
            self.cmd_vel.publish(Twist())
            return

        nc = self._nc

        # Transform robot position into path reference frame
        # (rotation by -yaw_rec, then translate by -(x_rec, y_rec))
        dx = self._robot_x - self._x_rec
        dy = self._robot_y - self._y_rec
        vehicle_x_rel =  self._cos_yaw_rec * dx + self._sin_yaw_rec * dy
        vehicle_y_rel = -self._sin_yaw_rec * dx + self._cos_yaw_rec * dy
        vehicle_rel = nc.Vec3(vehicle_x_rel, vehicle_y_rel, 0.0)

        # Yaw change since path was received
        vehicle_yaw_diff = self._robot_yaw - self._yaw_rec

        # joy_speed normalized [0, 1] — use 1.0 (max_speed already in params)
        joy_speed = 1.0
        slow_factor = 1.0
        safety_stop = 0

        out = nc.compute_control(
            vehicle_rel,
            vehicle_yaw_diff,
            self._nc_path,
            joy_speed,
            current_time,
            slow_factor,
            safety_stop,
            self._nc_params,
            self._nc_state,
        )

        # Apply exponential smoothing (alpha=0.3, same as pid backend)
        alpha = 0.3
        self._smooth_vx = (1 - alpha) * self._smooth_vx + alpha * out.cmd.vx
        self._smooth_wz = (1 - alpha) * self._smooth_wz + alpha * out.cmd.wz

        self.cmd_vel.publish(Twist(
            linear=Vector3(self._smooth_vx, out.cmd.vy, 0.0),
            angular=Vector3(0.0, 0.0, self._smooth_wz),
        ))

    # ── Python PID (fallback) ──────────────────────────────────────────────

    def _pid_step(self):
        """Simple Pure Pursuit in Python for testing."""
        if self._path_points is None or len(self._path_points) < 2:
            self.cmd_vel.publish(Twist())
            return

        robot = np.array([self._robot_x, self._robot_y])
        dists = np.linalg.norm(self._path_points - robot, axis=1)
        # Find lookahead point
        beyond = np.where(dists > self._lookahead)[0]
        if len(beyond) == 0:
            target = self._path_points[-1]
        else:
            target = self._path_points[beyond[0]]

        dx = target[0] - self._robot_x
        dy = target[1] - self._robot_y
        dist = math.hypot(dx, dy)

        if dist < 0.2:
            self._smooth_vx *= 0.5  # decay to zero
            self._smooth_wz *= 0.5
            if abs(self._smooth_vx) < 0.01:
                self._smooth_vx = 0.0
                self._smooth_wz = 0.0
            self.cmd_vel.publish(Twist(
                linear=Vector3(self._smooth_vx, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, self._smooth_wz),
            ))
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_err = desired_yaw - self._robot_yaw
        # Normalize to [-pi, pi]
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        # P controller with cos coupling — Go1 has low yaw drift (~2°/8s)
        wz = max(-0.8, min(0.8, yaw_err * 0.5))
        turn_factor = max(0.2, math.cos(yaw_err))
        vx = min(self._max_speed, max(0.15, dist * 0.25)) * turn_factor

        alpha = 0.2
        self._smooth_vx = (1 - alpha) * self._smooth_vx + alpha * vx
        self._smooth_wz = (1 - alpha) * self._smooth_wz + alpha * wz

        self.cmd_vel.publish(Twist(
            linear=Vector3(self._smooth_vx, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, self._smooth_wz),
        ))

    # ── Health ─────────────────────────────────────────────────────────────

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        h = {
            "backend": self._backend,
            "has_path": bool(self._nc_path) if self._backend == "nav_core"
                        else self._path_points is not None,
        }
        if self._backend == "nav_core" and self._nc_state is not None:
            h["vehicle_speed"] = self._nc_state.vehicle_speed
            h["nav_fwd"] = self._nc_state.nav_fwd
        info["path_follower"] = h
        return info
