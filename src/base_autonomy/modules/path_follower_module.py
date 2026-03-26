"""PathFollowerModule — path tracking as a pluggable Module.

Takes local path + odometry, produces cmd_vel to follow the path.

Backends:
  "pure_pursuit" — C++ path_follower (NativeModule, Pure Pursuit)
  "pid"          — Python PID controller (testing/simple)

Usage::

    bp.add(PathFollowerModule, backend="pure_pursuit")
"""

from __future__ import annotations

import logging
import math
from typing import Any, Dict, Optional

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import Twist, Vector3
from core.registry import register

logger = logging.getLogger(__name__)


@register("path_follower", "pure_pursuit",
          description="C++ Pure Pursuit path follower (NativeModule)")
class PathFollowerModule(Module, layer=2):
    """Path following — tracks local path and outputs cmd_vel.

    "pure_pursuit" backend: C++ path_follower NativeModule
    "pid" backend: Python PID controller (testing)
    """

    # -- Inputs --
    odometry: In[Odometry]
    local_path: In[Path]

    # -- Outputs --
    cmd_vel: Out[Twist]
    alive: Out[bool]

    def __init__(self, backend: str = "pure_pursuit",
                 max_speed: float = 0.4, lookahead: float = 1.5, **kw):
        super().__init__(**kw)
        self._backend = backend
        self._max_speed = max_speed
        self._lookahead = lookahead
        self._node = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._path_points = None

    def setup(self):
        self.odometry.subscribe(self._on_odom)
        self.local_path.subscribe(self._on_path)

        if self._backend == "pure_pursuit":
            self._setup_native()
        else:
            logger.info("PathFollowerModule: pid backend")

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

    def _on_odom(self, odom: Odometry):
        prev_x, prev_y = self._robot_x, self._robot_y
        self._robot_x = odom.pose.position.x
        self._robot_y = odom.pose.position.y

        # Estimate yaw from velocity or position delta (more reliable than quaternion
        # for robots where body frame doesn't align with motion direction)
        vx = odom.twist.linear.x if hasattr(odom.twist.linear, 'x') else 0.0
        vy = odom.twist.linear.y if hasattr(odom.twist.linear, 'y') else 0.0
        speed = math.hypot(vx, vy)
        if speed > 0.05:
            # Use velocity direction
            self._robot_yaw = math.atan2(vy, vx)
        else:
            # Fallback: position delta
            dx = self._robot_x - prev_x
            dy = self._robot_y - prev_y
            if math.hypot(dx, dy) > 0.01:
                self._robot_yaw = math.atan2(dy, dx)

        # PID backend: compute cmd_vel on each odom update
        if self._backend == "pid" and self._path_points is not None:
            self._pid_step()

    def _on_path(self, path: Path):
        if self._backend == "pid":
            pts = []
            for ps in path.poses:
                pts.append([ps.pose.position.x, ps.pose.position.y])
            self._path_points = np.array(pts) if len(pts) >= 2 else None

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
            self.cmd_vel.publish(Twist())
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_err = desired_yaw - self._robot_yaw
        # Normalize to [-pi, pi]
        while yaw_err > math.pi:
            yaw_err -= 2 * math.pi
        while yaw_err < -math.pi:
            yaw_err += 2 * math.pi

        # P controller with speed/yaw coupling
        wz = max(-0.5, min(0.5, yaw_err * 0.6))  # match policy range
        # Slow down when turning hard (cos coupling)
        turn_factor = max(0.3, math.cos(yaw_err))
        vx = min(self._max_speed, dist * 0.3) * turn_factor

        self.cmd_vel.publish(Twist(
            linear=Vector3(vx, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, wz),
        ))

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["path_follower"] = {
            "backend": self._backend,
            "has_path": self._path_points is not None,
        }
        return info
