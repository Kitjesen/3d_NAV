"""Stub connection -- no hardware, dead-reckoning odometry for CI/testing.

Same port interface as NovaDogConnection so it drops into any blueprint.
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict

from core.stream import In, Out
from core.module import Module
from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry


class StubConnection(Module, layer=1):
    """Fake robot driver for testing. Integrates cmd_vel into odometry."""

    cmd_vel: In[Twist]
    stop_signal: In[int]
    slam_odom: In[Odometry]
    odometry: Out[Odometry]
    alive: Out[bool]

    def __init__(self, dt: float = 0.02, **kw):
        super().__init__(**kw)
        self._dt = dt
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

    def setup(self):
        self.cmd_vel.subscribe(self._on_cmd)
        self.stop_signal.subscribe(self._on_stop)

    def start(self):
        super().start()
        self.alive.publish(True)

    def stop(self):
        self.alive.publish(False)
        super().stop()

    def _on_cmd(self, twist: Twist):
        self._vx = twist.linear.x
        self._vy = twist.linear.y
        self._wz = twist.angular.z
        self._integrate()
        self._publish_odom()

    def _on_stop(self, level: int):
        if level >= 1:
            self._vx = self._vy = self._wz = 0.0

    def _integrate(self):
        self._yaw += self._wz * self._dt
        self._x += (self._vx * math.cos(self._yaw) - self._vy * math.sin(self._yaw)) * self._dt
        self._y += (self._vx * math.sin(self._yaw) + self._vy * math.cos(self._yaw)) * self._dt

    def _publish_odom(self):
        q = Quaternion.from_yaw(self._yaw)
        self.odometry.publish(Odometry(
            pose=Pose(Vector3(self._x, self._y, 0.0), q),
            twist=Twist(Vector3(self._vx, self._vy, 0.0), Vector3(0.0, 0.0, self._wz)),
            ts=time.time(), frame_id="odom", child_frame_id="body",
        ))


def stub_blueprint(**config) -> "Blueprint":
    """Test blueprint -- StubConnection + new module architecture.

    NOTE: Blueprint factory — cross-layer imports are intentional here (see CLAUDE.md).
    """
    from core.blueprint import Blueprint
    from nav.navigation_module import NavigationModule
    from nav.safety_ring_module import SafetyRingModule

    bp = Blueprint()
    bp.add(StubConnection)
    bp.add(NavigationModule, planner=config.get("planner", "astar"))
    bp.add(SafetyRingModule)
    bp.wire("SafetyRingModule", "stop_cmd", "StubConnection", "stop_signal")
    bp.auto_wire()
    return bp
