# DEPRECATED: Use StubDogModule from core/blueprints/stub.py instead.
# StubDogModule is the canonical stub driver registered as ("driver", "stub")
# in the registry, with periodic odom publishing and health reporting.
# This file is kept for backward compatibility.
# TODO: Remove once all consumers are migrated.
"""Stub connection -- no hardware, dead-reckoning odometry for CI/testing.

DEPRECATED: Superseded by StubDogModule in core/blueprints/stub.py, which is
registered in the plugin registry and used by all current profiles/blueprints.

Same port interface as ThunderDriver so it drops into any blueprint.
"""

from __future__ import annotations

import math
import time
from typing import TYPE_CHECKING, Any, Dict

from core.module import Module

if TYPE_CHECKING:
    from core.blueprint import Blueprint
from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.stream import In, Out


class StubConnection(Module, layer=1):
    """Fake robot driver for testing. Integrates cmd_vel into odometry."""

    cmd_vel: In[Twist]
    stop_signal: In[int]
    slam_odom: In[Odometry]
    odometry: Out[Odometry]
    alive: Out[bool]

    def __init__(self, dt: float = 0.02, initial_x: float = 0.0,
                 initial_y: float = 0.0, initial_yaw: float = 0.0, **kw):
        super().__init__(**kw)
        self._dt = dt
        self._x = initial_x
        self._y = initial_y
        self._yaw = initial_yaw
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0

    def setup(self):
        self.cmd_vel.subscribe(self._on_cmd)
        self.stop_signal.subscribe(self._on_stop)

    def start(self):
        super().start()
        self.alive.publish(True)
        self._publish_odom()  # bootstrap: let consumers know initial position

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


def stub_blueprint(**config) -> Blueprint:
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
