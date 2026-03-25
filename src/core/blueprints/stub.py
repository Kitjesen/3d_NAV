"""Stub blueprint -- no hardware, fake odometry for CI / unit testing.

StubDogModule is a drop-in replacement for HanDogModule that publishes
synthetic odometry by integrating cmd_vel (simple dead reckoning).  No
gRPC, no hardware, no threads -- completely self-contained.

Usage::

    from core.blueprints.stub import stub_blueprint

    system = stub_blueprint().build()
    system.start()
"""

from __future__ import annotations

import math
import time
from typing import Any, Dict

from core.stream import In, Out
from core.module import Module
from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.registry import register
from core.blueprint import Blueprint


@register("driver", "stub", priority=0, description="Fake driver for CI/testing, no hardware")
class StubDogModule(Module, layer=1):
    """Stub driver for CI/testing.  No hardware.  Publishes fake odometry."""

    # -- Port declarations (same interface as HanDogModule) --
    cmd_vel: In[Twist]
    stop_signal: In[int]
    odometry: Out[Odometry]
    alive: Out[bool]

    def __init__(self, **kw: Any) -> None:
        super().__init__(**kw)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._yaw = 0.0
        self._last_ts: float = 0.0
        self._stopped = False

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd)
        self.stop_signal.subscribe(self._on_stop)

    def start(self) -> None:
        super().start()
        self._last_ts = time.time()
        self.alive.publish(True)

    def stop(self) -> None:
        self.alive.publish(False)
        super().stop()

    # -- Port callbacks ------------------------------------------------

    def _on_cmd(self, twist: Twist) -> None:
        """Integrate position from cmd_vel (simple dead reckoning)."""
        self._vx = twist.linear.x
        self._vy = twist.linear.y
        self._wz = twist.angular.z
        self._stopped = False

        now = time.time()
        dt = now - self._last_ts if self._last_ts > 0 else 0.0
        dt = min(dt, 1.0)  # clamp to avoid jumps
        self._last_ts = now

        if dt > 0:
            cos_y = math.cos(self._yaw)
            sin_y = math.sin(self._yaw)
            self._pos_x += (self._vx * cos_y - self._vy * sin_y) * dt
            self._pos_y += (self._vx * sin_y + self._vy * cos_y) * dt
            self._yaw += self._wz * dt

        odom = Odometry(
            pose=Pose(
                position=Vector3(self._pos_x, self._pos_y, 0.0),
                orientation=Quaternion(
                    0.0, 0.0,
                    math.sin(self._yaw / 2),
                    math.cos(self._yaw / 2),
                ),
            ),
            twist=Twist(
                linear=Vector3(self._vx, self._vy, 0.0),
                angular=Vector3(0.0, 0.0, self._wz),
            ),
            ts=now,
            frame_id="odom",
            child_frame_id="body",
        )
        self.odometry.publish(odom)

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0
            self._stopped = True

    # -- Health --------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        stats = super().port_summary()
        stats["stub"] = {"stopped": self._stopped}
        return stats


# -- Blueprint factory -------------------------------------------------

def stub_blueprint(**config: Any) -> Blueprint:
    """Test blueprint -- StubDogModule + new module architecture."""
    from nav.navigation_module import NavigationModule
    from nav.safety_ring_module import SafetyRingModule

    bp = Blueprint()
    bp.add(StubDogModule)
    bp.add(NavigationModule, planner=config.get("planner", "astar"))
    bp.add(SafetyRingModule)

    bp.wire("SafetyRingModule", "stop_cmd", "StubDogModule", "stop_signal")
    bp.auto_wire()
    return bp
