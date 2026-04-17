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
import threading
import time
from typing import Any, Dict

from core.blueprint import Blueprint
from core.module import Module
from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry
from core.msgs.semantic import SceneGraph
from core.msgs.sensor import Image
from core.registry import register
from core.stream import In, Out


@register("driver", "stub", priority=0, description="Fake driver for CI/testing, no hardware")
class StubDogModule(Module, layer=1):
    """Stub driver for CI/testing.  No hardware.  Publishes fake odometry.

    Also declares (but does not publish) the camera/depth/scene_graph Out
    ports that real drivers expose. Lets `full_stack_blueprint()` autowire
    downstream consumers (Perception, Reconstruction, Teleop, VisualServo,
    DepthVisualOdom) without errors. Consumers simply never see any frame
    when running against the stub — correct behaviour for CI.
    """

    # -- Port declarations (same interface as HanDogModule) --
    cmd_vel: In[Twist]
    stop_signal: In[int]
    odometry: Out[Odometry]
    alive: Out[bool]
    # Camera / perception placeholders — declared for wire compatibility,
    # never published by the stub driver.
    camera_image: Out[Image]
    depth_image: Out[Image]
    scene_graph: Out[SceneGraph]

    def __init__(self, initial_x: float = 0.0, initial_y: float = 0.0,
                 initial_yaw: float = 0.0, **kw: Any) -> None:
        super().__init__(**kw)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._pos_x = initial_x
        self._pos_y = initial_y
        self._yaw = initial_yaw
        self._last_ts: float = 0.0
        self._stopped = False
        self._odom_hz = 50.0
        self._odom_thread: threading.Thread | None = None
        self._running = False

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd)
        self.stop_signal.subscribe(self._on_stop)

    def start(self) -> None:
        super().start()
        self._last_ts = time.time()
        self._running = True
        self.alive.publish(True)
        self._publish_initial_odom()
        # Periodic odom loop (like a real robot)
        self._odom_thread = threading.Thread(target=self._odom_loop, daemon=True)
        self._odom_thread.start()

    def stop(self) -> None:
        self._running = False
        if self._odom_thread:
            self._odom_thread.join(timeout=1.0)
        self.alive.publish(False)
        super().stop()

    def _publish_initial_odom(self) -> None:
        now = time.time()
        self.odometry.publish(Odometry(
            pose=Pose(
                position=Vector3(self._pos_x, self._pos_y, 0.0),
                orientation=Quaternion(
                    0.0, 0.0,
                    math.sin(self._yaw / 2),
                    math.cos(self._yaw / 2),
                ),
            ),
            twist=Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0)),
            ts=now, frame_id="odom", child_frame_id="body",
        ))

    # -- Port callbacks ------------------------------------------------

    def _on_cmd(self, twist: Twist) -> None:
        """Accept velocity command — integration happens in _odom_loop."""
        self._vx = twist.linear.x
        self._vy = twist.linear.y
        self._wz = twist.angular.z
        self._stopped = False

    def _odom_loop(self) -> None:
        """Periodic dead-reckoning integration + odom publish (like a real robot)."""
        dt = 1.0 / self._odom_hz
        while self._running:
            now = time.time()
            cos_y = math.cos(self._yaw)
            sin_y = math.sin(self._yaw)
            self._pos_x += (self._vx * cos_y - self._vy * sin_y) * dt
            self._pos_y += (self._vx * sin_y + self._vy * cos_y) * dt
            self._yaw += self._wz * dt

            self.odometry.publish(Odometry(
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
            ))
            time.sleep(dt)

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0
            self._stopped = True

    # -- Health --------------------------------------------------------

    def health(self) -> dict[str, Any]:
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
