"""MuJoCo simulation connection -- TCP bridge to sim server.

Same port interface as NovaDogConnection. Falls back to dead-reckoning
if sim server is unreachable.

Expected sim server protocol (JSON lines over TCP)::

    -> {"cmd": "walk", "vx": 0.3, "vy": 0.0, "wz": 0.1}
    <- {"x": 1.2, "y": 0.5, "yaw": 0.3, "vx": 0.3, "vy": 0.0, "wz": 0.1}
"""

from __future__ import annotations

import json
import logging
import math
import socket
import threading
import time
from typing import Any, Dict, Optional

from core.stream import In, Out
from core.module import Module
from core.msgs.geometry import Pose, Quaternion, Twist, Vector3
from core.msgs.nav import Odometry

logger = logging.getLogger(__name__)


class MujocoConnection(Module, layer=1):
    """MuJoCo sim driver. Bridges cmd_vel to sim, reads back odometry."""

    cmd_vel: In[Twist]
    stop_signal: In[int]
    slam_odom: In[Odometry]
    odometry: Out[Odometry]
    alive: Out[bool]

    def __init__(
        self,
        sim_host: str = "localhost",
        sim_port: int = 8765,
        odom_rate: float = 50.0,
        **kw,
    ):
        super().__init__(**kw)
        self._host = sim_host
        self._port = sim_port
        self._odom_rate = odom_rate
        self._sock: Optional[socket.socket] = None
        self._connected = False
        self._shutdown = False
        # Dead-reckoning fallback state
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._odom_thread: Optional[threading.Thread] = None

    def setup(self):
        self.cmd_vel.subscribe(self._on_cmd)
        self.stop_signal.subscribe(self._on_stop)

    def start(self):
        super().start()
        self._shutdown = False
        self._try_connect()
        self._odom_thread = threading.Thread(target=self._odom_loop, daemon=True)
        self._odom_thread.start()
        self.alive.publish(True)

    def stop(self):
        self._shutdown = True
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
        self.alive.publish(False)
        super().stop()

    def _try_connect(self):
        try:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self._sock.settimeout(2.0)
            self._sock.connect((self._host, self._port))
            self._connected = True
            logger.info("Connected to sim at %s:%d", self._host, self._port)
        except Exception as e:
            logger.warning("Sim unreachable (%s:%d): %s — using dead reckoning", self._host, self._port, e)
            self._connected = False

    def _on_cmd(self, twist: Twist):
        self._vx = twist.linear.x
        self._vy = twist.linear.y
        self._wz = twist.angular.z
        if self._connected and self._sock:
            try:
                msg = json.dumps({"cmd": "walk", "vx": self._vx, "vy": self._vy, "wz": self._wz}) + "\n"
                self._sock.sendall(msg.encode())
            except Exception:
                self._connected = False

    def _on_stop(self, level: int):
        if level >= 1:
            self._vx = self._vy = self._wz = 0.0
            self._on_cmd(Twist())

    def _odom_loop(self):
        dt = 1.0 / self._odom_rate
        while not self._shutdown:
            if self._connected and self._sock:
                try:
                    self._sock.settimeout(dt)
                    data = self._sock.recv(4096).decode().strip()
                    if data:
                        state = json.loads(data.split("\n")[-1])
                        self._x = state.get("x", self._x)
                        self._y = state.get("y", self._y)
                        self._yaw = state.get("yaw", self._yaw)
                except Exception:
                    self._dead_reckon(dt)
            else:
                self._dead_reckon(dt)

            self._publish_odom()
            time.sleep(dt)

    def _dead_reckon(self, dt: float):
        self._yaw += self._wz * dt
        self._x += (self._vx * math.cos(self._yaw) - self._vy * math.sin(self._yaw)) * dt
        self._y += (self._vx * math.sin(self._yaw) + self._vy * math.cos(self._yaw)) * dt

    def _publish_odom(self):
        q = Quaternion.from_yaw(self._yaw)
        self.odometry.publish(Odometry(
            pose=Pose(Vector3(self._x, self._y, 0.0), q),
            twist=Twist(Vector3(self._vx, self._vy, 0.0), Vector3(0.0, 0.0, self._wz)),
            ts=time.time(), frame_id="odom", child_frame_id="body",
        ))


def simulation_blueprint(**config) -> "Blueprint":
    """Sim blueprint -- MujocoConnection + new module architecture."""
    from core.blueprint import Blueprint
    from nav.navigation_module import NavigationModule
    from nav.safety_ring_module import SafetyRingModule

    bp = Blueprint()
    bp.add(MujocoConnection,
           sim_host=config.get("sim_host", "localhost"),
           sim_port=config.get("sim_port", 8765))
    bp.add(NavigationModule, planner=config.get("planner", "astar"))
    bp.add(SafetyRingModule)
    bp.wire("SafetyRingModule", "stop_cmd", "MujocoConnection", "stop_signal")
    bp.auto_wire()
    return bp
