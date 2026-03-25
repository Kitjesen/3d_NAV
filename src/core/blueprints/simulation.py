"""Simulation blueprint -- MuJoCo (or any sim) driver + full Python stack.

SimDogModule bridges cmd_vel to a simulation backend via a simple TCP
socket protocol.  It exposes the same ports as HanDogModule so the rest
of the navigation stack is unaware it is running in simulation.

The expected sim server (e.g. ``nova_nav_bridge.py``) listens on
``sim_host:sim_port`` and exchanges JSON-line messages:

    -> {"cmd": "walk", "vx": 0.3, "vy": 0.0, "wz": 0.1}
    <- {"x": 1.2, "y": 0.5, "yaw": 0.3, "vx": 0.3, "vy": 0.0, "wz": 0.1}

If the sim server is unreachable the module falls back to dead-reckoning
(same behaviour as StubDogModule) so blueprints can still be built and
tested without a running simulator.

Usage::

    from core.blueprints.simulation import simulation_blueprint

    system = simulation_blueprint(sim_host="localhost", sim_port=8765).build()
    system.start()
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
from core.blueprint import Blueprint
from core.registry import register

logger = logging.getLogger(__name__)


@register("driver", "sim_mujoco", priority=5, platforms={"x86_64", "aarch64"}, description="MuJoCo simulation driver")
class SimDogModule(Module, layer=1):
    """MuJoCo simulation driver.  Same ports as HanDogModule."""

    # -- Port declarations (mirror HanDogModule) --
    cmd_vel: In[Twist]
    stop_signal: In[int]
    odometry: Out[Odometry]
    alive: Out[bool]

    def __init__(
        self,
        sim_host: str = "localhost",
        sim_port: int = 8765,
        odom_rate: float = 10.0,
        connect_timeout: float = 2.0,
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._sim_host = sim_host
        self._sim_port = sim_port
        self._odom_interval = 1.0 / max(odom_rate, 1.0)
        self._connect_timeout = connect_timeout

        # Internal state (dead-reckoning fallback)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._yaw = 0.0
        self._last_ts: float = 0.0
        self._stopped = False

        # Socket connection
        self._sock: Optional[socket.socket] = None
        self._connected = False
        self._shutdown = False
        self._odom_thread: Optional[threading.Thread] = None

    # -- Lifecycle -----------------------------------------------------

    def setup(self) -> None:
        self.cmd_vel.subscribe(self._on_cmd)
        self.stop_signal.subscribe(self._on_stop)

    def start(self) -> None:
        super().start()
        self._shutdown = False
        self._last_ts = time.time()
        self._try_connect()
        self.alive.publish(self._connected)

        # Odometry publishing thread
        self._odom_thread = threading.Thread(
            target=self._odom_loop, daemon=True)
        self._odom_thread.start()

    def stop(self) -> None:
        self._shutdown = True
        if self._odom_thread:
            self._odom_thread.join(timeout=2.0)
        self._disconnect()
        self.alive.publish(False)
        super().stop()

    # -- Port callbacks ------------------------------------------------

    def _on_cmd(self, twist: Twist) -> None:
        """Send cmd_vel to sim, fall back to dead reckoning."""
        self._vx = twist.linear.x
        self._vy = twist.linear.y
        self._wz = twist.angular.z
        self._stopped = False

        if self._connected:
            state = self._send_cmd(self._vx, self._vy, self._wz)
            if state:
                self._pos_x = state.get("x", self._pos_x)
                self._pos_y = state.get("y", self._pos_y)
                self._yaw = state.get("yaw", self._yaw)
                return

        # Dead-reckoning fallback
        self._dead_reckon()

    def _on_stop(self, level: int) -> None:
        if level >= 1:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0
            self._stopped = True
            if self._connected:
                self._send_cmd(0.0, 0.0, 0.0)

    # -- Odometry loop -------------------------------------------------

    def _odom_loop(self) -> None:
        """Periodically publish odometry at configured rate."""
        while not self._shutdown:
            self._publish_odometry()
            time.sleep(self._odom_interval)

    def _publish_odometry(self) -> None:
        now = time.time()
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

    # -- Dead reckoning ------------------------------------------------

    def _dead_reckon(self) -> None:
        now = time.time()
        dt = now - self._last_ts if self._last_ts > 0 else 0.0
        dt = min(dt, 1.0)
        self._last_ts = now

        if dt > 0:
            cos_y = math.cos(self._yaw)
            sin_y = math.sin(self._yaw)
            self._pos_x += (self._vx * cos_y - self._vy * sin_y) * dt
            self._pos_y += (self._vx * sin_y + self._vy * cos_y) * dt
            self._yaw += self._wz * dt

    # -- Sim socket communication --------------------------------------

    def _try_connect(self) -> None:
        """Attempt to connect to the sim server.  Non-fatal on failure."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self._connect_timeout)
            sock.connect((self._sim_host, self._sim_port))
            self._sock = sock
            self._connected = True
            logger.info("SimDogModule connected to %s:%d",
                        self._sim_host, self._sim_port)
        except (OSError, ConnectionRefusedError, socket.timeout) as exc:
            logger.warning(
                "SimDogModule could not connect to %s:%d (%s) "
                "-- falling back to dead reckoning",
                self._sim_host, self._sim_port, exc)
            self._connected = False

    def _disconnect(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            self._connected = False

    def _send_cmd(self, vx: float, vy: float, wz: float) -> Optional[Dict]:
        """Send walk command, return parsed state dict or None on error."""
        if not self._sock:
            return None
        msg = json.dumps({"cmd": "walk", "vx": vx, "vy": vy, "wz": wz}) + "\n"
        try:
            self._sock.sendall(msg.encode("utf-8"))
            self._sock.settimeout(0.5)
            data = self._sock.recv(4096)
            if data:
                return json.loads(data.decode("utf-8"))
        except (OSError, json.JSONDecodeError, socket.timeout) as exc:
            logger.warning("Sim communication error: %s", exc)
            self._connected = False
        return None

    # -- Health --------------------------------------------------------

    def health(self) -> Dict[str, Any]:
        stats = super().port_summary()
        stats["sim"] = {
            "connected": self._connected,
            "host": f"{self._sim_host}:{self._sim_port}",
            "stopped": self._stopped,
        }
        return stats


# -- Blueprint factory -------------------------------------------------

def simulation_blueprint(**config: Any) -> Blueprint:
    """Simulation blueprint -- SimDogModule + new module architecture."""
    from nav.navigation_module import NavigationModule
    from nav.safety_ring_module import SafetyRingModule

    bp = Blueprint()
    bp.add(SimDogModule,
           sim_host=config.get("sim_host", "localhost"),
           sim_port=config.get("sim_port", 8765))
    bp.add(NavigationModule, planner=config.get("planner", "astar"))
    bp.add(SafetyRingModule)

    bp.wire("SafetyRingModule", "stop_cmd", "SimDogModule", "stop_signal")
    bp.auto_wire()
    return bp
