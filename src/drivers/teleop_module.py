"""TeleopModule — WebSocket joystick remote control + live camera stream.

Phone/browser connects via WebSocket:
  ws://<robot>:5060/teleop

Receives JSON joystick commands:
  {"type": "joy", "lx": 0.5, "ly": 0.0, "az": -0.3}

Sends camera frames as JPEG:
  binary WebSocket messages (JPEG bytes)

When teleop is active, publishes cmd_vel with higher priority than autonomy.
Publishes stop_signal to pause NavigationModule during manual control.
Releases autonomy when joystick goes idle for > release_timeout seconds.

Ports:
  In:  color_image (Image) — camera stream to forward to client
  Out: cmd_vel (Twist)     — joystick → velocity command
       nav_stop (int)      — 1=pause autonomy, 0=release
"""

from __future__ import annotations

import asyncio
import json
import logging
import threading
import time
from typing import Any, Dict, Optional, Set

import numpy as np

from core.module import Module, skill
from core.stream import In, Out
from core.msgs.geometry import Twist, Vector3
from core.msgs.sensor import Image
from core.registry import register

logger = logging.getLogger(__name__)


@register("teleop", "default", description="WebSocket joystick teleop + camera stream")
class TeleopModule(Module, layer=6):
    """WebSocket-based remote control with live camera feedback.

    Joystick input → cmd_vel (direct to driver).
    Camera frames → JPEG → WebSocket → phone/browser.
    Auto-releases autonomy when joystick idle.
    """

    color_image: In[Image]

    cmd_vel:  Out[Twist]
    nav_stop: Out[int]

    def __init__(
        self,
        port: int = 5060,
        max_speed: float = 0.5,
        max_yaw_rate: float = 1.0,
        release_timeout: float = 3.0,
        jpeg_quality: int = 60,
        stream_fps: float = 10.0,
        **kw,
    ):
        super().__init__(**kw)
        self._port = port
        self._max_speed = max_speed
        self._max_yaw_rate = max_yaw_rate
        self._release_timeout = release_timeout
        self._jpeg_quality = jpeg_quality
        self._stream_interval = 1.0 / stream_fps

        self._active = False
        self._last_joy_time = 0.0
        self._clients: Set = set()
        self._latest_jpeg: Optional[bytes] = None
        self._server_thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._running = False

    def setup(self) -> None:
        self.color_image.subscribe(self._on_image)
        self.color_image.set_policy("latest")

    def start(self) -> None:
        super().start()
        self._running = True
        self._server_thread = threading.Thread(
            target=self._run_server, name="teleop_ws", daemon=True)
        self._server_thread.start()
        logger.info("TeleopModule: WebSocket server starting on port %d", self._port)

    def stop(self) -> None:
        self._running = False
        if self._loop and self._loop.is_running():
            try:
                self._loop.call_soon_threadsafe(self._loop.stop)
            except RuntimeError:
                pass  # loop already closed
        if self._server_thread:
            self._server_thread.join(timeout=3.0)
        self._release_autonomy()
        super().stop()

    # ── Camera stream ─────────────────────────────────────────────────────────

    def _on_image(self, img: Image) -> None:
        if not self._clients:
            return
        try:
            import cv2
            _, buf = cv2.imencode(".jpg", img.data,
                                  [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality])
            self._latest_jpeg = buf.tobytes()
        except ImportError:
            pass  # no cv2 = no stream

    # ── Joystick handling ─────────────────────────────────────────────────────

    def _on_joy(self, data: dict) -> None:
        lx = float(data.get("lx", 0.0))
        ly = float(data.get("ly", 0.0))
        az = float(data.get("az", 0.0))

        lx = max(-1.0, min(1.0, lx)) * self._max_speed
        ly = max(-1.0, min(1.0, ly)) * self._max_speed
        az = max(-1.0, min(1.0, az)) * self._max_yaw_rate

        self.cmd_vel.publish(Twist(
            linear=Vector3(x=lx, y=ly, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=az),
        ))

        now = time.time()
        self._last_joy_time = now

        if not self._active:
            self._active = True
            self.nav_stop.publish(1)
            logger.info("TeleopModule: manual control engaged")

    def _check_idle(self) -> None:
        if not self._active:
            return
        if time.time() - self._last_joy_time > self._release_timeout:
            self._release_autonomy()

    def _release_autonomy(self) -> None:
        if self._active:
            self._active = False
            self.cmd_vel.publish(Twist())  # zero velocity
            self.nav_stop.publish(0)
            logger.info("TeleopModule: manual control released, autonomy resumed")

    # ── WebSocket server ──────────────────────────────────────────────────────

    def _run_server(self) -> None:
        try:
            import websockets
        except ImportError:
            logger.warning("TeleopModule: websockets not installed, teleop disabled")
            return

        async def _handler(ws, path=None):
            self._clients.add(ws)
            logger.info("TeleopModule: client connected (%d total)", len(self._clients))
            try:
                # Start camera stream task
                stream_task = asyncio.ensure_future(self._stream_camera(ws))
                async for msg in ws:
                    try:
                        data = json.loads(msg)
                        if data.get("type") == "joy":
                            self._on_joy(data)
                        elif data.get("type") == "stop":
                            self._release_autonomy()
                    except json.JSONDecodeError:
                        pass
                stream_task.cancel()
            finally:
                self._clients.discard(ws)
                self._check_idle()
                logger.info("TeleopModule: client disconnected (%d total)", len(self._clients))

        async def _serve():
            self._loop = asyncio.get_event_loop()
            # Retry binding up to 5 times with 1-second back-off.
            # The port may still be in TIME_WAIT after a previous run was
            # killed (fuser -k) less than ~1 s ago.
            max_attempts = 5
            for attempt in range(1, max_attempts + 1):
                try:
                    async with websockets.serve(_handler, "0.0.0.0", self._port):
                        logger.info(
                            "TeleopModule: WebSocket server ready on port %d", self._port
                        )
                        while self._running:
                            self._check_idle()
                            await asyncio.sleep(0.5)
                    return
                except OSError as exc:
                    if attempt < max_attempts:
                        logger.warning(
                            "TeleopModule: port %d busy (attempt %d/%d), retrying in 1 s — %s",
                            self._port, attempt, max_attempts, exc,
                        )
                        await asyncio.sleep(1.0)
                    else:
                        logger.error(
                            "TeleopModule: cannot bind port %d after %d attempts. "
                            "Teleop disabled. Kill the process holding the port: "
                            "fuser -k %d/tcp",
                            self._port, max_attempts, self._port,
                        )

        asyncio.run(_serve())

    async def _stream_camera(self, ws) -> None:
        import websockets
        while self._running:
            if self._latest_jpeg:
                try:
                    await ws.send(self._latest_jpeg)
                except websockets.exceptions.ConnectionClosed:
                    break
            await asyncio.sleep(self._stream_interval)

    # ── @skill methods ────────────────────────────────────────────────────────

    @skill
    def get_teleop_status(self) -> dict:
        return {
            "active": self._active,
            "clients": len(self._clients),
            "port": self._port,
        }

    @skill
    def force_release(self) -> str:
        self._release_autonomy()
        return "Teleop released, autonomy resumed"
