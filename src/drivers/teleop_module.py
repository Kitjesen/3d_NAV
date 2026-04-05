"""TeleopModule — joystick remote control with live camera stream.

Teleop WebSocket is served by GatewayModule at ws://<robot>:5050/ws/teleop
(same port as the REST/SSE gateway — one process, one port).

This module handles:
  - Encoding camera frames as JPEG and pushing them to GatewayModule
  - Being the Module port bridge: cmd_vel Out + nav_stop Out
  - Providing @skill methods for REPL / MCP

GatewayModule.configure_teleop() injects speed/timeout config into the WS
handler, and calls TeleopModule.cmd_vel and nav_stop via on_system_modules().

Protocol (ws://<robot>:5050/ws/teleop)
  Client → server  JSON text:
    {"type": "joy",  "lx": 0.5, "ly": 0.0, "az": -0.3}
    {"type": "stop"}
  Server → client  binary:
    raw JPEG bytes  (camera frame, ~10 fps)
    OR JSON text:
    {"type": "pong", "ts": 1234567890.0}

Ports
  In:  color_image (Image) — camera frames to encode + forward
  Out: cmd_vel  (Twist)    — joystick → driver
       nav_stop (int)      — 1=pause autonomy, 0=resume
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Optional

from core.module import Module, skill
from core.stream import In, Out
from core.msgs.geometry import Twist, Vector3
from core.msgs.sensor import Image
from core.registry import register

logger = logging.getLogger(__name__)


@register("teleop", "default", description="Joystick teleop + camera stream via Gateway WS")
class TeleopModule(Module, layer=6):
    """Camera encoder + joystick bridge — WS served by GatewayModule.

    Subscribes to camera frames, encodes them as JPEG, and pushes the bytes
    to GatewayModule.push_jpeg() so the WS handler can forward them to
    connected clients.

    cmd_vel and nav_stop are published by GatewayModule's WS handler, which
    obtains references to these ports through on_system_modules().
    """

    color_image: In[Image]

    cmd_vel:  Out[Twist]
    nav_stop: Out[int]

    def __init__(
        self,
        max_speed: float = 0.5,
        max_yaw_rate: float = 1.0,
        release_timeout: float = 3.0,
        jpeg_quality: int = 60,
        stream_fps: float = 10.0,
        # Legacy: port kept for backwards compat / REPL status display
        port: int = 5050,
        **kw,
    ):
        super().__init__(**kw)
        self._max_speed = max_speed
        self._max_yaw_rate = max_yaw_rate
        self._release_timeout = release_timeout
        self._jpeg_quality = jpeg_quality
        self._stream_interval = 1.0 / max(1.0, stream_fps)
        self._port = port  # informational only (same as GatewayModule port)

        self._gateway = None  # set by on_system_modules()
        self._encode_thread: Optional[threading.Thread] = None
        self._running = False
        self._latest_raw: Optional[Any] = None   # latest Image.data
        self._raw_lock = threading.Lock()
        self._new_frame = threading.Event()

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        self.color_image.subscribe(self._on_image)
        self.color_image.set_policy("latest")

    def start(self) -> None:
        super().start()
        self._running = True
        self._encode_thread = threading.Thread(
            target=self._encode_loop, name="teleop-encode", daemon=True
        )
        self._encode_thread.start()
        logger.info("TeleopModule started (JPEG encoder active)")

    def stop(self) -> None:
        self._running = False
        self._new_frame.set()  # unblock encode loop
        if self._encode_thread:
            self._encode_thread.join(timeout=3.0)
        self._gateway = None
        super().stop()

    def on_system_modules(self, modules: Dict[str, Any]) -> None:
        """Inject GatewayModule reference + share port config."""
        gw = modules.get("GatewayModule")
        if gw is not None:
            self._gateway = gw
            # Give GatewayModule the port config and our cmd_vel/nav_stop ports
            gw.configure_teleop(
                max_speed=self._max_speed,
                max_yaw=self._max_yaw_rate,
                release_timeout=self._release_timeout,
            )
            # Inject our Out ports so GatewayModule can publish on our behalf
            gw._teleop_cmd_vel_port  = self.cmd_vel
            gw._teleop_nav_stop_port = self.nav_stop
            logger.info("TeleopModule: wired into GatewayModule")
        else:
            logger.warning(
                "TeleopModule: GatewayModule not found — "
                "teleop WebSocket will not be available"
            )

    # -- camera frame handling ----------------------------------------------

    def _on_image(self, img: Image) -> None:
        if self._gateway is None:
            return  # no clients to serve, skip encoding
        with self._raw_lock:
            self._latest_raw = img.data
        self._new_frame.set()

    def _encode_loop(self) -> None:
        """Dedicated thread: encode raw frames to JPEG at stream_fps."""
        try:
            import cv2
            have_cv2 = True
        except ImportError:
            have_cv2 = False
            logger.warning("TeleopModule: cv2 not available — camera stream disabled")

        while self._running:
            triggered = self._new_frame.wait(timeout=self._stream_interval)
            self._new_frame.clear()
            if not triggered or not self._running:
                continue

            gw = self._gateway
            if gw is None:
                continue

            with self._raw_lock:
                raw = self._latest_raw

            if raw is None:
                continue

            if have_cv2:
                try:
                    ok, buf = cv2.imencode(
                        ".jpg", raw,
                        [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
                    )
                    if ok:
                        gw.push_jpeg(buf.tobytes())
                except Exception:
                    pass

    # -- @skill methods (REPL / MCP) ----------------------------------------

    @skill
    def get_teleop_status(self) -> dict:
        """Return current teleop status."""
        gw = self._gateway
        if gw is None:
            return {"active": False, "clients": 0, "port": self._port}
        return {
            "active":  gw._teleop_active,
            "clients": gw._teleop_clients,
            "port":    gw._port,
        }

    @skill
    def force_release(self) -> str:
        """Force-release teleop control and resume autonomy."""
        gw = self._gateway
        if gw:
            gw._teleop_release()
        return "Teleop released, autonomy resumed"
