"""TeleopModule — joystick remote control with live camera stream.

All teleop state and logic lives here.  GatewayModule only forwards raw
WebSocket messages to this module via the ``joy_input`` port.

Responsibilities:
  - Scale joystick inputs → Twist
  - Manage teleop active/idle state (3s idle → auto-release)
  - Publish ``teleop_active`` so NavigationModule can pause/resume
  - Encode camera frames → JPEG → push to GatewayModule
  - Provide @skill methods for REPL / MCP

Protocol (ws://<robot>:5050/ws/teleop, handled by GatewayModule):
  Client → server  JSON text:
    {"type": "joy",  "lx": 0.5, "ly": 0.0, "az": -0.3}
    {"type": "stop"}
  Server → client  binary:
    raw JPEG bytes  (camera frame, ~10 fps)

Ports:
  In:  color_image (Image)  — camera frames to encode + forward
       joy_input   (dict)   — raw joystick message from GatewayModule WS
  Out: cmd_vel     (Twist)  — scaled joystick → CmdVelMux
       teleop_active (bool) — True while joystick is active
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
    """Joystick remote control + camera stream.

    Teleop state (active/idle/release) is managed entirely here.
    GatewayModule forwards raw WS joy messages via ``joy_input`` port.
    cmd_vel is published to CmdVelMux (not directly to the driver).
    """

    # -- Inputs --
    color_image: In[Image]
    joy_input:   In[dict]      # {"lx": float, "ly": float, "az": float}

    # -- Outputs --
    cmd_vel:        Out[Twist]
    teleop_active:  Out[bool]   # True = joystick active, False = released

    def __init__(
        self,
        max_speed: float = 0.5,
        max_yaw_rate: float = 1.0,
        release_timeout: float = 3.0,
        jpeg_quality: int = 60,
        stream_fps: float = 10.0,
        port: int = 5050,
        **kw,
    ):
        super().__init__(**kw)
        self._max_speed = max_speed
        self._max_yaw_rate = max_yaw_rate
        self._release_timeout = release_timeout
        self._jpeg_quality = jpeg_quality
        self._stream_interval = 1.0 / max(1.0, stream_fps)
        self._port = port

        # Teleop state
        self._active: bool = False
        self._last_joy_time: float = 0.0
        self._clients: int = 0

        # Gateway reference (for camera push + client count)
        self._gateway = None

        # Camera encoding
        self._encode_thread: Optional[threading.Thread] = None
        self._idle_thread: Optional[threading.Thread] = None
        self._running = False
        self._latest_raw: Optional[Any] = None
        self._raw_lock = threading.Lock()
        self._new_frame = threading.Event()

    # -- lifecycle ----------------------------------------------------------

    def setup(self) -> None:
        self.color_image.subscribe(self._on_image)
        self.color_image.set_policy("latest")
        self.joy_input.subscribe(self._on_joy)

    def start(self) -> None:
        super().start()
        self._running = True
        self._encode_thread = threading.Thread(
            target=self._encode_loop, name="teleop-encode", daemon=True
        )
        self._encode_thread.start()
        self._idle_thread = threading.Thread(
            target=self._idle_check_loop, name="teleop-idle", daemon=True
        )
        self._idle_thread.start()
        logger.info("TeleopModule started (JPEG encoder + idle checker active)")

    def stop(self) -> None:
        self._running = False
        self._new_frame.set()
        if self._encode_thread:
            self._encode_thread.join(timeout=3.0)
        if self._idle_thread:
            self._idle_thread.join(timeout=2.0)
        if self._active:
            self._release()
        self._gateway = None
        super().stop()

    def on_system_modules(self, modules: Dict[str, Any]) -> None:
        """Inject GatewayModule reference for camera push + config sharing."""
        gw = modules.get("GatewayModule")
        if gw is not None:
            self._gateway = gw
            # Share config so GatewayModule knows speed limits for display
            if hasattr(gw, "configure_teleop"):
                gw.configure_teleop(
                    max_speed=self._max_speed,
                    max_yaw=self._max_yaw_rate,
                    release_timeout=self._release_timeout,
                )
            # Give GatewayModule a reference to us for teleop state queries
            gw._teleop_module = self
            logger.info("TeleopModule: wired into GatewayModule")
        else:
            logger.warning(
                "TeleopModule: GatewayModule not found — "
                "teleop WebSocket will not be available"
            )

    # -- joystick handling --------------------------------------------------

    def _on_joy(self, msg: dict) -> None:
        """Process raw joystick input from GatewayModule WS handler."""
        lx = max(-1.0, min(1.0, float(msg.get("lx", 0)))) * self._max_speed
        ly = max(-1.0, min(1.0, float(msg.get("ly", 0)))) * self._max_speed
        az = max(-1.0, min(1.0, float(msg.get("az", 0)))) * self._max_yaw_rate

        twist = Twist(
            linear=Vector3(x=lx, y=ly, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=az),
        )
        self.cmd_vel.publish(twist)
        self._last_joy_time = time.monotonic()

        if not self._active:
            self._active = True
            self.teleop_active.publish(True)
            logger.info("TeleopModule: teleop engaged")

    def _release(self) -> None:
        """Release teleop control — publish zero velocity + active=False."""
        if self._active:
            self._active = False
            self.cmd_vel.publish(Twist())
            self.teleop_active.publish(False)
            logger.info("TeleopModule: teleop released")

    def on_client_connect(self) -> None:
        """Called by GatewayModule when a teleop WS client connects."""
        self._clients += 1

    def on_client_disconnect(self) -> None:
        """Called by GatewayModule when a teleop WS client disconnects."""
        self._clients = max(0, self._clients - 1)
        if self._clients == 0:
            self._release()

    # -- idle timeout -------------------------------------------------------

    def _idle_check_loop(self) -> None:
        """Background thread: release teleop if joystick goes quiet."""
        while self._running:
            time.sleep(0.5)
            if (self._active
                    and time.monotonic() - self._last_joy_time > self._release_timeout):
                self._release()

    # -- camera frame handling ----------------------------------------------

    def _on_image(self, img: Image) -> None:
        if self._gateway is None or self._clients == 0:
            return
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
        return {
            "active": self._active,
            "clients": self._clients,
            "port": self._port,
            "last_joy_age_ms": round(
                (time.monotonic() - self._last_joy_time) * 1000
            ) if self._last_joy_time > 0 else None,
        }

    @skill
    def force_release(self) -> str:
        """Force-release teleop control and resume autonomy."""
        self._release()
        return "Teleop released"
