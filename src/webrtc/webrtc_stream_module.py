"""WebRTCStreamModule — browser video over WebRTC.

Consumes the CameraBridge ``color_image`` stream (BGR8 numpy) and exposes
it as a WebRTC video track.  Browsers connect through a single
``POST /api/v1/webrtc/offer`` signalling endpoint served by
GatewayModule's FastAPI app; the module plugs its routes into the
existing uvicorn process so we stay on port 5050.

Why this over JPEG-over-WebSocket
---------------------------------
JPEG-over-WS is ~4 Mbit/s and 200-400 ms glass-to-glass because every
frame is re-encoded as a standalone image and decoded synchronously on
the browser's main thread.  WebRTC with H.264 is ~1-2 Mbit/s, uses the
browser's hardware decoder, and runs below 150 ms on a LAN.

Encoder path
------------
Phase A (this file) uses PyAV's libx264 — pure software, ~30 % of one
A78AE core at 720p30.  Phase B will swap to D-Robotics ``hobot_codec``
(BPU H.264 at 5.5 ms/frame, <5 % CPU) by feeding pre-encoded NALUs
through aiortc's ``H264PayloadContext`` without changing the wire
protocol or the browser side.

Fallback
--------
If aiortc / PyAV are unavailable at import, the module becomes a no-op
so the rest of the stack still boots — the JPEG-over-WS path remains
the only video route.
"""

from __future__ import annotations

import asyncio
import logging
import threading
from typing import Any

import numpy as np

from core.module import Module
from core.msgs.sensor import Image
from core.registry import register
from core.stream import In

logger = logging.getLogger(__name__)


try:
    from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
    from aiortc.contrib.media import MediaRelay
    from av import VideoFrame
    _HAVE_WEBRTC = True
except ImportError as _exc:  # pragma: no cover — exercised by deployment env
    logger.info("WebRTCStreamModule disabled: %s", _exc)
    _HAVE_WEBRTC = False


if _HAVE_WEBRTC:

    class _LatestFrameTrack(VideoStreamTrack):
        """aiortc track that yields whatever the producer last wrote.

        ``recv()`` awaits an asyncio.Event poked from the Module's
        producer thread; this rate-limits the encoder to the source's
        framerate (30 Hz) without any polling.  Multiple subscribers
        share one encoded stream via :class:`MediaRelay` upstream.
        """

        kind = "video"

        def __init__(self, module: "WebRTCStreamModule") -> None:
            super().__init__()
            self._mod = module

        async def recv(self):  # type: ignore[override]
            await self._mod._frame_event.wait()
            self._mod._frame_event.clear()
            bgr = self._mod._latest_bgr
            if bgr is None:
                # Event fired with no frame attached — wait for the next one
                # instead of returning garbage.
                return await self.recv()
            pts, time_base = await self.next_timestamp()
            frame = VideoFrame.from_ndarray(bgr, format="bgr24")
            frame.pts = pts
            frame.time_base = time_base
            return frame


def _force_h264_first(sdp: str) -> str:
    """Move H.264 payload types ahead of VP8/VP9 in every m=video line.

    iOS Safari only uses the first codec it recognises; leaving VP8 first
    means iPhones get a black tile.  We keep all codecs so desktop
    browsers still negotiate something useful if H.264 fails.
    """
    lines = sdp.split("\r\n")
    out: list[str] = []
    h264_pts: list[str] = []
    # First pass: discover which payload types map to H264.
    for line in lines:
        if line.startswith("a=rtpmap:") and "H264" in line:
            pt = line.split(":", 1)[1].split(" ", 1)[0]
            h264_pts.append(pt)
    if not h264_pts:
        return sdp
    for line in lines:
        if line.startswith("m=video "):
            head, _, tail = line.partition(" UDP/TLS/RTP/SAVPF ")
            if not tail:
                out.append(line)
                continue
            pts = tail.split(" ")
            reordered = [pt for pt in h264_pts if pt in pts] + [
                pt for pt in pts if pt not in h264_pts
            ]
            out.append(f"{head} UDP/TLS/RTP/SAVPF {' '.join(reordered)}")
        else:
            out.append(line)
    return "\r\n".join(out)


@register("webrtc", "aiortc", description="WebRTC H.264 video via aiortc + PyAV")
class WebRTCStreamModule(Module, layer=6):
    """Stream camera frames to the browser over WebRTC."""

    color_image: In[Image]

    _run_in_main: bool = False

    def __init__(self, **kw: Any) -> None:
        super().__init__(**kw)
        self._latest_bgr: np.ndarray | None = None
        self._raw_lock = threading.Lock()
        # asyncio primitives are created lazily on the gateway's event loop
        # (see on_system_modules / register_routes).
        self._frame_event: asyncio.Event | None = None  # type: ignore[assignment]
        self._loop: asyncio.AbstractEventLoop | None = None
        self._pcs: set = set()
        self._source_track: Any = None
        self._relay: Any = None
        self._gateway: Any = None

    # -- Module lifecycle ---------------------------------------------------

    def setup(self) -> None:
        self.color_image.subscribe(self._on_image)
        self.color_image.set_policy("latest")

    def on_system_modules(self, modules: dict[str, Any]) -> None:
        """Pick up the gateway reference; actual route is served by
        GatewayModule's ``/api/v1/webrtc/offer`` handler which delegates
        back to :meth:`handle_offer`.  Routes can't be added after the
        dashboard static mount, so this indirection keeps the single-port
        architecture intact.
        """
        self._gateway = modules.get("GatewayModule")
        if self._gateway is None:
            logger.warning(
                "WebRTCStreamModule: GatewayModule not found — signalling "
                "will not work",
            )

    def teardown(self) -> None:
        if self._loop is None:
            return
        for pc in list(self._pcs):
            try:
                asyncio.run_coroutine_threadsafe(pc.close(), self._loop)
            except Exception:
                pass
        self._pcs.clear()

    # -- Image callback (Module thread) -------------------------------------

    def _on_image(self, img: Image) -> None:
        data = getattr(img, "data", None)
        if data is None:
            return
        with self._raw_lock:
            self._latest_bgr = data
        ev = self._frame_event
        loop = self._loop
        if ev is not None and loop is not None:
            loop.call_soon_threadsafe(ev.set)

    # -- Signalling (asyncio, called from GatewayModule route) -------------

    async def handle_offer(self, payload: dict) -> dict:
        """Single-shot offer/answer — ~20 LOC, no TURN, no reconnection state.

        Expected payload: ``{"sdp": "...", "type": "offer"}``.
        Returns: ``{"sdp": "...", "type": "answer"}``.
        """
        if self._loop is None:
            self._loop = asyncio.get_running_loop()
            self._frame_event = asyncio.Event()
            self._source_track = _LatestFrameTrack(self)
            self._relay = MediaRelay()

        sdp = payload.get("sdp")
        kind = payload.get("type", "offer")
        if not isinstance(sdp, str) or kind != "offer":
            raise ValueError("webrtc offer requires {'sdp': str, 'type': 'offer'}")

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        @pc.on("connectionstatechange")
        async def _on_state() -> None:
            if pc.connectionState in ("failed", "closed", "disconnected"):
                await pc.close()
                self._pcs.discard(pc)

        pc.addTrack(self._relay.subscribe(self._source_track))

        await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type="offer"))
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        return {
            "sdp": _force_h264_first(pc.localDescription.sdp),
            "type": pc.localDescription.type,
        }

    # -- Diagnostics --------------------------------------------------------

    def health(self) -> dict[str, Any]:
        info = super().port_summary() if hasattr(super(), "port_summary") else {}
        info.update({
            "enabled": _HAVE_WEBRTC,
            "active_peers": len(self._pcs),
            "has_frame": self._latest_bgr is not None,
        })
        return info
