"""
LingTu WebRTC 实时相机流 — 浏览器零延迟查看

架构:
  ROS2 /camera/color/image_raw → VideoStreamTrack → WebRTC → 浏览器

用法:
  python3 tools/webrtc_stream.py
  # 浏览器打开 http://192.168.66.190:8070

延迟: ~50-100ms (WebRTC P2P, 无服务端缓冲)
"""

import asyncio
import json
import logging
import threading
import time

import cv2
import numpy as np
from aiohttp import web

# WebRTC
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.media import MediaRelay
from fractions import Fraction
from av import VideoFrame

# ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("webrtc")

# ============================================================
# ROS2 相机帧获取
# ============================================================

latest_frame = {"img": None, "ts": 0}


def ros2_camera_thread():
    rclpy.init()
    node = rclpy.create_node("webrtc_cam")

    def cb(msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        bgr = cv2.flip(bgr, -1)  # 相机装反，翻转
        latest_frame["img"] = cv2.resize(bgr, (640, 360))
        latest_frame["ts"] = time.time()

    node.create_subscription(Image, "/camera/color/image_raw", cb, qos_profile_sensor_data)
    logger.info("ROS2 subscribed to /camera/color/image_raw")
    rclpy.spin(node)


# ============================================================
# WebRTC Video Track
# ============================================================

class CameraVideoTrack(MediaStreamTrack):
    kind = "video"

    def __init__(self):
        super().__init__()
        self._start = time.time()
        self._timestamp = 0

    async def recv(self):
        # 等待新帧 (30fps target)
        await asyncio.sleep(1 / 30)

        img = latest_frame["img"]
        if img is None:
            # 黑屏占位
            img = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(img, "Waiting for camera...", (150, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        # BGR → RGB for VideoFrame
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        frame = VideoFrame.from_ndarray(rgb, format="rgb24")

        self._timestamp += 3000  # 90kHz clock / 30fps
        frame.pts = self._timestamp
        frame.time_base = Fraction(1, 90000)

        return frame


# ============================================================
# Web Server + Signaling
# ============================================================

pcs = set()

HTML = """<!DOCTYPE html><html><head>
<title>LingTu Live</title>
<style>
* { margin: 0; padding: 0; box-sizing: border-box; }
body { background: #0a0a0f; display: flex; flex-direction: column;
       align-items: center; justify-content: center; height: 100vh;
       font-family: 'Courier New', monospace; color: #0ff; }
video { max-width: 95vw; max-height: 80vh; background: #000;
        border: 1px solid #0ff3; border-radius: 4px; }
#info { margin-top: 12px; font-size: 13px; opacity: 0.7; }
h2 { margin-bottom: 16px; font-weight: normal; letter-spacing: 2px; }
.dot { display: inline-block; width: 8px; height: 8px;
       border-radius: 50%; margin-right: 6px; }
.live { background: #0f0; animation: blink 1s infinite; }
@keyframes blink { 50% { opacity: 0.3; } }
</style></head>
<body>
<h2>LINGTU <span style="color:#666">|</span> LIVE</h2>
<video id="vid" autoplay playsinline muted></video>
<div id="info"><span class="dot"></span>Connecting...</div>
<script>
async function start() {
    const info = document.getElementById('info');
    const vid = document.getElementById('vid');
    const pc = new RTCPeerConnection({iceServers: []});

    pc.ontrack = e => { vid.srcObject = e.streams[0]; };
    pc.oniceconnectionstatechange = () => {
        if (pc.iceConnectionState === 'connected') {
            info.innerHTML = '<span class="dot live"></span>Live · WebRTC P2P';
        } else if (pc.iceConnectionState === 'disconnected') {
            info.innerHTML = '<span class="dot"></span>Disconnected';
            setTimeout(()=>location.reload(), 2000);
        }
    };

    pc.addTransceiver('video', {direction: 'recvonly'});

    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);

    const resp = await fetch('/offer', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({sdp: offer.sdp, type: offer.type})
    });
    const answer = await resp.json();
    await pc.setRemoteDescription(answer);
}
start().catch(e => {
    document.getElementById('info').textContent = 'Error: ' + e.message;
});
</script>
</body></html>"""


async def index(request):
    return web.Response(content_type="text/html", text=HTML)


async def offer(request):
    params = await request.json()

    pc = RTCPeerConnection()
    pcs.add(pc)

    @pc.on("connectionstatechange")
    async def on_state():
        logger.info(f"Connection state: {pc.connectionState}")
        if pc.connectionState == "failed":
            await pc.close()
            pcs.discard(pc)

    # 添加视频轨道
    video_track = CameraVideoTrack()
    pc.addTrack(video_track)

    # 处理 offer → answer
    await pc.setRemoteDescription(
        RTCSessionDescription(sdp=params["sdp"], type=params["type"])
    )
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)

    return web.json_response({
        "sdp": pc.localDescription.sdp,
        "type": pc.localDescription.type,
    })


async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)
    pcs.clear()


# ============================================================
# Main
# ============================================================

def main():
    # 启动 ROS2 线程
    t = threading.Thread(target=ros2_camera_thread, daemon=True)
    t.start()

    # 等待第一帧
    logger.info("Waiting for camera frame...")
    t0 = time.time()
    while latest_frame["img"] is None and time.time() - t0 < 10:
        time.sleep(0.5)

    if latest_frame["img"] is not None:
        logger.info(f"Camera ready: {latest_frame['img'].shape}")
    else:
        logger.warning("No camera frame yet, starting anyway")

    # Web 服务
    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_post("/offer", offer)
    app.on_shutdown.append(on_shutdown)

    port = 8070
    logger.info(f"http://0.0.0.0:{port}")
    web.run_app(app, host="0.0.0.0", port=port, print=None)


if __name__ == "__main__":
    main()
