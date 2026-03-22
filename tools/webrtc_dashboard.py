"""
LingTu WebRTC 控制台 — 多流 + YOLO 开关 + 导航控制

功能:
  - RGB 实时流 (WebRTC, ~30fps)
  - 深度图实时流 (WebRTC)
  - YOLO 检测叠加 (异步, 不拖慢主流)
  - YOLO 分割叠加 (异步)
  - 导航状态面板
  - 控制按钮 (go/stop/tag/explore)

用法:
  python3 tools/webrtc_dashboard.py
  浏览器打开 http://192.168.66.190:8070
"""

import asyncio
import json
import logging
import threading
import time
from fractions import Fraction

import cv2
import numpy as np
from aiohttp import web

from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from av import VideoFrame

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("dashboard")

# ============================================================
# 全局状态
# ============================================================

state = {
    "rgb": None,           # 原始 BGR 帧
    "depth": None,         # 深度可视化帧 (BGR)
    "yolo_overlay": None,  # YOLO 检测结果叠加帧
    "yolo_enabled": False,
    "yolo_mode": "detect", # "detect" or "segment"
    "planner_status": "",
    "scene_objects": [],
    "robot_x": 0.0,
    "robot_y": 0.0,
}

# ROS2 发布器 (在 ros 线程中初始化)
ros_publishers = {}

# ============================================================
# YOLO 异步检测线程
# ============================================================

def yolo_thread():
    """独立线程跑 YOLO，不阻塞视频流。"""
    from ultralytics import YOLO
    model = YOLO("/home/sunrise/yolo11n-seg.pt")
    # 预热
    model(np.zeros((360, 640, 3), dtype=np.uint8), verbose=False)
    logger.info("YOLO model loaded")

    while True:
        if not state["yolo_enabled"] or state["rgb"] is None:
            time.sleep(0.1)
            continue

        img = state["rgb"].copy()
        results = model(img, verbose=False)
        annotated = results[0].plot()

        # 提取检测到的物体
        objects = []
        if results[0].boxes is not None:
            for box in results[0].boxes:
                name = model.names[int(box.cls[0])]
                conf = float(box.conf[0])
                objects.append(f"{name}({conf:.0%})")

        state["yolo_overlay"] = annotated
        state["scene_objects"] = objects
        time.sleep(0.05)  # ~15-20fps YOLO (受限于 CPU)


# ============================================================
# ROS2 线程
# ============================================================

def ros2_thread():
    rclpy.init()
    node = rclpy.create_node("dashboard")

    # RGB
    def rgb_cb(msg):
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        bgr = cv2.flip(bgr, -1)
        state["rgb"] = cv2.resize(bgr, (640, 360))

    # Depth
    def depth_cb(msg):
        h, w = msg.height, msg.width
        if msg.encoding == "16UC1":
            raw = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            # 归一化到 0-255 可视化
            d8 = cv2.convertScaleAbs(raw, alpha=255.0 / 4000.0)
        elif msg.encoding == "32FC1":
            raw = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            d8 = cv2.convertScaleAbs(raw, alpha=255.0 / 4.0)
        else:
            return
        colored = cv2.applyColorMap(d8, cv2.COLORMAP_JET)
        colored = cv2.flip(colored, -1)
        state["depth"] = cv2.resize(colored, (640, 360))

    # Planner status
    def status_cb(msg):
        state["planner_status"] = msg.data

    # Odometry
    from nav_msgs.msg import Odometry
    def odom_cb(msg):
        state["robot_x"] = msg.pose.pose.position.x
        state["robot_y"] = msg.pose.pose.position.y

    node.create_subscription(Image, "/camera/color/image_raw", rgb_cb, qos_profile_sensor_data)
    node.create_subscription(Image, "/camera/depth/image_raw", depth_cb, qos_profile_sensor_data)
    node.create_subscription(String, "/nav/planner_status", status_cb, 10)
    node.create_subscription(Odometry, "/nav/odometry", odom_cb, qos_profile_sensor_data)

    # 发布器
    ros_publishers["instruction"] = node.create_publisher(String, "/nav/semantic/instruction", 10)
    ros_publishers["bbox_nav"] = node.create_publisher(String, "/nav/semantic/bbox_navigate", 10)
    ros_publishers["tag"] = node.create_publisher(String, "/nav/semantic/tag_location", 10)
    ros_publishers["agent"] = node.create_publisher(String, "/nav/agent/input", 10)

    from geometry_msgs.msg import TwistStamped
    ros_publishers["cmd_vel"] = node.create_publisher(TwistStamped, "/nav/cmd_vel", 10)

    logger.info("ROS2 subscribed")
    rclpy.spin(node)


# ============================================================
# WebRTC Video Tracks
# ============================================================

class RGBTrack(MediaStreamTrack):
    kind = "video"
    def __init__(self):
        super().__init__()
        self._ts = 0

    async def recv(self):
        await asyncio.sleep(1 / 30)
        self._ts += 3000

        if state["yolo_enabled"] and state["yolo_overlay"] is not None:
            img = state["yolo_overlay"]
        elif state["rgb"] is not None:
            img = state["rgb"]
        else:
            img = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(img, "Waiting for camera...", (150, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        frame = VideoFrame.from_ndarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), format="rgb24")
        frame.pts = self._ts
        frame.time_base = Fraction(1, 90000)
        return frame


class DepthTrack(MediaStreamTrack):
    kind = "video"
    def __init__(self):
        super().__init__()
        self._ts = 0

    async def recv(self):
        await asyncio.sleep(1 / 15)  # 深度图 15fps 够了
        self._ts += 6000

        if state["depth"] is not None:
            img = state["depth"]
        else:
            img = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(img, "No depth", (250, 180),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 100, 255), 2)

        frame = VideoFrame.from_ndarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), format="rgb24")
        frame.pts = self._ts
        frame.time_base = Fraction(1, 90000)
        return frame


# ============================================================
# HTML 控制台
# ============================================================

HTML = """<!DOCTYPE html><html><head>
<meta charset="utf-8"><title>LingTu Dashboard</title>
<style>
* { margin:0; padding:0; box-sizing:border-box; }
body { background:#0a0a0f; color:#ccc; font-family:'Courier New',monospace; }
.header { background:#111; padding:10px 20px; display:flex; align-items:center; border-bottom:1px solid #0ff3; }
.header h1 { color:#0ff; font-size:18px; letter-spacing:3px; flex:1; }
.header .status { font-size:12px; color:#0f0; }
.main { display:flex; height:calc(100vh - 45px); }
.streams { flex:1; display:flex; flex-direction:column; padding:8px; gap:8px; }
.stream-row { display:flex; gap:8px; flex:1; }
.stream-box { flex:1; position:relative; background:#000; border:1px solid #333; border-radius:4px; overflow:hidden; }
.stream-box video { width:100%; height:100%; object-fit:contain; }
.stream-label { position:absolute; top:6px; left:10px; color:#0ff; font-size:11px; background:#0008; padding:2px 8px; border-radius:3px; }
.panel { width:280px; background:#111; border-left:1px solid #333; padding:12px; overflow-y:auto; }
.panel h3 { color:#0ff; font-size:13px; margin-bottom:8px; letter-spacing:1px; }
.panel .section { margin-bottom:16px; }
.btn { display:block; width:100%; padding:8px; margin:4px 0; background:#1a1a2e; color:#0ff; border:1px solid #0ff3;
       border-radius:4px; cursor:pointer; font-family:inherit; font-size:12px; text-align:left; }
.btn:hover { background:#0ff1; border-color:#0ff; }
.btn.active { background:#0ff2; border-color:#0ff; }
.btn.danger { color:#f44; border-color:#f443; }
.btn.danger:hover { background:#f441; }
.toggle { display:flex; align-items:center; gap:8px; margin:6px 0; font-size:12px; }
.toggle input { accent-color:#0ff; }
input[type=text] { width:100%; padding:6px 8px; background:#0a0a0f; border:1px solid #333; color:#fff;
                   border-radius:4px; font-family:inherit; font-size:12px; margin:4px 0; }
.info { font-size:11px; color:#888; margin:4px 0; }
.objects { font-size:11px; color:#aaa; max-height:100px; overflow-y:auto; }
.dot { display:inline-block; width:6px; height:6px; border-radius:50%; margin-right:4px; }
.dot.on { background:#0f0; } .dot.off { background:#f44; }
</style></head>
<body>
<div class="header">
  <h1>LINGTU DASHBOARD</h1>
  <div class="status" id="conn"><span class="dot off"></span> Connecting</div>
</div>
<div class="main">
  <div class="streams">
    <div class="stream-row">
      <div class="stream-box"><video id="rgb" autoplay playsinline muted></video><div class="stream-label">RGB Camera</div></div>
      <div class="stream-box"><video id="depth" autoplay playsinline muted></video><div class="stream-label">Depth</div></div>
    </div>
  </div>
  <div class="panel">
    <div class="section">
      <h3>VISION</h3>
      <div class="toggle"><input type="checkbox" id="yoloToggle" onchange="toggleYolo()"><label>YOLO Detection</label></div>
      <div class="toggle"><input type="checkbox" id="segToggle" onchange="toggleSeg()"><label>Instance Segmentation</label></div>
      <div class="objects" id="objects"></div>
    </div>
    <div class="section">
      <h3>NAVIGATE</h3>
      <input type="text" id="navTarget" placeholder="目标: 体育馆, 红色椅子..." onkeydown="if(event.key==='Enter')doNav()">
      <button class="btn" onclick="doNav()">Go 导航</button>
      <button class="btn" onclick="doFind()">Find 视觉搜索</button>
      <button class="btn" onclick="doExplore()">Explore 探索</button>
      <button class="btn danger" onclick="doStop()">STOP 停止</button>
    </div>
    <div class="section">
      <h3>MEMORY</h3>
      <input type="text" id="tagName" placeholder="地点名称..." onkeydown="if(event.key==='Enter')doTag()">
      <button class="btn" onclick="doTag()">Tag 标记当前位置</button>
    </div>
    <div class="section">
      <h3>AGENT</h3>
      <input type="text" id="agentInput" placeholder="对机器人说..." onkeydown="if(event.key==='Enter')doSay()">
      <button class="btn" onclick="doSay()">Say 对话</button>
    </div>
    <div class="section">
      <h3>STATUS</h3>
      <div class="info">Position: <span id="pos">-</span></div>
      <div class="info">Planner: <span id="planner">-</span></div>
      <div class="info">YOLO FPS: <span id="yoloFps">-</span></div>
    </div>
  </div>
</div>
<script>
let pc1, pc2;
const conn = document.getElementById('conn');

async function startStreams() {
  // RGB stream
  pc1 = new RTCPeerConnection({iceServers:[]});
  pc1.ontrack = e => { document.getElementById('rgb').srcObject = e.streams[0]; };
  pc1.addTransceiver('video', {direction:'recvonly'});
  let o1 = await pc1.createOffer();
  await pc1.setLocalDescription(o1);
  let r1 = await fetch('/offer/rgb', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({sdp:o1.sdp,type:o1.type})});
  let a1 = await r1.json();
  await pc1.setRemoteDescription(a1);

  // Depth stream
  pc2 = new RTCPeerConnection({iceServers:[]});
  pc2.ontrack = e => { document.getElementById('depth').srcObject = e.streams[0]; };
  pc2.addTransceiver('video', {direction:'recvonly'});
  let o2 = await pc2.createOffer();
  await pc2.setLocalDescription(o2);
  let r2 = await fetch('/offer/depth', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({sdp:o2.sdp,type:o2.type})});
  let a2 = await r2.json();
  await pc2.setRemoteDescription(a2);

  pc1.oniceconnectionstatechange = () => {
    if(pc1.iceConnectionState==='connected') conn.innerHTML='<span class="dot on"></span> Live';
    if(pc1.iceConnectionState==='disconnected') { conn.innerHTML='<span class="dot off"></span> Reconnecting'; setTimeout(()=>location.reload(),2000); }
  };
}

function toggleYolo() {
  const on = document.getElementById('yoloToggle').checked;
  fetch('/api/yolo', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enabled:on, mode:'detect'})});
}
function toggleSeg() {
  const on = document.getElementById('segToggle').checked;
  document.getElementById('yoloToggle').checked = on;
  fetch('/api/yolo', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({enabled:on, mode:'segment'})});
}
function doNav() { const t=document.getElementById('navTarget').value; if(t) fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'go',target:t})}); }
function doFind() { const t=document.getElementById('navTarget').value; if(t) fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'find',target:t})}); }
function doExplore() { fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'explore'})}); }
function doStop() { fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'stop'})}); }
function doTag() { const n=document.getElementById('tagName').value; if(n) fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'tag',name:n})}); }
function doSay() { const t=document.getElementById('agentInput').value; if(t) fetch('/api/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({cmd:'say',text:t})}); document.getElementById('agentInput').value=''; }

// 状态轮询
setInterval(async()=>{
  try {
    const r = await fetch('/api/status');
    const d = await r.json();
    document.getElementById('pos').textContent = `(${d.x.toFixed(1)}, ${d.y.toFixed(1)})`;
    document.getElementById('planner').textContent = d.planner || 'IDLE';
    document.getElementById('objects').innerHTML = (d.objects||[]).map(o=>'<div>'+o+'</div>').join('');
  } catch(e) {}
}, 1000);

startStreams().catch(e => conn.textContent = 'Error: '+e.message);
</script>
</body></html>"""


# ============================================================
# Web API
# ============================================================

pcs = set()

async def index(req):
    return web.Response(content_type="text/html", text=HTML)

async def offer_rgb(req):
    params = await req.json()
    pc = RTCPeerConnection()
    pcs.add(pc)
    pc.addTrack(RGBTrack())
    await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})

async def offer_depth(req):
    params = await req.json()
    pc = RTCPeerConnection()
    pcs.add(pc)
    pc.addTrack(DepthTrack())
    await pc.setRemoteDescription(RTCSessionDescription(sdp=params["sdp"], type=params["type"]))
    answer = await pc.createAnswer()
    await pc.setLocalDescription(answer)
    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})

async def api_yolo(req):
    params = await req.json()
    state["yolo_enabled"] = params.get("enabled", False)
    state["yolo_mode"] = params.get("mode", "detect")
    logger.info(f"YOLO: enabled={state['yolo_enabled']}, mode={state['yolo_mode']}")
    return web.json_response({"ok": True})

async def api_cmd(req):
    params = await req.json()
    cmd = params.get("cmd", "")
    msg = String()

    if cmd == "go":
        msg.data = params.get("target", "")
        ros_publishers["instruction"].publish(msg)
    elif cmd == "find":
        msg.data = json.dumps({"target": params.get("target", "")}, ensure_ascii=False)
        ros_publishers["bbox_nav"].publish(msg)
    elif cmd == "stop":
        msg.data = json.dumps({"stop": True})
        ros_publishers["bbox_nav"].publish(msg)
        from geometry_msgs.msg import TwistStamped
        twist = TwistStamped()
        ros_publishers["cmd_vel"].publish(twist)
    elif cmd == "tag":
        msg.data = json.dumps({"name": params.get("name", "")}, ensure_ascii=False)
        ros_publishers["tag"].publish(msg)
    elif cmd == "explore":
        msg.data = "explore"
        ros_publishers["instruction"].publish(msg)
    elif cmd == "say":
        msg.data = params.get("text", "")
        ros_publishers["agent"].publish(msg)

    logger.info(f"CMD: {cmd} {params}")
    return web.json_response({"ok": True})

async def api_status(req):
    return web.json_response({
        "x": state["robot_x"],
        "y": state["robot_y"],
        "planner": state["planner_status"],
        "objects": state["scene_objects"],
        "yolo": state["yolo_enabled"],
    })

async def on_shutdown(app):
    coros = [pc.close() for pc in pcs]
    await asyncio.gather(*coros)


# ============================================================
# Main
# ============================================================

def main():
    threading.Thread(target=ros2_thread, daemon=True).start()
    threading.Thread(target=yolo_thread, daemon=True).start()

    # 等相机
    logger.info("Waiting for camera...")
    t0 = time.time()
    while state["rgb"] is None and time.time() - t0 < 10:
        time.sleep(0.5)
    if state["rgb"] is not None:
        logger.info("Camera ready")
    else:
        logger.warning("No camera, starting anyway")

    app = web.Application()
    app.router.add_get("/", index)
    app.router.add_post("/offer/rgb", offer_rgb)
    app.router.add_post("/offer/depth", offer_depth)
    app.router.add_post("/api/yolo", api_yolo)
    app.router.add_post("/api/cmd", api_cmd)
    app.router.add_get("/api/status", api_status)
    app.on_shutdown.append(on_shutdown)

    logger.info("http://0.0.0.0:8070")
    web.run_app(app, host="0.0.0.0", port=8070, print=None)


if __name__ == "__main__":
    main()
