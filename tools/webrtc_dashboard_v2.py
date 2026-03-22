"""
LingTu Dashboard v2 — 单路 WebRTC + YOLO/Depth 开关 + 控制面板
基于已验证的 webrtc_stream.py, 单路编码不卡顿。
"""
import asyncio, json, logging, threading, time, cv2, numpy as np
from fractions import Fraction
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from av import VideoFrame
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("dashboard")

S = {"rgb": None, "depth": None, "yolo_img": None,
     "yolo_on": False, "depth_on": False,
     "status": "", "objs": [], "rx": 0.0, "ry": 0.0}
PUB = {}

def yolo_worker():
    from ultralytics import YOLO
    m = YOLO("/home/sunrise/yolo11n-seg.pt")
    m(np.zeros((270, 480, 3), dtype=np.uint8), verbose=False)
    logger.info("YOLO ready")
    while True:
        if not S["yolo_on"] or S["rgb"] is None:
            time.sleep(0.2); continue
        r = m(S["rgb"].copy(), verbose=False)
        S["yolo_img"] = r[0].plot()
        o = []
        if r[0].boxes is not None:
            for b in r[0].boxes:
                o.append(m.names[int(b.cls[0])] + "(" + f"{float(b.conf[0]):.0%}" + ")")
        S["objs"] = o
        time.sleep(0.05)

def ros_worker():
    rclpy.init()
    n = rclpy.create_node("dash")
    c = [0]
    def rgb_cb(msg):
        c[0] += 1
        if c[0] % 2: return
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        b = cv2.flip(cv2.cvtColor(a, cv2.COLOR_RGB2BGR), -1)
        S["rgb"] = cv2.resize(b, (480, 270))
    def dep_cb(msg):
        if not S["depth_on"]: return
        h, w = msg.height, msg.width
        if msg.encoding == "16UC1":
            r = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            d = cv2.convertScaleAbs(r, alpha=255.0 / 4000.0)
        else: return
        c2 = cv2.applyColorMap(d, cv2.COLORMAP_JET)
        S["depth"] = cv2.resize(cv2.flip(c2, -1), (480, 270))
    def st_cb(msg): S["status"] = msg.data
    from nav_msgs.msg import Odometry
    def od_cb(msg):
        S["rx"] = msg.pose.pose.position.x
        S["ry"] = msg.pose.pose.position.y
    n.create_subscription(Image, "/camera/color/image_raw", rgb_cb, qos_profile_sensor_data)
    n.create_subscription(Image, "/camera/depth/image_raw", dep_cb, qos_profile_sensor_data)
    n.create_subscription(String, "/nav/planner_status", st_cb, 10)
    n.create_subscription(Odometry, "/nav/odometry", od_cb, qos_profile_sensor_data)
    PUB["ins"] = n.create_publisher(String, "/nav/semantic/instruction", 10)
    PUB["bbox"] = n.create_publisher(String, "/nav/semantic/bbox_navigate", 10)
    PUB["tag"] = n.create_publisher(String, "/nav/semantic/tag_location", 10)
    PUB["agent"] = n.create_publisher(String, "/nav/agent/input", 10)
    from geometry_msgs.msg import TwistStamped
    PUB["vel"] = n.create_publisher(TwistStamped, "/nav/cmd_vel", 10)
    logger.info("ROS2 ready")
    rclpy.spin(n)

class VTrack(MediaStreamTrack):
    kind = "video"
    def __init__(self):
        super().__init__()
        self._t = 0
    async def recv(self):
        await asyncio.sleep(1 / 15)
        self._t += 6000
        if S["yolo_on"] and S["yolo_img"] is not None:
            img = S["yolo_img"]
        elif S["rgb"] is not None:
            img = S["rgb"]
        else:
            img = np.zeros((270, 480, 3), dtype=np.uint8)
            cv2.putText(img, "Waiting...", (160, 135), 0, 0.7, (0, 255, 255), 1)
        if S["depth_on"] and S["depth"] is not None:
            d = S["depth"]
            if d.shape[:2] != img.shape[:2]:
                d = cv2.resize(d, (img.shape[1], img.shape[0]))
            img = np.hstack([img, d])
        f = VideoFrame.from_ndarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), format="rgb24")
        f.pts = self._t
        f.time_base = Fraction(1, 90000)
        return f

pcs = set()

PAGE = (
    '<!DOCTYPE html><html><head><meta charset="utf-8"><title>LingTu</title>\n'
    '<style>\n'
    '*{margin:0;padding:0;box-sizing:border-box}\n'
    'body{background:#0a0a0f;color:#ccc;font-family:monospace}\n'
    '.h{background:#111;padding:8px 16px;display:flex;align-items:center;border-bottom:1px solid #0ff3}\n'
    '.h h1{color:#0ff;font-size:16px;letter-spacing:3px;flex:1}.h .s{font-size:12px}\n'
    '.m{display:flex;height:calc(100vh - 38px)}\n'
    '.v{flex:1;background:#000;display:flex;align-items:center;justify-content:center}\n'
    '.v video{max-width:100%;max-height:100%}\n'
    '.p{width:240px;background:#111;border-left:1px solid #333;padding:10px;overflow-y:auto;font-size:11px}\n'
    '.p h3{color:#0ff;font-size:11px;margin:10px 0 4px;letter-spacing:1px}\n'
    '.b{display:block;width:100%;padding:6px;margin:3px 0;background:#1a1a2e;color:#0ff;border:1px solid #0ff3;border-radius:3px;cursor:pointer;font-family:inherit;font-size:11px}\n'
    '.b:hover{background:#0ff1}.b.d{color:#f44;border-color:#f443}\n'
    '.t{display:flex;align-items:center;gap:6px;margin:3px 0}\n'
    '.t input{accent-color:#0ff}\n'
    'input[type=text]{width:100%;padding:5px;background:#0a0a0f;border:1px solid #333;color:#fff;border-radius:3px;font-family:inherit;font-size:11px;margin:2px 0}\n'
    '.i{color:#888;margin:2px 0;font-size:10px}\n'
    '.o{color:#aaa;max-height:60px;overflow-y:auto;font-size:10px}\n'
    '.dot{display:inline-block;width:6px;height:6px;border-radius:50%;margin-right:4px}\n'
    '.on{background:#0f0}.off{background:#f44}\n'
    '</style></head><body>\n'
    '<div class="h"><h1>LINGTU</h1><div class="s" id="c"><span class="dot off"></span> ...</div></div>\n'
    '<div class="m">\n'
    '<div class="v"><video id="v" autoplay playsinline muted></video></div>\n'
    '<div class="p">\n'
    '<h3>VISION</h3>\n'
    '<div class="t"><input type="checkbox" onchange="api(\'yolo\',{enabled:this.checked})"><label>YOLO</label></div>\n'
    '<div class="t"><input type="checkbox" onchange="api(\'depth\',{enabled:this.checked})"><label>Depth</label></div>\n'
    '<div class="o" id="o"></div>\n'
    '<h3>NAVIGATE</h3>\n'
    '<input type="text" id="nt" placeholder="Target..." onkeydown="if(event.key===\'Enter\')go()">\n'
    '<button class="b" onclick="go()">Go</button>\n'
    '<button class="b" onclick="find()">Find</button>\n'
    '<button class="b" onclick="cmd(\'explore\')">Explore</button>\n'
    '<button class="b d" onclick="cmd(\'stop\')">STOP</button>\n'
    '<h3>MEMORY</h3>\n'
    '<input type="text" id="tn" placeholder="Name..." onkeydown="if(event.key===\'Enter\')tag()">\n'
    '<button class="b" onclick="tag()">Tag</button>\n'
    '<h3>AGENT</h3>\n'
    '<input type="text" id="ai" placeholder="Say..." onkeydown="if(event.key===\'Enter\'){say();this.value=\'\';}">\n'
    '<button class="b" onclick="say()">Say</button>\n'
    '<h3>STATUS</h3>\n'
    '<div class="i" id="st">...</div>\n'
    '</div></div>\n'
    '<script>\n'
    'const $=id=>document.getElementById(id);\n'
    'async function init(){\n'
    '  const pc=new RTCPeerConnection({iceServers:[]});\n'
    '  pc.ontrack=e=>{$(\'v\').srcObject=e.streams[0]};\n'
    '  pc.oniceconnectionstatechange=()=>{\n'
    '    const s=pc.iceConnectionState;\n'
    '    $(\'c\').innerHTML=s===\'connected\'?\'<span class="dot on"></span> Live\':\'<span class="dot off"></span> \'+s;\n'
    '    if(s===\'disconnected\')setTimeout(()=>location.reload(),2000);\n'
    '  };\n'
    '  pc.addTransceiver(\'video\',{direction:\'recvonly\'});\n'
    '  const o=await pc.createOffer();await pc.setLocalDescription(o);\n'
    '  const r=await(await fetch(\'/offer\',{method:\'POST\',headers:{\'Content-Type\':\'application/json\'},body:JSON.stringify({sdp:o.sdp,type:o.type})})).json();\n'
    '  await pc.setRemoteDescription(r);\n'
    '}\n'
    'function api(e,d){fetch(\'/api/\'+e,{method:\'POST\',headers:{\'Content-Type\':\'application/json\'},body:JSON.stringify(d)})}\n'
    'function cmd(c,d){fetch(\'/api/cmd\',{method:\'POST\',headers:{\'Content-Type\':\'application/json\'},body:JSON.stringify({cmd:c,...(d||{})})})}\n'
    'function go(){cmd(\'go\',{target:$(\'nt\').value})}\n'
    'function find(){cmd(\'find\',{target:$(\'nt\').value})}\n'
    'function tag(){cmd(\'tag\',{name:$(\'tn\').value})}\n'
    'function say(){cmd(\'say\',{text:$(\'ai\').value});$(\'ai\').value=\'\'}\n'
    'setInterval(async()=>{try{const d=await(await fetch(\'/api/status\')).json();$(\'st\').innerHTML=\'Pos: (\'+d.x.toFixed(1)+\', \'+d.y.toFixed(1)+\') | \'+d.planner;$(\'o\').innerHTML=(d.objects||[]).map(x=>\'<div>\'+x+\'</div>\').join(\'\')}catch(e){}},1500);\n'
    'init();\n'
    '</script></body></html>'
)

async def h_index(r):
    return web.Response(content_type="text/html", text=PAGE)

async def h_offer(r):
    p = await r.json()
    pc = RTCPeerConnection()
    pcs.add(pc)
    pc.addTrack(VTrack())
    await pc.setRemoteDescription(RTCSessionDescription(sdp=p["sdp"], type=p["type"]))
    a = await pc.createAnswer()
    await pc.setLocalDescription(a)
    return web.json_response({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type})

async def h_yolo(r):
    p = await r.json()
    S["yolo_on"] = p.get("enabled", False)
    return web.json_response({"ok": 1})

async def h_depth(r):
    p = await r.json()
    S["depth_on"] = p.get("enabled", False)
    return web.json_response({"ok": 1})

async def h_cmd(r):
    p = await r.json()
    c = p.get("cmd", "")
    m = String()
    if c == "go":
        m.data = p.get("target", "")
        PUB["ins"].publish(m)
    elif c == "find":
        m.data = json.dumps({"target": p.get("target", "")})
        PUB["bbox"].publish(m)
    elif c == "stop":
        m.data = json.dumps({"stop": True})
        PUB["bbox"].publish(m)
        from geometry_msgs.msg import TwistStamped
        PUB["vel"].publish(TwistStamped())
    elif c == "tag":
        m.data = json.dumps({"name": p.get("name", "")})
        PUB["tag"].publish(m)
    elif c == "explore":
        m.data = "explore"
        PUB["ins"].publish(m)
    elif c == "say":
        m.data = p.get("text", "")
        PUB["agent"].publish(m)
    return web.json_response({"ok": 1})

async def h_status(r):
    return web.json_response({
        "x": S["rx"], "y": S["ry"],
        "planner": S["status"], "objects": S["objs"]
    })

def main():
    threading.Thread(target=ros_worker, daemon=True).start()
    threading.Thread(target=yolo_worker, daemon=True).start()
    logger.info("Waiting for camera...")
    t = time.time()
    while S["rgb"] is None and time.time() - t < 10:
        time.sleep(0.5)
    logger.info("Camera ready" if S["rgb"] is not None else "No camera")
    app = web.Application()
    app.router.add_get("/", h_index)
    app.router.add_post("/offer", h_offer)
    app.router.add_post("/api/yolo", h_yolo)
    app.router.add_post("/api/depth", h_depth)
    app.router.add_post("/api/cmd", h_cmd)
    app.router.add_get("/api/status", h_status)
    app.on_shutdown.append(lambda a: asyncio.gather(*[pc.close() for pc in pcs]))
    logger.info("http://0.0.0.0:8070")
    web.run_app(app, host="0.0.0.0", port=8070, print=None)

if __name__ == "__main__":
    main()
