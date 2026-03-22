"""LingTu 相机流 — 缩小 JPEG + WebSocket 推送"""
import asyncio, threading, time, cv2, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from aiohttp import web

latest = {"jpeg": None}

def ros_thread():
    rclpy.init()
    n = rclpy.create_node("cam0")
    c = [0]
    def cb(msg):
        c[0] += 1
        if c[0] % 3: return
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        b = cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        b = cv2.flip(b, -1)
        s = cv2.resize(b, (320, 180))
        _, jp = cv2.imencode(".jpg", s, [cv2.IMWRITE_JPEG_QUALITY, 50])
        latest["jpeg"] = jp.tobytes()
    n.create_subscription(Image, "/camera/color/image_raw", cb, qos_profile_sensor_data)
    print("ROS2 subscribed", flush=True)
    rclpy.spin(n)

PAGE = b"""<!DOCTYPE html><html><head><meta charset="utf-8"><title>LingTu</title>
<style>*{margin:0;padding:0}body{background:#111;display:flex;flex-direction:column;align-items:center;justify-content:center;height:100vh;font-family:monospace;color:#0ff}
img{max-width:95vw;max-height:85vh}#i{margin-top:8px;font-size:13px;opacity:0.7}</style></head>
<body><img id="v"/><div id="i">...</div>
<script>
const img=document.getElementById('v'),info=document.getElementById('i');
let fc=0,t0=performance.now(),ws;
function go(){
  ws=new WebSocket('ws://'+location.host+'/ws');
  ws.binaryType='blob';
  ws.onopen=()=>{info.textContent='Connected';};
  ws.onmessage=e=>{
    const u=URL.createObjectURL(e.data);
    img.onload=()=>{URL.revokeObjectURL(u);ws.send('n');};
    img.src=u;
    fc++;if(fc%10===0){info.textContent='LingTu Live | '+(10000/(performance.now()-t0)).toFixed(1)+' fps';t0=performance.now();}
  };
  ws.onclose=()=>{info.textContent='Reconnecting...';setTimeout(go,1000);};
}
go();
</script></body></html>"""

async def h_index(r):
    return web.Response(body=PAGE, content_type="text/html")

async def h_ws(r):
    ws = web.WebSocketResponse()
    await ws.prepare(r)
    print("Client connected", flush=True)
    # 先发一帧
    if latest["jpeg"]:
        await ws.send_bytes(latest["jpeg"])
    # 客户端确认收到后再发下一帧 (背压控制)
    async for msg in ws:
        if msg.type == web.WSMsgType.TEXT and msg.data == "n":
            if latest["jpeg"]:
                await ws.send_bytes(latest["jpeg"])
        elif msg.type == web.WSMsgType.ERROR:
            break
    return ws

def main():
    threading.Thread(target=ros_thread, daemon=True).start()
    time.sleep(3)
    print(f"JPEG size: {len(latest['jpeg'])//1024}KB" if latest["jpeg"] else "No frame yet", flush=True)
    app = web.Application()
    app.router.add_get("/", h_index)
    app.router.add_get("/ws", h_ws)
    print("http://0.0.0.0:8070", flush=True)
    web.run_app(app, host="0.0.0.0", port=8070, print=None)

if __name__ == "__main__":
    main()
