"""轻量推流: ROS2 相机 → FFmpeg x264 → MediaMTX RTSP"""
import subprocess, threading, time, cv2, numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

proc = subprocess.Popen([
    "ffmpeg", "-y",
    "-f", "rawvideo", "-pix_fmt", "bgr24",
    "-s", "320x180", "-r", "10",
    "-i", "pipe:0",
    "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
    "-b:v", "300k", "-maxrate", "300k", "-bufsize", "100k",
    "-g", "10",
    "-f", "rtsp", "-rtsp_transport", "tcp",
    "rtsp://127.0.0.1:8554/cam"
], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def ros():
    rclpy.init()
    n = rclpy.create_node("push")
    c = [0]
    def cb(msg):
        c[0] += 1
        if c[0] % 3: return
        a = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        b = cv2.flip(cv2.cvtColor(a, cv2.COLOR_RGB2BGR), -1)
        s = cv2.resize(b, (320, 180))
        try:
            proc.stdin.write(s.tobytes())
        except:
            pass
    n.create_subscription(Image, "/camera/color/image_raw", cb, qos_profile_sensor_data)
    rclpy.spin(n)

threading.Thread(target=ros, daemon=True).start()
print("Pushing 320x180 @10fps 300kbps", flush=True)
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    proc.terminate()
