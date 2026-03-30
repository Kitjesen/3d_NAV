#!/usr/bin/env python3
"""Live visualization via Rerun — point cloud, camera, robot pose, trajectory.

Usage on S100P:
    python3 scripts/rerun_live.py

View remotely:
    ssh -L 9090:127.0.0.1:9090 -L 9877:127.0.0.1:9877 sunrise@192.168.66.190
    Open http://localhost:9090
"""
import sys, os, time, math
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import logging
logging.basicConfig(level=logging.WARNING)
import numpy as np

from core.ros2_context import ensure_rclpy, get_shared_executor, shutdown_shared_executor
ensure_rclpy()

import rerun as rr
rr.init("lingtu_live")
server_uri = rr.serve_grpc(grpc_port=9877)
rr.serve_web_viewer(open_browser=False, web_port=9090, connect_to=server_uri)
print("Rerun: http://localhost:9090")

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, Image, CompressedImage

node = Node("rerun_viz")
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)

trajectory = []
counts = {"odom": 0, "cloud": 0, "color": 0, "depth": 0}


# ── Point Cloud ──
def on_cloud(msg):
    counts["cloud"] += 1
    try:
        n = msg.width * msg.height
        if n == 0:
            return
        step = msg.point_step
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        xyz = np.zeros((n, 3), dtype=np.float32)
        xyz[:, 0] = np.frombuffer(raw[:, 0:4].tobytes(), dtype=np.float32)
        xyz[:, 1] = np.frombuffer(raw[:, 4:8].tobytes(), dtype=np.float32)
        xyz[:, 2] = np.frombuffer(raw[:, 8:12].tobytes(), dtype=np.float32)
        valid = np.isfinite(xyz).all(axis=1)
        xyz = xyz[valid]
        if len(xyz) > 50000:
            idx = np.random.choice(len(xyz), 50000, replace=False)
            xyz = xyz[idx]
        if len(xyz) > 0:
            # Color by height
            z = xyz[:, 2]
            z_norm = np.clip((z - z.min()) / max(z.max() - z.min(), 0.01), 0, 1)
            colors = np.zeros((len(xyz), 3), dtype=np.uint8)
            colors[:, 0] = (z_norm * 255).astype(np.uint8)       # red = high
            colors[:, 2] = ((1 - z_norm) * 255).astype(np.uint8) # blue = low
            colors[:, 1] = 80
            rr.log("world/point_cloud", rr.Points3D(xyz, colors=colors, radii=0.02))
    except Exception:
        pass


# ── Robot Pose (with orientation arrow) ──
def on_odom(msg):
    counts["odom"] += 1
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    x, y, z = p.x, p.y, p.z

    # Robot position
    rr.log("world/robot", rr.Points3D([[x, y, z]], radii=0.15, colors=[[255, 0, 0]]))

    # Orientation arrow
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    dx, dy = math.cos(yaw) * 0.8, math.sin(yaw) * 0.8
    rr.log("world/heading", rr.Arrows3D(
        origins=[[x, y, z + 0.1]],
        vectors=[[dx, dy, 0]],
        colors=[[255, 255, 0]],
        radii=0.05,
    ))

    # Trajectory
    trajectory.append([x, y, z])
    if len(trajectory) > 2:
        rr.log("world/trajectory", rr.LineStrips3D(
            [trajectory[-1000:]],
            colors=[[0, 100, 255]],
        ))


# ── Camera Color ──
def _crop_square(img):
    """Crop center square from portrait image after rotation."""
    h, w = img.shape[:2]
    if h > w:
        margin = (h - w) // 2
        return img[margin:margin + w]
    return img

def on_color(msg):
    counts["color"] += 1
    if counts["color"] % 3 != 0:  # throttle to ~10fps
        return
    try:
        h, w = msg.height, msg.width
        encoding = msg.encoding.lower()
        if encoding in ("bgr8", "rgb8"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
            if encoding == "bgr8":
                img = img[:, :, ::-1]  # BGR → RGB
            img = _crop_square(np.rot90(img, k=1))  # vertical mount → square
            rr.log("camera/color", rr.Image(img))
        elif encoding in ("mono8", "8uc1"):
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
            img = _crop_square(np.rot90(img, k=1))
            rr.log("camera/color", rr.Image(img))
    except Exception:
        pass


# ── Camera Depth ──
def on_depth(msg):
    counts["depth"] += 1
    if counts["depth"] % 5 != 0:  # throttle
        return
    try:
        h, w = msg.height, msg.width
        encoding = msg.encoding.lower()
        if encoding in ("16uc1",):
            img = np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w)
            img = _crop_square(np.rot90(img, k=1))
            rr.log("camera/depth", rr.DepthImage(img, meter=1000.0))
        elif encoding in ("32fc1",):
            img = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
            img = _crop_square(np.rot90(img, k=1))
            rr.log("camera/depth", rr.DepthImage(img, meter=1.0))
    except Exception:
        pass


# Subscribe
node.create_subscription(Odometry, "/nav/odometry", on_odom, qos)
node.create_subscription(PointCloud2, "/nav/map_cloud", on_cloud, qos)
node.create_subscription(Image, "/camera/color/image_raw", on_color, qos)
node.create_subscription(Image, "/camera/depth/image_raw", on_depth, qos)
get_shared_executor().add_node(node)

print("Streaming: point_cloud + robot + trajectory + camera color/depth")
print("Ctrl+C to stop.")
try:
    while True:
        time.sleep(2)
        print("odom=%d cloud=%d color=%d depth=%d" % (
            counts["odom"], counts["cloud"], counts["color"], counts["depth"]))
except KeyboardInterrupt:
    pass

node.destroy_node()
shutdown_shared_executor()
print("Done.")
