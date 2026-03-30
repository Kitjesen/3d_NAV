#!/usr/bin/env python3
"""Live point cloud visualization via Rerun. SSH tunnel to view remotely.

Usage on S100P:
    python3 scripts/rerun_live.py

View remotely (run on your PC):
    ssh -L 9090:127.0.0.1:9090 -L 9877:127.0.0.1:9877 sunrise@192.168.66.190
    Open http://localhost:9090

Shows: point cloud (white), robot position (red dot), trajectory (blue line).
"""
import sys, os, time
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import logging
logging.basicConfig(level=logging.WARNING)
import numpy as np

# rclpy first
from core.ros2_context import ensure_rclpy, get_shared_executor, shutdown_shared_executor
ensure_rclpy()

# Rerun
import rerun as rr
rr.init("lingtu_live")
server_uri = rr.serve_grpc(grpc_port=9877)
rr.serve_web_viewer(open_browser=False, web_port=9090, connect_to=server_uri)
print("Rerun: http://localhost:9090")
print("SSH tunnel: ssh -L 9090:127.0.0.1:9090 -L 9877:127.0.0.1:9877 sunrise@192.168.66.190")

# Subscribe to ROS2 topics
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import struct

node = Node("rerun_viz")
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)

trajectory = []
counts = {"odom": 0, "cloud": 0}

def on_odom(msg):
    counts["odom"] += 1
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    rr.log("world/robot", rr.Points3D([[x, y, z]], radii=0.15, colors=[[255, 0, 0]]))
    trajectory.append([x, y, z])
    if len(trajectory) > 2:
        rr.log("world/trajectory", rr.LineStrips3D([trajectory[-500:]],
               colors=[[0, 100, 255]]))

def on_cloud(msg):
    counts["cloud"] += 1
    try:
        n = msg.width * msg.height
        step = msg.point_step
        if step == 16 and n > 0:
            pts = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)[:, :3]
        else:
            return
        valid = np.isfinite(pts).all(axis=1)
        pts = pts[valid]
        if len(pts) > 0:
            # Subsample for performance
            if len(pts) > 50000:
                idx = np.random.choice(len(pts), 50000, replace=False)
                pts = pts[idx]
            rr.log("world/point_cloud", rr.Points3D(pts, radii=0.02))
    except Exception:
        pass

node.create_subscription(Odometry, "/nav/odometry", on_odom, qos)
node.create_subscription(PointCloud2, "/nav/map_cloud", on_cloud, qos)
get_shared_executor().add_node(node)

print("Streaming... Ctrl+C to stop.")
try:
    while True:
        time.sleep(2)
        print("odom=%d cloud=%d traj=%d" % (counts["odom"], counts["cloud"], len(trajectory)))
except KeyboardInterrupt:
    pass

node.destroy_node()
shutdown_shared_executor()
print("Done.")
