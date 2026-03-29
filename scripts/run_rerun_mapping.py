#!/usr/bin/env python3
"""Minimal SLAM mapping with Rerun 3D visualization."""
import sys, os, time
sys.path.insert(0, "src")
for d in ["src/semantic/perception", "src/semantic/planner", "src/semantic/common"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
import logging
logging.basicConfig(level=logging.INFO)
import numpy as np

# 1. rclpy first
import rclpy
rclpy.init()
print("rclpy initialized")

# 2. Rerun
import rerun as rr
rr.init("lingtu_mapping")
server_uri = rr.serve_grpc(grpc_port=9877)
rr.serve_web_viewer(open_browser=False, web_port=9090, connect_to=server_uri)
print("Rerun: http://localhost:9090")

# 3. SLAM only (no full_stack — minimal)
from slam.slam_module import SLAMModule
from slam.slam_bridge_module import SlamBridgeModule

slam = SLAMModule(backend="fastlio2")
bridge = SlamBridgeModule()

slam.setup()
bridge.setup()

# Hook bridge to log to Rerun
def on_cloud(cloud):
    if cloud.points is not None and len(cloud.points) > 0:
        rr.log("world/point_cloud", rr.Points3D(cloud.points[:, :3], radii=0.02))

def on_odom(odom):
    pos = [odom.pose.position.x, odom.pose.position.y, odom.pose.position.z]
    rr.log("world/robot", rr.Points3D([pos], radii=0.15, colors=[[255, 0, 0]]))

bridge.map_cloud._add_callback(on_cloud)
bridge.odometry._add_callback(on_odom)

slam.start()
bridge.start()

print("SLAM + Rerun running. Ctrl+C to stop.")
try:
    while True:
        time.sleep(1)
        print("odom=%d cloud=%d" % (bridge.odometry.msg_count, bridge.map_cloud.msg_count))
except KeyboardInterrupt:
    pass

bridge.stop()
slam.stop()
rclpy.shutdown()
print("DONE")
