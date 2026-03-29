#!/usr/bin/env python3
"""SLAM mapping with Rerun 3D visualization — uses full_stack_blueprint."""
import sys, os, time
sys.path.insert(0, "src")
for d in ["src/semantic/perception", "src/semantic/planner", "src/semantic/common"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
import logging
logging.basicConfig(level=logging.WARNING)
import numpy as np

# Rerun setup
import rerun as rr
rr.init("lingtu_mapping")
server_uri = rr.serve_grpc(grpc_port=9877)
rr.serve_web_viewer(open_browser=False, web_port=9090, connect_to=server_uri)
print("Rerun: http://localhost:9090")

# Use full_stack (this is the path that was proven to work: odom=10 cloud=93)
from core.blueprints.full_stack import full_stack_blueprint

bp = full_stack_blueprint(
    robot="stub", slam_profile="fastlio2",
    enable_native=False, enable_semantic=False, enable_gateway=False,
    enable_map_modules=False,
)
system = bp.build()

bridge = system.modules.get("SlamBridgeModule")

def on_cloud(cloud):
    if cloud.points is not None and len(cloud.points) > 0:
        rr.log("world/point_cloud", rr.Points3D(cloud.points[:, :3], radii=0.02))

def on_odom(odom):
    pos = [odom.pose.position.x, odom.pose.position.y, odom.pose.position.z]
    rr.log("world/robot", rr.Points3D([pos], radii=0.15, colors=[[255, 0, 0]]))

if bridge:
    bridge.map_cloud._add_callback(on_cloud)
    bridge.odometry._add_callback(on_odom)
    print("Rerun callbacks hooked to SlamBridgeModule")

system.start()
print("System started. Ctrl+C to stop.")

try:
    while True:
        time.sleep(1)
        if bridge:
            o = bridge.odometry.msg_count
            c = bridge.map_cloud.msg_count
            print("odom=%d cloud=%d" % (o, c))
            if o > 0 and c > 0:
                print("DATA FLOWING — open http://localhost:9090")
except KeyboardInterrupt:
    pass

system.stop()
print("DONE")
