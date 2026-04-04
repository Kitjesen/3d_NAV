#!/usr/bin/env python3
"""Quick verification of 3 fixes: API key, CLIP cache, port cleanup."""
import sys, os, time, socket
sys.path.insert(0, "src")
for d in ["src/semantic/perception", "src/semantic/planner", "src/semantic/common"]:
    if os.path.isdir(d):
        sys.path.insert(0, d)
import logging
logging.basicConfig(level=logging.WARNING)

from core.blueprints.full_stack import full_stack_blueprint

bp = full_stack_blueprint(
    robot="sim_ros2", slam_profile="bridge",
    enable_native=False, enable_semantic=True, enable_gateway=True,
)
system = bp.build()
system.start()
time.sleep(3)

# Fix 1: API key → GoalResolver alive
planner = system.modules.get("SemanticPlannerModule")
has_resolver = planner._goal_resolver is not None if planner else False
print("FIX1 LLM_KEY: GoalResolver=%s" % has_resolver)

# Fix 2: CLIP model cached
enc = system.modules.get("EncoderModule")
has_backend = hasattr(enc, "_backend") and enc._backend is not None if enc else False
print("FIX2 CLIP: backend_loaded=%s" % has_backend)

# Fix 3: Ports open (no conflict)
for port in [5050, 8090]:
    s = socket.socket()
    s.settimeout(1)
    try:
        s.connect(("127.0.0.1", port))
        print("FIX3 PORT_%d: OPEN" % port)
        s.close()
    except Exception:
        print("FIX3 PORT_%d: CLOSED" % port)

# ROS2 bridges
slam = system.modules.get("SlamBridgeModule")
cam = system.modules.get("CameraBridgeModule")
print("ROS2_SLAM: node=%s" % (slam._node is not None if slam else False))
print("ROS2_CAM: node=%s" % (cam._node is not None if cam else False))

system.stop()
print("ALL_DONE")
