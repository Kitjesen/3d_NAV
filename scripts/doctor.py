#!/usr/bin/env python3
"""lingtu doctor — check all services, hardware, and data flow."""
import sys, os, subprocess, socket, time

def check(name, ok, detail=""):
    mark = "\033[32mOK\033[0m" if ok else "\033[31mFAIL\033[0m"
    print("  [%s] %s%s" % (mark, name, ("  " + detail) if detail else ""))
    return ok

def service_active(name):
    try:
        r = subprocess.run(["systemctl", "is-active", "--quiet", name], timeout=3)
        return r.returncode == 0
    except:
        return False

def port_open(host, port):
    s = socket.socket()
    s.settimeout(1)
    try:
        s.connect((host, port))
        s.close()
        return True
    except:
        return False

def run(cmd, timeout=5):
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip()
    except:
        return ""

print("\n  \033[1mLingTu Doctor\033[0m\n")

# 1. systemd services
print("  --- Services ---")
for svc in ["brainstem", "lidar", "camera"]:
    check(svc, service_active(svc))
for svc in ["slam", "slam_pgo", "localizer"]:
    active = service_active(svc)
    check(svc, True, "active" if active else "inactive (on-demand)")

# 2. Hardware
print("\n  --- Hardware ---")
lsusb = run(["lsusb"])
check("LiDAR (Livox)", "192.168.1.115" in run(["ping", "-c1", "-W1", "192.168.1.115"]), "192.168.1.115")
check("Camera (Orbbec)", "orbbec" in lsusb.lower() or "2bc5" in lsusb.lower())
check("IMU (CP210x)", "cp210x" in lsusb.lower() or "10c4" in lsusb.lower())
check("Xbox Controller", "8bitdo" in lsusb.lower() or "xbox" in lsusb.lower())
check("CAN Bus", os.path.exists("/sys/class/net/can0"))

# 3. Ports
print("\n  --- Ports ---")
check("brainstem gRPC :13145", port_open("127.0.0.1", 13145))
for name, port in [("Gateway :5050", 5050), ("MCP :8090", 8090), ("Teleop :5060", 5060)]:
    if port_open("127.0.0.1", port):
        check(name, True)
    else:
        print("  [\033[33m--\033[0m] %s  (lingtu not running)" % name)

# 4. ROS2 topics
print("\n  --- ROS2 Topics ---")
topics = run(["bash", "-c", "source /opt/ros/humble/setup.bash && source /opt/nova/lingtu/v1.8.0/install/setup.bash && ros2 topic list 2>/dev/null"], timeout=5)
for topic in ["/nav/odometry", "/nav/map_cloud", "/nav/registered_cloud",
              "/camera/color/image_raw", "/camera/depth/image_raw", "/nav/lidar_scan"]:
    check(topic, topic in topics)

# 5. Data flow (quick 3s test)
# Auto-start slam for accurate odom measurement
slam_was_off = not service_active("slam")
if slam_was_off:
    print("\n  [..] Starting slam for data flow test...")
    subprocess.run(["sudo", "systemctl", "start", "slam"], capture_output=True, timeout=5)
    time.sleep(8)  # SLAM launch file needs ~8s to initialize + DDS discovery
print("\n  --- Data Flow (3s) ---")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
try:
    from core.ros2_context import ensure_rclpy, get_shared_executor
    ensure_rclpy()
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, Image

    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
    counts = {"odom": 0, "cloud": 0, "color": 0, "depth": 0}
    node = Node("doctor")
    node.create_subscription(Odometry, "/nav/odometry", lambda m: counts.__setitem__("odom", counts["odom"]+1), qos)
    node.create_subscription(PointCloud2, "/nav/map_cloud", lambda m: counts.__setitem__("cloud", counts["cloud"]+1), qos)
    node.create_subscription(Image, "/camera/color/image_raw", lambda m: counts.__setitem__("color", counts["color"]+1), qos)
    node.create_subscription(Image, "/camera/depth/image_raw", lambda m: counts.__setitem__("depth", counts["depth"]+1), qos)
    ex = get_shared_executor()
    ex.add_node(node)
    time.sleep(3)
    check("SLAM odometry", counts["odom"] > 0, "%d msgs (%.0f Hz)" % (counts["odom"], counts["odom"]/3))
    check("SLAM cloud", counts["cloud"] > 0, "%d msgs" % counts["cloud"])
    check("Camera color", counts["color"] > 0, "%d msgs" % counts["color"])
    check("Camera depth", counts["depth"] > 0, "%d msgs" % counts["depth"])
    node.destroy_node()
    from core.ros2_context import shutdown_shared_executor
    shutdown_shared_executor()
except Exception as e:
    print("  [SKIP] Data flow test: %s" % e)

if slam_was_off:
    subprocess.run(["sudo", "systemctl", "stop", "slam"], capture_output=True, timeout=5)

# 6. Maps
print("\n  --- Maps ---")
map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps"))
if os.path.isdir(map_dir):
    maps = [d for d in os.listdir(map_dir) if os.path.isdir(os.path.join(map_dir, d)) and d != "active"]
    active = os.path.basename(os.readlink(os.path.join(map_dir, "active"))) if os.path.islink(os.path.join(map_dir, "active")) else "none"
    check("Maps directory", True, "%d maps, active=%s" % (len(maps), active))
else:
    check("Maps directory", False, map_dir)

# 7. Python env
print("\n  --- Python ---")
check("lingtu importable", True)
try:
    from core.blueprints.full_stack import full_stack_blueprint
    check("full_stack_blueprint", True)
except Exception as e:
    check("full_stack_blueprint", False, str(e))

print()
