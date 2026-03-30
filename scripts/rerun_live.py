#!/usr/bin/env python3
"""Live visualization via Rerun — full robot dashboard.

Channels:
  world/point_cloud   — LiDAR point cloud (height-colored)
  world/robot         — robot body (wireframe box)
  world/heading       — orientation arrow (yellow)
  world/trajectory    — path traveled (blue line)
  world/costmap       — 2D occupancy grid (colored mesh)
  world/detections    — detected objects (labeled 3D points)
  world/nav_path      — planned navigation path (green line)
  camera/color        — RGB camera (rotated for vertical mount)
  camera/depth        — depth image
  metrics/slam_hz     — SLAM update rate
  metrics/det_count   — detection count

Usage on S100P:
    python3 scripts/rerun_live.py

View remotely:
    ssh -L 9090:127.0.0.1:9090 -L 9877:127.0.0.1:9877 sunrise@192.168.66.190
    Open http://localhost:9090
"""
import sys, os, time, math, struct
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
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import PointCloud2, Image
from visualization_msgs.msg import MarkerArray

node = Node("rerun_viz")
qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)
qos_best = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=5)

trajectory = []
counts = {"odom": 0, "cloud": 0, "color": 0, "depth": 0, "costmap": 0, "det": 0, "path": 0}
_last_odom_t = 0.0

# Robot body dimensions (half-sizes in meters) — Thunder quadruped
ROBOT_HALF = [0.35, 0.155, 0.15]


# ── Point Cloud ──────────────────────────────────────────────────────────────
def on_cloud(msg):
    counts["cloud"] += 1
    if counts["cloud"] % 5 != 0:  # throttle cloud to ~2Hz (from 10Hz)
        return
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
        if len(xyz) > 20000:
            idx = np.random.choice(len(xyz), 20000, replace=False)
            xyz = xyz[idx]
        if len(xyz) > 0:
            z = xyz[:, 2]
            z_norm = np.clip((z - z.min()) / max(z.max() - z.min(), 0.01), 0, 1)
            colors = np.zeros((len(xyz), 3), dtype=np.uint8)
            colors[:, 0] = (z_norm * 255).astype(np.uint8)       # red = high
            colors[:, 2] = ((1 - z_norm) * 255).astype(np.uint8) # blue = low
            colors[:, 1] = 80
            rr.log("world/point_cloud", rr.Points3D(xyz, colors=colors, radii=0.02))
    except Exception:
        pass


# ── Robot Pose (wireframe box + heading arrow) ───────────────────────────────
def on_odom(msg):
    global _last_odom_t
    counts["odom"] += 1
    if counts["odom"] % 2 != 0:  # throttle to ~5Hz
        return
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    x, y, z = p.x, p.y, p.z

    # Robot body — wireframe box at robot position
    rr.log("world/robot", rr.Boxes3D(
        centers=[[x, y, z + ROBOT_HALF[2]]],
        half_sizes=[ROBOT_HALF],
        colors=[[0, 255, 127]],
        fill_mode="MajorWireframe",
    ))

    # Orientation arrow
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    dx, dy = math.cos(yaw) * 0.8, math.sin(yaw) * 0.8
    rr.log("world/heading", rr.Arrows3D(
        origins=[[x, y, z + 0.3]],
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

    # SLAM Hz metric
    now = time.time()
    if _last_odom_t > 0:
        dt = now - _last_odom_t
        if dt > 0:
            rr.log("metrics/slam_hz", rr.Scalars(1.0 / dt))
    _last_odom_t = now


# ── Costmap (2D occupancy grid → colored ground plane) ───────────────────────
def on_costmap(msg):
    counts["costmap"] += 1
    if counts["costmap"] % 5 != 0:  # throttle
        return
    try:
        w = msg.info.width
        h = msg.info.height
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y

        grid = np.array(msg.data, dtype=np.int8).reshape(h, w)

        # Build colored image: green=free, red=occupied, gray=unknown
        img = np.zeros((h, w, 3), dtype=np.uint8)
        free_mask = grid == 0
        occ_mask = grid > 50
        unk_mask = grid < 0

        img[free_mask] = [40, 80, 40]      # dark green = free
        img[occ_mask] = [200, 50, 50]       # red = occupied
        img[unk_mask] = [60, 60, 60]        # gray = unknown

        # Inflation zone (1-50)
        inflate_mask = (grid > 0) & (grid <= 50)
        img[inflate_mask] = [180, 160, 40]  # yellow = inflation

        # Build mesh vertices (flat ground plane at z=0.01)
        # 4 corners of the costmap
        x0, y0 = ox, oy
        x1, y1 = ox + w * res, oy + h * res
        vertices = np.array([
            [x0, y0, 0.01],
            [x1, y0, 0.01],
            [x1, y1, 0.01],
            [x0, y1, 0.01],
        ], dtype=np.float32)
        triangles = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
        texcoords = np.array([
            [0.0, 1.0],
            [1.0, 1.0],
            [1.0, 0.0],
            [0.0, 0.0],
        ], dtype=np.float32)

        rr.log("world/costmap", rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=triangles,
            vertex_texcoords=texcoords,
            albedo_texture=img,
        ))
    except Exception:
        pass


# ── Detections (3D labeled points from MarkerArray) ──────────────────────────
def on_detections(msg):
    counts["det"] += 1
    try:
        positions = []
        labels = []
        colors = []
        for marker in msg.markers:
            if marker.action == 2:  # DELETE
                continue
            px = marker.pose.position.x
            py = marker.pose.position.y
            pz = marker.pose.position.z
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(pz)):
                continue
            positions.append([px, py, pz])
            label = marker.text if marker.text else f"obj_{marker.id}"
            labels.append(label)
            r = int(marker.color.r * 255) if marker.color.r <= 1.0 else int(marker.color.r)
            g = int(marker.color.g * 255) if marker.color.g <= 1.0 else int(marker.color.g)
            b = int(marker.color.b * 255) if marker.color.b <= 1.0 else int(marker.color.b)
            colors.append([max(r, 50), max(g, 50), max(b, 50)])

        if positions:
            rr.log("world/detections", rr.Points3D(
                positions,
                labels=labels,
                colors=colors,
                radii=0.12,
            ))
            rr.log("metrics/det_count", rr.Scalars(len(positions)))
    except Exception:
        pass


# ── Navigation Path (green line) ─────────────────────────────────────────────
def on_nav_path(msg):
    counts["path"] += 1
    try:
        pts = []
        for pose_s in msg.poses:
            p = pose_s.pose.position
            pts.append([p.x, p.y, p.z])
        if len(pts) > 1:
            rr.log("world/nav_path", rr.LineStrips3D(
                [pts],
                colors=[[0, 255, 100]],
                radii=0.04,
            ))
    except Exception:
        pass


# ── Camera Color ─────────────────────────────────────────────────────────────
def _crop_square(img):
    """Crop center square from portrait image after rotation."""
    h, w = img.shape[:2]
    if h > w:
        margin = (h - w) // 2
        return img[margin:margin + w]
    return img

def on_color(msg):
    counts["color"] += 1
    if counts["color"] % 6 != 0:  # throttle to ~5fps
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


# ── Camera Depth ─────────────────────────────────────────────────────────────
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


# ── Subscribe ────────────────────────────────────────────────────────────────
node.create_subscription(Odometry, "/nav/odometry", on_odom, qos)
node.create_subscription(PointCloud2, "/nav/map_cloud", on_cloud, qos)
node.create_subscription(Image, "/camera/color/image_raw", on_color, qos)
node.create_subscription(Image, "/camera/depth/image_raw", on_depth, qos)
node.create_subscription(OccupancyGrid, "/nav/costmap", on_costmap, qos_best)
node.create_subscription(MarkerArray, "/nav/detections", on_detections, qos_best)
node.create_subscription(Path, "/nav/path", on_nav_path, qos_best)
get_shared_executor().add_node(node)

print("Streaming: point_cloud + robot(box) + heading + trajectory + costmap + detections + nav_path + camera")
print("Ctrl+C to stop.")
try:
    while True:
        time.sleep(2)
        print("odom=%d cloud=%d color=%d depth=%d costmap=%d det=%d path=%d" % (
            counts["odom"], counts["cloud"], counts["color"], counts["depth"],
            counts["costmap"], counts["det"], counts["path"]))
except KeyboardInterrupt:
    pass

node.destroy_node()
shutdown_shared_executor()
print("Done.")
