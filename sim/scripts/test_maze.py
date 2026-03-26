"""Go1 maze navigation -- complex obstacle layout.

Blueprint pipeline: Go1SimDriverModule -> LiveMapper -> NavigationModule
  -> LocalPlannerModule -> PathFollowerModule (closed loop)

Obstacles: L-shaped corridor with dead ends and narrow passages.
Start: (0, 1.5), Goal: (7, 1.5)
"""

from __future__ import annotations

import logging
import math
import os
import sys
import time
import threading
from typing import List, Optional

import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")

REPO = os.path.expanduser("~/data/SLAM/navigation")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, REPO)

OUTPUT = os.path.join(REPO, "maze_nav.mp4")

from core.module import Module
from core.blueprint import Blueprint
from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion
from core.msgs.nav import Odometry
from core.msgs.sensor import PointCloud

from drivers.sim.go1_sim_driver import Go1SimDriverModule
from nav.navigation_module import NavigationModule, MissionState
from base_autonomy.modules.local_planner_module import LocalPlannerModule
from base_autonomy.modules.path_follower_module import PathFollowerModule

from sim.engine.core.world import ObstacleConfig

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(name)s %(levelname)s %(message)s")
logger = logging.getLogger("maze_nav")


class LiveMapper(Module, layer=3):
    """Builds a 200x200 occupancy grid from LiDAR + odometry."""

    lidar_in: In[PointCloud]
    odom_in: In[Odometry]

    costmap_out: Out[dict]
    goal_cmd: Out[PoseStamped]

    GS = 200
    GR = 0.2

    def __init__(self, goal_xy=(7.0, 1.5), goal_delay=3.0, **kw):
        super().__init__(**kw)
        self._goal_xy = goal_xy
        self._goal_delay = goal_delay

        self._lock = threading.Lock()
        self.grid = np.zeros((self.GS, self.GS), dtype=np.float32)
        self.lidar_pts: np.ndarray = np.zeros((0, 3), dtype=np.float32)
        self.trail: List[tuple] = []
        self.scans = 0
        self.rx = 0.0
        self.ry = 0.0

        self._start_time: Optional[float] = None
        self._goal_sent = False

    def setup(self):
        self.lidar_in.subscribe(self._on_lidar)
        self.odom_in.subscribe(self._on_odom)

    def start(self):
        super().start()
        self._start_time = time.time()

    def _on_odom(self, odom: Odometry):
        x = odom.pose.position.x
        y = odom.pose.position.y
        with self._lock:
            self.rx = x
            self.ry = y
            if len(self.trail) == 0 or math.hypot(x - self.trail[-1][0],
                                                    y - self.trail[-1][1]) > 0.15:
                self.trail.append((x, y))

        if (not self._goal_sent
                and self._start_time is not None
                and time.time() - self._start_time >= self._goal_delay):
            self._goal_sent = True
            gx, gy = self._goal_xy
            goal = PoseStamped(
                pose=Pose(
                    position=Vector3(float(gx), float(gy), 0.0),
                    orientation=Quaternion(0, 0, 0, 1),
                ),
                frame_id="map",
                ts=time.time(),
            )
            logger.info("LiveMapper: sending goal (%.1f, %.1f)", gx, gy)
            self.goal_cmd.publish(goal)

    def _on_lidar(self, cloud: PointCloud):
        pts = np.asarray(cloud.points, dtype=np.float32)
        with self._lock:
            rx, ry = self.rx, self.ry
            self._update_grid(pts, rx, ry)
            self.lidar_pts = pts
            self.scans += 1
            count = self.scans

        if count % 3 == 0:
            with self._lock:
                grid_copy = self.grid.copy()
                rx_snap, ry_snap = self.rx, self.ry
            self.costmap_out.publish({
                "grid": grid_copy,
                "resolution": self.GR,
                "origin": [-self.GS * self.GR / 2.0, -self.GS * self.GR / 2.0],
                "rx": rx_snap,
                "ry": ry_snap,
            })

    def _update_grid(self, pts: np.ndarray, rx: float, ry: float):
        h = self.GS // 2
        for p in pts:
            d = math.hypot(float(p[0]) - rx, float(p[1]) - ry)
            if 0.3 < d < 8.0:
                gx = int(p[0] / self.GR + h)
                gy = int(p[1] / self.GR + h)
                if 0 <= gx < self.GS and 0 <= gy < self.GS:
                    self.grid[gy, gx] = min(self.grid[gy, gx] + 30, 100)

    def health(self):
        info = super().port_summary()
        info["live_mapper"] = {
            "scans": self.scans,
            "trail_len": len(self.trail),
            "goal_sent": self._goal_sent,
        }
        return info


# ---------------------------------------------------------------------------
# Maze obstacle layout
# ---------------------------------------------------------------------------

OBSTACLES = [
    # Outer walls (L-shaped corridor)
    ObstacleConfig("wall_N",  "box", [6, 0.1, 0.5], [3, 4, 0.25]),
    ObstacleConfig("wall_S",  "box", [6, 0.1, 0.5], [3, -1, 0.25]),
    ObstacleConfig("wall_W",  "box", [0.1, 2.5, 0.5], [-3, 1.5, 0.25]),
    ObstacleConfig("wall_E",  "box", [0.1, 2.5, 0.5], [9, 1.5, 0.25]),
    # Interior walls (force navigation around them)
    ObstacleConfig("inner1",  "box", [0.1, 2.0, 0.5], [3, 2.5, 0.25]),
    ObstacleConfig("inner2",  "box", [2.0, 0.1, 0.5], [5, 1, 0.25]),
    ObstacleConfig("block_a", "box", [0.4, 0.4, 0.4], [1.5, 1.5, 0.2]),
    ObstacleConfig("block_b", "box", [0.3, 0.5, 0.4], [6, 2.5, 0.2]),
    ObstacleConfig("narrow",  "box", [0.1, 1.5, 0.5], [4.5, 3, 0.25]),
]

GOAL_XY   = (7.0, 1.5)
START_POS = (0.0, 1.5, 0.30)

# ---------------------------------------------------------------------------
# Build Blueprint
# ---------------------------------------------------------------------------

logger.info("=== Go1 Maze Navigation -- LingTu Blueprint ===")

bp = Blueprint()

bp.add(Go1SimDriverModule,
       obstacles=OBSTACLES,
       goal_marker=(GOAL_XY[0], GOAL_XY[1], 0.15),
       start_pos=START_POS,
       sim_rate=50.0)

bp.add(LiveMapper, goal_xy=GOAL_XY, goal_delay=3.0)

bp.add(LocalPlannerModule, backend="simple")
bp.add(PathFollowerModule,  backend="pid", max_speed=0.4, lookahead=1.5)

bp.add(NavigationModule,
       planner="astar",
       waypoint_threshold=2.0,
       downsample_dist=1.0,
       stuck_timeout=12.0,
       max_replan_count=3)

# Exact same closed-loop wiring as go1_nav_full.py
bp.wire("Go1SimDriverModule", "odometry",    "NavigationModule",    "odometry")
bp.wire("Go1SimDriverModule", "odometry",    "LocalPlannerModule",  "odometry")
bp.wire("Go1SimDriverModule", "odometry",    "PathFollowerModule",  "odometry")
bp.wire("Go1SimDriverModule", "odometry",    "LiveMapper",          "odom_in")
bp.wire("Go1SimDriverModule", "lidar_cloud", "LiveMapper",          "lidar_in")
bp.wire("LiveMapper",         "costmap_out", "NavigationModule",    "costmap")
bp.wire("LiveMapper",         "goal_cmd",    "NavigationModule",    "goal_pose")
bp.wire("NavigationModule",   "waypoint",    "LocalPlannerModule",  "waypoint")
bp.wire("LocalPlannerModule", "local_path",  "PathFollowerModule",  "local_path")
bp.wire("PathFollowerModule", "cmd_vel",     "Go1SimDriverModule",  "cmd_vel")

system = bp.build()

driver: Go1SimDriverModule = system.get_module("Go1SimDriverModule")
mapper: LiveMapper          = system.get_module("LiveMapper")
nav: NavigationModule       = system.get_module("NavigationModule")

logger.info("Starting Module pipeline...")
system.start()
logger.info("Pipeline running. Main thread starts rendering loop.")

# ---------------------------------------------------------------------------
# Rendering (main thread only) — 2D overhead map, no EGL 3D render
# Fast enough for aarch64: pure numpy + cv2 drawing only.
# ---------------------------------------------------------------------------

import cv2

FPS     = 15
W, H    = 1280, 720
MAX_S   = 80
EXTRA_S = 3

# Wait for driver model to load (needed for qpos access)
t_wait = 0
while driver._model is None and t_wait < 10.0:
    time.sleep(0.1)
    t_wait += 0.1
if driver._model is None:
    logger.error("Driver model not loaded -- aborting render")
    system.stop()
    sys.exit(1)

model = driver._model
data  = driver._data

vid = cv2.VideoWriter(OUTPUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

# 2D map occupies full frame — robot centred, 0.1m/px resolution
MAP_GR   = 0.1          # metres per pixel (zoomed in vs LiveMapper's 0.2)
MAP_BG   = (20, 20, 20)

goal_reached = False
extra_frames = 0
frame_idx    = 0
max_frames   = MAX_S * FPS

logger.info("Recording up to %ds @ %d fps -> %s (2D map mode)", MAX_S, FPS, OUTPUT)

spf = 1.0 / FPS


def _render_2d_frame(grid, lidar_pts, trail, rx, ry, goal, nav_path, gs, gr_map, gr_frame):
    """Render full W x H 2D overhead navigation map frame using numpy ops."""
    frame = np.full((H, W, 3), MAP_BG, dtype=np.uint8)
    cx, cy = W // 2, H // 2   # robot always centred

    # --- Occupancy grid from LiveMapper (gr_map resolution) resampled to gr_frame ---
    # Map each frame pixel back to LiveMapper grid cell
    h = gs // 2
    # Build pixel coordinate arrays for the full frame
    pi = np.arange(W, dtype=np.int32)
    pj = np.arange(H, dtype=np.int32)
    # World coords of each frame pixel
    wx = (pi - cx) * gr_frame + rx   # shape (W,)
    wy = (pj - cy) * gr_frame + ry   # shape (H,)
    # LiveMapper grid indices
    gi = (wx / gr_map + h).astype(np.int32)   # shape (W,)
    gj = (wy / gr_map + h).astype(np.int32)   # shape (H,)
    # Clamp to valid range
    gi_c = np.clip(gi, 0, gs - 1)
    gj_c = np.clip(gj, 0, gs - 1)
    valid_i = (gi >= 0) & (gi < gs)
    valid_j = (gj >= 0) & (gj < gs)
    # Sample grid: grid[gj, gi] for each (j, i) combo — use outer indexing
    patch = grid[np.ix_(gj_c, gi_c)]           # shape (H, W)
    valid = np.outer(valid_j, valid_i)          # shape (H, W)
    intensity = np.clip((patch * 2.5), 0, 255).astype(np.uint8)
    mask = valid & (patch > 10)
    frame[:, :, 2] = np.where(mask, intensity, MAP_BG[2])

    # --- LiDAR points (green dots, every 3rd) ---
    if len(lidar_pts) > 0:
        pts = lidar_pts[::3]
        px = ((pts[:, 0] - rx) / gr_frame + cx).astype(np.int32)
        py = ((pts[:, 1] - ry) / gr_frame + cy).astype(np.int32)
        valid_mask = (px >= 0) & (px < W) & (py >= 0) & (py < H)
        for ppx, ppy in zip(px[valid_mask], py[valid_mask]):
            cv2.circle(frame, (int(ppx), int(ppy)), 1, (0, 200, 0), -1)

    # --- Nav path (cyan) ---
    if nav_path and len(nav_path) > 1:
        prev = None
        for wp in nav_path:
            wpx = int((float(wp[0]) - rx) / gr_frame + cx)
            wpy = int((float(wp[1]) - ry) / gr_frame + cy)
            if prev is not None:
                cv2.line(frame, prev, (wpx, wpy), (200, 200, 0), 2)
            prev = (wpx, wpy)

    # --- Trail (orange) ---
    if len(trail) > 1:
        pts_px = [(int((tx - rx) / gr_frame + cx), int((ty - ry) / gr_frame + cy))
                  for tx, ty in trail[-300:]]
        for i in range(1, len(pts_px)):
            cv2.line(frame, pts_px[i-1], pts_px[i], (20, 120, 255), 2)

    # --- Goal marker (red star) ---
    gpx = int((goal[0] - rx) / gr_frame + cx)
    gpy = int((goal[1] - ry) / gr_frame + cy)
    if 0 <= gpx < W and 0 <= gpy < H:
        cv2.drawMarker(frame, (gpx, gpy), (0, 0, 255), cv2.MARKER_STAR, 20, 3)
        cv2.putText(frame, "GOAL", (gpx + 12, gpy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # --- Robot (orange circle + white border) ---
    cv2.circle(frame, (cx, cy), 8, (255, 255, 255), -1)
    cv2.circle(frame, (cx, cy), 7, (0, 120, 255), -1)
    # Heading arrow (pointing right = +x)
    cv2.arrowedLine(frame, (cx, cy), (cx + 20, cy), (255, 255, 255), 2, tipLength=0.4)

    # --- Scale bar (bottom-left): 1m ---
    bar_px = int(1.0 / gr_frame)
    bx0, by = 20, H - 20
    cv2.line(frame, (bx0, by), (bx0 + bar_px, by), (200, 200, 200), 2)
    cv2.putText(frame, "1m", (bx0 + bar_px + 5, by + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

    return frame


last_progress = 0.0

try:
    while frame_idx < max_frames:
        t_frame_start = time.monotonic()
        frame_idx += 1
        t = frame_idx / FPS

        if data is None or model is None:
            break

        rx = float(data.qpos[0])
        ry = float(data.qpos[1])
        gdist = math.hypot(rx - GOAL_XY[0], ry - GOAL_XY[1])

        nav_state = nav._state
        wp_index  = nav._wp_index
        wp_total  = len(nav._path)

        with mapper._lock:
            grid_snap  = mapper.grid.copy()
            pts_snap   = mapper.lidar_pts.copy() if len(mapper.lidar_pts) > 0 else np.zeros((0, 3))
            trail_snap = list(mapper.trail)
            scans_snap = mapper.scans

        try:
            nav_path_snap = list(nav._path) if nav._path else []
            frame = _render_2d_frame(
                grid_snap, pts_snap, trail_snap,
                rx, ry, GOAL_XY,
                nav_path_snap, LiveMapper.GS, LiveMapper.GR, MAP_GR,
            )
        except Exception as e:
            logger.warning("Frame render error: %s", e)
            frame = np.full((H, W, 3), MAP_BG, dtype=np.uint8)

        cv2.putText(frame, "Go1 Maze Navigation - LingTu",
                    (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame,
                    "t=%.1fs pos=(%.1f,%.1f) dist=%.1fm" % (t, rx, ry, gdist),
                    (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(frame,
                    "STATE=%s wp=%d/%d map=%d scans=%d" % (
                        nav_state, wp_index, wp_total,
                        int((grid_snap > 40).sum()), scans_snap),
                    (15, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        if nav_state == MissionState.SUCCESS or gdist < 1.0:
            cv2.putText(frame, "GOAL REACHED!",
                        (W // 2 - 180, H // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
            goal_reached = True

        vid.write(frame)

        # Progress every 4 seconds
        if t - last_progress >= 4.0:
            last_progress = t
            logger.info("PROGRESS t=%2.0fs pos=(%.1f,%.1f) dist=%.1f state=%s wp=%d/%d scans=%d",
                        t, rx, ry, gdist, nav_state, wp_index, wp_total, scans_snap)
            print("PROGRESS t=%2.0fs pos=(%.1f,%.1f) dist=%.1f state=%s wp=%d/%d scans=%d" % (
                  t, rx, ry, gdist, nav_state, wp_index, wp_total, scans_snap), flush=True)

        if goal_reached:
            extra_frames += 1
            if extra_frames >= EXTRA_S * FPS:
                logger.info("Goal reached -- stopping after %.0fs extra footage.", EXTRA_S)
                break

        elapsed = time.monotonic() - t_frame_start
        wait = spf - elapsed
        if wait > 0:
            time.sleep(wait)

except Exception as e:
    logger.error("Render loop crashed: %s", e, exc_info=True)

finally:
    logger.info("Shutting down (frame_idx=%d goal_reached=%s)...", frame_idx, goal_reached)
    system.stop()
    vid.release()
    logger.info("vid.release() done")

size_mb = os.path.getsize(OUTPUT) / 1024 / 1024
logger.info("=== Video saved: %s (%.1f MB) ===", OUTPUT, size_mb)
print("\n=== Video: %s (%.1f MB) ===" % (OUTPUT, size_mb))
