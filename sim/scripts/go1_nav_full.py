"""Go1 autonomous navigation — Module Blueprint pipeline.

Uses the full LingTu Module stack:
  Go1SimDriverModule → LiveMapper → NavigationModule
  → LocalPlannerModule → PathFollowerModule → Go1SimDriverModule (closed loop)

Run:
  cd ~/data/SLAM/navigation
  MUJOCO_GL=egl PYTHONPATH=src:. python3 sim/scripts/go1_nav_full.py
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

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------

os.environ.setdefault("MUJOCO_GL", "egl")

REPO = os.path.expanduser("~/data/SLAM/navigation")
sys.path.insert(0, os.path.join(REPO, "src"))
sys.path.insert(0, REPO)

OUTPUT = os.path.join(REPO, "go1_nav_full.mp4")

# ---------------------------------------------------------------------------
# Imports — Module framework
# ---------------------------------------------------------------------------

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
logger = logging.getLogger("go1_nav_full")

# ---------------------------------------------------------------------------
# LiveMapper — inline Module (layer 3)
# ---------------------------------------------------------------------------

class LiveMapper(Module, layer=3):
    """Builds a 200x200 occupancy grid from LiDAR + odometry.

    Publishes costmap every 3 scans and emits the navigation goal after 3s.

    In:  lidar_in (PointCloud), odom_in (Odometry)
    Out: costmap_out (dict), goal_cmd (PoseStamped)
    """

    lidar_in: In[PointCloud]
    odom_in: In[Odometry]

    costmap_out: Out[dict]
    goal_cmd: Out[PoseStamped]

    # Occupancy grid parameters
    GS = 200        # cells
    GR = 0.2        # metres/cell

    def __init__(self, goal_xy=(6.0, 1.5), goal_delay=3.0, **kw):
        super().__init__(**kw)
        self._goal_xy = goal_xy
        self._goal_delay = goal_delay

        # Grid + tracking state (accessed from sim + render threads)
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

    # -- Handlers -----------------------------------------------------------

    def _on_odom(self, odom: Odometry):
        x = odom.pose.position.x
        y = odom.pose.position.y
        with self._lock:
            self.rx = x
            self.ry = y
            if len(self.trail) == 0 or math.hypot(x - self.trail[-1][0],
                                                    y - self.trail[-1][1]) > 0.15:
                self.trail.append((x, y))

        # Send goal once after delay
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
# Obstacle list
# ---------------------------------------------------------------------------

OBSTACLES = [
    ObstacleConfig("wall_L", "box", [4, 0.1, 0.5], [3, 3.5, 0.25],  [0.5, 0.5, 0.6, 1]),
    ObstacleConfig("wall_R", "box", [4, 0.1, 0.5], [3, -0.5, 0.25], [0.5, 0.5, 0.6, 1]),
    ObstacleConfig("block1", "box", [0.3, 0.6, 0.4], [2.5, 1.5, 0.2], [0.7, 0.3, 0.2, 1]),
    ObstacleConfig("block2", "box", [0.2, 0.4, 0.4], [4, 2, 0.2],   [0.7, 0.3, 0.2, 1]),
]

GOAL_XY = (6.0, 1.5)

# ---------------------------------------------------------------------------
# Build Blueprint
# ---------------------------------------------------------------------------

logger.info("=== Go1 Autonomous Navigation — LingTu Blueprint ===")

bp = Blueprint()

bp.add(Go1SimDriverModule,
       obstacles=OBSTACLES,
       goal_marker=(GOAL_XY[0], GOAL_XY[1], 0.15),
       start_pos=(0, 0, 0.30),
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

# Closed-loop wiring
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

# ---------------------------------------------------------------------------
# Retrieve module references for rendering (before start so refs are stable)
# ---------------------------------------------------------------------------

driver: Go1SimDriverModule = system.get_module("Go1SimDriverModule")
mapper: LiveMapper          = system.get_module("LiveMapper")
nav: NavigationModule       = system.get_module("NavigationModule")

# ---------------------------------------------------------------------------
# Start the pipeline — sim runs in its own thread inside the driver
# ---------------------------------------------------------------------------

logger.info("Starting Module pipeline...")
system.start()
logger.info("Pipeline running. Main thread starts rendering loop.")

# ---------------------------------------------------------------------------
# Rendering (main thread only — NO mj_step here)
# ---------------------------------------------------------------------------

import cv2
import mujoco

FPS   = 15
W, H  = 1280, 720
MAX_S = 60        # maximum recording seconds
EXTRA_S = 3       # extra seconds to record after goal reached

# Wait until driver has model loaded (it loads in setup(), before start)
t_wait = 0
while driver._model is None and t_wait < 10.0:
    time.sleep(0.1)
    t_wait += 0.1
if driver._model is None:
    logger.error("Driver model not loaded — aborting render")
    system.stop()
    sys.exit(1)

# Set render resolution on the model
model = driver._model
data  = driver._data

model.vis.global_.offwidth  = W
model.vis.global_.offheight = H

renderer = mujoco.Renderer(model, H, W)
cam = mujoco.MjvCamera()
cam.type      = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [3, 1.5, 0.3]
cam.distance  = 6.0
cam.elevation = -30
cam.azimuth   = -45

vid = cv2.VideoWriter(OUTPUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

MAP_PX   = 280          # overlay size (pixels)
MAP_HALF = MAP_PX // 2
MAP_BG   = (20, 20, 20)  # dark grey background

goal_reached   = False
extra_frames   = 0
frame_idx      = 0
max_frames     = MAX_S * FPS

logger.info("Recording up to %ds @ %d fps -> %s", MAX_S, FPS, OUTPUT)

spf = 1.0 / FPS   # seconds per rendered frame


def _draw_map_overlay(frame: np.ndarray,
                      grid: np.ndarray,
                      lidar_pts: np.ndarray,
                      trail: list,
                      rx: float, ry: float,
                      goal: tuple,
                      nav_path: list,
                      gs: int, gr: float) -> np.ndarray:
    """Draw 2D map overlay on the right side of the frame."""
    # Create overlay canvas
    ov = np.full((MAP_PX, MAP_PX, 3), MAP_BG, dtype=np.uint8)

    h = gs // 2
    cx, cy = MAP_HALF, MAP_HALF  # robot is always centred

    # Pixel scale: each grid cell = 1 pixel at this zoom
    # We show a MAP_PX x MAP_PX window centred on robot
    robot_gi = int(rx / gr + h)
    robot_gj = int(ry / gr + h)
    half = MAP_HALF

    # Occupied cells — red intensity
    for gj in range(MAP_PX):
        for gi in range(MAP_PX):
            src_gi = robot_gi - half + gi
            src_gj = robot_gj - half + gj
            if 0 <= src_gi < gs and 0 <= src_gj < gs:
                val = grid[src_gj, src_gi]
                if val > 10:
                    intensity = int(min(255, val * 2.5))
                    ov[gj, gi] = (0, 0, intensity)

    # LiDAR points — green dots (subsampled every 3rd)
    if len(lidar_pts) > 0:
        for i in range(0, len(lidar_pts), 3):
            px = int((lidar_pts[i, 0] - rx) / gr) + cx
            py = int((lidar_pts[i, 1] - ry) / gr) + cy
            if 0 <= px < MAP_PX and 0 <= py < MAP_PX:
                cv2.circle(ov, (px, py), 1, (0, 200, 0), -1)

    # Global path from NavigationModule — cyan line
    if nav_path and len(nav_path) > 1:
        prev = None
        for wp in nav_path:
            wpx = int((float(wp[0]) - rx) / gr) + cx
            wpy = int((float(wp[1]) - ry) / gr) + cy
            if prev is not None:
                cv2.line(ov, prev, (wpx, wpy), (200, 200, 0), 1)
            prev = (wpx, wpy)

    # Robot trail — blue/orange line
    if len(trail) > 1:
        pts_px = []
        for tx, ty in trail[-200:]:  # last 200 trail points
            tpx = int((tx - rx) / gr) + cx
            tpy = int((ty - ry) / gr) + cy
            pts_px.append((tpx, tpy))
        for i in range(1, len(pts_px)):
            cv2.line(ov, pts_px[i-1], pts_px[i], (180, 100, 0), 1)

    # Goal — red star (cross approximation)
    gpx = int((goal[0] - rx) / gr) + cx
    gpy = int((goal[1] - ry) / gr) + cy
    if 0 <= gpx < MAP_PX and 0 <= gpy < MAP_PX:
        cv2.drawMarker(ov, (gpx, gpy), (0, 0, 220),
                       cv2.MARKER_STAR, 12, 2)

    # Robot — orange filled circle with white border
    cv2.circle(ov, (cx, cy), 5, (255, 255, 255), -1)
    cv2.circle(ov, (cx, cy), 4, (0, 140, 255), -1)

    # Scale bar at bottom: 1m = 1/gr pixels
    bar_px = int(1.0 / gr)
    bx0 = 10
    by  = MAP_PX - 8
    cv2.line(ov, (bx0, by), (bx0 + bar_px, by), (200, 200, 200), 2)
    cv2.putText(ov, "1m", (bx0 + bar_px + 3, by + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)

    # Semi-transparent border
    cv2.rectangle(ov, (0, 0), (MAP_PX - 1, MAP_PX - 1), (80, 80, 80), 1)

    # Paste overlay into frame at top-right
    ox = W - MAP_PX - 10
    oy = 10
    frame[oy:oy + MAP_PX, ox:ox + MAP_PX] = ov
    return frame


t_render_start = time.time()

while frame_idx < max_frames:
    t_frame_start = time.monotonic()
    frame_idx += 1
    t = frame_idx / FPS

    # Read robot state (sim runs in its own thread — just read snapshots)
    if data is None or model is None:
        break

    rx = float(data.qpos[0]) if data is not None else 0.0
    ry = float(data.qpos[1]) if data is not None else 0.0
    gdist = math.hypot(rx - GOAL_XY[0], ry - GOAL_XY[1])

    # Determine display state
    nav_state = nav._state
    wp_index  = nav._wp_index
    wp_total  = len(nav._path)

    with mapper._lock:
        grid_snap  = mapper.grid.copy()
        pts_snap   = mapper.lidar_pts.copy() if len(mapper.lidar_pts) > 0 else np.zeros((0, 3))
        trail_snap = list(mapper.trail)
        scans_snap = mapper.scans
        rx_snap    = mapper.rx
        ry_snap    = mapper.ry

    # Render MuJoCo scene
    cam.lookat[:] = [rx, ry, 0.3]
    try:
        mujoco.mj_forward(model, data)
        renderer.update_scene(data, cam)
        rgb = renderer.render()
        frame = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    except Exception as e:
        logger.warning("Render error: %s", e)
        frame = np.zeros((H, W, 3), dtype=np.uint8)

    # 2D map overlay
    nav_path_snap = list(nav._path) if nav._path else []
    frame = _draw_map_overlay(
        frame, grid_snap, pts_snap, trail_snap,
        rx_snap, ry_snap, GOAL_XY,
        nav_path_snap, LiveMapper.GS, LiveMapper.GR,
    )

    # Text overlay — top-left
    cv2.putText(frame, "Go1 Autonomous Navigation - LingTu",
                (15, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    cv2.putText(frame,
                "t=%.1fs pos=(%.1f,%.1f) dist=%.1fm" % (t, rx, ry, gdist),
                (15, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
    cv2.putText(frame,
                "STATE=%s wp=%d/%d map=%d scans=%d" % (
                    nav_state, wp_index, wp_total,
                    int((grid_snap > 40).sum()), scans_snap),
                (15, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

    # Goal reached banner
    if nav_state == MissionState.SUCCESS or gdist < 1.0:
        cv2.putText(frame, "GOAL REACHED!",
                    (W // 2 - 180, H // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)
        goal_reached = True

    vid.write(frame)

    # Progress log every 5s
    if frame_idx % (FPS * 5) == 0:
        logger.info("t=%2.0fs pos=(%.1f,%.1f) dist=%.1f state=%s wp=%d/%d scans=%d",
                    t, rx, ry, gdist, nav_state, wp_index, wp_total, scans_snap)

    # After goal reached: record extra seconds then stop
    if goal_reached:
        extra_frames += 1
        if extra_frames >= EXTRA_S * FPS:
            logger.info("Goal reached — stopping after %.0fs extra footage.", EXTRA_S)
            break

    # Throttle to ~FPS rate (render loop, no mj_step)
    elapsed = time.monotonic() - t_frame_start
    wait = spf - elapsed
    if wait > 0:
        time.sleep(wait)

# ---------------------------------------------------------------------------
# Shutdown
# ---------------------------------------------------------------------------

system.stop()
renderer.close()
vid.release()

size_mb = os.path.getsize(OUTPUT) / 1024 / 1024
logger.info("=== Video saved: %s (%.1f MB) ===", OUTPUT, size_mb)
print("\n=== Video: %s (%.1f MB) ===" % (OUTPUT, size_mb))
