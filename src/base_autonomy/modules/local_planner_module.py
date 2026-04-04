"""LocalPlannerModule — local path planning as a pluggable Module.

Takes terrain map + waypoint + odometry, produces obstacle-free local path.

Backends:
  "nanobind" — C++ LocalPlannerCore via nanobind (full CMU scoring, zero ROS2) [PREFERRED]
  "cmu"      — C++ local_planner (NativeModule subprocess, needs ROS2)
  "cmu_py"   — Pure-Python CMU scoring via numpy (slower fallback, no ROS2)
  "simple"   — straight-line path for testing

Usage::

    bp.add(LocalPlannerModule, backend="nanobind")  # C++ in-process, no ROS2
    bp.add(LocalPlannerModule, backend="cmu")        # C++ subprocess, needs ROS2
"""

from __future__ import annotations

import logging
import math
import os
import time
from pathlib import Path as FsPath
from typing import Any, Dict, List, Optional

import numpy as np

from core.module import Module
from core.stream import In, Out
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion
from core.msgs.sensor import PointCloud2
from core.registry import register
from base_autonomy.modules._nav_core_loader import try_import_nav_core, nav_core_build_hint

logger = logging.getLogger(__name__)

# Pre-computed sin/cos lookup table for 36 rotation directions (10° steps, -180..+170°)
# Matches RotLUT in local_planner_core.hpp
_ROT_SIN = np.array([math.sin((10.0 * i - 180.0) * math.pi / 180.0) for i in range(36)])
_ROT_COS = np.array([math.cos((10.0 * i - 180.0) * math.pi / 180.0) for i in range(36)])

# Default planner parameters (mirrors localPlanner.cpp defaults)
_PATH_NUM  = 343
_GROUP_NUM = 7

_OBSTACLE_HEIGHT_THRE = 0.5   # m — points above this block paths
_GROUND_HEIGHT_THRE   = 0.1   # m — points above this contribute terrain penalty
_POINT_PER_PATH_THRE  = 2     # blocked path threshold (pointPerPathThre_)
_PATH_SCALE           = 1.0   # pathScale_ default
_PATH_RANGE           = 3.5   # adjacentRange_ default — scan radius in metres
_DIR_THRE             = 90.0  # degrees — max allowed direction deviation
_GOAL_CLEAR_RANGE     = 0.5   # m


def _load_paths(paths_dir: str) -> Optional[Dict]:
    """Load pre-computed paths from PLY files.

    Returns dict with keys:
      paths          — list of Nx3 float arrays, one per path_id
      group_of_path  — int array[pathNum], group_id for each path_id
      start_paths    — list of Mx3 float arrays, one per group_id (start segment)
      correspondences — list of lists, correspondences[voxel_idx] = [path_ids...]
      grid_params    — dict with voxel grid dimensions read from correspondences
    """
    pd = FsPath(paths_dir)

    # -- paths.ply: all path points with path_id and group_id --
    paths_ply = pd / "paths.ply"
    if not paths_ply.exists():
        logger.error("LocalPlannerModule [cmu_py]: paths.ply not found at %s", paths_ply)
        return None

    paths_data: List[List] = [[] for _ in range(_PATH_NUM)]
    with open(paths_ply, "rb") as f:
        raw = f.read()
    pos = raw.find(b"end_header\r\n")
    if pos == -1:
        pos = raw.find(b"end_header\n")
        if pos == -1:
            logger.error("LocalPlannerModule [cmu_py]: bad paths.ply header")
            return None
        pos += len(b"end_header\n")
    else:
        pos += len(b"end_header\r\n")
    text = raw[pos:].decode("ascii")
    for line in text.strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
        path_id  = int(parts[3])
        if 0 <= path_id < _PATH_NUM:
            paths_data[path_id].append((x, y, z))
    paths = [np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)
             for pts in paths_data]

    # -- pathList.ply: group_id for each path_id --
    path_list_ply = pd / "pathList.ply"
    group_of_path = np.zeros(_PATH_NUM, dtype=np.int32)
    with open(path_list_ply, "rb") as f:
        raw2 = f.read()
    pos2 = raw2.find(b"end_header\r\n")
    if pos2 == -1:
        pos2 = raw2.find(b"end_header\n") + len(b"end_header\n")
    else:
        pos2 += len(b"end_header\r\n")
    text2 = raw2[pos2:].decode("ascii")
    for line in text2.strip().splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        path_id  = int(parts[3])
        group_id = int(parts[4])
        if 0 <= path_id < _PATH_NUM and 0 <= group_id < _GROUP_NUM:
            group_of_path[path_id] = group_id

    # -- startPaths.ply: one group-start segment per group_id --
    start_ply = pd / "startPaths.ply"
    start_paths_data: List[List] = [[] for _ in range(_GROUP_NUM)]
    with open(start_ply, "rb") as f:
        raw3 = f.read()
    pos3 = raw3.find(b"end_header\r\n")
    if pos3 == -1:
        pos3 = raw3.find(b"end_header\n") + len(b"end_header\n")
    else:
        pos3 += len(b"end_header\r\n")
    text3 = raw3[pos3:].decode("ascii")
    for line in text3.strip().splitlines():
        parts = line.split()
        if len(parts) < 4:
            continue
        x, y, z     = float(parts[0]), float(parts[1]), float(parts[2])
        group_id     = int(parts[3])
        if 0 <= group_id < _GROUP_NUM:
            start_paths_data[group_id].append((x, y, z))
    start_paths = [np.array(pts, dtype=np.float32) if pts else np.zeros((0, 3), dtype=np.float32)
                   for pts in start_paths_data]

    # -- correspondences.txt: voxel_id → list of blocked path_ids --
    corr_txt = pd / "correspondences.txt"
    correspondences: Dict[int, List[int]] = {}
    max_voxel_id = 0
    with open(corr_txt) as f:
        for line in f:
            tokens = line.split()
            if not tokens:
                continue
            voxel_id = int(tokens[0])
            path_ids_for_voxel = [int(t) for t in tokens[1:] if t != "-1"]
            if path_ids_for_voxel:
                correspondences[voxel_id] = path_ids_for_voxel
            max_voxel_id = max(max_voxel_id, voxel_id)

    # Derive grid dimensions from max voxel id and known defaults
    # gridVoxelNumX=161, gridVoxelNumY=531 from local_planner_core.hpp defaults
    grid_voxel_num_x = 161
    grid_voxel_num_y = 531
    # Validate against max voxel id (voxel_id = gridVoxelNumY * indX + indY)
    expected_max = grid_voxel_num_x * grid_voxel_num_y - 1
    if max_voxel_id > expected_max:
        grid_voxel_num_y = (max_voxel_id // grid_voxel_num_x) + 2
        logger.warning("LocalPlannerModule [cmu_py]: adjusted gridVoxelNumY to %d", grid_voxel_num_y)

    logger.info(
        "LocalPlannerModule [cmu_py]: loaded %d paths, %d groups, %d non-empty correspondences",
        _PATH_NUM, _GROUP_NUM, len(correspondences),
    )
    return {
        "paths":          paths,
        "group_of_path":  group_of_path,
        "start_paths":    start_paths,
        "correspondences": correspondences,
        "grid_voxel_num_x": grid_voxel_num_x,
        "grid_voxel_num_y": grid_voxel_num_y,
    }


def _score_paths_numpy(
    obstacle_pts: np.ndarray,          # Nx3 float32, body-frame (x fwd, y left)
    rel_goal_x: float,
    rel_goal_y: float,
    rel_goal_dis: float,
    joy_dir_deg: float,                 # direction to goal in body frame (degrees)
    correspondences: Dict[int, List[int]],
    group_of_path: np.ndarray,
    grid_voxel_num_x: int,
    grid_voxel_num_y: int,
) -> np.ndarray:
    """Vectorised path scoring.  Returns clearPathPerGroupScore[36 * GROUP_NUM]."""

    # Grid params (mirror VoxelGridParams defaults)
    grid_voxel_size    = 0.02
    grid_voxel_offset_x = 3.2
    grid_voxel_offset_y = 5.25
    search_radius       = 0.45
    inv_gs = 1.0 / grid_voxel_size
    offset_x_half = grid_voxel_offset_x + grid_voxel_size * 0.5
    offset_y_half = grid_voxel_offset_y + grid_voxel_size * 0.5
    scale_y_coef_a = search_radius / grid_voxel_offset_y
    scale_y_coef_b = 1.0 / grid_voxel_offset_x
    path_range_sq = _PATH_RANGE * _PATH_RANGE

    # Scoring accumulators
    clear_path_list = np.zeros(_PATH_NUM * 36, dtype=np.int32)
    path_penalty_list = np.zeros(_PATH_NUM * 36, dtype=np.float32)
    clear_per_group_score = np.zeros(36 * _GROUP_NUM, dtype=np.float64)
    clear_per_group_num   = np.zeros(36 * _GROUP_NUM, dtype=np.int32)

    # Direction validity: skip rotations too far from goal direction
    ang_diff_list = np.array([
        abs(joy_dir_deg - (10.0 * d - 180.0)) % 360.0
        for d in range(36)
    ], dtype=np.float32)
    ang_diff_list = np.where(ang_diff_list > 180.0, 360.0 - ang_diff_list, ang_diff_list)
    rot_dir_valid = ang_diff_list <= _DIR_THRE  # shape (36,)

    if obstacle_pts.shape[0] > 0:
        xs = obstacle_pts[:, 0].astype(np.float64)
        ys = obstacle_pts[:, 1].astype(np.float64)
        # Use intensity column (index 3) if available for terrain height, else use z
        if obstacle_pts.shape[1] >= 4:
            hs = obstacle_pts[:, 3].astype(np.float64)
        else:
            hs = obstacle_pts[:, 2].astype(np.float64)

        dist_sq = xs * xs + ys * ys
        in_range = dist_sq < path_range_sq

        for rot_dir in range(36):
            if not rot_dir_valid[rot_dir]:
                continue
            rc = _ROT_COS[rot_dir]
            rs = _ROT_SIN[rot_dir]

            # Rotate obstacle points into this rotation frame
            x2 = rc * xs + rs * ys   # shape (N,)
            y2 = -rs * xs + rc * ys

            scale_y = x2 * scale_y_coef_b + scale_y_coef_a * (grid_voxel_offset_x - x2) * scale_y_coef_b
            valid_scale = np.abs(scale_y) > 1e-6

            ind_x = ((offset_x_half - x2) * inv_gs).astype(np.int32)
            # Avoid divide-by-zero in y scaling
            safe_scale_y = np.where(valid_scale, scale_y, 1.0)
            ind_y = ((offset_y_half - y2 / safe_scale_y) * inv_gs).astype(np.int32)

            grid_valid = (
                valid_scale
                & in_range
                & (ind_x >= 0) & (ind_x < grid_voxel_num_x)
                & (ind_y >= 0) & (ind_y < grid_voxel_num_y)
            )

            valid_indices = np.where(grid_valid)[0]
            for vi in valid_indices:
                voxel_id = int(ind_x[vi]) * grid_voxel_num_y + int(ind_y[vi])
                if voxel_id not in correspondences:
                    continue
                h = float(hs[vi])
                for path_id in correspondences[voxel_id]:
                    idx = _PATH_NUM * rot_dir + path_id
                    if h > _OBSTACLE_HEIGHT_THRE:
                        clear_path_list[idx] += 1
                    elif h > _GROUND_HEIGHT_THRE:
                        if path_penalty_list[idx] < h:
                            path_penalty_list[idx] = h

    # Score paths into group scores
    dir_weight   = 0.02
    omni_thre    = 5.0
    slope_weight = 0.0

    for rot_dir in range(36):
        if not rot_dir_valid[rot_dir]:
            continue
        ang_diff = float(ang_diff_list[rot_dir])
        dw = abs(dir_weight * ang_diff)
        sqrt_sqrt_dw = math.sqrt(math.sqrt(dw))

        # computeRotDirW
        if rot_dir < 18:
            rot_dir_w = abs(abs(rot_dir - 9.0) + 1.0)
        else:
            rot_dir_w = abs(abs(rot_dir - 27.0) + 1.0)
        rot_dir_w2 = rot_dir_w * rot_dir_w
        rot_dir_w4 = rot_dir_w2 * rot_dir_w2

        base = _PATH_NUM * rot_dir
        for path_id in range(_PATH_NUM):
            if clear_path_list[base + path_id] >= _POINT_PER_PATH_THRE:
                continue  # blocked
            group_id  = int(group_of_path[path_id])
            # computeGroupDirW
            group_dir_w = 4.0 - abs(group_id - 3.0)

            terrain_penalty = float(path_penalty_list[base + path_id])
            terrain_factor = (max(0.0, 1.0 - slope_weight * terrain_penalty)
                              if slope_weight > 0.0 else 1.0)

            if rel_goal_dis < omni_thre:
                score = (1.0 - sqrt_sqrt_dw) * group_dir_w * group_dir_w * terrain_factor
            else:
                score = (1.0 - sqrt_sqrt_dw) * rot_dir_w4 * terrain_factor

            group_idx = _GROUP_NUM * rot_dir + group_id
            clear_per_group_score[group_idx] += score
            clear_per_group_num[group_idx]   += 1

    return clear_per_group_score


@register("local_planner", "cmu", description="CMU-style C++ local planner (NativeModule)")
class LocalPlannerModule(Module, layer=2):
    """Local path planning — obstacle avoidance + path scoring.

    "cmu"    backend: C++ local_planner NativeModule (requires ROS2)
    "cmu_py" backend: Pure-Python CMU scorer via _nav_core + numpy (no ROS2)
    "simple" backend: straight-line to waypoint (testing)
    """

    # -- Inputs --
    odometry:    In[Odometry]
    terrain_map: In[PointCloud2]
    waypoint:    In[PoseStamped]
    boundary:    In[PointCloud2]
    added_obstacles: In[PointCloud2]

    # -- Outputs --
    local_path: Out[Path]
    alive:      Out[bool]

    def __init__(self, backend: str = "cmu", **kw):
        super().__init__(**kw)
        self._backend = backend
        self._node = None
        self._robot_pos = np.zeros(3)
        self._robot_yaw = 0.0
        self._latest_waypoint: Optional[PoseStamped] = None
        self._terrain_points: Optional[np.ndarray] = None
        self._boundary_points: Optional[np.ndarray] = None
        self._added_obstacle_points: Optional[np.ndarray] = None
        self._last_cmu_py_time: float = 0.0

        # cmu_py state
        self._path_data: Optional[Dict] = None
        self._nav_core = None

        # nanobind state
        self._core = None  # _nav_core.LocalPlannerCore

    def setup(self):
        self.odometry.subscribe(self._on_odom)
        self.terrain_map.subscribe(self._on_terrain)
        self.waypoint.subscribe(self._on_waypoint)
        self.boundary.subscribe(self._on_boundary)
        self.added_obstacles.subscribe(self._on_added_obstacles)

        if self._backend == "nanobind":
            self._setup_nanobind()
        elif self._backend == "cmu":
            self._setup_cmu()
        elif self._backend == "cmu_py":
            self._setup_cmu_py()
        else:
            logger.info("LocalPlannerModule: simple backend (straight-line)")

    # ------------------------------------------------------------------ #
    # Backend setup                                                        #
    # ------------------------------------------------------------------ #

    def _setup_nanobind(self):
        """C++ LocalPlannerCore via nanobind — full CMU scoring in-process, zero ROS2."""
        _nav_core = try_import_nav_core()
        if _nav_core is None:
            logger.info(
                "LocalPlannerModule: _nav_core.so not found — using cmu_py backend.\n"
                "  To enable C++ local planner:\n  %s", nav_core_build_hint()
            )
            self._backend = "cmu_py"
            self._setup_cmu_py()
            return
        try:
            params = _nav_core.LocalPlannerParams()
            try:
                from core.config import get_config
                cfg = get_config()
                params.vehicle_length = cfg.geometry.vehicle_length
                params.vehicle_width = cfg.geometry.vehicle_width
                params.sensor_offset_x = cfg.geometry.sensor_offset_x
                params.sensor_offset_y = cfg.geometry.sensor_offset_y
                params.obstacle_height_thre = cfg.safety.obstacle_height_thre
                params.ground_height_thre = cfg.safety.ground_height_thre
                params.max_speed = cfg.speed.max_speed
                params.autonomy_speed = cfg.speed.autonomy_speed
            except (ImportError, AttributeError):
                pass

            self._core = _nav_core.LocalPlannerCore(params)

            paths_dir = os.path.join(
                os.path.dirname(__file__), "..", "local_planner", "paths")
            paths_dir = os.path.normpath(paths_dir)
            if not self._core.load_paths(paths_dir):
                logger.error("LocalPlannerModule [nanobind]: failed to load paths from %s", paths_dir)
                self._core = None
                self._backend = "cmu_py"
                self._setup_cmu_py()
                return

            logger.info("LocalPlannerModule [nanobind]: C++ LocalPlannerCore loaded (343 paths × 36 dirs)")
        except Exception as e:
            logger.warning("LocalPlannerModule: _nav_core error: %s — using cmu_py backend", e)
            self._backend = "cmu_py"
            self._setup_cmu_py()

    def _setup_cmu(self):
        try:
            from core.config import get_config
            from core.native_factories import local_planner

            cfg = get_config()
            self._node = local_planner(cfg)
            try:
                self._node.setup()
            except (FileNotFoundError, PermissionError) as e:
                logger.warning("LocalPlannerModule: setup failed: %s", e)
                self._node = None
        except ImportError as e:
            logger.warning("LocalPlannerModule: C++ backend not available: %s", e)

    def _setup_cmu_py(self):
        # Try to use _nav_core for accelerated scoring even in cmu_py mode
        _nav_core = try_import_nav_core()
        if _nav_core is not None:
            self._nav_core = _nav_core
            logger.info("LocalPlannerModule [cmu_py]: _nav_core scoring acceleration loaded")
        else:
            logger.info("LocalPlannerModule [cmu_py]: using numpy-only scorer")

        # Load pre-computed path files
        paths_dir = os.path.join(
            os.path.dirname(__file__),
            "..", "local_planner", "paths"
        )
        paths_dir = os.path.normpath(paths_dir)
        self._path_data = _load_paths(paths_dir)
        if self._path_data is None:
            logger.error(
                "LocalPlannerModule [cmu_py]: failed to load paths — "
                "falling back to simple backend"
            )
            self._backend = "simple"

    # ------------------------------------------------------------------ #
    # Module lifecycle                                                     #
    # ------------------------------------------------------------------ #

    def start(self):
        super().start()
        if self._node:
            try:
                self._node.start()
                logger.info("LocalPlannerModule [cmu]: C++ node started")
            except Exception as e:
                logger.error("LocalPlannerModule: start failed: %s", e)
        self.alive.publish(True)

    def stop(self):
        if self._node:
            try:
                self._node.stop()
            except Exception:
                pass
            self._node = None
        self._core = None
        self.alive.publish(False)
        super().stop()

    # ------------------------------------------------------------------ #
    # Input handlers                                                       #
    # ------------------------------------------------------------------ #

    def _on_odom(self, odom: Odometry):
        self._robot_pos = np.array([odom.x, odom.y, getattr(odom, "z", 0.0)])
        self._robot_yaw = getattr(odom, "yaw", 0.0)

        if self._backend == "nanobind" and self._core is not None and self._latest_waypoint is not None:
            self._run_nanobind(odom.ts if hasattr(odom, "ts") else time.time())
        elif self._backend == "cmu_py" and self._latest_waypoint is not None:
            now = time.time()
            if now - self._last_cmu_py_time >= 1.0:
                self._last_cmu_py_time = now
                self._run_cmu_py()

    def _on_terrain(self, cloud: PointCloud2):
        """Store terrain obstacle points for local planning."""
        if cloud.points is not None:
            self._terrain_points = cloud.points

    def _on_boundary(self, cloud: PointCloud2):
        """Store geofence boundary points (treated as hard obstacles)."""
        if cloud.points is not None:
            self._boundary_points = cloud.points

    def _on_added_obstacles(self, cloud: PointCloud2):
        """Store externally injected obstacle points."""
        if cloud.points is not None:
            self._added_obstacle_points = cloud.points

    def _on_waypoint(self, wp: PoseStamped):
        """Store waypoint; simple backend generates path immediately."""
        self._latest_waypoint = wp

        if self._backend == "simple" and wp is not None:
            goal = np.array([wp.pose.position.x, wp.pose.position.y, 0.0])
            path_points = self._straight_line(self._robot_pos, goal, step=0.5)
            poses = [
                PoseStamped(
                    pose=Pose(
                        position=Vector3(p[0], p[1], p[2]),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id="map",
                )
                for p in path_points
            ]
            self.local_path.publish(Path(poses=poses))

    # ------------------------------------------------------------------ #
    # nanobind C++ scorer (full CMU algorithm, in-process)                 #
    # ------------------------------------------------------------------ #

    def _merge_obstacle_clouds(self) -> np.ndarray:
        """Merge terrain + boundary + added_obstacles into single Nx4 array.

        Boundary points get intensity=100 (hard obstacle, matches C++ boundaryHandler).
        Added obstacle points get intensity=200 (matches C++ addedObstaclesHandler).
        """
        clouds = []
        if self._terrain_points is not None and self._terrain_points.shape[0] > 0:
            pts = self._terrain_points.astype(np.float32)
            if pts.shape[1] == 3:
                buf = np.zeros((len(pts), 4), dtype=np.float32)
                buf[:, :3] = pts
                clouds.append(buf)
            elif pts.shape[1] >= 4:
                clouds.append(pts[:, :4].astype(np.float32, copy=False))

        if self._boundary_points is not None and self._boundary_points.shape[0] > 0:
            bpts = self._boundary_points.astype(np.float32)
            buf = np.zeros((len(bpts), 4), dtype=np.float32)
            buf[:, :min(3, bpts.shape[1])] = bpts[:, :min(3, bpts.shape[1])]
            buf[:, 3] = 100.0
            clouds.append(buf)

        if self._added_obstacle_points is not None and self._added_obstacle_points.shape[0] > 0:
            apts = self._added_obstacle_points.astype(np.float32)
            buf = np.zeros((len(apts), 4), dtype=np.float32)
            buf[:, :min(3, apts.shape[1])] = apts[:, :min(3, apts.shape[1])]
            buf[:, 3] = 200.0
            clouds.append(buf)

        if not clouds:
            return np.zeros((0, 4), dtype=np.float32)
        return np.concatenate(clouds, axis=0)

    def _run_nanobind(self, timestamp: float):
        """Run C++ LocalPlannerCore.plan() and publish result."""
        wp = self._latest_waypoint
        if wp is None or self._core is None:
            return

        self._core.set_vehicle(
            self._robot_pos[0], self._robot_pos[1], self._robot_pos[2],
            self._robot_yaw)
        self._core.set_goal(wp.pose.position.x, wp.pose.position.y)

        merged = self._merge_obstacle_clouds()
        obs_flat = merged.ravel().tolist() if merged.shape[0] > 0 else []

        result = self._core.plan(obs_flat, timestamp)

        if result.path:
            poses = []
            cos_yaw = math.cos(self._robot_yaw)
            sin_yaw = math.sin(self._robot_yaw)
            for v in result.path:
                wx = v.x * cos_yaw - v.y * sin_yaw + self._robot_pos[0]
                wy = v.x * sin_yaw + v.y * cos_yaw + self._robot_pos[1]
                wz = v.z + self._robot_pos[2]
                poses.append(PoseStamped(
                    pose=Pose(
                        position=Vector3(wx, wy, wz),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id="map",
                ))
            self.local_path.publish(Path(poses=poses))

    # ------------------------------------------------------------------ #
    # CMU Python scorer                                                    #
    # ------------------------------------------------------------------ #

    def _run_cmu_py(self):
        """Run the CMU local planner scoring algorithm and publish best path."""
        wp = self._latest_waypoint
        if wp is None or self._path_data is None:
            return

        pd = self._path_data
        correspondences    = pd["correspondences"]
        group_of_path      = pd["group_of_path"]
        start_paths        = pd["start_paths"]
        grid_voxel_num_x   = pd["grid_voxel_num_x"]
        grid_voxel_num_y   = pd["grid_voxel_num_y"]

        # Goal in body frame
        gx = wp.pose.position.x - self._robot_pos[0]
        gy = wp.pose.position.y - self._robot_pos[1]
        cos_yaw = math.cos(self._robot_yaw)
        sin_yaw = math.sin(self._robot_yaw)
        rel_goal_x = gx * cos_yaw + gy * sin_yaw
        rel_goal_y = -gx * sin_yaw + gy * cos_yaw
        rel_goal_dis = math.sqrt(rel_goal_x * rel_goal_x + rel_goal_y * rel_goal_y)
        joy_dir_deg = math.atan2(rel_goal_y, rel_goal_x) * 180.0 / math.pi
        # Clamp to avoid reverse paths
        joy_dir_deg = max(-95.0, min(95.0, joy_dir_deg))

        # Merge all obstacle sources and convert to body frame
        merged = self._merge_obstacle_clouds()
        obs_pts: np.ndarray
        if merged.shape[0] > 0:
            dx = merged[:, 0] - self._robot_pos[0]
            dy = merged[:, 1] - self._robot_pos[1]
            bx = dx * cos_yaw + dy * sin_yaw
            by = -dx * sin_yaw + dy * cos_yaw
            bz = merged[:, 2] if merged.shape[1] >= 3 else np.zeros(len(dx), dtype=np.float32)
            intensity = merged[:, 3]
            obs_pts = np.stack([bx, by, bz, intensity], axis=1).astype(np.float32)
        else:
            obs_pts = np.zeros((0, 3), dtype=np.float32)

        # Score all paths
        group_scores = _score_paths_numpy(
            obs_pts,
            rel_goal_x, rel_goal_y, rel_goal_dis,
            joy_dir_deg,
            correspondences, group_of_path,
            grid_voxel_num_x, grid_voxel_num_y,
        )

        # Select best group (mirrors selectBestGroup in local_planner_core.hpp)
        selected_group_id = -1
        max_score = 0.0
        for i in range(36 * _GROUP_NUM):
            if group_scores[i] > max_score:
                max_score = group_scores[i]
                selected_group_id = i

        if selected_group_id < 0 or max_score <= 0.0:
            # No feasible path — publish straight-line fallback
            goal_world = np.array([wp.pose.position.x, wp.pose.position.y, 0.0])
            path_points = self._straight_line(self._robot_pos, goal_world, step=0.5)
            poses = [
                PoseStamped(
                    pose=Pose(
                        position=Vector3(float(p[0]), float(p[1]), float(p[2])),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id="map",
                )
                for p in path_points
            ]
            self.local_path.publish(Path(poses=poses))
            return

        rot_dir    = selected_group_id // _GROUP_NUM
        group_id   = selected_group_id  % _GROUP_NUM
        rc = _ROT_COS[rot_dir]
        rs = _ROT_SIN[rot_dir]

        seg = start_paths[group_id]
        if seg.shape[0] == 0:
            return

        # Rotate path points from start-path frame back to world frame
        # Body-frame path: rotate by -rot_dir angle, then translate by robot pos + yaw
        # The start_paths are defined in robot body frame at rot_dir rotation.
        # To get world coordinates: rotate back (inverse rotation = transpose) then
        # apply robot pose.
        poses = []
        for pt in seg:
            # Inverse rotation: x_body = rc*px - rs*py, y_body = rs*px + rc*py
            # (inverse of: px = rc*x_body + rs*y_body, py = -rs*x_body + rc*y_body)
            x_body = rc * float(pt[0]) - rs * float(pt[1])
            y_body = rs * float(pt[0]) + rc * float(pt[1])
            z_body = float(pt[2])
            # Body frame → world frame via robot yaw
            x_world = (x_body * cos_yaw - y_body * sin_yaw) + self._robot_pos[0]
            y_world = (x_body * sin_yaw + y_body * cos_yaw) + self._robot_pos[1]
            z_world = z_body + self._robot_pos[2]
            poses.append(
                PoseStamped(
                    pose=Pose(
                        position=Vector3(x_world, y_world, z_world),
                        orientation=Quaternion(0, 0, 0, 1),
                    ),
                    frame_id="map",
                )
            )

        self.local_path.publish(Path(poses=poses))

    # ------------------------------------------------------------------ #
    # Helpers                                                              #
    # ------------------------------------------------------------------ #

    def _straight_line(self, start: np.ndarray, goal: np.ndarray, step: float = 0.5):
        diff = goal - start
        dist = np.linalg.norm(diff)
        if dist < step:
            return [goal]
        n = max(int(dist / step), 2)
        return [start + diff * (i / n) for i in range(1, n + 1)]

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        info["local_planner"] = {
            "backend":       self._backend,
            "paths_loaded":  self._path_data is not None or (self._core is not None and self._core.paths_loaded()),
            "terrain_pts":   (self._terrain_points.shape[0]
                              if self._terrain_points is not None else 0),
            "running":       (self._core is not None) or
                             (self._node is not None
                              and getattr(self._node, "_process", None) is not None),
        }
        return info
