#!/usr/bin/env python3
"""ROS 2 smoke test for Gazebo-backed frontier exploration.

The script starts LingTu's in-process WavefrontFrontierExplorer, feeds it a
simulation-only coverage grid updated from Gazebo odometry, forwards produced
frontier goals to /nav/goal_pose, and verifies the ROS navigation stack reacts:

    WavefrontFrontierExplorer -> /nav/goal_pose -> /nav/global_path
    -> /nav/local_path -> /nav/cmd_vel -> Gazebo odometry movement

It never publishes cmd_vel and is intended for server-side Gazebo gates only.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
for item in (ROOT, SRC):
    if str(item) not in sys.path:
        sys.path.insert(0, str(item))


@dataclass
class GazeboFrontierExplorationResult:
    ok: bool = False
    simulation_only: bool = True
    real_robot_motion: bool = False
    cmd_vel_sent_to_hardware: bool = False
    frontier_started: bool = False
    frontier_goal_seen: bool = False
    frontier_goal_published: bool = False
    odometry_seen: bool = False
    map_cloud_seen: bool = False
    cumulative_map_cloud_seen: bool = False
    global_path_seen: bool = False
    local_path_seen: bool = False
    cmd_vel_seen: bool = False
    cmd_vel_nonzero: bool = False
    odom_start_xy: tuple[float, float] | None = None
    odom_last_xy: tuple[float, float] | None = None
    odom_delta_m: float = 0.0
    known_cells_initial: int = 0
    known_cells_final: int = 0
    known_cells_delta: int = 0
    explored_area_initial_m2: float = 0.0
    explored_area_final_m2: float = 0.0
    explored_area_delta_m2: float = 0.0
    frontier_goal: tuple[float, float, float] | None = None
    frontier_count_max: int = 0
    cumulative_map_cloud: dict = field(default_factory=dict)
    registered_cloud: dict = field(default_factory=dict)
    static_obstacles: dict = field(default_factory=dict)
    samples: dict[str, int] = field(default_factory=dict)
    errors: list[str] = field(default_factory=list)

    def as_dict(self) -> dict:
        return {
            "schema_version": "lingtu.gazebo_frontier_exploration.v1",
            "ok": self.ok,
            "simulation_only": self.simulation_only,
            "real_robot_motion": self.real_robot_motion,
            "cmd_vel_sent_to_hardware": self.cmd_vel_sent_to_hardware,
            "frontier_started": self.frontier_started,
            "frontier_goal_seen": self.frontier_goal_seen,
            "frontier_goal_published": self.frontier_goal_published,
            "odometry_seen": self.odometry_seen,
            "map_cloud_seen": self.map_cloud_seen,
            "cumulative_map_cloud_seen": self.cumulative_map_cloud_seen,
            "global_path_seen": self.global_path_seen,
            "local_path_seen": self.local_path_seen,
            "cmd_vel_seen": self.cmd_vel_seen,
            "cmd_vel_nonzero": self.cmd_vel_nonzero,
            "odom_start_xy": self.odom_start_xy,
            "odom_last_xy": self.odom_last_xy,
            "odom_delta_m": self.odom_delta_m,
            "known_cells_initial": self.known_cells_initial,
            "known_cells_final": self.known_cells_final,
            "known_cells_delta": self.known_cells_delta,
            "explored_area_initial_m2": self.explored_area_initial_m2,
            "explored_area_final_m2": self.explored_area_final_m2,
            "explored_area_delta_m2": self.explored_area_delta_m2,
            "frontier_goal": self.frontier_goal,
            "frontier_count_max": self.frontier_count_max,
            "cumulative_map_cloud": self.cumulative_map_cloud,
            "registered_cloud": self.registered_cloud,
            "static_obstacles": self.static_obstacles,
            "samples": self.samples,
            "errors": self.errors,
        }


def _count(samples: dict[str, int], topic: str) -> None:
    samples[topic] = samples.get(topic, 0) + 1


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


STATIC_OBSTACLE_ROIS = {
    "block": ((5.0, 0.0, 0.55), (0.9, 0.9, 1.2)),
    "column": ((1.6, -0.35, 0.6), (0.7, 0.7, 1.4)),
    "wall_left": ((3.0, 1.4, 0.45), (2.2, 0.45, 1.2)),
    "wall_right": ((3.0, -1.4, 0.45), (2.2, 0.45, 1.2)),
}


class CoverageGrid:
    def __init__(self, *, size_m: float, resolution_m: float, reveal_radius_m: float) -> None:
        self.resolution = float(resolution_m)
        self.reveal_radius = float(reveal_radius_m)
        self.width = int(size_m / self.resolution)
        self.height = self.width
        self.origin_x = -size_m / 2.0
        self.origin_y = -size_m / 2.0
        self.grid: list[list[int]] | None = None
        import numpy as np

        self.grid = np.full((self.height, self.width), -1, dtype=np.int16)
        self._seed_forward_corridor()

    def _cell(self, x: float, y: float) -> tuple[int, int] | None:
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        if 0 <= row < self.height and 0 <= col < self.width:
            return row, col
        return None

    def _seed_forward_corridor(self) -> None:
        assert self.grid is not None
        steps = int(2.1 / self.resolution)
        lateral = int(0.45 / self.resolution)
        for i in range(steps + 1):
            x = i * self.resolution
            for j in range(-lateral, lateral + 1):
                y = j * self.resolution
                cell = self._cell(x, y)
                if cell is not None:
                    self.grid[cell] = 0

    def reveal(self, x: float, y: float) -> int:
        assert self.grid is not None
        center = self._cell(x, y)
        if center is None:
            return 0
        row0, col0 = center
        radius = max(1, int(self.reveal_radius / self.resolution))
        changed = 0
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if math.hypot(dr, dc) * self.resolution > self.reveal_radius:
                    continue
                row = row0 + dr
                col = col0 + dc
                if 0 <= row < self.height and 0 <= col < self.width and self.grid[row, col] < 0:
                    self.grid[row, col] = 0
                    changed += 1
        return changed

    def known_cells(self) -> int:
        assert self.grid is not None
        import numpy as np

        return int(np.count_nonzero(np.asarray(self.grid) >= 0))

    def known_points(self, *, max_points: int = 1200) -> list[tuple[float, float]]:
        assert self.grid is not None
        import numpy as np

        rows, cols = np.where(np.asarray(self.grid) >= 0)
        if len(rows) == 0:
            return []
        step = max(1, int(math.ceil(len(rows) / max_points)))
        return [
            (
                float(self.origin_x + (int(col) + 0.5) * self.resolution),
                float(self.origin_y + (int(row) + 0.5) * self.resolution),
            )
            for row, col in zip(rows[::step], cols[::step])
        ]

    def as_costmap(self) -> dict:
        assert self.grid is not None
        return {
            "grid": self.grid.copy(),
            "resolution": self.resolution,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "width": self.width,
            "height": self.height,
            "frame_id": "map",
        }


def run_smoke(args: argparse.Namespace) -> GazeboFrontierExplorationResult:
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from nav_msgs.msg import Odometry, Path as ROSPath
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2

    from core.msgs.geometry import Pose as CorePose
    from core.msgs.nav import Odometry as CoreOdometry
    from nav.frontier_explorer_module import WavefrontFrontierExplorer

    rclpy.init(args=None)
    node = Node("lingtu_gazebo_frontier_exploration_smoke")
    result = GazeboFrontierExplorationResult()
    lock = threading.Lock()
    coverage = CoverageGrid(
        size_m=args.coverage_size_m,
        resolution_m=args.coverage_resolution_m,
        reveal_radius_m=args.reveal_radius_m,
    )
    goal_pub = node.create_publisher(PoseStamped, "/nav/goal_pose", 10)

    explorer = WavefrontFrontierExplorer(
        min_frontier_size=args.min_frontier_size,
        safe_distance=0.0,
        lookahead_distance=6.0,
        max_explored_distance=8.0,
        info_gain_threshold=0.0,
        goal_timeout=args.frontier_goal_timeout_sec,
        explore_rate=args.frontier_rate_hz,
        costmap_max_age=10.0,
    )
    current_pose: tuple[float, float, float] | None = None
    current_goal: tuple[float, float, float] | None = None
    current_frontiers: list[dict[str, float]] = []
    current_global_path: list[tuple[float, float]] = []
    current_local_path: list[tuple[float, float]] = []
    current_cmd_vel: tuple[float, float, float] = (0.0, 0.0, 0.0)
    trace: list[dict] = []
    run_started_at = time.monotonic()
    cumulative_point_counts: list[int] = []
    cumulative_unique_counts: list[int] = []
    cumulative_frames: set[str] = set()
    cumulative_retention_ratios: list[float] = []
    cumulative_prev_voxels: set[tuple[int, int, int]] | None = None
    cumulative_growth_steps = 0
    cumulative_step_count = 0
    registered_unique_counts: list[int] = []
    static_centroids: dict[str, list[tuple[float, float, float]]] = {
        name: [] for name in STATIC_OBSTACLE_ROIS
    }

    def read_xyz_points(msg: PointCloud2, *, max_points: int) -> list[tuple[float, float, float]]:
        fields = {field.name for field in msg.fields}
        if not {"x", "y", "z"} <= fields:
            return []
        points: list[tuple[float, float, float]] = []
        for raw in point_cloud2.read_points(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True,
        ):
            x, y, z = float(raw[0]), float(raw[1]), float(raw[2])
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                points.append((x, y, z))
            if len(points) >= max_points:
                break
        return points

    def voxelize(points: list[tuple[float, float, float]]) -> set[tuple[int, int, int]]:
        size = args.cumulative_voxel_size_m
        return {
            (
                math.floor(x / size),
                math.floor(y / size),
                math.floor(z / size),
            )
            for x, y, z in points
        }

    def roi_centroids(points: list[tuple[float, float, float]]) -> dict[str, tuple[float, float, float]]:
        out: dict[str, tuple[float, float, float]] = {}
        for name, (center, span) in STATIC_OBSTACLE_ROIS.items():
            cx, cy, cz = center
            sx, sy, sz = span
            selected = [
                (x, y, z)
                for x, y, z in points
                if abs(x - cx) <= sx / 2.0
                and abs(y - cy) <= sy / 2.0
                and abs(z - cz) <= sz / 2.0
            ]
            if len(selected) < args.min_static_roi_points:
                continue
            n = float(len(selected))
            out[name] = (
                sum(x for x, _, _ in selected) / n,
                sum(y for _, y, _ in selected) / n,
                sum(z for _, _, z in selected) / n,
            )
        return out

    def max_centroid_drift(centroids: list[tuple[float, float, float]]) -> float:
        if len(centroids) < 2:
            return 0.0
        first = centroids[0]
        return max(
            math.sqrt(
                (item[0] - first[0]) ** 2
                + (item[1] - first[1]) ** 2
                + (item[2] - first[2]) ** 2
            )
            for item in centroids[1:]
        )

    def update_cumulative_report() -> None:
        nonlocal cumulative_growth_steps, cumulative_step_count
        initial_points = cumulative_point_counts[0] if cumulative_point_counts else 0
        final_points = cumulative_point_counts[-1] if cumulative_point_counts else 0
        initial_voxels = cumulative_unique_counts[0] if cumulative_unique_counts else 0
        final_voxels = cumulative_unique_counts[-1] if cumulative_unique_counts else 0
        point_growth_ratio = (
            float(final_points) / float(initial_points)
            if initial_points > 0
            else 0.0
        )
        voxel_growth_ratio = (
            float(final_voxels) / float(initial_voxels)
            if initial_voxels > 0
            else 0.0
        )
        growth_step_ratio = (
            float(cumulative_growth_steps) / float(cumulative_step_count)
            if cumulative_step_count > 0
            else 0.0
        )
        retention_min = min(cumulative_retention_ratios) if cumulative_retention_ratios else 0.0
        static_report = {
            name: {
                "samples": len(values),
                "centroid_drift_max_m": round(max_centroid_drift(values), 4),
            }
            for name, values in static_centroids.items()
            if values
        }
        result.cumulative_map_cloud = {
            "samples": len(cumulative_point_counts),
            "frame_ids": sorted(cumulative_frames),
            "point_count_initial": initial_points,
            "point_count_final": final_points,
            "point_count_delta": final_points - initial_points,
            "point_growth_ratio": round(point_growth_ratio, 4),
            "unique_voxels_initial": initial_voxels,
            "unique_voxels_final": final_voxels,
            "unique_voxels_delta": final_voxels - initial_voxels,
            "unique_voxel_growth_ratio": round(voxel_growth_ratio, 4),
            "growth_step_ratio": round(growth_step_ratio, 4),
            "retention_min": round(retention_min, 4),
        }
        result.static_obstacles = static_report
        median_registered = 0
        if registered_unique_counts:
            ordered = sorted(registered_unique_counts)
            median_registered = int(ordered[len(ordered) // 2])
        result.registered_cloud = {
            "samples": len(registered_unique_counts),
            "median_unique_voxels": median_registered,
            "map_vs_registered_voxel_ratio": round(
                float(final_voxels) / float(median_registered),
                4,
            )
            if median_registered > 0
            else 0.0,
        }

    def cumulative_gate_ready() -> bool:
        update_cumulative_report()
        stats = result.cumulative_map_cloud
        registered = result.registered_cloud
        static = result.static_obstacles
        if int(stats.get("samples") or 0) < args.min_cumulative_samples:
            return False
        point_delta = int(stats.get("point_count_delta") or 0)
        point_ratio = float(stats.get("point_growth_ratio") or 0.0)
        voxel_delta = int(stats.get("unique_voxels_delta") or 0)
        voxel_ratio = float(stats.get("unique_voxel_growth_ratio") or 0.0)
        if point_delta < args.min_cumulative_point_delta and point_ratio < args.min_cumulative_point_growth_ratio:
            return False
        if voxel_delta < args.min_cumulative_voxel_delta and voxel_ratio < args.min_cumulative_voxel_growth_ratio:
            return False
        if float(stats.get("growth_step_ratio") or 0.0) < args.min_cumulative_growth_step_ratio:
            return False
        if float(stats.get("retention_min") or 0.0) < args.min_cumulative_retention_ratio:
            return False
        if (
            float(registered.get("map_vs_registered_voxel_ratio") or 0.0)
            < args.min_map_vs_registered_voxel_ratio
        ):
            return False
        return any(
            int(item.get("samples") or 0) >= 2
            and float(item.get("centroid_drift_max_m") or 0.0) <= args.max_static_centroid_drift_m
            for item in static.values()
        )

    def path_points(msg: ROSPath, *, limit: int = 160) -> list[tuple[float, float]]:
        poses = list(msg.poses or [])
        if not poses:
            return []
        step = max(1, int(math.ceil(len(poses) / limit)))
        return [
            (float(p.pose.position.x), float(p.pose.position.y))
            for p in poses[::step]
        ]

    def capture_trace_snapshot() -> None:
        if not args.trace_out or current_pose is None:
            return
        known = coverage.known_cells()
        trace.append(
            {
                "t": round(time.monotonic() - run_started_at, 3),
                "pose": [round(float(v), 4) for v in current_pose],
                "goal": [round(float(v), 4) for v in current_goal] if current_goal else None,
                "known_cells": int(known),
                "explored_area_m2": round(known * coverage.resolution * coverage.resolution, 4),
                "frontiers": current_frontiers[:40],
                "global_path": current_global_path,
                "local_path": current_local_path,
                "cmd_vel": [round(float(v), 4) for v in current_cmd_vel],
                "known_points": coverage.known_points(max_points=args.trace_max_known_points),
            }
        )

    def publish_frontier_goal(goal) -> None:
        nonlocal current_goal
        ros_goal = PoseStamped()
        ros_goal.header.frame_id = getattr(goal, "frame_id", "map") or "map"
        ros_goal.header.stamp = node.get_clock().now().to_msg()
        ros_goal.pose.position.x = float(goal.pose.x)
        ros_goal.pose.position.y = float(goal.pose.y)
        ros_goal.pose.position.z = float(goal.pose.z)
        ros_goal.pose.orientation.w = 1.0
        with lock:
            current_goal = (
                float(ros_goal.pose.position.x),
                float(ros_goal.pose.position.y),
                float(ros_goal.pose.position.z),
            )
            result.frontier_goal = current_goal
            result.frontier_goal_seen = True
            result.frontier_goal_published = True
            _count(result.samples, "/nav/frontier_goal")
            _count(result.samples, "/nav/goal_pose")
        goal_pub.publish(ros_goal)

    def on_frontiers(frontiers: list) -> None:
        nonlocal current_frontiers
        current_frontiers = [
            {
                "cx": round(float(frontier.get("cx", 0.0)), 4),
                "cy": round(float(frontier.get("cy", 0.0)), 4),
                "size": int(frontier.get("size", 0)),
                "score": round(float(frontier.get("score", 0.0)), 4),
            }
            for frontier in list(frontiers or [])[:40]
            if isinstance(frontier, dict)
        ]
        with lock:
            _count(result.samples, "frontiers")
            result.frontier_count_max = max(result.frontier_count_max, len(frontiers or []))

    explorer.exploration_goal.subscribe(publish_frontier_goal)
    explorer.frontiers.subscribe(on_frontiers)
    explorer.setup()
    explorer.start()

    def on_odom(msg: Odometry) -> None:
        nonlocal current_pose
        _count(result.samples, "/nav/odometry")
        pose = msg.pose.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = _yaw_from_quat(
            float(pose.orientation.x),
            float(pose.orientation.y),
            float(pose.orientation.z),
            float(pose.orientation.w),
        )
        coverage.reveal(x, y)
        current_pose = (x, y, yaw)
        with lock:
            if not result.odometry_seen:
                result.odom_start_xy = (x, y)
                initial = coverage.known_cells()
                result.known_cells_initial = initial
                result.explored_area_initial_m2 = round(initial * coverage.resolution * coverage.resolution, 4)
            result.odometry_seen = True
            result.odom_last_xy = (x, y)
            if result.odom_start_xy is not None:
                result.odom_delta_m = math.hypot(
                    x - result.odom_start_xy[0],
                    y - result.odom_start_xy[1],
                )
        explorer.odometry._deliver(CoreOdometry(pose=CorePose(x, y, 0.0, yaw=yaw), frame_id="map"))
        explorer.costmap._deliver(coverage.as_costmap())

        goal = current_goal
        if goal is not None and math.hypot(x - goal[0], y - goal[1]) <= args.goal_reached_radius_m:
            explorer.goal_reached._deliver(True)

    def on_global_path(msg: ROSPath) -> None:
        nonlocal current_global_path
        current_global_path = path_points(msg)
        _count(result.samples, "/nav/global_path")
        result.global_path_seen = result.global_path_seen or bool(msg.poses)

    def on_local_path(msg: ROSPath) -> None:
        nonlocal current_local_path
        current_local_path = path_points(msg)
        _count(result.samples, "/nav/local_path")
        result.local_path_seen = result.local_path_seen or bool(msg.poses)

    def on_cmd_vel(msg: TwistStamped) -> None:
        nonlocal current_cmd_vel
        _count(result.samples, "/nav/cmd_vel")
        result.cmd_vel_seen = True
        tw = msg.twist
        current_cmd_vel = (
            float(tw.linear.x),
            float(tw.linear.y),
            float(tw.angular.z),
        )
        result.cmd_vel_nonzero = result.cmd_vel_nonzero or (
            abs(float(tw.linear.x)) > args.min_cmd_vel
            or abs(float(tw.linear.y)) > args.min_cmd_vel
            or abs(float(tw.angular.z)) > args.min_cmd_vel
        )

    def on_map_cloud(_msg: PointCloud2) -> None:
        _count(result.samples, "/nav/map_cloud")
        result.map_cloud_seen = True

    def on_registered_cloud(msg: PointCloud2) -> None:
        _count(result.samples, "/nav/registered_cloud")
        points = read_xyz_points(msg, max_points=args.max_cloud_points)
        if points:
            registered_unique_counts.append(len(voxelize(points)))
            update_cumulative_report()

    def on_cumulative_map_cloud(msg: PointCloud2) -> None:
        nonlocal cumulative_prev_voxels, cumulative_growth_steps, cumulative_step_count
        _count(result.samples, "/nav/cumulative_map_cloud")
        result.cumulative_map_cloud_seen = True
        cumulative_frames.add(str(msg.header.frame_id))
        cumulative_point_counts.append(int(msg.width) * int(msg.height))
        points = read_xyz_points(msg, max_points=args.max_cloud_points)
        voxels = voxelize(points)
        cumulative_unique_counts.append(len(voxels))
        if cumulative_prev_voxels is not None:
            cumulative_step_count += 1
            if len(voxels) >= len(cumulative_prev_voxels):
                cumulative_growth_steps += 1
            if cumulative_prev_voxels:
                cumulative_retention_ratios.append(
                    len(cumulative_prev_voxels.intersection(voxels))
                    / float(len(cumulative_prev_voxels))
                )
        cumulative_prev_voxels = voxels
        for name, centroid in roi_centroids(points).items():
            static_centroids[name].append(centroid)
        update_cumulative_report()

    node.create_subscription(Odometry, "/nav/odometry", on_odom, 10)
    node.create_subscription(ROSPath, "/nav/global_path", on_global_path, 10)
    node.create_subscription(ROSPath, "/nav/local_path", on_local_path, 10)
    node.create_subscription(TwistStamped, "/nav/cmd_vel", on_cmd_vel, 10)
    node.create_subscription(PointCloud2, "/nav/map_cloud", on_map_cloud, 10)
    node.create_subscription(PointCloud2, "/nav/registered_cloud", on_registered_cloud, 10)
    node.create_subscription(
        PointCloud2,
        "/nav/cumulative_map_cloud",
        on_cumulative_map_cloud,
        10,
    )

    deadline = time.monotonic() + args.timeout_sec
    started = False
    next_costmap_tick = time.monotonic()
    next_trace_tick = time.monotonic()
    pass_seen_at: float | None = None
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if current_pose is not None and not started:
                status = explorer.begin_exploration()
                result.frontier_started = status in {"started", "already_running"}
                started = result.frontier_started
            if current_pose is not None and now >= next_costmap_tick:
                x, y, yaw = current_pose
                explorer.odometry._deliver(CoreOdometry(pose=CorePose(x, y, 0.0, yaw=yaw), frame_id="map"))
                explorer.costmap._deliver(coverage.as_costmap())
                next_costmap_tick = now + args.coverage_update_sec
            known = coverage.known_cells()
            result.known_cells_final = known
            result.known_cells_delta = known - result.known_cells_initial
            area = round(known * coverage.resolution * coverage.resolution, 4)
            result.explored_area_final_m2 = area
            result.explored_area_delta_m2 = round(area - result.explored_area_initial_m2, 4)
            if now >= next_trace_tick:
                capture_trace_snapshot()
                next_trace_tick = now + args.trace_interval_sec
            if (
                result.frontier_goal_published
                and result.global_path_seen
                and result.local_path_seen
                and result.cmd_vel_nonzero
                and result.odom_delta_m >= args.min_odom_delta_m
                and result.explored_area_delta_m2 >= args.min_explored_area_delta_m2
                and (not args.require_cumulative_map or cumulative_gate_ready())
            ):
                if args.continue_after_pass_sec <= 0:
                    break
                if pass_seen_at is None:
                    pass_seen_at = now
                elif now - pass_seen_at >= args.continue_after_pass_sec:
                    break
    finally:
        explorer.end_exploration()
        explorer.stop()
        node.destroy_node()
        rclpy.shutdown()

    if not result.frontier_started:
        result.errors.append("WavefrontFrontierExplorer did not start")
    if not result.frontier_goal_seen:
        result.errors.append("frontier goal was not produced")
    if not result.frontier_goal_published:
        result.errors.append("/nav/goal_pose was not published from frontier")
    if not result.odometry_seen:
        result.errors.append("/nav/odometry was not observed")
    if not result.map_cloud_seen:
        result.errors.append("/nav/map_cloud was not observed")
    update_cumulative_report()
    if args.require_cumulative_map:
        stats = result.cumulative_map_cloud
        registered = result.registered_cloud
        static = result.static_obstacles
        if not result.cumulative_map_cloud_seen:
            result.errors.append("/nav/cumulative_map_cloud was not observed")
        if int(stats.get("samples") or 0) < args.min_cumulative_samples:
            result.errors.append(
                "/nav/cumulative_map_cloud samples "
                f"{int(stats.get('samples') or 0)} < {args.min_cumulative_samples}"
            )
        if "odom" not in set(stats.get("frame_ids") or []):
            result.errors.append("/nav/cumulative_map_cloud frame_id did not include odom")
        point_delta = int(stats.get("point_count_delta") or 0)
        point_ratio = float(stats.get("point_growth_ratio") or 0.0)
        if (
            point_delta < args.min_cumulative_point_delta
            and point_ratio < args.min_cumulative_point_growth_ratio
        ):
            result.errors.append(
                "cumulative map point growth too small: "
                f"delta={point_delta}, ratio={point_ratio:.3f}"
            )
        voxel_delta = int(stats.get("unique_voxels_delta") or 0)
        voxel_ratio = float(stats.get("unique_voxel_growth_ratio") or 0.0)
        if (
            voxel_delta < args.min_cumulative_voxel_delta
            and voxel_ratio < args.min_cumulative_voxel_growth_ratio
        ):
            result.errors.append(
                "cumulative map voxel growth too small: "
                f"delta={voxel_delta}, ratio={voxel_ratio:.3f}"
            )
        if float(stats.get("growth_step_ratio") or 0.0) < args.min_cumulative_growth_step_ratio:
            result.errors.append(
                "cumulative map growth step ratio "
                f"{float(stats.get('growth_step_ratio') or 0.0):.3f} "
                f"< {args.min_cumulative_growth_step_ratio:.3f}"
            )
        if float(stats.get("retention_min") or 0.0) < args.min_cumulative_retention_ratio:
            result.errors.append(
                "cumulative map retention_min "
                f"{float(stats.get('retention_min') or 0.0):.3f} "
                f"< {args.min_cumulative_retention_ratio:.3f}"
            )
        if int(registered.get("samples") or 0) <= 0:
            result.errors.append("/nav/registered_cloud was not observed for negative control")
        elif (
            float(registered.get("map_vs_registered_voxel_ratio") or 0.0)
            < args.min_map_vs_registered_voxel_ratio
        ):
            result.errors.append(
                "cumulative map did not exceed registered single-frame cloud: "
                f"ratio={float(registered.get('map_vs_registered_voxel_ratio') or 0.0):.3f}"
            )
        stable_static = [
            name
            for name, item in static.items()
            if int(item.get("samples") or 0) >= 2
            and float(item.get("centroid_drift_max_m") or 0.0)
            <= args.max_static_centroid_drift_m
        ]
        if not stable_static:
            result.errors.append("no static obstacle ROI had stable cumulative-map centroids")
    if not result.global_path_seen:
        result.errors.append("/nav/global_path with poses was not observed")
    if not result.local_path_seen:
        result.errors.append("/nav/local_path with poses was not observed")
    if not result.cmd_vel_seen:
        result.errors.append("/nav/cmd_vel was not observed")
    elif not result.cmd_vel_nonzero:
        result.errors.append("/nav/cmd_vel never became non-zero")
    if result.odom_delta_m < args.min_odom_delta_m:
        result.errors.append(
            f"/nav/odometry moved {result.odom_delta_m:.3f} m, "
            f"expected >= {args.min_odom_delta_m:.3f} m"
        )
    if result.explored_area_delta_m2 < args.min_explored_area_delta_m2:
        result.errors.append(
            f"explored area grew {result.explored_area_delta_m2:.3f} m^2, "
            f"expected >= {args.min_explored_area_delta_m2:.3f} m^2"
        )
    if result.frontier_count_max <= 0:
        result.errors.append("frontier candidates were not observed")
    result.ok = not result.errors
    if args.trace_out:
        capture_trace_snapshot()
        trace_payload = {
            "schema_version": "lingtu.gazebo_frontier_trace.v1",
            "result": result.as_dict(),
            "coverage": {
                "resolution_m": coverage.resolution,
                "origin_x": coverage.origin_x,
                "origin_y": coverage.origin_y,
                "width": coverage.width,
                "height": coverage.height,
            },
            "samples": trace,
        }
        path = Path(args.trace_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(trace_payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout-sec", type=float, default=45.0)
    parser.add_argument("--coverage-size-m", type=float, default=12.0)
    parser.add_argument("--coverage-resolution-m", type=float, default=0.1)
    parser.add_argument("--reveal-radius-m", type=float, default=0.7)
    parser.add_argument("--coverage-update-sec", type=float, default=0.25)
    parser.add_argument("--frontier-goal-timeout-sec", type=float, default=8.0)
    parser.add_argument("--frontier-rate-hz", type=float, default=2.0)
    parser.add_argument("--min-frontier-size", type=int, default=3)
    parser.add_argument("--goal-reached-radius-m", type=float, default=0.45)
    parser.add_argument("--min-cmd-vel", type=float, default=0.01)
    parser.add_argument("--min-odom-delta-m", type=float, default=0.05)
    parser.add_argument("--min-explored-area-delta-m2", type=float, default=0.15)
    parser.add_argument("--trace-out", default="")
    parser.add_argument("--trace-interval-sec", type=float, default=0.35)
    parser.add_argument("--trace-max-known-points", type=int, default=1200)
    parser.add_argument("--continue-after-pass-sec", type=float, default=0.0)
    parser.add_argument("--require-cumulative-map", action="store_true")
    parser.add_argument("--max-cloud-points", type=int, default=200000)
    parser.add_argument("--min-cumulative-samples", type=int, default=8)
    parser.add_argument("--cumulative-voxel-size-m", type=float, default=0.2)
    parser.add_argument("--min-cumulative-point-growth-ratio", type=float, default=1.25)
    parser.add_argument("--min-cumulative-point-delta", type=int, default=1000)
    parser.add_argument("--min-cumulative-voxel-growth-ratio", type=float, default=1.2)
    parser.add_argument("--min-cumulative-voxel-delta", type=int, default=80)
    parser.add_argument("--min-cumulative-growth-step-ratio", type=float, default=0.6)
    parser.add_argument("--min-cumulative-retention-ratio", type=float, default=0.6)
    parser.add_argument("--min-map-vs-registered-voxel-ratio", type=float, default=1.5)
    parser.add_argument("--max-static-centroid-drift-m", type=float, default=0.25)
    parser.add_argument("--min-static-roi-points", type=int, default=30)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--json-out", default="")
    args = parser.parse_args()

    try:
        result = run_smoke(args)
    except ImportError as exc:
        result = GazeboFrontierExplorationResult(errors=[f"dependencies unavailable: {exc}"])
    payload = json.dumps(result.as_dict(), ensure_ascii=False, indent=2)
    if args.json:
        print(payload)
    else:
        print(("PASSED" if result.ok else "FAILED") + ": LingTu Gazebo frontier exploration smoke")
        print(payload)
    if args.json_out:
        path = Path(args.json_out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(payload + "\n", encoding="utf-8")
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
