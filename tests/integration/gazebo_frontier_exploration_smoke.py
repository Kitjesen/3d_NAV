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
            "samples": self.samples,
            "errors": self.errors,
        }


def _count(samples: dict[str, int], topic: str) -> None:
    samples[topic] = samples.get(topic, 0) + 1


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


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

    node.create_subscription(Odometry, "/nav/odometry", on_odom, 10)
    node.create_subscription(ROSPath, "/nav/global_path", on_global_path, 10)
    node.create_subscription(ROSPath, "/nav/local_path", on_local_path, 10)
    node.create_subscription(TwistStamped, "/nav/cmd_vel", on_cmd_vel, 10)
    node.create_subscription(PointCloud2, "/nav/map_cloud", on_map_cloud, 10)

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
