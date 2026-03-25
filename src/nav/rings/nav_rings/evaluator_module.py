#!/usr/bin/env python3
"""
Ring 2 -- EvaluatorModule -- hive Module version
=================================================
Extracted pure evaluation logic from evaluator.py into core.Module.
"""

from __future__ import annotations

import math
import time
from enum import Enum
from typing import Any, List, Optional, Tuple

import numpy as np

import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from core import Module, In, Out
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import Twist, Vector3
from core.msgs.semantic import ExecutionEval


class Assessment(Enum):
    IDLE = "IDLE"
    ON_TRACK = "ON_TRACK"
    DRIFTING = "DRIFTING"
    STALLED = "STALLED"
    REGRESSING = "REGRESSING"


class EvaluatorModule(Module, layer=2):
    """Ring 2: closed-loop execution evaluator (hive Module).
    Pure evaluation logic, no ROS2. Driven by tick().
    """

    odometry: In[Odometry]
    path: In[Path]
    waypoint: In[Vector3]
    cmd_vel: In[Twist]

    execution_eval: Out[ExecutionEval]

    def __init__(self, **config: Any) -> None:
        super().__init__(**config)
        self._ct_warn = config.get("cross_track_warn", 1.5)
        self._ct_danger = config.get("cross_track_danger", 3.0)
        self._stall_thr = config.get("stall_threshold", 0.05)
        self._progress_window = config.get("progress_window_sec", 3.0)
        self._min_progress = config.get("min_progress_rate", -0.02)

        self._robot_xy = np.zeros(2)
        self._robot_yaw: float = 0.0
        self._path_points: Optional[np.ndarray] = None
        self._goal_xy: Optional[np.ndarray] = None
        self._waypoint_xy: Optional[np.ndarray] = None
        self._cmd_speed: float = 0.0
        self._actual_speed: float = 0.0
        self._progress_history: List[Tuple[float, float]] = []
        self._last_progress_time: float = time.monotonic()
        self._last_distance: float = float("inf")

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.path.subscribe(self._on_path)
        self.waypoint.subscribe(self._on_waypoint)
        self.cmd_vel.subscribe(self._on_cmdvel)

    def _on_odom(self, msg: Odometry) -> None:
        x, y = msg.x, msg.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        self._robot_xy = np.array([x, y])
        self._robot_yaw = msg.yaw
        vx, vy = msg.vx, msg.vy
        if math.isfinite(vx) and math.isfinite(vy):
            self._actual_speed = math.hypot(vx, vy)

    def _on_path(self, msg: Path) -> None:
        if len(msg) < 2:
            self._path_points = None
            self._goal_xy = None
            return
        pts = []
        for ps in msg.poses:
            px, py = ps.x, ps.y
            if math.isfinite(px) and math.isfinite(py):
                pts.append([px, py])
        if len(pts) >= 2:
            self._path_points = np.array(pts)
            self._goal_xy = self._path_points[-1].copy()
            self._progress_history.clear()
            self._last_progress_time = time.monotonic()
            self._last_distance = float("inf")

    def _on_waypoint(self, msg: Vector3) -> None:
        if math.isfinite(msg.x) and math.isfinite(msg.y):
            self._waypoint_xy = np.array([msg.x, msg.y])

    def _on_cmdvel(self, msg: Twist) -> None:
        vx, vy = msg.linear.x, msg.linear.y
        if math.isfinite(vx) and math.isfinite(vy):
            self._cmd_speed = math.hypot(vx, vy)

    def cross_track_error(self) -> float:
        """Distance from robot to nearest path segment."""
        if self._path_points is None or len(self._path_points) < 2:
            return 0.0
        p = self._robot_xy
        a = self._path_points[:-1]
        b = self._path_points[1:]
        ab = b - a
        ap = p - a
        ab_sq = np.sum(ab * ab, axis=1)
        ab_sq = np.where(ab_sq < 1e-10, 1.0, ab_sq)
        t = np.clip(np.sum(ap * ab, axis=1) / ab_sq, 0.0, 1.0)
        proj = a + t[:, np.newaxis] * ab
        dists = np.linalg.norm(p - proj, axis=1)
        return float(np.min(dists))

    def distance_to_goal(self) -> float:
        if self._goal_xy is None:
            return float("inf")
        return float(np.linalg.norm(self._robot_xy - self._goal_xy))

    def heading_error(self) -> float:
        """Heading error (rad), normalized to [-pi, pi]."""
        target = self._waypoint_xy if self._waypoint_xy is not None else self._goal_xy
        if target is None:
            return 0.0
        dx = target[0] - self._robot_xy[0]
        dy = target[1] - self._robot_xy[1]
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            return 0.0
        desired_yaw = math.atan2(dy, dx)
        err = desired_yaw - self._robot_yaw
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        return err

    def compute_progress_rate(self) -> float:
        """Sliding-window distance change rate (m/s, negative=approaching)."""
        now = time.monotonic()
        dist = self.distance_to_goal()
        self._progress_history.append((now, dist))
        cutoff = now - self._progress_window
        self._progress_history = [
            (t, d) for t, d in self._progress_history if t > cutoff
        ]
        if len(self._progress_history) < 2:
            return 0.0
        t0, d0 = self._progress_history[0]
        t1, d1 = self._progress_history[-1]
        dt = t1 - t0
        if dt < 0.1:
            return 0.0
        return (d1 - d0) / dt

    def assess(self, cte: float, progress_rate: float,
               stall_time: float) -> Assessment:
        """Assess current execution state."""
        if self._path_points is None:
            return Assessment.IDLE
        if progress_rate > abs(self._min_progress) and stall_time > 3.0:
            return Assessment.REGRESSING
        if stall_time > self._progress_window:
            return Assessment.STALLED
        if cte > self._ct_warn:
            return Assessment.DRIFTING
        return Assessment.ON_TRACK

    def tick(self) -> ExecutionEval:
        """Run one evaluation cycle. Publish and return ExecutionEval."""
        if self._path_points is None:
            result = ExecutionEval(assessment=Assessment.IDLE.value)
            self.execution_eval.publish(result)
            return result

        now = time.monotonic()
        cte = self.cross_track_error()
        dist = self.distance_to_goal()
        h_err = self.heading_error()
        progress_rate = self.compute_progress_rate()

        if dist < self._last_distance - 0.1:
            self._last_progress_time = now
            self._last_distance = dist
        stall_time = now - self._last_progress_time

        assessment = self.assess(cte, progress_rate, stall_time)
        vel_eff = (self._actual_speed / self._cmd_speed
                   if self._cmd_speed > 0.05 else 1.0)

        result = ExecutionEval(
            assessment=assessment.value,
            cross_track_error=round(cte, 2),
            distance_to_goal=round(dist, 1),
            progress_rate=round(progress_rate, 3),
            heading_error_deg=round(math.degrees(h_err), 1),
            stall_time=round(stall_time, 1),
            velocity_efficiency=round(min(vel_eff, 2.0), 2),
        )
        self.execution_eval.publish(result)
        return result

    @property
    def has_path(self) -> bool:
        return self._path_points is not None

    @property
    def robot_position(self) -> np.ndarray:
        return self._robot_xy.copy()
