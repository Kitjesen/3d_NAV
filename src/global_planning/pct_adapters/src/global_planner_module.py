"""GlobalPlannerModule — pluggable global path planner as an independent Module.

Wraps any GlobalPlanner backend (PCT C++ ele_planner, pure Python A*, NativeModule)
into a core Module. Swapping planners = one Blueprint argument.

Backends:
  "pct"     — C++ ele_planner via .so (S100P aarch64, fast)
  "astar"   — Pure Python A* on tomogram (any platform, no .so)
  "native"  — NativeModule subprocess (full ROS2 node, legacy)

Usage::

    bp.add(GlobalPlannerModule, planner="astar", tomogram="building2_9.pickle")
    bp.add(GlobalPlannerModule, planner="pct")  # needs ele_planner.so
"""

from __future__ import annotations

import logging
import os
import time
from typing import Any, Dict, List, Optional

import numpy as np

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", ".."))

from core.module import Module
from core.stream import In, Out
from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion
from core.msgs.nav import Path, Odometry
from core.registry import register

logger = logging.getLogger(__name__)


class PlanResult:
    """Global planner output."""
    __slots__ = ("path", "status", "plan_ms", "planner_name")

    def __init__(self, path: list, status: str = "SUCCESS",
                 plan_ms: float = 0.0, planner_name: str = ""):
        self.path = path          # list of (x, y, z) tuples
        self.status = status      # IDLE / PLANNING / SUCCESS / FAILED
        self.plan_ms = plan_ms
        self.planner_name = planner_name

    def __repr__(self):
        return f"PlanResult({self.status}, {len(self.path)} waypoints, {self.plan_ms:.1f}ms)"


@register("planner", "global", description="Pluggable global path planner")
class GlobalPlannerModule(Module, layer=5):
    """Pluggable global path planner Module.

    In:  goal_pose (PoseStamped) — where to go
         odometry  (Odometry)    — where we are
    Out: global_path (list)      — planned waypoints
         planner_status (str)    — IDLE/PLANNING/SUCCESS/FAILED
    """

    goal_pose: In[PoseStamped]
    odometry: In[Odometry]
    global_path: Out[list]
    planner_status: Out[str]

    def __init__(
        self,
        planner: str = "astar",
        tomogram: str = "",
        obstacle_thr: float = 49.9,
        **kw,
    ):
        super().__init__(**kw)
        self._planner_name = planner
        self._tomogram = tomogram
        self._obstacle_thr = obstacle_thr
        self._backend = None
        self._latest_pos: Optional[np.ndarray] = None
        self._plan_count = 0
        self._total_plan_ms = 0.0

    def setup(self):
        self._backend = self._create_backend()
        self.goal_pose.subscribe(self._on_goal)
        self.odometry.subscribe(self._on_odom)
        self.planner_status.publish("IDLE")

    def _create_backend(self):
        name = self._planner_name.lower()
        if name == "astar":
            return _AStarBackend(
                tomogram_path=self._tomogram,
                obstacle_thr=self._obstacle_thr,
            )
        elif name == "pct":
            return _PCTBackend(
                tomogram_path=self._tomogram,
                obstacle_thr=self._obstacle_thr,
            )
        else:
            raise ValueError(
                f"Unknown planner '{name}'. Available: astar, pct"
            )

    def _on_odom(self, odom: Odometry):
        self._latest_pos = np.array([odom.x, odom.y, getattr(odom, 'z', 0.0)])

    def _on_goal(self, goal: PoseStamped):
        if self._backend is None:
            self.planner_status.publish("FAILED")
            return
        if self._latest_pos is None:
            logger.warning("GlobalPlannerModule: no odometry yet, can't plan")
            self.planner_status.publish("FAILED")
            return

        self.planner_status.publish("PLANNING")
        start = self._latest_pos
        end = np.array([goal.pose.position.x, goal.pose.position.y,
                        goal.pose.position.z])

        t0 = time.time()
        try:
            path = self._backend.plan(start, end)
            plan_ms = (time.time() - t0) * 1000
            self._plan_count += 1
            self._total_plan_ms += plan_ms

            if path and len(path) > 0:
                self.global_path.publish(path)
                self.planner_status.publish("SUCCESS")
            else:
                self.planner_status.publish("FAILED")
        except Exception:
            logger.exception("Global planning failed")
            self.planner_status.publish("FAILED")

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        avg_ms = (self._total_plan_ms / self._plan_count
                  if self._plan_count > 0 else 0.0)
        info["planner"] = {
            "backend": self._planner_name,
            "loaded": self._backend is not None,
            "plans": self._plan_count,
            "avg_ms": round(avg_ms, 1),
        }
        return info


# ---------------------------------------------------------------------------
# Backends (pure algorithm, no ROS2)
# ---------------------------------------------------------------------------

class _AStarBackend:
    """Pure Python A* on tomogram grid."""

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 49.9):
        self._grid = None
        self._resolution = 1.0
        self._origin = np.zeros(2)
        self._obstacle_thr = obstacle_thr
        if tomogram_path and os.path.exists(tomogram_path):
            self._load_tomogram(tomogram_path)

    def _load_tomogram(self, path: str):
        import pickle
        with open(path, "rb") as f:
            data = pickle.load(f)
        if isinstance(data, dict):
            self._grid = data.get("grid", data.get("traversability"))
            self._resolution = data.get("resolution", 1.0)
            origin = data.get("origin", [0, 0])
            self._origin = np.array(origin[:2])
        logger.info("A* loaded tomogram: %s, grid=%s", path,
                     self._grid.shape if self._grid is not None else None)

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list:
        if self._grid is None:
            return []
        # World → grid coords
        si = int((start[0] - self._origin[0]) / self._resolution)
        sj = int((start[1] - self._origin[1]) / self._resolution)
        gi = int((goal[0] - self._origin[0]) / self._resolution)
        gj = int((goal[1] - self._origin[1]) / self._resolution)

        h, w = self._grid.shape
        si, sj = max(0, min(si, w-1)), max(0, min(sj, h-1))
        gi, gj = max(0, min(gi, w-1)), max(0, min(gj, h-1))

        # A* search
        import heapq
        open_set = [(0, si, sj)]
        came_from = {}
        g_score = {(si, sj): 0}
        closed = set()

        while open_set:
            _, cx, cy = heapq.heappop(open_set)
            if (cx, cy) in closed:
                continue
            closed.add((cx, cy))

            if cx == gi and cy == gj:
                break

            for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
                nx, ny = cx + dx, cy + dy
                if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in closed:
                    if self._grid[ny, nx] > self._obstacle_thr:
                        continue
                    cost = 1.414 if abs(dx) + abs(dy) == 2 else 1.0
                    ng = g_score[(cx, cy)] + cost
                    if ng < g_score.get((nx, ny), float('inf')):
                        g_score[(nx, ny)] = ng
                        f = ng + ((gi-nx)**2 + (gj-ny)**2)**0.5
                        heapq.heappush(open_set, (f, nx, ny))
                        came_from[(nx, ny)] = (cx, cy)

        # Reconstruct path
        if (gi, gj) not in came_from and (gi, gj) != (si, sj):
            return []
        path = []
        cur = (gi, gj)
        while cur in came_from:
            wx = cur[0] * self._resolution + self._origin[0]
            wy = cur[1] * self._resolution + self._origin[1]
            path.append((wx, wy, goal[2] if len(goal) > 2 else 0.0))
            cur = came_from[cur]
        path.reverse()
        return path


class _PCTBackend:
    """C++ ele_planner via .so (aarch64 only)."""

    def __init__(self, tomogram_path: str = "", obstacle_thr: float = 50):
        self._planner = None
        self._tomogram_path = tomogram_path
        self._obstacle_thr = obstacle_thr
        try:
            sys.path.insert(0, os.path.join(
                os.path.dirname(__file__), "..", "..", "PCT_planner", "planner", "lib"))
            from planner_wrapper import TomogramPlanner
            self._planner = TomogramPlanner(tomogram_path)
            logger.info("PCT ele_planner loaded: %s", tomogram_path)
        except ImportError:
            logger.warning("ele_planner.so not available — use planner='astar' instead")

    def plan(self, start: np.ndarray, goal: np.ndarray) -> list:
        if self._planner is None:
            return []
        try:
            result = self._planner.plan(
                start[0], start[1], start[2] if len(start) > 2 else 0.0,
                goal[0], goal[1], goal[2] if len(goal) > 2 else 0.0,
            )
            if result is not None:
                return [(p[0], p[1], p[2]) for p in result]
        except Exception:
            logger.exception("PCT planning failed")
        return []
