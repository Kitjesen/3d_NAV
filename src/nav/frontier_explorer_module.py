"""WavefrontFrontierExplorer — autonomous frontier-based exploration planner.

Finds boundaries between known-free and unknown space (frontiers) via BFS,
clusters them, scores each cluster, and publishes the best goal for exploration.

Grid cell conventions (standard ROS2 OccupancyGrid):
  FREE     =  0
  OCCUPIED = 100
  UNKNOWN  = -1

Scoring weights:
  30% information gain  (cluster size → proxy for how much unknown area opens up)
  30% novelty           (distance from previously visited goals)
  20% distance          (sweet spot 3-8 m from robot)
  15% safety            (clearance from nearest obstacle)
   5% momentum          (prefer continuing in current heading)

Usage::

    bp.add(WavefrontFrontierExplorer, min_frontier_size=5, explore_rate=2.0)
    # auto_wire connects costmap + odometry
"""

from __future__ import annotations

import logging
import math
import threading
import time
from collections import deque
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from core.module import Module, skill
from core.stream import In, Out
from core.msgs.nav import Odometry
from core.msgs.geometry import Pose, PoseStamped
from core.registry import register

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Grid constants
# ---------------------------------------------------------------------------

FREE     =  0
OCCUPIED = 100
UNKNOWN  = -1

# 4-connectivity neighbours (row_delta, col_delta)
_NEIGH4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

# 8-connectivity (used when expanding BFS wavefront)
_NEIGH8 = [
    (-1, -1), (-1, 0), (-1, 1),
    ( 0, -1),          ( 0, 1),
    ( 1, -1), ( 1, 0), ( 1, 1),
]


# ---------------------------------------------------------------------------
# Module
# ---------------------------------------------------------------------------

@register("navigation", "wavefront_frontier",
          description="Wavefront frontier exploration")
class WavefrontFrontierExplorer(Module, layer=2):
    """Autonomous exploration: BFS wavefront → frontier clusters → scored goal.

    Ports in:
        costmap      — dict from OccupancyGridModule with keys:
                        'grid'       np.ndarray (H×W, dtype int8/int16)
                        'resolution' float  (m/cell)
                        'origin_x'   float  (world x of cell [0,0])
                        'origin_y'   float  (world y of cell [0,0])
                        'width'      int
                        'height'     int
        odometry     — Odometry (robot world pose)
        goal_reached — bool  (True when NavigationModule signals arrival)

    Ports out:
        exploration_goal — PoseStamped  (next frontier centroid in map frame)
        frontiers        — list of dicts [{cx, cy, size, score}, ...]
        exploring        — bool  (True while exploration thread is running)
    """

    # -- Inputs --
    costmap:      In[dict]
    odometry:     In[Odometry]
    goal_reached: In[bool]

    # -- Outputs --
    exploration_goal: Out[PoseStamped]
    frontiers:        Out[list]
    exploring:        Out[bool]

    def __init__(
        self,
        min_frontier_size: int   = 5,
        safe_distance: float     = 1.0,    # m — minimum clearance from obstacles
        lookahead_distance: float = 5.0,   # m — max BFS expansion radius
        max_explored_distance: float = 15.0,  # m — frontier beyond this is ignored
        info_gain_threshold: float = 0.03,  # stop if all frontiers below this gain
        goal_timeout: float      = 30.0,   # s — give up on a goal after this
        explore_rate: float      = 2.0,    # Hz — how often to re-evaluate frontiers
        **kw,
    ):
        super().__init__(**kw)

        # Parameters
        self._min_size       = min_frontier_size
        self._safe_dist      = safe_distance
        self._lookahead      = lookahead_distance
        self._max_dist       = max_explored_distance
        self._info_gain_thr  = info_gain_threshold
        self._goal_timeout   = goal_timeout
        self._rate           = explore_rate

        # Costmap freshness
        self._costmap_ts: float = 0.0
        self._costmap_max_age: float = kw.get("costmap_max_age", 30.0)

        # Sensor state (updated by subscribers, read by exploration thread)
        self._costmap_data: Optional[dict] = None
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._robot_yaw: float = 0.0
        self._state_lock = threading.Lock()

        # Goal-reached event (set by subscriber, cleared by exploration thread)
        self._goal_reached_event = threading.Event()

        # Exploration thread
        self._explore_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # Visited goals: list of (x, y) to avoid revisiting
        self._visited_goals: List[Tuple[float, float]] = []
        self._consecutive_low_gain: int = 0
        self._max_low_gain: int = 3  # stop after N consecutive low-gain attempts

        # Current heading (radians) for momentum scoring
        self._current_heading: float = 0.0

    # -------------------------------------------------------------------------
    # Lifecycle
    # -------------------------------------------------------------------------

    def setup(self) -> None:
        self.costmap.subscribe(self._on_costmap)
        self.odometry.subscribe(self._on_odometry)
        self.goal_reached.subscribe(self._on_goal_reached)

    def start(self) -> None:
        super().start()
        self.exploring.publish(False)

    def stop(self) -> None:
        self._stop_event.set()
        if self._explore_thread and self._explore_thread.is_alive():
            self._explore_thread.join(timeout=3.0)
        super().stop()

    # -------------------------------------------------------------------------
    # Subscribers
    # -------------------------------------------------------------------------

    def _on_costmap(self, data: dict) -> None:
        with self._state_lock:
            self._costmap_data = data
            self._costmap_ts = time.time()

    def _on_odometry(self, odom: Odometry) -> None:
        with self._state_lock:
            self._robot_x   = odom.x
            self._robot_y   = odom.y
            self._robot_yaw = odom.yaw

    def _on_goal_reached(self, reached: bool) -> None:
        if reached:
            self._goal_reached_event.set()

    # -------------------------------------------------------------------------
    # Skills
    # -------------------------------------------------------------------------

    @skill
    def begin_exploration(self) -> str:
        """Start autonomous frontier exploration. Returns status string."""
        if self._explore_thread and self._explore_thread.is_alive():
            return "already_running"
        self._stop_event.clear()
        self._goal_reached_event.clear()
        self._visited_goals.clear()
        self._consecutive_low_gain = 0
        self._explore_thread = threading.Thread(
            target=self._exploration_loop,
            name="frontier_explorer",
            daemon=True,
        )
        self._explore_thread.start()
        self.exploring.publish(True)
        logger.info("WavefrontFrontierExplorer: exploration started")
        return "started"

    @skill
    def end_exploration(self) -> str:
        """Stop autonomous frontier exploration. Returns status string."""
        self._stop_event.set()
        if self._explore_thread and self._explore_thread.is_alive():
            self._explore_thread.join(timeout=3.0)
        self.exploring.publish(False)
        logger.info("WavefrontFrontierExplorer: exploration stopped")
        return "stopped"

    @skill
    def get_frontiers(self) -> list:
        """Return the most recently computed frontier cluster list."""
        with self._state_lock:
            costmap_data = self._costmap_data
            rx, ry = self._robot_x, self._robot_y

        if costmap_data is None:
            return []

        grid, meta = self._parse_costmap(costmap_data)
        if grid is None:
            return []

        clusters = self._find_frontier_clusters(grid, meta, rx, ry)
        return [
            {
                "cx": float(c["cx"]),
                "cy": float(c["cy"]),
                "size": int(c["size"]),
                "score": float(c.get("score", 0.0)),
            }
            for c in clusters
        ]

    # -------------------------------------------------------------------------
    # Exploration loop (runs in background thread)
    # -------------------------------------------------------------------------

    def _exploration_loop(self) -> None:
        period = 1.0 / max(self._rate, 0.1)

        while not self._stop_event.is_set():
            loop_start = time.monotonic()

            # Snapshot sensor state
            with self._state_lock:
                costmap_data = self._costmap_data
                costmap_ts = self._costmap_ts
                rx, ry, ryaw = self._robot_x, self._robot_y, self._robot_yaw

            # Costmap freshness check
            if costmap_ts > 0 and time.time() - costmap_ts > self._costmap_max_age:
                logger.warning(
                    "FrontierExplorer: costmap stale (%.1fs > %.1fs) — skipping cycle",
                    time.time() - costmap_ts, self._costmap_max_age,
                )
                self._stop_event.wait(timeout=period)
                continue

            if costmap_data is None:
                logger.debug("FrontierExplorer: waiting for costmap...")
                self._stop_event.wait(timeout=period)
                continue

            grid, meta = self._parse_costmap(costmap_data)
            if grid is None:
                self._stop_event.wait(timeout=period)
                continue

            # Detect and score frontiers
            clusters = self._find_frontier_clusters(grid, meta, rx, ry)
            scored   = self._score_clusters(clusters, rx, ry, ryaw, meta)

            # Publish frontiers for visualization (regardless of selection)
            self.frontiers.publish(scored)

            if not scored:
                logger.info("FrontierExplorer: no frontiers found — exploration complete")
                break

            best = scored[0]

            # Check information gain threshold
            max_gain = best.get("info_gain_norm", 0.0)
            if max_gain < self._info_gain_thr:
                self._consecutive_low_gain += 1
                logger.info(
                    "FrontierExplorer: low info gain (%.3f < %.3f), consecutive=%d",
                    max_gain, self._info_gain_thr, self._consecutive_low_gain,
                )
                if self._consecutive_low_gain >= self._max_low_gain:
                    logger.info("FrontierExplorer: info gain persistently low — stopping")
                    break
            else:
                self._consecutive_low_gain = 0

            # Publish exploration goal
            goal = self._make_pose_stamped(best["cx"], best["cy"])
            self.exploration_goal.publish(goal)
            self._visited_goals.append((best["cx"], best["cy"]))
            logger.debug(
                "FrontierExplorer: goal (%.2f, %.2f) score=%.3f size=%d",
                best["cx"], best["cy"], best["score"], best["size"],
            )

            # Wait for goal_reached or timeout
            self._goal_reached_event.clear()
            reached = self._goal_reached_event.wait(timeout=self._goal_timeout)
            if not reached:
                logger.debug(
                    "FrontierExplorer: goal timeout after %.1fs — selecting next frontier",
                    self._goal_timeout,
                )

            # Sleep remaining period time (avoids tight loop on fast goal_reached)
            elapsed = time.monotonic() - loop_start
            remaining = period - elapsed
            if remaining > 0:
                self._stop_event.wait(timeout=remaining)

        self.exploring.publish(False)
        logger.info("WavefrontFrontierExplorer: exploration loop exited")

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        exploring = self._explore_thread is not None and self._explore_thread.is_alive()
        info["exploration_state"] = "exploring" if exploring else "idle"
        info["frontier_count"] = len(self._visited_goals)
        return info

    # -------------------------------------------------------------------------
    # Costmap parsing
    # -------------------------------------------------------------------------

    def _parse_costmap(
        self, data: dict
    ) -> Tuple[Optional[np.ndarray], Optional[dict]]:
        """Extract grid array and geometry metadata from costmap dict.

        Returns (grid_2d, meta) where meta has resolution/origin_x/origin_y,
        or (None, None) on error.
        """
        try:
            grid = data["grid"]
            if not isinstance(grid, np.ndarray):
                grid = np.array(grid, dtype=np.int16)
            else:
                grid = grid.astype(np.int16)

            h = int(data.get("height", grid.shape[0]))
            w = int(data.get("width", grid.shape[1] if grid.ndim == 2 else 0))

            if grid.ndim == 1:
                grid = grid.reshape(h, w)

            meta = {
                "resolution": float(data.get("resolution", 0.05)),
                "origin_x":   float(data.get("origin_x", 0.0)),
                "origin_y":   float(data.get("origin_y", 0.0)),
                "width":      w,
                "height":     h,
            }
            return grid, meta

        except Exception as exc:
            logger.warning("FrontierExplorer: costmap parse error: %s", exc)
            return None, None

    # -------------------------------------------------------------------------
    # Wavefront BFS frontier detection
    # -------------------------------------------------------------------------

    def _find_frontier_clusters(
        self,
        grid: np.ndarray,
        meta: dict,
        robot_x: float,
        robot_y: float,
    ) -> List[dict]:
        """BFS from robot position to find frontier cells, then cluster them.

        A frontier cell is a FREE cell that has at least one UNKNOWN neighbour.
        Returns a list of cluster dicts: {cx, cy, size, cells}.
        """
        res      = meta["resolution"]
        origin_x = meta["origin_x"]
        origin_y = meta["origin_y"]
        h, w     = grid.shape

        # Convert robot world position to grid cell
        start_col = int((robot_x - origin_x) / res)
        start_row = int((robot_y - origin_y) / res)

        if not (0 <= start_row < h and 0 <= start_col < w):
            logger.debug("FrontierExplorer: robot outside grid bounds")
            return []

        # BFS max radius in cells
        max_radius_cells = int(self._max_dist / res)

        # BFS through free cells, collect frontier cells
        visited     = np.zeros((h, w), dtype=bool)
        frontier_mask = np.zeros((h, w), dtype=bool)

        queue = deque()
        queue.append((start_row, start_col, 0))
        visited[start_row, start_col] = True

        while queue:
            r, c, depth = queue.popleft()

            if depth > max_radius_cells:
                continue

            cell_val = int(grid[r, c])

            # Only propagate through free cells
            if cell_val == OCCUPIED or cell_val > 50:
                continue

            # Check if this free cell borders an unknown cell → frontier
            is_frontier = False
            for dr, dc in _NEIGH4:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w:
                    nval = int(grid[nr, nc])
                    if nval == UNKNOWN or nval < 0:
                        is_frontier = True
                        break

            if is_frontier:
                frontier_mask[r, c] = True

            # Expand neighbours
            for dr, dc in _NEIGH8:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc]:
                    nval = int(grid[nr, nc])
                    # Traverse free or mildly inflated cells (< 50)
                    if nval != OCCUPIED and nval <= 50:
                        visited[nr, nc] = True
                        queue.append((nr, nc, depth + 1))

        # Connected-component labelling on frontier_mask
        clusters: List[dict] = []
        cluster_visited = np.zeros((h, w), dtype=bool)

        frontier_rows, frontier_cols = np.where(frontier_mask)

        for r0, c0 in zip(frontier_rows.tolist(), frontier_cols.tolist()):
            if cluster_visited[r0, c0]:
                continue

            # Flood-fill this cluster with 8-connectivity
            cells: List[Tuple[int, int]] = []
            cq = deque()
            cq.append((r0, c0))
            cluster_visited[r0, c0] = True

            while cq:
                cr, cc = cq.popleft()
                cells.append((cr, cc))
                for dr, dc in _NEIGH8:
                    nr, nc = cr + dr, cc + dc
                    if (0 <= nr < h and 0 <= nc < w
                            and not cluster_visited[nr, nc]
                            and frontier_mask[nr, nc]):
                        cluster_visited[nr, nc] = True
                        cq.append((nr, nc))

            if len(cells) < self._min_size:
                continue

            # Centroid in world coords
            mean_row = sum(r for r, _ in cells) / len(cells)
            mean_col = sum(c for _, c in cells) / len(cells)
            cx = origin_x + mean_col * res
            cy = origin_y + mean_row * res

            clusters.append({
                "cx":    cx,
                "cy":    cy,
                "size":  len(cells),
                "cells": cells,
            })

        return clusters

    # -------------------------------------------------------------------------
    # Cluster scoring
    # -------------------------------------------------------------------------

    def _score_clusters(
        self,
        clusters: List[dict],
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        meta: dict,
    ) -> List[dict]:
        """Score and sort clusters (highest first).

        Weights:
          0.30 — information gain  (normalised cluster size)
          0.30 — novelty           (distance from nearest visited goal)
          0.20 — distance          (sweet-spot 3-8 m)
          0.15 — safety            (clearance from obstacles, approx via costmap)
          0.05 — momentum          (alignment with current heading)
        """
        if not clusters:
            return []

        max_size = max(c["size"] for c in clusters) or 1

        # Compute raw scores
        for c in clusters:
            dx = c["cx"] - robot_x
            dy = c["cy"] - robot_y
            dist = math.hypot(dx, dy)

            # Information gain: normalised cluster size
            info_gain_norm = c["size"] / max_size
            c["info_gain_norm"] = info_gain_norm

            # Novelty: minimum distance from all previously visited goals
            if self._visited_goals:
                novelty_dist = min(
                    math.hypot(c["cx"] - vx, c["cy"] - vy)
                    for vx, vy in self._visited_goals
                )
                # Saturate at 10 m
                novelty_score = min(novelty_dist / 10.0, 1.0)
            else:
                novelty_score = 1.0

            # Distance: sweet spot 3-8 m → peak at 5.5 m
            sweet_low, sweet_high = 3.0, 8.0
            sweet_mid = (sweet_low + sweet_high) / 2.0
            dist_score = max(0.0, 1.0 - abs(dist - sweet_mid) / sweet_mid)

            # Safety: approximate using distance to nearest OCCUPIED cell
            # We use the robot→centroid midpoint check (cheap proxy)
            safety_score = self._approx_safety(
                c["cx"], c["cy"], robot_x, robot_y, meta
            )

            # Momentum: dot-product alignment with robot heading
            if dist > 0.01:
                goal_angle = math.atan2(dy, dx)
                angle_diff = abs(_wrap_angle(goal_angle - robot_yaw))
                momentum_score = max(0.0, 1.0 - angle_diff / math.pi)
            else:
                momentum_score = 0.0

            # Weighted sum
            score = (
                0.30 * info_gain_norm
                + 0.30 * novelty_score
                + 0.20 * dist_score
                + 0.15 * safety_score
                + 0.05 * momentum_score
            )

            c["score"]          = score
            c["info_gain_norm"] = info_gain_norm
            c["novelty_score"]  = novelty_score
            c["dist_score"]     = dist_score
            c["safety_score"]   = safety_score
            c["momentum_score"] = momentum_score
            c["distance"]       = dist

        # Sort descending by score; strip heavy 'cells' list for output
        clusters.sort(key=lambda c: c["score"], reverse=True)

        output = []
        for c in clusters:
            output.append({
                "cx":             c["cx"],
                "cy":             c["cy"],
                "size":           c["size"],
                "score":          c["score"],
                "info_gain_norm": c["info_gain_norm"],
                "novelty_score":  c["novelty_score"],
                "dist_score":     c["dist_score"],
                "safety_score":   c["safety_score"],
                "momentum_score": c["momentum_score"],
                "distance":       c["distance"],
            })
        return output

    def _approx_safety(
        self,
        cx: float,
        cy: float,
        robot_x: float,
        robot_y: float,
        meta: dict,
    ) -> float:
        """Estimate clearance at the frontier centroid.

        Samples cells in a small radius around the centroid and counts
        obstacle-free cells as a fraction → safety score in [0, 1].
        """
        with self._state_lock:
            costmap_data = self._costmap_data
        if costmap_data is None:
            return 0.5

        try:
            grid, _ = self._parse_costmap(costmap_data)
            if grid is None:
                return 0.5

            res      = meta["resolution"]
            origin_x = meta["origin_x"]
            origin_y = meta["origin_y"]
            h, w     = grid.shape

            safe_cells_r = int(self._safe_dist / res)
            cr = int((cy - origin_y) / res)
            cc = int((cx - origin_x) / res)

            total = 0
            free  = 0
            for dr in range(-safe_cells_r, safe_cells_r + 1):
                for dc in range(-safe_cells_r, safe_cells_r + 1):
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < h and 0 <= nc < w:
                        total += 1
                        val = int(grid[nr, nc])
                        if val != OCCUPIED and val <= 50:
                            free += 1

            return (free / total) if total > 0 else 0.5

        except Exception:
            return 0.5

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _make_pose_stamped(self, x: float, y: float) -> PoseStamped:
        """Create a PoseStamped in the map frame at (x, y, 0)."""
        ps = PoseStamped()
        ps.x = x
        ps.y = y
        ps.z = 0.0
        ps.frame_id = "map"
        ps.ts = time.time()
        return ps


# ---------------------------------------------------------------------------
# Angle utility
# ---------------------------------------------------------------------------

def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi
