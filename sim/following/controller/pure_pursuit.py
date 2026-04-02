"""Pure Pursuit following controller — geometric path tracking.

Maintains a short buffer of recent target positions as a local path,
then applies the pure pursuit geometric law for smoother trajectory
tracking compared to PD.

The key insight: instead of pointing directly at the target (PD),
Pure Pursuit picks a lookahead point along the target's recent path,
producing smooth arcs instead of abrupt direction changes.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from sim.following.interfaces import FollowCommand, PerceivedTarget


class PurePursuitFollower:
    """Pure Pursuit geometric follower with path history.

    Collects target positions over time to form a local path.
    Uses the classic pure pursuit law: curvature = 2*sin(alpha)/L
    where alpha is the angle to the lookahead point and L is the
    lookahead distance.
    """

    def __init__(
        self,
        target_distance: float = 1.5,
        lookahead_distance: float = 2.5,
        max_vx: float = 1.0,
        max_vy: float = 0.3,
        max_dyaw: float = 1.0,
        kp_linear: float = 0.8,
        path_buffer_size: int = 50,
    ):
        self.target_distance = target_distance
        self.lookahead_distance = lookahead_distance
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_dyaw = max_dyaw
        self.kp_linear = kp_linear
        # Path history: ring buffer of (x, y, timestamp)
        self._path: deque = deque(maxlen=path_buffer_size)
        self._last_target_pos: np.ndarray | None = None

    def compute(
        self,
        robot_pos: np.ndarray,
        robot_yaw: float,
        target: PerceivedTarget,
        dt: float,
    ) -> FollowCommand:
        target_xy = target.position_world[:2]

        # Add to path history (deduplicate close points)
        if (
            self._last_target_pos is None
            or np.linalg.norm(target_xy - self._last_target_pos) > 0.05
        ):
            self._path.append(target_xy.copy())
            self._last_target_pos = target_xy.copy()

        # Compute distance to current target
        dx_w = target_xy[0] - robot_pos[0]
        dy_w = target_xy[1] - robot_pos[1]
        distance = math.hypot(dx_w, dy_w)

        # Find lookahead point on the path
        lookahead_pt = self._find_lookahead(robot_pos[:2])

        # Transform lookahead point to robot body frame
        lx_w = lookahead_pt[0] - robot_pos[0]
        ly_w = lookahead_pt[1] - robot_pos[1]
        cos_y = math.cos(-robot_yaw)
        sin_y = math.sin(-robot_yaw)
        lx_b = cos_y * lx_w - sin_y * ly_w   # forward
        ly_b = sin_y * lx_w + cos_y * ly_w   # left

        lookahead_dist = math.hypot(lx_b, ly_b)
        if lookahead_dist < 0.01:
            return FollowCommand()

        # Pure pursuit: curvature = 2 * sin(alpha) / L
        alpha = math.atan2(ly_b, lx_b)
        curvature = 2.0 * math.sin(alpha) / max(lookahead_dist, 0.1)

        # Forward velocity from distance error
        dist_error = distance - self.target_distance
        vx = np.clip(self.kp_linear * dist_error, -self.max_vx, self.max_vx)

        # Slow down when close
        if distance < self.target_distance * 0.5:
            vx *= 0.3

        # Yaw rate from curvature (bicycle model: dyaw = curvature * vx)
        dyaw = np.clip(
            curvature * max(abs(vx), 0.2),
            -self.max_dyaw,
            self.max_dyaw,
        )

        # Lateral velocity: small correction for holonomic robot
        vy = np.clip(0.3 * ly_b / max(lookahead_dist, 0.1), -self.max_vy, self.max_vy)

        return FollowCommand(vx=float(vx), vy=float(vy), dyaw=float(dyaw))

    def _find_lookahead(self, robot_xy: np.ndarray) -> np.ndarray:
        """Find the lookahead point on the path history.

        Walks backward from the most recent path point to find the
        point closest to lookahead_distance from the robot.
        If no path history, returns the latest target position.
        """
        if len(self._path) == 0:
            return robot_xy

        # Find closest point on path to robot
        path_arr = np.array(list(self._path))
        dists_to_robot = np.linalg.norm(path_arr - robot_xy, axis=1)

        # Walk forward from closest point to find lookahead
        closest_idx = int(np.argmin(dists_to_robot))

        for i in range(closest_idx, len(path_arr)):
            if np.linalg.norm(path_arr[i] - robot_xy) >= self.lookahead_distance:
                return path_arr[i]

        # If no point is far enough, use the latest (furthest) point
        return path_arr[-1]

    def reset(self) -> None:
        self._path.clear()
        self._last_target_pos = None
