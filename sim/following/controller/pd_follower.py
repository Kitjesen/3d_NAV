"""PD following controller — proportional velocity control.

Transforms perceived target into robot body frame, applies
proportional gains on distance and angular error.
"""
from __future__ import annotations

import math

import numpy as np

from sim.following.interfaces import FollowCommand, PerceivedTarget


class PDFollower:
    """Simple proportional-derivative following controller.

    Computes (vx, vy, dyaw) from robot state and perceived target position.
    """

    def __init__(
        self,
        target_distance: float = 1.5,
        kp_linear: float = 0.8,
        kp_lateral: float = 0.4,
        kp_angular: float = 1.2,
        max_vx: float = 1.0,
        max_vy: float = 0.3,
        max_dyaw: float = 1.0,
    ):
        self.target_distance = target_distance
        self.kp_linear = kp_linear
        self.kp_lateral = kp_lateral
        self.kp_angular = kp_angular
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_dyaw = max_dyaw

    def compute(
        self,
        robot_pos: np.ndarray,
        robot_yaw: float,
        target: PerceivedTarget,
        dt: float,
    ) -> FollowCommand:
        # Vector from robot to target in world frame
        dx_w = target.position_world[0] - robot_pos[0]
        dy_w = target.position_world[1] - robot_pos[1]

        # Transform to robot body frame
        cos_y = math.cos(-robot_yaw)
        sin_y = math.sin(-robot_yaw)
        dx_b = cos_y * dx_w - sin_y * dy_w  # forward
        dy_b = sin_y * dx_w + cos_y * dy_w  # left

        distance = math.hypot(dx_b, dy_b)
        angle_to_target = math.atan2(dy_b, dx_b)

        # Distance error (positive = too far, need to move forward)
        dist_error = distance - self.target_distance

        # Velocity commands
        vx = np.clip(self.kp_linear * dist_error, -self.max_vx, self.max_vx)
        vy = np.clip(self.kp_lateral * dy_b, -self.max_vy, self.max_vy)
        dyaw = np.clip(self.kp_angular * angle_to_target, -self.max_dyaw, self.max_dyaw)

        # Slow down when close
        if distance < self.target_distance * 0.5:
            vx *= 0.3
            vy *= 0.3

        return FollowCommand(vx=float(vx), vy=float(vy), dyaw=float(dyaw))

    def reset(self) -> None:
        pass
