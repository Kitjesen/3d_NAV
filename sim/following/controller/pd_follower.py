"""PD following controller with velocity prediction.

Instead of chasing the person's current position, predicts where
they will be in predict_dt seconds and chases that point.
This eliminates the "always behind" problem.

The follow point is computed as:
  predicted_pos = person_pos + person_vel * predict_dt
  follow_point  = predicted_pos + (robot→predicted) * target_distance / dist
"""
from __future__ import annotations

import math

import numpy as np

from sim.following.interfaces import FollowCommand, PerceivedTarget


class PDFollower:
    """PD controller with velocity prediction for person following."""

    def __init__(
        self,
        target_distance: float = 1.5,
        predict_dt: float = 0.5,
        kp_linear: float = 1.2,
        kp_lateral: float = 0.5,
        kp_angular: float = 1.5,
        max_vx: float = 1.0,
        max_vy: float = 0.4,
        max_dyaw: float = 1.5,
    ):
        self.target_distance = target_distance
        self.predict_dt = predict_dt
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
        # Predict where person will be
        pred_x = target.position_world[0] + target.velocity_world[0] * self.predict_dt
        pred_y = target.position_world[1] + target.velocity_world[1] * self.predict_dt

        # Compute follow point: target_distance behind the predicted position
        # "Behind" = in the direction from predicted pos toward robot
        dx_w = pred_x - robot_pos[0]
        dy_w = pred_y - robot_pos[1]
        dist_to_pred = math.hypot(dx_w, dy_w)

        if dist_to_pred < 0.01:
            return FollowCommand()

        # Follow point: offset from predicted pos toward robot by target_distance
        follow_x = pred_x - (dx_w / dist_to_pred) * self.target_distance
        follow_y = pred_y - (dy_w / dist_to_pred) * self.target_distance

        # Vector from robot to follow point in world frame
        fx_w = follow_x - robot_pos[0]
        fy_w = follow_y - robot_pos[1]

        # Transform to robot body frame
        cos_y = math.cos(-robot_yaw)
        sin_y = math.sin(-robot_yaw)
        fx_b = cos_y * fx_w - sin_y * fy_w  # forward
        fy_b = sin_y * fx_w + cos_y * fy_w  # left

        dist_to_follow = math.hypot(fx_b, fy_b)
        angle_to_follow = math.atan2(fy_b, fx_b)

        # Velocity commands — proportional to distance to follow point
        vx = np.clip(self.kp_linear * fx_b, -self.max_vx, self.max_vx)
        vy = np.clip(self.kp_lateral * fy_b, -self.max_vy, self.max_vy)
        dyaw = np.clip(self.kp_angular * angle_to_follow, -self.max_dyaw, self.max_dyaw)

        # Decelerate when within target distance
        if dist_to_pred < self.target_distance:
            scale = max(0.1, dist_to_pred / self.target_distance)
            vx *= scale
            vy *= scale

        return FollowCommand(vx=float(vx), vy=float(vy), dyaw=float(dyaw))

    def reset(self) -> None:
        pass
