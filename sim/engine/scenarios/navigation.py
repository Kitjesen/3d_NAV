"""
A-to-B point navigation test scenario

Sets a start and goal position, waits for the robot to reach the goal.
Success: robot enters goal_radius around the target point.
Failure: timeout.
"""
from typing import Tuple, Optional

import numpy as np

from .base import Scenario


class NavigationScenario(Scenario):
    """A-to-B point navigation test.

    Usage:
        scenario = NavigationScenario(
            start=(0.0, 0.0, 0.35),
            goal=(10.0, 5.0),
            goal_radius=1.0,
            max_time=120.0,
        )
    """

    name = "navigation"
    description = "A-to-B navigation: drive from start to goal, check arrival"

    def __init__(
        self,
        start: Tuple[float, float, float] = (0.0, 0.0, 0.35),
        goal: Tuple[float, float] = (10.0, 5.0),
        goal_radius: float = 1.0,
        max_time: float = 120.0,
    ):
        super().__init__()
        self.start = start
        self.goal = goal
        self.goal_radius = goal_radius
        self.max_time = max_time

        self._goal_reached = False
        self._goal_pub = None  # ROS2 publisher (lazy init)
        self._last_dist: float = float("inf")
        self._path_length: float = 0.0
        self._last_pos: Optional[np.ndarray] = None

    # ── Lifecycle ─────────────────────────────────────────────────────────────

    def setup(self, engine) -> None:
        super().setup(engine)

        # Set robot initial pose
        engine.set_robot_pose(self.start[0], self.start[1], self.start[2])

        # Publish goal_pose via ROS2
        self._publish_goal(engine)

        print(
            f"[NavigationScenario] start={self.start} goal={self.goal} "
            f"radius={self.goal_radius}m timeout={self.max_time}s"
        )

    def is_complete(self, engine) -> bool:
        if self.is_timeout():
            self._metrics["failure_reason"] = "timeout"
            return True

        if engine is None:
            return self._goal_reached

        state = engine.get_robot_state()
        if state is None:
            return False

        pos = state.position
        dist = self._distance_2d(pos, self.goal)
        self._last_dist = dist

        # Accumulate path length
        if self._last_pos is not None:
            self._path_length += self._distance_2d(pos, self._last_pos)
        self._last_pos = np.array(pos[:2], dtype=float)

        if dist <= self.goal_radius:
            self._goal_reached = True
            self._metrics["dist_to_goal"] = round(dist, 3)
            self._metrics["path_length_m"] = round(self._path_length, 2)
            return True

        return False

    def is_success(self, engine) -> bool:
        return self._goal_reached

    def get_metrics(self):
        metrics = super().get_metrics()
        metrics["goal"] = list(self.goal)
        metrics["goal_radius_m"] = self.goal_radius
        metrics["dist_to_goal_final"] = round(self._last_dist, 3)
        metrics["path_length_m"] = round(self._path_length, 2)
        return metrics

    # ── Internal ──────────────────────────────────────────────────────────────

    def _publish_goal(self, engine):
        """Publish goal to /nav/goal_pose via ROS2."""
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped

            node = getattr(engine, "_bridge_node", None)
            if node is None:
                return

            pub = node.create_publisher(PoseStamped, "/nav/goal_pose", 10)
            msg = PoseStamped()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = float(self.goal[0])
            msg.pose.position.y = float(self.goal[1])
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0
            pub.publish(msg)
            self._goal_pub = pub
            print(f"[NavigationScenario] goal published: {self.goal}")
        except Exception as e:
            print(f"[NavigationScenario] goal publish failed: {e}")
