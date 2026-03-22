"""
A→B 点导航测试场景

设定起点和终点，等待机器人到达目标。
成功条件: 机器人到达目标点 goal_radius 范围内。
失败条件: 超时。
"""
from typing import Tuple, Optional

import numpy as np

from .base import Scenario


class NavigationScenario(Scenario):
    """A→B 点导航测试。

    用法:
        scenario = NavigationScenario(
            start=(0.0, 0.0, 0.35),
            goal=(10.0, 5.0),
            goal_radius=1.0,
            max_time=120.0,
        )
    """

    name = "navigation"
    description = "A→B 点导航: 从起点驶向终点，检查是否到达"

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
        self._goal_pub = None  # ROS2 publisher (延迟初始化)
        self._last_dist: float = float("inf")
        self._path_length: float = 0.0
        self._last_pos: Optional[np.ndarray] = None

    # ── 生命周期 ──────────────────────────────────────────────────────────────

    def setup(self, engine) -> None:
        super().setup(engine)

        # 设置机器人初始位姿
        engine.set_robot_pose(self.start[0], self.start[1], self.start[2])

        # 通过 ROS2 发布 goal_pose
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

        # 累计路程
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

    # ── 内部 ──────────────────────────────────────────────────────────────────

    def _publish_goal(self, engine):
        """通过 ROS2 发布 /nav/goal_pose."""
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
