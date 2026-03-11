#!/usr/bin/env python3
"""
MissionArc — 任务生命周期状态机

将全局规划、航点推进、卡死恢复、安全降级串成统一的 mission 级别 FSM。
综合 /nav/planner_status + /nav/adapter_status + /nav/odometry + /nav/stop
输出一致的 /nav/mission_status 供 App / gRPC / 日志系统消费。

状态流:
  IDLE ──goal_pose──→ PLANNING ──success──→ EXECUTING ──goal_reached──→ COMPLETE
                         │                     │
                       failed               stuck/warn
                         │                     │
                         ↓                     ↓
                       FAILED              RECOVERING ──timeout──→ REPLANNING
                                               │                     │
                                            cleared               success
                                               │                     │
                                               ↓                     ↓
                                           EXECUTING             EXECUTING
                                                                    │
                                                              replan_count > max
                                                                    │
                                                                    ↓
                                                                 FAILED

话题:
  订阅:
    /nav/goal_pose        (PoseStamped)  — 新任务触发
    /nav/planner_status   (String)       — 全局规划器状态
    /nav/adapter_status   (String JSON)  — 航点跟踪事件
    /nav/odometry         (Odometry)     — 机器人位置
    /nav/stop             (Int8)         — 安全信号 (0=clear, 2=stop)

  发布:
    /nav/mission_status   (String JSON)  — 任务状态 + 进度
    /nav/goal_pose        (PoseStamped)  — replan 时重发目标 (复用话题)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int8

import json
import math
import time
from enum import Enum


class MissionState(Enum):
    IDLE = "IDLE"
    PLANNING = "PLANNING"
    EXECUTING = "EXECUTING"
    RECOVERING = "RECOVERING"
    REPLANNING = "REPLANNING"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


# 可从 PLANNING 接受目标的状态
_ACCEPT_GOAL_STATES = {MissionState.IDLE, MissionState.COMPLETE, MissionState.FAILED}


class MissionArc(Node):
    def __init__(self):
        super().__init__('mission_arc')

        # ── 参数 ──
        self.declare_parameter('max_replan_count', 3)
        self.declare_parameter('recovery_timeout_sec', 8.0)
        self.declare_parameter('planning_timeout_sec', 30.0)
        self.declare_parameter('mission_timeout_sec', 600.0)
        self.declare_parameter('publish_hz', 2.0)

        self.max_replan = self.get_parameter('max_replan_count').value
        self.recovery_timeout = self.get_parameter('recovery_timeout_sec').value
        self.planning_timeout = self.get_parameter('planning_timeout_sec').value
        self.mission_timeout = self.get_parameter('mission_timeout_sec').value
        publish_hz = self.get_parameter('publish_hz').value

        # ── 状态 ──
        self._state = MissionState.IDLE
        self._goal = None              # PoseStamped
        self._goal_xy = (0.0, 0.0)     # 缓存目标 xy
        self._robot_xy = (0.0, 0.0)    # 当前位置
        self._mission_start = 0.0
        self._state_enter_time = 0.0
        self._replan_count = 0
        self._wp_completed = 0
        self._wp_total = 0
        self._estop_active = False
        self._last_planner_status = "IDLE"
        self._failure_reason = ""

        # ── QoS ──
        qos_goal = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ── 订阅 ──
        self.create_subscription(
            PoseStamped, '/nav/goal_pose', self._on_goal, qos_goal)
        self.create_subscription(
            String, '/nav/planner_status', self._on_planner_status, 10)
        self.create_subscription(
            String, '/nav/adapter_status', self._on_adapter_status, 10)
        self.create_subscription(
            Odometry, '/nav/odometry', self._on_odom, 10)
        self.create_subscription(
            Int8, '/nav/stop', self._on_stop, 10)

        # ── 发布 ──
        # mission_arc 是纯状态观察者，不主动发 goal_pose
        # replan 由 pct_path_adapter 自行处理 (它有 replanning 事件)
        self._status_pub = self.create_publisher(String, '/nav/mission_status', 10)

        # ── 定时器 ──
        self.create_timer(1.0 / publish_hz, self._tick)

        self.get_logger().info(
            f'MissionArc started (max_replan={self.max_replan}, '
            f'recovery_timeout={self.recovery_timeout}s)')

    # ================================================================
    # 状态转换
    # ================================================================

    def _set_state(self, new_state: MissionState, reason: str = ""):
        old = self._state
        if old == new_state:
            return
        self._state = new_state
        self._state_enter_time = time.monotonic()
        self.get_logger().info(f'Mission: {old.value} → {new_state.value}  {reason}')

        if new_state == MissionState.FAILED:
            self._failure_reason = reason
        if new_state in (MissionState.COMPLETE, MissionState.FAILED):
            self._publish_status()  # 立即发一次终态

    # ================================================================
    # 回调
    # ================================================================

    def _on_goal(self, msg: PoseStamped):
        """新目标 → 启动 mission"""
        if self._state not in _ACCEPT_GOAL_STATES and self._state != MissionState.EXECUTING:
            # EXECUTING 中收到新 goal → 覆盖当前任务
            if self._state in (MissionState.PLANNING, MissionState.RECOVERING,
                               MissionState.REPLANNING):
                self.get_logger().warn(
                    f'Goal received during {self._state.value}, overriding')

        self._goal = msg
        self._goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self._replan_count = 0
        self._wp_completed = 0
        self._wp_total = 0
        self._failure_reason = ""
        self._mission_start = time.monotonic()
        self._set_state(MissionState.PLANNING, "new goal received")

    def _on_planner_status(self, msg: String):
        status = msg.data.strip()
        self._last_planner_status = status

        if self._state == MissionState.PLANNING:
            if status == "SUCCESS":
                self._set_state(MissionState.EXECUTING, "path planned")
            elif status == "FAILED":
                self._set_state(MissionState.FAILED, "planner returned FAILED")

        elif self._state == MissionState.REPLANNING:
            if status == "SUCCESS":
                self._set_state(MissionState.EXECUTING, f"replan #{self._replan_count} succeeded")
            elif status == "FAILED":
                self._set_state(MissionState.FAILED,
                                f"replan #{self._replan_count} failed")

        elif self._state == MissionState.EXECUTING:
            if status == "WARN_STUCK":
                self._set_state(MissionState.RECOVERING, "pathFollower WARN_STUCK")
            elif status == "STUCK":
                self._set_state(MissionState.RECOVERING, "pathFollower STUCK")
            elif status == "GOAL_REACHED":
                self._set_state(MissionState.COMPLETE, "planner_status GOAL_REACHED")
            elif status == "FAILED":
                self._set_state(MissionState.FAILED, "planner FAILED during execution")

    def _on_adapter_status(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        event = data.get('event', '')

        if event == 'path_received':
            self._wp_total = data.get('total', 0)
            self._wp_completed = 0

        elif event == 'waypoint_reached':
            self._wp_completed = data.get('index', 0) + 1
            self._wp_total = data.get('total', self._wp_total)
            # 如果在 RECOVERING 中收到 waypoint_reached → 恢复
            if self._state == MissionState.RECOVERING:
                self._set_state(MissionState.EXECUTING, "recovered (waypoint advancing)")

        elif event == 'goal_reached':
            self._set_state(MissionState.COMPLETE, "adapter goal_reached")

        elif event == 'replanning':
            self._replan_count += 1

        elif event == 'stuck_final':
            if self._state in (MissionState.EXECUTING, MissionState.RECOVERING):
                self._try_replan("adapter stuck_final")

        elif event == 'failed':
            if self._state in (MissionState.EXECUTING, MissionState.PLANNING):
                self._set_state(MissionState.FAILED,
                                data.get('reason', 'adapter failed'))

    def _on_odom(self, msg: Odometry):
        self._robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_stop(self, msg: Int8):
        self._estop_active = (msg.data >= 2)

    # ================================================================
    # 核心逻辑
    # ================================================================

    def _try_replan(self, reason: str):
        """尝试 replan，超出预算则 FAIL

        mission_arc 不主动重发 goal_pose。replan 的实际执行由
        pct_path_adapter 完成 (它会发 replanning 事件并重新请求路径)。
        这里只记录状态，等待 planner_status 回报 SUCCESS/FAILED。
        """
        if self._replan_count >= self.max_replan:
            self._set_state(MissionState.FAILED,
                            f"exceeded max replan ({self.max_replan}): {reason}")
            return

        self._replan_count += 1
        self._set_state(MissionState.REPLANNING,
                        f"replan #{self._replan_count}: {reason}")

    def _tick(self):
        """定时检查超时 + 发布状态"""
        now = time.monotonic()
        dt_state = now - self._state_enter_time

        # PLANNING 超时
        if self._state == MissionState.PLANNING:
            if dt_state > self.planning_timeout:
                self._set_state(MissionState.FAILED,
                                f"planning timeout ({self.planning_timeout}s)")

        # RECOVERING 超时 → replan
        elif self._state == MissionState.RECOVERING:
            if dt_state > self.recovery_timeout:
                self._try_replan("recovery timeout")

        # REPLANNING 超时 (同 planning)
        elif self._state == MissionState.REPLANNING:
            if dt_state > self.planning_timeout:
                self._set_state(MissionState.FAILED,
                                f"replan timeout ({self.planning_timeout}s)")

        # 全局任务超时
        if self._state in (MissionState.PLANNING, MissionState.EXECUTING,
                           MissionState.RECOVERING, MissionState.REPLANNING):
            if now - self._mission_start > self.mission_timeout:
                self._set_state(MissionState.FAILED,
                                f"mission timeout ({self.mission_timeout}s)")

        self._publish_status()

    # ================================================================
    # 状态发布
    # ================================================================

    def _distance_to_goal(self) -> float:
        dx = self._goal_xy[0] - self._robot_xy[0]
        dy = self._goal_xy[1] - self._robot_xy[1]
        return math.sqrt(dx * dx + dy * dy)

    def _publish_status(self):
        now = time.monotonic()
        elapsed = now - self._mission_start if self._mission_start > 0 else 0.0

        status = {
            "state": self._state.value,
            "elapsed_sec": round(elapsed, 1),
            "replan_count": self._replan_count,
            "estop": self._estop_active,
        }

        if self._goal is not None:
            status["goal"] = {
                "x": round(self._goal_xy[0], 2),
                "y": round(self._goal_xy[1], 2),
                "frame": self._goal.header.frame_id,
            }
            status["distance_to_goal"] = round(self._distance_to_goal(), 2)

        if self._wp_total > 0:
            status["progress"] = {
                "waypoints_completed": self._wp_completed,
                "waypoints_total": self._wp_total,
                "percent": round(100.0 * self._wp_completed / self._wp_total, 1),
            }

        if self._state == MissionState.FAILED:
            status["failure_reason"] = self._failure_reason

        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MissionArc()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
