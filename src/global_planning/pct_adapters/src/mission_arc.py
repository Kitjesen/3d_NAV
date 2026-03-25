#!/usr/bin/env python3
"""
MissionArc — 任务生命周期状态机

将全局规划、航点推进、卡死恢复、安全降级串成统一的 mission 级别 FSM。
综合 /nav/planner_status + /nav/adapter_status + /nav/odometry + /nav/stop
输出一致的 /nav/mission_status 供 App / gRPC / 日志系统消费。

支持两种模式:
  1. 单目标模式: 收到 /nav/goal_pose → 导航到目标 → COMPLETE
  2. 巡检模式:   收到 /nav/patrol_goals → 依次导航到每个航点 → 循环或完成

状态流:
  IDLE ──goal_pose──→ PLANNING ──success──→ EXECUTING ──goal_reached──→ COMPLETE
                         │                     │                          │
                       failed               stuck/warn              (巡检模式)
                         │                     │                     下一航点
                         ↓                     ↓                       │
                       FAILED              RECOVERING                  ↓
                                               │                   PLANNING
                                            cleared/timeout
                                               │
                                            REPLANNING

  任何状态 ──/nav/cancel──→ IDLE (清空所有状态)

话题:
  订阅:
    /nav/goal_pose        (PoseStamped)  — 单目标触发
    /nav/patrol_goals     (String JSON)  — 巡检模式: {waypoints, loop, route_name}
    /nav/cancel           (Empty)        — 取消当前任务
    /nav/planner_status   (String)       — 全局规划器状态
    /nav/adapter_status   (String JSON)  — 航点跟踪事件
    /nav/odometry         (Odometry)     — 机器人位置
    /nav/stop             (Int8)         — 安全信号 (0=clear, 2=stop)

  发布:
    /nav/mission_status   (String JSON)  — 任务状态 + 进度
    /nav/goal_pose        (PoseStamped)  — 向规划器发送当前目标
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Int8, Empty

import json
import math
import time
from enum import Enum
from typing import List, Optional

from semantic.common.semantic_common.sanitize import safe_json_dumps, safe_json_loads


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

        # ── 巡检模式状态 ──
        self._patrol_waypoints: List[dict] = []   # [{x, y, z, name}, ...]
        self._patrol_index = 0                     # 当前巡检航点索引
        self._patrol_loop = False                  # 是否循环
        self._patrol_name = ""                     # 路线名称
        self._patrol_laps = 0                      # 已完成圈数

        # ── QoS ──
        qos_goal = QoSProfile(depth=5,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ── 订阅 ──
        self.create_subscription(
            PoseStamped, '/nav/goal_pose', self._on_goal, qos_goal)
        self.create_subscription(
            String, '/nav/patrol_goals', self._on_patrol_goals, 10)
        self.create_subscription(
            Empty, '/nav/cancel', self._on_cancel, 10)
        self.create_subscription(
            String, '/nav/planner_status', self._on_planner_status, 10)
        self.create_subscription(
            String, '/nav/adapter_status', self._on_adapter_status, 10)
        self.create_subscription(
            Odometry, '/nav/odometry', self._on_odom, 10)
        self.create_subscription(
            Int8, '/nav/stop', self._on_stop, 10)

        # ── 发布 ──
        self._status_pub = self.create_publisher(String, '/nav/mission_status', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/nav/goal_pose', qos_goal)

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

        # 巡检模式: COMPLETE 一个航点后推进到下一个
        if new_state == MissionState.COMPLETE and self._patrol_waypoints:
            self._publish_status()  # 先发一次当前航点完成
            if self._advance_patrol():
                return  # 已推进到下一航点, 不停留在 COMPLETE

        if new_state in (MissionState.COMPLETE, MissionState.FAILED):
            self._publish_status()  # 立即发一次终态

    # ================================================================
    # 回调
    # ================================================================

    def _on_goal(self, msg: PoseStamped):
        """新单目标 → 启动 mission (清除巡检模式)"""
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        if not (math.isfinite(gx) and math.isfinite(gy)):
            self.get_logger().warn(f'Rejecting NaN/Inf goal: ({gx}, {gy})')
            return

        if self._state not in _ACCEPT_GOAL_STATES and self._state != MissionState.EXECUTING:
            if self._state in (MissionState.PLANNING, MissionState.RECOVERING,
                               MissionState.REPLANNING):
                self.get_logger().warn(
                    f'Goal received during {self._state.value}, overriding')

        # 单目标模式: 清除巡检状态
        self._patrol_waypoints = []
        self._patrol_index = 0
        self._patrol_loop = False
        self._patrol_name = ""
        self._patrol_laps = 0

        self._goal = msg
        self._goal_xy = (gx, gy)
        self._replan_count = 0
        self._wp_completed = 0
        self._wp_total = 0
        self._failure_reason = ""
        self._mission_start = time.monotonic()
        self._set_state(MissionState.PLANNING, "new goal received")

    def _on_patrol_goals(self, msg: String):
        """巡检路线 → 启动多航点任务"""
        data = safe_json_loads(msg.data)
        if not data:
            return

        waypoints = data.get('waypoints', [])
        if not waypoints:
            self.get_logger().warn('Patrol: empty waypoints, ignoring')
            return

        # 校验航点
        valid = []
        for wp in waypoints:
            x = wp.get('x', 0.0)
            y = wp.get('y', 0.0)
            if math.isfinite(x) and math.isfinite(y):
                valid.append({
                    'x': x, 'y': y,
                    'z': wp.get('z', 0.0),
                    'name': wp.get('name', f'WP{len(valid)}'),
                })
        if not valid:
            self.get_logger().warn('Patrol: no valid waypoints')
            return

        self._patrol_waypoints = valid
        self._patrol_loop = data.get('loop', False)
        self._patrol_name = data.get('route_name', '')
        self._patrol_index = 0
        self._patrol_laps = 0
        self._replan_count = 0
        self._failure_reason = ""
        self._mission_start = time.monotonic()

        self.get_logger().info(
            f'Patrol started: "{self._patrol_name}" '
            f'{len(valid)} waypoints, loop={self._patrol_loop}')

        self._send_patrol_goal()

    def _on_cancel(self, msg: Empty):
        """取消当前任务 → 回到 IDLE"""
        if self._state == MissionState.IDLE:
            return
        old_state = self._state
        self._patrol_waypoints = []
        self._patrol_index = 0
        self._patrol_loop = False
        self._patrol_name = ""
        self._patrol_laps = 0
        self._set_state(MissionState.IDLE, f"cancelled from {old_state.value}")
        self.get_logger().info('Mission cancelled by user')

    def _send_patrol_goal(self):
        """发送当前巡检航点到规划器"""
        if self._patrol_index >= len(self._patrol_waypoints):
            return

        wp = self._patrol_waypoints[self._patrol_index]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = float(wp['x'])
        goal.pose.position.y = float(wp['y'])
        goal.pose.position.z = float(wp.get('z', 0.0))
        goal.pose.orientation.w = 1.0

        self._goal = goal
        self._goal_xy = (wp['x'], wp['y'])
        self._wp_completed = 0
        self._wp_total = 0
        self._replan_count = 0

        self._goal_pub.publish(goal)
        self._set_state(
            MissionState.PLANNING,
            f'patrol waypoint {self._patrol_index + 1}/{len(self._patrol_waypoints)} '
            f'"{wp.get("name", "")}" ({wp["x"]:.1f}, {wp["y"]:.1f})')

    def _advance_patrol(self):
        """当前航点完成 → 推进到下一个或结束"""
        if not self._patrol_waypoints:
            return False  # 不在巡检模式

        self._patrol_index += 1

        if self._patrol_index >= len(self._patrol_waypoints):
            self._patrol_laps += 1
            if self._patrol_loop:
                self._patrol_index = 0
                self.get_logger().info(
                    f'Patrol lap {self._patrol_laps} complete, restarting')
                self._send_patrol_goal()
                return True
            else:
                self.get_logger().info(
                    f'Patrol complete: {len(self._patrol_waypoints)} waypoints')
                return False  # 全部完成
        else:
            wp = self._patrol_waypoints[self._patrol_index]
            self.get_logger().info(
                f'Advancing to waypoint {self._patrol_index + 1}/'
                f'{len(self._patrol_waypoints)} "{wp.get("name", "")}"')
            self._send_patrol_goal()
            return True

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
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.isfinite(x) and math.isfinite(y):
            self._robot_xy = (x, y)

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

        # 巡检模式额外信息
        if self._patrol_waypoints:
            patrol_total = len(self._patrol_waypoints)
            patrol_pct = round(100.0 * self._patrol_index / patrol_total, 1)
            current_wp = (self._patrol_waypoints[self._patrol_index]
                          if self._patrol_index < patrol_total else None)
            status["patrol"] = {
                "route_name": self._patrol_name,
                "current_index": self._patrol_index,
                "total_waypoints": patrol_total,
                "percent": patrol_pct,
                "loop": self._patrol_loop,
                "laps_completed": self._patrol_laps,
                "current_waypoint_name": current_wp.get('name', '') if current_wp else None,
            }

        if self._state == MissionState.FAILED:
            status["failure_reason"] = self._failure_reason

        msg = String()
        msg.data = safe_json_dumps(status)
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
