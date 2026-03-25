#!/usr/bin/env python3
"""
Ring 2 — Evaluator (认知环闭环评估器)
======================================
对比执行效果与规划预期，检测异常并触发自适应。
这是把流水线变成闭环的关键组件。

核心指标:
  1. cross_track_error — 机器人偏离规划路径的距离 (m)
  2. progress_rate     — 向目标接近的速率 (m/s, 负=远离)
  3. heading_error     — 航向偏差 (rad)
  4. stall_time        — 无进展时长 (s)

评估:
  ON_TRACK   — 正常跟踪, 偏差在容忍范围内
  DRIFTING   — 跨径偏差大, 但仍在前进
  STALLED    — 停滞, 无进展
  REGRESSING — 远离目标

话题:
  订阅:
    /nav/odometry       (Odometry)     — 机器人位置
    /nav/global_path    (Path)         — 当前规划路径
    /nav/way_point      (PointStamped) — 当前航点目标
    /nav/cmd_vel        (TwistStamped) — 指令速度

  发布:
    /nav/execution_eval (String JSON)  — 评估结果
"""

import math
import time
from enum import Enum
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String

from semantic.common.semantic_common.sanitize import safe_json_dumps, sanitize_float


class Assessment(Enum):
    IDLE = "IDLE"               # 无任务
    ON_TRACK = "ON_TRACK"       # 正常跟踪
    DRIFTING = "DRIFTING"       # 偏离路径但前进
    STALLED = "STALLED"         # 停滞
    REGRESSING = "REGRESSING"   # 远离目标


class Evaluator(Node):
    """Ring 2: 认知环闭环评估器。"""

    def __init__(self):
        super().__init__('evaluator')

        # ── 参数 ──
        self.declare_parameter('eval_hz', 5.0)
        self.declare_parameter('cross_track_warn', 1.5)     # m, 跨径偏差警告阈值
        self.declare_parameter('cross_track_danger', 3.0)    # m, 跨径偏差危险阈值
        self.declare_parameter('stall_threshold', 0.05)      # m/s, 低于此视为停滞
        self.declare_parameter('progress_window_sec', 3.0)   # 进度评估滑窗
        self.declare_parameter('min_progress_rate', -0.02)   # m/s, 低于此视为倒退

        hz = self.get_parameter('eval_hz').value
        self._ct_warn = self.get_parameter('cross_track_warn').value
        self._ct_danger = self.get_parameter('cross_track_danger').value
        self._stall_thr = self.get_parameter('stall_threshold').value
        self._progress_window = self.get_parameter('progress_window_sec').value
        self._min_progress = self.get_parameter('min_progress_rate').value

        # ── 状态 ──
        self._robot_xy = np.zeros(2)
        self._robot_yaw = 0.0
        self._path_points: Optional[np.ndarray] = None   # (N, 2) 规划路径点
        self._goal_xy: Optional[np.ndarray] = None        # 最终目标
        self._waypoint_xy: Optional[np.ndarray] = None    # 当前航点

        self._cmd_speed = 0.0
        self._actual_speed = 0.0

        # 进度历史: [(timestamp, distance_to_goal)]
        self._progress_history: List[Tuple[float, float]] = []

        # 上一次有意义进展的时间
        self._last_progress_time = time.monotonic()
        self._last_distance = float('inf')

        # ── QoS ──
        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        # ── 订阅 ──
        self.create_subscription(Odometry, '/nav/odometry', self._on_odom, be)
        self.create_subscription(Path, '/nav/global_path', self._on_path, latched)
        self.create_subscription(PointStamped, '/nav/way_point', self._on_waypoint, 10)
        self.create_subscription(TwistStamped, '/nav/cmd_vel', self._on_cmdvel, be)

        # ── 发布 ──
        self._pub_eval = self.create_publisher(String, '/nav/execution_eval', 10)

        # ── 定时器 ──
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(f'Evaluator started @ {hz}Hz')

    # ================================================================
    # 回调
    # ================================================================

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not (math.isfinite(x) and math.isfinite(y)):
            return
        self._robot_xy = np.array([x, y])

        # 从四元数提取 yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

        # 实际速度
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        if math.isfinite(vx) and math.isfinite(vy):
            self._actual_speed = math.hypot(vx, vy)

    def _on_path(self, msg: Path):
        if len(msg.poses) < 2:
            self._path_points = None
            self._goal_xy = None
            return
        pts = []
        for ps in msg.poses:
            px = ps.pose.position.x
            py = ps.pose.position.y
            if math.isfinite(px) and math.isfinite(py):
                pts.append([px, py])
        if len(pts) >= 2:
            self._path_points = np.array(pts)
            self._goal_xy = self._path_points[-1].copy()
            # 重置进度追踪
            self._progress_history.clear()
            self._last_progress_time = time.monotonic()
            self._last_distance = float('inf')
            self.get_logger().info(
                f'Evaluator: new path ({len(pts)} pts), '
                f'goal=({self._goal_xy[0]:.1f}, {self._goal_xy[1]:.1f})')

    def _on_waypoint(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y
        if math.isfinite(x) and math.isfinite(y):
            self._waypoint_xy = np.array([x, y])

    def _on_cmdvel(self, msg: TwistStamped):
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        if math.isfinite(vx) and math.isfinite(vy):
            self._cmd_speed = math.hypot(vx, vy)

    # ================================================================
    # 评估逻辑
    # ================================================================

    def _cross_track_error(self) -> float:
        """机器人到规划路径最近线段的距离。"""
        if self._path_points is None or len(self._path_points) < 2:
            return 0.0
        # 向量化: 计算到每条线段的距离, 取最小值
        p = self._robot_xy
        a = self._path_points[:-1]   # (N-1, 2)
        b = self._path_points[1:]    # (N-1, 2)
        ab = b - a
        ap = p - a
        # 参数 t = dot(ap, ab) / dot(ab, ab), clamped to [0, 1]
        ab_sq = np.sum(ab * ab, axis=1)
        ab_sq = np.where(ab_sq < 1e-10, 1.0, ab_sq)  # 防零除
        t = np.clip(np.sum(ap * ab, axis=1) / ab_sq, 0.0, 1.0)
        # 最近点
        proj = a + t[:, np.newaxis] * ab
        dists = np.linalg.norm(p - proj, axis=1)
        return float(np.min(dists))

    def _distance_to_goal(self) -> float:
        if self._goal_xy is None:
            return float('inf')
        return float(np.linalg.norm(self._robot_xy - self._goal_xy))

    def _heading_error(self) -> float:
        """机器人航向与指向航点方向的偏差 (rad)。"""
        target = self._waypoint_xy if self._waypoint_xy is not None else self._goal_xy
        if target is None:
            return 0.0
        dx = target[0] - self._robot_xy[0]
        dy = target[1] - self._robot_xy[1]
        if abs(dx) < 0.1 and abs(dy) < 0.1:
            return 0.0
        desired_yaw = math.atan2(dy, dx)
        err = desired_yaw - self._robot_yaw
        # 归一化到 [-pi, pi]
        while err > math.pi:
            err -= 2 * math.pi
        while err < -math.pi:
            err += 2 * math.pi
        return err

    def _compute_progress_rate(self) -> float:
        """滑窗内目标距离变化率 (m/s, 负值=接近目标)。"""
        now = time.monotonic()
        dist = self._distance_to_goal()

        self._progress_history.append((now, dist))
        # 清理窗口外的记录
        cutoff = now - self._progress_window
        self._progress_history = [
            (t, d) for t, d in self._progress_history if t > cutoff]

        if len(self._progress_history) < 2:
            return 0.0

        t0, d0 = self._progress_history[0]
        t1, d1 = self._progress_history[-1]
        dt = t1 - t0
        if dt < 0.1:
            return 0.0
        return (d1 - d0) / dt  # 负值 = 接近目标

    def _assess(self, cte: float, progress_rate: float,
                stall_time: float) -> Assessment:
        """综合评估当前执行状态。"""
        if self._path_points is None:
            return Assessment.IDLE

        # 远离目标
        if progress_rate > abs(self._min_progress) and stall_time > 3.0:
            return Assessment.REGRESSING

        # 停滞
        if stall_time > self._progress_window:
            return Assessment.STALLED

        # 偏离
        if cte > self._ct_warn:
            return Assessment.DRIFTING

        return Assessment.ON_TRACK

    # ================================================================
    # 主循环
    # ================================================================

    def _tick(self):
        if self._path_points is None:
            # 无任务, 发 IDLE
            msg = String()
            msg.data = safe_json_dumps({'assessment': Assessment.IDLE.value})
            self._pub_eval.publish(msg)
            return

        now = time.monotonic()

        # 计算指标
        cte = self._cross_track_error()
        dist = self._distance_to_goal()
        heading_err = self._heading_error()
        progress_rate = self._compute_progress_rate()

        # 进展追踪: 距离有意义缩短 → 更新时间戳
        if dist < self._last_distance - 0.1:
            self._last_progress_time = now
            self._last_distance = dist
        stall_time = now - self._last_progress_time

        # 评估
        assessment = self._assess(cte, progress_rate, stall_time)

        # 速度效率
        vel_eff = (self._actual_speed / self._cmd_speed
                   if self._cmd_speed > 0.05 else 1.0)

        # 发布
        result = {
            'assessment': assessment.value,
            'cross_track_error': round(sanitize_float(cte), 2),
            'distance_to_goal': round(sanitize_float(dist), 1),
            'progress_rate': round(sanitize_float(progress_rate), 3),
            'heading_error_deg': round(math.degrees(heading_err), 1),
            'stall_time': round(stall_time, 1),
            'velocity_efficiency': round(sanitize_float(min(vel_eff, 2.0)), 2),
        }

        msg = String()
        msg.data = safe_json_dumps(result)
        self._pub_eval.publish(msg)

        # 状态变化日志
        prev = getattr(self, '_prev_assessment', Assessment.IDLE)
        if assessment != prev:
            self._prev_assessment = assessment
            if assessment in (Assessment.STALLED, Assessment.REGRESSING):
                self.get_logger().warn(
                    f'Eval: {assessment.value} (cte={cte:.2f}m, '
                    f'stall={stall_time:.1f}s, rate={progress_rate:.3f}m/s)')
            elif assessment == Assessment.DRIFTING:
                self.get_logger().info(
                    f'Eval: DRIFTING (cte={cte:.2f}m)')
            else:
                self.get_logger().info(f'Eval: {assessment.value}')


def main(args=None):
    rclpy.init(args=args)
    node = Evaluator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
