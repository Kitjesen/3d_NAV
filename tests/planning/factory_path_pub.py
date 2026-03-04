#!/usr/bin/env python3
"""
factory_path_pub.py — 工厂多楼层路径发布器 (SITL 演示)

替代 global_planner 为 sim_robot_node 提供 3D 全局路径:
  G0 地面层 → 楼梯斜坡 (Z: 0→13.5m) → RF 屋顶层

路径格式: nav_msgs/Path (含真实 Z), 发布到 /nav/global_path
触发条件: 收到 /nav/goal_pose 后在 DELAY 秒后发布路径
"""

import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# ── 路径参数 ────────────────────────────────────────────────────────────────
_START_X  = float(os.environ.get('SIM_START_X', '15.0'))
_START_Y  = float(os.environ.get('SIM_START_Y', '12.0'))
_GOAL_X   = float(os.environ.get('SIM_GOAL_X',  '50.0'))
_GOAL_Y   = float(os.environ.get('SIM_GOAL_Y',  '40.0'))
_GOAL_Z   = float(os.environ.get('SIM_GOAL_Z',  '13.5'))

# 楼梯位置 (X=15m, Y: 12→47m, Z: 0→13.5m)
STAIR_X      = 15.0
STAIR_Y_BASE = 12.0
STAIR_Y_TOP  = 47.0
STAIR_Z_BOT  = 0.0
STAIR_Z_TOP  = 13.5

# QoS with transient_local for path subscriber
_LATCH_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


def _make_pose(x: float, y: float, z: float, stamp) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = 'map'
    ps.header.stamp = stamp
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.w = 1.0
    return ps


def _build_factory_path(now) -> Path:
    """构建工厂多楼层 3D 路径.

    段 1 — G0 地面层: 起点 → 楼梯入口
    段 2 — 楼梯斜坡: Z=0 爬升至 Z=13.5m (35m 斜道)
    段 3 — RF 屋顶层: 楼梯顶部 → 目标点
    """
    waypoints = []

    # ── 段 1: G0 地面层 ── 从起点向楼梯入口走 ─────────────────────────────────
    # 简单直线从起点到楼梯基部
    g0_pts = [
        (_START_X, _START_Y, 0.0),
        (STAIR_X, STAIR_Y_BASE, 0.0),   # 楼梯入口
    ]
    # 如果起点不在楼梯附近, 插入过渡点
    dist_to_stair = math.hypot(_START_X - STAIR_X, _START_Y - STAIR_Y_BASE)
    if dist_to_stair > 3.0:
        # 添加中间点保持 1m 间距
        n_seg = max(2, int(dist_to_stair / 1.0))
        for i in range(n_seg + 1):
            t = i / n_seg
            wx = _START_X + t * (STAIR_X - _START_X)
            wy = _START_Y + t * (STAIR_Y_BASE - _START_Y)
            waypoints.append((wx, wy, 0.0))
    else:
        waypoints.append((_START_X, _START_Y, 0.0))
        waypoints.append((STAIR_X, STAIR_Y_BASE, 0.0))

    # ── 段 2: 楼梯斜坡 ── Z: 0 → 13.5m ───────────────────────────────────────
    stair_len = STAIR_Y_TOP - STAIR_Y_BASE   # 35m 斜道
    n_stair = int(stair_len / 0.8) + 1       # 约 0.8m 间距
    for i in range(1, n_stair + 1):
        t = i / n_stair
        wy = STAIR_Y_BASE + t * stair_len
        wz = STAIR_Z_BOT + t * (STAIR_Z_TOP - STAIR_Z_BOT)
        waypoints.append((STAIR_X, wy, wz))

    # ── 段 3: RF 屋顶层 ── 楼梯顶 → 目标点 ──────────────────────────────────
    rf_y_top = STAIR_Y_TOP
    rf_x_end = _GOAL_X
    rf_y_end = _GOAL_Y
    rf_dist  = math.hypot(rf_x_end - STAIR_X, rf_y_end - rf_y_top)
    n_rf     = max(2, int(rf_dist / 1.0))
    for i in range(1, n_rf + 1):
        t = i / n_rf
        wx = STAIR_X + t * (rf_x_end - STAIR_X)
        wy = rf_y_top + t * (rf_y_end - rf_y_top)
        waypoints.append((wx, wy, STAIR_Z_TOP))

    # ── 构建 nav_msgs/Path ──────────────────────────────────────────────────
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = now
    for (wx, wy, wz) in waypoints:
        path.poses.append(_make_pose(wx, wy, wz, now))

    return path


class FactoryPathPublisher(Node):
    def __init__(self):
        super().__init__('factory_path_pub')
        self._pub_path = self.create_publisher(Path, '/nav/global_path', _LATCH_QOS)
        self.create_subscription(
            PoseStamped, '/nav/goal_pose', self._on_goal, 10)
        self._published = False
        self.get_logger().info(
            f'FactoryPathPublisher ready. Waiting for /nav/goal_pose...\n'
            f'  Start: ({_START_X},{_START_Y})  Goal: ({_GOAL_X},{_GOAL_Y},Z={_GOAL_Z})\n'
            f'  Stair: X={STAIR_X}, Y=[{STAIR_Y_BASE},{STAIR_Y_TOP}], Z=[0,{STAIR_Z_TOP}]')

    def _on_goal(self, msg: PoseStamped):
        if self._published:
            return
        self._published = True
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gz = msg.pose.position.z
        self.get_logger().info(f'Goal received: ({gx:.2f},{gy:.2f},{gz:.2f}) — publishing 3D path in 1s...')
        # One-shot timer: publish once after 1s delay
        self._timer = self.create_timer(1.0, self._publish_path_once)

    def _publish_path_once(self):
        """Publish the 3D path exactly once, then cancel the timer."""
        self._timer.cancel()
        now = self.get_clock().now().to_msg()
        path = _build_factory_path(now)
        z_vals = [ps.pose.position.z for ps in path.poses]
        self.get_logger().info(
            f'Publishing factory path: {len(path.poses)} pts, '
            f'Z=[{min(z_vals):.1f},{max(z_vals):.1f}]m  (one-shot, latched)')
        self._pub_path.publish(path)


def main():
    rclpy.init()
    node = FactoryPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
