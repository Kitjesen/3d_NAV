#!/usr/bin/env python3
"""
PCT A* 全局规划节点 (Python 实现，零 .so 依赖)
================================================
用途: 在没有 ARM64 编译 .so 的环境下（x86_64 开发机、CI）作为
      全局规划器备选，使用纯 Python 8-连通 A* 在预生成 tomogram 上规划路径。

话题:
  订阅  /nav/goal_pose    geometry_msgs/PoseStamped  — 导航目标
  订阅  /nav/odometry     nav_msgs/Odometry           — 当前位置
  发布  /nav/global_path  nav_msgs/Path               — A* 规划路径 (latched-like, 1Hz 重发)
  发布  /nav/planner_status  std_msgs/String          — IDLE/PLANNING/SUCCESS/FAILED

参数:
  tomogram_file  (string, 必填) — .pickle 文件路径，由 building2_9.pickle 格式生成
  obstacle_thr   (float, 49.9)  — traversability 障碍阈值
  republish_hz   (float, 1.0)   — 路径重发频率 (Hz)，保持 pct_path_adapter 稳定

与原 global_planner.py 的区别:
  - 不依赖 a_star.so / ele_planner.so (ARM64 only)
  - 仅支持 2D 平面 A*（ground-floor slice），不做 3D 爬坡规划
  - 无 TF 依赖：直接从 /nav/odometry 读当前位置
  - 适合 building2_9.pickle 等预生成 tomogram
"""
import heapq
import os
import pickle
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                        HistoryPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


# ── A* helpers ────────────────────────────────────────────────────────────────

def _load_tomogram(path: str):
    """Load building2_9.pickle format tomogram.

    Returns (trav, res, cx, cy, nx, ny, ox, oy)
    where trav[i,j] < obstacle_thr means traversable.
    """
    with open(path, 'rb') as f:
        d = pickle.load(f)
    trav = d['data'][0, 0]          # ground-floor traversability (nx, ny)
    res  = float(d['resolution'])
    cx   = float(d['center'][0])
    cy   = float(d['center'][1])
    nx, ny = trav.shape
    return trav, res, cx, cy, nx, ny, nx // 2, ny // 2


def _w2g(wx, wy, cx, cy, res, ox, oy, nx, ny):
    """World (m) → grid index (clipped)."""
    ix = int(round((wx - cx) / res)) + ox
    iy = int(round((wy - cy) / res)) + oy
    return np.clip(ix, 0, nx - 1), np.clip(iy, 0, ny - 1)


def _g2w(ix, iy, cx, cy, res, ox, oy):
    """Grid index → world (m)."""
    return (ix - ox) * res + cx, (iy - oy) * res + cy


def _astar(trav, start, goal, obs_thr):
    """8-connected A* on traversability grid.

    Returns list of (ix, iy) from start to goal, or None if unreachable.
    """
    h = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])
    open_q = [(h(start, goal), 0.0, start, [])]
    visited = {}
    while open_q:
        f, g, cur, path = heapq.heappop(open_q)
        if cur in visited:
            continue
        visited[cur] = True
        path = path + [cur]
        if cur == goal:
            return path
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ni, nj = cur[0] + dx, cur[1] + dy
            if 0 <= ni < trav.shape[0] and 0 <= nj < trav.shape[1]:
                if trav[ni, nj] < obs_thr and (ni, nj) not in visited:
                    cost = g + (1.414 if dx and dy else 1.0)
                    heapq.heappush(
                        open_q, (cost + h((ni, nj), goal), cost, (ni, nj), path))
    return None


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class PctPlannerAstar(Node):
    """Pure-Python A* global planner using a pre-built tomogram pickle."""

    def __init__(self):
        super().__init__('pct_planner_astar')
        self._cbg = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('tomogram_file', '')
        self.declare_parameter('obstacle_thr',  49.9)
        self.declare_parameter('republish_hz',  1.0)

        tomo_file    = self.get_parameter('tomogram_file').value
        self._obs    = self.get_parameter('obstacle_thr').value
        repub_hz     = self.get_parameter('republish_hz').value

        if not tomo_file or not os.path.isfile(tomo_file):
            self.get_logger().error(
                f'tomogram_file not found: "{tomo_file}"\n'
                '  Set parameter tomogram_file:=/path/to/building.pickle')
            raise RuntimeError('tomogram_file required')

        # Load tomogram
        self.get_logger().info(f'Loading tomogram: {tomo_file}')
        (self._trav, self._res, self._cx, self._cy,
         self._nx, self._ny, self._ox, self._oy) = _load_tomogram(tomo_file)
        free_count = int((self._trav < self._obs).sum())
        self.get_logger().info(
            f'Tomogram ready: {self._trav.shape}, res={self._res}m, '
            f'center=({self._cx:.3f},{self._cy:.3f}), '
            f'free={free_count} cells / {self._nx * self._ny}')

        # Robot state
        self._rx = 0.0
        self._ry = 0.0
        self._plan_lock = threading.Lock()
        self._last_path: Path | None = None

        # QoS
        be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=10)
        latched = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)

        # Publishers
        self._pub_path   = self.create_publisher(Path,   '/nav/global_path',     latched)
        self._pub_status = self.create_publisher(String, '/nav/planner_status',  10)

        # Subscribers
        self.create_subscription(
            Odometry, '/nav/odometry', self._on_odom, be,
            callback_group=self._cbg)
        self.create_subscription(
            PoseStamped, '/nav/goal_pose', self._on_goal, 10,
            callback_group=self._cbg)

        # Re-publish timer (keep pct_path_adapter fed)
        self.create_timer(1.0 / repub_hz, self._republish)

        self._publish_status('IDLE')
        self.get_logger().info(
            'PctPlannerAstar ready — waiting for /nav/goal_pose')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        self._rx = msg.pose.pose.position.x
        self._ry = msg.pose.pose.position.y

    def _on_goal(self, msg: PoseStamped):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        self.get_logger().info(f'Goal received: ({gx:.2f}, {gy:.2f})')
        if self._plan_lock.locked():
            self.get_logger().warn('Planner busy, dropping goal')
            return
        self._plan_and_publish(gx, gy)

    # ── planning ──────────────────────────────────────────────────────────────

    def _plan_and_publish(self, gx: float, gy: float):
        with self._plan_lock:
            self._publish_status('PLANNING')
            sx, sy = self._rx, self._ry
            si, sj = _w2g(sx, sy, self._cx, self._cy,
                          self._res, self._ox, self._oy, self._nx, self._ny)
            gi, gj = _w2g(gx, gy, self._cx, self._cy,
                          self._res, self._ox, self._oy, self._nx, self._ny)

            cells = _astar(self._trav, (si, sj), (gi, gj), self._obs)
            if cells is None:
                self.get_logger().warn(
                    f'A* no path ({sx:.2f},{sy:.2f})→({gx:.2f},{gy:.2f}), '
                    'using straight-line fallback')
                cells = [(si, sj), (gi, gj)]
                self._publish_status('FAILED')
            else:
                self._publish_status('SUCCESS')

            path_msg = Path()
            path_msg.header.stamp    = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            for ci, cj in cells:
                wx, wy = _g2w(ci, cj, self._cx, self._cy, self._res,
                              self._ox, self._oy)
                ps = PoseStamped()
                ps.header.frame_id = 'map'
                ps.pose.position.x = wx
                ps.pose.position.y = wy
                ps.pose.position.z = 0.0
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)

            self._last_path = path_msg
            self._pub_path.publish(path_msg)
            self.get_logger().info(
                f'A* path published: {len(cells)} cells  '
                f'({sx:.2f},{sy:.2f})→({gx:.2f},{gy:.2f})')

    def _republish(self):
        if self._last_path is not None:
            self._last_path.header.stamp = self.get_clock().now().to_msg()
            self._pub_path.publish(self._last_path)

    def _publish_status(self, status: str):
        self._pub_status.publish(String(data=status))


# ── entry ─────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = PctPlannerAstar()
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
