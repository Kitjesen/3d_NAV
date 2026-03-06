#!/usr/bin/env python3
"""
T6: Full-chain closed-loop integration test harness.

Acts as a "virtual robot" for the 6 real ROS2 nodes:
  terrainAnalysis, localPlanner, pathFollower,
  pct_planner_astar, pct_path_adapter, static_transform_publisher

This script replaces the physical robot by:
  1. Publishing /nav/map_cloud    -- synthetic flat ground point cloud (centered on robot)
  2. Publishing /nav/odometry     -- position integrated from cmd_vel (2D kinematics)
  3. Broadcasting TF odom -> body
  4. Publishing /nav/stop = 0     -- prevent pathFollower safetyStop
  5. After warmup, publishing /nav/goal_pose
  6. Monitoring /nav/cmd_vel, /nav/planner_status, /nav/adapter_status,
     /nav/global_path, /nav/terrain_map

Checks (5 pass/fail criteria):
  1. planner_status_success   -- planner publishes SUCCESS within 10s of goal
  2. adapter_path_received    -- adapter publishes path_received event
  3. cmd_vel_non_zero         -- at least 10 cmd_vel messages with |vx| > 0.01
  4. robot_moves_toward_goal  -- robot x increases toward goal
  5. goal_reached             -- adapter goal_reached event OR robot within 2m of goal

Usage:
  python3 tests/integration/test_full_chain.py \\
      --map-file /path/to/building2_9.pickle \\
      --start-x -5.5 --start-y 7.3 \\
      --goal-x 5.0 --goal-y 7.3 \\
      --timeout 90
"""

import argparse
import json
import math
import sys
import threading
import time
from collections import defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                        ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Int8, String
from tf2_ros import TransformBroadcaster


# ── Colors ────────────────────────────────────────────────────────────────────
class C:
    GREEN  = '\033[0;32m'
    RED    = '\033[0;31m'
    YELLOW = '\033[1;33m'
    BLUE   = '\033[0;34m'
    BOLD   = '\033[1m'
    NC     = '\033[0m'


# ── Constants ─────────────────────────────────────────────────────────────────
ODOM_HZ       = 20     # Odometry publish rate
CLOUD_HZ      = 5      # Point cloud publish rate
KINEMATIC_HZ  = 50     # Kinematics integration rate
WARMUP_SEC    = 8.0    # Warmup before sending goal
TERRAIN_RADIUS = 10.0  # Flat terrain extent (m)
TERRAIN_STEP   = 0.4   # Terrain grid step (m)


def _make_flat_cloud(cx: float, cy: float, stamp) -> PointCloud2:
    """Generate a flat XYZI point cloud centered on (cx, cy) in odom frame.

    This provides clean terrain data so terrainAnalysis produces a traversable
    terrain_map without near-field E-stop triggers.
    """
    r = int(TERRAIN_RADIUS / TERRAIN_STEP)
    pts = []
    for ix in range(-r, r + 1):
        for iy in range(-r, r + 1):
            pts.append([cx + ix * TERRAIN_STEP, cy + iy * TERRAIN_STEP, 0.0, 0.0])
    arr = np.array(pts, dtype=np.float32)

    msg = PointCloud2()
    msg.header.frame_id = 'odom'
    if stamp is not None:
        msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(arr)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.fields = [
        PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step   = 16 * len(arr)
    msg.data       = arr.tobytes()
    return msg


class FullChainTestNode(Node):
    """Virtual robot node for full-chain closed-loop test."""

    def __init__(self, start_x, start_y, goal_x, goal_y, timeout_sec):
        super().__init__('full_chain_test_node')

        # ── Robot state ──
        self.x   = start_x
        self.y   = start_y
        self.yaw = 0.0
        self.vx  = 0.0
        self.wz  = 0.0

        self.start_x = start_x
        self.start_y = start_y
        self.goal_x  = goal_x
        self.goal_y  = goal_y
        self.timeout_sec = timeout_sec

        # ── Monitoring state ──
        self._lock = threading.Lock()
        self.cmd_vel_msgs     = []      # (vx, wz, timestamp)
        self.planner_statuses = []      # string messages
        self.adapter_events   = []      # parsed JSON events
        self.global_path_count = 0
        self.terrain_map_count = 0
        self.position_history  = []     # (t, x, y)

        # ── Check results ──
        self.checks = {
            'planner_status_success': False,
            'adapter_path_received':  False,
            'cmd_vel_non_zero':       False,
            'robot_moves_toward_goal': False,
            'goal_reached':           False,
        }

        self.t_start   = None
        self.t_goal_sent = None
        self.goal_sent = False
        self.finished  = False

        # ── TF ──
        self.tf_br = TransformBroadcaster(self)

        # ── Publishers ──
        self.pub_odom    = self.create_publisher(Odometry,    '/nav/odometry',       10)
        self.pub_cloud   = self.create_publisher(PointCloud2, '/nav/map_cloud',      10)
        self.pub_terrain = self.create_publisher(PointCloud2, '/nav/terrain_map',    10)
        self.pub_te_ext  = self.create_publisher(PointCloud2, '/nav/terrain_map_ext', 10)
        self.pub_goal    = self.create_publisher(PoseStamped, '/nav/goal_pose',      10)
        self.pub_stop    = self.create_publisher(Int8,        '/nav/stop',           10)
        self.pub_speed   = self.create_publisher(Float32,     '/nav/speed',          10)

        # ── Subscribers ──
        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.create_subscription(
            TwistStamped, '/nav/cmd_vel', self._on_cmd_vel, be_qos)
        self.create_subscription(
            String, '/nav/planner_status', self._on_planner_status, 10)
        self.create_subscription(
            String, '/nav/adapter_status', self._on_adapter_status, 10)
        self.create_subscription(
            Path, '/nav/global_path', self._on_global_path, latched_qos)
        self.create_subscription(
            PointCloud2, '/nav/terrain_map', self._on_terrain_map, 10)

        # ── Timers ──
        # Kinematics integration at 50 Hz
        self.create_timer(1.0 / KINEMATIC_HZ, self._tick_kinematics)
        # Odometry + TF at 20 Hz
        self.create_timer(1.0 / ODOM_HZ, self._tick_odom)
        # Point cloud at 5 Hz
        self.create_timer(1.0 / CLOUD_HZ, self._tick_cloud)
        # Stop + speed at 2 Hz
        self.create_timer(0.5, self._tick_stop)
        # Main check loop at 2 Hz
        self.create_timer(0.5, self._tick_check)

        self.t_start = time.time()
        self.get_logger().info(
            f'[T6] Test harness started. '
            f'Start=({start_x:.1f},{start_y:.1f}) '
            f'Goal=({goal_x:.1f},{goal_y:.1f}) '
            f'Timeout={timeout_sec}s')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: TwistStamped):
        vx = msg.twist.linear.x
        wz = msg.twist.angular.z
        with self._lock:
            self.vx = vx
            self.wz = wz
            self.cmd_vel_msgs.append((vx, wz, time.time()))

    def _on_planner_status(self, msg: String):
        status = msg.data.strip()
        with self._lock:
            self.planner_statuses.append(status)
        if status == 'SUCCESS':
            self.checks['planner_status_success'] = True
            self.get_logger().info(f'[T6] Planner status: {status}')
        elif status == 'GOAL_REACHED':
            self.checks['goal_reached'] = True
            self.get_logger().info(f'[T6] GOAL_REACHED from pathFollower')

    def _on_adapter_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            event = data.get('event', '')
            with self._lock:
                self.adapter_events.append(data)
            if event == 'path_received':
                self.checks['adapter_path_received'] = True
                self.get_logger().info(f'[T6] Adapter: path_received')
            elif event == 'goal_reached':
                self.checks['goal_reached'] = True
                self.get_logger().info(f'[T6] Adapter: goal_reached')
            elif event == 'waypoint_reached':
                idx = data.get('index', '?')
                total = data.get('total', '?')
                self.get_logger().info(
                    f'[T6] Adapter: waypoint_reached {idx}/{total}')
        except (json.JSONDecodeError, AttributeError):
            pass

    def _on_global_path(self, msg: Path):
        if msg.poses:
            with self._lock:
                self.global_path_count += 1
            self.get_logger().info(
                f'[T6] Global path received: {len(msg.poses)} poses')

    def _on_terrain_map(self, msg: PointCloud2):
        with self._lock:
            self.terrain_map_count += 1

    # ── Kinematics integration (50 Hz) ────────────────────────────────────────

    def _tick_kinematics(self):
        if self.finished:
            return
        dt = 1.0 / KINEMATIC_HZ
        with self._lock:
            vx = self.vx
            wz = self.wz
        self.yaw += wz * dt
        self.yaw = (self.yaw + math.pi) % (2 * math.pi) - math.pi
        self.x += vx * math.cos(self.yaw) * dt
        self.y += vx * math.sin(self.yaw) * dt

    # ── Odometry + TF (20 Hz) ────────────────────────────────────────────────

    def _tick_odom(self):
        if self.finished:
            return
        now = self.get_clock().now().to_msg()

        # Broadcast odom -> body TF
        tf = TransformStamped()
        tf.header.stamp    = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id  = 'body'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z    = math.sin(self.yaw / 2)
        tf.transform.rotation.w    = math.cos(self.yaw / 2)
        self.tf_br.sendTransform(tf)

        # Publish odometry
        od = Odometry()
        od.header.stamp    = now
        od.header.frame_id = 'odom'
        od.child_frame_id  = 'body'
        od.pose.pose.position.x    = self.x
        od.pose.pose.position.y    = self.y
        od.pose.pose.position.z    = 0.0
        od.pose.pose.orientation.z = math.sin(self.yaw / 2)
        od.pose.pose.orientation.w = math.cos(self.yaw / 2)
        with self._lock:
            od.twist.twist.linear.x  = self.vx
            od.twist.twist.angular.z = self.wz
        self.pub_odom.publish(od)

    # ── Point cloud (5 Hz) ───────────────────────────────────────────────────

    def _tick_cloud(self):
        if self.finished:
            return
        now = self.get_clock().now().to_msg()
        cloud = _make_flat_cloud(self.x, self.y, stamp=now)
        self.pub_cloud.publish(cloud)
        # Also publish as terrain_map_ext so localPlanner has both inputs
        self.pub_te_ext.publish(cloud)

    # ── Stop + Speed (2 Hz) ──────────────────────────────────────────────────

    def _tick_stop(self):
        if self.finished:
            return
        # Continuously publish stop=0 to prevent pathFollower safetyStop
        stop_msg = Int8()
        stop_msg.data = 0
        self.pub_stop.publish(stop_msg)

        # Publish speed factor (1.0 = normal)
        speed_msg = Float32()
        speed_msg.data = 1.0
        self.pub_speed.publish(speed_msg)

    # ── Main check loop (2 Hz) ──────────────────────────────────────────────

    def _tick_check(self):
        if self.finished:
            return

        elapsed = time.time() - self.t_start

        # Record position history
        self.position_history.append((elapsed, self.x, self.y))

        # ── Warmup phase: send goal after WARMUP_SEC ──
        if not self.goal_sent and elapsed >= WARMUP_SEC:
            self.goal_sent = True
            self.t_goal_sent = time.time()
            now = self.get_clock().now().to_msg()
            msg = PoseStamped()
            msg.header.stamp    = now
            msg.header.frame_id = 'map'
            msg.pose.position.x = self.goal_x
            msg.pose.position.y = self.goal_y
            msg.pose.position.z = 0.0
            msg.pose.orientation.w = 1.0
            self.pub_goal.publish(msg)
            self.get_logger().info(
                f'[T6] Goal sent: ({self.goal_x:.1f}, {self.goal_y:.1f})')

        # ── Running phase: check conditions ──
        if self.goal_sent:
            # Check cmd_vel_non_zero: at least 10 messages with |vx| > 0.01
            with self._lock:
                nonzero_count = sum(
                    1 for vx, wz, t in self.cmd_vel_msgs
                    if abs(vx) > 0.01)
            if nonzero_count >= 10:
                self.checks['cmd_vel_non_zero'] = True

            # Check robot_moves_toward_goal: compare x now vs x at goal_sent
            if len(self.position_history) > 20:
                # Find position near when goal was sent
                early_positions = [
                    (t, x, y) for t, x, y in self.position_history
                    if abs(t - WARMUP_SEC) < 2.0
                ]
                if early_positions:
                    _, x_early, y_early = early_positions[0]
                    dist_early = math.hypot(
                        self.goal_x - x_early, self.goal_y - y_early)
                    dist_now = math.hypot(
                        self.goal_x - self.x, self.goal_y - self.y)
                    if dist_now < dist_early - 0.5:
                        self.checks['robot_moves_toward_goal'] = True

            # Check proximity-based goal reached
            dist_to_goal = math.hypot(
                self.x - self.goal_x, self.y - self.goal_y)
            if dist_to_goal < 2.0:
                self.checks['goal_reached'] = True
                self.get_logger().info(
                    f'[T6] Proximity goal reached: dist={dist_to_goal:.2f}m')

            # Periodic status log (every ~5s)
            if int(elapsed * 2) % 10 == 0:
                with self._lock:
                    n_cmd = len(self.cmd_vel_msgs)
                self.get_logger().info(
                    f'[T6] t={elapsed:.0f}s  '
                    f'pos=({self.x:.2f},{self.y:.2f})  '
                    f'dist={dist_to_goal:.2f}m  '
                    f'cmds={n_cmd}  '
                    f'terrain={self.terrain_map_count}  '
                    f'paths={self.global_path_count}')

        # ── Termination conditions ──
        all_passed = all(self.checks.values())
        timed_out = elapsed > self.timeout_sec

        if all_passed or timed_out:
            self.finished = True
            if timed_out and not all_passed:
                self.get_logger().warn(
                    f'[T6] Timeout after {elapsed:.0f}s')
            else:
                self.get_logger().info(
                    f'[T6] All checks passed in {elapsed:.0f}s')


def run_test(args):
    """Run the full-chain test and return exit code."""
    rclpy.init()

    node = FullChainTestNode(
        start_x=args.start_x,
        start_y=args.start_y,
        goal_x=args.goal_x,
        goal_y=args.goal_y,
        timeout_sec=args.timeout,
    )

    # Use a multi-threaded executor so timers fire concurrently
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        while rclpy.ok() and not node.finished:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warn('[T6] Interrupted')
        node.finished = True
    finally:
        executor.shutdown()

    # ── Evaluate results ──────────────────────────────────────────────────────
    checks = node.checks
    elapsed = time.time() - node.t_start

    print('')
    print('=' * 60)
    print(f'{C.BOLD}  T6 Full-chain Test Results{C.NC}')
    print('=' * 60)

    passed = 0
    failed = 0
    for name, ok in checks.items():
        if ok:
            print(f'  {C.GREEN}PASS{C.NC}  {name}')
            passed += 1
        else:
            print(f'  {C.RED}FAIL{C.NC}  {name}')
            failed += 1

    print(f'')
    print(f'  Elapsed:       {elapsed:.1f}s')
    print(f'  Final pos:     ({node.x:.2f}, {node.y:.2f})')
    final_dist = math.hypot(node.x - node.goal_x, node.y - node.goal_y)
    print(f'  Dist to goal:  {final_dist:.2f}m')
    with node._lock:
        n_cmd = len(node.cmd_vel_msgs)
        n_events = len(node.adapter_events)
        n_status = len(node.planner_statuses)
    print(f'  cmd_vel msgs:  {n_cmd}')
    print(f'  Adapter events: {n_events}')
    print(f'  Planner status msgs: {n_status}')
    print(f'  Terrain maps:  {node.terrain_map_count}')
    print(f'  Global paths:  {node.global_path_count}')
    print('=' * 60)

    if failed == 0:
        print(f'{C.GREEN}T6 PASSED: {passed}/{passed + failed} checks{C.NC}')
    else:
        print(f'{C.RED}T6 FAILED: {failed}/{passed + failed} checks failed{C.NC}')
    print('')

    # ── Save JSON result ──────────────────────────────────────────────────────
    result = {
        'test': 'T6_full_chain',
        'passed': failed == 0,
        'checks': checks,
        'elapsed_sec': round(elapsed, 1),
        'final_pos': [round(node.x, 3), round(node.y, 3)],
        'final_dist_to_goal': round(final_dist, 3),
        'start': [node.start_x, node.start_y],
        'goal': [node.goal_x, node.goal_y],
        'cmd_vel_count': n_cmd,
        'adapter_event_count': n_events,
        'terrain_map_count': node.terrain_map_count,
        'global_path_count': node.global_path_count,
    }
    result_path = '/tmp/t6_result.json'
    try:
        with open(result_path, 'w') as f:
            json.dump(result, f, indent=2)
        print(f'Result saved: {result_path}')
    except Exception as e:
        print(f'Warning: could not save result: {e}')

    node.destroy_node()
    rclpy.shutdown()

    return 0 if failed == 0 else 1


def main():
    parser = argparse.ArgumentParser(
        description='T6 Full-chain closed-loop integration test')
    parser.add_argument('--map-file', type=str, default='',
                        help='Path to tomogram pickle (for reference only)')
    parser.add_argument('--start-x', type=float, default=-5.5,
                        help='Robot start X (m)')
    parser.add_argument('--start-y', type=float, default=7.3,
                        help='Robot start Y (m)')
    parser.add_argument('--goal-x', type=float, default=5.0,
                        help='Navigation goal X (m)')
    parser.add_argument('--goal-y', type=float, default=7.3,
                        help='Navigation goal Y (m)')
    parser.add_argument('--timeout', type=float, default=90.0,
                        help='Test timeout (seconds)')
    args = parser.parse_args()

    sys.exit(run_test(args))


if __name__ == '__main__':
    main()
