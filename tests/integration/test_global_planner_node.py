#!/usr/bin/env python3
"""
全局规划器节点集成测试 (T4)

验证 pct_planner_astar.py 接收 goal_pose 后输出 global_path。

前置启动:
  source /opt/ros/humble/setup.bash
  source ~/lingtu/install/setup.bash

  # Static TF
  ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom &

  # Python A* planner (不需要 ele_planner.so)
  PCT_SHARE=$(ros2 pkg prefix pct_planner)/share/pct_planner
  python3 ${PCT_SHARE}/planner/scripts/pct_planner_astar.py --ros-args \
      -r /goal_pose:=/nav/goal_pose \
      -r /pct_path:=/nav/global_path \
      -r /pct_planner/status:=/nav/planner_status \
      -p map_file:=/tmp/sim_maps/building_nav.pickle

运行本测试:
  python3 tests/integration/test_global_planner_node.py
"""

import json
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


# Goal and start positions (must be reachable in building_nav tomogram)
START_X, START_Y = 2.0, 2.0
GOAL_X, GOAL_Y = 18.0, 9.0

# QoS for global_path (TRANSIENT_LOCAL to catch latched)
PATH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class GlobalPlannerTestNode(Node):
    def __init__(self):
        super().__init__('global_planner_test')
        self._lock = threading.Lock()

        # Publishers
        self.pub_odom = self.create_publisher(Odometry, '/nav/odometry', 10)
        self.pub_goal = self.create_publisher(PoseStamped, '/nav/goal_pose', 10)

        # Subscribers
        self.global_paths = []
        self.planner_statuses = []

        self.create_subscription(
            Path, '/nav/global_path', self._path_cb, PATH_QOS)
        self.create_subscription(
            String, '/nav/planner_status', self._status_cb, 10)

        # Odom timer
        self.create_timer(0.05, self._pub_odom_tick)

    def _path_cb(self, msg):
        with self._lock:
            self.global_paths.append(msg)
        n = len(msg.poses)
        self.get_logger().info(f'Received global_path: {n} poses')

    def _status_cb(self, msg):
        with self._lock:
            self.planner_statuses.append(msg.data)
        self.get_logger().info(f'planner_status: {msg.data}')

    def _pub_odom_tick(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'body'
        msg.pose.pose.position.x = START_X
        msg.pose.pose.position.y = START_Y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(msg)

    def send_goal(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = GOAL_X
        msg.pose.position.y = GOAL_Y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.pub_goal.publish(msg)
        self.get_logger().info(f'Sent goal_pose: ({GOAL_X}, {GOAL_Y})')


def main():
    print('=' * 60)
    print('  Global Planner (pct_planner_astar) Integration Test (T4)')
    print('=' * 60)

    rclpy.init()
    node = GlobalPlannerTestNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    results = {}

    # Warmup — let odom publish so planner knows robot position
    print('\n[Warmup] Publishing odom for 3s...')
    time.sleep(3.0)

    # Send goal
    print(f'\n[Test] Sending goal ({GOAL_X}, {GOAL_Y})...')
    node.send_goal()

    # Wait up to 15s for planner response
    deadline = time.time() + 15.0
    while time.time() < deadline:
        time.sleep(0.2)
        with node._lock:
            has_path = len(node.global_paths) > 0
            statuses = list(node.planner_statuses)
        if has_path and 'SUCCESS' in statuses:
            break

    # ── Check 1: planner_status = SUCCESS ──
    with node._lock:
        statuses = list(node.planner_statuses)
    has_success = 'SUCCESS' in statuses
    results['planner_status_success'] = has_success
    print(f'\n  [{"PASS" if has_success else "FAIL"}] planner_status_success: '
          f'statuses={statuses}')

    # ── Check 2: global_path received ──
    with node._lock:
        paths = list(node.global_paths)
    has_path = len(paths) > 0
    results['global_path_received'] = has_path
    print(f'  [{"PASS" if has_path else "FAIL"}] global_path_received: '
          f'{len(paths)} paths')

    if has_path:
        last_path = paths[-1]
        n_poses = len(last_path.poses)

        # ── Check 3: path has reasonable length ──
        path_long_enough = n_poses >= 5
        results['path_length_reasonable'] = path_long_enough
        print(f'  [{"PASS" if path_long_enough else "FAIL"}] '
              f'path_length_reasonable: {n_poses} poses')

        # ── Check 4: path start near robot ──
        p0 = last_path.poses[0].pose.position
        dist_start = math.sqrt((p0.x - START_X)**2 + (p0.y - START_Y)**2)
        start_ok = dist_start < 5.0  # within 5m of start
        results['path_start_near_robot'] = start_ok
        print(f'  [{"PASS" if start_ok else "FAIL"}] path_start_near_robot: '
              f'({p0.x:.1f}, {p0.y:.1f}), dist={dist_start:.1f}m')

        # ── Check 5: path end near goal ──
        pn = last_path.poses[-1].pose.position
        dist_end = math.sqrt((pn.x - GOAL_X)**2 + (pn.y - GOAL_Y)**2)
        end_ok = dist_end < 5.0  # within 5m of goal
        results['path_end_near_goal'] = end_ok
        print(f'  [{"PASS" if end_ok else "FAIL"}] path_end_near_goal: '
              f'({pn.x:.1f}, {pn.y:.1f}), dist={dist_end:.1f}m')
    else:
        for name in ['path_length_reasonable', 'path_start_near_robot',
                      'path_end_near_goal']:
            results[name] = False
            print(f'  [FAIL] {name}: no path received')

    # ── Summary ──
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    all_pass = all(results.values())
    print('\n' + '=' * 60)
    for k, v in results.items():
        print(f'  [{"PASS" if v else "FAIL"}] {k}')
    print(f'\n  {sum(results.values())}/{len(results)} passed')
    print('=' * 60)
    print(json.dumps(results, indent=2))

    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
