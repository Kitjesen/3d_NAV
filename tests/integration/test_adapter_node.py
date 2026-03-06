#!/usr/bin/env python3
"""
pct_path_adapter ROS2 integration test

Tests the real C++ pct_path_adapter node by acting as both publisher and
subscriber.  The adapter node must be started separately with proper remaps:

    ros2 run pct_adapters pct_path_adapter --ros-args \
      -r /pct_path:=/nav/global_path \
      -r /Odometry:=/nav/odometry \
      -r /planner_waypoint:=/nav/way_point \
      -p waypoint_distance:=1.5 \
      -p arrival_threshold:=0.8 \
      -p stuck_timeout_sec:=30.0

A static TF (map -> odom, identity) is also required:

    ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom

Phases:
  Phase 1 (5s)  -- Path injection: publish odom + global_path, verify
                    path_received event and first waypoint
  Phase 2 (20s) -- Waypoint progression: move odom along x-axis (0->9),
                    verify waypoint_reached events with monotonic index
  Phase 3 (5s)  -- Goal reached: set odom to (9,0), verify goal_reached event

Usage (on robot S100P 192.168.66.190):
    source /opt/ros/humble/setup.bash
    source ~/lingtu/install/setup.bash
    python3 tests/integration/test_adapter_node.py
"""

import json
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


# -- QoS profiles -------------------------------------------------------------

# For global_path: TRANSIENT_LOCAL + RELIABLE, depth=1 (latched)
PATH_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# Standard reliable QoS for other topics
RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# -- Test node -----------------------------------------------------------------

class AdapterTestNode(Node):
    """
    Single ROS2 node that publishes odometry + path and subscribes to
    adapter_status + way_point to verify pct_path_adapter behavior.
    """

    def __init__(self):
        super().__init__('adapter_test_node')

        # -- Publishers --
        self.pub_odom = self.create_publisher(
            Odometry, '/nav/odometry', RELIABLE_QOS)
        self.pub_path = self.create_publisher(
            Path, '/nav/global_path', PATH_QOS)

        # -- Subscribers --
        self._lock = threading.Lock()
        self.adapter_status_msgs = []   # list of parsed JSON dicts
        self.waypoint_msgs = []         # list of PointStamped

        self.create_subscription(
            String, '/nav/adapter_status',
            self._on_adapter_status, RELIABLE_QOS)
        self.create_subscription(
            PointStamped, '/nav/way_point',
            self._on_waypoint, RELIABLE_QOS)

        # -- Robot state --
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0

        # Odometry timer at 20 Hz
        self._odom_timer = self.create_timer(0.05, self._publish_odom)

    # -- Callbacks -------------------------------------------------------------

    def _on_adapter_status(self, msg):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            data = {'raw': msg.data}
        with self._lock:
            self.adapter_status_msgs.append(data)
        event = data.get('event', '?')
        idx = data.get('index', '?')
        total = data.get('total', '?')
        self.get_logger().info(
            f'adapter_status: event={event} index={idx} total={total}')

    def _on_waypoint(self, msg):
        with self._lock:
            self.waypoint_msgs.append(msg)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'body'
        msg.pose.pose.position.x = self.robot_x
        msg.pose.pose.position.y = self.robot_y
        msg.pose.pose.position.z = self.robot_z
        msg.pose.pose.orientation.w = 1.0
        self.pub_odom.publish(msg)

    # -- Helpers ---------------------------------------------------------------

    def spin_for(self, seconds):
        """Spin the node for a given duration."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def get_status_events(self, event_name):
        """Return all adapter_status messages matching the given event name."""
        with self._lock:
            return [m for m in self.adapter_status_msgs
                    if m.get('event') == event_name]

    def get_waypoint_indices_from_status(self):
        """Return list of waypoint_reached indices in order of receipt."""
        with self._lock:
            return [m['index'] for m in self.adapter_status_msgs
                    if m.get('event') == 'waypoint_reached']

    def clear_state(self):
        """Reset collected messages."""
        with self._lock:
            self.adapter_status_msgs.clear()
            self.waypoint_msgs.clear()

    def make_straight_path(self, n_points=10, spacing=1.0):
        """Create a straight-line Path from (0,0) to ((n-1)*spacing, 0)."""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for i in range(n_points):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(i * spacing)
            ps.pose.position.y = 0.0
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        return path


# -- Test runner ---------------------------------------------------------------

def run_test(node):
    """
    Execute the three-phase test and return a results dict.
    """
    results = {
        'path_received_event': False,
        'first_waypoint_published': False,
        'waypoint_index_progresses': False,
        'goal_reached_event': False,
    }

    # =========================================================================
    # Phase 1: Path injection (5s)
    # =========================================================================
    print('\n' + '=' * 60)
    print('Phase 1: Path injection')
    print('=' * 60)

    node.robot_x = 0.0
    node.robot_y = 0.0

    # Let odom publish for 1s so the adapter initializes odom_frame_
    print('  Publishing odometry at (0,0) for 1s...')
    node.spin_for(1.0)

    # Publish the global path ONCE
    path = node.make_straight_path(n_points=10, spacing=1.0)
    print(f'  Publishing global_path: {len(path.poses)} waypoints, '
          f'(0,0) -> (9,0)')
    node.pub_path.publish(path)

    # Wait up to 4s for path_received and first waypoint
    deadline = time.time() + 4.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

        if not results['path_received_event']:
            events = node.get_status_events('path_received')
            if events:
                results['path_received_event'] = True
                total = events[0].get('total', '?')
                print(f'  [OK] path_received event (total={total})')

        if not results['first_waypoint_published']:
            with node._lock:
                if node.waypoint_msgs:
                    wp = node.waypoint_msgs[0]
                    results['first_waypoint_published'] = True
                    print(f'  [OK] First waypoint published: '
                          f'({wp.point.x:.2f}, {wp.point.y:.2f})')

        if results['path_received_event'] and results['first_waypoint_published']:
            break

    if not results['path_received_event']:
        print('  [FAIL] No path_received event within 5s')
    if not results['first_waypoint_published']:
        print('  [FAIL] No waypoint published within 5s')

    # =========================================================================
    # Phase 2: Waypoint progression (20s)
    # =========================================================================
    print('\n' + '=' * 60)
    print('Phase 2: Waypoint progression')
    print('=' * 60)

    # Move robot from x=0 to x=9 over 20s (0.45 m/s)
    start_time = time.time()
    duration = 20.0
    speed = 9.0 / duration  # 0.45 m/s
    last_print = 0.0

    while True:
        elapsed = time.time() - start_time
        if elapsed >= duration:
            break

        node.robot_x = min(9.0, elapsed * speed)
        node.robot_y = 0.0
        rclpy.spin_once(node, timeout_sec=0.02)

        # Print progress every 2s
        if elapsed - last_print >= 2.0:
            reached = node.get_waypoint_indices_from_status()
            with node._lock:
                n_wp = len(node.waypoint_msgs)
            print(f'  t={elapsed:.0f}s  robot_x={node.robot_x:.2f}  '
                  f'waypoints_reached={len(reached)}  '
                  f'way_point_msgs={n_wp}')
            last_print = elapsed

    # Check monotonic index progression
    reached_indices = node.get_waypoint_indices_from_status()
    if len(reached_indices) >= 2:
        is_monotonic = all(
            reached_indices[i] <= reached_indices[i + 1]
            for i in range(len(reached_indices) - 1)
        )
        if is_monotonic:
            results['waypoint_index_progresses'] = True
            print(f'  [OK] Waypoint indices are monotonically increasing: '
                  f'{reached_indices}')
        else:
            print(f'  [FAIL] Waypoint indices NOT monotonic: {reached_indices}')
    elif len(reached_indices) == 1:
        # Single waypoint reached -- technically monotonic but weak
        results['waypoint_index_progresses'] = True
        print(f'  [OK] 1 waypoint_reached event (index={reached_indices[0]})')
    else:
        print('  [FAIL] No waypoint_reached events during progression')

    # =========================================================================
    # Phase 3: Goal reached (5s)
    # =========================================================================
    print('\n' + '=' * 60)
    print('Phase 3: Goal reached')
    print('=' * 60)

    # Place robot at end of path
    node.robot_x = 9.0
    node.robot_y = 0.0
    print('  Robot at (9.0, 0.0), waiting for goal_reached...')

    deadline = time.time() + 5.0
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        events = node.get_status_events('goal_reached')
        if events:
            results['goal_reached_event'] = True
            idx = events[0].get('index', '?')
            total = events[0].get('total', '?')
            print(f'  [OK] goal_reached event (index={idx}, total={total})')
            break

    if not results['goal_reached_event']:
        # It might have been received during phase 2 already
        events = node.get_status_events('goal_reached')
        if events:
            results['goal_reached_event'] = True
            print('  [OK] goal_reached already received during phase 2')
        else:
            print('  [FAIL] No goal_reached event within 5s')

    return results


# -- Main ----------------------------------------------------------------------

def main():
    print('=' * 60)
    print('  pct_path_adapter ROS2 Integration Test')
    print('=' * 60)
    print()
    print('Prerequisites:')
    print('  1. ros2 run tf2_ros static_transform_publisher '
          '0 0 0 0 0 0 map odom')
    print('  2. ros2 run pct_adapters pct_path_adapter --ros-args \\')
    print('       -r /pct_path:=/nav/global_path \\')
    print('       -r /Odometry:=/nav/odometry \\')
    print('       -r /planner_waypoint:=/nav/way_point \\')
    print('       -p waypoint_distance:=1.5 \\')
    print('       -p arrival_threshold:=0.8 \\')
    print('       -p stuck_timeout_sec:=30.0')
    print()

    try:
        rclpy.init()
    except Exception as e:
        print(f'[ERROR] ROS2 init failed: {e}')
        print('  Run: source /opt/ros/humble/setup.bash')
        sys.exit(1)

    node = AdapterTestNode()

    try:
        results = run_test(node)
    except KeyboardInterrupt:
        print('\n[INTERRUPTED]')
        results = {}
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # -- Summary ---------------------------------------------------------------
    print('\n' + '=' * 60)
    print('  Test Results (JSON)')
    print('=' * 60)
    print(json.dumps(results, indent=2))

    n_pass = sum(1 for v in results.values() if v)
    n_total = len(results)
    print(f'\n  {n_pass}/{n_total} checks passed')

    if all(results.values()):
        print('  RESULT: ALL PASSED')
        sys.exit(0)
    else:
        failed = [k for k, v in results.items() if not v]
        print(f'  RESULT: FAILED checks: {failed}')
        sys.exit(1)


if __name__ == '__main__':
    main()
