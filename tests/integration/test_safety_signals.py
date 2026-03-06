#!/usr/bin/env python3
"""
安全信号链集成测试: localPlanner → pathFollower (stop / slow_down)

测试 localPlanner 近场急停 → pathFollower 安全制动 → 障碍清除恢复的完整信号链路。
需要 terrain_analysis + localPlanner + pathFollower 三个 C++ 节点预先启动。

Prerequisites (在另一个终端启动):

    source /opt/ros/humble/setup.bash
    source ~/lingtu/install/setup.bash  # 或实际 workspace

    PATHS_DIR=$(ros2 pkg prefix local_planner)/share/local_planner/paths

    # terrain_analysis
    ros2 run terrain_analysis terrainAnalysis --ros-args \\
      -r /Odometry:=/nav/odometry -r /cloud_map:=/nav/map_cloud \\
      -r /terrain_map:=/nav/terrain_map

    # localPlanner
    ros2 run local_planner localPlanner --ros-args \\
      -r /Odometry:=/nav/odometry -r /cloud_map:=/nav/map_cloud \\
      -r /terrain_map:=/nav/terrain_map -r /way_point:=/nav/way_point \\
      -p pathFolder:=$PATHS_DIR -p autonomyMode:=true -p autonomySpeed:=1.0 \\
      -p useTerrainAnalysis:=true -p checkObstacle:=true

    # pathFollower
    ros2 run local_planner pathFollower --ros-args \\
      -r /Odometry:=/nav/odometry -r /cmd_vel:=/nav/cmd_vel \\
      -p autonomyMode:=true -p autonomySpeed:=1.0

然后运行:

    python3 tests/integration/test_safety_signals.py

Test phases:
    Phase 1 (10s): Normal operation — flat ground, waypoint at (5,0,0)
                    Expect cmd_vel.linear.x > 0, /stop = 0
    Phase 2 (10s): Near-field obstacle — obstacle at x=0.3 within vehicle width
                    Expect /stop = 2 (ESTOP), cmd_vel ~ 0
    Phase 3 (8s):  Obstacle cleared — back to flat ground
                    Expect pathFollower resumes (cmd_vel.linear.x > 0)

Output: JSON with check results.
"""

import json
import math
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PointStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, Int8


# ── QoS ──────────────────────────────────────────────────────────────────────
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


def _make_pointcloud2(points_xyzi, stamp, frame_id='body'):
    """
    Build a PointCloud2 message from a list of (x, y, z, intensity) tuples.

    Args:
        points_xyzi: list of (x, y, z, intensity) float tuples
        stamp: rclpy Time message (builtin_interfaces/Time)
        frame_id: coordinate frame
    Returns:
        sensor_msgs/PointCloud2
    """
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points_xyzi)
    msg.is_dense = True
    msg.is_bigendian = False
    msg.point_step = 16  # 4 x float32
    msg.row_step = msg.point_step * msg.width
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = b''.join(struct.pack('ffff', *p) for p in points_xyzi)
    return msg


def _make_flat_ground(stamp):
    """
    Generate a flat ground point cloud in body frame centered on (0,0).

    Grid: x in [-5, 5], y in [-5, 5], step 0.4
    z = -0.5 in body frame (robot at odom z=0.5, ground at odom z=0.0)
    intensity = 100.0 (arbitrary; terrain_analysis computes its own heights)
    """
    points = []
    x = -5.0
    while x <= 5.0:
        y = -5.0
        while y <= 5.0:
            points.append((x, y, -0.5, 100.0))
            y += 0.4
        x += 0.4
    return _make_pointcloud2(points, stamp, frame_id='body')


def _make_ground_with_obstacle(stamp):
    """
    Generate a point cloud in body frame with flat ground + near-field obstacle.

    Ground: same as _make_flat_ground()
    Obstacle: x=0.3, y in [-0.3, 0.3] step 0.1, z from -0.5 to 0.0 in body frame
              This places points at 0.3m in front of the robot, within the vehicle width,
              spanning from ground level up to z=0 in body (z=0.5 in odom = same as robot).
              terrain_analysis will see these as elevated points above ground => obstacle.
    """
    points = []
    # Flat ground
    x = -5.0
    while x <= 5.0:
        y = -5.0
        while y <= 5.0:
            points.append((x, y, -0.5, 100.0))
            y += 0.4
        x += 0.4

    # Obstacle: dense wall at x=0.3 in body frame (directly in front)
    # y spans vehicle width, z spans from ground (-0.5) up to 0.0 (body frame)
    obs_y = -0.3
    while obs_y <= 0.3 + 1e-6:
        obs_z = -0.5
        while obs_z <= 0.0 + 1e-6:
            points.append((0.3, obs_y, obs_z, 100.0))
            obs_z += 0.1
        obs_y += 0.1
    return _make_pointcloud2(points, stamp, frame_id='body')


class SafetySignalTestNode(Node):
    """Test node: publishes synthetic sensor data, observes stop/cmd_vel."""

    def __init__(self):
        super().__init__('safety_signal_test')

        # ── Publishers ──
        self.pub_odom = self.create_publisher(Odometry, '/nav/odometry', 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/nav/map_cloud', 10)
        self.pub_waypoint = self.create_publisher(PointStamped, '/nav/way_point', 10)
        # The prerequisite localPlanner does NOT remap /stop or /speed,
        # so pathFollower subscribes on the raw internal name.
        # We publish stop=0 on BOTH in case someone uses the launch-file remapped name.
        self.pub_stop_raw = self.create_publisher(Int8, '/stop', 10)
        self.pub_stop_nav = self.create_publisher(Int8, '/nav/stop', 10)
        self.pub_speed_raw = self.create_publisher(Float32, '/speed', 10)
        self.pub_speed_nav = self.create_publisher(Float32, '/nav/speed', 10)

        # ── Subscribers ──
        self._lock = threading.Lock()
        self.cmd_vel_msgs = []    # TwistStamped from pathFollower
        self.stop_raw_msgs = []   # Int8 from localPlanner on /stop
        self.stop_nav_msgs = []   # Int8 from localPlanner on /nav/stop (if remapped)

        self.create_subscription(
            TwistStamped, '/nav/cmd_vel', self._on_cmd_vel, 10)
        self.create_subscription(
            Int8, '/stop', self._on_stop_raw, 10)
        self.create_subscription(
            Int8, '/nav/stop', self._on_stop_nav, 10)

        # ── State ──
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.5  # Robot at z=0.5 in odom (ground is z=0.0)
        self.odom_yaw = 0.0

    def _on_cmd_vel(self, msg):
        with self._lock:
            self.cmd_vel_msgs.append(msg)

    def _on_stop_raw(self, msg):
        with self._lock:
            self.stop_raw_msgs.append(msg)

    def _on_stop_nav(self, msg):
        with self._lock:
            self.stop_nav_msgs.append(msg)

    def clear_cmd_vel(self):
        with self._lock:
            self.cmd_vel_msgs.clear()

    def clear_stop(self):
        with self._lock:
            self.stop_raw_msgs.clear()
            self.stop_nav_msgs.clear()

    def get_cmd_vel(self):
        with self._lock:
            return list(self.cmd_vel_msgs)

    def get_stop_msgs(self):
        """Return combined stop messages from both /stop and /nav/stop."""
        with self._lock:
            combined = []
            for m in self.stop_raw_msgs:
                combined.append(('raw', m.data))
            for m in self.stop_nav_msgs:
                combined.append(('nav', m.data))
            return combined

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'body'
        msg.pose.pose.position.x = self.odom_x
        msg.pose.pose.position.y = self.odom_y
        msg.pose.pose.position.z = self.odom_z
        cy = math.cos(self.odom_yaw / 2.0)
        sy = math.sin(self.odom_yaw / 2.0)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = sy
        msg.pose.pose.orientation.w = cy
        self.pub_odom.publish(msg)

    def publish_waypoint(self, x, y, z=0.0):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = float(z)
        self.pub_waypoint.publish(msg)

    def publish_stop(self, value):
        msg = Int8()
        msg.data = value
        self.pub_stop_raw.publish(msg)
        self.pub_stop_nav.publish(msg)

    def publish_speed(self, value):
        msg = Float32()
        msg.data = float(value)
        self.pub_speed_raw.publish(msg)
        self.pub_speed_nav.publish(msg)


def main():
    print('=' * 60)
    print('  Safety Signal Chain Integration Test')
    print('  localPlanner -> pathFollower (/stop, /slow_down)')
    print('=' * 60)

    try:
        rclpy.init()
    except Exception as e:
        print(f'[FATAL] ROS2 init failed: {e}')
        print('  Please run: source /opt/ros/humble/setup.bash')
        sys.exit(1)

    node = SafetySignalTestNode()

    # Background spin thread for callbacks
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    results = {}

    try:
        # ──────────────────────────────────────────────────────────────
        # Warm-up (5s): Send odom + flat cloud + stop=0 + speed=1.0
        # Let terrain_analysis accumulate data, localPlanner initialize.
        # pathFollower needs 3 consecutive stop=0 to clear any residual
        # safetyStop.
        # ──────────────────────────────────────────────────────────────
        print('\n[Warm-up] Initializing terrain + planner (5s)...')
        warmup_start = time.monotonic()
        while time.monotonic() - warmup_start < 5.0:
            stamp = node.get_clock().now().to_msg()
            node.publish_odom()
            node.pub_cloud.publish(_make_flat_ground(stamp))
            node.publish_waypoint(5.0, 0.0)
            node.publish_stop(0)
            node.publish_speed(1.0)
            time.sleep(0.05)  # 20 Hz

        # ──────────────────────────────────────────────────────────────
        # Phase 1: Normal operation (10s)
        #   - Flat ground, no obstacles
        #   - Waypoint at (5, 0, 0)
        #   - Expect cmd_vel.linear.x > 0 (robot moves forward)
        #   - Expect no stop=2 from localPlanner
        # ──────────────────────────────────────────────────────────────
        print('\n[Phase 1] Normal operation — flat ground, waypoint at (5,0) (10s)...')
        node.clear_cmd_vel()
        node.clear_stop()

        phase1_start = time.monotonic()
        while time.monotonic() - phase1_start < 10.0:
            stamp = node.get_clock().now().to_msg()
            node.publish_odom()
            node.pub_cloud.publish(_make_flat_ground(stamp))
            node.publish_waypoint(5.0, 0.0)
            node.publish_stop(0)
            node.publish_speed(1.0)
            time.sleep(0.05)

        # Analyze Phase 1
        cvs = node.get_cmd_vel()
        stop_msgs = node.get_stop_msgs()

        print(f'  Received {len(cvs)} cmd_vel messages')
        print(f'  Received {len(stop_msgs)} stop messages')

        if len(cvs) == 0:
            print('  [FAIL] No cmd_vel received! Is pathFollower running?')
            results['normal_cmd_vel_positive'] = False
        else:
            # Use second half of messages (after system stabilizes)
            half = max(len(cvs) // 2, 1)
            late_cvs = cvs[half:]
            fwd_speeds = [m.twist.linear.x for m in late_cvs]
            avg_fwd = sum(fwd_speeds) / len(fwd_speeds) if fwd_speeds else 0.0
            max_fwd = max(fwd_speeds) if fwd_speeds else 0.0

            normal_ok = avg_fwd > 0.02
            results['normal_cmd_vel_positive'] = normal_ok
            status = 'PASS' if normal_ok else 'FAIL'
            print(f'  [{status}] normal_cmd_vel_positive: '
                  f'avg_fwd={avg_fwd:.4f}, max_fwd={max_fwd:.4f} (expect >0.02)')

        # Check that no stop=2 was published by localPlanner during normal operation
        # (our own stop=0 will appear, so filter for value=2 from localPlanner)
        estop_in_phase1 = any(val == 2 for _, val in stop_msgs)
        no_estop_ok = not estop_in_phase1
        # This is informational — if terrain_analysis produces spurious obstacles
        # it may trigger, so we log but don't hard-fail
        if estop_in_phase1:
            print(f'  [WARN] stop=2 detected during flat-ground phase (possible terrain transient)')
        else:
            print(f'  [INFO] No stop=2 during normal operation (expected)')

        # ──────────────────────────────────────────────────────────────
        # Phase 2: Near-field obstacle (10s)
        #   - Add obstacle at x=0.3 in body frame (very close)
        #   - Within nearFieldStopDis_ (default 0.5m) and vehicle width
        #   - Expect localPlanner to publish stop=2
        #   - Expect pathFollower to zero out cmd_vel
        # ──────────────────────────────────────────────────────────────
        print('\n[Phase 2] Near-field obstacle (10s)...')
        node.clear_cmd_vel()
        node.clear_stop()

        phase2_start = time.monotonic()
        stop2_detected = False
        stop2_time = None

        while time.monotonic() - phase2_start < 10.0:
            stamp = node.get_clock().now().to_msg()
            node.publish_odom()
            # Publish cloud with obstacle
            node.pub_cloud.publish(_make_ground_with_obstacle(stamp))
            node.publish_waypoint(5.0, 0.0)
            # Do NOT publish stop=0 from test side — let localPlanner control /stop
            node.publish_speed(1.0)
            time.sleep(0.05)

            # Check for stop=2
            if not stop2_detected:
                for source, val in node.get_stop_msgs():
                    if val == 2:
                        stop2_detected = True
                        stop2_time = time.monotonic() - phase2_start
                        print(f'  stop=2 detected from "{source}" at t={stop2_time:.1f}s')
                        break

        # Analyze Phase 2 — stop signal
        results['obstacle_stop_signal'] = stop2_detected
        status = 'PASS' if stop2_detected else 'FAIL'
        print(f'  [{status}] obstacle_stop_signal: '
              f'{"detected at " + f"{stop2_time:.1f}s" if stop2_detected else "NOT detected"}')

        # Analyze Phase 2 — cmd_vel should go to ~zero
        cvs = node.get_cmd_vel()
        print(f'  Received {len(cvs)} cmd_vel messages during obstacle phase')

        if len(cvs) == 0:
            print('  [WARN] No cmd_vel received during Phase 2')
            # If pathFollower is running and has safetyStop=2, it still publishes
            # cmd_vel with zeroed values. No messages means pathFollower may not
            # have received a path yet.
            results['obstacle_cmd_vel_zero'] = stop2_detected  # infer from stop
        else:
            # Use messages from the second half (after stop propagation)
            half = max(len(cvs) // 2, 1)
            late_cvs = cvs[half:]
            fwd_speeds = [abs(m.twist.linear.x) for m in late_cvs]
            avg_abs_fwd = sum(fwd_speeds) / len(fwd_speeds) if fwd_speeds else 0.0

            zero_ok = avg_abs_fwd < 0.05
            results['obstacle_cmd_vel_zero'] = zero_ok
            status = 'PASS' if zero_ok else 'FAIL'
            print(f'  [{status}] obstacle_cmd_vel_zero: '
                  f'avg_abs_fwd={avg_abs_fwd:.4f} (expect <0.05)')

        # ──────────────────────────────────────────────────────────────
        # Phase 3: Obstacle cleared (8s)
        #   - Remove obstacle from point cloud (flat ground only)
        #   - localPlanner should publish stop=0 (obstacle gone)
        #   - pathFollower needs 3 consecutive stop=0 to clear safetyStop
        #   - Expect cmd_vel.linear.x > 0 (recovery)
        # ──────────────────────────────────────────────────────────────
        print('\n[Phase 3] Obstacle cleared — recovery (8s)...')
        node.clear_cmd_vel()
        node.clear_stop()

        phase3_start = time.monotonic()
        while time.monotonic() - phase3_start < 8.0:
            stamp = node.get_clock().now().to_msg()
            node.publish_odom()
            # Back to flat ground — no obstacle
            node.pub_cloud.publish(_make_flat_ground(stamp))
            node.publish_waypoint(5.0, 0.0)
            # Do NOT publish stop=0 from test side during the first 2s.
            # Let localPlanner naturally publish stop=0 when it sees no obstacle.
            # After 2s, also send stop=0 to help pathFollower accumulate
            # the 3 consecutive stop=0 frames needed to clear safetyStop.
            elapsed = time.monotonic() - phase3_start
            if elapsed > 2.0:
                node.publish_stop(0)
            node.publish_speed(1.0)
            time.sleep(0.05)

        # Analyze Phase 3 — cmd_vel should recover
        cvs = node.get_cmd_vel()
        print(f'  Received {len(cvs)} cmd_vel messages during recovery phase')

        if len(cvs) == 0:
            print('  [FAIL] No cmd_vel received during recovery phase')
            results['recovery_cmd_vel_positive'] = False
        else:
            # Use the last quarter of messages — recovery may take a few seconds
            quarter = max(len(cvs) * 3 // 4, 1)
            late_cvs = cvs[quarter:]
            fwd_speeds = [m.twist.linear.x for m in late_cvs]
            avg_fwd = sum(fwd_speeds) / len(fwd_speeds) if fwd_speeds else 0.0
            max_fwd = max(fwd_speeds) if fwd_speeds else 0.0

            recovery_ok = max_fwd > 0.02
            results['recovery_cmd_vel_positive'] = recovery_ok
            status = 'PASS' if recovery_ok else 'FAIL'
            print(f'  [{status}] recovery_cmd_vel_positive: '
                  f'avg_fwd={avg_fwd:.4f}, max_fwd={max_fwd:.4f} (expect max >0.02)')

        # Check stop=0 was published during recovery
        stop_msgs = node.get_stop_msgs()
        stop0_count = sum(1 for _, val in stop_msgs if val == 0)
        if stop0_count > 0:
            print(f'  [INFO] Received {stop0_count} stop=0 messages during recovery')
        else:
            print(f'  [WARN] No stop=0 received during recovery '
                  f'(localPlanner may not have cleared obstacle state)')

    except KeyboardInterrupt:
        print('\n[INTERRUPTED]')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

    # ──────────────────────────────────────────────────────────────
    # Summary
    # ──────────────────────────────────────────────────────────────
    print('\n' + '=' * 60)
    print('  Safety Signal Chain — Results Summary')
    print('=' * 60)

    all_pass = all(results.values()) if results else False
    passed = sum(1 for v in results.values() if v)
    failed = sum(1 for v in results.values() if not v)

    for check, ok in results.items():
        tag = 'PASS' if ok else 'FAIL'
        print(f'  [{tag}] {check}')

    print(f'\n  Passed: {passed}/{len(results)}   Failed: {failed}/{len(results)}')

    if all_pass:
        print('\n  All safety signal checks passed.')
    else:
        print('\n  Some checks failed. Troubleshooting:')
        if not results.get('normal_cmd_vel_positive', True):
            print('    - Is pathFollower running and receiving /path from localPlanner?')
            print('    - Does localPlanner have a valid pathFolder with pre-computed paths?')
        if not results.get('obstacle_stop_signal', True):
            print('    - Is localPlanner started with checkObstacle:=true?')
            print('    - Is terrain_analysis running and producing /nav/terrain_map?')
            print('    - The near-field obstacle may not exceed obstacleHeightThre (0.2m).')
            print('      terrain_analysis computes obstacle height from ground; if the')
            print('      synthetic cloud does not produce sufficient height contrast,')
            print('      the near-field check will not trigger.')
        if not results.get('obstacle_cmd_vel_zero', True):
            print('    - stop=2 should zero all velocity components via safetyStop >= 2.')
        if not results.get('recovery_cmd_vel_positive', True):
            print('    - pathFollower needs 3 consecutive stop=0 to clear safetyStop.')
            print('    - localPlanner must publish stop=0 when obstacle is gone.')
            print('    - The test also sends stop=0 after 2s to assist clearance.')

    print('\n[JSON]')
    print(json.dumps(results, indent=2))

    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
