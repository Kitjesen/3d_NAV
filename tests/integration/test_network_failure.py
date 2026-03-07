#!/usr/bin/env python3
"""
T8: han_dog_bridge 集成测试 — Mock CMS gRPC Server

通过启动本地 mock CMS 服务端，测试 han_dog_bridge 在各种网络故障场景下的行为:

  T8-1: 重连 — CMS 服务未启动时，bridge 应自动重试
  T8-2: IMU 数据流 → /Odometry 正确发布 (四元数 + 位置积分)
  T8-3: cmd_vel → Walk() 速度归一化正确
  T8-4: 看门狗 — cmd_vel 超时后自动发送零速
  T8-5: 流中断恢复 — ListenImu 异常后 bridge 自动重连

运行前提:
  source /opt/ros/humble/setup.bash
  source ~/lingtu/install/setup.bash
  pip install grpcio-tools  # 用于编译 mock proto

运行方式:
  # 使用 mock server (推荐):
  python3 tests/integration/test_network_failure.py

  # 可选: 指定真实 dog 地址验证兼容性:
  python3 tests/integration/test_network_failure.py --real-dog 192.168.4.100:13145

架构:
  本测试脚本                    测试目标
  ┌──────────────────────┐     ┌───────────────────────┐
  │ MockCmsServer(:13200)│◄────│ han_dog_bridge (ros2) │
  │  ListenImu stream    │     │  /Odometry             │
  │  ListenJoint stream  │     │  /robot_state          │
  │  Walk() recorder     │     │  /driver/watchdog_active│
  └──────────────────────┘     └───────────────────────┘
  ┌──────────────────────┐
  │ TestNode (ROS2)      │──cmd_vel──►  bridge
  │  /Odometry sub       │◄──odom──── bridge
  └──────────────────────┘
"""

import argparse
import asyncio
import importlib
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Optional, List

# ─── 颜色 ─────────────────────────────────────────────────────────────────────
class C:
    GREEN  = '\033[0;32m'
    RED    = '\033[0;31m'
    YELLOW = '\033[1;33m'
    BLUE   = '\033[0;34m'
    BOLD   = '\033[1m'
    NC     = '\033[0m'


PASS = f"{C.GREEN}✓ PASS{C.NC}"
FAIL = f"{C.RED}✗ FAIL{C.NC}"
WARN = f"{C.YELLOW}⚠ WARN{C.NC}"
SKIP = f"{C.YELLOW}⊘ SKIP{C.NC}"

# ─── Proto 编译 ───────────────────────────────────────────────────────────────

PROTO_DIR = Path(__file__).parent / 'mock'
PROTO_FILE = PROTO_DIR / 'han_dog_cms.proto'


def compile_proto(out_dir: str) -> Optional[str]:
    """编译 han_dog_cms.proto，返回 import 路径，失败返回 None."""
    try:
        import grpc_tools.protoc as protoc
    except ImportError:
        return None

    ret = protoc.main([
        'grpc_tools.protoc',
        f'--proto_path={PROTO_DIR}',
        f'--python_out={out_dir}',
        f'--grpc_python_out={out_dir}',
        str(PROTO_FILE),
    ])
    if ret != 0:
        return None
    return out_dir


def load_mock_proto(out_dir: str):
    """加载编译好的 proto stubs，返回 (pb2, pb2_grpc) 或 None."""
    try:
        sys.path.insert(0, out_dir)
        import han_dog_cms_pb2 as pb2
        import han_dog_cms_pb2_grpc as pb2_grpc
        return pb2, pb2_grpc
    except ImportError as e:
        print(f"  {WARN} 无法加载 mock proto: {e}")
        return None, None


# ─── Mock CMS Servicer ────────────────────────────────────────────────────────

class MockCmsServicer:
    """
    模拟 han_dog CMS gRPC 服务.

    行为可通过实例变量控制:
      imu_messages   : 要流式发送的 Imu 消息列表
      joint_messages : 要流式发送的 JointUpdate 消息列表
      walk_calls     : 记录所有收到的 Walk() 参数
      raise_on_imu   : 若非 None，在 ListenImu 中抛出此异常
      imu_delay      : 每条 IMU 消息间的延迟 (秒)
    """

    def __init__(self, pb2, pb2_grpc):
        self.pb2 = pb2
        self.pb2_grpc = pb2_grpc
        self.imu_messages: List = []
        self.joint_messages: List = []
        self.walk_calls: List = []
        self.raise_on_imu: Optional[Exception] = None
        self.imu_delay: float = 0.05
        self._lock = threading.Lock()

    def make_servicer(self):
        """返回 gRPC servicer 实例."""
        pb2 = self.pb2
        mock = self

        class _Svc(self.pb2_grpc.CmsServicer):
            async def Enable(self, request, context):
                return pb2.Empty()

            async def Disable(self, request, context):
                return pb2.Empty()

            async def StandUp(self, request, context):
                return pb2.Empty()

            async def SitDown(self, request, context):
                return pb2.Empty()

            async def Walk(self, request, context):
                with mock._lock:
                    mock.walk_calls.append({
                        'x': request.x,
                        'y': request.y,
                        'z': request.z,
                    })
                return pb2.Empty()

            async def ListenImu(self, request, context):
                if mock.raise_on_imu is not None:
                    raise mock.raise_on_imu
                for imu in mock.imu_messages:
                    yield imu
                    await asyncio.sleep(mock.imu_delay)

            async def ListenJoint(self, request, context):
                for joint in mock.joint_messages:
                    yield joint
                    await asyncio.sleep(mock.imu_delay)

            async def ListenHistory(self, request, context):
                # 空流 — 立刻结束
                return
                yield  # noqa: unreachable

        return _Svc()


# ─── 测试服务器管理 ────────────────────────────────────────────────────────────

class MockServerRunner:
    """在独立线程运行 mock gRPC asyncio 服务器."""

    def __init__(self, pb2_grpc, servicer, port: int = 13200):
        self._pb2_grpc = pb2_grpc
        self._servicer = servicer
        self._port = port
        self._server = None
        self._loop = None
        self._thread = None
        self._ready = threading.Event()

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._ready.wait(timeout=5.0)

    def _run(self):
        import grpc.aio as grpc_aio
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._loop.run_until_complete(self._serve())

    async def _serve(self):
        import grpc.aio as grpc_aio
        self._server = grpc_aio.server()
        self._pb2_grpc.add_CmsServicer_to_server(
            self._servicer, self._server)
        self._server.add_insecure_port(f'[::]:{self._port}')
        await self._server.start()
        self._ready.set()
        await self._server.wait_for_termination()

    def stop(self):
        if self._server and self._loop:
            asyncio.run_coroutine_threadsafe(
                self._server.stop(grace=1.0), self._loop)


# ─── ROS2 测试节点 ─────────────────────────────────────────────────────────────

def _try_import_rclpy():
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import Bool, Int8
        return rclpy, Node, Odometry, TwistStamped, Bool, Int8
    except ImportError:
        return None, None, None, None, None, None


# ─── 测试套件 ─────────────────────────────────────────────────────────────────

class T8TestSuite:

    MOCK_PORT = 13200

    def __init__(self, pb2, pb2_grpc, real_dog: Optional[str] = None):
        self.pb2 = pb2
        self.pb2_grpc = pb2_grpc
        self.real_dog = real_dog
        self.passed = 0
        self.failed = 0
        self.skipped = 0

        rclpy, Node, Odometry, TwistStamped, Bool, Int8 = _try_import_rclpy()
        self._rclpy = rclpy
        self._node_cls = Node
        self._Odometry = Odometry
        self._TwistStamped = TwistStamped
        self._Bool = Bool
        self._Int8 = Int8
        self._ros_ok = rclpy is not None

    def _ok(self, name):
        print(f"  {PASS}  {name}")
        self.passed += 1

    def _fail(self, name, reason=''):
        print(f"  {FAIL}  {name}" + (f"  ({reason})" if reason else ''))
        self.failed += 1

    def _skip(self, name, reason=''):
        print(f"  {SKIP}  {name}" + (f"  ({reason})" if reason else ''))
        self.skipped += 1

    def _make_imu(self, gx=0.0, gy=0.0, gz=0.1,
                  qx=0.0, qy=0.0, qz=0.707, qw=0.707) -> object:
        """构造一个 Imu 消息 (yaw ≈ 90°)."""
        imu = self.pb2.Imu()
        imu.gyroscope.x = gx
        imu.gyroscope.y = gy
        imu.gyroscope.z = gz
        imu.quaternion.x = qx
        imu.quaternion.y = qy
        imu.quaternion.z = qz
        imu.quaternion.w = qw
        return imu

    def _make_joint_all(self, n: int = 16) -> object:
        """构造 JointUpdate(all_joints) — 12 DOF + 4 foot."""
        ju = self.pb2.JointUpdate()
        for i in range(n):
            ju.all_joints.position.values.append(float(i) * 0.01)
            ju.all_joints.velocity.values.append(0.0)
            ju.all_joints.torque.values.append(float(i) * 0.1)
        return ju

    # ── T8-1: 重连测试 ────────────────────────────────────────────────────────
    def test_t8_1_reconnect(self):
        """T8-1: bridge 在 CMS 不可达时自动重试连接."""
        print(f"\n{C.BLUE}T8-1: 重连测试 — CMS 服务先不可达，再启动{C.NC}")

        unused_port = self.MOCK_PORT + 10  # 先用一个没人监听的端口

        # 启动 bridge 指向无效端口
        proc = subprocess.Popen(
            [
                'ros2', 'run', 'robot_driver', 'han_dog_bridge',
                '--ros-args',
                '-p', f'dog_port:={unused_port}',
                '-p', 'reconnect_interval:=1.0',
                '-p', 'auto_enable:=false',
                '-p', 'auto_standup:=false',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        # 等 bridge 至少尝试连接一次 (reconnect_interval=1s → 3s 内至少 2 次)
        time.sleep(3.0)

        # 检查 bridge 是否还在运行 (应该在 — 不会因重连失败而退出)
        alive = proc.poll() is None
        proc.terminate()
        proc.wait(timeout=5)

        if alive:
            self._ok("bridge 在连接失败后保持运行并持续重试")
        else:
            rc = proc.returncode
            if rc is not None and rc != 0:
                self._fail("bridge 意外退出", f"returncode={rc}")
            else:
                self._ok("bridge 在连接失败后保持运行并持续重试")

    # ── T8-2: IMU → Odometry ─────────────────────────────────────────────────
    def test_t8_2_imu_to_odom(self):
        """T8-2: mock ListenImu → /Odometry 四元数正确."""
        print(f"\n{C.BLUE}T8-2: IMU 数据流 → /Odometry 发布{C.NC}")

        if not self._ros_ok:
            self._skip("T8-2", "ROS2 不可用")
            return

        # 构造 50 条 IMU 消息 (yaw ≈ 90°, gz=0.1 rad/s)
        imu_msgs = [self._make_imu(gz=0.1, qz=0.707, qw=0.707) for _ in range(50)]

        svc = MockCmsServicer(self.pb2, self.pb2_grpc)
        svc.imu_messages = imu_msgs
        svc.imu_delay = 0.02  # 50 Hz

        runner = MockServerRunner(self.pb2_grpc, svc.make_servicer(), port=self.MOCK_PORT)
        runner.start()

        # 启动 bridge
        proc = subprocess.Popen(
            [
                'ros2', 'run', 'robot_driver', 'han_dog_bridge',
                '--ros-args',
                '-p', f'dog_host:=localhost',
                '-p', f'dog_port:={self.MOCK_PORT}',
                '-p', 'auto_enable:=false',
                '-p', 'auto_standup:=false',
                '-p', 'odom_pub_rate:=10.0',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        odom_received = []
        odom_lock = threading.Lock()

        rclpy = self._rclpy
        Node = self._node_cls
        Odometry = self._Odometry

        class OdomSubscriber(Node):
            def __init__(self_inner):
                super().__init__('t8_odom_sub')
                self_inner.create_subscription(
                    Odometry, '/Odometry',
                    lambda m: odom_received.append(m), 10)

        rclpy.init()
        node = OdomSubscriber()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 等待 odom 消息 (最多 5s)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and len(odom_received) < 3:
            time.sleep(0.1)

        proc.terminate()
        proc.wait(timeout=5)
        runner.stop()
        executor.shutdown(wait=False)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

        if len(odom_received) < 3:
            self._fail("T8-2 /Odometry", f"只收到 {len(odom_received)} 条 (期望 ≥ 3)")
            return

        # 验证四元数
        msg = odom_received[-1]
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        expected_yaw = 2 * math.atan2(qz, qw)
        # yaw ≈ 90° = π/2
        yaw_ok = abs(expected_yaw - math.pi / 2) < 0.1
        if yaw_ok:
            self._ok(f"T8-2 /Odometry 四元数正确 (yaw≈90°, actual={math.degrees(expected_yaw):.1f}°)")
        else:
            self._fail("T8-2 /Odometry 四元数",
                       f"yaw={math.degrees(expected_yaw):.1f}° 期望≈90°")

    # ── T8-3: cmd_vel → Walk() 归一化 ─────────────────────────────────────────
    def test_t8_3_cmd_vel_to_walk(self):
        """T8-3: /cmd_vel → Walk(x,y,z) 速度归一化正确."""
        print(f"\n{C.BLUE}T8-3: cmd_vel → Walk() 归一化{C.NC}")

        if not self._ros_ok:
            self._skip("T8-3", "ROS2 不可用")
            return

        svc = MockCmsServicer(self.pb2, self.pb2_grpc)
        # 无限 IMU 流 (让 bridge 保持 _standing=True 需要 auto_standup=true)
        svc.imu_messages = [self._make_imu() for _ in range(200)]
        svc.imu_delay = 0.01

        runner = MockServerRunner(self.pb2_grpc, svc.make_servicer(), port=self.MOCK_PORT)
        runner.start()

        proc = subprocess.Popen(
            [
                'ros2', 'run', 'robot_driver', 'han_dog_bridge',
                '--ros-args',
                '-p', 'dog_host:=localhost',
                '-p', f'dog_port:={self.MOCK_PORT}',
                '-p', 'auto_enable:=true',
                '-p', 'auto_standup:=true',
                '-p', 'max_linear_speed:=1.0',
                '-p', 'max_angular_speed:=1.0',
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        # 等待 bridge 连接 + StandUp 完成
        time.sleep(2.0)

        rclpy = self._rclpy
        Node = self._node_cls
        TwistStamped = self._TwistStamped

        class CmdVelPublisher(Node):
            def __init__(self_inner):
                super().__init__('t8_cmd_vel_pub')
                self_inner.pub = self_inner.create_publisher(
                    TwistStamped, '/cmd_vel', 10)

            def send(self_inner, vx, vy, wz):
                msg = TwistStamped()
                msg.header.stamp = self_inner.get_clock().now().to_msg()
                msg.twist.linear.x = vx
                msg.twist.linear.y = vy
                msg.twist.angular.z = wz
                self_inner.pub.publish(msg)

        rclpy.init()
        node = CmdVelPublisher()

        # 发布 vx=0.5 (max=1.0 → Walk.x 应 ≈ 0.5)
        for _ in range(10):
            node.send(vx=0.5, vy=0.0, wz=0.0)
            time.sleep(0.05)

        time.sleep(0.5)  # 等待 Walk() 调用落地

        proc.terminate()
        proc.wait(timeout=5)
        runner.stop()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

        walk_calls = svc.walk_calls
        print(f"  收到 Walk() 调用 {len(walk_calls)} 次")

        if len(walk_calls) == 0:
            self._fail("T8-3 Walk() 调用", "未收到任何 Walk() (bridge 可能未 StandUp)")
            return

        # 取最后几次调用的均值
        recent = walk_calls[-min(5, len(walk_calls)):]
        avg_x = sum(c['x'] for c in recent) / len(recent)
        if abs(avg_x - 0.5) < 0.1:
            self._ok(f"T8-3 Walk.x 归一化正确 (avg={avg_x:.3f}, 期望≈0.5)")
        else:
            self._fail("T8-3 Walk.x 归一化",
                       f"avg={avg_x:.3f}, 期望≈0.5 (max_linear=1.0, vx=0.5)")

    # ── T8-4: 看门狗 ──────────────────────────────────────────────────────────
    def test_t8_4_watchdog(self):
        """T8-4: cmd_vel 超时 → bridge 自动发送零速."""
        print(f"\n{C.BLUE}T8-4: 看门狗 — cmd_vel 超时 → 自动零速{C.NC}")

        if not self._ros_ok:
            self._skip("T8-4", "ROS2 不可用")
            return

        svc = MockCmsServicer(self.pb2, self.pb2_grpc)
        svc.imu_messages = [self._make_imu() for _ in range(500)]
        svc.imu_delay = 0.01

        runner = MockServerRunner(self.pb2_grpc, svc.make_servicer(), port=self.MOCK_PORT)
        runner.start()

        proc = subprocess.Popen(
            [
                'ros2', 'run', 'robot_driver', 'han_dog_bridge',
                '--ros-args',
                '-p', 'dog_host:=localhost',
                '-p', f'dog_port:={self.MOCK_PORT}',
                '-p', 'auto_enable:=true',
                '-p', 'auto_standup:=true',
                '-p', 'max_linear_speed:=1.0',
                '-p', 'cmd_vel_timeout_ms:=300',  # 300ms 超时
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        rclpy = self._rclpy
        Node = self._node_cls
        TwistStamped = self._TwistStamped
        Bool = self._Bool

        class WatchdogMonitor(Node):
            def __init__(self_inner):
                super().__init__('t8_watchdog_mon')
                self_inner.pub_cmd = self_inner.create_publisher(
                    TwistStamped, '/cmd_vel', 10)
                self_inner.watchdog_active = False
                self_inner.create_subscription(
                    Bool, '/driver/watchdog_active',
                    lambda m: setattr(self_inner, 'watchdog_active', m.data),
                    10)

            def send_cmd(self_inner, vx):
                msg = TwistStamped()
                msg.header.stamp = self_inner.get_clock().now().to_msg()
                msg.twist.linear.x = vx
                self_inner.pub_cmd.publish(msg)

        rclpy.init()
        node = WatchdogMonitor()
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        # 等 bridge 连接 + StandUp
        time.sleep(2.0)

        # 发送 cmd_vel 5 次，然后停止
        for _ in range(5):
            node.send_cmd(0.3)
            time.sleep(0.05)

        # 等看门狗超时 (300ms timeout + 1s margin)
        time.sleep(1.5)

        watchdog_triggered = node.watchdog_active

        # 检查看门狗触发后是否有零速 Walk() 调用
        zero_vel_calls = [c for c in svc.walk_calls
                          if abs(c['x']) < 0.01 and abs(c['y']) < 0.01 and abs(c['z']) < 0.01]

        proc.terminate()
        proc.wait(timeout=5)
        runner.stop()
        executor.shutdown(wait=False)
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

        print(f"  Walk() 总调用: {len(svc.walk_calls)}, 零速调用: {len(zero_vel_calls)}")

        if watchdog_triggered or len(zero_vel_calls) > 0:
            self._ok("T8-4 看门狗触发并发送零速")
        else:
            self._fail("T8-4 看门狗", "未检测到 watchdog_active 或 Walk(0,0,0)")

    # ── T8-5: 流中断恢复 ──────────────────────────────────────────────────────
    def test_t8_5_stream_recovery(self):
        """T8-5: ListenImu 流中断后 bridge 自动重连."""
        print(f"\n{C.BLUE}T8-5: 流中断恢复 — ListenImu 异常 → 自动重连{C.NC}")

        if not self._ros_ok:
            self._skip("T8-5", "ROS2 不可用")
            return

        connect_count = [0]
        connect_lock = threading.Lock()

        pb2 = self.pb2
        pb2_grpc = self.pb2_grpc
        imu_template = [self._make_imu() for _ in range(5)]

        class RecoveryServicer(pb2_grpc.CmsServicer):
            async def Enable(self, req, ctx):
                return pb2.Empty()

            async def Disable(self, req, ctx):
                return pb2.Empty()

            async def StandUp(self, req, ctx):
                return pb2.Empty()

            async def SitDown(self, req, ctx):
                return pb2.Empty()

            async def Walk(self, req, ctx):
                return pb2.Empty()

            async def ListenImu(self, req, ctx):
                with connect_lock:
                    connect_count[0] += 1
                    n = connect_count[0]

                if n == 1:
                    # 第一次: 发 3 条然后抛出异常 (模拟流中断)
                    for imu in imu_template[:3]:
                        yield imu
                        await asyncio.sleep(0.02)
                    raise RuntimeError("模拟 IMU 流中断")
                else:
                    # 后续: 正常流
                    for imu in imu_template:
                        yield imu
                        await asyncio.sleep(0.02)

            async def ListenJoint(self, req, ctx):
                return
                yield

            async def ListenHistory(self, req, ctx):
                return
                yield

        runner = MockServerRunner(pb2_grpc, RecoveryServicer(), port=self.MOCK_PORT)
        runner.start()

        proc = subprocess.Popen(
            [
                'ros2', 'run', 'robot_driver', 'han_dog_bridge',
                '--ros-args',
                '-p', 'dog_host:=localhost',
                '-p', f'dog_port:={self.MOCK_PORT}',
                '-p', 'auto_enable:=false',
                '-p', 'auto_standup:=false',
                '-p', 'reconnect_interval:=0.5',  # 快速重连
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )

        # 等待第一次连接 + 断开 + 重连
        time.sleep(4.0)

        proc.terminate()
        proc.wait(timeout=5)
        runner.stop()

        reconnects = connect_count[0]
        print(f"  ListenImu 被调用 {reconnects} 次 (期望 ≥ 2)")

        if reconnects >= 2:
            self._ok(f"T8-5 流中断后重连 ({reconnects} 次连接)")
        else:
            self._fail("T8-5 流中断恢复",
                       f"ListenImu 只被调用 {reconnects} 次，期望 ≥ 2")

    # ── 汇总 ──────────────────────────────────────────────────────────────────
    def summary(self) -> bool:
        total = self.passed + self.failed + self.skipped
        print(f"\n{'='*60}")
        print(f"{C.BOLD}T8 han_dog_bridge 网络故障测试汇总{C.NC}")
        print(f"{'='*60}")
        print(f"  {C.GREEN}通过{C.NC}: {self.passed}")
        print(f"  {C.RED}失败{C.NC}: {self.failed}")
        print(f"  {C.YELLOW}跳过{C.NC}: {self.skipped}")
        print(f"  总计  : {total}")
        print(f"{'='*60}")
        if self.failed == 0:
            print(f"{C.GREEN}✅ T8 测试通过{C.NC}")
        else:
            print(f"{C.RED}❌ 有 {self.failed} 项失败{C.NC}")
        print()
        return self.failed == 0


# ─── 主入口 ───────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='T8: han_dog_bridge 网络故障集成测试')
    parser.add_argument('--real-dog', default=None, metavar='HOST:PORT',
                        help='改用真实 dog 地址 (跳过 mock server)')
    parser.add_argument('--mock-port', type=int, default=13200,
                        help='mock CMS 服务端口 (默认 13200)')
    args = parser.parse_args()

    print('=' * 60)
    print(f"{C.BOLD}  T8: han_dog_bridge 网络故障集成测试{C.NC}")
    print('=' * 60)

    # ── 编译 mock proto ──
    if args.real_dog:
        print(f"\n{C.YELLOW}模式: 真实 dog @ {args.real_dog} (仅 T8-1 reconnect 测试){C.NC}")
        pb2, pb2_grpc = None, None
    else:
        print("\n[准备] 编译 mock proto...")
        tmp_dir = tempfile.mkdtemp(prefix='han_dog_mock_')
        pb2_dir = compile_proto(tmp_dir)

        if pb2_dir is None:
            print(f"  {WARN} grpcio-tools 未安装，跳过所有测试")
            print("  安装: pip install grpcio-tools")
            sys.exit(0)

        pb2, pb2_grpc = load_mock_proto(pb2_dir)
        if pb2 is None:
            print(f"  {FAIL} 无法加载编译好的 proto stubs")
            sys.exit(1)

        print(f"  {PASS} mock proto 编译成功 → {tmp_dir}")

    suite = T8TestSuite(pb2, pb2_grpc, real_dog=args.real_dog)
    suite.MOCK_PORT = args.mock_port

    try:
        suite.test_t8_1_reconnect()
        if pb2 is not None:
            suite.test_t8_2_imu_to_odom()
            suite.test_t8_3_cmd_vel_to_walk()
            suite.test_t8_4_watchdog()
            suite.test_t8_5_stream_recovery()
    except KeyboardInterrupt:
        print(f"\n{C.YELLOW}[INTERRUPTED]{C.NC}")

    ok = suite.summary()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
