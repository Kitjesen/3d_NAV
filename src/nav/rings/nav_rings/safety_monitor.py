#!/usr/bin/env python3
"""
Ring 1 — SafetyMonitor (反射弧)
================================
聚合所有安全信号，监控关键链路心跳，统一对外报告安全状态。

设计原则:
  - 内环最高优先级: 可以打断外环，不可被外环绕过
  - 始终运行: 不依赖任何规划/感知节点
  - 观察者 + 升级者: 常态下只观察，链路断裂时主动升级

话题:
  订阅:
    /nav/stop              (Int8)         — local_planner 碰撞急停
    /nav/slow_down         (Int8)         — 地形减速
    /driver/watchdog_active (Bool)        — 驱动看门狗
    /nav/planner_status    (String)       — 卡死检测 (WARN_STUCK/STUCK)
    /nav/odometry          (Odometry)     — SLAM 心跳
    /nav/terrain_map       (PointCloud2 header) — 地形分析心跳
    /nav/cmd_vel           (TwistStamped) — pathFollower 心跳

  发布:
    /nav/safety_state      (String JSON)  — 统一安全状态
    /nav/stop              (Int8)         — 升级: 关键链路断裂时发 stop=2
"""

import math
import time
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Int8, String, Float32

from core.utils.sanitize import safe_json_dumps


class SafetyLevel(Enum):
    OK = "OK"               # 所有链路正常，无安全信号
    DEGRADED = "DEGRADED"   # 非关键链路超时 (terrain_map)
    WARN = "WARN"           # stuck 预警或减速中
    DANGER = "DANGER"       # 关键链路断裂或碰撞急停
    ESTOP = "ESTOP"         # 硬件急停 (stop=2 持续)


@dataclass
class LinkStatus:
    """一条被监控的链路。"""
    name: str
    topic: str
    last_seen: float = 0.0      # time.monotonic()
    timeout_sec: float = 2.0
    critical: bool = True        # 超时是否升级到 DANGER

    @property
    def alive(self) -> bool:
        if self.last_seen == 0.0:
            return True  # 尚未收到第一帧，不算超时
        return (time.monotonic() - self.last_seen) < self.timeout_sec

    @property
    def stale_sec(self) -> float:
        if self.last_seen == 0.0:
            return 0.0
        return time.monotonic() - self.last_seen


class SafetyMonitor(Node):
    """Ring 1: 反射弧安全聚合器。"""

    def __init__(self):
        super().__init__('safety_monitor')

        # ── 参数 ──
        self.declare_parameter('monitor_hz', 20.0)
        self.declare_parameter('odom_timeout_sec', 2.0)
        self.declare_parameter('driver_timeout_sec', 2.0)
        self.declare_parameter('terrain_timeout_sec', 5.0)
        self.declare_parameter('cmdvel_timeout_sec', 3.0)
        self.declare_parameter('escalation_holdoff_sec', 1.0)
        self.declare_parameter('relocalize_cooldown_sec', 30.0)
        self.declare_parameter('loc_quality_warn_threshold', 0.3)

        hz = self.get_parameter('monitor_hz').value
        odom_to = self.get_parameter('odom_timeout_sec').value
        driver_to = self.get_parameter('driver_timeout_sec').value
        terrain_to = self.get_parameter('terrain_timeout_sec').value
        cmdvel_to = self.get_parameter('cmdvel_timeout_sec').value
        self._holdoff = self.get_parameter('escalation_holdoff_sec').value
        self._relocalize_cooldown = self.get_parameter('relocalize_cooldown_sec').value
        self._loc_quality_warn = self.get_parameter('loc_quality_warn_threshold').value

        # ── 被监控的链路 ──
        self._links: Dict[str, LinkStatus] = {
            'slam': LinkStatus(
                name='SLAM', topic='/nav/odometry',
                timeout_sec=odom_to, critical=True),
            'driver': LinkStatus(
                name='Driver', topic='/driver/watchdog_active',
                timeout_sec=driver_to, critical=True),
            'terrain': LinkStatus(
                name='Terrain', topic='/nav/terrain_map',
                timeout_sec=terrain_to, critical=False),
            'cmdvel': LinkStatus(
                name='PathFollower', topic='/nav/cmd_vel',
                timeout_sec=cmdvel_to, critical=False),
        }

        # ── 安全信号缓存 ──
        self._stop_level = 0       # 0=clear, 1=soft, 2=hard
        self._slow_down = 0
        self._watchdog_active = False
        self._planner_status = "IDLE"
        self._last_escalation = 0.0
        self._navigating = False    # 是否在导航中 (有 cmd_vel 活动)

        # ── 定位恢复 ──
        self._loc_quality = 1.0           # 最近的 ICP 匹配质量
        self._slam_was_alive = False      # SLAM 是否曾经活过
        self._last_relocalize_time = 0.0  # 上次 relocalize 尝试时间
        self._relocalize_client = None    # 延迟创建服务客户端

        # ── QoS ──
        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── 订阅 ──
        self.create_subscription(Int8, '/nav/stop', self._on_stop, 10)
        self.create_subscription(Int8, '/nav/slow_down', self._on_slow_down, 10)
        self.create_subscription(Bool, '/driver/watchdog_active', self._on_watchdog, 10)
        self.create_subscription(String, '/nav/planner_status', self._on_planner_status, 10)
        self.create_subscription(Odometry, '/nav/odometry', self._on_odom, be)
        self.create_subscription(PointCloud2, '/nav/terrain_map', self._on_terrain, be)
        self.create_subscription(TwistStamped, '/nav/cmd_vel', self._on_cmdvel, be)
        self.create_subscription(Float32, '/nav/localization_quality', self._on_loc_quality, 10)

        # ── 发布 ──
        self._pub_state = self.create_publisher(String, '/nav/safety_state', 10)
        self._pub_stop = self.create_publisher(Int8, '/nav/stop', 10)

        # ── 主循环 ──
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(f'SafetyMonitor started @ {hz}Hz')

    # ================================================================
    # 回调: 更新链路心跳和安全信号
    # ================================================================

    def _on_odom(self, msg: Odometry):
        self._links['slam'].last_seen = time.monotonic()
        self._slam_was_alive = True

    def _on_loc_quality(self, msg: Float32):
        if math.isfinite(msg.data):
            self._loc_quality = msg.data

    def _on_watchdog(self, msg: Bool):
        self._links['driver'].last_seen = time.monotonic()
        self._watchdog_active = msg.data

    def _on_terrain(self, msg: PointCloud2):
        self._links['terrain'].last_seen = time.monotonic()

    def _on_cmdvel(self, msg: TwistStamped):
        self._links['cmdvel'].last_seen = time.monotonic()
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self._navigating = (math.isfinite(vx) and math.isfinite(vy)
                            and (abs(vx) > 0.01 or abs(vy) > 0.01))

    def _on_stop(self, msg: Int8):
        self._stop_level = msg.data

    def _on_slow_down(self, msg: Int8):
        self._slow_down = msg.data

    def _on_planner_status(self, msg: String):
        self._planner_status = msg.data.strip()

    # ================================================================
    # 主循环: 评估 + 发布
    # ================================================================

    def _tick(self):
        now = time.monotonic()
        level = SafetyLevel.OK
        issues = []

        # 1. 检查链路心跳
        for key, link in self._links.items():
            if not link.alive and link.last_seen > 0:
                stale = link.stale_sec
                if link.critical:
                    level = max(level, SafetyLevel.DANGER, key=lambda x: x.value)
                    issues.append(f"{link.name} stale {stale:.1f}s (CRITICAL)")
                else:
                    if level.value < SafetyLevel.WARN.value:
                        level = SafetyLevel.DEGRADED
                    issues.append(f"{link.name} stale {stale:.1f}s")

        # 2. 检查安全信号
        if self._stop_level >= 2:
            level = SafetyLevel.ESTOP
            issues.append("stop=2 (collision/estop)")
        elif self._stop_level == 1:
            if level.value < SafetyLevel.WARN.value:
                level = SafetyLevel.WARN
            issues.append("stop=1 (soft stop)")

        if self._watchdog_active:
            if level.value < SafetyLevel.WARN.value:
                level = SafetyLevel.WARN
            issues.append("watchdog active (no cmd_vel)")

        if self._planner_status == "STUCK":
            if level.value < SafetyLevel.WARN.value:
                level = SafetyLevel.WARN
            issues.append("STUCK detected")
        elif self._planner_status == "WARN_STUCK":
            if level.value < SafetyLevel.WARN.value:
                level = SafetyLevel.WARN
            issues.append("WARN_STUCK")

        if self._slow_down > 0:
            if level.value < SafetyLevel.DEGRADED.value:
                level = SafetyLevel.DEGRADED
            issues.append(f"slow_down={self._slow_down}")

        # 2b. 定位质量检查
        if self._loc_quality < self._loc_quality_warn and self._loc_quality > 0:
            if level.value < SafetyLevel.WARN.value:
                level = SafetyLevel.WARN
            issues.append(f"loc_quality={self._loc_quality:.2f} (low)")

        # 2c. SLAM 丢失恢复: 曾经活过但现在断了 → 尝试 relocalize
        slam_link = self._links['slam']
        if (self._slam_was_alive
                and not slam_link.alive
                and slam_link.last_seen > 0
                and (now - self._last_relocalize_time) > self._relocalize_cooldown):
            self._try_relocalize()

        # 3. 升级: 关键链路断裂 + 正在导航 → 主动发 stop=2
        if (level in (SafetyLevel.DANGER, SafetyLevel.ESTOP)
                and self._navigating
                and (now - self._last_escalation) > self._holdoff):
            self._last_escalation = now
            stop_msg = Int8()
            stop_msg.data = 2
            self._pub_stop.publish(stop_msg)
            self.get_logger().error(
                f'SAFETY ESCALATION → stop=2: {"; ".join(issues)}')

        # 4. 发布状态
        links_summary = {}
        for key, link in self._links.items():
            links_summary[key] = {
                'alive': link.alive or link.last_seen == 0.0,
                'stale_sec': round(link.stale_sec, 1) if link.last_seen > 0 else None,
            }

        state = {
            'level': level.value,
            'navigating': self._navigating,
            'stop': self._stop_level,
            'slow_down': self._slow_down,
            'watchdog': self._watchdog_active,
            'planner': self._planner_status,
            'links': links_summary,
            'issues': issues if issues else None,
        }

        msg = String()
        msg.data = safe_json_dumps(state)
        self._pub_state.publish(msg)

        # 5. 状态变化时打日志
        if issues and not hasattr(self, '_prev_issues'):
            self._prev_issues = []
        if issues != getattr(self, '_prev_issues', []):
            self._prev_issues = issues
            if level.value >= SafetyLevel.WARN.value:
                self.get_logger().warn(f'Safety {level.value}: {"; ".join(issues)}')
            elif issues:
                self.get_logger().info(f'Safety {level.value}: {"; ".join(issues)}')


    def _try_relocalize(self):
        """SLAM 心跳丢失 → 尝试调用 relocalize 服务"""
        self._last_relocalize_time = time.monotonic()

        # 延迟创建服务客户端 (避免启动时 interface 包未安装导致崩溃)
        if self._relocalize_client is None:
            try:
                from interface.srv import Relocalize
                self._relocalize_client = self.create_client(
                    Relocalize, '/nav/relocalize')
            except (ImportError, Exception) as e:
                self.get_logger().warn(
                    f'Cannot create relocalize client: {e}. '
                    f'Localization recovery disabled.')
                self._relocalize_cooldown = float('inf')  # 永不重试
                return

        if not self._relocalize_client.service_is_ready():
            self.get_logger().info(
                'Relocalize service not ready, will retry later')
            return

        try:
            from interface.srv import Relocalize
            req = Relocalize.Request()
            future = self._relocalize_client.call_async(req)
            self.get_logger().warn(
                'SLAM heartbeat lost — relocalize request sent')
        except Exception as e:
            self.get_logger().error(f'Relocalize call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
