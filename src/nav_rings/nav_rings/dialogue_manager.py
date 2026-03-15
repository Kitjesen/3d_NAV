#!/usr/bin/env python3
"""
Ring 3 — DialogueManager (对话环)
==================================
聚合所有状态信息，提供统一的用户交互状态出口。
用户（Flutter App / gRPC / 日志）只需订阅一个话题就能理解机器人在干什么。

产品承诺: 机器人能用一句话说清自己的状态。

话题:
  订阅:
    /nav/mission_status    (String JSON)  — 任务 FSM
    /nav/planner_status    (String)       — 规划器状态
    /nav/adapter_status    (String JSON)  — 航点进度
    /nav/semantic/status   (String JSON)  — 语义理解状态
    /nav/safety_state      (String JSON)  — SafetyMonitor
    /nav/execution_eval    (String JSON)  — Evaluator
    /nav/odometry          (Odometry)     — 位置
    /nav/semantic/instruction (String)    — 用户指令

  发布:
    /nav/dialogue_state    (String JSON)  — 统一认知状态
"""

import json
import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from std_msgs.msg import String

from semantic_common.sanitize import safe_json_dumps, safe_json_loads


# ── 状态描述模板 (中文, 面向终端用户) ──

_ACTION_TEXT = {
    'IDLE':        '待命中',
    'PLANNING':    '正在规划路线',
    'EXECUTING':   '正在前往目标',
    'RECOVERING':  '遇到障碍，尝试恢复',
    'REPLANNING':  '重新规划路线',
    'COMPLETE':    '已到达目标',
    'FAILED':      '任务失败',
}

_ASSESSMENT_TEXT = {
    'IDLE':        '',
    'ON_TRACK':    '路线跟踪正常',
    'DRIFTING':    '偏离规划路线，正在修正',
    'STALLED':     '前进受阻，等待恢复',
    'REGRESSING':  '偏离目标，可能需要重新规划',
}

_SAFETY_TEXT = {
    'OK':       '安全',
    'DEGRADED': '部分传感器离线',
    'WARN':     '注意: 减速或障碍预警',
    'DANGER':   '危险: 关键链路异常',
    'ESTOP':    '急停',
}


class DialogueManager(Node):
    """Ring 3: 对话环统一状态管理器。"""

    def __init__(self):
        super().__init__('dialogue_manager')

        # ── 参数 ──
        self.declare_parameter('publish_hz', 2.0)

        hz = self.get_parameter('publish_hz').value

        # ── 缓存的各方状态 ──
        self._mission: Dict = {}         # from /nav/mission_status
        self._planner_status = 'IDLE'    # from /nav/planner_status
        self._adapter: Dict = {}         # from /nav/adapter_status (latest event)
        self._semantic: Dict = {}        # from /nav/semantic/status
        self._safety: Dict = {}          # from /nav/safety_state
        self._eval: Dict = {}            # from /nav/execution_eval
        self._instruction = ''           # 最近的用户指令
        self._robot_xy = (0.0, 0.0)
        self._last_response = ''        # from /nav/voice/response
        self._last_response_time = 0.0  # timestamp

        # ── QoS ──
        be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        # ── 订阅 ──
        self.create_subscription(String, '/nav/mission_status', self._on_mission, 10)
        self.create_subscription(String, '/nav/planner_status', self._on_planner, 10)
        self.create_subscription(String, '/nav/adapter_status', self._on_adapter, 10)
        self.create_subscription(String, '/nav/semantic/status', self._on_semantic, 10)
        self.create_subscription(String, '/nav/safety_state', self._on_safety, 10)
        self.create_subscription(String, '/nav/execution_eval', self._on_eval, 10)
        self.create_subscription(String, '/nav/semantic/instruction', self._on_instruction, 10)
        self.create_subscription(String, '/nav/voice/response', self._on_response, 10)
        self.create_subscription(Odometry, '/nav/odometry', self._on_odom, be)

        # ── 发布 ──
        self._pub = self.create_publisher(String, '/nav/dialogue_state', 10)

        # ── 定时器 ──
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(f'DialogueManager started @ {hz}Hz')

    # ================================================================
    # 回调: 缓存各方状态
    # ================================================================

    def _on_mission(self, msg: String):
        data = safe_json_loads(msg.data)
        if data:
            self._mission = data

    def _on_planner(self, msg: String):
        self._planner_status = msg.data.strip()

    def _on_adapter(self, msg: String):
        data = safe_json_loads(msg.data)
        if data:
            self._adapter = data

    def _on_semantic(self, msg: String):
        data = safe_json_loads(msg.data)
        if data:
            self._semantic = data

    def _on_safety(self, msg: String):
        data = safe_json_loads(msg.data)
        if data:
            self._safety = data

    def _on_eval(self, msg: String):
        data = safe_json_loads(msg.data)
        if data:
            self._eval = data

    def _on_instruction(self, msg: String):
        text = msg.data.strip()
        if text:
            self._instruction = text

    def _on_response(self, msg: String):
        data = safe_json_loads(msg.data)
        if data and isinstance(data, dict):
            self._last_response = data.get('response', '')
            self._last_response_time = data.get('timestamp', time.time())
        elif msg.data.strip():
            self._last_response = msg.data.strip()
            self._last_response_time = time.time()

    def _on_odom(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.isfinite(x) and math.isfinite(y):
            self._robot_xy = (x, y)

    # ================================================================
    # 聚合逻辑
    # ================================================================

    def _build_state(self) -> Dict:
        """从各方缓存聚合成统一的对话状态。"""

        # 任务状态
        mission_state = self._mission.get('state', 'IDLE')
        action_text = _ACTION_TEXT.get(mission_state, mission_state)

        # 用户意图
        understood = self._instruction or self._semantic.get('instruction', '')
        if not understood:
            goal = self._mission.get('goal')
            if goal:
                understood = f"前往 ({goal.get('x', 0)}, {goal.get('y', 0)})"

        # 进度
        progress = self._mission.get('progress', {})
        progress_pct = progress.get('percent', 0) if progress else 0

        # 距离
        dist = self._mission.get('distance_to_goal')
        if dist is None:
            dist = self._eval.get('distance_to_goal')

        # 执行评估
        assessment = self._eval.get('assessment', 'IDLE')
        eval_text = _ASSESSMENT_TEXT.get(assessment, '')

        # 安全
        safety_level = self._safety.get('level', 'OK')
        safety_text = _SAFETY_TEXT.get(safety_level, safety_level)
        safety_issues = self._safety.get('issues')

        # 当前问题 (优先级: 安全 > 评估 > 规划)
        issue = None
        if safety_level in ('DANGER', 'ESTOP'):
            issues = safety_issues or []
            issue = '; '.join(issues[:2]) if issues else '安全异常'
        elif assessment in ('STALLED', 'REGRESSING'):
            issue = eval_text
        elif assessment == 'DRIFTING':
            cte = self._eval.get('cross_track_error', 0)
            issue = f'偏离路线 {cte:.1f}m' if cte else eval_text
        elif safety_level == 'WARN':
            issues = safety_issues or []
            issue = '; '.join(issues[:2]) if issues else '预警'
        elif self._planner_status == 'FAILED':
            failure = self._mission.get('failure_reason', '规划失败')
            issue = failure

        # ETA 估算 (粗略: 距离 / 平均速度)
        eta_sec = None
        if dist and dist > 0.5 and mission_state == 'EXECUTING':
            progress_rate = self._eval.get('progress_rate', 0)
            if progress_rate < -0.05:  # 正在接近 (负值)
                eta_sec = round(dist / abs(progress_rate))
                eta_sec = min(eta_sec, 9999)  # 上限

        # Voice response (only include if recent — within last 30 seconds)
        response = None
        if self._last_response and (time.time() - self._last_response_time) < 30.0:
            response = self._last_response

        # 组装描述句
        doing = action_text
        if mission_state == 'EXECUTING' and eval_text:
            doing = f'{action_text} — {eval_text}'

        state = {
            'understood': understood or None,
            'doing': doing,
            'progress_pct': round(progress_pct, 1),
            'distance_m': round(dist, 1) if dist is not None else None,
            'issue': issue,
            'safety': safety_level,
            'safety_text': safety_text,
            'eta_sec': eta_sec,
            'mission_state': mission_state,
            'response': response,
            'position': {
                'x': round(self._robot_xy[0], 2),
                'y': round(self._robot_xy[1], 2),
            },
        }

        return state

    def _tick(self):
        state = self._build_state()

        msg = String()
        msg.data = safe_json_dumps(state)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DialogueManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
