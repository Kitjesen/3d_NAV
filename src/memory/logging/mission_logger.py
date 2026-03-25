#!/usr/bin/env python3
"""
MissionLogger — 任务历史记录节点

记录每次导航任务的生命周期: 开始/结束时间、轨迹、距离、结果等。
支持查询历史记录。

话题:
  订阅:
    /nav/mission_status  (String JSON) — 任务状态 (来自 mission_arc)
    /nav/odometry        (Odometry)    — 轨迹记录
    /nav/dialogue_state  (String JSON) — 对话状态快照
    /nav/history/query   (String JSON) — 历史查询指令
  发布:
    /nav/history/response (String JSON) — 查询结果
"""

import os
import math
import json
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from core.utils.sanitize import safe_json_dumps, safe_json_loads


# 最大保留任务记录数
MAX_RECORDS = 500

# 轨迹采样最小间隔 (秒)
TRAJECTORY_SAMPLE_INTERVAL = 1.0


class MissionLogger(Node):
    def __init__(self):
        super().__init__('mission_logger')

        # 参数
        default_log_dir = os.path.expanduser('~/.lingtu/mission_history')
        self.declare_parameter('log_dir', default_log_dir)
        self._log_dir = Path(self.get_parameter('log_dir').value)
        self._log_dir.mkdir(parents=True, exist_ok=True)

        # 当前任务记录
        self._current_mission = None  # 活跃任务的 dict
        self._last_odom_time = 0.0
        self._last_x = 0.0
        self._last_y = 0.0
        self._total_distance = 0.0
        self._latest_dialogue_state = None

        # 订阅
        self.create_subscription(String, '/nav/mission_status', self._on_mission_status, 10)
        self.create_subscription(Odometry, '/nav/odometry', self._on_odom, 10)
        self.create_subscription(String, '/nav/dialogue_state', self._on_dialogue_state, 10)
        self.create_subscription(String, '/nav/history/query', self._on_query, 10)

        # 发布
        self._resp_pub = self.create_publisher(String, '/nav/history/response', 10)

        self.get_logger().info(f'MissionLogger 启动 | log_dir={self._log_dir}')

    # ── 任务状态监听 ──────────────────────────────────

    def _on_mission_status(self, msg: String):
        status = safe_json_loads(msg.data, default={})
        state = status.get('state', '')

        # IDLE -> PLANNING: 开始新任务
        if state == 'PLANNING' and self._current_mission is None:
            now = datetime.now(timezone.utc)
            self._current_mission = {
                'id': now.isoformat(),
                'start_time': now.isoformat(),
                'end_time': None,
                'result': 'IN_PROGRESS',
                'duration_sec': 0,
                'distance_m': 0.0,
                'route_name': status.get('route_name', ''),
                'goal': status.get('goal', {}),
                'replan_count': 0,
                'trajectory': [],
                'final_dialogue_state': None,
            }
            self._total_distance = 0.0
            self._last_odom_time = 0.0
            self.get_logger().info(f'任务开始: {self._current_mission["id"]}')

        # 累计 replan 次数
        if state == 'REPLANNING' and self._current_mission is not None:
            self._current_mission['replan_count'] += 1

        # COMPLETE / FAILED: 结束任务
        if state in ('COMPLETE', 'FAILED') and self._current_mission is not None:
            now = datetime.now(timezone.utc)
            self._current_mission['end_time'] = now.isoformat()
            self._current_mission['result'] = state
            self._current_mission['distance_m'] = round(self._total_distance, 2)
            self._current_mission['final_dialogue_state'] = self._latest_dialogue_state

            # 计算时长
            try:
                start = datetime.fromisoformat(self._current_mission['start_time'])
                duration = (now - start).total_seconds()
                self._current_mission['duration_sec'] = round(duration, 1)
            except Exception:
                pass

            self._save_mission(self._current_mission)
            self._enforce_max_records()

            self.get_logger().info(
                f'任务结束: {self._current_mission["id"]} | '
                f'result={state} | '
                f'duration={self._current_mission["duration_sec"]}s | '
                f'distance={self._current_mission["distance_m"]}m'
            )

            self._current_mission = None
            self._latest_dialogue_state = None

    # ── 轨迹记录 ─────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        if self._current_mission is None:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # 累计移动距离
        if self._last_odom_time > 0.0:
            dx = x - self._last_x
            dy = y - self._last_y
            dist = math.sqrt(dx * dx + dy * dy)
            # 过滤跳变 (> 5m 视为异常)
            if dist < 5.0:
                self._total_distance += dist

        # 降采样到 ~1Hz
        if now_sec - self._last_odom_time >= TRAJECTORY_SAMPLE_INTERVAL:
            self._current_mission['trajectory'].append([
                round(x, 3),
                round(y, 3),
                round(now_sec, 2),
            ])
            self._last_odom_time = now_sec

        self._last_x = x
        self._last_y = y

    # ── 对话状态快照 ──────────────────────────────────

    def _on_dialogue_state(self, msg: String):
        self._latest_dialogue_state = safe_json_loads(msg.data, default=None)

    # ── 历史查询 ──────────────────────────────────────

    def _on_query(self, msg: String):
        cmd = safe_json_loads(msg.data, default={})
        action = cmd.get('action', '')
        resp = {'action': action, 'success': False}

        try:
            if action == 'list':
                resp = self._query_list(cmd.get('count', 10))
            elif action == 'detail':
                resp = self._query_detail(cmd.get('id', ''))
            else:
                resp['message'] = f'未知查询操作: {action}'
        except Exception as e:
            resp['message'] = str(e)
            self.get_logger().error(f'历史查询异常: {e}')

        out = String()
        out.data = safe_json_dumps(resp)
        self._resp_pub.publish(out)

    def _query_list(self, count: int) -> dict:
        """列出最近 N 条任务摘要 (不含轨迹)"""
        files = sorted(self._log_dir.glob('*.json'), reverse=True)
        missions = []
        for f in files[:count]:
            try:
                data = json.loads(f.read_text(encoding='utf-8'))
                # 返回摘要，不包含轨迹
                missions.append({
                    'id': data.get('id', ''),
                    'start_time': data.get('start_time', ''),
                    'end_time': data.get('end_time', ''),
                    'result': data.get('result', ''),
                    'duration_sec': data.get('duration_sec', 0),
                    'distance_m': data.get('distance_m', 0.0),
                    'route_name': data.get('route_name', ''),
                })
            except Exception as e:
                self.get_logger().warn(f'读取任务记录失败: {f.name}: {e}')
        return {'action': 'list', 'success': True, 'missions': missions}

    def _query_detail(self, mission_id: str) -> dict:
        """查询单条任务详情 (含轨迹)"""
        if not mission_id:
            return {'action': 'detail', 'success': False, 'message': '缺少任务 ID'}

        # 用 ID (ISO timestamp) 找文件
        safe_name = mission_id.replace(':', '-')
        candidates = list(self._log_dir.glob(f'{safe_name}*.json'))

        if not candidates:
            # 尝试遍历找 id 匹配
            for f in self._log_dir.glob('*.json'):
                try:
                    data = json.loads(f.read_text(encoding='utf-8'))
                    if data.get('id') == mission_id:
                        return {'action': 'detail', 'success': True, 'mission': data}
                except Exception:
                    continue
            return {'action': 'detail', 'success': False, 'message': f'任务不存在: {mission_id}'}

        try:
            data = json.loads(candidates[0].read_text(encoding='utf-8'))
            return {'action': 'detail', 'success': True, 'mission': data}
        except Exception as e:
            return {'action': 'detail', 'success': False, 'message': str(e)}

    # ── 持久化 ────────────────────────────────────────

    def _save_mission(self, mission: dict):
        """保存单条任务记录为 JSON 文件"""
        # 文件名: ISO 时间戳，冒号替换为连字符 (Windows 兼容)
        safe_id = mission['id'].replace(':', '-')
        path = self._log_dir / f'{safe_id}.json'
        try:
            with open(path, 'w', encoding='utf-8') as f:
                json.dump(mission, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.get_logger().error(f'保存任务记录失败: {e}')

    def _enforce_max_records(self):
        """保持最多 MAX_RECORDS 条记录，超出则删除最老的"""
        files = sorted(self._log_dir.glob('*.json'))
        if len(files) > MAX_RECORDS:
            to_delete = files[:len(files) - MAX_RECORDS]
            for f in to_delete:
                try:
                    f.unlink()
                except OSError as e:
                    self.get_logger().warn(f'删除旧记录失败: {f.name}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
