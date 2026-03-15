#!/usr/bin/env python3
"""
TaskScheduler — 定时任务调度节点

基于时间表自动触发巡检等任务，支持按星期/时间配置。

话题:
  订阅:
    /nav/schedule/command  (String JSON) — 调度操作指令
  发布:
    /nav/schedule/response (String JSON) — 操作结果
    /nav/patrol/command    (String JSON) — 触发巡检 (发给 PatrolManager)
"""

import os
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from semantic_common.sanitize import safe_json_dumps, safe_json_loads

try:
    import yaml
except ImportError:
    yaml = None


class TaskScheduler(Node):
    def __init__(self):
        super().__init__('task_scheduler')

        # 参数
        default_file = os.path.expanduser('~/.lingtu/schedules.yaml')
        self.declare_parameter('schedule_file', default_file)
        self._schedule_file = Path(self.get_parameter('schedule_file').value)
        self._schedule_file.parent.mkdir(parents=True, exist_ok=True)

        # 调度数据: {name: {patrol_route, hour, minute, weekdays, enabled}}
        self._schedules = self._load_schedules()

        # 记录每个 schedule 的上次触发时间 (minute 粒度)，避免重复触发
        # key: schedule_name, value: "YYYY-MM-DD HH:MM"
        self._last_fired = {}

        # 订阅
        self.create_subscription(String, '/nav/schedule/command', self._on_cmd, 10)

        # 发布
        self._resp_pub = self.create_publisher(String, '/nav/schedule/response', 10)
        self._patrol_cmd_pub = self.create_publisher(String, '/nav/patrol/command', 10)

        # 每 30 秒检查一次调度
        self.create_timer(30.0, self._check_schedules)

        self.get_logger().info(
            f'TaskScheduler 启动 | file={self._schedule_file} | '
            f'已加载 {len(self._schedules)} 个调度'
        )

    def _on_cmd(self, msg: String):
        cmd = safe_json_loads(msg.data, default={})
        action = cmd.get('action', '')
        resp = {'action': action, 'success': False}

        try:
            if action == 'add':
                resp = self._add_schedule(cmd)
            elif action == 'remove':
                resp = self._remove_schedule(cmd.get('name', ''))
            elif action == 'list':
                resp = self._list_schedules()
            elif action == 'enable':
                resp = self._set_enabled(cmd.get('name', ''), True)
            elif action == 'disable':
                resp = self._set_enabled(cmd.get('name', ''), False)
            else:
                resp['message'] = f'未知操作: {action}'
        except Exception as e:
            resp['message'] = str(e)
            self.get_logger().error(f'调度操作异常: {e}')

        out = String()
        out.data = safe_json_dumps(resp)
        self._resp_pub.publish(out)

    def _add_schedule(self, cmd: dict) -> dict:
        name = cmd.get('name', '')
        if not name:
            return {'action': 'add', 'success': False, 'message': '缺少调度名称'}

        patrol_route = cmd.get('patrol_route', '')
        if not patrol_route:
            return {'action': 'add', 'success': False, 'message': '缺少巡检路线名称'}

        hour = int(cmd.get('hour', 0))
        minute = int(cmd.get('minute', 0))
        weekdays = cmd.get('weekdays', [0, 1, 2, 3, 4, 5, 6])
        enabled = cmd.get('enabled', True)

        self._schedules[name] = {
            'patrol_route': patrol_route,
            'hour': hour,
            'minute': minute,
            'weekdays': weekdays,
            'enabled': enabled,
        }
        self._save_schedules()
        self.get_logger().info(
            f'调度已添加: {name} | route={patrol_route} | '
            f'{hour:02d}:{minute:02d} | weekdays={weekdays}'
        )
        return {'action': 'add', 'success': True, 'message': f'调度已添加: {name}'}

    def _remove_schedule(self, name: str) -> dict:
        if not name:
            return {'action': 'remove', 'success': False, 'message': '缺少调度名称'}
        if name not in self._schedules:
            return {'action': 'remove', 'success': False, 'message': f'调度不存在: {name}'}

        del self._schedules[name]
        self._last_fired.pop(name, None)
        self._save_schedules()
        self.get_logger().info(f'调度已删除: {name}')
        return {'action': 'remove', 'success': True, 'message': f'调度已删除: {name}'}

    def _list_schedules(self) -> dict:
        items = []
        for name, data in self._schedules.items():
            items.append({
                'name': name,
                'patrol_route': data.get('patrol_route', ''),
                'hour': data.get('hour', 0),
                'minute': data.get('minute', 0),
                'weekdays': data.get('weekdays', []),
                'enabled': data.get('enabled', True),
                'last_fired': self._last_fired.get(name, ''),
            })
        return {'action': 'list', 'success': True, 'schedules': items}

    def _set_enabled(self, name: str, enabled: bool) -> dict:
        action = 'enable' if enabled else 'disable'
        if not name:
            return {'action': action, 'success': False, 'message': '缺少调度名称'}
        if name not in self._schedules:
            return {'action': action, 'success': False, 'message': f'调度不存在: {name}'}

        self._schedules[name]['enabled'] = enabled
        self._save_schedules()
        state = '启用' if enabled else '禁用'
        self.get_logger().info(f'调度已{state}: {name}')
        return {'action': action, 'success': True, 'message': f'调度已{state}: {name}'}

    # ── 定时检查 ──────────────────────────────────────

    def _check_schedules(self):
        now = datetime.now()
        current_weekday = now.weekday()  # 0=Monday, 6=Sunday
        current_hour = now.hour
        current_minute = now.minute
        minute_key = now.strftime('%Y-%m-%d %H:%M')

        for name, data in self._schedules.items():
            if not data.get('enabled', True):
                continue

            # 检查星期
            if current_weekday not in data.get('weekdays', []):
                continue

            # 检查时间
            if current_hour != data.get('hour', -1) or current_minute != data.get('minute', -1):
                continue

            # 检查是否已在同一分钟内触发过
            if self._last_fired.get(name) == minute_key:
                continue

            # 触发巡检
            self._last_fired[name] = minute_key
            patrol_route = data.get('patrol_route', '')

            patrol_cmd = String()
            patrol_cmd.data = safe_json_dumps({
                'action': 'start',
                'name': patrol_route,
            })
            self._patrol_cmd_pub.publish(patrol_cmd)

            self.get_logger().info(f'调度触发: {name} -> 巡检路线 {patrol_route}')

    # ── 持久化 ────────────────────────────────────────

    def _load_schedules(self) -> dict:
        if not self._schedule_file.exists():
            return {}
        try:
            with open(self._schedule_file, 'r', encoding='utf-8') as f:
                if yaml is not None:
                    data = yaml.safe_load(f)
                else:
                    import json
                    data = json.load(f)
                return data if isinstance(data, dict) else {}
        except Exception as e:
            self.get_logger().warn(f'加载调度文件失败: {e}')
            return {}

    def _save_schedules(self):
        try:
            self._schedule_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self._schedule_file, 'w', encoding='utf-8') as f:
                if yaml is not None:
                    yaml.safe_dump(self._schedules, f, allow_unicode=True, default_flow_style=False)
                else:
                    import json
                    json.dump(self._schedules, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.get_logger().error(f'保存调度文件失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TaskScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
