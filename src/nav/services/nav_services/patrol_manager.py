#!/usr/bin/env python3
"""
PatrolManager — 巡检路线管理节点

管理巡检路线的保存/加载/列表/删除，以及启动/停止巡检任务。

话题:
  订阅:
    /nav/patrol/command  (String JSON) — 巡检操作指令
  发布:
    /nav/patrol/response (String JSON) — 操作结果
    /nav/patrol_goals    (String JSON) — 发送航点列表给 mission_arc
    /nav/cancel          (Empty)       — 取消当前任务
"""

import os
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty

from semantic.common.semantic_common.sanitize import safe_json_dumps, safe_json_loads

try:
    import yaml
except ImportError:
    yaml = None


class PatrolManager(Node):
    def __init__(self):
        super().__init__('patrol_manager')

        # 参数
        default_routes_dir = os.path.expanduser('~/.lingtu/patrol_routes')
        self.declare_parameter('routes_dir', default_routes_dir)
        self._routes_dir = Path(self.get_parameter('routes_dir').value)
        self._routes_dir.mkdir(parents=True, exist_ok=True)

        # 当前巡检状态
        self._active_route = None  # 当前正在执行的路线名

        # 订阅
        self.create_subscription(String, '/nav/patrol/command', self._on_cmd, 10)

        # 发布
        self._resp_pub = self.create_publisher(String, '/nav/patrol/response', 10)
        self._goals_pub = self.create_publisher(String, '/nav/patrol_goals', 10)
        self._cancel_pub = self.create_publisher(Empty, '/nav/cancel', 10)

        self.get_logger().info(f'PatrolManager 启动 | routes_dir={self._routes_dir}')

    def _on_cmd(self, msg: String):
        cmd = safe_json_loads(msg.data, default={})
        action = cmd.get('action', '')
        resp = {'action': action, 'success': False}

        try:
            if action == 'save':
                resp = self._save_route(cmd)
            elif action == 'load':
                resp = self._load_route(cmd.get('name', ''))
            elif action == 'list':
                resp = self._list_routes()
            elif action == 'delete':
                resp = self._delete_route(cmd.get('name', ''))
            elif action == 'start':
                resp = self._start_patrol(cmd.get('name', ''))
            elif action == 'stop':
                resp = self._stop_patrol()
            else:
                resp['message'] = f'未知操作: {action}'
        except Exception as e:
            resp['message'] = str(e)
            self.get_logger().error(f'巡检操作异常: {e}')

        out = String()
        out.data = safe_json_dumps(resp)
        self._resp_pub.publish(out)

    def _save_route(self, cmd: dict) -> dict:
        name = cmd.get('name', '')
        if not name:
            return {'action': 'save', 'success': False, 'message': '缺少路线名称'}

        waypoints = cmd.get('waypoints', [])
        if not waypoints:
            return {'action': 'save', 'success': False, 'message': '航点列表为空'}

        route_data = {
            'name': name,
            'waypoints': waypoints,
            'loop': cmd.get('loop', False),
            'created': datetime.now(timezone.utc).isoformat(),
        }

        path = self._routes_dir / f'{name}.yaml'
        self._save_yaml(path, route_data)
        self.get_logger().info(f'路线已保存: {name} ({len(waypoints)} 个航点)')
        return {
            'action': 'save',
            'success': True,
            'message': f'路线已保存: {name} ({len(waypoints)} 个航点)',
        }

    def _load_route(self, name: str) -> dict:
        if not name:
            return {'action': 'load', 'success': False, 'message': '缺少路线名称'}

        path = self._routes_dir / f'{name}.yaml'
        if not path.exists():
            return {'action': 'load', 'success': False, 'message': f'路线不存在: {name}'}

        data = self._load_yaml(path)
        return {'action': 'load', 'success': True, 'route': data}

    def _list_routes(self) -> dict:
        routes = []
        for f in sorted(self._routes_dir.glob('*.yaml')):
            data = self._load_yaml(f)
            if data:
                routes.append({
                    'name': data.get('name', f.stem),
                    'waypoint_count': len(data.get('waypoints', [])),
                    'loop': data.get('loop', False),
                    'created': data.get('created', ''),
                })
        return {'action': 'list', 'success': True, 'routes': routes}

    def _delete_route(self, name: str) -> dict:
        if not name:
            return {'action': 'delete', 'success': False, 'message': '缺少路线名称'}

        path = self._routes_dir / f'{name}.yaml'
        if not path.exists():
            return {'action': 'delete', 'success': False, 'message': f'路线不存在: {name}'}

        try:
            path.unlink()
            self.get_logger().info(f'路线已删除: {name}')
            return {'action': 'delete', 'success': True, 'message': f'路线已删除: {name}'}
        except OSError as e:
            return {'action': 'delete', 'success': False, 'message': str(e)}

    def _start_patrol(self, name: str) -> dict:
        if not name:
            return {'action': 'start', 'success': False, 'message': '缺少路线名称'}

        path = self._routes_dir / f'{name}.yaml'
        if not path.exists():
            return {'action': 'start', 'success': False, 'message': f'路线不存在: {name}'}

        data = self._load_yaml(path)
        if not data or not data.get('waypoints'):
            return {'action': 'start', 'success': False, 'message': f'路线数据无效: {name}'}

        # 发送航点列表给 mission_arc
        goals_msg = String()
        goals_msg.data = safe_json_dumps({
            'waypoints': data['waypoints'],
            'loop': data.get('loop', False),
            'route_name': name,
        })
        self._goals_pub.publish(goals_msg)

        self._active_route = name
        self.get_logger().info(f'巡检已启动: {name} ({len(data["waypoints"])} 个航点)')
        return {
            'action': 'start',
            'success': True,
            'message': f'巡检已启动: {name}',
            'route_name': name,
        }

    def _stop_patrol(self) -> dict:
        self._cancel_pub.publish(Empty())
        route_name = self._active_route or '(无)'
        self._active_route = None
        self.get_logger().info(f'巡检已停止: {route_name}')
        return {'action': 'stop', 'success': True, 'message': f'巡检已停止: {route_name}'}

    # ── YAML 工具 ─────────────────────────────────────

    def _load_yaml(self, path: Path, default=None):
        if default is None:
            default = {}
        if not path.exists():
            return default
        try:
            with open(path, 'r', encoding='utf-8') as f:
                if yaml is not None:
                    data = yaml.safe_load(f)
                else:
                    import json
                    data = json.load(f)
                return data if data is not None else default
        except Exception as e:
            self.get_logger().warn(f'加载 {path} 失败: {e}')
            return default

    def _save_yaml(self, path: Path, data):
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                if yaml is not None:
                    yaml.safe_dump(data, f, allow_unicode=True, default_flow_style=False)
                else:
                    import json
                    json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.get_logger().error(f'保存 {path} 失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
