#!/usr/bin/env python3
"""
GeofenceManager — 电子围栏管理节点

管理禁区多边形的增删、启停，并以 1Hz 频率发布活跃围栏到 /nav/navigation_boundary。
同时监测机器人是否进入禁区并发布预警。

话题:
  订阅:
    /nav/geofence/command  (String JSON) — 围栏操作指令
    /nav/odometry          (Odometry)    — 机器人位姿
  发布:
    /nav/geofence/response       (String JSON)       — 操作结果
    /nav/navigation_boundary     (PolygonStamped)    — 活跃围栏多边形 (1Hz)
"""

import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PolygonStamped, Point32

from semantic_common.sanitize import safe_json_dumps, safe_json_loads

try:
    import yaml
except ImportError:
    yaml = None


class GeofenceManager(Node):
    def __init__(self):
        super().__init__('geofence_manager')

        # 参数
        default_file = os.path.expanduser('~/.lingtu/geofences.yaml')
        self.declare_parameter('geofence_file', default_file)
        self._geofence_file = Path(self.get_parameter('geofence_file').value)
        self._geofence_file.parent.mkdir(parents=True, exist_ok=True)

        # 围栏数据: {name: {polygon: [[x,y],...], enabled: bool}}
        self._fences = self._load_fences()

        # 缓存机器人位置
        self._robot_x = 0.0
        self._robot_y = 0.0

        # 订阅
        self.create_subscription(String, '/nav/geofence/command', self._on_cmd, 10)
        self.create_subscription(Odometry, '/nav/odometry', self._on_odom, 10)

        # 发布
        self._resp_pub = self.create_publisher(String, '/nav/geofence/response', 10)
        self._boundary_pub = self.create_publisher(PolygonStamped, '/nav/navigation_boundary', 10)

        # 1Hz 定时器发布围栏 + 检测入侵
        self.create_timer(1.0, self._publish_boundary)

        self.get_logger().info(
            f'GeofenceManager 启动 | file={self._geofence_file} | '
            f'已加载 {len(self._fences)} 个围栏'
        )

    def _on_cmd(self, msg: String):
        cmd = safe_json_loads(msg.data, default={})
        action = cmd.get('action', '')
        resp = {'action': action, 'success': False}

        try:
            if action == 'add':
                resp = self._add_fence(cmd)
            elif action == 'remove':
                resp = self._remove_fence(cmd.get('name', ''))
            elif action == 'list':
                resp = self._list_fences()
            elif action == 'clear':
                resp = self._clear_fences()
            elif action == 'enable':
                resp = self._set_enabled(cmd.get('name', ''), True)
            elif action == 'disable':
                resp = self._set_enabled(cmd.get('name', ''), False)
            else:
                resp['message'] = f'未知操作: {action}'
        except Exception as e:
            resp['message'] = str(e)
            self.get_logger().error(f'围栏操作异常: {e}')

        out = String()
        out.data = safe_json_dumps(resp)
        self._resp_pub.publish(out)

    def _add_fence(self, cmd: dict) -> dict:
        name = cmd.get('name', '')
        polygon = cmd.get('polygon', [])
        if not name:
            return {'action': 'add', 'success': False, 'message': '缺少围栏名称'}
        if len(polygon) < 3:
            return {'action': 'add', 'success': False, 'message': '多边形至少需要 3 个顶点'}

        self._fences[name] = {
            'polygon': polygon,
            'enabled': True,
        }
        self._save_fences()
        self.get_logger().info(f'围栏已添加: {name} ({len(polygon)} 个顶点)')
        return {'action': 'add', 'success': True, 'message': f'围栏已添加: {name}'}

    def _remove_fence(self, name: str) -> dict:
        if not name:
            return {'action': 'remove', 'success': False, 'message': '缺少围栏名称'}
        if name not in self._fences:
            return {'action': 'remove', 'success': False, 'message': f'围栏不存在: {name}'}

        del self._fences[name]
        self._save_fences()
        self.get_logger().info(f'围栏已删除: {name}')
        return {'action': 'remove', 'success': True, 'message': f'围栏已删除: {name}'}

    def _list_fences(self) -> dict:
        fences = []
        for name, data in self._fences.items():
            fences.append({
                'name': name,
                'vertex_count': len(data.get('polygon', [])),
                'enabled': data.get('enabled', True),
            })
        return {'action': 'list', 'success': True, 'fences': fences}

    def _clear_fences(self) -> dict:
        count = len(self._fences)
        self._fences.clear()
        self._save_fences()
        self.get_logger().info(f'已清除 {count} 个围栏')
        return {'action': 'clear', 'success': True, 'message': f'已清除 {count} 个围栏'}

    def _set_enabled(self, name: str, enabled: bool) -> dict:
        action = 'enable' if enabled else 'disable'
        if not name:
            return {'action': action, 'success': False, 'message': '缺少围栏名称'}
        if name not in self._fences:
            return {'action': action, 'success': False, 'message': f'围栏不存在: {name}'}

        self._fences[name]['enabled'] = enabled
        self._save_fences()
        state = '启用' if enabled else '禁用'
        self.get_logger().info(f'围栏已{state}: {name}')
        return {'action': action, 'success': True, 'message': f'围栏已{state}: {name}'}

    # ── 定时发布 + 入侵检测 ───────────────────────────

    def _publish_boundary(self):
        # 找到第一个启用的围栏发布 (localPlanner 只接受一个 polygon)
        active_fence = None
        for name, data in self._fences.items():
            if data.get('enabled', True):
                active_fence = (name, data)
                break

        if active_fence is not None:
            _, data = active_fence
            polygon_msg = PolygonStamped()
            polygon_msg.header.stamp = self.get_clock().now().to_msg()
            polygon_msg.header.frame_id = 'map'

            for pt in data['polygon']:
                p = Point32()
                p.x = float(pt[0])
                p.y = float(pt[1])
                p.z = 0.0
                polygon_msg.polygon.points.append(p)

            self._boundary_pub.publish(polygon_msg)

        # 入侵检测: 检查机器人是否在任何启用的禁区内
        for name, data in self._fences.items():
            if not data.get('enabled', True):
                continue
            if self._point_in_polygon(self._robot_x, self._robot_y, data['polygon']):
                warn_msg = String()
                warn_msg.data = safe_json_dumps({
                    'action': 'warning',
                    'type': 'intrusion',
                    'fence': name,
                    'robot_x': self._robot_x,
                    'robot_y': self._robot_y,
                    'message': f'机器人在禁区内: {name}',
                })
                self._resp_pub.publish(warn_msg)
                self.get_logger().warn(f'机器人在禁区内: {name}')

    @staticmethod
    def _point_in_polygon(px: float, py: float, polygon: list) -> bool:
        """射线法判断点是否在多边形内"""
        n = len(polygon)
        if n < 3:
            return False

        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = float(polygon[i][0]), float(polygon[i][1])
            xj, yj = float(polygon[j][0]), float(polygon[j][1])

            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside
            j = i

        return inside

    # ── Odom 回调 ─────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    # ── 持久化 ────────────────────────────────────────

    def _load_fences(self) -> dict:
        if not self._geofence_file.exists():
            return {}
        try:
            with open(self._geofence_file, 'r', encoding='utf-8') as f:
                if yaml is not None:
                    data = yaml.safe_load(f)
                else:
                    import json
                    data = json.load(f)
                return data if isinstance(data, dict) else {}
        except Exception as e:
            self.get_logger().warn(f'加载围栏文件失败: {e}')
            return {}

    def _save_fences(self):
        try:
            self._geofence_file.parent.mkdir(parents=True, exist_ok=True)
            with open(self._geofence_file, 'w', encoding='utf-8') as f:
                if yaml is not None:
                    yaml.safe_dump(self._fences, f, allow_unicode=True, default_flow_style=False)
                else:
                    import json
                    json.dump(self._fences, f, ensure_ascii=False, indent=2)
        except Exception as e:
            self.get_logger().error(f'保存围栏文件失败: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GeofenceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
