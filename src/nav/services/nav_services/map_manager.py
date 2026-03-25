#!/usr/bin/env python3
"""
MapManager — 地图与兴趣点(POI)管理节点

管理地图的列表/保存/删除/重命名/激活，以及 POI 的增删查导航。

话题:
  订阅:
    /nav/map/command   (String JSON) — 地图操作指令
    /nav/poi/command   (String JSON) — POI 操作指令
    /nav/odometry      (Odometry)    — 缓存当前位姿
  发布:
    /nav/map/response  (String JSON) — 地图操作结果
    /nav/poi/response  (String JSON) — POI 操作结果
    /nav/goal_pose     (PoseStamped) — POI navigate 时发布目标
"""

import os
import shutil
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from semantic.common.semantic_common.sanitize import safe_json_dumps, safe_json_loads

try:
    import yaml
except ImportError:
    yaml = None

# SaveMaps 服务 — 可能未编译或不在当前环境
try:
    from interface.srv import SaveMaps
    _HAS_SAVE_MAPS = True
except ImportError:
    _HAS_SAVE_MAPS = False


class MapManager(Node):
    def __init__(self):
        super().__init__('map_manager')

        # 参数
        default_data_dir = os.path.expanduser('~/.lingtu')
        default_map_dir = os.environ.get('NAV_MAP_DIR', os.path.expanduser('~/data/maps'))

        self.declare_parameter('data_dir', default_data_dir)
        self.declare_parameter('map_dir', default_map_dir)

        self._data_dir = Path(self.get_parameter('data_dir').value)
        self._map_dir = Path(self.get_parameter('map_dir').value)

        self._data_dir.mkdir(parents=True, exist_ok=True)
        self._map_dir.mkdir(parents=True, exist_ok=True)

        # POI 与活跃地图持久化路径
        self._poi_file = self._data_dir / 'pois.yaml'
        self._active_map_file = self._data_dir / 'active_map.yaml'

        # 载入 POI 与活跃地图
        self._pois = self._load_yaml(self._poi_file, default={})
        self._active_map = self._load_yaml(self._active_map_file, default={}).get('active', '')

        # 缓存当前位姿
        self._current_pose = None

        # save_map 服务客户端
        self._save_map_client = None
        if _HAS_SAVE_MAPS:
            self._save_map_client = self.create_client(SaveMaps, '/nav/save_map')
            self.get_logger().info('SaveMaps 服务客户端已创建')
        else:
            self.get_logger().warn('interface.srv.SaveMaps 不可用，save 功能将受限')

        # 订阅
        self.create_subscription(String, '/nav/map/command', self._on_map_cmd, 10)
        self.create_subscription(String, '/nav/poi/command', self._on_poi_cmd, 10)
        self.create_subscription(Odometry, '/nav/odometry', self._on_odom, 10)

        # 发布
        self._map_resp_pub = self.create_publisher(String, '/nav/map/response', 10)
        self._poi_resp_pub = self.create_publisher(String, '/nav/poi/response', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/nav/goal_pose', 10)

        self.get_logger().info(
            f'MapManager 启动 | data_dir={self._data_dir} | map_dir={self._map_dir}'
        )

    # ── 地图操作 ──────────────────────────────────────

    def _on_map_cmd(self, msg: String):
        cmd = safe_json_loads(msg.data, default={})
        action = cmd.get('action', '')
        resp = {'action': action, 'success': False}

        try:
            if action == 'list':
                resp = self._map_list()
            elif action == 'save':
                resp = self._map_save(cmd.get('name', ''))
            elif action == 'delete':
                resp = self._map_delete(cmd.get('name', ''))
            elif action == 'rename':
                resp = self._map_rename(cmd.get('name', ''), cmd.get('new_name', ''))
            elif action == 'set_active':
                resp = self._map_set_active(cmd.get('name', ''))
            else:
                resp['message'] = f'未知操作: {action}'
        except Exception as e:
            resp['message'] = str(e)
            self.get_logger().error(f'地图操作异常: {e}')

        out = String()
        out.data = safe_json_dumps(resp)
        self._map_resp_pub.publish(out)

    def _map_list(self) -> dict:
        maps = sorted([f.stem for f in self._map_dir.glob('*.pcd')])
        return {
            'action': 'list',
            'success': True,
            'maps': maps,
            'active': self._active_map,
        }

    def _map_save(self, name: str) -> dict:
        if not name:
            return {'action': 'save', 'success': False, 'message': '缺少地图名称'}

        save_path = str(self._map_dir / f'{name}.pcd')

        # 尝试调用 SLAM 的 save_map 服务
        if self._save_map_client is not None:
            if not self._save_map_client.wait_for_service(timeout_sec=3.0):
                return {'action': 'save', 'success': False, 'message': '/nav/save_map 服务不可用'}

            req = SaveMaps.Request()
            req.file_path = save_path
            req.save_patches = False

            future = self._save_map_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

            if future.result() is not None and future.result().success:
                self.get_logger().info(f'地图已保存: {save_path}')
                return {'action': 'save', 'success': True, 'message': f'地图已保存: {name}'}
            else:
                err = future.result().message if future.result() else '服务调用超时'
                return {'action': 'save', 'success': False, 'message': err}
        else:
            return {'action': 'save', 'success': False, 'message': 'SaveMaps 服务不可用'}

    def _map_delete(self, name: str) -> dict:
        if not name:
            return {'action': 'delete', 'success': False, 'message': '缺少地图名称'}

        path = self._map_dir / f'{name}.pcd'
        if not path.exists():
            return {'action': 'delete', 'success': False, 'message': f'地图不存在: {name}'}

        try:
            path.unlink()
            if self._active_map == name:
                self._active_map = ''
                self._save_active_map()
            self.get_logger().info(f'地图已删除: {name}')
            return {'action': 'delete', 'success': True, 'message': f'地图已删除: {name}'}
        except OSError as e:
            return {'action': 'delete', 'success': False, 'message': str(e)}

    def _map_rename(self, name: str, new_name: str) -> dict:
        if not name or not new_name:
            return {'action': 'rename', 'success': False, 'message': '缺少名称'}

        src = self._map_dir / f'{name}.pcd'
        dst = self._map_dir / f'{new_name}.pcd'

        if not src.exists():
            return {'action': 'rename', 'success': False, 'message': f'地图不存在: {name}'}
        if dst.exists():
            return {'action': 'rename', 'success': False, 'message': f'目标名已存在: {new_name}'}

        try:
            shutil.move(str(src), str(dst))
            if self._active_map == name:
                self._active_map = new_name
                self._save_active_map()
            self.get_logger().info(f'地图已重命名: {name} -> {new_name}')
            return {'action': 'rename', 'success': True, 'message': f'{name} -> {new_name}'}
        except OSError as e:
            return {'action': 'rename', 'success': False, 'message': str(e)}

    def _map_set_active(self, name: str) -> dict:
        if not name:
            return {'action': 'set_active', 'success': False, 'message': '缺少地图名称'}

        path = self._map_dir / f'{name}.pcd'
        if not path.exists():
            return {'action': 'set_active', 'success': False, 'message': f'地图不存在: {name}'}

        self._active_map = name
        self._save_active_map()
        self.get_logger().info(f'激活地图: {name}')
        return {'action': 'set_active', 'success': True, 'active': name, 'message': f'已激活: {name}'}

    def _save_active_map(self):
        self._save_yaml(self._active_map_file, {'active': self._active_map})

    # ── POI 操作 ──────────────────────────────────────

    def _on_poi_cmd(self, msg: String):
        cmd = safe_json_loads(msg.data, default={})
        action = cmd.get('action', '')
        resp = {'action': action, 'success': False}

        try:
            if action == 'set':
                resp = self._poi_set(cmd)
            elif action == 'delete':
                resp = self._poi_delete(cmd.get('name', ''))
            elif action == 'list':
                resp = self._poi_list()
            elif action == 'navigate':
                resp = self._poi_navigate(cmd.get('name', ''))
            else:
                resp['message'] = f'未知 POI 操作: {action}'
        except Exception as e:
            resp['message'] = str(e)
            self.get_logger().error(f'POI 操作异常: {e}')

        out = String()
        out.data = safe_json_dumps(resp)
        self._poi_resp_pub.publish(out)

    def _poi_set(self, cmd: dict) -> dict:
        name = cmd.get('name', '')
        if not name:
            return {'action': 'set', 'success': False, 'message': '缺少 POI 名称'}

        self._pois[name] = {
            'x': float(cmd.get('x', 0.0)),
            'y': float(cmd.get('y', 0.0)),
            'z': float(cmd.get('z', 0.0)),
        }
        self._save_yaml(self._poi_file, self._pois)
        self.get_logger().info(f'POI 已设置: {name} ({self._pois[name]})')
        return {'action': 'set', 'success': True, 'message': f'POI 已设置: {name}'}

    def _poi_delete(self, name: str) -> dict:
        if name not in self._pois:
            return {'action': 'delete', 'success': False, 'message': f'POI 不存在: {name}'}

        del self._pois[name]
        self._save_yaml(self._poi_file, self._pois)
        self.get_logger().info(f'POI 已删除: {name}')
        return {'action': 'delete', 'success': True, 'message': f'POI 已删除: {name}'}

    def _poi_list(self) -> dict:
        return {'action': 'list', 'success': True, 'pois': self._pois}

    def _poi_navigate(self, name: str) -> dict:
        if name not in self._pois:
            return {'action': 'navigate', 'success': False, 'message': f'POI 不存在: {name}'}

        poi = self._pois[name]
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = poi['x']
        goal.pose.position.y = poi['y']
        goal.pose.position.z = poi['z']
        goal.pose.orientation.w = 1.0

        self._goal_pub.publish(goal)
        self.get_logger().info(f'导航到 POI: {name} ({poi})')
        return {'action': 'navigate', 'success': True, 'message': f'已发送导航目标: {name}'}

    # ── Odom 回调 ─────────────────────────────────────

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        self._current_pose = {'x': p.x, 'y': p.y, 'z': p.z}

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
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
