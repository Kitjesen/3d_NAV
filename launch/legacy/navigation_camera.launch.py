"""
camera-only navigation launch (无 LiDAR).

启动组件:
  - Stub TF: map→odom→body (静态单位变换)
  - IMU 里程计中继: /nav/dog_odometry → /nav/odometry
  - terrain_analysis + local_planner (terrain 从空点云, 无障碍检测)
  - goal_to_waypoint: /nav/semantic/resolved_goal → /nav/way_point

外部服务 (systemd, 需已启动):
  - nav-perception: BPU 相机检测 + scene_graph
  - nav-semantic:   语义目标解析器
  - nav-driver:     han_dog_bridge (提供 /nav/dog_odometry)

用法:
  ros2 launch launch/navigation_camera.launch.py

注意:
  - 相机须朝外才能获得有效场景图
  - 无 LiDAR 时不做障碍物回避
  - 机器人从 (0,0) 出发, 目标位置基于 TF fallback 近似
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

_NAV_DIR = os.environ.get('NAV_DIR', '/opt/nav')
_TOOLS = os.path.join(_NAV_DIR, 'tools')

# ROS2 环境 (子进程继承)
_ROS_ENV = {
    **os.environ,
    'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp',
    'CYCLONEDDS_URI': f'file://{_NAV_DIR}/config/cyclonedds.xml',
    'PYTHONPATH': ':'.join([
        f'{_NAV_DIR}/install/semantic_common/lib/python3.10/site-packages',
        os.environ.get('PYTHONPATH', ''),
    ]),
}


def generate_launch_description():
    return LaunchDescription([
        # ── Stub TF chain: map→odom (identity) ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='stub_map_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            output='screen',
        ),
        # ── Stub TF: odom→body (identity, 无 SLAM 时机器人在原点) ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='stub_body_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'body'],
            output='screen',
        ),
        # ── IMU 里程计中继 ──
        ExecuteProcess(
            cmd=['python3', os.path.join(_TOOLS, 'odom_relay.py')],
            output='screen',
            env=_ROS_ENV,
        ),
        # ── 地形分析 + 局部规划器 ──
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(_NAV_DIR, 'launch/subsystems/autonomy.launch.py')
            ),
        ),
        # ── 语义目标 → 航点桥接 ──
        ExecuteProcess(
            cmd=['python3', os.path.join(_TOOLS, 'goal_to_waypoint.py')],
            output='screen',
            env=_ROS_ENV,
        ),
    ])
