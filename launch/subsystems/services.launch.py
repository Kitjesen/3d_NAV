"""
运营服务 launch — 地图管理 + 巡检路线 + 电子围栏 + 任务调度 + 任务日志

所有服务节点都是轻量级 Python 节点，订阅标准话题，提供命令/响应接口。
数据持久化到 ~/.lingtu/ 目录。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value=os.path.expanduser('~/.lingtu'),
        description='服务数据持久化根目录',
    )
    map_dir_arg = DeclareLaunchArgument(
        'map_dir',
        default_value=os.environ.get('NAV_MAP_DIR',
                                     os.path.expanduser('~/data/maps')),
        description='地图文件存储目录',
    )

    data_dir = LaunchConfiguration('data_dir')
    map_dir = LaunchConfiguration('map_dir')

    return LaunchDescription([
        data_dir_arg,
        map_dir_arg,

        # 地图管理
        Node(
            package='nav_services',
            executable='map_manager',
            name='map_manager',
            parameters=[{
                'data_dir': data_dir,
                'map_dir': map_dir,
            }],
            output='screen',
        ),

        # 巡检路线管理
        Node(
            package='nav_services',
            executable='patrol_manager',
            name='patrol_manager',
            parameters=[{
                'routes_dir': data_dir,  # 内部自动 append /patrol_routes
            }],
            output='screen',
        ),

        # 电子围栏
        Node(
            package='nav_services',
            executable='geofence_manager',
            name='geofence_manager',
            parameters=[{
                'geofence_file': data_dir,  # 内部自动 append /geofences.yaml
            }],
            output='screen',
        ),

        # 定时任务调度
        Node(
            package='nav_services',
            executable='task_scheduler',
            name='task_scheduler',
            parameters=[{
                'schedule_file': data_dir,  # 内部自动 append /schedules.yaml
            }],
            output='screen',
        ),

        # 任务历史记录
        Node(
            package='nav_services',
            executable='mission_logger',
            name='mission_logger',
            parameters=[{
                'log_dir': data_dir,  # 内部自动 append /mission_history
            }],
            output='screen',
        ),
    ])
