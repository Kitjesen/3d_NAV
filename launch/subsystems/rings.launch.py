"""
三环认知架构 launch — SafetyMonitor + Evaluator + DialogueManager

Ring 1 (反射弧): safety_monitor — 20Hz, 聚合安全信号
Ring 2 (认知环): evaluator      — 5Hz, 闭环评估
Ring 3 (对话环): dialogue_manager — 2Hz, 统一状态
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Ring 1: 反射弧 — 最先启动, 最高优先级
        Node(
            package='nav_rings',
            executable='safety_monitor',
            name='safety_monitor',
            parameters=[{
                'monitor_hz': 20.0,
                'odom_timeout_sec': 2.0,
                'driver_timeout_sec': 2.0,
                'terrain_timeout_sec': 5.0,
                'cmdvel_timeout_sec': 3.0,
            }],
            output='screen',
        ),

        # Ring 2: 认知环
        Node(
            package='nav_rings',
            executable='evaluator',
            name='evaluator',
            parameters=[{
                'eval_hz': 5.0,
                'cross_track_warn': 1.5,
                'cross_track_danger': 3.0,
            }],
            output='screen',
        ),

        # Ring 3: 对话环
        Node(
            package='nav_rings',
            executable='dialogue_manager',
            name='dialogue_manager',
            parameters=[{
                'publish_hz': 2.0,
            }],
            output='screen',
        ),
    ])
