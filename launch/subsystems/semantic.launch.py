"""
语义导航子系统 — semantic perception + semantic planner

启动节点:
  1. semantic_perception_node  — GroundingDINO 检测 + ConceptGraphs 场景图
  2. semantic_planner_node     — Cloud LLM 语义规划器

依赖:
  - SLAM 层 (/nav/odometry, TF)
  - Orbbec RGB-D 相机 (/camera/color/image_raw, /camera/depth/image_rect_raw)
  - third_party/ 已通过 clone_semantic_deps.sh 安装

用法:
  不直接启动 — 通过 navigation_run.launch.py enable_semantic:=true 启用
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 参数声明 ──
    config_dir = os.path.join(
        os.path.dirname(__file__), "..", "..", "config"
    )

    perception_config_arg = DeclareLaunchArgument(
        "perception_config",
        default_value=os.path.join(config_dir, "semantic_perception.yaml"),
        description="语义感知配置文件路径",
    )
    planner_config_arg = DeclareLaunchArgument(
        "planner_config",
        default_value=os.path.join(config_dir, "semantic_planner.yaml"),
        description="语义规划器配置文件路径",
    )

    # ── 语义感知节点 ──
    perception_node = Node(
        package="semantic_perception",
        executable="semantic_perception_node",
        name="semantic_perception_node",
        output="screen",
        parameters=[LaunchConfiguration("perception_config")],
        remappings=[
            # 将标准 Orbbec 话题映射到节点内部名称
            ("color_image", "/camera/color/image_raw"),
            ("depth_image", "/camera/depth/image_rect_raw"),
            ("camera_info", "/camera/color/camera_info"),
            ("odometry", "/nav/odometry"),
            # 输出话题
            ("detections_3d", "/nav/semantic/detections_3d"),
            ("scene_graph", "/nav/semantic/scene_graph"),
        ],
    )

    # ── 语义规划节点 ──
    planner_node = Node(
        package="semantic_planner",
        executable="semantic_planner_node",
        name="semantic_planner_node",
        output="screen",
        parameters=[LaunchConfiguration("planner_config")],
        remappings=[
            ("instruction", "/nav/semantic/instruction"),
            ("scene_graph", "/nav/semantic/scene_graph"),
            ("resolved_goal", "/nav/semantic/resolved_goal"),
            ("status", "/nav/semantic/status"),
            ("odometry", "/nav/odometry"),
        ],
    )

    return LaunchDescription(
        [
            perception_config_arg,
            planner_config_arg,
            perception_node,
            planner_node,
        ]
    )
