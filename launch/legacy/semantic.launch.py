"""
语义导航子系统 — semantic perception + semantic planner

启动节点:
  1. semantic_perception_node  — YOLO-World 检测 + ConceptGraphs 场景图
  2. semantic_planner_node     — Cloud LLM 语义规划器

依赖:
  - SLAM 层 (/nav/odometry, TF)
  - Orbbec RGB-D 相机 (/camera/color/image_raw, /camera/depth/image_rect_raw)
  - third_party/ 已通过 clone_semantic_deps.sh 安装

用法:
  不直接启动 — 通过 navigation_run.launch.py enable_semantic:=true 启用
  探索模式通过 navigation_explore.launch.py 启动, 传递 initial_instruction 和 llm_backend
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    initial_instruction_arg = DeclareLaunchArgument(
        "initial_instruction",
        default_value="",
        description="启动后自动发送的语义指令 (空=不自动发送)",
    )
    llm_backend_arg = DeclareLaunchArgument(
        "llm_backend",
        default_value="",
        description="覆盖 LLM 后端 (空=使用yaml配置, mock/kimi/openai/claude)",
    )
    enable_reconstruction_arg = DeclareLaunchArgument(
        "enable_reconstruction",
        default_value="false",
        description="启用三维重建节点 (true=启用彩色语义点云聚合)",
    )
    semantic_map_path_arg = DeclareLaunchArgument(
        "semantic_map_path",
        default_value="",
        description="语义地图持久化路径 (场景图 JSON, 如 /data/maps/office_semantic.json)",
    )
    tagged_locations_path_arg = DeclareLaunchArgument(
        "tagged_locations_path",
        default_value="",
        description="标签地点记忆路径 (JSON, 如 /data/maps/office_tags.json)",
    )

    # ── 语义感知节点 ──
    perception_node = Node(
        package="semantic_perception",
        executable="semantic_perception_node",
        name="semantic_perception_node",
        output="screen",
        parameters=[
            LaunchConfiguration("perception_config"),
            {"semantic_map_path": LaunchConfiguration("semantic_map_path")},
        ],
        remappings=[
            # 将标准 Orbbec 话题映射到节点内部名称
            ("color_image", "/camera/color/image_raw"),
            ("depth_image", "/camera/depth/image_raw"),
            ("camera_info", "/camera/color/camera_info"),
            ("odometry", "/nav/odometry"),
            # 输出话题
            ("detections_3d", "/nav/semantic/detections_3d"),
            ("scene_graph", "/nav/semantic/scene_graph"),
        ],
    )

    # ── 语义规划节点 ──
    # initial_instruction 和 llm.backend 作为参数覆盖 yaml 中的默认值
    # 注意: 空字符串会覆盖 YAML — planner_node 内部对空值做了 fallback 处理
    planner_node = Node(
        package="semantic_planner",
        executable="semantic_planner_node",
        name="semantic_planner_node",
        output="screen",
        parameters=[
            LaunchConfiguration("planner_config"),
            {
                "initial_instruction": LaunchConfiguration("initial_instruction"),
                "llm.backend": LaunchConfiguration("llm_backend"),
                "tagged_locations_path": LaunchConfiguration("tagged_locations_path"),
            },
        ],
        remappings=[
            ("instruction", "/nav/semantic/instruction"),
            ("scene_graph", "/nav/semantic/scene_graph"),
            ("resolved_goal", "/nav/goal_pose"),          # 直接路由到局部规划标准输入
            ("status", "/nav/semantic/status"),
            ("odometry", "/nav/odometry"),
            ("cmd_vel", "/nav/cmd_vel"),                  # TwistStamped → 标准速度指令
            ("costmap_out", "/nav/costmap"),              # 发布给 perception_node SCG 使用
        ],
    )

    # ── 三维重建节点（可选）──
    reconstruction_node = Node(
        package="reconstruction",
        executable="reconstruction_node",
        name="reconstruction_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_reconstruction")),
        remappings=[
            ("color_image", "/camera/color/image_raw"),
            ("depth_image", "/camera/depth/image_rect_raw"),
            ("camera_info", "/camera/color/camera_info"),
            ("scene_graph", "/nav/semantic/scene_graph"),
            ("semantic_cloud", "/nav/reconstruction/semantic_cloud"),
            ("stats", "/nav/reconstruction/stats"),
            ("save_ply", "/nav/reconstruction/save_ply"),
        ],
    )

    return LaunchDescription(
        [
            perception_config_arg,
            planner_config_arg,
            initial_instruction_arg,
            llm_backend_arg,
            enable_reconstruction_arg,
            semantic_map_path_arg,
            tagged_locations_path_arg,
            perception_node,
            planner_node,
            reconstruction_node,
        ]
    )
