"""
Agent 子系统 — LLM 决策大脑

启动节点:
  agent_node — LangChain Agent + ROS2 集成

依赖:
  - semantic_planner 包中的 agent_node
  - LLM API key (MOONSHOT_API_KEY 或 OPENAI_API_KEY)

用法:
  不直接启动 — 通过 navigation_run.launch.py enable_agent:=true 启用
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    llm_backend_arg = DeclareLaunchArgument(
        "agent_llm_backend",
        default_value="mock",
        description="Agent LLM 后端 (mock/kimi/openai)",
    )
    llm_model_arg = DeclareLaunchArgument(
        "agent_llm_model",
        default_value="",
        description="LLM 模型名 (空=使用默认)",
    )
    language_arg = DeclareLaunchArgument(
        "agent_language",
        default_value="zh",
        description="Agent 语言 (zh/en)",
    )

    agent_node = Node(
        package="semantic_planner",
        executable="agent_node",
        name="agent_node",
        output="screen",
        parameters=[{
            "llm.backend": LaunchConfiguration("agent_llm_backend"),
            "llm.model": LaunchConfiguration("agent_llm_model"),
            "language": LaunchConfiguration("agent_language"),
        }],
        remappings=[
            ("odometry", "/nav/odometry"),
        ],
    )

    return LaunchDescription([
        llm_backend_arg,
        llm_model_arg,
        language_arg,
        agent_node,
    ])
