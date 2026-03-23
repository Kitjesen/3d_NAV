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
    tagged_locations_arg = DeclareLaunchArgument(
        "tagged_locations_path",
        default_value="",
        description="标签地点 JSON 文件路径 (空=仅内存)",
    )
    conversation_db_arg = DeclareLaunchArgument(
        "conversation_db_path",
        default_value="",
        description="对话历史 SQLite 路径 (空=不持久化)",
    )
    mcp_enable_arg = DeclareLaunchArgument(
        "mcp_enable",
        default_value="false",
        description="是否启动 MCP Server",
    )
    mcp_port_arg = DeclareLaunchArgument(
        "mcp_port",
        default_value="8090",
        description="MCP Server 端口",
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
            "tagged_locations_path": LaunchConfiguration("tagged_locations_path"),
            "conversation_db_path": LaunchConfiguration("conversation_db_path"),
            "mcp.enable": LaunchConfiguration("mcp_enable"),
            "mcp.port": LaunchConfiguration("mcp_port"),
        }],
        remappings=[
            ("odometry", "/nav/odometry"),
        ],
    )

    return LaunchDescription([
        llm_backend_arg,
        llm_model_arg,
        language_arg,
        tagged_locations_arg,
        conversation_db_arg,
        mcp_enable_arg,
        mcp_port_arg,
        agent_node,
    ])
