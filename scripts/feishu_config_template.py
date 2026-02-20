#!/usr/bin/env python3
"""
飞书机器人配置模板
复制此文件并重命名为 feishu_config.py，然后填入你的实际配置
"""

# ============================================
# 飞书应用凭证
# 从 https://open.feishu.cn/ 获取
# ============================================

# App ID (以 cli_ 开头)
APP_ID = "cli_xxxxxxxxxxxxxxxx"

# App Secret (32位字符串)
APP_SECRET = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

# 接收者 ID
# - 发送给个人: 使用 open_id (以 ou_ 开头)
# - 发送给群组: 使用 chat_id (以 oc_ 开头)
RECEIVE_ID = "ou_xxxxxxxxxxxxxxxxxxxxxxxx"

# 接收者类型
# - "open_id": 发送给个人用户
# - "chat_id": 发送给群组
RECEIVE_ID_TYPE = "open_id"


# ============================================
# ROS2 配置
# ============================================

# 监听的话题名称
STATUS_TOPIC = "/nav/semantic/status"

# 节点名称
NODE_NAME = "feishu_monitor_bot"


# ============================================
# 消息配置
# ============================================

# 是否使用卡片消息（更美观）
USE_CARD_MESSAGE = True

# 启动消息
STARTUP_MESSAGE = {
    "title": "🤖 thunder 机器人已启动",
    "content": "**3D-NAV 监控系统**\n\n✅ 已连接到 ROS2\n✅ 正在监听导航状态"
}

# 停止消息
SHUTDOWN_MESSAGE = "⚠️ thunder 机器人已停止"


# ============================================
# 使用示例
# ============================================

"""
1. 复制此文件:
   cp feishu_config_template.py feishu_config.py

2. 编辑 feishu_config.py，填入实际配置

3. 在 feishu_monitor_bot.py 中导入:
   from feishu_config import APP_ID, APP_SECRET, RECEIVE_ID

4. 运行机器人:
   python3 feishu_monitor_bot.py
"""
