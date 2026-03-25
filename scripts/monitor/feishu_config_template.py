#!/usr/bin/env python3
"""
Feishu (Lark) bot configuration template.
Copy this file to feishu_config.py and fill in your actual credentials.
"""

# ============================================
# Feishu app credentials
# Obtain from https://open.feishu.cn/
# ============================================

# App ID (starts with cli_)
APP_ID = "cli_xxxxxxxxxxxxxxxx"

# App Secret (32-character string)
APP_SECRET = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"

# Receiver ID
# - Send to a user:  use open_id  (starts with ou_)
# - Send to a group: use chat_id  (starts with oc_)
RECEIVE_ID = "ou_xxxxxxxxxxxxxxxxxxxxxxxx"

# Receiver type
# - "open_id": send to an individual user
# - "chat_id": send to a group
RECEIVE_ID_TYPE = "open_id"


# ============================================
# ROS2 configuration
# ============================================

# Topic to subscribe to
STATUS_TOPIC = "/nav/semantic/status"

# ROS2 node name
NODE_NAME = "feishu_monitor_bot"


# ============================================
# Message configuration
# ============================================

# Use interactive card messages (better formatting)
USE_CARD_MESSAGE = True

# Startup message
STARTUP_MESSAGE = {
    "title": "thunder robot started",
    "content": "**3D-NAV Monitor**\n\nConnected to ROS2\nListening for navigation status"
}

# Shutdown message
SHUTDOWN_MESSAGE = "thunder robot stopped"


# ============================================
# Usage
# ============================================

"""
1. Copy this file:
   cp feishu_config_template.py feishu_config.py

2. Edit feishu_config.py with your actual credentials.

3. Import in feishu_monitor_bot.py:
   from feishu_config import APP_ID, APP_SECRET, RECEIVE_ID

4. Run the bot:
   python3 feishu_monitor_bot.py
"""
