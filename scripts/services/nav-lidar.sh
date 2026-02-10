#!/bin/bash
# ── nav-lidar.service 启动脚本 ──
# 启动 Livox MID360 LiDAR 驱动
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

echo "[nav-lidar] Starting Livox MID360 driver..."
exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
