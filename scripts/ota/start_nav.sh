#!/bin/bash
# ============================================================================
# start_nav.sh — 导航系统 systemd 启动入口
#
# 此脚本由 navigation.service 调用，负责:
#   1. source ROS 2 环境
#   2. source colcon install 环境
#   3. 设置 FastDDS 配置
#   4. 启动导航 launch 文件
#
# 安装到机器人:
#   sudo cp scripts/ota/start_nav.sh /opt/robot/navigation/start_nav.sh
#   sudo chmod +x /opt/robot/navigation/start_nav.sh
# ============================================================================
set -e

NAV_DIR="/opt/robot/navigation/current"

# Source ROS 2
source /opt/ros/humble/setup.bash

# Source 导航工作空间
if [ -f "$NAV_DIR/install/setup.bash" ]; then
    source "$NAV_DIR/install/setup.bash"
else
    echo "ERROR: $NAV_DIR/install/setup.bash 不存在"
    exit 1
fi

# FastDDS 配置
if [ -f "$NAV_DIR/fastdds_no_shm.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$NAV_DIR/fastdds_no_shm.xml"
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 打印版本信息
if [ -f "$NAV_DIR/metadata.json" ]; then
    echo "Navigation version: $(python3 -c "import json; print(json.load(open('$NAV_DIR/metadata.json'))['version'])" 2>/dev/null || echo 'unknown')"
fi

# 启动导航系统 (运行模式)
exec ros2 launch "$NAV_DIR/launch/navigation_run.launch.py"
