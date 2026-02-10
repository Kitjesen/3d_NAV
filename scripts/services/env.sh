#!/bin/bash
# ── 导航系统公共环境变量 ──
# 所有 nav-*.service 的 wrapper 脚本 source 此文件

export NAV_DIR="/home/sunrise/data/SLAM/navigation"
export ROS_DISTRO="humble"
# RMW: 使用系统默认 (rmw_fastrtps_cpp)
# export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# Source ROS2 + workspace
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${NAV_DIR}/install/setup.bash"

# 日志目录
export NAV_LOG_DIR="${NAV_DIR}/logs"
mkdir -p "$NAV_LOG_DIR"
