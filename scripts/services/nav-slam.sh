#!/bin/bash
# ── nav-slam.service 启动脚本 ──
# 启动 Fast-LIO2 SLAM (带正确的 topic remap)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

CONFIG_PATH="$(ros2 pkg prefix fastlio2)/share/fastlio2/config/lio.yaml"

echo "[nav-slam] Starting Fast-LIO2 with config: $CONFIG_PATH"
exec ros2 run fastlio2 lio_node --ros-args \
    -r __ns:=/fastlio2 \
    -r __node:=lio_node \
    -p config_path:="$CONFIG_PATH" \
    -r body_cloud:=/cloud_registered \
    -r world_cloud:=/cloud_map \
    -r lio_odom:=/Odometry \
    -r lio_path:=/lio_path
