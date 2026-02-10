#!/bin/bash
# ── nav-autonomy.service 启动脚本 ──
# 启动地形分析 + 局部规划 + 路径跟踪
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

LAUNCH_FILE="${NAV_DIR}/scripts/launch/nav_autonomy_launch.py"

echo "[nav-autonomy] Starting terrain analysis + local planner + path follower..."
exec ros2 launch "$LAUNCH_FILE"
