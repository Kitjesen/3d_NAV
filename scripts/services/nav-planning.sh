#!/bin/bash
# ── nav-planning.service 启动脚本 ──
# 启动 Localizer + PCT 全局规划 + 路径适配器
#
# 参数 (通过 systemd Environment 或命令行):
#   NAV_MAP_PATH  - 地图文件路径 (不含扩展名)
#   NAV_INIT_X/Y/Z/YAW - 初始位姿
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

# 默认值 (可通过 systemd EnvironmentFile 覆盖)
NAV_MAP_PATH="${NAV_MAP_PATH:-${NAV_DIR}/src/global_planning/PCT_planner/rsc/tomogram/spiral0.3_2}"
NAV_INIT_X="${NAV_INIT_X:-0.0}"
NAV_INIT_Y="${NAV_INIT_Y:-0.0}"
NAV_INIT_Z="${NAV_INIT_Z:-0.0}"
NAV_INIT_YAW="${NAV_INIT_YAW:-0.0}"

LAUNCH_FILE="${NAV_DIR}/scripts/launch/nav_planning_launch.py"

echo "[nav-planning] map_path=$NAV_MAP_PATH"
exec ros2 launch "$LAUNCH_FILE" \
    map_path:="$NAV_MAP_PATH" \
    x:="$NAV_INIT_X" y:="$NAV_INIT_Y" z:="$NAV_INIT_Z" yaw:="$NAV_INIT_YAW"
