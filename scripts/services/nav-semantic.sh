#!/bin/bash
# ── nav-semantic.service 启动脚本 ──
# 启动语义感知 + 语义规划 (NaviMind)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

# Source API key if available
[ -f /etc/nav/semantic.env ] && source /etc/nav/semantic.env

# Ensure semantic_common is on PYTHONPATH (manual install, no colcon build)
export PYTHONPATH="${NAV_DIR}/install/semantic_common/lib/python3.10/site-packages:${PYTHONPATH}"

echo "[nav-semantic] Starting Semantic Planner..."
echo "[nav-semantic] LLM backend: kimi/kimi-k2.5"

# Note: perception_node requires PyTorch/CLIP (not yet installed on S100P)
# Run planner only with vision.enable:=false
exec ros2 run semantic_planner semantic_planner_node --ros-args \
    --params-file "${NAV_DIR}/config/semantic_planner.yaml" \
    -p vision.enable:=false \
    -r instruction:=/nav/semantic/instruction \
    -r scene_graph:=/nav/semantic/scene_graph \
    -r resolved_goal:=/nav/goal_pose \
    -r status:=/nav/semantic/status \
    -r odometry:=/nav/odometry \
    -r cmd_vel:=/nav/cmd_vel
