#!/bin/bash
# ── nav-semantic.service 启动脚本 ──
# 启动语义感知 + 语义规划 (NaviMind)
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

echo "[nav-semantic] Starting Semantic Perception + Planner..."
echo "[nav-semantic] LLM backend: ${LLM_BACKEND:-kimi}"

exec ros2 launch "${NAV_DIR}/launch/subsystems/semantic.launch.py"
