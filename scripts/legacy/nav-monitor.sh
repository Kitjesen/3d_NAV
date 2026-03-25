#!/bin/bash
# ── nav-monitor.service 启动脚本 ──
# 启动三环认知架构 (nav_rings) + 系统健康状态 HTTP 服务器
#
# 组件:
#   1. Ring 1 SafetyMonitor (20Hz) — 安全聚合
#   2. Ring 2 Evaluator (5Hz) — 闭环评估
#   3. Ring 3 DialogueManager (2Hz) — 统一对话状态
#   4. system_health_server — HTTP :8067 红/黄/绿灯
set -e
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"

# 使用 CycloneDDS 避免 FastDDS SHM 锁文件错误
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${NAV_DIR}/config/cyclonedds.xml"

echo "[nav-monitor] Starting three-ring architecture + health server..."

# 启动 Ring 1: SafetyMonitor (反射弧)
ros2 run nav_rings safety_monitor --ros-args \
    -p monitor_hz:=20.0 \
    -p odom_timeout_sec:=2.0 \
    -p driver_timeout_sec:=2.0 \
    -p terrain_timeout_sec:=5.0 \
    -p cmdvel_timeout_sec:=3.0 &
R1_PID=$!

# 启动 Ring 2: Evaluator (认知环)
ros2 run nav_rings evaluator --ros-args \
    -p eval_hz:=5.0 \
    -p cross_track_warn:=1.5 \
    -p cross_track_danger:=3.0 &
R2_PID=$!

# 启动 Ring 3: DialogueManager (对话环)
ros2 run nav_rings dialogue_manager --ros-args \
    -p publish_hz:=2.0 &
R3_PID=$!

# 启动健康状态 HTTP 服务器
python3 "${NAV_DIR}/tools/system_health_server.py" &
HEALTH_PID=$!

echo "[nav-monitor] R1=$R1_PID R2=$R2_PID R3=$R3_PID Health=$HEALTH_PID"

# 清理函数
cleanup() {
    echo "[nav-monitor] Shutting down..."
    kill $R1_PID $R2_PID $R3_PID $HEALTH_PID 2>/dev/null || true
    wait
}
trap cleanup EXIT INT TERM

# 等待任意一个退出
wait -n $R1_PID $R2_PID $R3_PID $HEALTH_PID
EXIT_CODE=$?

exit $EXIT_CODE
