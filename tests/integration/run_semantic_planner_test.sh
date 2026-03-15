#!/bin/bash
# run_semantic_planner_test.sh — 轻量级语义规划集成测试
#
# 启动 semantic_planner_node (mock LLM) + 静态 TF + 测试节点
# 不需要 MuJoCo/LiDAR/相机
#
# 用法: bash run_semantic_planner_test.sh
set -e

echo "=== Semantic Planner Integration Test ==="
echo "  Mode: mock LLM (no API key required)"
echo "  Date: $(date)"
echo ""

# ── 清理旧进程 ──
echo "[0] Cleaning up ..."
pkill -9 -f semantic_planner_node 2>/dev/null || true
pkill -9 -f semantic_planner_test 2>/dev/null || true
pkill -9 -f "static_transform_publisher.*map.*odom" 2>/dev/null || true
sleep 1

# ── Source ROS2 + workspace ──
source /opt/ros/humble/setup.bash
if [ -f /opt/nova/lingtu/v1.8.0/install/setup.bash ]; then
    source /opt/nova/lingtu/v1.8.0/install/setup.bash
    echo "[OK] Sourced /opt/nova/lingtu/v1.8.0/install/setup.bash"
elif [ -f /home/sunrise/data/SLAM/navigation/install/setup.bash ]; then
    source /home/sunrise/data/SLAM/navigation/install/setup.bash
    echo "[OK] Sourced /home/sunrise/data/SLAM/navigation/install/setup.bash"
else
    echo "[ERROR] No workspace found!"
    exit 1
fi

# ── [1] 静态 TF: map → odom → body ──
echo "[1] Starting static TF publishers ..."
ros2 run tf2_ros static_transform_publisher \
    --x 0.0 --y 0.0 --z 0.0 --yaw 0.0 --pitch 0.0 --roll 0.0 \
    --frame-id map --child-frame-id odom \
    > /tmp/test_tf_map_odom.log 2>&1 &
TF1_PID=$!

ros2 run tf2_ros static_transform_publisher \
    --x 2.0 --y 2.0 --z 0.35 --yaw 0.0 --pitch 0.0 --roll 0.0 \
    --frame-id odom --child-frame-id body \
    > /tmp/test_tf_odom_body.log 2>&1 &
TF2_PID=$!
sleep 1

# ── [2] 启动 semantic_planner_node (mock LLM) ──
echo "[2] Starting semantic_planner_node (mock LLM) ..."
CONFIG_FILE=/opt/nova/lingtu/v1.8.0/config/semantic_planner.yaml
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE=/home/sunrise/data/SLAM/navigation/config/semantic_planner.yaml
fi

ros2 run semantic_planner semantic_planner_node \
    --ros-args \
    -r instruction:=/nav/semantic/instruction \
    -r scene_graph:=/nav/semantic/scene_graph \
    -r odometry:=/nav/odometry \
    -r resolved_goal:=/nav/goal_pose \
    -r status:=/nav/semantic/status \
    -r cmd_vel:=/nav/semantic/cmd_vel \
    --params-file "$CONFIG_FILE" \
    -p llm.backend:=mock \
    -p llm_fallback.backend:=mock \
    -p exploration.enable:=false \
    -p goal_resolution.replan_on_failure:=false \
    > /tmp/test_semantic_planner.log 2>&1 &
SP_PID=$!
sleep 5

if ! kill -0 $SP_PID 2>/dev/null; then
    echo "[ERROR] semantic_planner_node died!"
    echo "--- Log (last 30 lines) ---"
    tail -30 /tmp/test_semantic_planner.log
    kill $TF1_PID $TF2_PID 2>/dev/null
    exit 1
fi
echo "  semantic_planner_node running (PID=$SP_PID)"

# ── [3] 运行测试 ──
echo ""
echo "[3] Running integration tests ..."
echo "─────────────────────────────────────────"
TEST_RC=0
python3 /tmp/test_semantic_planner_live.py --timeout 60 || TEST_RC=$?

# ── [4] 清理 ──
echo ""
echo "[4] Cleanup ..."
kill $SP_PID $TF1_PID $TF2_PID 2>/dev/null || true
sleep 1

echo ""
echo "--- semantic_planner log (last 20) ---"
tail -20 /tmp/test_semantic_planner.log
echo ""

if [ $TEST_RC -eq 0 ]; then
    echo "*** ALL TESTS PASSED ***"
else
    echo "*** SOME TESTS FAILED (rc=$TEST_RC) ***"
fi
exit $TEST_RC
