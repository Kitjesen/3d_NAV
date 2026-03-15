#!/bin/bash
# ═══════════════════════════════════════════════════════════
# LingTu 语义导航 Demo 启动脚本
#
# 启动组件:
#   1. Stub TF (map→odom→body)
#   2. Stub odometry (10Hz)
#   3. Demo scene graph publisher (1Hz, 18 objects)
#   4. Semantic planner node (Claude Opus 4.6)
#   5. Orbbec camera (可选, 检测到设备时启动)
#
# Usage:
#   bash scripts/demo_semantic.sh
#   bash scripts/demo_semantic.sh --no-camera    # 跳过摄像头
#   bash scripts/demo_semantic.sh --stop         # 停止所有
#
# 测试:
#   ros2 topic pub /nav/semantic/instruction std_msgs/msg/String \
#       "{data: '导航到传送带'}" --once
# ═══════════════════════════════════════════════════════════
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAV_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
LOG_DIR="/tmp/lingtu_demo"
mkdir -p "$LOG_DIR"

# ── 参数 ──
NO_CAMERA=false
STOP_ALL=false
for arg in "$@"; do
    case "$arg" in
        --no-camera) NO_CAMERA=true ;;
        --stop) STOP_ALL=true ;;
    esac
done

# ── 停止所有 ──
stop_all() {
    echo "[demo] Stopping all demo processes..."
    pkill -f semantic_planner_node 2>/dev/null || true
    pkill -f demo_scene_graph_publisher 2>/dev/null || true
    pkill -f "static_transform_publisher.*stub" 2>/dev/null || true
    pkill -f "topic pub.*/nav/odometry" 2>/dev/null || true
    pkill -f "orbbec_camera" 2>/dev/null || true
    echo "[demo] All stopped."
}

if $STOP_ALL; then
    stop_all
    exit 0
fi

# ── 环境 ──
source /opt/ros/humble/setup.bash
source "${NAV_DIR}/install/setup.bash" 2>/dev/null || true

# Source API key
if [ -z "$ANTHROPIC_API_KEY" ]; then
    source ~/.bashrc 2>/dev/null || true
fi
if [ -z "$ANTHROPIC_API_KEY" ]; then
    echo "[demo] WARNING: ANTHROPIC_API_KEY not set. Slow Path (LLM) will not work."
fi

echo "═══════════════════════════════════════════"
echo "  LingTu 语义导航 Demo"
echo "═══════════════════════════════════════════"
echo "  NAV_DIR: ${NAV_DIR}"
echo "  LLM:     claude/claude-opus-4-6"
echo "  Camera:  $(if $NO_CAMERA; then echo 'DISABLED'; else echo 'auto-detect'; fi)"
echo ""

# ── 1. Stub SLAM (TF + Odometry) ──
echo "[1/4] Starting stub SLAM (TF + odometry)..."

# Stop real SLAM services if running
sudo systemctl stop nav-slam 2>/dev/null || true
sudo systemctl stop nav-lidar 2>/dev/null || true

# Kill existing stub processes
pkill -f "static_transform_publisher.*stub" 2>/dev/null || true
pkill -f "topic pub.*/nav/odometry" 2>/dev/null || true

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id map --child-frame-id odom \
    --node-name stub_tf_map_odom \
    > "$LOG_DIR/stub_tf_map.log" 2>&1 &

ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
    --frame-id odom --child-frame-id body \
    --node-name stub_tf_odom_body \
    > "$LOG_DIR/stub_tf_odom.log" 2>&1 &

ros2 topic pub /nav/odometry nav_msgs/msg/Odometry \
    "{header: {frame_id: 'odom'}, child_frame_id: 'body', \
      pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
    --rate 10 > "$LOG_DIR/stub_odom.log" 2>&1 &

echo "  + TF map→odom→body"
echo "  + Odometry 10Hz"

# ── 2. Scene Graph Publisher ──
echo "[2/4] Starting demo scene graph publisher..."
pkill -f demo_scene_graph_publisher 2>/dev/null || true
python3 /tmp/demo_scene_graph_publisher.py --scenario factory --rate 1.0 \
    > "$LOG_DIR/scene_graph.log" 2>&1 &
echo "  + 18 objects, 8 regions, 1Hz"

# ── 3. Camera (optional) ──
if ! $NO_CAMERA; then
    if lsusb 2>/dev/null | grep -qi orbbec; then
        echo "[3/4] Starting Orbbec camera..."
        ros2 launch orbbec_camera gemini_330_series.launch.py \
            enable_color:=true enable_depth:=true \
            depth_width:=640 depth_height:=480 \
            color_width:=640 color_height:=480 \
            > "$LOG_DIR/camera.log" 2>&1 &
        echo "  + Orbbec Gemini 335 (640x480)"
    else
        echo "[3/4] No Orbbec camera detected, skipping"
    fi
else
    echo "[3/4] Camera disabled (--no-camera)"
fi

# ── 4. Semantic Planner ──
echo "[4/4] Starting semantic planner..."
pkill -f semantic_planner_node 2>/dev/null || true
sleep 2

ros2 run semantic_planner semantic_planner_node --ros-args \
    --params-file "${NAV_DIR}/config/semantic_planner.yaml" \
    -p vision.enable:=false \
    -r instruction:=/nav/semantic/instruction \
    -r scene_graph:=/nav/semantic/scene_graph \
    -r resolved_goal:=/nav/goal_pose \
    -r status:=/nav/semantic/status \
    -r odometry:=/nav/odometry \
    -r cmd_vel:=/nav/cmd_vel \
    > "$LOG_DIR/planner.log" 2>&1 &

sleep 5
echo "  + Semantic planner (Claude Opus 4.6)"

echo ""
echo "═══════════════════════════════════════════"
echo "  Demo ready!"
echo "═══════════════════════════════════════════"
echo ""
echo "  Logs: ${LOG_DIR}/"
echo ""
echo "  测试指令:"
echo "    ros2 topic pub /nav/semantic/instruction std_msgs/msg/String \\"
echo "        \"{data: '导航到传送带'}\" --once"
echo ""
echo "  可用目标: 大门 门 办公室 传送带 控制台 控制室"
echo "            楼梯 楼梯间 货架 仓储区 设备间 灭火器"
echo ""
echo "  停止: bash scripts/demo_semantic.sh --stop"
echo ""
