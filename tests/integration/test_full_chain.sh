#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# T6: Full-chain closed-loop integration test
#
# Starts 6 real ROS2 nodes (terrainAnalysis, localPlanner, pathFollower,
# pct_planner_astar, pct_path_adapter, static_transform_publisher) plus
# a Python test harness that acts as the "virtual robot".
#
# Verifies the complete goal_pose -> global_path -> way_point -> local_path
# -> cmd_vel -> odometry closed loop.
#
# Platform: S100P robot (192.168.66.190, user: sunrise)
#
# Usage:
#   bash tests/integration/test_full_chain.sh
#
# Prereqs:
#   - ROS2 Humble sourced
#   - Workspace built (install/setup.bash exists)
#   - building2_9.pickle available (auto-detected from install or src)
# ─────────────────────────────────────────────────────────────────────────────

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$WORKSPACE_DIR"

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

# ── PID tracking ──────────────────────────────────────────────────────────────
TF_PID=""
TA_PID=""
LP_PID=""
PF_PID=""
GP_PID=""
PA_PID=""

cleanup() {
    echo ""
    echo -e "${YELLOW}Cleaning up all test nodes...${NC}"
    for pid in $TF_PID $TA_PID $LP_PID $PF_PID $GP_PID $PA_PID; do
        [ -n "$pid" ] && kill "$pid" 2>/dev/null || true
    done
    sleep 1
    # Fallback: kill by name
    pkill -9 -f terrainAnalysis 2>/dev/null || true
    pkill -9 -f localPlanner 2>/dev/null || true
    pkill -9 -f pathFollower 2>/dev/null || true
    pkill -9 -f pct_path_adapter 2>/dev/null || true
    pkill -9 -f pct_planner_astar 2>/dev/null || true
    pkill -9 -f "static_transform_publisher.*map.*odom" 2>/dev/null || true
    pkill -9 -f test_full_chain.py 2>/dev/null || true
}
trap cleanup EXIT

# ── Kill leftover nodes from previous runs ────────────────────────────────────
echo -e "${YELLOW}Killing leftover navigation nodes...${NC}"
pkill -9 -f terrainAnalysis 2>/dev/null || true
pkill -9 -f localPlanner 2>/dev/null || true
pkill -9 -f pathFollower 2>/dev/null || true
pkill -9 -f pct_path_adapter 2>/dev/null || true
pkill -9 -f pct_planner_astar 2>/dev/null || true
pkill -9 -f "static_transform_publisher.*map.*odom" 2>/dev/null || true
sleep 1

echo "=========================================="
echo -e "${BOLD}  T6: Full-chain closed-loop test${NC}"
echo "=========================================="
echo ""

# ── Environment check ─────────────────────────────────────────────────────────
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${RED}ROS2 Humble not found${NC}"
    exit 1
fi
source /opt/ros/humble/setup.bash || { echo -e "${RED}Failed to source ROS2${NC}"; exit 1; }

if [ ! -f "install/setup.bash" ]; then
    echo -e "${RED}install/ not found. Run: make build${NC}"
    exit 1
fi
source install/setup.bash || { echo -e "${RED}Failed to source workspace${NC}"; exit 1; }

# ── Locate tomogram pickle ───────────────────────────────────────────────────
# Try installed path first, then source path
PCT_SHARE=$(ros2 pkg prefix pct_planner 2>/dev/null)/share/pct_planner || true
MAP_FILE=""
for candidate in \
    "${PCT_SHARE}/rsc/pcd/building2_9.pickle" \
    "src/global_planning/PCT_planner/rsc/pcd/building2_9.pickle" \
    "/home/sunrise/data/SLAM/navigation/install/pct_planner/share/pct_planner/rsc/pcd/building2_9.pickle" \
    "/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/pcd/building2_9.pickle"; do
    if [ -f "$candidate" ]; then
        MAP_FILE="$candidate"
        break
    fi
done

if [ -z "$MAP_FILE" ]; then
    echo -e "${RED}building2_9.pickle not found. Cannot run global planner.${NC}"
    echo "  Looked in: \$PCT_SHARE/rsc/pcd/, src/global_planning/PCT_planner/rsc/pcd/"
    exit 1
fi
echo -e "${BLUE}Tomogram: ${MAP_FILE}${NC}"

# ── Locate paths folder for localPlanner ──────────────────────────────────────
LP_SHARE=$(ros2 pkg prefix local_planner 2>/dev/null)/share/local_planner || true
PATHS_DIR="${LP_SHARE}/paths"
if [ ! -d "$PATHS_DIR" ]; then
    PATHS_DIR="install/local_planner/share/local_planner/paths"
fi
echo -e "${BLUE}Paths dir: ${PATHS_DIR}${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="/tmp/t6_logs"
mkdir -p "$LOG_DIR"

# ── Node 1: Static TF map -> odom (identity) ─────────────────────────────────
echo -e "${BLUE}[1/6] Static TF: map -> odom${NC}"
ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 map odom \
    > "$LOG_DIR/tf.log" 2>&1 &
TF_PID=$!

# ── Node 2: terrainAnalysis ──────────────────────────────────────────────────
echo -e "${BLUE}[2/6] terrainAnalysis${NC}"
ros2 run terrain_analysis terrainAnalysis --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /cloud_map:=/nav/map_cloud \
    -r /map_clearing:=/nav/map_clearing \
    -r /terrain_map:=/nav/terrain_map \
    -p scanVoxelSize:=0.1 \
    -p terrainVoxelSize:=0.2 \
    -p obstacleHeightThre:=0.2 \
    -p groundHeightThre:=0.1 \
    > "$LOG_DIR/terrain.log" 2>&1 &
TA_PID=$!

# ── Node 3: localPlanner ────────────────────────────────────────────────────
echo -e "${BLUE}[3/6] localPlanner${NC}"
ros2 run local_planner localPlanner --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /cloud_map:=/nav/map_cloud \
    -r /terrain_map:=/nav/terrain_map \
    -r /terrain_map_ext:=/nav/terrain_map_ext \
    -r /way_point:=/nav/way_point \
    -r /speed:=/nav/speed \
    -r /path:=/nav/local_path \
    -r /stop:=/nav/stop \
    -r /slow_down:=/nav/slow_down \
    -r /navigation_boundary:=/nav/navigation_boundary \
    -r /added_obstacles:=/nav/added_obstacles \
    -r /check_obstacle:=/nav/check_obstacle \
    -p pathFolder:="$PATHS_DIR" \
    -p autonomyMode:=true \
    -p autonomySpeed:=0.8 \
    -p maxSpeed:=1.0 \
    -p useTerrainAnalysis:=true \
    -p checkObstacle:=true \
    -p twoWayDrive:=false \
    -p joyToSpeedDelay:=2.0 \
    > "$LOG_DIR/local_planner.log" 2>&1 &
LP_PID=$!

# ── Node 4: pathFollower ────────────────────────────────────────────────────
echo -e "${BLUE}[4/6] pathFollower${NC}"
ros2 run local_planner pathFollower --ros-args \
    -r /Odometry:=/nav/odometry \
    -r /path:=/nav/local_path \
    -r /speed:=/nav/speed \
    -r /stop:=/nav/stop \
    -r /slow_down:=/nav/slow_down \
    -r /cmd_vel:=/nav/cmd_vel \
    -p autonomyMode:=true \
    -p autonomySpeed:=0.8 \
    -p maxSpeed:=1.0 \
    -p twoWayDrive:=false \
    -p dirDiffThre:=0.5 \
    -p lookAheadDis:=0.5 \
    -p yawRateGain:=3.5 \
    -p stopYawRateGain:=3.5 \
    -p maxYawRate:=45.0 \
    -p maxAccel:=0.5 \
    -p joyToSpeedDelay:=2.0 \
    -p noRotAtGoal:=true \
    > "$LOG_DIR/path_follower.log" 2>&1 &
PF_PID=$!

# ── Node 5: pct_planner_astar (Python A* global planner) ────────────────────
echo -e "${BLUE}[5/6] pct_planner_astar (Python A*)${NC}"
# Find the script in the installed share directory
PCT_SCRIPT="${PCT_SHARE}/planner/scripts/pct_planner_astar.py"
if [ ! -f "$PCT_SCRIPT" ]; then
    # Fallback: source tree
    PCT_SCRIPT="src/global_planning/PCT_planner/planner/scripts/pct_planner_astar.py"
fi
if [ ! -f "$PCT_SCRIPT" ]; then
    echo -e "${RED}pct_planner_astar.py not found${NC}"
    exit 1
fi

python3 "$PCT_SCRIPT" --ros-args \
    -p tomogram_file:="$MAP_FILE" \
    -p obstacle_thr:=49.9 \
    -p republish_hz:=0.02 \
    -p astar_timeout_sec:=5.0 \
    > "$LOG_DIR/global_planner.log" 2>&1 &
GP_PID=$!

# ── Node 6: pct_path_adapter ────────────────────────────────────────────────
echo -e "${BLUE}[6/6] pct_path_adapter${NC}"
ros2 run pct_adapters pct_path_adapter --ros-args \
    -r /pct_path:=/nav/global_path \
    -r /Odometry:=/nav/odometry \
    -r /planner_waypoint:=/nav/way_point \
    -p waypoint_distance:=1.5 \
    -p arrival_threshold:=0.8 \
    -p stuck_timeout_sec:=60.0 \
    -p lookahead_dist:=2.0 \
    > "$LOG_DIR/adapter.log" 2>&1 &
PA_PID=$!

# ── Wait for nodes to start ─────────────────────────────────────────────────
echo ""
echo -e "${BLUE}Waiting for nodes to initialize (10s)...${NC}"
for i in $(seq 1 10); do
    echo -n "."
    sleep 1
done
echo ""

# ── Verify all PIDs still alive ──────────────────────────────────────────────
echo ""
echo -e "${YELLOW}Node liveness check:${NC}"
ALL_ALIVE=true
for NAME_PID in "TF:$TF_PID" "terrain:$TA_PID" "localPlanner:$LP_PID" "pathFollower:$PF_PID" "globalPlanner:$GP_PID" "adapter:$PA_PID"; do
    NAME="${NAME_PID%%:*}"
    PID="${NAME_PID##*:}"
    if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
        echo -e "  ${GREEN}OK${NC}  $NAME (PID=$PID)"
    else
        echo -e "  ${RED}DEAD${NC}  $NAME (PID=$PID)"
        ALL_ALIVE=false
    fi
done

if [ "$ALL_ALIVE" = false ]; then
    echo ""
    echo -e "${RED}Some nodes died during startup. Check logs:${NC}"
    echo "  cat $LOG_DIR/terrain.log"
    echo "  cat $LOG_DIR/local_planner.log"
    echo "  cat $LOG_DIR/path_follower.log"
    echo "  cat $LOG_DIR/global_planner.log"
    echo "  cat $LOG_DIR/adapter.log"
    exit 1
fi
echo ""

# ── Run the Python test harness ──────────────────────────────────────────────
echo -e "${BOLD}Running test harness...${NC}"
echo ""

python3 "${SCRIPT_DIR}/test_full_chain.py" \
    --map-file "$MAP_FILE" \
    --start-x -5.5 --start-y 7.3 \
    --goal-x 5.0 --goal-y 7.3 \
    --timeout 90
TEST_EXIT=$?

echo ""
if [ $TEST_EXIT -eq 0 ]; then
    echo -e "${GREEN}T6 Full-chain closed-loop test PASSED${NC}"
else
    echo -e "${RED}T6 Full-chain closed-loop test FAILED${NC}"
    echo ""
    echo "Logs:"
    echo "  cat $LOG_DIR/terrain.log"
    echo "  cat $LOG_DIR/local_planner.log"
    echo "  cat $LOG_DIR/path_follower.log"
    echo "  cat $LOG_DIR/global_planner.log"
    echo "  cat $LOG_DIR/adapter.log"
    echo "  cat /tmp/t6_result.json"
fi

# cleanup via trap
exit $TEST_EXIT
