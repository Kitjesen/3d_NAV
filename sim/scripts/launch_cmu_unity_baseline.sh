#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  sim/scripts/launch_cmu_unity_baseline.sh start [--rviz]
  sim/scripts/launch_cmu_unity_baseline.sh resume
  sim/scripts/launch_cmu_unity_baseline.sh rviz
  sim/scripts/launch_cmu_unity_baseline.sh status
  sim/scripts/launch_cmu_unity_baseline.sh stop

Environment:
  LINGTU_CMU_AUTONOMY_WS   CMU autonomy_stack workspace
  LINGTU_CMU_BASELINE_DIR  log/artifact directory
  ROS_DOMAIN_ID            isolated ROS 2 domain, defaults to 74
  DISPLAY                  GUI display, defaults to :1
  LINGTU_CMU_BASELINE_RVIZ optional RViz config override
  LINGTU_CMU_BASELINE_TARE_SCENARIO
                           defaults to indoor_large
  LINGTU_CMU_BASELINE_AUTO_RESUME
                           defaults to 1; sends the RViz Resume Navigation
                           joy message so CMU waypoint mode starts without a
                           manual click

This is a CMU reference run only. It does not start LingTu modules, does not
validate LingTu planning, and must not be reported as LingTu product closure.
EOF
}

repo_root() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  cd "$script_dir/../.." && pwd
}

setup_runtime_env() {
  local cmu_ws="$1"
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$cmu_ws/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-74}"
  export DISPLAY="${DISPLAY:-:1}"
  export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
  export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
}

kill_pattern() {
  local pattern="$1"
  local signal="${2:-TERM}"
  local pid
  while IFS= read -r pid; do
    [[ -z "$pid" || "$pid" == "$$" || "$pid" == "${PPID:-}" ]] && continue
    kill "-$signal" "$pid" 2>/dev/null || true
  done < <(pgrep -f "$pattern" 2>/dev/null || true)
}

stop_baseline() {
  local cmu_ws="${LINGTU_CMU_AUTONOMY_WS:-/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform}"
  kill_pattern '[l]aunch_cmu_unity_baseline.sh start'
  kill_pattern '[s]ystem_simulation_with_exploration_planner.launch'
  kill_pattern '[s]ystem_simulation.launch'
  kill_pattern '[r]viz2.*vehicle_simulator.rviz'
  kill_pattern '[r]viz2.*tare_planner_ground.rviz'
  kill_pattern '[j]oy_node'
  kill_pattern '[M]odel.x86_64'
  kill_pattern "$cmu_ws/install/"
  kill_pattern "$cmu_ws/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
  sleep 1
  kill_pattern '[l]aunch_cmu_unity_baseline.sh start' KILL
  kill_pattern '[s]ystem_simulation_with_exploration_planner.launch' KILL
  kill_pattern '[s]ystem_simulation.launch' KILL
  kill_pattern '[r]viz2.*vehicle_simulator.rviz' KILL
  kill_pattern '[r]viz2.*tare_planner_ground.rviz' KILL
  kill_pattern '[j]oy_node' KILL
  kill_pattern '[M]odel.x86_64' KILL
  kill_pattern "$cmu_ws/install/" KILL
  kill_pattern "$cmu_ws/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64" KILL
}

status_baseline() {
  pgrep -af 'rviz2|joy_node|Model.x86_64|default_server_endpoint|sensorScanGeneration|terrainAnalysis|tare_planner_node|vehicleSimulator|pathFollower|localPlanner|system_simulation_with_exploration_planner.launch' || true
}

publish_resume_once() {
  local cmu_ws="${LINGTU_CMU_AUTONOMY_WS:-/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform}"
  setup_runtime_env "$cmu_ws"
  /usr/bin/python3 - <<'PY'
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

rclpy.init()
node = Node("lingtu_cmu_baseline_resume")
pub = node.create_publisher(Joy, "/joy", 5)
msg = Joy()
msg.header.frame_id = "lingtu_cmu_baseline_resume"
msg.axes = [0.0, 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0]
msg.buttons = [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
for _ in range(10):
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
PY
}

start_resume_guard() {
  local run_dir="$1"
  (
    for _ in $(seq 1 18); do
      publish_resume_once
      sleep 5
    done
  ) > "$run_dir/resume_guard.log" 2>&1 &
  echo $! > "$run_dir/resume_guard.pid"
}

wait_for_endpoint() {
  local run_dir="$1"
  for i in $(seq 1 35); do
    if ss -ltn 2>/dev/null | grep -q ':10000'; then
      echo "endpoint_ready_after=${i}s" | tee "$run_dir/startup_state.txt"
      return 0
    fi
    sleep 1
  done
  echo "endpoint_not_ready" | tee "$run_dir/startup_state.txt"
  return 1
}

launch_rviz() {
  local root cmu_ws run_dir rviz_config
  root="$(repo_root)"
  cmu_ws="${LINGTU_CMU_AUTONOMY_WS:-/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform}"
  run_dir="${LINGTU_CMU_BASELINE_DIR:-$root/artifacts/server_sim_closure/cmu_unity_baseline_latest}"
  rviz_config="${LINGTU_CMU_BASELINE_RVIZ:-$cmu_ws/src/exploration_planner/tare_planner/rviz/tare_planner_ground.rviz}"

  setup_runtime_env "$cmu_ws"
  mkdir -p "$run_dir"
  if [[ ! -f "$rviz_config" ]]; then
    echo "CMU baseline RViz config missing: $rviz_config" >&2
    return 2
  fi
  (
    cd "$cmu_ws"
    rviz2 -d "$rviz_config"
  ) > "$run_dir/rviz.log" 2>&1 &
  echo $! > "$run_dir/rviz.pid"
  echo "CMU baseline RViz started"
  echo "  config=$rviz_config"
  echo "  log=$run_dir/rviz.log"
}

start_baseline() {
  local run_rviz="$1"
  local root cmu_ws run_dir scenario
  root="$(repo_root)"
  cmu_ws="${LINGTU_CMU_AUTONOMY_WS:-/home/bsrl/hongsenpang/autonomy_stack_mecanum_wheel_platform}"
  run_dir="${LINGTU_CMU_BASELINE_DIR:-$root/artifacts/server_sim_closure/cmu_unity_baseline_latest}"
  scenario="${LINGTU_CMU_BASELINE_TARE_SCENARIO:-indoor_large}"

  mkdir -p "$run_dir"
  run_dir="$(cd "$run_dir" && pwd)"
  stop_baseline
  setup_runtime_env "$cmu_ws"

  (
    cd "$cmu_ws"
    ros2 launch vehicle_simulator system_simulation_with_exploration_planner.launch \
      exploration_planner_config:="$scenario"
  ) > "$run_dir/cmu_ros_launch.log" 2>&1 &
  echo $! > "$run_dir/cmu_ros_launch.pid"

  wait_for_endpoint "$run_dir"
  (
    cd "$cmu_ws"
    ./src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64
  ) > "$run_dir/unity.log" 2>&1 &
  echo $! > "$run_dir/unity.pid"

  echo "CMU Unity baseline started"
  echo "  ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
  echo "  DISPLAY=$DISPLAY"
  echo "  logs=$run_dir"
  echo "  TARE scenario=$scenario"
  echo "  LingTu modules are not started in this baseline run"

  if [[ "$run_rviz" == "1" ]]; then
    sleep 5
    launch_rviz
  fi
  if [[ "${LINGTU_CMU_BASELINE_AUTO_RESUME:-1}" == "1" ]]; then
    start_resume_guard "$run_dir"
    echo "  auto_resume=1"
  fi
}

cmd="${1:-start}"
shift || true
case "$cmd" in
  start)
    rviz=0
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --rviz) rviz=1 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage; exit 2 ;;
      esac
      shift
    done
    start_baseline "$rviz"
    ;;
  resume)
    publish_resume_once
    ;;
  rviz)
    launch_rviz
    ;;
  status)
    status_baseline
    ;;
  stop)
    stop_baseline
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown command: $cmd" >&2
    usage
    exit 2
    ;;
esac
