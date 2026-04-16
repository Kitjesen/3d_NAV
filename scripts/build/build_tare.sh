#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════
# TARE Planner colcon build
#
# Third-party: https://github.com/caochao39/tare_planner (humble-jazzy)
# Clone via `git submodule update --init --recursive` first.
#
# On S100P (aarch64) OR-Tools is vendored inside the repo and supported
# by upstream — no extra install step needed. Depends on PCL + Eigen +
# libgoogle-glog-dev from apt.
# ══════════════════════════════════════════════════════════
set -e

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$_SCRIPT_DIR/../.." && pwd)"
TARE_DIR="$WORKSPACE_DIR/third_party/tare_planner"

if [ ! -d "$TARE_DIR/src/tare_planner" ]; then
    echo "ERROR: $TARE_DIR/src/tare_planner not found."
    echo "Run first:  git submodule update --init --recursive"
    exit 1
fi

echo ">>> [TARE] Sourcing ROS 2 Humble..."
if [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck source=/dev/null
    source /opt/ros/humble/setup.bash
else
    echo "WARNING: /opt/ros/humble/setup.bash not found — assuming already sourced"
fi

echo ">>> [TARE] apt prerequisites..."
# Idempotent — skip if already installed
if ! dpkg -s libgoogle-glog-dev >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y libgoogle-glog-dev libpcl-dev
fi

echo ">>> [TARE] colcon build tare_planner..."
cd "$TARE_DIR"
# Only build the tare_planner package — ignore vehicle_simulator etc.
colcon build \
    --symlink-install \
    --packages-select tare_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ">>> [TARE] Done. Source with:"
echo "    source $TARE_DIR/install/setup.bash"
