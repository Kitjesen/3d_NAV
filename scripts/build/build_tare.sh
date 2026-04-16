#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════
# TARE Planner colcon build
#
# Vendored in-tree under src/exploration/tare_planner (BSD, per upstream
# package.xml). OR-Tools binaries are NOT committed — run
# scripts/build/fetch_ortools.sh first to download them for your arch.
# ══════════════════════════════════════════════════════════
set -e

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$_SCRIPT_DIR/../.." && pwd)"
TARE_DIR="$WORKSPACE_DIR/src/exploration/tare_planner"
ORTOOLS_DIR="$TARE_DIR/or-tools"

if [ ! -f "$TARE_DIR/CMakeLists.txt" ]; then
    echo "ERROR: $TARE_DIR/CMakeLists.txt not found — vendored tree missing?"
    exit 1
fi

if [ ! -f "$ORTOOLS_DIR/lib/libortools.so" ]; then
    echo ">>> [TARE] OR-Tools not yet fetched. Running fetch_ortools.sh..."
    bash "$_SCRIPT_DIR/fetch_ortools.sh"
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

echo ">>> [TARE] colcon build tare_planner (from LingTu workspace)..."
cd "$WORKSPACE_DIR"
# Build only the tare_planner package — colcon will auto-discover it by
# walking the workspace and picking up the package.xml at
# src/exploration/tare_planner/.
colcon build \
    --symlink-install \
    --packages-select tare_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ">>> [TARE] Done. Source LingTu's install:"
echo "    source $WORKSPACE_DIR/install/setup.bash"
