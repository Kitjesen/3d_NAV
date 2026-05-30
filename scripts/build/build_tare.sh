#!/usr/bin/env bash
set -euo pipefail

# Build the vendored TARE planner inside the LingTu ROS 2 workspace.
# OR-Tools binaries are intentionally not committed; fetch them once per host.

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${_SCRIPT_DIR}/../.." && pwd)"
TARE_DIR="${WORKSPACE_DIR}/src/exploration/tare_planner"
ORTOOLS_DIR="${TARE_DIR}/or-tools"
ROS_DISTRO="${LINGTU_ROS_DISTRO:-humble}"
SKIP_APT="${LINGTU_SKIP_APT:-0}"
SUDO_PASSWORD="${LINGTU_SUDO_PASSWORD:-}"

sudo_cmd() {
    if [[ "${SKIP_APT}" == "1" ]]; then
        return 1
    fi
    if [[ "$(id -u)" == "0" ]]; then
        "$@"
    elif sudo -n true >/dev/null 2>&1; then
        sudo "$@"
    elif [[ -n "${SUDO_PASSWORD}" ]]; then
        printf '%s\n' "${SUDO_PASSWORD}" | sudo -S "$@"
    else
        sudo "$@"
    fi
}

install_tare_deps() {
    if [[ "${SKIP_APT}" == "1" ]]; then
        echo ">>> [TARE] LINGTU_SKIP_APT=1; skipping apt prerequisites"
        return
    fi

    local packages=(
        libabsl-dev
        libgoogle-glog-dev
        libpcl-dev
        "ros-${ROS_DISTRO}-pcl-ros"
        "ros-${ROS_DISTRO}-pcl-conversions"
        "ros-${ROS_DISTRO}-tf2-geometry-msgs"
    )
    local missing=()
    local pkg
    for pkg in "${packages[@]}"; do
        if ! dpkg -s "$pkg" >/dev/null 2>&1; then
            missing+=("$pkg")
        fi
    done
    if [[ "${#missing[@]}" -eq 0 ]]; then
        echo ">>> [TARE] apt prerequisites already installed"
        return
    fi

    echo ">>> [TARE] apt prerequisites: ${missing[*]}"
    sudo_cmd apt-get update
    sudo_cmd apt-get install -y "${missing[@]}"
}

if [[ ! -f "${TARE_DIR}/CMakeLists.txt" ]]; then
    echo "ERROR: ${TARE_DIR}/CMakeLists.txt not found; vendored tree missing?"
    exit 1
fi

if [[ ! -f "${ORTOOLS_DIR}/lib/libortools.so" ]]; then
    echo ">>> [TARE] OR-Tools not yet fetched. Running fetch_ortools.sh..."
    bash "${_SCRIPT_DIR}/fetch_ortools.sh"
fi

echo ">>> [TARE] Sourcing ROS 2 ${ROS_DISTRO}..."
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck source=/dev/null
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
else
    echo "WARNING: /opt/ros/${ROS_DISTRO}/setup.bash not found; assuming already sourced"
fi

echo ">>> [TARE] apt prerequisites..."
install_tare_deps

echo ">>> [TARE] colcon build tare_planner (from LingTu workspace)..."
cd "${WORKSPACE_DIR}"
COLCON_LAYOUT_ARGS=()
if [[ -f "${WORKSPACE_DIR}/install/.colcon_install_layout" ]] \
    && grep -qx "merged" "${WORKSPACE_DIR}/install/.colcon_install_layout"; then
    COLCON_LAYOUT_ARGS+=(--merge-install)
fi
colcon build \
    "${COLCON_LAYOUT_ARGS[@]}" \
    --symlink-install \
    --packages-select tare_planner \
    --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ">>> [TARE] Done. Source LingTu's install:"
echo "    source ${WORKSPACE_DIR}/install/setup.bash"
