#!/usr/bin/env bash
set -euo pipefail

# Build every ROS 2 package discovered under src/ and verify that installed
# ELF executables do not have unresolved dynamic libraries.
# This is non-motion: it only builds and inspects software artifacts.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "${ROOT}"

ROS_DISTRO="${LINGTU_ROS_DISTRO:-humble}"
INSTALL_SYSTEM_DEPS="${LINGTU_INSTALL_SYSTEM_DEPS:-1}"
SKIP_APT="${LINGTU_SKIP_APT:-0}"
SUDO_PASSWORD="${LINGTU_SUDO_PASSWORD:-}"
BUILD_TARE="${LINGTU_BUILD_TARE:-1}"
BUILD_GNSS="${LINGTU_BUILD_GNSS:-1}"
BUILD_TYPE="${LINGTU_BUILD_TYPE:-Release}"

log() {
  printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"
}

have() {
  command -v "$1" >/dev/null 2>&1
}

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

source_ros() {
  if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    set +u
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
  elif ! have ros2; then
    printf 'ERROR: ROS 2 not found. Set LINGTU_ROS_DISTRO or source ROS before running.\n' >&2
    return 1
  fi

  if [[ -f "${ROOT}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    set +u
    source "${ROOT}/install/setup.bash"
    set -u
  fi
}

install_system_deps() {
  if [[ "${INSTALL_SYSTEM_DEPS}" != "1" || "${SKIP_APT}" == "1" ]]; then
    return
  fi

  local packages=(
    build-essential
    cmake
    python3-colcon-common-extensions
    libboost-all-dev
    libeigen3-dev
    libpcl-dev
    libyaml-cpp-dev
    libgoogle-glog-dev
    "ros-${ROS_DISTRO}-pcl-ros"
    "ros-${ROS_DISTRO}-pcl-conversions"
    "ros-${ROS_DISTRO}-tf2-geometry-msgs"
    "ros-${ROS_DISTRO}-sophus"
  )
  local missing=()
  local pkg
  for pkg in "${packages[@]}"; do
    if ! dpkg -s "$pkg" >/dev/null 2>&1; then
      missing+=("$pkg")
    fi
  done
  if [[ "${#missing[@]}" -eq 0 ]]; then
    log "system build dependencies already installed"
    return
  fi

  log "installing system build dependencies: ${missing[*]}"
  sudo_cmd apt-get update
  sudo_cmd apt-get install -y "${missing[@]}"
}

prepare_optional_packages() {
  local skip=()

  if [[ "${BUILD_TARE}" == "1" && -d src/exploration/tare_planner ]]; then
    if [[ ! -f src/exploration/tare_planner/or-tools/lib/libortools.so ]]; then
      log "fetching OR-Tools for tare_planner"
      bash scripts/build/fetch_ortools.sh
    fi
  else
    skip+=(tare_planner)
  fi

  if [[ "${BUILD_GNSS}" != "1" ]]; then
    skip+=(wtrtk980_ros2_reader)
  fi

  if [[ "${#skip[@]}" -gt 0 ]]; then
    printf '%s\n' "${skip[@]}"
  fi
}

build_workspace() {
  source_ros
  install_system_deps

  local skip_packages=()
  mapfile -t skip_packages < <(prepare_optional_packages)

  local gtsam_dir="${ROOT}/src/global_planning/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib/cmake/GTSAM"
  local cmake_args=(-DCMAKE_BUILD_TYPE="${BUILD_TYPE}")
  if [[ -d "${gtsam_dir}" ]]; then
    export CMAKE_PREFIX_PATH="${ROOT}/src/global_planning/pct_planner/planner/lib/3rdparty/gtsam-4.1.1/install:${CMAKE_PREFIX_PATH:-}"
    cmake_args+=(-DGTSAM_DIR="${gtsam_dir}")
  fi

  log "discovered ROS packages"
  colcon list --base-paths src

  local cmd=(colcon build --base-paths src --event-handlers console_direct+)
  if [[ "${#skip_packages[@]}" -gt 0 ]]; then
    cmd+=(--packages-skip "${skip_packages[@]}")
  fi
  cmd+=(--cmake-args "${cmake_args[@]}")

  log "building ROS workspace"
  printf 'command:'
  printf ' %q' "${cmd[@]}"
  printf '\n'
  "${cmd[@]}"
}

verify_installed_elfs() {
  log "checking installed ELF runtime dependencies"
  local missing=0
  local f
  while IFS= read -r f; do
    if file "$f" | grep -q 'ELF'; then
      local bad
      bad="$(ldd "$f" | grep 'not found' || true)"
      if [[ -n "${bad}" ]]; then
        printf 'MISSING_LIB:%s\n%s\n' "$f" "$bad"
        missing=1
      fi
    fi
  done < <(find install -type f -perm /111)

  if [[ "${missing}" -ne 0 ]]; then
    return 1
  fi
  printf 'ELF_LDD_OK\n'
}

build_workspace
source_ros
verify_installed_elfs

log "ROS workspace build complete"
