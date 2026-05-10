#!/usr/bin/env bash
set -euo pipefail

# Build OR-Tools from source and install it where the vendored TARE planner
# expects it. Use this on hosts where scripts/build/fetch_ortools.sh cannot
# provide a usable libortools.so for the current architecture.
#
# Output:
#   src/exploration/tare_planner/or-tools/lib/libortools.so
#
# Typical use:
#   bash scripts/build/build_ortools_from_source.sh
#   bash scripts/build/build_tare.sh

FORCE=0
for arg in "$@"; do
  if [[ "$arg" == "--force" ]]; then
    FORCE=1
  fi
done

VERSION="${ORTOOLS_VERSION:-v9.15}"
JOBS="${BUILD_JOBS:-$(nproc 2>/dev/null || echo 4)}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DEST="${WORKSPACE_DIR}/src/exploration/tare_planner/or-tools"
SRC_DIR="${ORTOOLS_SRC_DIR:-${HOME}/src/or-tools}"

if [[ -f "${DEST}/lib/libortools.so" && "${FORCE}" -eq 0 ]]; then
  echo "[ortools] ${DEST}/lib/libortools.so exists; skip (use --force to rebuild)"
  exit 0
fi

echo "[ortools] 1/5 check build dependencies"
for cmd in cmake git make g++ curl tar; do
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "missing: ${cmd} (install build-essential cmake git curl tar)"
    exit 1
  fi
done

echo "[ortools] 2/5 fetch source tarball (${VERSION}) -> ${SRC_DIR}"
if [[ ! -f "${SRC_DIR}/CMakeLists.txt" ]]; then
  mkdir -p "$(dirname "${SRC_DIR}")"
  tarball="/tmp/or-tools-${VERSION}.tar.gz"
  url="https://codeload.github.com/google/or-tools/tar.gz/refs/tags/${VERSION}"
  echo "[ortools] downloading ${url}"
  for attempt in 1 2 3 4 5; do
    if curl -fL --progress-bar -C - -o "${tarball}" "${url}"; then
      echo "[ortools] download OK (attempt ${attempt})"
      break
    fi
    echo "[ortools] attempt ${attempt} failed, retry in 10s"
    sleep 10
  done
  if [[ ! -s "${tarball}" ]]; then
    echo "[ortools] tarball is empty after retries"
    exit 1
  fi
  rm -rf "${SRC_DIR}"
  mkdir -p "${SRC_DIR}"
  tar -xzf "${tarball}" -C "${SRC_DIR}" --strip-components=1
else
  echo "[ortools] source tree already present; skip fetch"
fi

echo "[ortools] 3/5 configure CMake (BUILD_DEPS=ON)"
cd "${SRC_DIR}"
rm -rf build
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_DEPS=ON \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_SAMPLES=OFF \
  -DBUILD_TESTING=OFF \
  -DUSE_PYTHON=OFF \
  -DUSE_DOTNET=OFF \
  -DUSE_JAVA=OFF \
  -DBUILD_PYTHON=OFF \
  -DCMAKE_INSTALL_PREFIX="${DEST}"

echo "[ortools] 4/5 build (${JOBS} jobs)"
cmake --build build -j"${JOBS}"

echo "[ortools] 5/5 install to ${DEST}"
cmake --install build

if [[ -f "${DEST}/lib/libortools.so" ]]; then
  echo "[ortools] SUCCESS"
  ls -la "${DEST}/lib/libortools.so"
  echo "Next: bash scripts/build/build_tare.sh"
else
  echo "[ortools] FAIL: libortools.so was not produced"
  exit 2
fi
