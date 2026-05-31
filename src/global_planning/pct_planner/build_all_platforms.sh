#!/usr/bin/env bash
# Build PCT planner .so for x86_64 and aarch64 via Docker.
#
# Usage:
#   bash build_all_platforms.sh          # build both
#   bash build_all_platforms.sh x86_64   # x86_64 only
#   bash build_all_platforms.sh aarch64  # aarch64 only
#
# Output:
#   planner/lib/x86_64/*.so
#   planner/lib/aarch64/*.so

set -euo pipefail
cd "$(dirname "$0")"

build_arch() {
    local arch=$1
    local platform=""
    local tag="pct-build-${arch}"
    local outdir="planner/lib/${arch}"

    case $arch in
        x86_64)  platform="linux/amd64" ;;
        aarch64) platform="linux/arm64" ;;
        *) echo "Unknown arch: $arch"; exit 1 ;;
    esac

    echo "=== Building PCT for ${arch} (${platform}) ==="
    mkdir -p "$outdir"

    docker buildx build \
        --platform "$platform" \
        -f Dockerfile.build \
        -t "$tag" \
        --load \
        .

    docker run --rm -v "$(pwd)/${outdir}:/output" "$tag"
    echo "=== ${arch} done: $(ls ${outdir}/*.so 2>/dev/null | wc -l) .so files ==="
    ls -la "${outdir}/"*.so 2>/dev/null
    echo ""
}

TARGET=${1:-all}

case $TARGET in
    x86_64)  build_arch x86_64 ;;
    aarch64) build_arch aarch64 ;;
    all)
        build_arch x86_64
        build_arch aarch64
        ;;
    *) echo "Usage: $0 [x86_64|aarch64|all]"; exit 1 ;;
esac

echo "=== All builds complete ==="
