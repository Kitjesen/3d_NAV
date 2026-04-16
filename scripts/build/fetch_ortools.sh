#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════
# Fetch Google OR-Tools (Apache 2.0) for the TARE planner.
#
# TARE's CMakeLists links against
#   src/exploration/tare_planner/or-tools/lib/libortools.so
# and includes headers from
#   src/exploration/tare_planner/or-tools/include/
#
# The binary is ~130 MB per arch so we do NOT commit it. Run this script
# once per machine before the first ``build_tare.sh``.
# ══════════════════════════════════════════════════════════
set -euo pipefail

VERSION="${ORTOOLS_VERSION:-9.10.4067}"
_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$_SCRIPT_DIR/../.." && pwd)"
DEST="$WORKSPACE_DIR/src/exploration/tare_planner/or-tools"

ARCH="$(uname -m)"
case "$ARCH" in
    x86_64|amd64)
        OT_ARCH="amd64"
        ;;
    aarch64|arm64)
        OT_ARCH="arm64"
        ;;
    *)
        echo "ERROR: unsupported architecture '$ARCH' — OR-Tools only ships amd64 / arm64"
        exit 1
        ;;
esac

# OR-Tools dropped Ubuntu detection after 9.x; 22.04 tarballs work for Humble.
TARBALL="or-tools_${OT_ARCH}_ubuntu-22.04_cpp_v${VERSION}.tar.gz"
URL="https://github.com/google/or-tools/releases/download/v${VERSION%.*}/${TARBALL}"

echo ">>> [OR-Tools] Fetching ${TARBALL}"
echo "    from ${URL}"
mkdir -p "$DEST"
cd "$DEST"

if [ -f "lib/libortools.so" ]; then
    echo ">>> [OR-Tools] Already installed at $DEST — delete to re-fetch."
    exit 0
fi

TMPDIR_T="$(mktemp -d)"
trap 'rm -rf "$TMPDIR_T"' EXIT

# Prefer curl, fall back to wget
if command -v curl >/dev/null; then
    curl -fL --progress-bar "$URL" -o "$TMPDIR_T/$TARBALL"
elif command -v wget >/dev/null; then
    wget -q --show-progress "$URL" -O "$TMPDIR_T/$TARBALL"
else
    echo "ERROR: need curl or wget to fetch OR-Tools"
    exit 1
fi

echo ">>> [OR-Tools] Extracting..."
tar -xzf "$TMPDIR_T/$TARBALL" -C "$TMPDIR_T"

# Tarball top-level dir: or-tools_${OT_ARCH}_ubuntu-22.04_cpp_v${VERSION}
EXTRACTED="$(find "$TMPDIR_T" -maxdepth 1 -type d -name 'or-tools_*' | head -1)"
if [ -z "$EXTRACTED" ]; then
    echo "ERROR: extraction produced no or-tools_* directory"
    exit 1
fi

# Copy lib + include only (skip docs, examples, etc.)
mkdir -p "$DEST/lib" "$DEST/include"
cp -r "$EXTRACTED/lib/"*     "$DEST/lib/"
cp -r "$EXTRACTED/include/"* "$DEST/include/"

echo ">>> [OR-Tools] Installed to $DEST/"
echo ">>> Arch: $OT_ARCH  version: $VERSION"
ls "$DEST/lib/" | head -5
