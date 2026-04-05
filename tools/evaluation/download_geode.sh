#!/bin/bash
# Download GEODE dataset (degeneracy benchmark) for offline evaluation.
#
# GEODE: 64 trajectories, 7 degeneracy scenarios (tunnel/corridor/open/mixed),
# Multi-LiDAR (Velodyne, Ouster, Livox) + stereo camera + IMU
# Paper: https://github.com/PengYu-Team/GEODE_dataset
#
# Usage:
#   bash tools/evaluation/download_geode.sh [output_dir]
#   bash tools/evaluation/download_geode.sh ~/data/geode
#
# Default output: ~/data/geode/

set -e

OUTPUT_DIR="${1:-$HOME/data/geode}"
mkdir -p "$OUTPUT_DIR"

echo "=== GEODE Dataset Download ==="
echo "Output: $OUTPUT_DIR"
echo ""

# GEODE is hosted on Google Drive / Baidu Pan
# The repo README has download links — clone the repo first for metadata
if [ ! -d "$OUTPUT_DIR/GEODE_dataset" ]; then
    echo "[1/3] Cloning GEODE_dataset repo (metadata + ground truth)..."
    git clone --depth=1 https://github.com/PengYu-Team/GEODE_dataset.git "$OUTPUT_DIR/GEODE_dataset" 2>/dev/null || {
        echo "WARN: Failed to clone GEODE repo. Continuing with manual download instructions."
    }
else
    echo "[1/3] GEODE repo already cloned."
fi

echo ""
echo "[2/3] Dataset download instructions:"
echo ""
echo "  GEODE rosbags are large (30-100GB total). Download specific sequences:"
echo ""
echo "  RECOMMENDED for degeneracy testing (Livox Mid-360 compatible):"
echo "    - corridor_01 — long indoor corridor (primary degeneracy test)"
echo "    - corridor_02 — L-shaped corridor"
echo "    - tunnel_01   — narrow tunnel"
echo "    - open_01     — open field / parking lot"
echo "    - mixed_01    — corridor → outdoor transition"
echo ""
echo "  Download from GEODE project page:"
echo "    https://github.com/PengYu-Team/GEODE_dataset"
echo ""
echo "  Place rosbags in: $OUTPUT_DIR/bags/"
echo ""

mkdir -p "$OUTPUT_DIR/bags"
mkdir -p "$OUTPUT_DIR/results"

# Also download the ntnu-arl tunnel dataset (smaller, faster to test)
echo "[3/3] Downloading ntnu-arl degeneracy dataset (tunnel + corridor)..."
if [ ! -d "$OUTPUT_DIR/ntnu_degeneracy" ]; then
    git clone --depth=1 https://github.com/ntnu-arl/lidar_degeneracy_datasets.git "$OUTPUT_DIR/ntnu_degeneracy" 2>/dev/null || {
        echo "WARN: Failed to clone ntnu dataset repo."
    }
else
    echo "  ntnu dataset repo already cloned."
fi

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "  1. Download GEODE rosbags to $OUTPUT_DIR/bags/"
echo "  2. Run evaluation:"
echo "     python tools/evaluation/evaluate_degeneracy.py \\"
echo "       --bag $OUTPUT_DIR/bags/corridor_01.bag \\"
echo "       --backend fastlio2 \\"
echo "       --output $OUTPUT_DIR/results/"
echo ""
