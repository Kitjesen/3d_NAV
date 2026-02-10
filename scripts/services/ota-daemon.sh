#!/bin/bash
# ── ota-daemon.service 启动脚本 ──
# 启动 OTA Daemon (文件管理/OTA更新, 端口 50052)
set -e
NAV_DIR="/home/sunrise/data/SLAM/navigation"
CONFIG="${NAV_DIR}/src/ota_daemon/config/ota_daemon.yaml"

echo "[ota-daemon] Starting OTA Daemon on port 50052..."
exec "${NAV_DIR}/install/ota_daemon/bin/ota_daemon" -c "$CONFIG"
