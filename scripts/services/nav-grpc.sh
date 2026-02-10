#!/bin/bash
# ── nav-grpc.service 启动脚本 ──
# 启动 gRPC Gateway (远程监控/控制入口)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/env.sh"

echo "[nav-grpc] Starting gRPC Gateway on port 50051..."
exec ros2 launch remote_monitoring grpc_gateway.launch.py
