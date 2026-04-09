#!/bin/bash
# Install LingTu service files to S100P
# Run from repo root: bash docs/04-deployment/services/install.sh
set -e

SERVICES_DIR="$(cd "$(dirname "$0")" && pwd)"
SYSTEMD_DIR="/etc/systemd/system"
LINGTU_CONFIG="/opt/lingtu/config"

echo "=== Installing LingTu service files ==="

# 1. Create config dir
sudo mkdir -p "$LINGTU_CONFIG"

# 2. Install ros2-env.sh
sudo cp "$SERVICES_DIR/ros2-env.sh" "$LINGTU_CONFIG/ros2-env.sh"
sudo chmod +x "$LINGTU_CONFIG/ros2-env.sh"
echo "[ok] $LINGTU_CONFIG/ros2-env.sh"

# 3. Install service files
for svc in robot-lidar robot-camera robot-brainstem robot-fastlio2 robot-localizer lingtu; do
  sudo cp "$SERVICES_DIR/${svc}.service" "$SYSTEMD_DIR/${svc}.service"
  echo "[ok] $SYSTEMD_DIR/${svc}.service"
done

# 4. Reload systemd
sudo systemctl daemon-reload
echo "[ok] systemd daemon-reload"

# 5. Enable all services
sudo systemctl enable robot-lidar robot-camera robot-brainstem robot-fastlio2 robot-localizer lingtu
echo "[ok] all services enabled"

echo ""
echo "=== Done. Verify with: ==="
echo "  systemctl list-unit-files | grep -E 'robot-|lingtu'"
echo ""
echo "=== To start everything now: ==="
echo "  sudo systemctl start robot-lidar robot-camera robot-brainstem robot-fastlio2 robot-localizer lingtu"
