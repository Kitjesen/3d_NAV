#!/usr/bin/env bash
#
# install_gnss.sh — 安装 WTRTK-980 GNSS 驱动到 S100P
#
# 使用:
#   sudo bash scripts/deploy/install_gnss.sh
#
# 流程:
#   1. 安装 ironoa/um982_ros2_driver 到 /opt/lingtu/gnss/
#   2. 配置 /dev/ttyUSB0 udev 规则 (可选)
#   3. 安装 gnss.service systemd 单元
#   4. enable + start
#
# 前置条件:
#   - ROS2 Humble 已安装
#   - WTRTK-980 已插入 USB 口
#   - 当前用户在 dialout 组 (sudo usermod -a -G dialout sunrise)

set -euo pipefail

INSTALL_DIR="/opt/lingtu/gnss"
DRIVER_REPO="https://github.com/ironoa/um982_ros2_driver.git"
SERVICE_FILE="$(dirname "$0")/gnss.service"
SYSTEMD_DIR="/etc/systemd/system"

log() { echo -e "\033[1;34m[gnss-install]\033[0m $*"; }
err() { echo -e "\033[1;31m[gnss-install]\033[0m $*" >&2; }

# --- 1. Root check ----------------------------------------------------

if [[ $EUID -ne 0 ]]; then
    err "This script must be run with sudo"
    exit 1
fi

# --- 2. Verify ROS2 Humble -------------------------------------------

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    err "ROS2 Humble not found at /opt/ros/humble"
    err "Install: https://docs.ros.org/en/humble/Installation.html"
    exit 1
fi

# --- 3. Check device --------------------------------------------------

if [[ ! -e /dev/ttyUSB0 ]] && [[ ! -e /dev/ttyACM0 ]]; then
    err "Warning: no /dev/ttyUSB0 or /dev/ttyACM0 detected"
    err "Plug in WTRTK-980 first, or continue to install driver anyway"
    read -p "Continue? (y/N) " -n 1 -r
    echo
    [[ ! $REPLY =~ ^[Yy]$ ]] && exit 1
fi

# --- 4. Clone + build driver -----------------------------------------

log "Installing ironoa/um982_ros2_driver to $INSTALL_DIR"
mkdir -p "$INSTALL_DIR/src"

if [[ -d "$INSTALL_DIR/src/um982_ros2_driver" ]]; then
    log "Driver already cloned, pulling latest"
    cd "$INSTALL_DIR/src/um982_ros2_driver"
    git pull --ff-only || log "git pull failed (maybe offline)"
else
    log "Cloning $DRIVER_REPO"
    git clone "$DRIVER_REPO" "$INSTALL_DIR/src/um982_ros2_driver"
fi

log "colcon build"
cd "$INSTALL_DIR"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- 5. Install systemd service --------------------------------------

if [[ ! -f "$SERVICE_FILE" ]]; then
    err "Service file not found: $SERVICE_FILE"
    exit 1
fi

log "Installing gnss.service to $SYSTEMD_DIR"
cp "$SERVICE_FILE" "$SYSTEMD_DIR/gnss.service"

systemctl daemon-reload
systemctl enable gnss.service
log "gnss.service enabled (will start on boot)"

# --- 6. udev rule (optional) -----------------------------------------

UDEV_RULE="/etc/udev/rules.d/99-wtrtk980.rules"
if [[ ! -f "$UDEV_RULE" ]]; then
    log "Creating udev rule for stable device name"
    cat > "$UDEV_RULE" <<'EOF'
# WTRTK-980 / UM980 — create /dev/wtrtk symlink
# Vendor/product IDs for CP2102 / CH340 are common — adjust as needed
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="wtrtk", GROUP="dialout", MODE="0664"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="wtrtk", GROUP="dialout", MODE="0664"
EOF
    udevadm control --reload-rules
    udevadm trigger
    log "udev rule installed: /dev/wtrtk → ttyUSB0 (when plugged in)"
fi

# --- 7. Start service -------------------------------------------------

log "Starting gnss.service"
systemctl restart gnss.service
sleep 2

if systemctl is-active --quiet gnss.service; then
    log "gnss.service is active"
else
    err "gnss.service failed to start"
    err "Check logs: journalctl -u gnss -n 50"
    exit 1
fi

# --- 8. Verify topic --------------------------------------------------

log "Verifying /gps/fix topic (5s timeout)"
if timeout 5 ros2 topic list 2>/dev/null | grep -q "/gps/fix"; then
    log "/gps/fix topic is publishing"
    log "Sample:"
    timeout 3 ros2 topic echo /gps/fix --once 2>/dev/null | head -15 || true
else
    err "Warning: /gps/fix not seen yet (may need a few more seconds)"
    err "Check: ros2 topic list | grep gps"
fi

log "Done. WTRTK-980 GNSS driver installed."
log ""
log "Next steps:"
log "  1. Enable in config: set gnss.enabled=true in config/robot_config.yaml"
log "  2. Pin map origin: let first run auto-init, then copy to config"
log "  3. Start LingTu: sudo systemctl restart lingtu"
log "  4. Verify: curl http://localhost:5050/api/v1/health | jq .sensors.gnss"
