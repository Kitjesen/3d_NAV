#!/usr/bin/env bash
# scripts/install_go2rtc.sh — install go2rtc sidecar on S100P (aarch64).
#
# Why a sidecar?
#   Python aiortc has ~10-20 ms fixed overhead in its Python SRTP/RTP
#   layers.  go2rtc is a single ~8 MB Go binary that handles the media
#   hotpath natively, typically 30-60 ms glass-to-glass on LAN vs our
#   current 80-150 ms.  See src/webrtc/README.md § 图传模式 for the full
#   architecture rationale.
#
# Usage:
#   sudo bash scripts/install_go2rtc.sh           # systemd mode
#   bash scripts/install_go2rtc.sh --no-systemd   # just install the binary
#
set -euo pipefail

GO2RTC_VERSION="${GO2RTC_VERSION:-v1.9.10}"
BIN_URL="https://github.com/AlexxIT/go2rtc/releases/download/${GO2RTC_VERSION}/go2rtc_linux_arm64"
BIN_PATH="${BIN_PATH:-/usr/local/bin/go2rtc}"
CONFIG_DIR="${CONFIG_DIR:-/etc/go2rtc}"
CONFIG_PATH="${CONFIG_DIR}/go2rtc.yaml"
SYSTEMD_UNIT="/etc/systemd/system/go2rtc.service"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

INSTALL_SYSTEMD=1
for arg in "$@"; do
    case "$arg" in
        --no-systemd) INSTALL_SYSTEMD=0 ;;
        --help|-h)
            sed -n '2,/^set -e/p' "$0" | sed 's/^# \?//'
            exit 0
            ;;
    esac
done

arch="$(uname -m)"
if [[ "$arch" != "aarch64" && "$arch" != "arm64" ]]; then
    echo "WARNING: this installer targets aarch64 (S100P); detected $arch" >&2
fi

echo "==> Downloading go2rtc ${GO2RTC_VERSION} → ${BIN_PATH}"
sudo mkdir -p "$(dirname "$BIN_PATH")"
sudo curl -fsSL "$BIN_URL" -o "$BIN_PATH"
sudo chmod +x "$BIN_PATH"
"$BIN_PATH" --version || { echo "FAIL: go2rtc binary not executable"; exit 1; }

echo "==> Installing config → ${CONFIG_PATH}"
sudo mkdir -p "$CONFIG_DIR"
if [[ -f "$CONFIG_PATH" ]]; then
    echo "    (existing config kept; template at ${REPO_ROOT}/config/go2rtc.yaml)"
else
    sudo cp "${REPO_ROOT}/config/go2rtc.yaml" "$CONFIG_PATH"
fi

if [[ "$INSTALL_SYSTEMD" == "1" ]]; then
    echo "==> Writing systemd unit → ${SYSTEMD_UNIT}"
    sudo tee "$SYSTEMD_UNIT" > /dev/null <<EOF
[Unit]
Description=go2rtc — low-latency camera gateway (LingTu sidecar)
After=network.target

[Service]
Type=simple
ExecStart=${BIN_PATH} -config ${CONFIG_PATH}
Restart=always
RestartSec=2
# Give it realtime-ish priority so ffmpeg ingestion doesn't starve.
Nice=-5
# v4l2 device + dma-heap (HW codec) access
SupplementaryGroups=video render

[Install]
WantedBy=multi-user.target
EOF
    sudo systemctl daemon-reload
    sudo systemctl enable --now go2rtc
    sleep 1
    echo "==> Service status"
    sudo systemctl --no-pager status go2rtc | head -12 || true
    echo
    echo "✅ go2rtc running.  WHEP endpoint:  http://<robot-ip>:1984/api/webrtc?src=cam"
    echo "   Web UI (debug):                   http://<robot-ip>:1984/"
    echo "   Logs:                             journalctl -u go2rtc -f"
else
    echo
    echo "✅ Installed.  Run manually: ${BIN_PATH} -config ${CONFIG_PATH}"
fi

cat <<'POST'

Next steps:
  1. Edit ${CONFIG_PATH} and point it at your camera device
     (default: /dev/video0 — Orbbec Gemini RGB UVC node).
     Run `v4l2-ctl --list-devices` to discover the right path.
  2. Restart the service:  sudo systemctl restart go2rtc
  3. Open the LingTu dashboard; CameraFeed will auto-detect go2rtc
     and switch from aiortc → WHEP.  HUD should show "Go2RTC" label.
  4. Benchmark: see src/webrtc/README.md § 3 for the秒表/webrtc-internals
     measurement protocol.
POST
