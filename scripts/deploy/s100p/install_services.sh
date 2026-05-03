#!/bin/bash
# Deploy S100P SLAM systemd services
# Usage: scp scripts/deploy/s100p/*.service sunrise@192.168.66.190:/tmp/
#        ssh sunrise@192.168.66.190 'bash /tmp/install_services.sh'
# Or:    ssh sunrise@192.168.66.190 'bash -s' < scripts/deploy/s100p/install_services.sh

set -e

SERVICES=(lidar slam slam_pgo localizer super_lio)
SRC_DIR="${1:-/tmp}"

echo "=== Installing S100P SLAM services ==="
for svc in "${SERVICES[@]}"; do
    if [ -f "${SRC_DIR}/${svc}.service" ]; then
        sudo cp "${SRC_DIR}/${svc}.service" /etc/systemd/system/
        echo "  Installed: ${svc}.service"
        if [ "$svc" = "super_lio" ]; then
            sudo ln -sf /etc/systemd/system/super_lio.service /etc/systemd/system/robot-super-lio.service
            echo "  Alias: robot-super-lio.service -> super_lio.service"
        fi
    else
        echo "  MISSING: ${SRC_DIR}/${svc}.service"
    fi
done

sudo systemctl daemon-reload

for svc in "${SERVICES[@]}"; do
    sudo systemctl enable "${svc}.service" 2>/dev/null
    echo "  Enabled: ${svc}.service"
done

echo ""
echo "=== Done. Quick start: ==="
echo "  sudo systemctl start lidar slam slam_pgo    # mapping mode"
echo "  sudo systemctl start lidar slam localizer   # navigation mode"
echo "  sudo systemctl start lidar super_lio        # experimental Super-LIO mode"
echo "  sudo systemctl start robot-super-lio        # production alias"
echo "  sudo systemctl status slam                  # check status"
echo "  journalctl -u slam -f                       # follow logs"
