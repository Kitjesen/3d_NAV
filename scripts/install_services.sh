#!/bin/bash
# ══════════════════════════════════════════════════════════
# 导航系统 systemd 服务安装脚本
# 用法: sudo bash scripts/install_services.sh
# ══════════════════════════════════════════════════════════
set -e

NAV_DIR="/home/sunrise/data/SLAM/navigation"
SYSTEMD_DIR="/etc/systemd/system"
SUDOERS_FILE="/etc/sudoers.d/nav-services"

echo "═══════════════════════════════════════════"
echo "  导航系统服务安装"
echo "═══════════════════════════════════════════"

# ── 1. 设置脚本可执行权限 ──
echo "[1/5] Setting script permissions..."
chmod +x "$NAV_DIR"/scripts/services/*.sh

# ── 2. 安装 systemd 服务文件 ──
echo "[2/5] Installing systemd service units..."
SERVICES=(
    nav-lidar
    nav-slam
    nav-autonomy
    nav-planning
    nav-grpc
    ota-daemon
)

for svc in "${SERVICES[@]}"; do
    cp "$NAV_DIR/systemd/${svc}.service" "$SYSTEMD_DIR/${svc}.service"
    echo "  ✓ ${svc}.service"
done

# ── 3. 创建 sudoers 规则 (允许 sunrise 用户管理 nav-* 服务) ──
echo "[3/5] Configuring sudoers for service management..."

cat > "$SUDOERS_FILE" << 'EOF'
# 允许 sunrise 用户无密码管理导航系统服务
# 由 install_services.sh 自动生成
sunrise ALL=(root) NOPASSWD: /bin/systemctl start nav-lidar.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl stop nav-lidar.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl restart nav-lidar.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl start nav-slam.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl stop nav-slam.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl restart nav-slam.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl start nav-autonomy.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl stop nav-autonomy.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl restart nav-autonomy.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl start nav-planning.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl stop nav-planning.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl restart nav-planning.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl start nav-grpc.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl stop nav-grpc.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl restart nav-grpc.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl start ota-daemon.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl stop ota-daemon.service
sunrise ALL=(root) NOPASSWD: /bin/systemctl restart ota-daemon.service
EOF

chmod 440 "$SUDOERS_FILE"
visudo -cf "$SUDOERS_FILE" && echo "  ✓ sudoers validated" || {
    echo "  ✗ sudoers validation failed, removing..."
    rm -f "$SUDOERS_FILE"
    exit 1
}

# ── 4. 创建规划环境文件目录 ──
echo "[4/5] Creating environment config directory..."
mkdir -p /etc/nav
if [ ! -f /etc/nav/planning.env ]; then
    cat > /etc/nav/planning.env << EOF2
# nav-planning.service 运行时参数
# NAV_MAP_PATH=/home/sunrise/data/SLAM/navigation/src/global_planning/PCT_planner/rsc/tomogram/spiral0.3_2
# NAV_INIT_X=0.0
# NAV_INIT_Y=0.0
# NAV_INIT_Z=0.0
# NAV_INIT_YAW=0.0
EOF2
    echo "  ✓ /etc/nav/planning.env (template)"
fi

# ── 5. 重新加载 systemd ──
echo "[5/5] Reloading systemd daemon..."
systemctl daemon-reload

echo ""
echo "═══════════════════════════════════════════"
echo "  安装完成!"
echo "═══════════════════════════════════════════"
echo ""
echo "可用服务:"
for svc in "${SERVICES[@]}"; do
    status=$(systemctl is-active "${svc}.service" 2>/dev/null || echo "inactive")
    printf "  %-20s %s\n" "${svc}.service" "$status"
done
echo ""
echo "使用示例:"
echo "  sudo systemctl start nav-lidar    # 启动 LiDAR"
echo "  sudo systemctl start nav-slam     # 启动 SLAM"
echo "  systemctl status nav-slam         # 查看状态"
echo "  journalctl -u nav-slam -f         # 查看日志"
echo ""
echo "开机自启 (可选):"
echo "  sudo systemctl enable nav-grpc ota-daemon"
echo ""
