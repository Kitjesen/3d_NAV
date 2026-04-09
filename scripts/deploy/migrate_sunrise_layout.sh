#!/usr/bin/env bash
# migrate_sunrise_layout.sh — 统一 sunrise 代码目录到 ~/data/inovxio/
#
# 在 sunrise 上执行一次:
#   bash ~/data/SLAM/navigation/scripts/deploy/migrate_sunrise_layout.sh
#
# 执行前请先停掉所有服务:
#   systemctl --user stop lingtu lingtu-manager
#   sudo systemctl stop brainstem

set -euo pipefail

NEW_ROOT="$HOME/data/inovxio"
OLD_LINGTU="$HOME/data/SLAM/navigation"
OLD_BRAINSTEM="$HOME/data/brainstem"
OLD_MAPS="$HOME/data/nova/maps"
OLD_LOGS="$HOME/data/nova/logs"

echo "[migrate] 目标目录: $NEW_ROOT"

# 1. 创建目录结构
mkdir -p "$NEW_ROOT/data/maps" "$NEW_ROOT/data/logs"

# 2. 搬 lingtu
if [ -d "$OLD_LINGTU" ] && [ ! -L "$OLD_LINGTU" ]; then
  echo "[migrate] 搬 lingtu: $OLD_LINGTU → $NEW_ROOT/lingtu"
  mv "$OLD_LINGTU" "$NEW_ROOT/lingtu"
  ln -sfn "$NEW_ROOT/lingtu" "$OLD_LINGTU"
  echo "[migrate] 保留旧路径 symlink: $OLD_LINGTU → $NEW_ROOT/lingtu"
else
  echo "[migrate] lingtu 已搬或是 symlink，跳过"
fi

# 3. 搬 brainstem
if [ -d "$OLD_BRAINSTEM" ] && [ ! -L "$OLD_BRAINSTEM" ]; then
  echo "[migrate] 搬 brainstem: $OLD_BRAINSTEM → $NEW_ROOT/brainstem"
  mv "$OLD_BRAINSTEM" "$NEW_ROOT/brainstem"
  ln -sfn "$NEW_ROOT/brainstem" "$OLD_BRAINSTEM"
  echo "[migrate] 保留旧路径 symlink: $OLD_BRAINSTEM → $NEW_ROOT/brainstem"
else
  echo "[migrate] brainstem 已搬或是 symlink，跳过"
fi

# 4. 搬地图数据
if [ -d "$OLD_MAPS" ] && [ ! -L "$OLD_MAPS" ]; then
  echo "[migrate] 搬 maps: $OLD_MAPS → $NEW_ROOT/data/maps"
  cp -rn "$OLD_MAPS"/* "$NEW_ROOT/data/maps/" 2>/dev/null || true
  mv "$OLD_MAPS" "${OLD_MAPS}.bak"
  ln -sfn "$NEW_ROOT/data/maps" "$OLD_MAPS"
  echo "[migrate] 旧 maps 备份为 ${OLD_MAPS}.bak"
fi

# 5. 搬日志
if [ -d "$OLD_LOGS" ] && [ ! -L "$OLD_LOGS" ]; then
  echo "[migrate] 搬 logs: $OLD_LOGS → $NEW_ROOT/data/logs"
  cp -rn "$OLD_LOGS"/* "$NEW_ROOT/data/logs/" 2>/dev/null || true
  mv "$OLD_LOGS" "${OLD_LOGS}.bak"
  ln -sfn "$NEW_ROOT/data/logs" "$OLD_LOGS"
fi

# 6. 更新 systemd service 文件 (从 repo 里 copy)
LINGTU_REPO="$NEW_ROOT/lingtu"
echo ""
echo "[migrate] 复制新 service 文件到 systemd..."
cp "$LINGTU_REPO/scripts/deploy/lingtu.service" "$HOME/.config/systemd/user/" 2>/dev/null || true
cp "$LINGTU_REPO/scripts/deploy/lingtu-manager.service" "$HOME/.config/systemd/user/" 2>/dev/null || true
sudo cp "$LINGTU_REPO/scripts/deploy/brainstem.service" /etc/systemd/system/ 2>/dev/null || true

echo "[migrate] systemctl daemon-reload..."
systemctl --user daemon-reload 2>/dev/null || true
sudo systemctl daemon-reload 2>/dev/null || true

echo ""
echo "=========================================="
echo "[migrate] 完成！新布局:"
echo ""
echo "  $NEW_ROOT/"
echo "  ├── brainstem/     (git repo)"
echo "  ├── lingtu/        (git repo)"
echo "  └── data/"
echo "      ├── maps/"
echo "      └── logs/"
echo ""
echo "旧路径保留为 symlink，向后兼容。"
echo ""
echo "下一步："
echo "  1. systemctl --user start lingtu lingtu-manager"
echo "  2. sudo systemctl start brainstem"
echo "  3. 验证: curl http://localhost:5050/api/v1/health"
echo "=========================================="
