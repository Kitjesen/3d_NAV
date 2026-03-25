#!/bin/bash
# ============================================================================
# backup_config.sh — 备份配置文件、服务文件和启动脚本
#
# 备份内容:
#   - config/*.yaml
#   - systemd/*.service
#   - launch/**/*.py
#
# 备份位置: /opt/lingtu/nav/backups/config-<timestamp>.tar.gz
# 保留策略: 最近 5 个备份
#
# 用法:
#   bash scripts/backup_config.sh
# ============================================================================
set -euo pipefail

# ---- 颜色 ----
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# ---- 路径 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
BACKUP_DIR="/opt/lingtu/nav/backups"
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
BACKUP_FILE="${BACKUP_DIR}/config-${TIMESTAMP}.tar.gz"
MAX_BACKUPS=5

echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  配置备份${NC}"
echo -e "${BLUE}============================================${NC}"

# ---- 创建备份目录 ----
if [ ! -d "$BACKUP_DIR" ]; then
    echo -e "${YELLOW}创建备份目录: ${BACKUP_DIR}${NC}"
    sudo mkdir -p "$BACKUP_DIR"
    sudo chown "$(whoami)" "$BACKUP_DIR"
fi

# ---- 执行备份 ----
echo -e "${GREEN}[1/3] 打包配置文件 ...${NC}"
cd "$WORKSPACE_DIR"

tar czf "$BACKUP_FILE" \
    --ignore-failed-read \
    config/*.yaml \
    systemd/*.service \
    $(find launch -name '*.py' 2>/dev/null) \
    2>/dev/null

echo -e "  ${GREEN}完成${NC}"

# ---- 显示备份信息 ----
echo -e "${GREEN}[2/3] 备份信息${NC}"
BACKUP_SIZE=$(du -sh "$BACKUP_FILE" | awk '{print $1}')
echo -e "  文件: ${BACKUP_FILE}"
echo -e "  大小: ${BACKUP_SIZE}"

# ---- 清理旧备份 ----
echo -e "${GREEN}[3/3] 清理旧备份 (保留最近 ${MAX_BACKUPS} 个) ...${NC}"
BACKUP_COUNT=$(ls -1 "${BACKUP_DIR}"/config-*.tar.gz 2>/dev/null | wc -l)
if [ "$BACKUP_COUNT" -gt "$MAX_BACKUPS" ]; then
    REMOVE_COUNT=$((BACKUP_COUNT - MAX_BACKUPS))
    ls -1t "${BACKUP_DIR}"/config-*.tar.gz | tail -n "$REMOVE_COUNT" | while read -r old_backup; do
        echo -e "  ${YELLOW}删除: $(basename "$old_backup")${NC}"
        rm -f "$old_backup"
    done
else
    echo -e "  当前 ${BACKUP_COUNT} 个备份，无需清理"
fi

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${GREEN}  备份完成!${NC}"
echo -e "${BLUE}============================================${NC}"
echo -e "  ${BACKUP_FILE} (${BACKUP_SIZE})"
