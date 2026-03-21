#!/usr/bin/env bash
# ──────────────────────────────────────────
# LingTu 一键部署脚本
# 用法: bash scripts/deploy.sh [--agent]
#
# 在全新 S100P 上从零部署 LingTu 导航系统
# ──────────────────────────────────────────
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
INSTALL_AGENT=false
[ "$1" = "--agent" ] && INSTALL_AGENT=true

echo "=== LingTu 部署 ==="

# Step 1: 检查平台
echo "[1/7] 检查平台..."
ARCH=$(uname -m)
echo "  架构: $ARCH"
echo "  OS: $(lsb_release -ds 2>/dev/null || head -1 /etc/os-release)"
# 检查 ROS2
if ! command -v ros2 &>/dev/null; then
    echo "  ERROR: ROS2 未安装。请先安装 ROS2 Humble。"
    exit 1
fi
echo "  ROS2: $(ros2 --version 2>/dev/null || echo 'unknown')"

# Step 2: 系统依赖
echo "[2/7] 安装系统依赖..."
sudo apt-get update -qq
sudo apt-get install -y -qq python3-pip python3-colcon-common-extensions \
    python3-rosdep libgoogle-glog-dev libunwind-dev 2>/dev/null || true

# Step 3: Python 依赖
echo "[3/7] 安装 Python 依赖..."
pip3 install -r "$PROJECT_DIR/requirements.txt" --quiet
if [ "$INSTALL_AGENT" = true ]; then
    echo "  安装 Agent 依赖 (LangChain)..."
    pip3 install -r "$PROJECT_DIR/requirements-agent.txt" --quiet
fi

# Step 4: 编译 ROS2 工作空间
echo "[4/7] 编译 ROS2..."
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
colcon build 2>&1 | tail -5
source install/setup.bash

# Step 5: 创建数据目录
echo "[5/7] 创建数据目录..."
mkdir -p /data/maps
mkdir -p /opt/lingtu/nav

# Step 6: 安装 CLI
echo "[6/7] 安装 CLI..."
chmod +x "$PROJECT_DIR/lingtu"
if [ ! -L /usr/local/bin/lingtu ]; then
    sudo ln -sf "$PROJECT_DIR/lingtu" /usr/local/bin/lingtu
    echo "  已创建 /usr/local/bin/lingtu 符号链接"
fi

# Step 7: 验证
echo "[7/7] 验证安装..."
lingtu version
python3 -c "
from semantic_perception.storage import SqliteStore
from semantic_planner.skill_registry import SkillRegistry, LingTuNavigationSkills
reg = SkillRegistry()
nav = LingTuNavigationSkills()
count = reg.register_instance(nav)
print(f'  Skills: {count}')
print('  验证通过!')
"

echo ""
echo "=== 部署完成! ==="
echo ""
echo "快速开始:"
echo "  lingtu map                    # 建图"
echo "  lingtu nav --map <name>       # 导航"
echo "  lingtu explore '找到餐桌'      # 语义探索"
if [ "$INSTALL_AGENT" = true ]; then
echo "  lingtu agent --llm kimi       # Agent 模式"
echo "  lingtu chat                   # 交互对话"
fi
echo ""
echo "查看帮助: lingtu --help"
