#!/bin/bash
# 飞书机器人 thunder 安装脚本

echo "=========================================="
echo "飞书机器人 thunder 安装脚本"
echo "=========================================="
echo ""

# 检查 Python3
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 未找到 python3"
    echo "请先安装 Python 3.8+"
    exit 1
fi

PYTHON_VERSION=$(python3 --version | cut -d' ' -f2)
echo "✅ Python 版本: $PYTHON_VERSION"
echo ""

# 检查 pip3
if ! command -v pip3 &> /dev/null; then
    echo "❌ 错误: 未找到 pip3"
    echo "请先安装 pip3"
    exit 1
fi

echo "✅ pip3 已安装"
echo ""

# 安装 lark-oapi
echo "📦 安装飞书 SDK (lark-oapi)..."
pip3 install lark-oapi

if [ $? -eq 0 ]; then
    echo "✅ lark-oapi 安装成功"
else
    echo "❌ lark-oapi 安装失败"
    exit 1
fi

echo ""

# 检查 ROS2
echo "🔍 检查 ROS2 环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  警告: ROS2 环境未激活"
    echo "请运行: source /opt/ros/humble/setup.bash"
else
    echo "✅ ROS2 环境: $ROS_DISTRO"
fi

echo ""
echo "=========================================="
echo "✅ 安装完成！"
echo "=========================================="
echo ""
echo "📋 下一步操作:"
echo ""
echo "1. 创建飞书应用"
echo "   访问: https://open.feishu.cn/"
echo "   创建企业自建应用，名称: thunder"
echo ""
echo "2. 获取凭证"
echo "   - App ID (以 cli_ 开头)"
echo "   - App Secret"
echo "   - Receive ID (用户 open_id 或群 chat_id)"
echo ""
echo "3. 测试配置"
echo "   cd $(dirname "$0")"
echo "   python3 test_feishu.py"
echo ""
echo "4. 编辑机器人配置"
echo "   编辑 feishu_monitor_bot.py 第 149-151 行"
echo "   填入你的 App ID, App Secret, Receive ID"
echo ""
echo "5. 运行机器人"
echo "   source /opt/ros/humble/setup.bash"
echo "   source ~/3d_NAV/install/setup.bash"
echo "   python3 feishu_monitor_bot.py"
echo ""
echo "📚 详细配置指南:"
echo "   ../docs/guides/FEISHU_BOT_SETUP.md"
echo ""
echo "=========================================="
