#!/bin/bash
# thunder 机器人启动脚本

echo "🤖 启动 thunder 机器人..."
echo ""

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 检查配置文件
if [ ! -f "$SCRIPT_DIR/feishu_monitor_bot.py" ]; then
    echo "❌ 错误: 找不到 feishu_monitor_bot.py"
    exit 1
fi

# 检查 Python3
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 未找到 python3"
    exit 1
fi

# 检查 lark-oapi
if ! python3 -c "import lark_oapi" 2>/dev/null; then
    echo "❌ 错误: 未安装 lark-oapi"
    echo "请运行: pip3 install lark-oapi"
    exit 1
fi

# 激活 ROS2 环境
echo "🔧 激活 ROS2 环境..."

if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble 已激活"
else
    echo "⚠️  警告: 未找到 ROS2 Humble"
fi

# 激活工作空间
if [ -f "$HOME/3d_NAV/install/setup.bash" ]; then
    source "$HOME/3d_NAV/install/setup.bash"
    echo "✅ 3D-NAV 工作空间已激活"
else
    echo "⚠️  警告: 未找到 3D-NAV 工作空间"
fi

echo ""
echo "=========================================="
echo "🚀 启动 thunder 机器人"
echo "=========================================="
echo ""
echo "📡 监听话题: /nav/semantic/status"
echo "🛑 停止: Ctrl+C"
echo ""

# 运行机器人
cd "$SCRIPT_DIR"
python3 feishu_monitor_bot.py

# 捕获退出状态
EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $EXIT_CODE -eq 0 ]; then
    echo "✅ thunder 机器人已正常退出"
else
    echo "❌ thunder 机器人异常退出 (代码: $EXIT_CODE)"
fi
echo "=========================================="

exit $EXIT_CODE
