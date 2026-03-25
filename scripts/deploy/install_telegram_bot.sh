#!/bin/bash
# Telegram 监控机器人一键安装脚本

echo "🤖 3D-NAV Telegram 监控机器人安装脚本"
echo "=========================================="

# 检查 Python
if ! command -v python3 &> /dev/null; then
    echo "❌ 错误: 未找到 Python3"
    exit 1
fi

echo "✅ Python3 已安装: $(python3 --version)"

# 检查 pip
if ! command -v pip3 &> /dev/null; then
    echo "❌ 错误: 未找到 pip3"
    exit 1
fi

echo "✅ pip3 已安装"

# 安装依赖
echo ""
echo "📦 安装 Python 依赖..."
pip3 install python-telegram-bot aiohttp

if [ $? -eq 0 ]; then
    echo "✅ 依赖安装成功"
else
    echo "❌ 依赖安装失败"
    exit 1
fi

# 创建配置文件
echo ""
echo "📝 创建配置文件..."

CONFIG_FILE="$HOME/.3dnav_telegram_config.json"

if [ ! -f "$CONFIG_FILE" ]; then
    cat > "$CONFIG_FILE" << EOF
{
  "bot_token": "YOUR_BOT_TOKEN_HERE",
  "chat_id": "YOUR_CHAT_ID_HERE",
  "status_topic": "/nav/semantic/status",
  "update_interval": 5.0
}
EOF
    echo "✅ 配置文件已创建: $CONFIG_FILE"
    echo ""
    echo "⚠️  请编辑配置文件，填入你的 Bot Token 和 Chat ID"
    echo "   编辑命令: nano $CONFIG_FILE"
else
    echo "ℹ️  配置文件已存在: $CONFIG_FILE"
fi

# 创建启动脚本
echo ""
echo "🚀 创建启动脚本..."

LAUNCH_SCRIPT="$HOME/start_telegram_bot.sh"

cat > "$LAUNCH_SCRIPT" << 'EOF'
#!/bin/bash
# 启动 Telegram 监控机器人

# 激活 ROS2 环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# 激活工作空间
if [ -f "$HOME/3d_NAV/install/setup.bash" ]; then
    source $HOME/3d_NAV/install/setup.bash
fi

# 运行机器人
cd $(dirname $0)
python3 telegram_monitor_bot.py
EOF

chmod +x "$LAUNCH_SCRIPT"
echo "✅ 启动脚本已创建: $LAUNCH_SCRIPT"

# 完成
echo ""
echo "=========================================="
echo "✅ 安装完成！"
echo ""
echo "📋 下一步操作："
echo "1. 在 Telegram 中创建机器人 (搜索 @BotFather)"
echo "2. 编辑配置文件: nano $CONFIG_FILE"
echo "3. 启动机器人: $LAUNCH_SCRIPT"
echo ""
echo "📚 详细文档: docs/guides/TELEGRAM_BOT_SETUP.md"
echo "=========================================="
