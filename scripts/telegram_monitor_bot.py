#!/usr/bin/env python3
"""
3D-NAV Telegram 监控机器人
用于远程监控机器人导航状态
"""

import asyncio
import json
from telegram import Update
from telegram.ext import Application, CommandHandler, ContextTypes
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TelegramMonitorBot(Node):
    """ROS2 节点 + Telegram Bot"""

    def __init__(self, bot_token: str, chat_id: str):
        super().__init__('telegram_monitor_bot')

        # Telegram 配置
        self.bot_token = bot_token
        self.chat_id = chat_id
        self.app = None

        # ROS2 订阅
        self.sub_status = self.create_subscription(
            String,
            '/nav/semantic/status',
            self.status_callback,
            10
        )

        self.get_logger().info('Telegram Monitor Bot initialized')

    def status_callback(self, msg: String):
        """接收导航状态并发送到 Telegram"""
        try:
            status = json.loads(msg.data)
            message = self._format_status(status)

            # 发送到 Telegram
            asyncio.create_task(self.send_message(message))

        except Exception as e:
            self.get_logger().error(f'Status callback error: {e}')

    def _format_status(self, status: dict) -> str:
        """格式化状态消息"""
        return f"""
🤖 3D-NAV 状态更新

📍 当前状态: {status.get('state', 'UNKNOWN')}
🎯 目标: {status.get('target', 'None')}
📏 距离: {status.get('distance', 0):.2f}m
⏱️ 时间: {status.get('elapsed_time', 0):.1f}s
✅ 成功率: {status.get('success_rate', 0):.1%}
"""

    async def send_message(self, text: str):
        """发送消息到 Telegram"""
        if self.app:
            await self.app.bot.send_message(
                chat_id=self.chat_id,
                text=text
            )

    async def cmd_status(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """处理 /status 命令"""
        await update.message.reply_text("查询机器人状态...")
        # TODO: 查询当前状态

    async def cmd_start(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """处理 /start 命令"""
        await update.message.reply_text(
            "🤖 3D-NAV 监控机器人已启动\n\n"
            "可用命令:\n"
            "/status - 查询当前状态\n"
            "/stop - 停止导航\n"
            "/help - 帮助信息"
        )

    async def setup_telegram(self):
        """设置 Telegram Bot"""
        self.app = Application.builder().token(self.bot_token).build()

        # 注册命令处理器
        self.app.add_handler(CommandHandler("start", self.cmd_start))
        self.app.add_handler(CommandHandler("status", self.cmd_status))

        # 启动 Bot
        await self.app.initialize()
        await self.app.start()
        await self.app.updater.start_polling()

        self.get_logger().info('Telegram Bot started')


def main():
    """主函数"""
    # 配置（需要替换为实际值）
    BOT_TOKEN = "YOUR_BOT_TOKEN_HERE"  # 从 @BotFather 获取
    CHAT_ID = "YOUR_CHAT_ID_HERE"      # 你的 Telegram Chat ID

    # 初始化 ROS2
    rclpy.init()

    # 创建节点
    bot = TelegramMonitorBot(BOT_TOKEN, CHAT_ID)

    # 启动 Telegram Bot
    asyncio.run(bot.setup_telegram())

    # ROS2 spin
    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        pass
    finally:
        bot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
