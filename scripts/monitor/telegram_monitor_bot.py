#!/usr/bin/env python3
"""
3D-NAV Telegram monitor bot.
Remote monitoring of robot navigation status via Telegram messages.
"""

import asyncio
import json
from telegram import Update
from telegram.ext import Application, CommandHandler, ContextTypes
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TelegramMonitorBot(Node):
    """ROS2 node + Telegram Bot integration."""

    def __init__(self, bot_token: str, chat_id: str):
        super().__init__('telegram_monitor_bot')

        self.bot_token = bot_token
        self.chat_id = chat_id
        self.app = None

        # ROS2 subscription
        self.sub_status = self.create_subscription(
            String,
            '/nav/semantic/status',
            self.status_callback,
            10
        )

        self.get_logger().info('Telegram monitor bot initialized')

    def status_callback(self, msg: String):
        """Receive navigation status and forward to Telegram."""
        try:
            status = json.loads(msg.data)
            message = self._format_status(status)
            asyncio.create_task(self.send_message(message))
        except Exception as e:
            self.get_logger().error(f'Status callback error: {e}')

    def _format_status(self, status: dict) -> str:
        """Format a status dict into a human-readable message."""
        return (
            f"3D-NAV Status Update\n\n"
            f"State:    {status.get('state', 'UNKNOWN')}\n"
            f"Target:   {status.get('target', 'None')}\n"
            f"Distance: {status.get('distance', 0):.2f} m\n"
            f"Elapsed:  {status.get('elapsed_time', 0):.1f} s\n"
            f"SR:       {status.get('success_rate', 0):.1%}\n"
        )

    async def send_message(self, text: str):
        """Send a text message to Telegram."""
        if self.app:
            await self.app.bot.send_message(
                chat_id=self.chat_id,
                text=text
            )

    async def cmd_status(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /status command."""
        await update.message.reply_text("Querying robot status...")
        # TODO: query current status from ROS2

    async def cmd_start(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
        """Handle /start command."""
        await update.message.reply_text(
            "3D-NAV Monitor Bot started\n\n"
            "Available commands:\n"
            "/status - query current status\n"
            "/stop   - stop navigation\n"
            "/help   - show help"
        )

    async def setup_telegram(self):
        """Initialize and start the Telegram bot."""
        self.app = Application.builder().token(self.bot_token).build()

        self.app.add_handler(CommandHandler("start", self.cmd_start))
        self.app.add_handler(CommandHandler("status", self.cmd_status))

        await self.app.initialize()
        await self.app.start()
        await self.app.updater.start_polling()

        self.get_logger().info('Telegram bot started')


def main():
    # Credentials — replace with actual values
    BOT_TOKEN = "YOUR_BOT_TOKEN_HERE"   # from @BotFather
    CHAT_ID = "YOUR_CHAT_ID_HERE"       # your Telegram chat ID

    rclpy.init()
    bot = TelegramMonitorBot(BOT_TOKEN, CHAT_ID)

    asyncio.run(bot.setup_telegram())

    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        pass
    finally:
        bot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
