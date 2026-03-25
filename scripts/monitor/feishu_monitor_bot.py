#!/usr/bin/env python3
"""
3D-NAV Feishu (Lark) monitor bot — thunder robot.
Remote monitoring of robot navigation status via Feishu messages.
"""

import asyncio
import json
import time
from typing import Optional

import lark_oapi as lark
from lark_oapi.api.im.v1 import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FeishuMonitorBot(Node):
    """ROS2 node + Feishu (Lark) bot integration."""

    def __init__(self, app_id: str, app_secret: str, receive_id: str):
        super().__init__('feishu_monitor_bot')

        # Feishu credentials
        self.app_id = app_id
        self.app_secret = app_secret
        self.receive_id = receive_id  # user open_id or group chat_id

        # Build Feishu client
        self.client = lark.Client.builder() \
            .app_id(self.app_id) \
            .app_secret(self.app_secret) \
            .build()

        # ROS2 subscription
        self.sub_status = self.create_subscription(
            String,
            '/nav/semantic/status',
            self.status_callback,
            10
        )

        self.get_logger().info('Feishu monitor bot initialized')

    def status_callback(self, msg: String):
        """Receive navigation status and forward to Feishu."""
        try:
            status = json.loads(msg.data)
            message = self._format_status(status)
            self.send_message(message)
        except Exception as e:
            self.get_logger().error(f'Status callback error: {e}')

    def _format_status(self, status: dict) -> str:
        """Format a status dict into a human-readable message."""
        return (
            f"3D-NAV Status Update (thunder)\n\n"
            f"State:    {status.get('state', 'UNKNOWN')}\n"
            f"Target:   {status.get('target', 'None')}\n"
            f"Distance: {status.get('distance', 0):.2f} m\n"
            f"Elapsed:  {status.get('elapsed_time', 0):.1f} s\n"
            f"SR:       {status.get('success_rate', 0):.1%}\n"
        )

    def send_message(self, text: str):
        """Send a plain-text message to Feishu."""
        try:
            request = CreateMessageRequest.builder() \
                .receive_id_type("open_id") \
                .request_body(
                    CreateMessageRequestBody.builder()
                    .receive_id(self.receive_id)
                    .msg_type("text")
                    .content(json.dumps({"text": text}))
                    .build()
                ) \
                .build()

            response = self.client.im.v1.message.create(request)

            if not response.success():
                self.get_logger().error(
                    f"Send failed: code={response.code}, msg={response.msg}"
                )
            else:
                self.get_logger().debug("Message sent successfully")

        except Exception as e:
            self.get_logger().error(f"Send exception: {e}")

    def send_card_message(self, title: str, content: str):
        """Send an interactive card message to Feishu (richer formatting)."""
        try:
            card = {
                "config": {"wide_screen_mode": True},
                "header": {
                    "title": {"tag": "plain_text", "content": title},
                    "template": "blue"
                },
                "elements": [
                    {
                        "tag": "div",
                        "text": {"tag": "lark_md", "content": content}
                    }
                ]
            }

            request = CreateMessageRequest.builder() \
                .receive_id_type("open_id") \
                .request_body(
                    CreateMessageRequestBody.builder()
                    .receive_id(self.receive_id)
                    .msg_type("interactive")
                    .content(json.dumps(card))
                    .build()
                ) \
                .build()

            response = self.client.im.v1.message.create(request)

            if not response.success():
                self.get_logger().error(
                    f"Card send failed: code={response.code}, msg={response.msg}"
                )

        except Exception as e:
            self.get_logger().error(f"Card send exception: {e}")


def main():
    # Credentials — replace with actual values from the Feishu Open Platform
    APP_ID = "cli_a9f23f624b649ceb"
    APP_SECRET = "F4aHJepltjOioMCyDW0zWfvDwKrpdHeQ"
    RECEIVE_ID = "ou_7d8a6e6df7621556ce0d21922b676706ccs"

    rclpy.init()
    bot = FeishuMonitorBot(APP_ID, APP_SECRET, RECEIVE_ID)

    bot.send_card_message(
        "thunder robot started",
        "**3D-NAV Monitor**\n\nConnected to ROS2\nListening for navigation status"
    )

    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        bot.send_message("thunder robot stopped")
    finally:
        bot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
