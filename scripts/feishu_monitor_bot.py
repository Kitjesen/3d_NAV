#!/usr/bin/env python3
"""
3D-NAV 飞书监控机器人 - thunder
用于远程监控机器人导航状态
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
    """ROS2 节点 + 飞书机器人"""

    def __init__(self, app_id: str, app_secret: str, receive_id: str):
        super().__init__('feishu_monitor_bot')

        # 飞书配置
        self.app_id = app_id
        self.app_secret = app_secret
        self.receive_id = receive_id  # 接收消息的用户 open_id 或群 chat_id

        # 创建飞书客户端
        self.client = lark.Client.builder() \
            .app_id(self.app_id) \
            .app_secret(self.app_secret) \
            .build()

        # ROS2 订阅
        self.sub_status = self.create_subscription(
            String,
            '/nav/semantic/status',
            self.status_callback,
            10
        )

        self.get_logger().info('飞书监控机器人 thunder 已初始化')

    def status_callback(self, msg: String):
        """接收导航状态并发送到飞书"""
        try:
            status = json.loads(msg.data)
            message = self._format_status(status)

            # 发送到飞书
            self.send_message(message)

        except Exception as e:
            self.get_logger().error(f'状态回调错误: {e}')

    def _format_status(self, status: dict) -> str:
        """格式化状态消息"""
        return f"""🤖 3D-NAV 状态更新 (thunder)

📍 当前状态: {status.get('state', 'UNKNOWN')}
🎯 目标: {status.get('target', 'None')}
📏 距离: {status.get('distance', 0):.2f}m
⏱️ 时间: {status.get('elapsed_time', 0):.1f}s
✅ 成功率: {status.get('success_rate', 0):.1%}
"""

    def send_message(self, text: str):
        """发送文本消息到飞书"""
        try:
            # 构建消息请求
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

            # 发送消息
            response = self.client.im.v1.message.create(request)

            # 检查响应
            if not response.success():
                self.get_logger().error(
                    f"发送消息失败: code={response.code}, msg={response.msg}"
                )
            else:
                self.get_logger().debug("消息发送成功")

        except Exception as e:
            self.get_logger().error(f"发送消息异常: {e}")

    def send_card_message(self, title: str, content: str):
        """发送卡片消息到飞书（更美观）"""
        try:
            card = {
                "config": {
                    "wide_screen_mode": True
                },
                "header": {
                    "title": {
                        "tag": "plain_text",
                        "content": title
                    },
                    "template": "blue"
                },
                "elements": [
                    {
                        "tag": "div",
                        "text": {
                            "tag": "lark_md",
                            "content": content
                        }
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
                    f"发送卡片失败: code={response.code}, msg={response.msg}"
                )

        except Exception as e:
            self.get_logger().error(f"发送卡片异常: {e}")


def main():
    """主函数"""
    # 配置（需要替换为实际值）
    APP_ID = "cli_a9f23f624b649ceb"  # 从飞书开放平台获取
    APP_SECRET = "F4aHJepltjOioMCyDW0zWfvDwKrpdHeQ"  # 从飞书开放平台获取
    RECEIVE_ID = "ou_7d8a6e6df7621556ce0d21922b676706ccs"  # 接收消息的用户 open_id

    # 初始化 ROS2
    rclpy.init()

    # 创建节点
    bot = FeishuMonitorBot(APP_ID, APP_SECRET, RECEIVE_ID)

    # 发送启动消息
    bot.send_card_message(
        "🤖 thunder 机器人已启动",
        "**3D-NAV 监控系统**\n\n✅ 已连接到 ROS2\n✅ 正在监听导航状态"
    )

    # ROS2 spin
    try:
        rclpy.spin(bot)
    except KeyboardInterrupt:
        bot.send_message("⚠️ thunder 机器人已停止")
    finally:
        bot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
