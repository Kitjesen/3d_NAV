#!/usr/bin/env python3
"""
测试飞书机器人配置
用于验证 App ID, App Secret 和 receive_id 是否正确
"""

import json
import sys

try:
    import lark_oapi as lark
    from lark_oapi.api.im.v1 import *
except ImportError:
    print("❌ 错误: 未安装 lark-oapi")
    print("请运行: pip3 install lark-oapi")
    sys.exit(1)


def test_feishu_config(app_id: str, app_secret: str, receive_id: str):
    """测试飞书配置"""

    print("🔍 开始测试飞书配置...")
    print(f"App ID: {app_id[:10]}...")
    print(f"App Secret: {app_secret[:10]}...")
    print(f"Receive ID: {receive_id[:10]}...")
    print()

    # 创建客户端
    try:
        client = lark.Client.builder() \
            .app_id(app_id) \
            .app_secret(app_secret) \
            .build()
        print("✅ 飞书客户端创建成功")
    except Exception as e:
        print(f"❌ 客户端创建失败: {e}")
        return False

    # 发送测试消息
    try:
        request = CreateMessageRequest.builder() \
            .receive_id_type("open_id") \
            .request_body(
                CreateMessageRequestBody.builder()
                .receive_id(receive_id)
                .msg_type("text")
                .content(json.dumps({"text": "🤖 thunder 测试消息\n\n如果你看到这条消息，说明配置成功！"}))
                .build()
            ) \
            .build()

        response = client.im.v1.message.create(request)

        if response.success():
            print("✅ 测试消息发送成功！")
            print("📱 请检查飞书是否收到消息")
            return True
        else:
            print(f"❌ 消息发送失败:")
            print(f"   错误码: {response.code}")
            print(f"   错误信息: {response.msg}")
            print()
            print("💡 常见问题:")
            print("   - code=99991663: App ID 或 App Secret 错误")
            print("   - code=230002: 权限不足，需要添加 im:message 权限")
            print("   - code=230011: receive_id 无效")
            return False

    except Exception as e:
        print(f"❌ 发送消息异常: {e}")
        return False


def main():
    """主函数"""
    print("=" * 60)
    print("飞书机器人配置测试工具 - thunder")
    print("=" * 60)
    print()

    # 从用户输入获取配置
    print("请输入飞书配置信息:")
    print("(可以从飞书开放平台 https://open.feishu.cn/ 获取)")
    print()

    app_id = input("App ID: ").strip()
    app_secret = input("App Secret: ").strip()
    receive_id = input("Receive ID (open_id): ").strip()

    print()

    # 验证输入
    if not app_id or not app_secret or not receive_id:
        print("❌ 错误: 所有字段都必须填写")
        sys.exit(1)

    if not app_id.startswith("cli_"):
        print("⚠️  警告: App ID 通常以 'cli_' 开头")

    if not receive_id.startswith("ou_") and not receive_id.startswith("oc_"):
        print("⚠️  警告: Receive ID 通常以 'ou_' (用户) 或 'oc_' (群组) 开头")

    print()

    # 测试配置
    success = test_feishu_config(app_id, app_secret, receive_id)

    print()
    print("=" * 60)

    if success:
        print("🎉 配置测试成功！")
        print()
        print("下一步:")
        print("1. 编辑 feishu_monitor_bot.py")
        print("2. 将配置信息填入第 149-151 行")
        print("3. 运行: python3 feishu_monitor_bot.py")
    else:
        print("❌ 配置测试失败")
        print()
        print("请检查:")
        print("1. App ID 和 App Secret 是否正确")
        print("2. 应用权限是否已配置 (im:message)")
        print("3. 应用是否已发布")
        print("4. 机器人是否已添加为好友或在群中")
        print()
        print("详细配置指南: docs/guides/FEISHU_BOT_SETUP.md")

    print("=" * 60)


if __name__ == '__main__':
    main()
