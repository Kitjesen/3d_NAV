# 飞书机器人配置指南 - thunder

## 📱 什么是飞书？

飞书（Lark/Feishu）是字节跳动推出的企业协作平台，类似钉钉、企业微信。

**优势：**
- ✅ 在中国可以直接使用（不需要翻墙）
- ✅ 支持机器人 API
- ✅ 界面现代、功能强大
- ✅ 支持卡片消息（更美观）

---

## 🚀 快速配置步骤

### 步骤 1: 创建飞书应用

1. **访问飞书开放平台**
   - 网址: https://open.feishu.cn/
   - 使用飞书账号登录

2. **创建企业自建应用**
   - 点击"开发者后台"
   - 点击"创建企业自建应用"
   - 应用名称: `thunder`
   - 应用描述: `3D-NAV 机器人监控系统`
   - 上传应用图标（可选）

3. **获取凭证**
   - 进入应用详情页
   - 找到"凭证与基础信息"
   - 复制 **App ID**: `cli_xxxxxxxxxxxxxxxx`
   - 复制 **App Secret**: `xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx`
   - **保存这两个值！**

### 步骤 2: 配置权限

1. **添加应用能力**
   - 在应用管理页面，点击"添加应用能力"
   - 选择"机器人"

2. **配置权限范围**
   - 点击"权限管理"
   - 添加以下权限：
     - `im:message` - 获取与发送单聊、群组消息
     - `im:message:send_as_bot` - 以应用的身份发消息
   - 点击"保存"

3. **发布版本**
   - 点击"版本管理与发布"
   - 创建版本并提交审核
   - 审核通过后发布（企业内部应用通常秒过）

### 步骤 3: 获取接收者 ID

#### 方法 A: 获取用户 open_id（发送给个人）

1. **添加机器人为好友**
   - 在飞书中搜索你的应用名称 `thunder`
   - 添加为联系人

2. **获取 open_id**
   - 方式 1: 在开放平台"开发调试" → "在线调试" → "获取用户信息"
   - 方式 2: 让机器人发送一条消息，在回调中获取

#### 方法 B: 获取群 chat_id（发送给群组）

1. **创建群聊**
   - 在飞书中创建一个群
   - 将机器人添加到群中

2. **获取 chat_id**
   - 在开放平台使用"获取群列表" API
   - 或在群设置中查看群 ID

---

## 🔧 配置代码

编辑文件：`D:/robot/code/3dnav/3d_NAV/scripts/feishu_monitor_bot.py`

修改第 109-111 行：

```python
APP_ID = "cli_xxxxxxxxxxxxxxxx"  # 替换为你的 App ID
APP_SECRET = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"  # 替换为你的 App Secret
RECEIVE_ID = "ou_xxxxxxxxxxxxxxxxxxxxxxxx"  # 替换为接收者的 open_id
```

---

## 🎯 运行机器人

```bash
# 确保 ROS2 环境已激活
source /opt/ros/humble/setup.bash
source ~/3d_NAV/install/setup.bash

# 运行飞书监控机器人
cd D:/robot/code/3dnav/3d_NAV/scripts
python3 feishu_monitor_bot.py
```

---

## 📊 功能特性

### 1. 自动状态推送
机器人会自动监听 `/nav/semantic/status` 话题，当导航状态变化时发送消息到飞书。

### 2. 美观的卡片消息
使用飞书卡片消息，显示更美观：
- 蓝色标题栏
- Markdown 格式内容
- 支持表情符号

### 3. 启动通知
机器人启动时会发送欢迎卡片。

---

## 🔍 测试机器人

### 测试发送消息

创建测试脚本 `test_feishu.py`：

```python
import lark_oapi as lark
from lark_oapi.api.im.v1 import *
import json

APP_ID = "你的_APP_ID"
APP_SECRET = "你的_APP_SECRET"
RECEIVE_ID = "你的_RECEIVE_ID"

# 创建客户端
client = lark.Client.builder() \
    .app_id(APP_ID) \
    .app_secret(APP_SECRET) \
    .build()

# 发送测试消息
request = CreateMessageRequest.builder() \
    .receive_id_type("open_id") \
    .request_body(
        CreateMessageRequestBody.builder()
        .receive_id(RECEIVE_ID)
        .msg_type("text")
        .content(json.dumps({"text": "🤖 thunder 测试消息"}))
        .build()
    ) \
    .build()

response = client.im.v1.message.create(request)

if response.success():
    print("✅ 消息发送成功！")
else:
    print(f"❌ 发送失败: {response.code} - {response.msg}")
```

运行测试：
```bash
python3 test_feishu.py
```

---

## 🐛 故障排查

### 问题 1: "app_access_token invalid"
- **原因**: App ID 或 App Secret 错误
- **解决**: 检查凭证是否正确复制

### 问题 2: "no permission"
- **原因**: 权限未配置或未发布版本
- **解决**:
  1. 检查权限管理中是否添加了 `im:message` 权限
  2. 确保应用已发布

### 问题 3: "invalid receive_id"
- **原因**: 接收者 ID 错误
- **解决**:
  1. 确认使用的是 open_id（以 `ou_` 开头）
  2. 确认机器人已添加为好友或在群中

### 问题 4: 收不到消息
- **原因**: 机器人未添加到群或未添加好友
- **解决**:
  1. 在飞书中搜索应用名称
  2. 添加机器人为联系人

---

## 📚 参考资源

- 飞书开放平台: https://open.feishu.cn/
- 官方文档: https://open.feishu.cn/document/home/introduction-to-custom-app-development/self-built-application-development-process
- Python SDK: https://github.com/larksuite/oapi-sdk-python
- API 参考: https://open.feishu.cn/document/server-docs/im-v1/message/create

---

## 🎉 完成！

配置完成后，你就可以通过飞书远程监控你的 3D-NAV 机器人了！

**thunder 机器人特性：**
- ✅ 实时状态推送
- ✅ 美观的卡片消息
- ✅ 不需要翻墙
- ✅ 支持群聊和私聊
