# Telegram 监控机器人配置指南

## 📱 什么是 Telegram？

Telegram 是一个免费的即时通讯应用，类似微信，但：
- ✅ 在中国可以使用（需要翻墙）
- ✅ 支持机器人 API
- ✅ 完全免费
- ✅ 隐私保护更好

---

## 🚀 快速配置步骤

### 步骤 1: 安装 Telegram

**手机端：**
- Android: Google Play Store 搜索 "Telegram"
- iOS: App Store 搜索 "Telegram"

**电脑端：**
- Windows: https://telegram.org/dl/desktop/win64
- 或使用网页版: https://web.telegram.org

### 步骤 2: 创建 Telegram Bot

1. **打开 Telegram，搜索 `@BotFather`**
   - 这是 Telegram 官方的机器人管理工具

2. **发送命令创建机器人：**
   ```
   /newbot
   ```

3. **按提示操作：**
   ```
   BotFather: Alright, a new bot. How are we going to call it?
   你: 3D-NAV Monitor Bot

   BotFather: Good. Now let's choose a username for your bot.
   你: dnav_monitor_bot

   BotFather: Done! Your bot token is:
   1234567890:ABCdefGHIjklMNOpqrsTUVwxyz
   ```

4. **保存 Bot Token**（重要！）
   - 示例: `1234567890:ABCdefGHIjklMNOpqrsTUVwxyz`

### 步骤 3: 获取你的 Chat ID

1. **搜索并启动你的机器人**
   - 在 Telegram 搜索 `@dnav_monitor_bot`
   - 点击 "Start" 或发送 `/start`

2. **获取 Chat ID：**
   - 访问: `https://api.telegram.org/bot<YOUR_BOT_TOKEN>/getUpdates`
   - 替换 `<YOUR_BOT_TOKEN>` 为你的实际 token
   - 在返回的 JSON 中找到 `"chat":{"id":123456789}`
   - 这个数字就是你的 Chat ID

### 步骤 4: 安装依赖

```bash
# 安装 python-telegram-bot
pip install python-telegram-bot

# 或使用 conda
conda install -c conda-forge python-telegram-bot
```

### 步骤 5: 配置机器人

编辑 `telegram_monitor_bot.py` 文件：

```python
# 第 85-86 行，替换为你的实际值
BOT_TOKEN = "1234567890:ABCdefGHIjklMNOpqrsTUVwxyz"  # 从 BotFather 获取
CHAT_ID = "123456789"  # 你的 Chat ID
```

### 步骤 6: 运行机器人

```bash
# 确保 ROS2 环境已激活
source /opt/ros/humble/setup.bash
source ~/3d_NAV/install/setup.bash

# 运行监控机器人
python3 telegram_monitor_bot.py
```

---

## 🎯 使用方法

### 在 Telegram 中发送命令：

```
/start   - 启动机器人
/status  - 查询当前导航状态
/stop    - 停止导航
/help    - 查看帮助
```

### 自动接收状态更新

机器人会自动监听 `/nav/semantic/status` 话题，当导航状态变化时自动发送消息到你的 Telegram。

---

## 🔧 高级配置

### 1. 添加更多命令

编辑 `telegram_monitor_bot.py`，添加新的命令处理器：

```python
async def cmd_goto(self, update: Update, context: ContextTypes.DEFAULT_TYPE):
    """处理 /goto 命令"""
    if not context.args:
        await update.message.reply_text("用法: /goto <目标物体>")
        return

    target = " ".join(context.args)
    # TODO: 发布导航目标到 ROS2
    await update.message.reply_text(f"导航到: {target}")

# 在 setup_telegram 中注册
self.app.add_handler(CommandHandler("goto", self.cmd_goto))
```

### 2. 添加图像发送

```python
async def send_image(self, image_path: str):
    """发送图像到 Telegram"""
    with open(image_path, 'rb') as photo:
        await self.app.bot.send_photo(
            chat_id=self.chat_id,
            photo=photo,
            caption="当前视角"
        )
```

### 3. 配置多个接收者

```python
# 支持多个 Chat ID
CHAT_IDS = ["123456789", "987654321"]

async def send_message(self, text: str):
    """发送到所有接收者"""
    for chat_id in CHAT_IDS:
        await self.app.bot.send_message(chat_id=chat_id, text=text)
```

---

## 🐛 故障排查

### 问题 1: "Unauthorized" 错误
- **原因**: Bot Token 错误
- **解决**: 检查 token 是否正确复制

### 问题 2: 收不到消息
- **原因**: Chat ID 错误或未启动机器人
- **解决**:
  1. 确保在 Telegram 中点击了 "Start"
  2. 重新获取 Chat ID

### 问题 3: 导入错误
- **原因**: 未安装 python-telegram-bot
- **解决**: `pip install python-telegram-bot`

### 问题 4: ROS2 连接失败
- **原因**: ROS2 环境未激活
- **解决**: `source install/setup.bash`

---

## 📚 参考资源

- Telegram Bot API 文档: https://core.telegram.org/bots/api
- python-telegram-bot 文档: https://docs.python-telegram-bot.org/
- BotFather 命令列表: https://core.telegram.org/bots#botfather

---

## ⚠️ 注意事项

1. **不要泄露 Bot Token**
   - Token 相当于密码，不要提交到 Git
   - 使用环境变量或配置文件

2. **网络要求**
   - Telegram 在中国需要翻墙
   - 确保机器人运行的服务器可以访问 Telegram API

3. **安全性**
   - 建议只允许特定 Chat ID 发送命令
   - 添加命令权限验证

---

## 🎉 完成！

配置完成后，你就可以通过 Telegram 远程监控你的 3D-NAV 机器人了！
