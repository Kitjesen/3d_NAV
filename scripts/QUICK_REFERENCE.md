# thunder 机器人快速参考

## ⚡ 一键安装

```bash
cd D:/robot/code/3dnav/3d_NAV/scripts
./install_feishu_bot.sh
```

## 🔑 配置信息

编辑 `feishu_monitor_bot.py` 第 149-151 行：

```python
APP_ID = "cli_xxxxxxxxxxxxxxxx"      # 飞书开放平台获取
APP_SECRET = "xxxxxxxxxxxxxxxx"       # 飞书开放平台获取
RECEIVE_ID = "ou_xxxxxxxxxxxxxxxx"    # 用户 open_id
```

## 🧪 测试配置

```bash
python3 test_feishu.py
```

## 🚀 运行机器人

```bash
# 激活 ROS2 环境
source /opt/ros/humble/setup.bash
source ~/3d_NAV/install/setup.bash

# 启动 thunder
python3 feishu_monitor_bot.py
```

## 📱 获取凭证

1. **访问**: https://open.feishu.cn/
2. **创建应用**: 企业自建应用 → 名称: thunder
3. **获取凭证**: 凭证与基础信息 → 复制 App ID 和 App Secret
4. **配置权限**: 权限管理 → 添加 `im:message` 和 `im:message:send_as_bot`
5. **发布应用**: 版本管理与发布 → 创建版本并发布
6. **获取 ID**:
   - 用户: 开发调试 → 获取用户信息 → open_id
   - 群组: 获取群列表 → chat_id

## 🎯 监听话题

```
/nav/semantic/status
```

## 📊 消息格式

```
🤖 3D-NAV 状态更新 (thunder)
📍 当前状态: NAVIGATING
🎯 目标: kitchen
📏 距离: 2.35m
⏱️ 时间: 15.2s
✅ 成功率: 87.5%
```

## 🐛 常见错误

| 错误 | 原因 | 解决 |
|------|------|------|
| `app_access_token invalid` | 凭证错误 | 检查 App ID/Secret |
| `no permission` | 权限未配置 | 添加 im:message 权限 |
| `invalid receive_id` | ID 错误 | 确认 open_id 格式 |
| 收不到消息 | 未添加好友 | 搜索应用并添加 |

## 📚 完整文档

`docs/guides/FEISHU_BOT_SETUP.md`

---

**thunder** - 3D-NAV 飞书监控机器人 🤖
