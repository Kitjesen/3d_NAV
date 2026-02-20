# 3D-NAV 脚本工具集

本目录包含 3D-NAV 项目的各种实用脚本和监控工具。

## 📱 飞书机器人 - thunder

远程监控机器人导航状态的飞书机器人。

### 文件列表

| 文件 | 说明 |
|------|------|
| `feishu_monitor_bot.py` | 飞书监控机器人主程序 |
| `test_feishu.py` | 飞书配置测试工具 |
| `diagnose_thunder.py` | 自动诊断工具 |
| `install_feishu_bot.sh` | 一键安装脚本 |
| `start_thunder.sh` | 启动脚本（自动配置环境） |
| `thunder_service.sh` | 系统服务管理脚本 |
| `thunder.service` | systemd 服务配置文件 |
| `requirements_feishu.txt` | Python 依赖列表 |
| `feishu_config_template.py` | 配置文件模板 |
| `QUICK_REFERENCE.md` | 快速参考卡片 |
| `DEPLOYMENT_CHECKLIST.md` | 部署检查清单 |

### 快速开始

```bash
# 1. 安装依赖
cd D:/robot/code/3dnav/3d_NAV/scripts
./install_feishu_bot.sh

# 或手动安装
pip3 install -r requirements_feishu.txt

# 2. 运行诊断（推荐）
./diagnose_thunder.py

# 3. 测试配置
python3 test_feishu.py

# 4. 编辑配置
# 编辑 feishu_monitor_bot.py 第 149-151 行
# 填入你的 App ID, App Secret, Receive ID

# 5. 运行机器人
./start_thunder.sh

# 或手动运行
source /opt/ros/humble/setup.bash
source ~/3d_NAV/install/setup.bash
python3 feishu_monitor_bot.py
```

### 系统服务（可选）

如果需要开机自启动：

```bash
# 安装服务
sudo ./thunder_service.sh install

# 启动服务
sudo ./thunder_service.sh start

# 查看状态
sudo ./thunder_service.sh status

# 查看日志
sudo ./thunder_service.sh logs

# 停止服务
sudo ./thunder_service.sh stop

# 卸载服务
sudo ./thunder_service.sh uninstall
```

### 详细文档

完整配置指南: [FEISHU_BOT_SETUP.md](../docs/guides/FEISHU_BOT_SETUP.md)

---

## 🤖 功能特性

### thunder 机器人

- ✅ 实时监听 `/nav/semantic/status` 话题
- ✅ 自动推送导航状态变化
- ✅ 支持文本消息和美观的卡片消息
- ✅ 启动/停止通知
- ✅ 不需要翻墙，国内直接使用

### 消息格式

```
🤖 3D-NAV 状态更新 (thunder)

📍 当前状态: NAVIGATING
🎯 目标: kitchen
📏 距离: 2.35m
⏱️ 时间: 15.2s
✅ 成功率: 87.5%
```

---

## 📋 配置要求

### 飞书应用配置

1. **创建应用**: https://open.feishu.cn/
2. **获取凭证**: App ID, App Secret
3. **配置权限**: `im:message`, `im:message:send_as_bot`
4. **获取接收者**: open_id (用户) 或 chat_id (群组)

### ROS2 环境

- ROS2 Humble 或更高版本
- 已编译的 3D-NAV 工作空间
- `/nav/semantic/status` 话题可用

---

## 🐛 故障排查

### 常见问题

**问题**: "app_access_token invalid"
- **解决**: 检查 App ID 和 App Secret 是否正确

**问题**: "no permission"
- **解决**: 确保已添加 `im:message` 权限并发布应用

**问题**: "invalid receive_id"
- **解决**: 确认使用正确的 open_id (以 `ou_` 开头)

**问题**: 收不到消息
- **解决**: 确保机器人已添加为好友或在群中

---

## 📚 参考资源

- 飞书开放平台: https://open.feishu.cn/
- Python SDK: https://github.com/larksuite/oapi-sdk-python
- API 文档: https://open.feishu.cn/document/server-docs/im-v1/message/create

---

## 🎉 开始使用

配置完成后，你就可以通过飞书远程监控你的 3D-NAV 机器人了！

**需要帮助？** 查看详细文档或在 Issues 中提问。
