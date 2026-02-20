# thunder - 3D-NAV 飞书监控机器人

## 📖 项目概述

thunder 是为 3D-NAV 机器人导航系统开发的飞书监控机器人，可以实时推送导航状态到飞书，方便远程监控机器人运行状态。

### 主要特性

- ✅ **实时监控**: 自动监听 `/nav/semantic/status` 话题
- ✅ **即时推送**: 导航状态变化时立即发送消息到飞书
- ✅ **美观展示**: 支持文本消息和卡片消息两种格式
- ✅ **国内可用**: 基于飞书平台，无需翻墙
- ✅ **灵活部署**: 支持直接运行和系统服务两种方式
- ✅ **完善工具**: 提供测试、诊断、安装等完整工具链

---

## 🚀 快速开始

### 1. 安装依赖

```bash
cd D:/robot/code/3dnav/3d_NAV/scripts
./install_feishu_bot.sh
```

### 2. 创建飞书应用

访问 https://open.feishu.cn/ 并完成以下步骤：

1. 创建企业自建应用，名称: **thunder**
2. 获取 **App ID** 和 **App Secret**
3. 配置权限: `im:message` 和 `im:message:send_as_bot`
4. 发布应用
5. 获取接收者 **open_id** 或 **chat_id**

详细步骤: [FEISHU_BOT_SETUP.md](../docs/guides/FEISHU_BOT_SETUP.md)

### 3. 配置机器人

编辑 `feishu_monitor_bot.py` 第 149-151 行：

```python
APP_ID = "cli_xxxxxxxxxxxxxxxx"      # 你的 App ID
APP_SECRET = "xxxxxxxxxxxxxxxx"       # 你的 App Secret
RECEIVE_ID = "ou_xxxxxxxxxxxxxxxx"    # 接收者 open_id
```

### 4. 测试配置

```bash
python3 test_feishu.py
```

### 5. 运行机器人

```bash
./start_thunder.sh
```

---

## 📁 文件结构

```
scripts/
├── feishu_monitor_bot.py          # 主程序
├── test_feishu.py                 # 测试工具
├── diagnose_thunder.py            # 诊断工具
├── install_feishu_bot.sh          # 安装脚本
├── start_thunder.sh               # 启动脚本
├── thunder_service.sh             # 服务管理脚本
├── thunder.service                # systemd 服务文件
├── requirements_feishu.txt        # 依赖列表
├── feishu_config_template.py      # 配置模板
├── README.md                      # 脚本说明
├── QUICK_REFERENCE.md             # 快速参考
├── DEPLOYMENT_CHECKLIST.md        # 部署清单
└── THUNDER_SUMMARY.md             # 本文件

docs/guides/
└── FEISHU_BOT_SETUP.md            # 详细配置指南
```

---

## 🛠️ 工具说明

### 核心程序

**feishu_monitor_bot.py**
- ROS2 节点 + 飞书机器人
- 监听导航状态并推送到飞书
- 支持文本和卡片两种消息格式

### 辅助工具

**test_feishu.py**
- 测试飞书配置是否正确
- 验证 App ID、App Secret、Receive ID
- 发送测试消息

**diagnose_thunder.py**
- 自动诊断系统环境
- 检查 Python、ROS2、依赖包
- 验证配置文件和脚本权限

**install_feishu_bot.sh**
- 一键安装所有依赖
- 检查 Python 和 ROS2 环境
- 显示后续配置步骤

**start_thunder.sh**
- 自动激活 ROS2 环境
- 启动 thunder 机器人
- 显示运行状态

**thunder_service.sh**
- 管理 systemd 服务
- 支持安装、启动、停止、查看日志
- 实现开机自启动

---

## 📊 消息格式

### 状态更新消息

```
🤖 3D-NAV 状态更新 (thunder)

📍 当前状态: NAVIGATING
🎯 目标: kitchen
📏 距离: 2.35m
⏱️ 时间: 15.2s
✅ 成功率: 87.5%
```

### 启动消息（卡片格式）

```
┌─────────────────────────────┐
│ 🤖 thunder 机器人已启动     │
├─────────────────────────────┤
│ 3D-NAV 监控系统             │
│                             │
│ ✅ 已连接到 ROS2            │
│ ✅ 正在监听导航状态         │
└─────────────────────────────┘
```

---

## 🔧 部署方式

### 方式 1: 直接运行（开发/测试）

```bash
./start_thunder.sh
```

**优点**: 简单直接，便于调试
**缺点**: 需要手动启动，终端关闭后停止

### 方式 2: 系统服务（生产环境）

```bash
# 安装服务
sudo ./thunder_service.sh install

# 启动服务
sudo ./thunder_service.sh start

# 查看状态
sudo ./thunder_service.sh status

# 查看日志
sudo ./thunder_service.sh logs
```

**优点**: 开机自启，后台运行，自动重启
**缺点**: 需要 root 权限，配置稍复杂

---

## 🐛 故障排查

### 使用诊断工具

```bash
./diagnose_thunder.py
```

诊断工具会自动检查：
- Python 环境
- pip3 安装
- lark-oapi SDK
- ROS2 环境
- ROS2 话题
- 配置文件
- 脚本权限

### 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| `app_access_token invalid` | 凭证错误 | 检查 App ID 和 App Secret |
| `no permission` | 权限未配置 | 添加 `im:message` 权限并发布应用 |
| `invalid receive_id` | ID 格式错误 | 确认使用正确的 open_id (以 `ou_` 开头) |
| 收不到消息 | 未添加好友 | 在飞书中搜索应用并添加为联系人 |
| ROS2 话题不存在 | 导航节点未运行 | 启动 3D-NAV 导航系统 |

### 查看日志

**直接运行模式**:
```bash
# 日志会直接输出到终端
./start_thunder.sh
```

**系统服务模式**:
```bash
# 查看实时日志
sudo ./thunder_service.sh logs

# 或使用 journalctl
sudo journalctl -u thunder -f
```

---

## 📚 文档索引

- **详细配置指南**: [FEISHU_BOT_SETUP.md](../docs/guides/FEISHU_BOT_SETUP.md)
- **快速参考卡片**: [QUICK_REFERENCE.md](QUICK_REFERENCE.md)
- **部署检查清单**: [DEPLOYMENT_CHECKLIST.md](DEPLOYMENT_CHECKLIST.md)
- **脚本工具说明**: [README.md](README.md)

---

## 🔐 安全建议

1. **不要提交敏感信息**
   - `.gitignore` 已配置忽略 `feishu_config.py`
   - 不要将 App Secret 提交到版本控制

2. **权限最小化**
   - 只添加必需的飞书权限
   - 使用专用账号运行服务

3. **定期更新**
   - 保持 lark-oapi SDK 最新版本
   - 关注飞书 API 变更

---

## 📈 性能特点

- **低延迟**: 状态变化后立即推送（< 1秒）
- **低资源**: 内存占用 < 50MB
- **高可靠**: 自动重连，异常恢复
- **可扩展**: 易于添加新的监控话题

---

## 🎯 使用场景

1. **远程监控**: 在办公室监控实验室的机器人
2. **多人协作**: 团队成员共同关注机器人状态
3. **异常告警**: 导航失败时及时通知
4. **数据记录**: 飞书消息可作为运行日志

---

## 🔄 版本历史

**v1.0.0** (2026-02-18)
- ✅ 初始版本发布
- ✅ 支持状态监控和消息推送
- ✅ 提供完整工具链
- ✅ 支持系统服务部署

---

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

---

## 📞 支持

如有问题，请查看：
1. [故障排查](#-故障排查)
2. [文档索引](#-文档索引)
3. 运行诊断工具: `./diagnose_thunder.py`

---

**thunder** - 让远程监控变得简单 🚀
