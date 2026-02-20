# thunder 版本信息

## 当前版本

**v1.0.0** - 2026-02-18

---

## 版本历史

### v1.0.0 (2026-02-18)

#### 🎉 初始发布

**核心功能**
- ✅ ROS2 节点集成，监听 `/nav/semantic/status` 话题
- ✅ 飞书消息推送（文本和卡片格式）
- ✅ 启动/停止通知
- ✅ 状态格式化和美化

**工具链**
- ✅ `feishu_monitor_bot.py` - 主程序
- ✅ `test_feishu.py` - 配置测试工具
- ✅ `diagnose_thunder.py` - 自动诊断工具
- ✅ `install_feishu_bot.sh` - 安装脚本
- ✅ `start_thunder.sh` - 启动脚本
- ✅ `thunder_service.sh` - 服务管理脚本

**文档**
- ✅ `FEISHU_BOT_SETUP.md` - 详细配置指南
- ✅ `QUICK_REFERENCE.md` - 快速参考
- ✅ `DEPLOYMENT_CHECKLIST.md` - 部署清单
- ✅ `THUNDER_SUMMARY.md` - 项目总结
- ✅ `README.md` - 脚本说明

**部署支持**
- ✅ 直接运行模式
- ✅ systemd 服务模式
- ✅ 开机自启动支持

**依赖**
- Python >= 3.8
- lark-oapi >= 1.5.3
- ROS2 Humble
- rclpy

---

## 依赖版本

| 组件 | 版本要求 | 说明 |
|------|----------|------|
| Python | >= 3.8 | 运行环境 |
| lark-oapi | >= 1.5.3 | 飞书官方 SDK |
| ROS2 | Humble | 机器人操作系统 |
| rclpy | - | ROS2 Python 客户端 |

---

## 兼容性

### 操作系统
- ✅ Ubuntu 22.04 LTS
- ✅ Ubuntu 20.04 LTS
- ⚠️ 其他 Linux 发行版（未测试）

### ROS2 版本
- ✅ ROS2 Humble
- ⚠️ ROS2 Foxy（未测试）
- ⚠️ ROS2 Galactic（未测试）

### Python 版本
- ✅ Python 3.8
- ✅ Python 3.9
- ✅ Python 3.10
- ✅ Python 3.11

---

## 已知问题

目前无已知问题。

---

## 计划功能

### v1.1.0 (计划中)
- [ ] 支持多个接收者
- [ ] 自定义消息模板
- [ ] 消息频率控制
- [ ] 更多话题监听

### v1.2.0 (计划中)
- [ ] Web 配置界面
- [ ] 消息历史记录
- [ ] 性能监控面板
- [ ] 告警规则配置

### v2.0.0 (未来)
- [ ] 支持其他消息平台（钉钉、企业微信）
- [ ] 双向控制（通过飞书控制机器人）
- [ ] 数据可视化
- [ ] 机器学习异常检测

---

## 更新日志格式

每个版本的更新日志包含：

- **🎉 新功能**: 新增的功能特性
- **🐛 Bug 修复**: 修复的问题
- **⚡ 性能优化**: 性能改进
- **📚 文档**: 文档更新
- **🔧 其他**: 其他改进

---

## 升级指南

### 从源码更新

```bash
cd D:/robot/code/3dnav/3d_NAV
git pull origin main

# 更新依赖
cd scripts
pip3 install -r requirements_feishu.txt --upgrade

# 重启服务（如果使用系统服务）
sudo ./thunder_service.sh restart
```

### 检查版本

```bash
# 查看 lark-oapi 版本
pip3 show lark-oapi

# 运行诊断工具
./diagnose_thunder.py
```

---

## 贡献指南

如果你想为 thunder 贡献代码：

1. Fork 项目
2. 创建功能分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 创建 Pull Request

---

## 许可证

本项目是 3D-NAV 项目的一部分。

---

**thunder v1.0.0** - 2026-02-18
