# thunder 项目文件清单

本文档列出了 thunder 飞书监控机器人的所有文件及其用途。

---

## 📂 核心程序文件

### scripts/feishu_monitor_bot.py
**类型**: Python 脚本
**用途**: thunder 机器人主程序
**功能**:
- ROS2 节点，监听 `/nav/semantic/status` 话题
- 飞书客户端，发送消息到飞书
- 支持文本消息和卡片消息
- 启动/停止通知

**配置位置**: 第 149-151 行
```python
APP_ID = "cli_xxxxxxxxxxxxxxxx"
APP_SECRET = "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
RECEIVE_ID = "ou_xxxxxxxxxxxxxxxxxxxxxxxx"
```

---

## 🧪 测试和诊断工具

### scripts/test_feishu.py
**类型**: Python 脚本
**用途**: 测试飞书配置
**功能**:
- 验证 App ID、App Secret、Receive ID
- 发送测试消息
- 检测常见配置错误
- 提供修复建议

**使用**: `python3 test_feishu.py`

### scripts/diagnose_thunder.py
**类型**: Python 脚本（可执行）
**用途**: 自动诊断系统环境
**功能**:
- 检查 Python 版本
- 检查 pip3 和依赖包
- 检查 ROS2 环境
- 检查 ROS2 话题
- 检查配置文件
- 检查脚本权限
- 生成诊断报告

**使用**: `./diagnose_thunder.py`

---

## 🚀 安装和启动脚本

### scripts/install_feishu_bot.sh
**类型**: Bash 脚本（可执行）
**用途**: 一键安装依赖
**功能**:
- 检查 Python 和 pip3
- 安装 lark-oapi SDK
- 检查 ROS2 环境
- 显示后续配置步骤

**使用**: `./install_feishu_bot.sh`

### scripts/start_thunder.sh
**类型**: Bash 脚本（可执行）
**用途**: 启动 thunder 机器人
**功能**:
- 自动激活 ROS2 环境
- 激活 3D-NAV 工作空间
- 启动机器人
- 显示运行状态

**使用**: `./start_thunder.sh`

---

## 🔧 系统服务文件

### scripts/thunder_service.sh
**类型**: Bash 脚本（可执行）
**用途**: 管理 systemd 服务
**功能**:
- 安装/卸载服务
- 启动/停止/重启服务
- 查看服务状态
- 查看实时日志

**使用**:
```bash
sudo ./thunder_service.sh install   # 安装服务
sudo ./thunder_service.sh start     # 启动服务
sudo ./thunder_service.sh status    # 查看状态
sudo ./thunder_service.sh logs      # 查看日志
```

### scripts/thunder.service
**类型**: systemd 服务配置文件
**用途**: systemd 服务定义
**功能**:
- 定义服务启动方式
- 配置环境变量
- 设置重启策略
- 配置日志输出

**安装位置**: `/etc/systemd/system/thunder.service`

---

## 📋 配置文件

### scripts/requirements_feishu.txt
**类型**: pip 依赖列表
**用途**: Python 依赖管理
**内容**:
```
lark-oapi>=1.5.3
```

**使用**: `pip3 install -r requirements_feishu.txt`

### scripts/feishu_config_template.py
**类型**: Python 配置模板
**用途**: 配置文件模板
**功能**:
- 提供配置示例
- 说明各配置项含义
- 使用说明

**使用**: 复制为 `feishu_config.py` 并修改

### scripts/.gitignore
**类型**: Git 忽略规则
**用途**: 防止提交敏感信息
**内容**:
- `feishu_config.py` - 配置文件
- `*.secret` - 密钥文件
- `__pycache__/` - Python 缓存
- `*.log` - 日志文件

---

## 📚 文档文件

### docs/guides/FEISHU_BOT_SETUP.md
**类型**: Markdown 文档
**用途**: 详细配置指南
**内容**:
- 飞书平台介绍
- 创建飞书应用步骤
- 配置权限说明
- 获取凭证方法
- 代码配置说明
- 运行指南
- 故障排查
- 参考资源

**适用**: 首次配置用户

### scripts/README.md
**类型**: Markdown 文档
**用途**: 脚本目录说明
**内容**:
- 文件列表
- 快速开始
- 功能特性
- 配置要求
- 故障排查
- 参考资源

**适用**: 了解脚本工具

### scripts/QUICK_REFERENCE.md
**类型**: Markdown 文档
**用途**: 快速参考卡片
**内容**:
- 一键安装命令
- 配置信息位置
- 测试命令
- 运行命令
- 获取凭证步骤
- 常见错误对照表

**适用**: 快速查询

### scripts/DEPLOYMENT_CHECKLIST.md
**类型**: Markdown 文档
**用途**: 部署检查清单
**内容**:
- 环境准备检查项
- 飞书应用配置检查项
- 依赖安装检查项
- 配置文件检查项
- 测试验证检查项
- ROS2 环境检查项
- 运行测试检查项
- 系统服务检查项
- 功能验证检查项
- 故障排查指南

**适用**: 部署前检查

### scripts/THUNDER_SUMMARY.md
**类型**: Markdown 文档
**用途**: 项目总结文档
**内容**:
- 项目概述
- 快速开始
- 文件结构
- 工具说明
- 消息格式
- 部署方式
- 故障排查
- 文档索引
- 安全建议
- 性能特点
- 使用场景
- 版本历史

**适用**: 全面了解项目

### scripts/VERSION.md
**类型**: Markdown 文档
**用途**: 版本信息和历史
**内容**:
- 当前版本
- 版本历史
- 依赖版本
- 兼容性说明
- 已知问题
- 计划功能
- 升级指南
- 贡献指南

**适用**: 版本管理

### scripts/FILE_MANIFEST.md
**类型**: Markdown 文档
**用途**: 文件清单（本文件）
**内容**:
- 所有文件列表
- 文件类型和用途
- 使用方法
- 文件关系

**适用**: 了解项目结构

---

## 📊 文件统计

### 按类型分类

| 类型 | 数量 | 文件 |
|------|------|------|
| Python 脚本 | 3 | feishu_monitor_bot.py, test_feishu.py, diagnose_thunder.py |
| Bash 脚本 | 3 | install_feishu_bot.sh, start_thunder.sh, thunder_service.sh |
| 配置文件 | 4 | requirements_feishu.txt, feishu_config_template.py, thunder.service, .gitignore |
| 文档文件 | 7 | FEISHU_BOT_SETUP.md, README.md, QUICK_REFERENCE.md, DEPLOYMENT_CHECKLIST.md, THUNDER_SUMMARY.md, VERSION.md, FILE_MANIFEST.md |
| **总计** | **17** | |

### 按功能分类

| 功能 | 文件数 | 说明 |
|------|--------|------|
| 核心功能 | 1 | 主程序 |
| 测试诊断 | 2 | 测试和诊断工具 |
| 安装部署 | 5 | 安装、启动、服务管理 |
| 配置管理 | 4 | 依赖、配置、忽略规则 |
| 文档说明 | 7 | 各类文档 |

---

## 🔗 文件关系图

```
thunder 项目
│
├── 核心程序
│   └── feishu_monitor_bot.py (主程序)
│
├── 工具链
│   ├── test_feishu.py (测试)
│   ├── diagnose_thunder.py (诊断)
│   ├── install_feishu_bot.sh (安装)
│   ├── start_thunder.sh (启动)
│   └── thunder_service.sh (服务管理)
│
├── 配置
│   ├── requirements_feishu.txt (依赖)
│   ├── feishu_config_template.py (模板)
│   ├── thunder.service (服务配置)
│   └── .gitignore (忽略规则)
│
└── 文档
    ├── FEISHU_BOT_SETUP.md (详细指南)
    ├── README.md (脚本说明)
    ├── QUICK_REFERENCE.md (快速参考)
    ├── DEPLOYMENT_CHECKLIST.md (部署清单)
    ├── THUNDER_SUMMARY.md (项目总结)
    ├── VERSION.md (版本信息)
    └── FILE_MANIFEST.md (文件清单)
```

---

## 📝 使用流程

### 首次部署流程

1. **阅读文档**: `FEISHU_BOT_SETUP.md`
2. **安装依赖**: `./install_feishu_bot.sh`
3. **创建应用**: 访问飞书开放平台
4. **配置机器人**: 编辑 `feishu_monitor_bot.py`
5. **运行诊断**: `./diagnose_thunder.py`
6. **测试配置**: `python3 test_feishu.py`
7. **启动机器人**: `./start_thunder.sh`
8. **检查清单**: `DEPLOYMENT_CHECKLIST.md`

### 日常使用流程

1. **启动**: `./start_thunder.sh`
2. **查看状态**: 检查飞书消息
3. **停止**: `Ctrl+C`

### 生产部署流程

1. **安装服务**: `sudo ./thunder_service.sh install`
2. **启动服务**: `sudo ./thunder_service.sh start`
3. **查看状态**: `sudo ./thunder_service.sh status`
4. **查看日志**: `sudo ./thunder_service.sh logs`

---

## 🎯 快速查找

**我想...**

- **首次配置** → `FEISHU_BOT_SETUP.md`
- **快速查询命令** → `QUICK_REFERENCE.md`
- **检查部署** → `DEPLOYMENT_CHECKLIST.md`
- **了解项目** → `THUNDER_SUMMARY.md`
- **查看版本** → `VERSION.md`
- **了解文件** → `FILE_MANIFEST.md` (本文件)
- **测试配置** → `test_feishu.py`
- **诊断问题** → `diagnose_thunder.py`
- **安装依赖** → `install_feishu_bot.sh`
- **启动机器人** → `start_thunder.sh`
- **管理服务** → `thunder_service.sh`

---

**thunder v1.0.0** - 完整文件清单
