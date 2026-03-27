# 灵途导航 — 脚本工具集

## lingtu.sh — 统一启动入口

```bash
./lingtu.sh map     # 建图模式（手柄遥控 + SLAM）
./lingtu.sh save    # 保存地图（PCD + Tomogram 全自动）
./lingtu.sh nav     # 导航模式（自动加载最新地图）
./lingtu.sh status  # 检查系统状态（话题、地图、TF）
```

---

## 目录结构

### deploy/

部署与安装脚本。包括依赖安装、网络配置、语义导航环境搭建、版本同步、感知 demo 启动等。

| 脚本 | 说明 |
|------|------|
| `deploy.sh` | 通用部署入口 |
| `install_deps.sh` | 基础依赖安装 |
| `install_semantic_deps.sh` | 语义导航依赖安装 |
| `setup_network.sh` | 网络配置 |
| `setup_semantic.sh` | 语义导航环境搭建 |
| `install_services.sh` | systemd 服务安装 |
| `sync_versions.sh` / `.ps1` | 版本号同步 (Linux/Windows) |
| `health_check.sh` | 系统健康检查 |
| `backup_config.sh` | 配置备份 |
| `start_perception_demo.sh` | 感知 demo 启动 |
| `install_feishu_bot.sh` | 飞书机器人安装 |
| `install_telegram_bot.sh` | Telegram 机器人安装 |
| `start_thunder.sh` | 飞书监控启动 |
| `thunder_service.sh` / `.service` | systemd 服务管理 |

### launch/

ROS2 Launch 文件。

| 文件 | 说明 |
|------|------|
| `nav_autonomy_launch.py` | 自主导航启动 |
| `nav_planning_launch.py` | 规划模块启动 |

### monitor/

远程监控机器人（飞书 / Telegram）。详见 [monitor/README.md](monitor/README.md)。

### ota/

OTA 远程更新系统。包括打包、推送、安装、manifest 生成等完整流程。

| 脚本 | 说明 |
|------|------|
| `build_nav_package.sh` | 打包导航软件包 |
| `push_to_robot.sh` | 推送到机器人 |
| `deploy_to_robot.sh` | 部署到机器人 |
| `install_nav.sh` | 机器人端安装 |
| `setup_robot.sh` | 机器人初始化 |
| `generate_manifest.py` | 生成 OTA manifest |
| `start_mapping.sh` / `start_nav.sh` | 机器人端启动 |
| `mapping.service` / `navigation.service` | systemd 服务 |

### proto/

Protobuf/gRPC 协议代码生成。

| 脚本 | 说明 |
|------|------|
| `proto_gen.sh` | Linux 生成脚本 |
| `proto_gen.ps1` | Windows 生成脚本 |

### test/

测试与验证脚本。

| 脚本 | 说明 |
|------|------|
| `test_end_to_end.py` | 端到端集成测试 |
| `test_semantic_nav.sh` | 语义导航测试 |
| `test_services.sh` | 服务状态测试 |
| `test_feishu.py` | 飞书配置测试 |
| `diagnose_thunder.py` | thunder 诊断工具 |
| `validate_config.py` | 配置校验 |
| `validate_topics.py` | ROS2 话题校验 |
| `verify_phase0.sh` | Phase 0 验证 |

### legacy/

遗留的独立服务启动脚本（`env.sh`、`nav-slam.sh`、`nav-lidar.sh`、`nav-driver.sh`、`nav-planning.sh`、`nav-autonomy.sh`、`nav-semantic.sh`、`nav-grpc.sh`、`nav-monitor.sh`、`ota-daemon.sh`）。

> 已被 `lingtu.sh` 和 `deploy/install_services.sh` 取代，保留供参考。

---

## 根目录其他脚本

| 脚本 | 说明 |
|------|------|
| `clone_semantic_deps.sh` | 克隆 VLN 第三方依赖（LOVON、GroundingDINO、ConceptGraphs、SG-Nav） |
| `demo_semantic.sh` | 语义导航 Demo 启动（Stub TF + 场景图 + LLM Planner） |
