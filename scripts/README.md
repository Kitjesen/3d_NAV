# 灵途导航 — 脚本工具集

## 入口速查（优先用这些）

| 场景 | 推荐入口 |
|------|----------|
| Module-First 主流程（profile、REPL、daemon） | 仓库根目录 `python lingtu.py` 或 `lingtu`（见 `cli/`） |
| Shell 统一导航（建图 / 保存 / 导航 / 状态） | 仓库根目录执行 `scripts/lingtu.sh`（本文件所在目录下的 `lingtu.sh`） |
| 建图 / 规划栈 / 保存地图 / colcon 全量构建 | 根目录薄封装：`./mapping.sh`、`./planning.sh`、`./save_map.sh`、`./build_all.sh` → 实现见 **`ws/`**（[ws/README.md](ws/README.md)） |
| 环境自检 | `lingtu doctor` → 调用 `scripts/doctor.py` |
| Rerun 3D 可视化 | `lingtu rerun` → 调用 `scripts/rerun_live.py` |
| 部署 / systemd / 健康检查 | **`deploy/`** |
| OTA 打包与上机 | **`ota/`** |
| Protobuf 生成 | **`proto/`** |

---

## 分层说明

| 层级 | 目录 / 文件 | 说明 |
|------|-------------|------|
| **核心运维** | `deploy/`、`ota/`、`proto/`、`ws/` | 部署、上机、协议生成、ROS 工作区日常命令；应长期维护 |
| **CLI 挂钩** | `doctor.py`、`rerun_live.py` | 被 `cli/main.py` 的 `doctor` / `rerun` 子命令直接调用，勿随意改名或删 |
| **可选工具** | `monitor/`、`manager/` | 飞书/Telegram 监控；Web 管理端（`deploy/lingtu-manager.service` 引用 `manager/manager.py`） |
| **测试与一次性脚本** | `test/`、根目录 `test_*.py`、`live_*.py`、`run_rerun_mapping.py` 等 | 开发/台架用；无统一入口，按需保留 |
| **辅助 Launch** | `launch/` | 独立 Python launch 文件；仓库另有顶层 `launch/`（ROS2）。若新栈用 `lingtu.py`/统一 launch，此处仅作参考 |
| **遗留** | `legacy/` | 旧版按服务拆的 shell，已被 `lingtu.sh` + `deploy/install_services.sh` 取代，仅作对照 |

---

## lingtu.sh — Shell 统一启动

在 `brain/lingtu` 根目录下：

```bash
./scripts/lingtu.sh map     # 建图模式（手柄遥控 + SLAM）
./scripts/lingtu.sh save    # 保存地图（PCD + Tomogram 全自动）
./scripts/lingtu.sh nav     # 导航模式（自动加载最新地图）
./scripts/lingtu.sh status  # 检查系统状态（话题、地图、TF）
```

---

## 目录结构

### ws/

ROS / colcon 工作区辅助脚本；根目录 `mapping.sh`、`planning.sh`、`save_map.sh`、`build_all.sh` 为薄封装，指向此处。详见 [ws/README.md](ws/README.md)。

### deploy/

部署与安装脚本。包括依赖安装、网络配置、语义导航环境搭建、版本同步、感知 demo 启动、**infra/stack**（Docker、额外 systemd、cron/logrotate）等。

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

ROS2 Launch（Python）。与仓库顶层 `launch/` 可能功能重叠；新集成优先走 `lingtu.py` 与统一 launch 约定。

| 文件 | 说明 |
|------|------|
| `nav_autonomy_launch.py` | 自主导航启动 |
| `nav_planning_launch.py` | 规划模块启动 |

### monitor/

远程监控机器人（飞书 / Telegram）。详见 [monitor/README.md](monitor/README.md)。

### manager/

Web 管理端（`manager.py`）。systemd 单元 `deploy/lingtu-manager.service` 使用 `scripts/manager/manager.py`；内部会调用 `scripts/rerun_live.py` 等。

### ota/

OTA 远程更新：打包、推送、安装、manifest 等。

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
| `manifest_template.json` | manifest 模板 |

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

## 根目录其他脚本（`scripts/` 直下）

| 脚本 | 说明 |
|------|------|
| `doctor.py` | 环境/依赖自检（`lingtu doctor`） |
| `rerun_live.py` | Rerun 实时可视化（`lingtu rerun`；manager 亦可拉起） |
| `clone_semantic_deps.sh` | 克隆 VLN 第三方依赖（LOVON、GroundingDINO、ConceptGraphs、SG-Nav） |
| `demo_semantic.sh` | 语义导航 Demo 启动（Stub TF + 场景图 + LLM Planner） |
| `live_track.py` / `live_detect.py` | 现场调试/检测辅助（命令行见各文件头注释） |
| `run_rerun_mapping.py` | 建图 + Rerun 相关流程辅助 |
| `test_mapping.py`、`test_nav_planning.py`、`test_3fixes.py`、`test_mcp_full.py`、`test_s100p_start.py` | 定向测试脚本，非正式 CLI 入口 |
