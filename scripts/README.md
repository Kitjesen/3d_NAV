# scripts/ — LingTu 运维工具集

## 日常操作速查

推荐使用 `python lingtu.py`（主入口），以下脚本用于部署、运维、调试等场景。

| 我想… | 命令 |
|-------|------|
| 启动导航系统 | `python lingtu.py nav` |
| 环境自检 | `python lingtu.py doctor` |
| Shell 统一操作 | `./scripts/lingtu.sh map / save / nav / status` |
| 部署到 S100P | `bash scripts/deploy_s100p.sh` |
| 健康检查 | `bash scripts/deploy/health_check.sh` |
| OTA 打包上机 | `bash scripts/ota/build_nav_package.sh && bash scripts/ota/deploy_to_robot.sh` |
| 生成 Protobuf | `bash scripts/proto/proto_gen.sh` |
| Rerun 3D 可视化 | `python lingtu.py rerun` |

## 目录总览

### deploy/ — 部署与服务管理

S100P 真机部署相关的全部脚本和 systemd 服务文件。

**部署流程：**

| 脚本 | 用途 |
|------|------|
| `deploy.sh` | 通用部署入口（git pull + build + restart） |
| `deploy_s100p.sh` (上级) | S100P 一键部署 |
| `install_deps.sh` | 基础依赖安装 (apt + pip) |
| `install_semantic_deps.sh` | 语义导航依赖 (CLIP, ChromaDB 等) |
| `install_services.sh` | 安装全部 systemd 服务 |
| `setup_network.sh` | 网络配置 (IP, DDS domain) |
| `setup_semantic.sh` | 语义导航环境搭建 |

**运维工具：**

| 脚本 | 用途 |
|------|------|
| `health_check.sh` | 系统健康检查 (服务状态 + 话题 + 传感器) |
| `verify_linux_full.sh` | Linux 环境全量验证 |
| `backup_config.sh` | 配置文件备份 |
| `sync_versions.sh` / `.ps1` | 多仓库版本号同步 (Linux / Windows) |
| `sync_sunrise.ps1` | Windows → sunrise 同步 |
| `migrate_sunrise_layout.sh` | sunrise 目录迁移 (~data/nova → ~data/inovxio) |
| `debug_registry.py` | Module Registry 调试工具 |

**systemd 服务：**

| 文件 | 服务 |
|------|------|
| `lingtu.service` | LingTu 导航主服务 |
| `lingtu-manager.service` | Web 管理端 |
| `slam.service` | Fast-LIO2 SLAM |
| `slam_pgo.service` | Pose Graph Optimization |
| `localizer.service` | ICP 定位 |
| `brainstem.service` | Brainstem 运动控制 |
| `thunder.service` | Thunder 四足平台 |

**感知与监控启动：**

| 脚本 | 用途 |
|------|------|
| `start_perception_demo.sh` | BPU 感知 demo |
| `start_thunder.sh` | Thunder 平台启动 |
| `install_feishu_bot.sh` | 飞书监控机器人安装 |
| `install_telegram_bot.sh` | Telegram 监控机器人安装 |

---

### ota/ — OTA 远程更新

打包导航软件为 OTA 包，推送到机器人并安装。

| 脚本 | 用途 |
|------|------|
| `build_nav_package.sh` | 打包导航软件包 |
| `push_to_robot.sh` | 推送到机器人 |
| `deploy_to_robot.sh` | 部署到机器人 (push + install) |
| `install_nav.sh` | 机器人端安装脚本 |
| `setup_robot.sh` | 机器人初始化 (首次部署) |
| `generate_manifest.py` | 生成 OTA manifest JSON |
| `manifest_template.json` | manifest 模板 |
| `start_mapping.sh` | 机器人端启动建图 |
| `start_nav.sh` | 机器人端启动导航 |
| `mapping.service` | 建图 systemd 服务 |
| `navigation.service` | 导航 systemd 服务 |

---

### build/ — 构建脚本

ROS2 colcon 构建和传统 shell 启动器。

| 脚本 | 用途 |
|------|------|
| `build_all.sh` | colcon 全量构建 |
| `mapping.sh` | 传统建图启动 (被 `lingtu.py map` 替代) |
| `planning.sh` | 传统导航启动 (被 `lingtu.py nav` 替代) |
| `save_map.sh` | 保存地图 (被 REPL `map save` 替代) |

---

### monitor/ — 远程监控

飞书 / Telegram 机器人，实时推送机器人状态告警。

| 文件 | 用途 |
|------|------|
| `feishu_monitor_bot.py` | 飞书监控机器人 |
| `telegram_monitor_bot.py` | Telegram 监控机器人 |
| `feishu_config_template.py` | 飞书配置模板 |
| `requirements_feishu.txt` | 飞书依赖 |
| `requirements_telegram.txt` | Telegram 依赖 |

---

### launch/ — ROS2 Launch

独立的 Python launch 文件。新项目优先使用 `lingtu.py` profile 启动。

| 文件 | 用途 |
|------|------|
| `nav_autonomy_launch.py` | 自主导航 (terrain + local planner + path follower) |
| `nav_planning_launch.py` | 全局规划模块 |

---

### proto/ — Protobuf 代码生成

从 `shared/proto/` 生成 Python/Dart gRPC 代码。

| 文件 | 用途 |
|------|------|
| `proto_gen.sh` | Linux 生成 |
| `proto_gen.ps1` | Windows 生成 |

---

### manager/ — Web 管理端

轻量 Web 管理服务，由 `lingtu-manager.service` 管理。

| 文件 | 用途 |
|------|------|
| `manager.py` | 管理端主程序 |

---

### legacy/ — 遗留脚本

旧版按服务拆分的 shell 启动脚本。已被 `lingtu.py` + `lingtu.sh` 替代，保留供参考。

包含：`env.sh`, `nav-slam.sh`, `nav-lidar.sh`, `nav-driver.sh`, `nav-planning.sh`, `nav-autonomy.sh`, `nav-semantic.sh`, `nav-grpc.sh`, `nav-monitor.sh`, `ota-daemon.sh`

---

## 根目录脚本

`scripts/` 目录直接下的独立脚本：

### CLI 挂钩 (被 `lingtu.py` 调用)

| 脚本 | 触发方式 | 用途 |
|------|---------|------|
| `doctor.py` | `lingtu doctor` | 环境/依赖自检 |
| `rerun_live.py` | `lingtu rerun` | Rerun 3D 实时可视化 |

### 统一操作

| 脚本 | 用途 |
|------|------|
| `lingtu.sh` | Shell 统一入口 (`map` / `save` / `nav` / `status`) |
| `deploy_s100p.sh` | S100P 一键部署 |
| `build_nav_core.sh` | C++ nav_core 独立构建 |
| `clone_semantic_deps.sh` | 克隆语义导航第三方依赖 |
| `demo_semantic.sh` | 语义导航 Demo |

### 调试与检测

| 脚本 | 用途 |
|------|------|
| `live_detect.py` | 实时目标检测 (30fps Web 预览) |
| `live_track.py` | 实时追踪 + CLIP 选人 |
| `run_rerun_mapping.py` | 建图 + Rerun 流程 |

### 测试脚本

| 脚本 | 用途 |
|------|------|
| `test_mapping.py` | 建图流程测试 |
| `test_nav_planning.py` | 导航规划测试 |
| `test_mcp_full.py` | MCP Server 全量测试 |
| `test_s100p_start.py` | S100P 启动验证 |
| `test_3fixes.py` | 三项修复验证 |
| `test_gateway.py` | Gateway 隔离测试 |
