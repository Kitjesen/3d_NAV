# 服务编排系统 — systemd + gRPC 自动启停

> 日期: 2026-02-11
> 范围: 机器人端 (systemd 服务 + gRPC Gateway + OTA Daemon)

## 1. 概述

将导航系统的各子系统拆分为独立的 **systemd 服务**，通过 gRPC `ControlService.SetMode()` 实现 **App 一键切换模式，自动启停对应服务**。

### 之前

```
手动 SSH → 打开多个终端 → 分别 ros2 launch ...
无法从 App 控制服务启停
```

### 之后

```
App SetMode(MAPPING)
  → gRPC Gateway → ServiceOrchestrator
    → systemctl start nav-lidar
    → systemctl start nav-slam
    → (autonomy/planning 自动停止)
```

---

## 2. 服务清单

| 服务 | 包含组件 | 依赖 | 端口 |
|------|---------|------|------|
| `nav-lidar.service` | Livox MID360 驱动 | nav-lidar-network | - |
| `nav-slam.service` | Fast-LIO2 SLAM | nav-lidar | - |
| `nav-autonomy.service` | terrain_analysis + terrain_analysis_ext + local_planner + pathFollower | nav-slam | - |
| `nav-planning.service` | Localizer + PCT Planner + PCT Adapter | nav-slam | - |
| `nav-grpc.service` | gRPC Gateway (远程监控/控制) | network | 50051 |
| `ota-daemon.service` | OTA Daemon (文件/更新管理) | network | 50052 |

### 开机自启

- `nav-grpc.service` — **已启用** (App 入口必须在线)
- `ota-daemon.service` — **已启用** (文件管理必须在线)
- 其余服务 — **按需启动** (由 App 通过 SetMode 控制)

---

## 3. 模式 → 服务映射

| App 模式 | RobotMode 枚举 | 启动的服务 | 停止的服务 |
|----------|---------------|-----------|-----------|
| 空闲 | `IDLE (1)` | lidar, slam | autonomy, planning |
| 手动 | `MANUAL (2)` | lidar, slam | autonomy, planning |
| 遥操作 | `TELEOP (3)` | lidar, slam, autonomy | planning |
| 自主导航 | `AUTONOMOUS (4)` | lidar, slam, autonomy, planning | - |
| 建图 | `MAPPING (5)` | lidar, slam | autonomy, planning |
| 急停 | `ESTOP (6)` | (不改变服务) | (发 /stop 指令) |

### 模式转换守卫

- `→ AUTONOMOUS`: 需要 TF 链完整 (map→odom→body) + 定位有效
- `→ TELEOP`: 需要持有有效 Lease
- `→ MAPPING`: 仅需从 IDLE 进入
- `ESTOP → *`: 必须通过 `ClearEmergencyStop()` 先回到 IDLE

---

## 4. 调用方式

### 4.1 通过 gRPC (App 端)

```dart
// Flutter 客户端
final response = await controlService.setMode(
  SetModeRequest(
    base: RequestBase(requestId: uuid()),
    mode: RobotMode.ROBOT_MODE_MAPPING,
  ),
);
// response.currentMode == ROBOT_MODE_MAPPING
// 后台自动: lidar + slam 已启动
```

### 4.2 通过 grpcurl (调试)

```bash
# 切到建图模式
grpcurl -plaintext -d '{"base":{"request_id":"001"},"mode":5}' \
  192.168.66.190:50051 robot.v1.ControlService/SetMode

# 切到自主导航
grpcurl -plaintext -d '{"base":{"request_id":"002"},"mode":4}' \
  192.168.66.190:50051 robot.v1.ControlService/SetMode

# 回到空闲
grpcurl -plaintext -d '{"base":{"request_id":"003"},"mode":1}' \
  192.168.66.190:50051 robot.v1.ControlService/SetMode
```

### 4.3 通过 OTA Daemon 直接管理 (高级)

```bash
# 查看所有服务状态
grpcurl -plaintext 192.168.66.190:50052 \
  robot.v1.OtaService/GetDeviceInfo

# 手动启停单个服务
grpcurl -plaintext -d '{"service_name":"nav-slam.service","action":1}' \
  192.168.66.190:50052 robot.v1.OtaService/ManageService
# action: 1=START, 2=STOP, 3=RESTART, 4=STATUS
```

### 4.4 通过 systemctl (SSH)

```bash
sudo systemctl start nav-lidar    # 启动 LiDAR
sudo systemctl start nav-slam     # 启动 SLAM
systemctl status nav-slam         # 查看状态
journalctl -u nav-slam -f         # 实时日志
```

---

## 5. 文件结构

```
navigation/
├── scripts/
│   ├── services/                  # 服务启动脚本
│   │   ├── env.sh                 # 公共环境 (ROS2 setup)
│   │   ├── nav-lidar.sh
│   │   ├── nav-slam.sh
│   │   ├── nav-autonomy.sh
│   │   ├── nav-planning.sh
│   │   ├── nav-grpc.sh
│   │   └── ota-daemon.sh
│   ├── launch/                    # 多节点 launch 文件
│   │   ├── nav_autonomy_launch.py # 地形分析+局部规划+路径跟踪
│   │   └── nav_planning_launch.py # Localizer+PCT+Adapter
│   └── install_services.sh        # 一键安装脚本
├── systemd/                       # systemd 单元文件
│   ├── nav-lidar.service
│   ├── nav-slam.service
│   ├── nav-autonomy.service
│   ├── nav-planning.service
│   ├── nav-grpc.service
│   └── ota-daemon.service
└── src/remote_monitoring/
    └── src/core/
        └── service_orchestrator.cpp  # 核心编排逻辑
```

---

## 6. 安装 / 更新

```bash
# 首次安装或更新服务文件后执行:
cd /home/sunrise/data/SLAM/navigation
sudo bash scripts/install_services.sh
```

安装脚本会:
1. 设置脚本可执行权限
2. 复制 `.service` 文件到 `/etc/systemd/system/`
3. 配置 sudoers 规则 (sunrise 用户无密码管理 nav-* 服务)
4. 创建 `/etc/nav/planning.env` 配置模板
5. 重载 systemd daemon

### 规划参数配置

编辑 `/etc/nav/planning.env` 来设置 nav-planning 的默认地图和初始位姿:

```bash
NAV_MAP_PATH=/home/sunrise/data/SLAM/navigation/maps/my_map
NAV_INIT_X=1.5
NAV_INIT_Y=0.0
NAV_INIT_Z=0.0
NAV_INIT_YAW=0.0
```

---

## 7. 代码改动摘要

### 新增文件

| 文件 | 说明 |
|------|------|
| `scripts/services/*.sh` | 各服务的 ROS2 环境包装脚本 |
| `scripts/launch/nav_autonomy_launch.py` | autonomy 子系统 launch 文件 |
| `scripts/launch/nav_planning_launch.py` | planning 子系统 launch 文件 |
| `scripts/install_services.sh` | 服务安装脚本 |
| `systemd/*.service` | 6 个 systemd 服务单元 |
| `src/remote_monitoring/.../service_orchestrator.hpp/cpp` | 服务编排器 |

### 修改文件

| 文件 | 改动 |
|------|------|
| `src/remote_monitoring/src/grpc_gateway.cpp` | 创建并注入 ServiceOrchestrator |
| `src/remote_monitoring/.../mode_manager.hpp/cpp` | 添加 ServiceOrchestrator 成员,模式切换时调用 |
| `src/remote_monitoring/.../health_monitor.hpp/cpp` | IDLE 模式下不触发 FAULT→ESTOP |
| `src/remote_monitoring/CMakeLists.txt` | 添加 service_orchestrator.cpp 源文件 |
| `src/ota_daemon/config/ota_daemon.yaml` | 注册 6 个 managed_services |
| `src/ota_daemon/src/utils.cpp` | ManageService 使用 sudo systemctl |

---

## 8. 故障排查

### 服务启动失败

```bash
# 查看详细日志
journalctl -u nav-slam.service --no-pager -n 50

# 常见原因:
# 1. ROS2 环境问题 → 检查 scripts/services/env.sh
# 2. 依赖服务未启动 → systemctl status nav-lidar
# 3. 端口冲突 → ss -tlnp | grep 50051
```

### SetMode 返回 MODE_CONFLICT

```bash
# 查看当前模式
grpcurl -plaintext -d '{}' 192.168.66.190:50051 \
  robot.v1.SystemService/GetRobotInfo

# 如果卡在 ESTOP，需要先清除:
# (目前需要重启 nav-grpc 服务来重置状态)
sudo systemctl restart nav-grpc
```

### 权限问题

```bash
# 验证 sudoers 规则
sudo -l | grep systemctl
# 应显示: NOPASSWD: /bin/systemctl start nav-*.service 等

# 如果缺失，重新安装:
sudo bash scripts/install_services.sh
```
