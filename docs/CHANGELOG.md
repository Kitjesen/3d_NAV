# Changelog

所有重大变更、升级记录和修复日志。

> 格式参考 [Keep a Changelog](https://keepachangelog.com/)

---

## 路线图

```
2026 Q1 (当前)
├── ✅ 数学优化 (SLAM/规划核心改进)
├── ✅ 坐标系统一修复
├── ✅ 四层安全架构
├── ✅ 远程监控 + Flutter App (MapPilot)
├── ✅ gRPC Relocalize / SaveMap 实现
├── ✅ 状态机守卫注入
├── ✅ 一键启动 Launch 文件
├── ✅ TaskManager 任务管理
├── ✅ 断联自动降级
├── ✅ terrain_map_ext 接入
├── ✅ 定位质量监控
├── ✅ 近场急停
├── ✅ Proto 健康/围栏扩展
├── ✅ OTA 更新系统 (直接下载/断点续传/回滚)
├── ✅ OTA v2: Ed25519 签名 + 安全等级 + 依赖管理 + 事务日志
├── ✅ 遥控避障: SafetyGate 近场避障 + /cmd_vel 仲裁
├── 🔲 colcon 构建验证
├── 🔲 Proto Dart 代码重新生成
└── 🔲 Flutter App 健康/围栏/巡检 UI

2026 Q2
├── 🔲 TaskManager JSON 解析升级
├── 🔲 断联降级可配置化
├── 🔲 pct_adapters 到达事件
├── 🔲 rosbag 集成
└── 🔲 定位质量阈值标定

2026 Q3+
├── 🔲 BehaviorTree 替代状态机
├── 🔲 多机器人协调
└── 🔲 仿真测试框架
```

---

## [v1.0.0] - 2026-02-08

首个稳定版本。涵盖从感知到控制的完整导航链路 + 远程监控 + OTA 更新。

### 新增

#### OTA 更新系统
- `DownloadFromUrl` — 机器人直接从 GitHub 下载，免手机中转
- `UploadFile` 断点续传 — WiFi 中断后可从断点继续
- `CheckUpdateReadiness` — 安装前预检查（磁盘/电量/硬件兼容/网络）
- `ApplyUpdate` — SHA256 校验 + 备份 + 安装 + manifest 管理
- `GetInstalledVersions` / `Rollback` — 版本查询与一键回滚
- `manifest.json` 格式定义与 `generate_manifest.py` 自动生成工具

#### OTA v2 产品级增强
- **Ed25519 签名链** — manifest 签名验证，防伪造；`generate_manifest.py --signing-key` 支持
- **安全等级分级** — HOT (地图/配置) / WARM (模型) / COLD (固件/MCU: sit → disable → 维护态)
- **原子安装 + 事务日志** — 写 `txn_{name}.json` → 安装 → 成功清理 / 崩溃自动回滚
- **依赖管理** — `dependencies` 字段表达制品间版本约束，CheckUpdateReadiness 自动检查
- **系统边界 (owner_module)** — brain / navigation / config_service / system / mcu 各负其责
- **用户体验规范** — 更新提示、COLD 确认流程、断电恢复、极端场景兜底
- 详见 [OTA_GUIDE.md](OTA_GUIDE.md)

#### 遥控避障 + /cmd_vel 仲裁
- **SafetyGate 近场避障** — 订阅 `/terrain_map` (odom 坐标系)，实时转 body 坐标系检测前方障碍
  - `obstacle_stop`：前方 < 0.8m 有超高障碍 → 线速度归零
  - `obstacle_slow`：前方 0.8~2.0m 有障碍 → 线性减速 `max(0.2, dist/2.0)`
  - 角速度不受影响，允许原地转向避让
- **模式门禁** — SafetyGate 仅在 TELEOP 模式下发布 `/cmd_vel`，从根本上消除与 pathFollower 的冲突
- **TELEOP 退出清除** — ModeManager.ExitState(TELEOP) 通过 SafetyGate 发零速度，清除残余 cmd_vel
- 参数：`obstacle_height_thre`, `stop_distance`, `slow_distance`, `vehicle_width`, `vehicle_width_margin`
- App 端通过 `TeleopFeedback.limit_reasons` 自动收到 "obstacle_stop" / "obstacle_slow" 原因

#### TaskManager 任务管理
- 航点队列：接收 N 个目标按序下发 `/way_point`
- 到达检测：订阅 `/Odometry`，欧氏距离 ≤ `arrival_radius` 判定到达
- 循环巡检：`INSPECTION` 类型自动 `loop=true`
- 状态机：IDLE → RUNNING → PAUSED → COMPLETED / FAILED / CANCELLED
- 进度回调 → EventBuffer → gRPC StreamEvents → App

#### 一键启动 Launch 文件
- `navigation_bringup.launch.py` — 建图模式
- `navigation_run.launch.py` — 运行模式（定位 + 自主导航）

#### 定位质量监控
- Localizer 发布 ICP fitness score → `/localization_quality`
- HealthMonitor 纳入判定：< 0.1 OK / < 0.3 DEGRADED / ≥ 0.3 CRITICAL

#### 近场急停
- local_planner 检测前方 0.5m 内障碍物 → 直接发布 `/stop=2`
- 带状态记忆，避免重复发布

#### Proto 健康/围栏扩展
- `telemetry.proto` 新增 `HealthStatus`、`GeofenceStatus` 消息
- `SlowState` 新增 `health` (field 8) 和 `geofence` (field 9)

### 改进

#### 系统安全架构升级 (2026-02-07)

8 项关键改进，从 "能用" 到 "敢户外用"：

| 项目 | 说明 |
|------|------|
| gRPC Relocalize/SaveMap | 空壳 → 实际调用 ROS 2 Service |
| 状态机守卫注入 | 7 个守卫从纸面变为代码 |
| 断联自动降级 | < 30s 正常 / 30s-5min 减速 50% / > 5min 停车 |
| terrain_map_ext 接入 | local_planner 合并连通性信息，避免死胡同 |

#### 四层解耦安全架构 (2026-02-06)

```
Layer 4: HealthMonitor   — 子系统健康聚合 + 自动降级
Layer 3: ModeManager     — 形式化状态机 (转换守卫矩阵)
Layer 2: GeofenceMonitor — 围栏越界检测 (射线法 + 三级预警)
Layer 1: Driver Watchdog — 底盘自保护 (200ms cmd_vel 超时)
```

核心原则：任何一层崩溃不影响其他层。4 条独立停车路径。

#### 数学优化 (2026-02-03)

| 改动 | 文件 | 收益 |
|------|------|------|
| 平面估计除零保护 | `commons.cpp` | 消除 NaN 传播 |
| IESKF `.inverse()` → `.ldlt().solve()` | `ieskf.cpp` | 数值稳定 |
| Jacobian Bug (`t_wi` → `t_il`) | `lidar_processor.cpp` | 修正偏导错误 |
| 删除重复 `transformPointCloud` | `lidar_processor.cpp` | 修复 T² 变换 |
| 缓存 `R_wi`/`R_wl` 到循环外 | `lidar_processor.cpp` | 省 N 次矩阵乘法 |
| 三步欧拉旋转 → 预计算矩阵 | `terrainAnalysis.cpp` | 每点 9 vs 18 乘法 |
| `sqrt(sqrt())` NaN 防御 | `localPlanner.cpp` | 负参数不再 NaN |
| `57.3` → `180.0/M_PI` | `lidar_processor.cpp` | 精确常量 |

### 修复

#### 坐标系统一修复 (2026-02-03)

**问题**: terrain_analysis / terrain_analysis_ext / local_planner 坐标系混用 — body 系点云与 odom 系位姿直接相减，输出声称 `map` 系实际不是。

**修复**:

| 模块 | 修复前 | 修复后 |
|------|--------|--------|
| terrain_analysis | 输入 `/cloud_registered` (body), 输出 `"map"` | 输入 `/cloud_map` (odom), 输出 `"odom"` |
| terrain_analysis_ext | 同上 | 同上 |
| local_planner | 输入 body+odom 混合, 输出 `"vehicle"` | 输入 odom, 转换后输出 `"body"` |
| pathFollower | 输出 `"vehicle"` | 输出 `"body"` |
| TF 树 | `sensor` → `vehicle` (不连续) | `body` → `lidar` → `camera` (完整) |

修复后 TF 树：`map → odom → body → {lidar, camera}`

**验证方法**:
```bash
ros2 topic echo /terrain_map --field header.frame_id --once  # 期望: odom
ros2 topic echo /path --field header.frame_id --once          # 期望: body
ros2 run tf2_tools view_frames                                 # 期望: 完整连续
```

---

## 待办事项

### 高优先级
- [ ] colcon 完整编译验证
- [ ] Proto 重新生成 Dart 代码 (`scripts/proto_gen.sh`)
- [ ] Flutter App 健康/围栏/巡检 UI

### 中优先级
- [ ] pct_adapters 到达事件（更精准的航点切换）
- [ ] TaskManager JSON 解析升级（nlohmann/json）
- [ ] 断联降级阈值可配置化
- [ ] 近场急停距离参数化
- [ ] 定位质量阈值实际标定

### 低优先级
- [ ] rosbag 集成（gRPC 触发录制）
- [ ] BehaviorTree.CPP 替代 ModeManager
- [ ] localization_valid 守卫结合 ICP score
- [ ] 多机器人协调
- [ ] 仿真测试框架 (Gazebo/Isaac Sim)

---

## 文件变更汇总

### OTA 更新 (2026-02-08)

| 文件 | 变更 |
|------|------|
| `robot_proto/proto/data.proto` | OTA RPCs + 消息; **v2: +OtaSafetyLevel, +ArtifactDependency, +OtaTransactionLog** |
| `remote_monitoring/src/services/data_service.cpp` | OTA 实现; **v2: +安全等级检查, +依赖检查, +事务日志** |
| `remote_monitoring/include/.../data_service.hpp` | OTA 声明 |
| `remote_monitoring/CMakeLists.txt` | +OpenSSL 依赖 |
| `remote_monitoring/config/grpc_gateway.yaml` | +OTA 参数 |
| `client/flutter_monitor/lib/core/grpc/robot_client.dart` | OTA 客户端 |
| `client/flutter_monitor/lib/features/settings/cloud_ota_service.dart` | manifest 解析 |
| `docs/OTA_GUIDE.md` | **v2 重写: 产品级规范 (签名/安全等级/原子安装/依赖/UX)** |
| `scripts/ota/manifest_template.json` | **v2: +signature, +dependencies, +safety_level** |
| `scripts/ota/generate_manifest.py` | **v2: +Ed25519 签名, +密钥生成** |

### 系统升级 (2026-02-07)

| 文件 | 变更 |
|------|------|
| `launch/navigation_bringup.launch.py` | 新增：建图模式启动 |
| `launch/navigation_run.launch.py` | 新增：运行模式启动 |
| `remote_monitoring/src/core/task_manager.{hpp,cpp}` | 新增：任务管理器 |
| `remote_monitoring/src/services/system_service.{hpp,cpp}` | Relocalize/SaveMap + 心跳 |
| `remote_monitoring/src/services/control_service.{hpp,cpp}` | StartTask/CancelTask |
| `remote_monitoring/src/grpc_gateway.{hpp,cpp}` | 守卫注入 + 断联降级 |
| `remote_monitoring/src/core/health_monitor.{hpp,cpp}` | +定位质量 |
| `remote_monitoring/src/status_aggregator.{hpp,cpp}` | +健康/围栏 |
| `robot_proto/proto/telemetry.proto` | +HealthStatus/GeofenceStatus |
| `base_autonomy/local_planner/src/localPlanner.cpp` | +terrain_map_ext + 近场急停 |
| `slam/localizer/src/localizers/icp_localizer.{h,cpp}` | +fitness score |
| `slam/localizer/src/localizer_node.cpp` | +/localization_quality |

### 安全架构 (2026-02-06)

| 文件 | 变更 |
|------|------|
| `remote_monitoring/src/core/geofence_monitor.{hpp,cpp}` | 新增：围栏监控 |
| `remote_monitoring/src/core/health_monitor.{hpp,cpp}` | 新增：健康监控 |
| `remote_monitoring/src/core/mode_manager.{hpp,cpp}` | 重写：形式化状态机 |
| `drivers/robot_driver/driver_node.py` | 重写：独立看门狗 |
| `base_autonomy/local_planner/src/pathFollower.cpp` | /stop max 优先级 |

### 数学优化 + 坐标修复 (2026-02-03)

| 文件 | 变更 |
|------|------|
| `slam/fastlio2/src/map_builder/commons.cpp` | 除零保护 |
| `slam/fastlio2/src/map_builder/ieskf.cpp` | LDLT 分解 |
| `slam/fastlio2/src/map_builder/lidar_processor.cpp` | Jacobian + 缓存 + 常量 |
| `base_autonomy/terrain_analysis/src/terrainAnalysis.cpp` | 坐标修复 + 旋转优化 |
| `base_autonomy/terrain_analysis_ext/src/terrainAnalysisExt.cpp` | 坐标修复 |
| `base_autonomy/local_planner/src/localPlanner.cpp` | 坐标修复 + NaN 防御 |
| `base_autonomy/local_planner/src/pathFollower.cpp` | frame_id 修复 |
| `base_autonomy/local_planner/launch/local_planner.launch` | TF 发布器修正 |

---

*最后更新: 2026-02-08*
