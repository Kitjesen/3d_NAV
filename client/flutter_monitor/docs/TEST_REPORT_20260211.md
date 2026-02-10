# 测试报告 — 服务编排 & 状态机 & Bug 修复

**日期**: 2026-02-11  
**测试环境**: robot (aarch64, ROS2 Humble, rmw_fastrtps_cpp)

---

## 1. Bug 修复

### 1.1 `global_planner.py` 启动失败 — `librcl_action.so` 找不到

| 项目 | 内容 |
|------|------|
| **现象** | `nav-planning` 服务启动后 `global_planner.py` 立即崩溃: `ImportError: librcl_action.so: cannot open shared object file` |
| **根因** | `nav_planning_launch.py` 使用 `env={'PYTHONPATH': ...}` 启动节点, ROS2 launch 中 `env=` **替换整个进程环境**, 导致 `LD_LIBRARY_PATH` 丢失 |
| **修复** | 改为 `additional_env={'PYTHONPATH': ...}`, 只追加 PYTHONPATH, 保留 LD_LIBRARY_PATH |
| **文件** | `scripts/launch/nav_planning_launch.py` |

### 1.2 Tomogram 文件名解析错误

| 项目 | 内容 |
|------|------|
| **现象** | `global_planner.py` 报 `FileNotFoundError: spiral0.pickle`, 但实际文件是 `spiral0.3_2.pickle` |
| **根因** | `planner_wrapper.py` 用 `os.path.splitext('spiral0.3_2')` 提取基名, Python 把 `.3_2` 当扩展名, base 变成 `spiral0` |
| **修复** | 只剥离已知扩展名 `.pickle` / `.pcd`, 不使用 `splitext` |
| **文件** | `src/global_planning/PCT_planner/planner/scripts/planner_wrapper.py` |

### 1.3 ServiceOrchestrator `reset-failed` 缺失

| 项目 | 内容 |
|------|------|
| **现象** | 服务从 `failed` 状态重新启动时失败 |
| **修复** | 启动前检测 `failed` 状态并自动 `reset-failed`, 同时更新 sudoers 权限 |
| **文件** | `src/remote_monitoring/src/core/service_orchestrator.cpp`, `scripts/install_services.sh` |

---

## 2. 测试结果

### 2.1 Bash 集成测试 (`scripts/test_services.sh`)

**运行**: `bash scripts/test_services.sh`

```
30 PASS / 0 FAIL / 0 SKIP
```

| 测试组 | 测试项 | 结果 | 备注 |
|--------|--------|------|------|
| **前置检查** | sudoers 权限 | PASS | |
| | ROS2 环境 | PASS | |
| | grpcurl 可用 | PASS | v1.8.9 |
| **TEST 1: nav-lidar** | 启动 | PASS | |
| | 停止 | PASS | inactive |
| **TEST 2: nav-slam** | 启动 (自动拉起 lidar) | PASS | lidar 联动启动 |
| | /Odometry 话题 | PASS | 9.954 Hz |
| | 停止后 lidar 保留 | PASS | |
| **TEST 3: nav-autonomy** | 启动 (拉起 lidar+slam) | PASS | 全依赖链 |
| | /terrain_map 话题 | PASS | 16.8 Hz |
| | 停止 | PASS | |
| **TEST 4: nav-planning** | 启动 | PASS | |
| | librcl_action 修复验证 | PASS | 无错误 |
| | tomogram 加载 | PASS | 150163 pts |
| **TEST 5: gRPC 编排** | nav-grpc 启动 | PASS | |
| | SetMode MAPPING → lidar+slam | PASS | 服务自动启动 |
| | SetMode IDLE → 服务保留 | PASS | |
| | TELEOP 无 lease 拒绝 | PASS | MODE_CONFLICT |
| | AcquireLease | PASS | |
| | SetMode TELEOP → autonomy 启动 | PASS | 服务自动启动 |
| | TELEOP→IDLE → autonomy 停止 | PASS | |
| | ReleaseLease | PASS | |

### 2.2 Dart 状态机测试 (`tools/test/control/mode_test.dart`)

**运行**: `dart run tools/test/control/mode_test.dart 127.0.0.1 50051`

```
12 PASS / 0 FAIL
```

| 测试 | 场景 | 结果 | 返回 |
|------|------|------|------|
| T1 | IDLE → MAPPING | PASS | ROBOT_MODE_MAPPING |
| T2 | MAPPING → IDLE | PASS | ROBOT_MODE_IDLE |
| T3 | IDLE → TELEOP (无 lease) | PASS | MODE_CONFLICT "TELEOP requires valid lease" |
| T4 | AcquireLease | PASS | token=lease_xx... ttl=30s |
| T5 | IDLE → TELEOP (有 lease) | PASS | ROBOT_MODE_TELEOP |
| T6 | TELEOP → IDLE | PASS | ROBOT_MODE_IDLE |
| T7 | IDLE → AUTONOMOUS | PASS | ROBOT_MODE_AUTONOMOUS |
| T8 | MAPPING → TELEOP (非法) | PASS | MODE_CONFLICT "TELEOP only reachable from IDLE or AUTONOMOUS" |
| T9 | EmergencyStop | PASS | stopped=true |
| T10 | ESTOP → MAPPING (阻断) | PASS | MODE_CONFLICT "ESTOP can only transition to IDLE via ClearEmergencyStop()" |
| T11 | ESTOP → IDLE (恢复) | PASS | ROBOT_MODE_IDLE |
| T12 | ReleaseLease | PASS | ERROR_CODE_OK |

### 2.3 Dart 编排测试 (`tools/test/control/orchestration_test.dart`)

**运行**: `dart run tools/test/control/orchestration_test.dart 127.0.0.1 50051`

```
5 PASS / 0 FAIL
```

| 测试 | 场景 | 期望服务状态 | 结果 |
|------|------|-------------|------|
| T1 | IDLE→MAPPING | lidar=active, slam=active | PASS |
| T2 | MAPPING→IDLE | lidar=active, slam=active (保留) | PASS |
| T3 | AcquireLease + TELEOP | lidar+slam+autonomy=active | PASS |
| T4 | TELEOP→IDLE | autonomy=inactive, lidar+slam=active | PASS |
| T5 | ReleaseLease | - | PASS |

---

## 3. 状态机定义

### 3.1 源码位置

| 文件 | 职责 |
|------|------|
| `src/remote_monitoring/include/remote_monitoring/core/mode_manager.hpp` | 状态/守卫/接口定义 |
| `src/remote_monitoring/src/core/mode_manager.cpp` | 转换函数 `CheckTransition()` + 进入/退出动作 |
| `src/remote_monitoring/src/core/service_orchestrator.cpp` | 模式 → systemd 服务映射 |
| `src/remote_monitoring/src/services/control_service.cpp` | gRPC `SetMode` 入口 |

### 3.2 形式化定义

```
Mealy 机 M = (S, Σ, Δ, δ, λ, s₀)

S  = {IDLE, MANUAL, TELEOP, AUTONOMOUS, MAPPING, ESTOP}
s₀ = IDLE

转换函数 δ(from, to) + 守卫:
```

### 3.3 完整转换矩阵

| from \ to | IDLE | MANUAL | TELEOP | AUTONOMOUS | MAPPING | ESTOP |
|-----------|------|--------|--------|------------|---------|-------|
| **IDLE** | — | OK | 需 lease | 需 TF+定位 | OK | EmergencyStop() |
| **MANUAL** | OK | — | ✗ | ✗ | ✗ | EmergencyStop() |
| **TELEOP** | OK | ✗ | — | 需 TF+定位 | ✗ | EmergencyStop() |
| **AUTONOMOUS** | OK | ✗ | 需 lease | — | ✗ | EmergencyStop() |
| **MAPPING** | OK | ✗ | ✗ | ✗ | — | EmergencyStop() |
| **ESTOP** | SwitchMode OK | ✗ | ✗ | ✗ | ✗ | — |

### 3.4 模式 → 服务编排

| 模式 | 自动启动的 systemd 服务 |
|------|------------------------|
| IDLE | nav-lidar, nav-slam |
| MAPPING | nav-lidar, nav-slam |
| MANUAL | nav-lidar, nav-slam |
| TELEOP | nav-lidar, nav-slam, nav-autonomy |
| AUTONOMOUS | nav-lidar, nav-slam, nav-autonomy, nav-planning |
| ESTOP | 不变 (仅发停车指令) |

---

## 4. 测试文件清单

| 文件 | 类型 | 用途 |
|------|------|------|
| `scripts/test_services.sh` | Bash | systemd 服务启停 + gRPC 编排端到端测试 |
| `client/flutter_monitor/tools/test/control/mode_test.dart` | Dart | 状态机转换矩阵完整测试 (12 cases) |
| `client/flutter_monitor/tools/test/control/orchestration_test.dart` | Dart | SetMode 后 systemd 服务状态验证 |

---

## 5. 已知问题 & 待完善

### 5.1 已确认正常

- [x] 所有 6 个 systemd 服务启停正常
- [x] 服务依赖自动拉起 (slam→lidar, autonomy→slam→lidar)
- [x] gRPC SetMode 编排正确映射模式到服务
- [x] Lease 守卫正确拒绝无权操作
- [x] EmergencyStop 双路径停车 (lock-free atomic + /stop 话题)
- [x] HealthMonitor IDLE 模式下不触发误报 ESTOP
- [x] global_planner.py 正常加载 tomogram

### 5.2 待完善 (建议优先级)

| 优先级 | 项目 | 说明 |
|--------|------|------|
| **高** | TF/定位守卫实际接入 | 当前 `tf_ok`/`localization_valid` 守卫在无雷达时未配置, AUTONOMOUS 可无条件进入 |
| **高** | AUTONOMOUS 模式端到端测试 | 需要雷达 + 地图 + Relocalize 完整链路 |
| **中** | ClearEmergencyStop gRPC 接口 | `ModeManager.ClearEmergencyStop()` 已实现, 但 proto 中无独立 RPC, 当前通过 SetMode(IDLE) 恢复 |
| **中** | Flutter TELEOP 按钮 | 主控制界面缺少 TELEOP 模式按钮 (仅 BLE 控制界面有) |
| **低** | 服务启动失败升级策略 | ServiceOrchestrator 启动失败只记日志, 无重试/通知 |
| **低** | HealthMonitor 注释更新 | 头部注释写 "odom_hz > 50", 实际阈值为 5.0 Hz |
