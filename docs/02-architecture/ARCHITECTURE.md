# System Architecture — Multi-Board Robot Control & Navigation

> 本文档描述双板硬件架构、通信协议、数据流、安全体系和模式切换。
> 导航模块详情（SLAM、地形分析、规划器话题/参数）请参见 [AGENTS.md](../AGENTS.md)。

## Overview

The system consists of **4 independent entities** across **2 hardware boards** and **2 remote clients**:

```
                         ┌─────────────────────┐
                         │    Flutter App       │
                         │    (手机/平板)        │
                         └──┬──────┬────────┬───┘
                            │      │        │
              ┌─────────────┘      │        └─────────────────┐
              │                    │                          │
     gRPC :50051           gRPC :13145                      BLE
     robot::v1 协议        han_dog CMS 协议               基础控制
     (导航+监控+任务)      (直连行走控制)               (紧急停止/WiFi配置)
              │                    │                          │
              ▼                    │                          │
    ┌───────────────────┐         │                          │
    │   Nav Board       │         │                          │
    │   (导航板)         │         │                          │
    │ ───────────────── │         │                          │
    │ SLAM (Fast-LIO2)  │         │                          │
    │ Terrain Analysis   │         │                          │
    │ Local Planner      │         │                          │
    │ PCT Path Adapter   │         │                          │
    │ GrpcGateway :50051 │         │                          │
    │   └ SystemService  │         │                          │
    │   └ ControlService │         │                          │
    │   └ TelemetryService│        │                          │
    │   └ DataService    │         │                          │
    │ han_dog_bridge ────┼─gRPC────┤                          │
    │   (CMS client)     │  :13145 │                          │
    └───────────────────┘         │                          │
                                   ▼                          ▼
                         ┌──────────────────────────────────────┐
                         │         Dog Board                    │
                         │         (机器狗控制板)                 │
                         │ ──────────────────────────────────── │
                         │  CMS gRPC Server :13145              │
                         │  RL Policy Inference (~20ms/cycle)   │
                         │  Motor PD Controller (16 joints)     │
                         │  IMU / Joint Sensor Streaming        │
                         │  BLE Service (基础控制)               │
                         │                                      │
                         │  ┌─────────────── Arbiter ──────┐   │
                         │  │  Priority 1: YUNZHUO RC      │   │
                         │  │  Priority 2: gRPC clients    │   │
                         │  │  (Walk/StandUp/SitDown 经仲裁)│   │
                         │  └──────────────────────────────┘   │
                         │            ▲                         │
                         └────────────┼─────────────────────────┘
                                      │
                            ┌─────────┴─────────┐
                            │   YUNZHUO 遥控器   │
                            │  (SBUS/PPM 直连)   │
                            │  最高优先级, 硬件级  │
                            └───────────────────┘
```

## Hardware Boards

### Dog Board (机器狗控制板)

**职责**: 底层运动控制 — RL 推理、电机 PD 控制、传感器读取

| Component | Description |
|-----------|-------------|
| CMS gRPC Server | Port `13145`, `han_dog` protocol |
| RL Policy | ~20ms inference cycle, outputs 16 joint targets |
| PD Controller | Position + velocity control with kp/kd gains |
| IMU | Quaternion orientation + gyroscope (body frame) |
| 16 Joints | 4 legs x 4 joints (hip, thigh, calf, foot) |
| BLE Service | Basic commands: estop, mode switch, WiFi config |
| Arbiter | Priority arbitration: RC > gRPC |

**CMS gRPC Interface** (`han_dog.Cms`):

| RPC | Type | 仲裁 | Description |
|-----|------|------|-------------|
| `Enable` / `Disable` | Unary | 不经仲裁 | 电机使能/禁用 (硬件级) |
| `Walk(Vector3)` | Unary | 经仲裁 | 行走 (x,y,z 归一化到 [-1,1]) |
| `StandUp` / `SitDown` | Unary | 经仲裁 | 站立/坐下 |
| `ListenImu` | Server stream | 不限 | 实时 IMU 流 |
| `ListenJoint` | Server stream | 不限 | 实时关节数据流 |
| `ListenHistory` | Server stream | 不限 | RL 推理历史流 (~50Hz) |
| `GetParams` | Unary | 不限 | 机器人模型参数 |

### Nav Board (导航板)

**职责**: 高层导航智能 — SLAM、路径规划、远程监控网关

| Component | Description |
|-----------|-------------|
| Fast-LIO2 | LiDAR-Inertial SLAM, publishes `/Odometry` |
| Localizer | Map-based relocalization |
| Terrain Analysis | Traversability analysis from pointcloud |
| Local Planner | Obstacle avoidance, path following |
| PCT Planner | Global path planning (Python, separate process) |
| GrpcGateway | Port `50051`, `robot::v1` protocol → Flutter App |
| han_dog_bridge | CMS gRPC client → Dog Board :13145 |

**GrpcGateway Interface** (`robot.v1.*`):

| Service | RPCs | Description |
|---------|------|-------------|
| SystemService | Login, Heartbeat, Relocalize, SaveMap | 系统管理 |
| ControlService | AcquireLease, StreamTeleop, SetMode, StartTask | 控制操作 |
| TelemetryService | StreamFastState (30Hz), StreamSlowState (1Hz) | 状态遥测 |
| DataService | Subscribe (camera/pointcloud/map), WebRTC | 数据流 |

## Remote Clients

### Flutter App (手机/平板)

**三种连接方式**:

| 方式 | Target | Protocol | Port | Capabilities |
|------|--------|----------|------|-------------|
| WiFi/gRPC → Nav Board | GrpcGateway | `robot::v1` | 50051 | **Full**: 导航, 建图, 遥操作, 遥测, 点云, 地图, OTA |
| WiFi/gRPC → Dog Board | CMS directly | `han_dog` | 13145 | **Basic**: Walk, StandUp/SitDown, IMU/Joint 监控 |
| BLE → Dog Board | BLE Service | Custom BLE | — | **Minimal**: 紧急停止, 模式切换, WiFi 配置 |

### YUNZHUO 遥控器

| Property | Value |
|----------|-------|
| Connection | SBUS/PPM 硬件直连 Dog Board |
| Priority | **最高** (Arbiter 中优先级 1) |
| Features | 实时行走控制, 硬件级急停 |
| Dependency | **无** (不依赖 Nav Board 或网络) |

## Data Flow Diagrams

### Flow 1: Full Navigation Mode (App → Nav Board → Dog Board)

```
Flutter App
    │ StreamTeleop (robot::v1)         ← TeleopFeedback (safety status, latency)
    ▼
GrpcGateway (ControlService)
    │ ValidateLease()
    │ SafetyGate.ProcessTeleopCommand()
    │   ├ Mode guard: 非 TELEOP 模式 → 返回零速度, 不发布 /cmd_vel
    │   ├ Speed limit (max 1.0 m/s)
    │   ├ Angular limit (max 1.0 rad/s)
    │   ├ Tilt protection (30°)
    │   ├ ★ 近场避障 (订阅 /terrain_map)
    │   │   ├ 前方 < 0.8m 有障碍 → 线速度归零 (obstacle_stop)
    │   │   ├ 前方 0.8~2.0m 有障碍 → 线性减速 (obstacle_slow)
    │   │   └ 角速度不受影响 (允许原地转向避让)
    │   └ E-stop check (最高优先级)
    ▼
/cmd_vel (TwistStamped, m/s)         ← ROS2 Topic (仅 TELEOP 模式下发布)
    │
    ▼
han_dog_bridge
    │ _twist_to_walk():
    │   Walk.x = clamp(linear.x / max_speed, -1, 1)
    │   Walk.y = clamp(linear.y / max_speed, -1, 1)
    │   Walk.z = clamp(angular.z / max_angular, -1, 1)
    ▼
CMS.Walk(Vector3)                    ← gRPC :13145
    │
    ▼
Dog Board Arbiter
    │ if (YUNZHUO RC active) → REJECT
    │ else → ACCEPT
    ▼
RL Policy → PD Controller → Motors
```

### Flow 2: Autonomous Navigation

```
Flutter App
    │ StartTask(waypoints)
    ▼
GrpcGateway (ControlService)
    │ ModeManager.SwitchMode(AUTONOMOUS)
    │   ├ Guard: TF chain valid?
    │   ├ Guard: Localization valid?
    │   └ Guard: Lease held?
    ▼
/goal_pose (PoseStamped)             ← ROS2 Topic
    │
    ▼
Local Planner
    │ Terrain analysis + obstacle avoidance
    │
    ▼
/cmd_vel (TwistStamped)              ← ROS2 Topic
    │
    ▼
han_dog_bridge → CMS.Walk()          (same as Flow 1)
```

### Flow 3: Telemetry Feedback (Dog Board → Nav Board → App)

```
Dog Board
    │ CMS.ListenImu()     → IMU quaternion + gyroscope (stream)
    │ CMS.ListenJoint()   → 16-joint positions/velocities/torques (stream)
    ▼
han_dog_bridge
    │ Publishes:
    │   /Odometry      (nav_msgs/Odometry)    → StatusAggregator → FastState
    │   /robot_state   (interface/RobotState)  → StatusAggregator → FastState
    ▼
GrpcGateway (TelemetryService)
    │ StreamFastState (30Hz): pose, velocity, RPY, joint angles
    │ StreamSlowState (1Hz):  mode, battery, health, topic rates
    ▼
Flutter App
```

### Flow 4: RC Override (Highest Priority)

```
YUNZHUO RC
    │ SBUS/PPM signal (hardware)
    ▼
Dog Board Arbiter
    │ RC active → gRPC Walk commands REJECTED
    │          → RC commands ACCEPTED
    ▼
RL Policy → PD Controller → Motors

Note: RC override 是完全透明的:
  - Nav Board 继续发送 Walk 但被拒绝 (无错误, 静默忽略)
  - 监控流 (ListenImu/ListenJoint) 不受影响, 继续输出数据
  - App 可通过 TeleopFeedback 中的 limit_reasons 检测到仲裁拒绝
```

### Flow 5: Direct Dog Control (App → Dog Board, no Nav Board)

```
Flutter App
    │ gRPC :13145 (han_dog CMS protocol)
    │ CmsStub.Walk(Vector3)
    │ CmsStub.StandUp()
    │ CmsStub.ListenImu()
    ▼
Dog Board CMS
    │ Arbiter: RC > App direct
    ▼
RL Policy → PD Controller → Motors

Use case: Nav Board 离线或不可用时的基础遥控
Limitation: 无导航, 无安全网关, 无路径规划
```

## Safety Architecture (Multi-Layer)

```
Priority  Layer              Location       Mechanism              Timeout
─────────────────────────────────────────────────────────────────────────
   1      YUNZHUO RC         Dog Board      Hardware arbiter       Instant
   2      CMS Arbiter        Dog Board      gRPC rejection         Instant
   3      han_dog_bridge     Nav Board      cmd_vel watchdog       200ms
          Watchdog
   4      SafetyGate         Nav Board      Deadman switch         300ms
                                            Speed limit (1m/s)
                                            Tilt protection (30°)
                                            ★ 近场避障 (terrain_map)
                                              stop < 0.8m / slow < 2.0m
                                            Mode guard (非 TELEOP 不发布)
                                            E-stop
   5      ModeManager        Nav Board      State machine guards   —
                                            Lease validation
                                            TELEOP 退出时发零速度清残余
   6      HealthMonitor      Nav Board      Subsystem monitoring   —
                                            → auto ESTOP on FAULT
   7      GeofenceMonitor    Nav Board      Position boundary      —
                                            → auto STOP on violation
   8      Disconnect         Nav Board      Heartbeat timeout      30s→slow
          Handling                                                  5min→stop
```

**Safety Guarantee**: Even if ALL software layers fail:
- Layer 1 (YUNZHUO RC): Operator can always override via hardware
- han_dog_bridge watchdog: No cmd_vel for 200ms → zero velocity to dog
- Dog Board can be independently E-stopped via BLE

## Network Topology

```
┌──────────────────────────────────────────────────────┐
│                  Robot Body                           │
│                                                      │
│  ┌─────────────┐   Ethernet/USB   ┌──────────────┐  │
│  │  Nav Board  ├───────────────────┤  Dog Board   │  │
│  │ 192.168.x.A │   gRPC :13145    │ 192.168.x.B  │  │
│  │ :50051      │                   │ :13145       │  │
│  └──────┬──────┘                   └──────┬───────┘  │
│         │                                 │          │
│     WiFi AP                            BLE          │
│   192.168.4.1                                        │
└─────────┼─────────────────────────────────┼──────────┘
          │                                 │
     ┌────┴────┐                       ┌────┴────┐
     │  App    │                       │  App    │
     │  WiFi   │                       │  BLE    │
     └─────────┘                       └─────────┘

         ┌───────────┐
         │ YUNZHUO RC │──── SBUS/PPM ──→ Dog Board
         └───────────┘     (独立信号线)
```

### 多客户端并发连接

gRPC 基于 HTTP/2，**单个端口可同时服务多个客户端**。TCP 连接由四元组 `(源IP, 源端口, 目标IP, 目标端口)` 唯一标识，不同客户端的源 IP/端口不同，因此不会冲突。

```
Nav Board :50051
    │
    ├── Flutter App (手机 A)       ← 独立 TCP 连接，可同时监控
    ├── Flutter App (手机 B)       ← 独立 TCP 连接，可同时监控
    ├── grpcurl / 调试脚本         ← 独立 TCP 连接
    └── test_integration.py        ← 独立 TCP 连接
```

**注意事项**:

| 操作类型 | 多客户端行为 | 机制 |
|---------|------------|------|
| **监控** (StreamFastState, StreamSlowState) | 所有客户端同时收到数据流 | 各自独立 stream |
| **数据** (Subscribe, UploadFile, OTA) | 各客户端独立操作，互不影响 | 无锁定 |
| **控制** (StreamTeleop, SetMode) | **同一时间仅一个客户端**可操控 | `AcquireLease` 租约互斥 |

**不能共用端口的唯一场景**: 在同一台机器上启动两个 gRPC Server 都 `bind` 同一端口 → 第二个会报 `Address already in use`。

## Configuration Reference

### Dog Board CMS

| Parameter | Default | Description |
|-----------|---------|-------------|
| gRPC Port | `13145` | CMS service port |
| Walk range | `[-1, 1]` | Normalized velocity vector |
| Inference cycle | ~20ms | RL policy frequency |

### Nav Board GrpcGateway

| Parameter | Default | Description |
|-----------|---------|-------------|
| `grpc_port` | `50051` | Gateway service port |
| `fast_state_hz` | `30.0` | FastState stream frequency |
| `slow_state_hz` | `1.0` | SlowState stream frequency |
| `deadman_timeout_ms` | `300.0` | SafetyGate deadman timeout |
| `max_speed` | `1.0` | Max linear speed (m/s) |
| `max_angular` | `1.0` | Max angular speed (rad/s) |
| `obstacle_height_thre` | `0.2` | 障碍物高度阈值 (m), 与 local_planner 一致 |
| `stop_distance` | `0.8` | 急停距离 (m): 前方此范围内有障碍→线速度归零 |
| `slow_distance` | `2.0` | 减速距离 (m): 前方此范围内有障碍→线性减速 |
| `vehicle_width` | `0.6` | 车身宽度 (m) |
| `vehicle_width_margin` | `0.1` | 宽度安全裕度 (m) |

### han_dog_bridge

| Parameter | Default | Description |
|-----------|---------|-------------|
| `dog_host` | `127.0.0.1` | Dog Board CMS address |
| `dog_port` | `13145` | Dog Board CMS port |
| `max_linear_speed` | `1.0` | Walk normalization base (m/s) |
| `max_angular_speed` | `1.0` | Walk normalization base (rad/s) |
| `cmd_vel_timeout_ms` | `200.0` | Bridge watchdog timeout |
| `auto_enable` | `true` | Auto-enable motors on connect |
| `auto_standup` | `true` | Auto-standup on connect |

## Protocol Summary

| Protocol | Package | Language | Port | Speaker → Listener |
|----------|---------|----------|------|---------------------|
| `han_dog` CMS | `han_dog_message` | Python/Dart/C++ | 13145 | Bridge/App → Dog Board |
| `robot::v1` | `robot_proto` | C++/Dart | 50051 | App → Nav Board |
| BLE Custom | `ble_protocol.dart` | Dart | — | App → Dog Board |
| ROS2 DDS | `nav_msgs`, `interface` | C++/Python | — | Nav Board internal |
| SBUS/PPM | — | Hardware | — | YUNZHUO RC → Dog Board |

## Flutter App Dual-Connection Architecture

**核心设计**: Flutter App 可以同时维护两个独立 gRPC 连接。

```
┌─────────────────────────────────────────────────────────────┐
│                     Flutter App                             │
│                                                             │
│  ┌───────────────────────┐    ┌──────────────────────────┐  │
│  │   RobotClient         │    │   DogDirectClient        │  │
│  │   (robot::v1 协议)     │    │   (han_dog CMS 协议)     │  │
│  │   ↓ gRPC :50051       │    │   ↓ gRPC :13145          │  │
│  │   Nav Board           │    │   Dog Board              │  │
│  │                       │    │                          │  │
│  │   Features:           │    │   Features:              │  │
│  │   ✓ 导航/自主任务     │    │   ✓ 低延迟 IMU 流       │  │
│  │   ✓ 遥操作 (带安全)   │    │   ✓ 关节数据流          │  │
│  │   ✓ 遥测 (FastState)  │    │   ✓ RL 推理历史         │  │
│  │   ✓ 地图/点云/相机    │    │   ✓ 直连行走控制        │  │
│  │   ✓ OTA 固件更新      │    │   ✓ Enable/Disable      │  │
│  │   ✓ 租约/模式管理     │    │   ✓ StandUp/SitDown     │  │
│  └───────────────────────┘    └──────────────────────────┘  │
│           │                            │                    │
│           ▼                            ▼                    │
│  ┌────────────────────────────────────────────────────────┐ │
│  │            RobotConnectionProvider                     │ │
│  │   _client:    RobotClientBase?   (Nav Board)           │ │
│  │   _dogClient: DogDirectClient?   (Dog Board)           │ │
│  │   两者独立管理, 互不影响                                  │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Connection Modes

| Mode | Nav Board | Dog Board | Use Case |
|------|-----------|-----------|----------|
| **Full** (推荐) | Connected :50051 | Connected :13145 | 完整功能 + 底层诊断 |
| **Nav Only** | Connected :50051 | — | 仅导航功能 (传感器数据通过 bridge 中继) |
| **Dog Only** | — | Connected :13145 | Nav Board 不可用, 基础遥控 |
| **BLE Only** | — | BLE | 最小功能: 急停 + WiFi 配置 |

### Why Dual Connection?

1. **低延迟传感器**: Dog Board 的 IMU/Joint 流直达 App, 不经 Nav Board 中继
2. **降级容错**: Nav Board 崩溃时, App 仍有 Dog Board 直连可用
3. **RL 诊断**: ListenHistory 提供 policy 输入输出, 只有直连才能获取
4. **关节级监控**: 直接看到 16 个关节的状态码, 用于电机故障诊断

### Implementation

```dart
// Flutter App 中的双连接使用示例
final provider = context.read<RobotConnectionProvider>();

// 1. 连接 Nav Board (主连接)
await provider.connect(RobotClient(host: '192.168.4.1', port: 50051));

// 2. 同时连接 Dog Board (辅助连接)
await provider.connectDog(host: '192.168.4.100', port: 13145);

// 3. 使用 Nav Board 功能
provider.client!.setMode(RobotMode.ROBOT_MODE_TELEOP);

// 4. 同时使用 Dog Board 底层数据
provider.dogClient!.imuStream.listen((imu) {
  // 实时 IMU 数据 (直达, 不经 Nav Board)
});
provider.dogClient!.jointStream.listen((joint) {
  // 关节级数据
});
```

## Mode Switching — Complete Chain

### Mode State Machine (Nav Board ModeManager)

```
                     ┌──────────┐
          ┌──────────│   IDLE   │──────────┐
          │          └─┬──┬──┬──┘          │
          │            │  │  │             │
          ▼            │  │  │             ▼
     ┌────────┐        │  │  │       ┌──────────┐
     │ MANUAL │        │  │  │       │ MAPPING  │
     └────────┘        │  │  │       └──────────┘
                       │  │  │
          ┌────────────┘  │  └────────────┐
          ▼               │               ▼
     ┌─────────┐          │         ┌────────────┐
     │ TELEOP  │◄─────────┼────────►│ AUTONOMOUS │
     └─────────┘          │         └────────────┘
                          │
            ┌─────────────▼──────────────┐
            │          ESTOP             │
            │  (any state → ESTOP)       │
            │  (ESTOP → IDLE only)       │
            └────────────────────────────┘
```

### Transition Guards

| Target Mode | Required Guards |
|---|---|
| IDLE | Always allowed (from any non-ESTOP state) |
| MANUAL | From IDLE only |
| TELEOP | From IDLE/AUTONOMOUS + `has_lease` |
| AUTONOMOUS | From IDLE/TELEOP + `tf_ok` + `localization_valid` |
| MAPPING | From IDLE only |
| ESTOP | Via `EmergencyStop()` only (lock-free, any state) |
| Clear ESTOP → IDLE | `tilt_safe` + `fence_safe` |

### How Mode Switching Propagates

```
Flutter App                     Nav Board                      Dog Board
─────────────────────────────────────────────────────────────────────────

1. User taps "Teleop" in App
   │
   ├─→ robot.v1.SetMode(TELEOP) ─→ ModeManager.SwitchMode()
   │                                  ├─ CheckTransition()
   │                                  │    └─ has_lease? ✓
   │                                  ├─ SafetyGate.SetCurrentMode(TELEOP)
   │                                  └─ EnterState(TELEOP)
   │                                       └─ StopPathFollower()
   │
   ├─ Response: OK, mode=TELEOP
   │
2. User moves joystick
   │
   ├─→ StreamTeleop(velocity) ─→ SafetyGate.ProcessTeleop()
   │                               ├─ Mode guard: 非 TELEOP → 返回零速
   │                               ├─ Speed limit check
   │                               ├─ Tilt check
   │                               ├─ ★ 近场避障 (/terrain_map)
   │                               │   ├─ <0.8m → obstacle_stop (线速度=0)
   │                               │   ├─ 0.8~2m → obstacle_slow (线性减速)
   │                               │   └─ 角速度不受影响
   │                               ├─ E-stop check
   │                               └─ Publish /cmd_vel
   │                                    │
   │                                    ▼
   │                               han_dog_bridge
   │                                    │ twist_to_walk()
   │                                    ▼
   │                               CMS.Walk(Vector3) ──→ Arbiter
   │                                                      ├─ RC active? → REJECT
   │                                                      └─ No RC → ACCEPT
   │                                                           ▼
   │                                                      RL Policy → Motors
   │
   ├─ TeleopFeedback(actual_vel, safety, latency)
   │   └─ limit_reasons 包含: obstacle_stop / obstacle_slow / max_speed 等
```

### Dog Board Direct Mode (No Nav Board)

```
Flutter App                                      Dog Board
──────────────────────────────────────────────────────────

1. App connects directly to Dog Board :13145
   │
   ├─→ DogDirectClient.connect()
   │     └─ CmsClient.getParams()  → Robot type, model
   │
2. App enables motors + standup
   │
   ├─→ CmsClient.enable()
   ├─→ CmsClient.standUp()
   │
3. App sends walk via joystick
   │
   ├─→ CmsClient.walk(Vector3)  → Arbiter → RL Policy → Motors
   │
   │   (NO SafetyGate, NO lease, NO mode manager)
   │   (Only Dog Board arbiter: RC > gRPC)
```

## Proto Compatibility

### Two Proto Ecosystems

| | `robot_proto` (Nav Board) | `han_dog_message` (Dog Board) |
|---|---|---|
| Package | `robot.v1` | `han_dog` |
| Port | `:50051` | `:13145` |
| Dart protobuf | `^6.0.0` | `^6.0.0` (升级后) |
| Dart grpc | `^5.1.0` | `^5.1.0` (升级后) |
| Well-known types | `package:protobuf/well_known_types/...` | 需重新生成 |

### Proto Regeneration (Required Once)

`han_dog_message` 的 Dart 代码需要用新版 `protoc-gen-dart` 重新生成:

```bash
# 1. Install new protoc-gen-dart
dart pub global activate protoc_plugin

# 2. Regenerate
cd han_dog_message
bash tool/regen_dart.sh

# 3. Verify
cd dart && dart pub get
```

## OTA Update Architecture

```
┌──────────────────┐    HTTPS     ┌──────────────────┐     gRPC      ┌──────────────────┐
│  GitHub Releases │◄─────────────│   Flutter App    │──────────────►│   Nav Board      │
│                  │              │                  │               │   DataService    │
│  manifest.json   │  直接下载    │  1. 签名验证     │               │                  │
│  (Ed25519 签名)  │◄─ ─ ─ ─ ─ ─ │  2. 版本对比     │               │  1. 签名验证     │
│  *.onnx / .deb   │              │  3. 依赖检查     │               │  2. 依赖检查     │
│  *.pcd / .yaml   │              │  4. 预检查       │               │  3. 安全模式     │
│                  │              │  5. 进度展示     │               │  4. 事务日志     │
└──────────────────┘              └──────────────────┘               │  5. 备份→安装    │
                                                                     └──────────────────┘
```

### OTA 生命周期

1. **检查更新** — 拉取 `manifest.json` + Ed25519 签名验证 + 版本对比
2. **预检查** — `CheckUpdateReadiness`: 磁盘/电量/硬件兼容/依赖/安全等级/网络
3. **安全模式** — HOT: 无需停机 / WARM: 暂停导航 / COLD: sit → disable → 维护态
4. **下载** — 方式 A: `DownloadFromUrl` 机器人直连 / 方式 B: `UploadFile` 手机中转（断点续传）
5. **原子安装** — 事务日志 → SHA256 → 依赖检查 → 备份 → 安装 → 清理日志
6. **崩溃恢复** — 检测未完成事务 → 自动回滚
7. **回滚** — `Rollback` 手动从备份恢复

### OTA 制品安全等级

| 等级 | 类型 | apply_action | 安装时机器人状态 |
|------|------|-------------|----------------|
| **HOT** | 地图 / 配置 | `copy_only` | 可运行中安装 |
| **WARM** | 模型 (ONNX) | `reload_model` | 导航暂停 |
| **COLD** | 固件 / MCU | `install_deb` / `flash_mcu` | sit + disable + 维护态 |

### OTA 系统边界

| owner_module | 负责制品 | 安装方式 |
|-------------|---------|---------|
| brain | ONNX 模型 | 复制 + Dog Board 热加载 |
| navigation | PCD 地图 | 复制 + Localizer 重加载 |
| config_service | YAML 配置 | 复制 + ROS2 参数事件 |
| system | DEB 固件 | dpkg -i + 重启 |
| mcu | HEX/BIN | 刷写脚本 + 重启 |

详见 [OTA_GUIDE.md](OTA_GUIDE.md)

---

## Deployment Scenarios

### Scenario A: Full System (推荐)
- Nav Board + Dog Board + LiDAR + App
- App connects to Nav Board :50051 + Dog Board :13145 (dual)
- Full navigation + safety + monitoring + low-latency sensors
- RC as emergency override

### Scenario B: Dog Board Only (调试/简单遥控)
- Dog Board only (no Nav Board)
- App connects directly to Dog Board :13145
- Basic walk/stand/sit control
- No navigation, no safety gateway
- RC as primary control

### Scenario C: RC Only (最小系统)
- Dog Board + YUNZHUO RC only
- No network, no app
- Pure hardware control
- Suitable for outdoor field testing
