# LingTu (灵途) v1.8.0 — 系统架构全景

> 本文档是 LingTu 导航系统的软件架构全景，描述 7 层 + 3 环正交架构、感知/规划/基础设施各子系统的设计与数据流。
> 硬件双板架构、通信协议、安全体系详见 [ARCHITECTURE.md](./ARCHITECTURE.md)。
> ROS2 话题契约详见 [TOPIC_CONTRACT.md](./TOPIC_CONTRACT.md)。

**最后更新**: 2026-03-15 | **版本**: 1.8.0

---

## 一、7层 + 3环 正交架构

```
         ┌─── 层 (Layer): 技术依赖，自下而上 ───┐
         │                                       │
    L6   │  Interaction    gRPC :50051 + Flutter App + Dashboard
    L5   │  Orchestration  mission_arc FSM + pct_path_adapter + nav_services
    L4   │  Planning       semantic_planner(VLN) + PCT(全局A*) + local_planner(局部)
    L3   │  Perception     semantic_perception(YOLO+CLIP) + terrain_analysis
    L2   │  SLAM           Fast-LIO2 / Point-LIO + ICP 重定位
    L1   │  Drivers        Livox LiDAR + han_dog_bridge(gRPC↔ROS2) + Orbbec RGB-D
    L0   │  Common         semantic_common (validation / sanitize / robustness)
         └───────────────────────────────────────┘

         ┌─── 环 (Ring): 认知控制，正交于层 ───┐
    R1   │  SafetyMonitor   20Hz  反射弧: 心跳监控 + 急停升级
    R2   │  Evaluator        5Hz  认知环: 闭环评估 (偏差/进度/停滞)
    R3   │  DialogueManager  2Hz  对话环: 统一用户状态 + ETA
         └───────────────────────────────────────┘

依赖规则: Layer N 只能 import Layer 0..N-1，层间走 ROS2 话题
Ring 正交于 Layer，可订阅任意层的话题，但不修改数据流
完整定义: config/layer_contract.yaml
```

---

## 二、感知层 (L3 Perception)

### 2.1 双检测管线

```
路径 A: BPU 硬件加速 (S100P 生产)
  Orbbec RGB-D → BPUDetector (.hbm, NV12) → Detection2D (bbox+mask+label)
  ├─ YOLO11s-seg (COCO 80类, 22ms)       ← 通用检测
  └─ YOLOE-26s-seg (导航 125类, 29ms)    ← 导航专用词汇表

路径 B: CPU/GPU 推理 (Jetson / 开发)
  RGB-D → YOLO-E / YOLO-World → Detection2D
```

路径选择由 `config/semantic_perception.yaml` 的 `detector.backend` 控制。BPU 管线在 S100P (Nash BPU 128 TOPS) 上实现实时推理。

### 2.2 完整感知管线

```
相机帧 → 模糊检测 → YOLO 检测 → MobileCLIP 文本编码(缓存)
  → mask+depth 3D投影 → TF变换(camera→map)
  → InstanceTracker (IoU+CLIP 双指标融合)
  → ConceptGraphs 场景图 (Floor/Room/Group/Object + 空间关系)
  → 发布 /nav/semantic/scene_graph (JSON)
```

### 2.3 跟踪子系统

| 方案 | 实现 | 延迟 | FPS | 适用场景 |
|------|------|------|-----|---------|
| 轻量跟踪 | BPUTracker (BPU检测 + BoT-SORT) | 44ms | 23 | 通用实时跟踪 |
| 专业跟踪 | qp_perception (FusionMOT + OSNet Re-ID) | 80ms | 13 | 跨遮挡重识别 |
| 人物跟随 | PersonTracker (VLM选人 + 视觉跟踪) | — | — | "跟着红衣服的人" |

### 2.4 关键文件

| 文件 | 路径前缀 `src/semantic_perception/semantic_perception/` | 职责 |
|------|------|------|
| `perception_node.py` | | ROS2 主节点 (15Hz USS-Nav 管线) |
| `bpu_detector.py` | | BPU YOLO 检测 (YOLOE 端到端 + YOLO11 多尺度) |
| `bpu_tracker.py` | | BPU + BoT-SORT 实时跟踪 |
| `bpu_qp_bridge.py` | | qp_perception FusionMOT + OSNet 桥接 |
| `instance_tracker.py` | | 场景图构建 (BA-HSG + KG + BP, ~3400行) |
| `mobileclip_encoder.py` | | USS-Nav 文本编码 (缓存后零开销) |
| `clip_encoder.py` | | 完整 CLIP (图像+文本, HOV-SG 三源编码) |
| `projection.py` | | mask→3D 点云投影 (USS-Nav §IV-C) |
| `person_tracker.py` | | VLM 选人 + EMA 平滑 + 速度预测 + 外观 Re-ID |

---

## 三、规划层 (L4 Planning)

### 3.1 语义规划器 — Fast-Slow 双进程

```
用户指令 "导航到餐桌"
  ↓
task_decomposer: 指令 → [NAVIGATE, FIND, APPROACH, VERIFY] 子目标
  ↓
goal_resolver (Fast-Slow):
  ├─ Fast Path (~170ms, >70% hit): 场景图直接匹配
  │   权重: 标签35% + CLIP35% + 检测器15% + 空间15%
  │   阈值: ≥0.75 → 直接输出坐标
  │
  └─ Slow Path (~2s): ESCA 过滤(200→15 对象) → LLM 推理
      触发: score<0.75 || entropy>1.5 (AdaNav)
      LLM: Kimi(默认) → Qwen(fallback)
  ↓
action_executor: 子目标 → ActionCommand (PoseStamped / TwistStamped)
  ↓
发布 /nav/goal_pose → 全局规划 → 局部规划 → 到达
```

### 3.2 全局规划

```
PCT Planner:
  ├─ 生产: ele_planner.so (C++ A*, 5-10ms)
  └─ 开发: pct_planner_astar.py (纯 Python, 50-200ms)

pct_path_adapter:
  全局路径 → 逐个航点推进 → /nav/way_point
  保护: max_index_jump(3) + max_first_waypoint_dist(10m)
  事件: waypoint_reached / goal_reached / stuck_final
```

### 3.3 局部规划

```
pathFollower (C++):
  /nav/way_point + /nav/terrain_map → Pure Pursuit → /nav/cmd_vel

  渐进卡死检测:
    0~5s 无进展 → WARN_STUCK
    5~10s 无进展 → STUCK
    恢复: 连续 3帧 v>0.05m/s

terrain_analysis (C++):
  /nav/map_cloud → 地面估计 + 可通行性 → /nav/terrain_map
  slopeWeight 参数: 默认0, 建议3-6 (坡度影响路径代价)
```

### 3.4 记忆系统

| 组件 | 类型 | 容量 | 用途 |
|------|------|------|------|
| TopologicalMemory | 拓扑图 | 500 节点 | 位置记忆 + 房间关联 + FSR-VLN 视点边 |
| EpisodicMemory | 时空 FIFO | 500 记录 | ReMEmbR 风格检索 (关键词+空间) |
| KnowledgeGraph | 工业 KG | 72 概念 | 物体属性 + 安全约束 + CLIP 词汇扩展 |

### 3.5 关键文件

| 文件 | 路径前缀 `src/semantic_planner/semantic_planner/` | 职责 |
|------|------|------|
| `planner_node.py` | | ROS2 主节点 (~3500行, 异步线程模型) |
| `goal_resolver.py` | | Fast-Slow 核心 + AdaNav 熵触发 |
| `task_decomposer.py` | | SayCan 风格子目标分解 |
| `action_executor.py` | | 6 种动作原语 (LOVON) + LERa 恢复 |
| `frontier_scorer.py` | | MTU3D frontier 评分 + TSP 排序 |
| `topological_memory.py` | | 拓扑图 + FSR-VLN 视点边 (Jaccard-weighted) |
| `episodic_memory.py` | | 时空情节记忆 (ReMEmbR) |
| `sgnav_reasoner.py` | | SG-Nav 场景图推理 + 可信度 EMA |
| `implicit_fsm_policy.py` | | 隐式 FSM (参数化线性模型) |
| `llm_client.py` | | 多后端 LLM (Kimi/Qwen/OpenAI/Claude) |
| `person_tracker.py` | | VLM 选人 + Re-ID 跟踪 |

---

## 四、基础设施层

### 4.1 SLAM (L2)

```
Fast-LIO2 (生产默认):
  Livox Mid-360 CustomMsg + IMU → EKF 融合 → /nav/odometry + /nav/map_cloud
  切换: slam_profile:=fastlio2 | pointlio | stub

Point-LIO (备选):
  同样传感器输入 → iVox + EKF → 四足振动鲁棒性更优 (0.006m 漂移)

ICP 重定位:
  localizer → /nav/localization_quality
  SafetyMonitor 监控: quality<0.3 → WARN, 丢失 → 自动 relocalize (30s 冷却)
```

### 4.2 驱动层 (L1)

```
han_dog_bridge.py:
  /nav/cmd_vel → gRPC Walk(vx,vy,vyaw) → brainstem CMS :13145
  ListenImu() → /nav/dog_odometry (Hamilton→ROS 四元数转换)
  ListenJoint() → /robot_state (16 DOF)
  看门狗: 200ms 无 cmd_vel → Walk(0,0,0)
  SLAM 位置重置: 每 5s 从 /nav/odometry 同步
```

### 4.3 三环认知 (nav_rings)

| Ring | 频率 | 职责 | 输出话题 |
|------|------|------|---------|
| R1 SafetyMonitor | 20Hz | 心跳+急停+链路断裂→stop=2 | `/nav/safety_state` |
| R2 Evaluator | 5Hz | cross_track_error + progress_rate + stall | `/nav/execution_eval` |
| R3 DialogueManager | 2Hz | 聚合所有状态→人类可读描述+ETA | `/nav/dialogue_state` |

### 4.4 交互层 (L6)

```
gRPC Gateway (C++, :50051):
  ← Flutter App / Dashboard / 远程客户端
  → /nav/goal_pose, /nav/semantic/instruction

Flutter Monitor (Dart):
  Android APK + Windows Desktop + iOS
  实时遥测 + 地图 + 手动控制 + 语义导航输入

Web Dashboard (HTML+JS):
  http://robot_ip:8066
  系统健康 + 语义状态面板
```

---

## 五、启动系统

### 三种运行模式

| 模式 | 入口 | 预建地图 | 自主导航 | 语义 |
|------|------|---------|---------|------|
| Mapping | `navigation_bringup.launch.py` | 不需要 | ❌ | ❌ |
| Navigation | `navigation_run.launch.py` | ✅ 必需 | ✅ | 可选 |
| Exploration | `navigation_explore.launch.py` | ❌ | ✅ (Frontier) | ✅ 强制 |

### 子系统启动链

```
navigation_run.launch.py
  ├─ lidar.launch.py          → Livox Mid-360
  ├─ slam.launch.py           → Fast-LIO2 (profile 可切: fastlio2 | pointlio | stub)
  ├─ planning.launch.py       → ICP Localizer + PCT A* + mission_arc
  ├─ autonomy.launch.py       → terrain_analysis + local_planner + pathFollower
  ├─ driver.launch.py         → han_dog_bridge (gRPC ↔ brainstem CMS :13145)
  ├─ grpc.launch.py           → gRPC Gateway :50051
  ├─ semantic.launch.py       → perception_node + planner_node (可选)
  ├─ rings.launch.py          → 三环认知 R1/R2/R3 (可选)
  └─ services.launch.py       → map/patrol/geofence/task managers (可选)
```

---

## 六、Systemd 服务 (S100P 部署)

```
启动顺序 (依赖链):
  nav-lidar → nav-slam → nav-planning → nav-autonomy → nav-driver
                                      → nav-grpc
                                      → nav-semantic (可选)
                                      → nav-monitor (可选)
  ota-daemon (独立, :50052)

用户: sunrise
KillMode: control-group (graceful shutdown ~0.37s)
Restart: on-failure
```

服务文件位于 `systemd/`，安装脚本: `scripts/install_services.sh`。

---

## 七、数据流全景

```
用户 (Flutter/Dashboard/gRPC)
  │  指令: "找到灭火器" 或 坐标(x,y,z)
  ↓
┌─ L6 gRPC Gateway ──────────────────────────────────────┐
│  发布: /nav/semantic/instruction 或 /nav/goal_pose      │
└─────────────────────────────────────────────────────────┘
  ↓
┌─ L4 Semantic Planner ──────────────────────────────────┐
│  Fast Path (170ms): 场景图直接匹配                       │
│  Slow Path (2s): ESCA + LLM 推理                        │
│  输出: /nav/goal_pose                                    │
└─────────────────────────────────────────────────────────┘
  ↓
┌─ L4 PCT Global Planner ───────────────────────────────┐
│  A* 搜索 (5-10ms) → /nav/global_path                   │
│  pct_path_adapter → /nav/way_point (逐个推进)            │
└─────────────────────────────────────────────────────────┘
  ↓
┌─ L4 Local Planner ────────────────────────────────────┐
│  Pure Pursuit + terrain_map → /nav/cmd_vel              │
│  卡死检测: WARN_STUCK (5s) → STUCK (10s)                │
└─────────────────────────────────────────────────────────┘
  ↓
┌─ L1 han_dog_bridge ───────────────────────────────────┐
│  cmd_vel → gRPC Walk() → brainstem CMS :13145           │
│  ← IMU/Joint 反馈 → /nav/dog_odometry, /robot_state     │
└─────────────────────────────────────────────────────────┘
  ↓
┌─ L2 SLAM (Fast-LIO2) ────────────────────────────────┐
│  LiDAR + IMU → EKF → /nav/odometry + /nav/map_cloud    │
└─────────────────────────────────────────────────────────┘
  ↓
┌─ L3 Perception ───────────────────────────────────────┐
│  RGB-D → BPU YOLO (29ms) → CLIP → 3D投影 → 场景图       │
│  → /nav/semantic/scene_graph (闭环回 L4)                 │
└─────────────────────────────────────────────────────────┘

并行监控:
  R1 SafetyMonitor (20Hz) → 急停保护
  R2 Evaluator (5Hz) → 闭环评估 → 自适应重规划
  R3 DialogueManager (2Hz) → 用户状态播报
```

---

## 八、硬件部署拓扑 (S100P)

```
┌─────────────────────────────────────────────────┐
│  S100P (RDK X5, 16GB RAM, Nash BPU 128 TOPS)   │
│  ├─ eMMC 45G (OS + ROS2 + 服务)                │
│  └─ NVMe SSD 238G (models/data/lingtu)          │
│                                                  │
│  传感器:                                         │
│  ├─ Livox Mid-360 (LiDAR, Ethernet)             │
│  ├─ Orbbec Gemini 335 (RGB-D, USB3.2, 30fps)    │
│  └─ brainstem IMU (via gRPC :13145)             │
│                                                  │
│  BPU 模型:                                       │
│  ├─ yolo11s_seg_nashe_640x640_nv12.hbm (19MB)   │
│  ├─ yoloe26s_seg_nav125_nashe_640x640_nv12.hbm  │
│  └─ + 4 个备选 YOLO 模型                         │
│                                                  │
│  网络: 192.168.66.190                            │
│  gRPC: :50051 (导航) + :50052 (OTA)             │
└─────────────────────────────────────────────────┘
         │ WiFi/Ethernet
         ↓
┌─────────────────────────────────────────────────┐
│  brainstem CMS (Dart Runtime, 同板或分板)         │
│  :13145 gRPC, RL Policy ONNX                    │
│  FSM: Zero→Grounded→Standing→Walking            │
│  Arbiter: 遥控器 > gRPC (硬件优先级)              │
└─────────────────────────────────────────────────┘
         │ WiFi
         ↓
┌─────────────────────────────────────────────────┐
│  Flutter Monitor App (Windows/Android/iOS)       │
│  实时遥测 + 地图 + 语义导航 + 手动控制             │
│  双连接: :50051 (Nav) + :13145 (Dog)             │
└─────────────────────────────────────────────────┘
```

---

## 九、配置系统

```
config/
├─ robot_config.yaml          ★ 唯一真源 (机器人物理参数: 速度/尺寸/安全阈值)
├─ layer_contract.yaml        7 层依赖规则 + Ring 定义 + 包分配
├─ topic_contract.yaml        ROS2 话题合约 (/nav/* 前缀, 60+ 话题)
├─ semantic_planner.yaml      VLN 参数: Fast-Slow 阈值 + LLM 后端 + 探索策略
├─ semantic_perception.yaml   YOLO + CLIP 配置: 检测器后端 + 模型路径
├─ semantic_exploration.yaml  探索模式覆盖 (SCG 自动扩展 + GCM + mock LLM)
├─ qos_profiles.yaml          DDS QoS 配置
├─ cyclonedds.xml              CycloneDDS 传输配置
├─ lio.yaml                    Fast-LIO2 参数
└─ pointlio.yaml               Point-LIO 参数
```

---

## 十、源码包总览

| 包 | 路径 | 语言 | 层 | 职责 |
|---|---|---|---|---|
| `semantic_common` | `src/semantic_common/` | Python | L0 | 验证/清洗/重试装饰器 (共享基础) |
| `livox_ros_driver2` | `src/drivers/livox_ros_driver2/` | C++ | L1 | Livox LiDAR 驱动 |
| `robot_driver` | `src/drivers/robot_driver/` | Python | L1 | han_dog_bridge (gRPC↔ROS2) |
| `fastlio2` | `src/slam/fastlio2/` | C++ | L2 | Fast-LIO2 SLAM |
| `pointlio` | `src/slam/pointlio/` | C++ | L2 | Point-LIO SLAM (备选) |
| `localizer` | `src/slam/localizer/` | C++ | L2 | ICP 重定位 |
| `terrain_analysis` | `src/base_autonomy/terrain_analysis/` | C++ | L3 | 地面估计 + 可通行性 |
| `semantic_perception` | `src/semantic_perception/` | Python | L3 | YOLO+CLIP+场景图 (~22k LOC) |
| `local_planner` | `src/base_autonomy/local_planner/` | C++ | L4 | 局部避障 + pathFollower |
| `PCT_planner` | `src/global_planning/PCT_planner/` | C++/Py | L4 | 全局 A* 路径规划 |
| `pct_adapters` | `src/global_planning/pct_adapters/` | Python | L5 | 航点适配 + mission_arc FSM |
| `semantic_planner` | `src/semantic_planner/` | Python | L4 | VLN Fast-Slow 语义规划 (~9.6k LOC) |
| `nav_rings` | `src/nav_rings/` | Python | R1-R3 | 三环认知架构 |
| `nav_services` | `src/nav_services/` | Python | L5 | 地图/巡逻/地理围栏/任务管理 |
| `remote_monitoring` | `src/remote_monitoring/` | C++ | L6 | gRPC Gateway (遥测/控制/OTA/WebRTC) |
| `robot_proto` | `src/robot_proto/` | Protobuf | — | 共享协议定义 (git submodule) |

---

## 十一、测试状态

| 测试 | 状态 | 关键指标 |
|------|------|---------|
| semantic_planner 单元测试 | 388/388 PASS | 13 个测试文件, ~90% 覆盖 |
| Fast-Slow benchmark | 15/15 PASS | Fast Path >70% hit rate |
| PCT adapter 逻辑测试 | 20/20 PASS | 航点跟踪+卡死检测 |
| 全链路集成 (T6) | 5/5 PASS | 18s 端到端 |
| 语义 E2E | PASS | 107s, Fast Path conf=0.95 |
| BPU 跟踪验证 | PASS | 5 目标, 10/10 帧 100% 稳定 |
| YOLOE-26 BPU | PASS | 29ms, 125 类, 检测正确 |
| SLAM 振动鲁棒性 | PASS | Point-LIO 0.006m / Fast-LIO2 0.008m 漂移 |

详见 `tests/README.md` 和 `docs/07-testing/`。

---

## 十二、性能目标

| 组件 | 目标 | 实测 |
|------|------|------|
| Fast Path 响应 | <200ms | ~170ms |
| Fast Path 命中率 | >70% | ~75% |
| BPU YOLO11s-seg | >20 FPS | 45 FPS (22ms) |
| BPU YOLOE-26s-seg | >20 FPS | 35 FPS (29ms) |
| BPU + BoT-SORT 跟踪 | >15 FPS | 23 FPS (44ms) |
| CLIP 缓存命中率 | 60-80% | — |
| 场景图更新 | 1-2 Hz | 1-2 Hz |
| 全局规划 (C++ A*) | <50ms | 5-10ms |
| 全局规划 (Python A*) | <500ms | 50-200ms |
| Systemd graceful shutdown | <5s | 0.37s |

---

## 相关文档

| 文档 | 内容 |
|------|------|
| [ARCHITECTURE.md](./ARCHITECTURE.md) | 双板硬件架构、通信协议、安全体系、模式切换 |
| [TASK_ORCHESTRATION.md](./TASK_ORCHESTRATION.md) | 多 Agent 任务编排 (Askme→orchestrator→LingTu) |
| [TOPIC_CONTRACT.md](./TOPIC_CONTRACT.md) | ROS2 话题契约 (60+ 话题定义) |
| [ALGORITHM_REFERENCE.md](./ALGORITHM_REFERENCE.md) | 核心算法参考 (论文→代码映射) |
| `config/layer_contract.yaml` | 7 层 + 3 环完整定义 (机器可读) |
