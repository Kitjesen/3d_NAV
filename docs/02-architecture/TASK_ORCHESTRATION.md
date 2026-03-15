# Task Orchestration Architecture — 多 Agent 任务编排

> 本文档描述从外部触发（语音/App/gRPC）到 LingTu 内部语义导航执行的完整任务编排链路。
> 硬件双板架构和底层数据流请参见 [ARCHITECTURE.md](./ARCHITECTURE.md)。

**版本**: 1.8.0 | **最后更新**: 2026-03-12

---

## 系统全景

```
┌─────────────────────────────────────────────────────────────────────┐
│                    NOVA Dog Runtime (products/nova-dog/runtime)      │
│                                                                     │
│  ┌──────────────┐   ┌────────────────────┐   ┌──────────────────┐  │
│  │    Askme      │──▶│ mission-orchestrator│──▶│   nav-gateway    │  │
│  │  (语音意图)   │   │  (唯一任务 owner)   │   │  (协议翻译层)    │  │
│  └──────────────┘   └────────────────────┘   └────────┬─────────┘  │
│                                                        │            │
│  ┌──────────────┐                                      │            │
│  │  Flutter App  │─── gRPC :50051 ─────────────────────┤            │
│  │  (UI 触发)    │                                     │            │
│  └──────────────┘                                      │            │
└────────────────────────────────────────────────────────┼────────────┘
                                                         │
                                              LingTuGrpcBridge
                                              gRPC :50051 (LingTu)
                                                         │
┌────────────────────────────────────────────────────────┼────────────┐
│                    LingTu (brain/lingtu)                │            │
│                                                         ▼            │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  ControlService / TaskManager                                 │   │
│  │  → /nav/semantic/instruction (ROS2 String, JSON)              │   │
│  └──────────────────────────────┬───────────────────────────────┘   │
│                                  ▼                                   │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  semantic_planner_node                                        │   │
│  │  → goal_resolver (Fast Path / Slow Path)                      │   │
│  │  → FrontierScorer / TSG (探索)                                │   │
│  │  → /nav/goal_pose                                             │   │
│  └──────────────────────────────┬───────────────────────────────┘   │
│                                  ▼                                   │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  localPlanner → pathFollower → /nav/cmd_vel                   │   │
│  └──────────────────────────────┬───────────────────────────────┘   │
│                                  ▼                                   │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  han_dog_bridge → gRPC :13145 → Dog Board → Motors            │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 触发方式汇总

| 方式 | 入口 | 经过层 | 适用场景 |
|------|------|--------|----------|
| **语音** | Askme "去厨房" | Askme → orchestrator → gateway → LingTu | NOVA Dog 整机 |
| **Flutter App** | UI 点击目标 | App → gRPC :50051 (LingTu) | 远程监控 |
| **grpcurl** | 直接调 LingTu :50051 | ControlService → TaskManager | **开发调试推荐** |
| **ROS2 topic** | `ros2 topic pub /nav/semantic/instruction` | 直接到 planner_node | 最底层调试 |
| **launch 参数** | `target:="找到餐桌"` | planner_node 3s 延迟自触发 | 探索模式冷启动 |

---

## 链路 1: 语音触发 (Askme → LingTu)

完整路径：

```
用户语音: "去厨房"
    │
    ▼
┌─────────────────────────────────────────────────────┐
│  Askme VoiceIntentPlanner                            │
│  _SEMANTIC_NAV_PATTERNS:                             │
│    "去(.+)", "导航到(.+)", "带我去(.+)",              │
│    "走到(.+)", "过去(.+)", "前往(.+)"                 │
│  regex match → target = "厨房"                       │
│                                                      │
│  输出: VoicePlan(                                    │
│    action_type = "semantic_nav",                     │
│    mission_draft = {target: "厨房"}                  │
│  )                                                   │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│  mission-orchestrator                                │
│  创建 Mission(type=SEMANTIC_NAV, params={target})    │
│  状态: PENDING → DISPATCHED                          │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│  nav-gateway                                         │
│  收到 dispatch 指令                                   │
│  调用 LingTuGrpcBridge.start_semantic_nav(           │
│    mission_id = "m-xxx",                             │
│    semantic_target = "厨房",                          │
│    language = "zh",                                  │
│    explore_if_unknown = True,                        │
│    timeout_sec = 300                                 │
│  )                                                   │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│  LingTu ControlService/StartTask                     │
│  task_type = TASK_TYPE_SEMANTIC_NAV (6)              │
│  TaskManager 发布 JSON:                               │
│  {                                                   │
│    "instruction": "厨房",                            │
│    "language": "zh",                                 │
│    "explore_if_unknown": true,                       │
│    "timeout_sec": 300,                               │
│    "arrival_radius": 1.0                             │
│  }                                                   │
│  → /nav/semantic/instruction (ROS2 topic)            │
└──────────────────────┬──────────────────────────────┘
                       ▼
┌─────────────────────────────────────────────────────┐
│  semantic_planner_node._instruction_callback()       │
│  → goal_resolver.resolve("厨房")                     │
│    Fast Path: 场景图匹配 → 有?直接导航               │
│    Slow Path: LLM推理 → 没见过?frontier探索          │
│  → /nav/goal_pose                                    │
│  → localPlanner → pathFollower → /nav/cmd_vel        │
│  → han_dog_bridge → Dog Board → 机器人走             │
└─────────────────────────────────────────────────────┘
```

### 关键代码位置

| 组件 | 文件 | 关键函数/行 |
|------|------|-------------|
| VoiceIntentPlanner | `products/nova-dog/runtime/services/askme-edge-service/src/.../planner.py` | `_SEMANTIC_NAV_PATTERNS`, `plan()` |
| mission-orchestrator | `products/nova-dog/runtime/services/mission-orchestrator/` | `dispatch_mission()` |
| LingTuGrpcBridge | `products/nova-dog/runtime/services/nav-gateway/src/.../lingtu_grpc_bridge.py` | `start_semantic_nav()` |
| ControlService | `src/remote_monitoring/src/core/control_service.cpp` | `StartTask()` |
| TaskManager | `src/remote_monitoring/src/core/task_manager.cpp:446-462` | `TASK_TYPE_SEMANTIC_NAV` |
| planner_node | `src/semantic_planner/semantic_planner/planner_node.py` | `_instruction_callback()` |
| goal_resolver | `src/semantic_planner/semantic_planner/goal_resolver.py` | `resolve()` |

---

## 链路 2: gRPC 直接触发 (开发调试)

跳过 NOVA Dog Runtime，直接调 LingTu 的 gRPC:

```bash
# 带语义目标的探索
grpcurl -plaintext -d '{
  "task_type": 6,
  "semantic_nav_params": {
    "instruction": "找到餐桌",
    "language": "zh",
    "explore_if_unknown": true,
    "timeout_sec": 300,
    "arrival_radius": 1.0
  }
}' localhost:50051 robot.v1.ControlService/StartTask

# 纯覆盖探索（无特定目标）
grpcurl -plaintext -d '{
  "task_type": 6,
  "semantic_nav_params": {
    "instruction": "探索整个环境",
    "language": "zh",
    "explore_if_unknown": true,
    "timeout_sec": 600
  }
}' localhost:50051 robot.v1.ControlService/StartTask

# 查询任务状态
grpcurl -plaintext localhost:50051 robot.v1.ControlService/GetTaskStatus
```

**前提**: `nav-grpc` 和 `nav-semantic` systemd 服务必须在运行。

```bash
# 检查服务状态
sudo systemctl status nav-grpc nav-semantic

# 启动服务（如果未运行）
sudo systemctl start nav-grpc nav-semantic
```

---

## 链路 3: ROS2 Topic 直接触发 (最底层)

```bash
# 在机器人上直接发布指令
ros2 topic pub --once /nav/semantic/instruction std_msgs/String \
  '{"data": "{\"instruction\": \"找到餐桌\", \"explore_if_unknown\": true, \"language\": \"zh\"}"}'
```

**注意**: 这种方式跳过了 gRPC 安全校验和租约机制，仅建议在调试时使用。

---

## 链路 4: Launch 参数触发 (冷启动探索)

```bash
# 探索模式启动时自动触发语义指令
ros2 launch navigation_explore.launch.py target:="找到餐桌" llm_backend:=kimi

# 纯覆盖探索（不传 target）
ros2 launch navigation_explore.launch.py
```

**实现机制**: `planner_node.py` 声明 `initial_instruction` 参数，启动后 3 秒通过 timer 自动注入到 `_instruction_callback()`。

---

## 任务状态回传

```
LingTu semantic_planner_node
    │ 发布 /nav/semantic/status (JSON)
    │ 发布 /nav/planner_status (String)
    │ 发布 /nav/adapter_status (JSON)
    ▼
LingTu ControlService/GetTaskStatus
    │ 汇总各话题状态 → TaskStatus proto
    ▼
LingTuGrpcBridge._poll_loop()
    │ 每 3 秒轮询 GetTaskStatus
    │ 状态变化时回调通知
    ▼
nav-gateway → mission-orchestrator
    │ 更新 Mission 状态
    │ RUNNING → SUCCEEDED / FAILED
    ▼
Askme / Flutter App
    │ 语音播报 "已到达目标" 或 UI 更新
```

### 任务状态值

| 状态 | 含义 |
|------|------|
| `IDLE` | 无任务 |
| `PLANNING` | 正在规划路径 |
| `SUCCESS` | 路径规划成功，正在执行 |
| `FAILED` | 规划失败（无法到达） |
| `GOAL_REACHED` | 到达目标 |
| `WARN_STUCK` | 疑似卡住（半超时预警） |
| `STUCK` | 确认卡住（全超时） |

---

## 核心设计原则

### 1. 单一任务 Owner

**mission-orchestrator 是唯一的任务生命周期管理者**。Askme、Flutter App、CLI 都只能*提交*任务，不能直接控制 LingTu。

```
❌ Askme → 直接调 LingTu gRPC
✅ Askme → mission-orchestrator → nav-gateway → LingTu
```

### 2. LingTu 单任务模型

LingTu 内部的 TaskManager 同时只允许一个任务运行。新任务会自动取消旧任务。多任务排队和并行编排在 mission-orchestrator 层处理。

### 3. 协议翻译层

nav-gateway 中的 `LingTuGrpcBridge` 负责把 NOVA Dog Runtime 的 mission 协议翻译成 LingTu 的 `robot.v1` gRPC 协议。

**双模式兼容**:
- **模式 A**: 有编译好的 protobuf stubs (`robot_proto.python.robot.v1`) → 直接调用
- **模式 B**: 无 stubs → 用原始 protobuf 字节编码 (fallback)

### 4. 安全边界

| 层 | 安全机制 |
|----|----------|
| mission-orchestrator | 任务冲突检测、优先级仲裁 |
| nav-gateway | 租约验证、模式检查 |
| LingTu ControlService | LeaseManager、SafetyGate |
| LingTu planner_node | 超时保护、LERa 失败恢复 |
| localPlanner | 避障、急停 |
| han_dog_bridge | 200ms 看门狗 |
| Dog Board Arbiter | 遥控器最高优先级 |

---

## 语义导航内部流程

```
/nav/semantic/instruction (JSON)
    │
    ▼
planner_node._instruction_callback()
    │
    ├─ 解析 JSON: instruction, explore_if_unknown, language
    │
    ▼
goal_resolver.resolve(instruction)
    │
    ├─ Fast Path (System 1, ~0.17ms)
    │   └─ 场景图关键词匹配 + 空间推理
    │   └─ 置信度融合: label 35% + CLIP 35% + detector 15% + spatial 15%
    │   └─ 阈值 0.75 → PASS: 直接返回 PoseStamped
    │
    ├─ AdaNav 熵触发
    │   └─ score_entropy > 1.5 且 confidence < 0.85 → 强制 Slow Path
    │
    ├─ Slow Path (System 2, ~2s)
    │   └─ ESCA 选择性接地: 200→~15 objects (92.5% token reduction)
    │   └─ LLM 推理 (kimi/openai/claude/qwen)
    │   └─ OmniNav room hint → 层级子目标
    │
    └─ 未找到目标 + explore_if_unknown=true
        │
        ▼
    FrontierScorer / TSG 探索
        │
        ├─ frontier 打分 (4 权重):
        │   distance 0.25 + novelty 0.35 + language 0.20 + grounding 0.20
        │
        ├─ TSG Layer 1 快速预筛 (~1ms):
        │   IG(n) = S_sem × N × U (信息增益)
        │
        ├─ 选择最佳 frontier → PoseStamped
        │
        └─ 最多切换 N 次目标 (max_explore_steps)
            导航模式: 20, 探索模式: 50
```

---

## LingTuGrpcBridge 支持的任务类型

| 任务类型 | TaskType 枚举 | Bridge 方法 | 说明 |
|----------|--------------|-------------|------|
| 普通导航 | `TASK_TYPE_NAVIGATE` (1) | `start_navigation()` | 点到点导航 |
| 语义导航 | `TASK_TYPE_SEMANTIC_NAV` (6) | `start_semantic_nav()` | 语义目标 + 探索 |
| 建图 | `TASK_TYPE_MAPPING` (2) | `start_mapping()` | 手动建图 |
| 路径跟随 | `TASK_TYPE_FOLLOW_PATH` (3) | `start_follow_path()` | 预定义路径 |
| 人员跟随 | `TASK_TYPE_FOLLOW_PERSON` (7) | `start_follow_person()` | 跟随行人 |

---

## Systemd 服务依赖

```
nav-lidar        (LiDAR 驱动)
    ↓
nav-slam         (SLAM 建图/定位)
    ↓
nav-autonomy     (地形分析 + 局部规划)
    ↓
nav-planning     (全局规划)
nav-semantic     (语义感知 + 语义规划)    ← 语义导航任务需要
nav-driver       (底盘驱动)
nav-grpc         (gRPC Gateway)           ← 外部触发需要
```

**启动语义导航的最小服务集**:
```bash
sudo systemctl start nav-lidar nav-slam nav-autonomy nav-semantic nav-driver nav-grpc
```

---

## 相关文档

- [ARCHITECTURE.md](./ARCHITECTURE.md) — 硬件双板架构、数据流、安全体系
- [TOPIC_CONTRACT.md](./TOPIC_CONTRACT.md) — ROS2 话题接口契约
- [../AGENTS.md](../AGENTS.md) — 导航节点详细话题/参数
- [../../config/semantic_planner.yaml](../../config/semantic_planner.yaml) — 语义规划配置
- [../../config/semantic_exploration.yaml](../../config/semantic_exploration.yaml) — 探索模式配置
