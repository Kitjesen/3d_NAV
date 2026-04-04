# Memory — 模块组织索引

> 文件均位于 `src/memory/`
> 共 30 个 .py 文件（含子目录），约 10 436 行
> 本文档说明各文件的功能归属，便于导航和维护。

---

## 概览：记忆层四大子系统

```
记忆层 (memory/)
├── modules/      — Module-First 封装（对接 core.Module 框架）
├── knowledge/    — 知识图谱 + 语义先验 + 贝叶斯置信网络
├── spatial/      — 空间记忆（物体在哪、机器人去过哪）
├── storage/      — 持久化后端（SQLite + 时序存储）
├── scheduling/   — VoI 调度（何时再感知 / 慢推理）
└── logging/      — 任务日志（导航历史记录）
```

---

## Module 层 — `modules/` (6)

> 这六个文件是 Module-First 架构的记忆入口，对接 `core.Module` 框架。
> 算法实现在 `spatial/` 和 `knowledge/`，Module 只负责端口 + 生命周期。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `modules/semantic_mapper_module.py` | 320 | **SemanticMapperModule**：订阅实时 `SceneGraph`，驱动 `RoomObjectKG` + `TopologySemGraph` 增量更新，发布 `map_updated` 信号 |
| `modules/vector_memory_module.py` | 272 | **VectorMemoryModule**：CLIP 特征 + ChromaDB 向量检索，`query_location(text)` 返回最近似的历史场景位置；无 ChromaDB 时自动降级到 numpy 暴力搜索 |
| `modules/episodic_module.py` | 90 | **EpisodicMemoryModule**：时空情节记忆封装，把 `(timestamp, pose, scene_hash)` 写入 `EpisodicMemory` |
| `modules/tagged_locations_module.py` | 134 | **TaggedLocationsModule**：命名地点存取（用户/Agent 用 `tag_location(name)` 保存当前位置），JSON 持久化，支持模糊匹配 |
| `modules/topological_module.py` | 117 | **TopologicalMemoryModule**：拓扑记忆封装，维护已探索位置图、frontier、关键帧列表 |
| `modules/temporal_memory_module.py` | 398 | **TemporalMemoryModule**：带时间索引的场景快照，支持自然语言时间查询（"上午去过的地方"）和相对时间窗口检索 |

---

## Knowledge 层 — `knowledge/` (6)

> 静态与动态知识：室内本体、物体-房间共现统计、语义先验、神经符号信念网络。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `knowledge/knowledge_graph.py` | 568 | **IndustrialKnowledgeGraph**：室内场景本体，物体-属性-关系三元组，安全约束，可供性，开放词汇映射；加载自 `knowledge_data.py` |
| `knowledge/knowledge_data.py` | 1 944 | **知识图谱数据**：预定义物体分类（导航相关 ~125 类）、属性词典、房间-物体先验共现表 `ROOM_OBJECT_PRIORS`；仅数据，无逻辑 |
| `knowledge/room_object_kg.py` | 457 | **RoomObjectKG**：在线运行时房间-物体共现 KG；增量建图统计、JSON 持久化/加载、与 `SemanticPriorEngine` 对接，用于 GoalResolver Fast Path |
| `knowledge/semantic_prior.py` | 615 | **SemanticPriorEngine**：语义先验推断引擎 — 房间↔物体联想分数、拓扑感知探索评分、VoI 先验，供 FrontierScorer 和 GoalResolver 使用 |
| `knowledge/belief/network.py` | 652 | **KGBeliefNetwork**：神经符号 GCN，将场景图节点的几何+语义特征 → Beta(α,β) 存在性置信度；含合成数据集生成和 PyTorch 训练循环（torch 可选） |
| `knowledge/belief/propagation.py` | 601 | **BeliefPropagationMixin**：Loopy BP 在场景图上推断语义标签后验概率，多阶段迭代收敛；供 `InstanceTracker` 的 BA-HSG 贝叶斯置信融合使用 |

---

## Spatial 层 — `spatial/` (6)

> 纯算法：物体在哪里、机器人走过哪里、房间结构如何。不依赖 `core.Module`。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `spatial/topology_graph.py` | 536 | **TopologySemGraph (TSG)**：房间级拓扑语义图；ViewNode + RoomNode，Dijkstra 最短路，`query_by_position` / `query_by_label`，信息增益 frontier 评分 |
| `spatial/topology_types.py` | 460 | **TSG 数据类型**：ViewNode、RoomNode、TopoEdge 的 dataclass 定义、常量（UNKNOWN_ROOM 等）、纯工具函数（从 `topology_graph` 拆出，避免循环导入） |
| `spatial/topological.py` | 762 | **TopologicalMemory**：探索位置序列、关键帧列表、frontier 优先队列、CLIP 节点匹配；`add_observation` / `find_similar_place` / `get_exploration_frontier` |
| `spatial/room_manager.py` | 959 | **RoomManagerMixin**：区域 DBSCAN 聚类、房间/楼层动态合并分裂、物体-房间归属、拓扑边维护、前沿提取；从 `instance_tracker.py` 拆出的大型 Mixin |
| `spatial/episodic.py` | 205 | **EpisodicMemory**：轻量时空情节记忆（ReMEmbR/VLingMem 风格）；`(timestamp, pose, scene_hash, context)` 三元组，numpy 存储，时间窗口检索 |
| `spatial/tagged_locations.py` | 138 | **TaggedLocationStore**：命名地点的持久化存储和模糊查找；JSON 文件读写，支持按距离排序和名称模糊匹配（jieba + 词语前缀） |

---

## Storage 层 — `storage/` (2)

> 持久化后端。适配自 DimOS（已去除 RxPy 依赖）。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `storage/sqlite_store.py` | 265 | **SQLiteStore**：SQLite 场景存储后端；支持 `:memory:` 内存模式和文件路径；JSON 序列化，带索引的按时间/标签查询 |
| `storage/timeseries_store.py` | 318 | **TimeSeriesStore**：时序存储抽象基类（去除 RxPy）；保留 seek / duration / loop 迭代、相对时间窗口查询；适合录制场景快照用于离线回放 |

---

## Scheduling 层 — `scheduling/` (1)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `scheduling/voi_scheduler.py` | 315 | **VoIScheduler**：VoI（Value of Information）驱动的快慢推理与再感知调度器（BA-HSG §3.4.4）；Shannon 熵量化信息价值，效用函数选择 continue / reperceive / slow_reason 三种动作 |

---

## Logging 层 — `logging/` (1)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `modules/mission_logger_module.py` | 210 | **MissionLoggerModule**：导航任务历史记录 (Module 模式)；记录每次任务的生命周期（开始/结束时间、轨迹、里程、结果），持久化到 JSON 文件 |

---

## 层级依赖关系

```
core.Module (框架层)
    ↓
modules/          — SemanticMapperModule / VectorMemoryModule / ...
    ↓ 调用
spatial/          — EpisodicMemory / TopologySemGraph / TaggedLocationStore / ...
knowledge/        — RoomObjectKG / SemanticPriorEngine / BeliefPropagationMixin / ...
    ↓ 持久化
storage/          — SQLiteStore / TimeSeriesStore

scheduling/       — VoIScheduler  ← 被 semantic/planner/ 调用（不在 Module 链上）
modules/          — MissionLoggerModule ← 已接入 NavigationModule.mission_status 端口
```

### 外部依赖关系

| 被谁调用 | 调用的 memory 组件 |
|---------|-------------------|
| `semantic/planner/goal_resolver.py` | `RoomObjectKG`、`TaggedLocationStore`、`VoIScheduler` |
| `semantic/planner/semantic_planner_module.py` | `VectorMemoryModule`（`on_system_modules` 注入）、`TaggedLocationsModule` |
| `semantic/perception/instance_tracker.py` | `RoomManagerMixin`（Mixin 继承）、`BeliefPropagationMixin` |
| `nav/navigation_module.py` | 无直接依赖（通过 Blueprint auto_wire 间接连接） |

> **设计原则**: `modules/` 只依赖 `spatial/` 和 `knowledge/`；`spatial/` 和 `knowledge/` 不依赖 `core.Module`；`storage/` 是纯后端，不依赖任何上层。
