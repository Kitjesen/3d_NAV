# Semantic Planner — 模块组织索引

> 文件均位于 `src/semantic_planner/semantic_planner/`
> 共 39 个 .py 文件，18 071 行
> 本文档说明各文件的功能归属，便于导航和维护。

---

## 根节点 (2)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `planner_node.py` | 497 | ROS2 主节点：订阅 /nav/semantic/instruction，协调 Fast-Slow 目标解析、LERa 恢复、EpisodicMemory 更新 |
| `planner_state.py` | 20 | 枚举：PlannerState (IDLE / PLANNING / EXECUTING / STUCK / ERROR) |

---

## Agent — LLM Agent + Skill + MCP (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `agent_node.py` | 664 | ROS2 Agent 节点：多轮对话、工具调用、会话管理 |
| `skill_registry.py` | 547 | @skill 装饰器 + 注册中心；动态发现并注册导航技能 |
| `mcp_server.py` | 284 | MCP 协议服务端：将导航 ROS2 服务暴露为 MCP 工具 |

---

## Goal — 目标解析 (4)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `goal_resolver.py` | 376 | Fast-Slow 协调器：路由 Fast/Slow Path，AdaNav 熵触发，OmniNav 层次子目标 |
| `fast_path.py` | 939 | System 1 快速路径：关键词 + 场景图匹配，置信度融合 (label 35% + CLIP 35% + det 15% + spatial 15%)，目标 <200ms |
| `slow_path.py` | 945 | System 2 慢速路径：ESCA 选择性接地 (200→~15 objects)，LLM 推理，OmniNav 房间提示，目标 ~2s |
| `adacot.py` | 208 | AdaCoT 自适应推理路由：7 维规则 + 熵判断，输出 FAST/SLOW/AUTO 决策 |

---

## Execution — 执行 + 导航动作 (5)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `action_executor.py` | 382 | 动作原语执行器；LERa 3 步失败恢复 (retry / expand / requery / abort) |
| `bbox_navigator.py` | 379 | 基于检测框的视觉伺服导航；发布 /nav/way_point |
| `vlm_bbox_query.py` | 365 | 向 VLM 查询目标包围框；多轮追问 + 置信过滤 |
| `person_tracker.py` | 480 | 行人跟踪器：卡尔曼滤波 + ReID 特征匹配，跟随模式 |
| `exploration_strategy.py` | 125 | 探索策略：frontier 选择 + 覆盖优先级调度 |

---

## Task — 任务分解 (2)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `task_decomposer.py` | 337 | SayCan 风格任务分解：自然语言指令 → 有序子目标序列 |
| `task_rules.py` | 871 | 规则库：场所-技能映射、常见任务模板、约束条件 |

---

## Memory — 空间与情节记忆 (4)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `episodic_memory.py` | 205 | ReMEmbR 风格情节记忆：500 条 FIFO，按文本/位置检索，输出 LLM 上下文 |
| `topological_memory.py` | 762 | 拓扑地图：TopoNode + FSR-VLN Jaccard 加权边 + VLingMem 区域摘要 |
| `tagged_locations.py` | 138 | 带标签的位置记录：持久化命名地点，支持 "去上次的充电站" |
| `semantic_prior.py` | 615 | 语义先验：场所 CLIP 描述、predict_room_type_from_labels()、房间-物体共现统计 |

---

## Frontier — Frontier 探索与场景推理 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `frontier_scorer.py` | 596 | Frontier 评分：MTU3D 接地潜力 + L3MVN 自然语言描述 + TSP 排序 |
| `frontier_types.py` | 402 | 数据类型：FrontierNode、FrontierCluster、ScoredFrontier |
| `sgnav_reasoner.py` | 801 | SGNav 场景图推理：多视角 ObservationRecord 累积 + 一致性投票 |

---

## LLM — 大模型客户端 + Prompt (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `llm_client.py` | 741 | 多后端 LLM 客户端：kimi / openai / claude / qwen，自动 fallback，重试，流式 |
| `prompt_templates.py` | 715 | Prompt 模板：H-CoT 4 步推理、frontier_descriptions、explored_summaries |
| `chinese_tokenizer.py` | 508 | jieba 分词封装：中文关键词提取、停用词过滤、与 Fast Path 集成 |

---

## Mixin — Planner 内部 Mixin (8)

> 这些文件仅供 `planner_node.py` 通过多重继承组合使用，不对外暴露。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `init_mixin.py` | 632 | 初始化 Mixin：ROS2 订阅/发布声明，参数读取，子模块实例化 |
| `nav2_mixin.py` | 872 | Nav2 接口 Mixin：ActionClient (NavigateToPose)，目标发送/取消/状态查询 |
| `subgoal_mixin.py` | 605 | 子目标推进 Mixin：航点队列管理，OmniNav 层次子目标切换 |
| `bbox_nav_mixin.py` | 223 | 包围框 Mixin：BBoxNavigator 生命周期管理，视觉伺服触发条件 |
| `operational_mixin.py` | 266 | 运营 Mixin：健康检查，metrics 上报，watchdog |
| `goal_mixin.py` | 382 | 解析 Mixin：goal_resolver 调用封装，结果路由到 Nav2 / frontier / bbox |
| `callbacks_mixin.py` | 302 | 回调 Mixin：scene_graph / odometry / instruction ROS2 回调实现 |
| `state_mixin.py` | 373 | 状态 Mixin：PlannerState FSM 转换，/nav/semantic/status 发布 |

---

## Viz — 可视化 (1)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `rerun_viewer.py` | 443 | Rerun SDK 可视化：场景图、轨迹、frontier、置信度实时渲染 |

---

## Utils — 通用工具 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `implicit_fsm_policy.py` | 298 | 隐式 FSM 导航策略 (LOVON 风格)：无显式状态机的策略执行 |
| `voi_scheduler.py` | 315 | VOI (Value of Information) 调度：决定何时触发感知、LLM 查询 |
| `room_object_kg.py` | 457 | 房间-物体知识图谱：先验共现关系，支持 predict_room_type 和 ESCA 过滤 |
