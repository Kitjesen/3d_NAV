# 我们的方法（当前实现版，2026-02-17）

本文档描述的是**当前代码已经落地的实现**，不是理想化方案。

适用范围：`src/semantic_perception` + `src/semantic_planner` 当前主分支。

---

## 1. 一句话总览

我们当前系统是一个 ROS2 语义导航闭环：

1. 感知侧构建层次 3D 语义场景图（objects/relations/rooms/groups/subgraphs）
   - 并维护关键视角层（views）用于后续精细推理扩展
2. 规划侧先做 Fast/Slow 目标解析（GoalResolver）
3. 探索阶段走 SG-Nav 对齐策略（子图推理 + frontier 插值）
4. 执行阶段由分解子目标驱动（NAVIGATE/FIND/APPROACH/VERIFY/EXPLORE/BACKTRACK）
5. 支持隐式 FSM 模式：`s_{t+1} = f_theta(obs_t, s_t, instruction)`
6. 子目标重试耗尽后支持基于执行反馈的 LLM 重规划

---

## 2. 感知层怎么做

### 2.1 输入与输出

- 输入：RGB-D、检测结果、机器人位姿
- 输出：`scene_graph`（JSON，发布给 planner）

主文件：

- `src/semantic_perception/semantic_perception/perception_node.py`
- `src/semantic_perception/semantic_perception/instance_tracker.py`

### 2.2 场景图结构（当前）

当前 `get_scene_graph_json()` 输出包含：

- `objects`: 物体实例（id/label/position/score/detection_count）
- `relations`: 空间关系（near/left/right/front/behind/...）
- `regions`: 区域聚类
- `rooms`: room 层节点
- `groups`: group 层节点
- `hierarchy_edges`: room->group->object 层次边
- `subgraphs`: 供规划侧做子图推理的摘要
- `views`: 关键视角节点（view_id/room_id/position/object_ids/key_labels）
- `summary`: 文本摘要

对应实现：`src/semantic_perception/semantic_perception/instance_tracker.py`

- `compute_groups()`
- `compute_rooms()`
- `_build_hierarchy_edges()`
- `_build_view_edges()`
- `_build_subgraphs()`
- `get_scene_graph_json()`

这部分对应 SG-Nav/ConceptGraphs 的“层次图 + 子图输入”思想，但实现是工程化简化版。

---

## 3. 规划层怎么做

主文件：`src/semantic_planner/semantic_planner/planner_node.py`

### 3.1 任务分解

分解器：`src/semantic_planner/semantic_planner/task_decomposer.py`

- 简单指令优先走规则分解：`decompose_with_rules()`
- 复杂指令走 LLM 分解：`build_decomposition_prompt()` + `parse_decomposition_response()`

输出为 `TaskPlan`，由 `SubGoal` 序列驱动执行。

### 3.2 Goal Resolver（Fast/Slow）

主文件：`src/semantic_planner/semantic_planner/goal_resolver.py`

#### Fast Path

- `fast_resolve()` 对场景图候选做多源融合评分：
  - label match
  - detector score
  - spatial hint
  - 可用时加 CLIP 相似度
- 分数低于阈值则转 Slow Path

#### Slow Path

- `resolve()` 先执行 `_selective_grounding()`（ESCA 风格过滤）
- 再调用 LLM 推理目标
- 解析响应用 `_extract_json()`，支持 markdown fenced JSON

#### 失败容错

- `_call_with_fallback()`：主 LLM 失败时自动切备用 LLM
- `generate_exploration_waypoint()`：目标未知时给探索方向
- `vision_grounding()`：可选图像确认（OpenAI Vision 路径）

---

## 4. SG-Nav 对齐探索怎么做

### 4.1 入口

在 `planner_node.py`，当 `exploration.strategy == "sg_nav"`：

- `_handle_explore_result()` 进入 SG-Nav 分支
- `_generate_sgnav_waypoint()` 执行子图推理 + frontier 选择

### 4.2 具体流程

1. 从 costmap 提取 frontiers（`frontier_scorer.extract_frontiers`）
2. 先算基础 frontier 分（`frontier_scorer.score_frontiers`）
3. `SGNavReasoner.reason_subgraphs()` 对 room/group 子图评分（启发式 + 可选LLM）
4. `SGNavReasoner._apply_room_level_gating()` 执行 room->group/view 层次门控
5. `SGNavReasoner._interpolate_to_frontier()` 将子图分数按距离衰减插值到 frontier
6. 融合基础分和子图信号，选最优 frontier
7. 若 SG-Nav 失败，按回退链路执行：`sg_nav -> frontier -> llm explore`

核心代码：

- `src/semantic_planner/semantic_planner/sgnav_reasoner.py`
- `src/semantic_planner/semantic_planner/frontier_scorer.py`
- `src/semantic_planner/semantic_planner/planner_node.py`
- `src/semantic_planner/semantic_planner/prompt_templates.py`（`build_sgnav_subgraph_prompt`）

---

## 5. Re-perception（目标可信度）怎么做

目标：降低假阳性目标导致的错误导航。

机制：

1. planner 在导航前触发 `_sgnav_reperception_check()`
2. reasoner 的 `evaluate_target_credibility()` 使用以下信号更新可信度：
   - 场景图里是否持续看到目标标签
   - 目标 score
   - 当前路径置信度
   - 可选视觉确认（vision grounding）
3. 低于阈值时拒绝当前目标，转回探索

对应代码：

- `src/semantic_planner/semantic_planner/planner_node.py`
- `src/semantic_planner/semantic_planner/sgnav_reasoner.py`

---

## 6. 执行层怎么做

执行器：`src/semantic_planner/semantic_planner/action_executor.py`

当前原语：

- `generate_navigate_command()`
- `generate_approach_command()`
- `generate_look_around_command()`
- `generate_verify_command()`
- `generate_backtrack_command()`

拓扑记忆：`src/semantic_planner/semantic_planner/topological_memory.py`

- `update_position()` 维护节点/边
- `query_by_text()` 文本检索历史节点
- `get_backtrack_position()` 支持回溯
- `get_least_visited_direction()` 支持探索偏置

---

## 7. 隐式 FSM（LOVON 风格）怎么做

主文件：`src/semantic_planner/semantic_planner/implicit_fsm_policy.py`

### 7.1 目标

将状态转移从硬编码规则扩展为参数化策略：

`s_{t+1} = f_theta(obs_t, s_t, instruction)`

### 7.2 当前实现

- 状态空间：`MISSION_STATES = [success, searching_1, searching_0, running]`
- 搜索记忆：`SEARCH_STATES = [had_searching_1, had_searching_0]`
- 输入特征：匹配度、置信度、方位/尺寸 proxy、指令变化、历史状态 one-hot
- 输出：
  - `mission_state_out`
  - `search_state_out`
  - `motion_vector`
- 支持从 `.npz` 加载参数；无权重时用默认参数

planner 接入点：

- 参数：`fsm.mode`, `fsm.implicit.weights_path`, `fsm.implicit.strict`
- 在 `_resolve_goal()` 中调用 `_apply_implicit_fsm_transition()`
- 当前可切换：`implicit` / `explicit`

注意：这是工程可运行的隐式 FSM 版本，不等同 LOVON 原论文的完整训练管线复现。

---

## 8. 当前回退与鲁棒性策略

### 8.1 探索回退链

- `sg_nav` 不可用 -> `frontier` -> `llm explore`

### 8.2 目标解析回退链

- Fast Path 失败 -> Slow Path
- 主 LLM 失败 -> 备用 LLM
- Vision 不可用 -> 文本/图结构链路继续工作

### 8.3 任务级鲁棒性

- 子目标支持重试
- 支持 backtrack、replan（包含失败反馈驱动的 LLM 重规划）
- 支持 cancel

---

## 9. 现在“已实现”和“未完成”边界

### 已实现（代码已落地）

- 层次场景图发布
- Fast/Slow 目标解析
- SG-Nav 子图推理 + frontier 插值
- re-perception 可信度过滤
- 隐式 FSM 可切换模式

### 仍需补齐（投稿必须）

- 公开基准的端到端指标闭环（SR/SPL/NE/时间）
- 真实四足平台的系统化实验报告与统计显著性
- 对比与消融实验脚本、日志、可复现实验包

---

## 10. 投稿时建议怎么写

建议表述：

- “SG-Nav inspired hierarchical scene-graph reasoning with frontier interpolation on a quadruped ROS2 stack.”
- “An integrated system with fast/slow resolution, re-perception, and optional implicit FSM transition.”

避免表述：

- “完全复现某论文全部细节”
- “全面超越 SOTA” （除非有完整可复现实验）
