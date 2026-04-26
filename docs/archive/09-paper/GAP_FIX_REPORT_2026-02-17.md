# 缺陷修复报告（2026-02-17）

针对“层次推理浅、无 View 节点、缺执行反馈重规划”等问题，本次已完成以下代码级修复。

---

## 已修复

### 1) Scene Graph 新增 View 层

- 新增 `ViewNode` 数据结构，记录关键视角位置与观测对象。
- 在感知处理流程中，每帧更新后调用 `record_view()` 维护视角节点。
- 场景图 JSON 新增：`views` 字段。
- 层次边新增：`room -> view`、`view -> object`。

关键文件：

- `src/semantic_perception/semantic_perception/instance_tracker.py`
- `src/semantic_perception/semantic_perception/perception_node.py`

### 2) 多视角特征融合（非覆盖）

- 追踪器中 CLIP 特征从“覆盖更新”改为“EMA 融合 + 归一化”，降低单帧误检对特征漂移的影响。

关键文件：

- `src/semantic_perception/semantic_perception/instance_tracker.py`

### 3) SG-Nav 层次门控增强

- 在子图评分后增加 room-level gating：
  - 非 room 子图（group/view）分数融合对应 room 分数
  - 强化“先 room 再细化”的层次约束
- 新增参数：`exploration.sgnav.room_gate_weight`

关键文件：

- `src/semantic_planner/semantic_planner/sgnav_reasoner.py`
- `src/semantic_planner/semantic_planner/planner_node.py`
- `config/semantic_planner.yaml`

### 4) 失败反馈驱动重规划

- 子目标重试耗尽后，不再直接终止；触发 `REPLANNING`。
- 将失败子目标与失败原因注入 decomposition prompt，让 LLM 基于当前场景重新生成后续 plan。

关键文件：

- `src/semantic_planner/semantic_planner/planner_node.py`

---

## 回归结果

- `pytest src/semantic_perception/test -q` → `50 passed`
- `pytest src/semantic_planner/test -q` → `121 passed, 4 skipped`

---

## 仍待完成（下一阶段）

1. View 节点与 VLM 图像精细确认的直接联动（当前仅构图，未进入 slow-path 图像筛选闭环）
2. 公开基准与真机的统一评测协议（SR/SPL/NE/时间）
3. 经验记忆（失败案例回灌）与长期学习闭环
