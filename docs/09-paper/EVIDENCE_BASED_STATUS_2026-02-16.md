# 论文级语义导航 - 证据化现状报告

**日期**: 2026-02-16  
**范围**: `src/semantic_planner` + `src/semantic_perception` + `config/semantic_*.yaml` + `docs/09-paper`  
**目的**: 给论文写作提供可追溯、可复验的工程事实，区分「已验证」与「待验证」。

---

## 1) 已验证实现（代码层）

### 1.1 规划层（semantic_planner）

- Fast/Slow 双路径目标解析已实现：`src/semantic_planner/semantic_planner/goal_resolver.py`
  - Fast Path: 多源融合评分（label/clip/detector/spatial）
  - Slow Path: selective grounding 后调用 LLM
- 任务分解（规则 + LLM）已实现：`src/semantic_planner/semantic_planner/task_decomposer.py`
- 动作原语执行（look around / approach / backtrack 等）已实现：`src/semantic_planner/semantic_planner/action_executor.py`
- 拓扑记忆更新与回溯接口已实现：`src/semantic_planner/semantic_planner/topological_memory.py`
- VLM 验证流程已接入主状态机：`src/semantic_planner/semantic_planner/planner_node.py`

### 1.2 感知层（semantic_perception）

- 开放词汇检测、CLIP 编码、实例追踪、场景图发布链路具备实现：
  - `src/semantic_perception/semantic_perception/perception_node.py`
  - `src/semantic_perception/semantic_perception/yolo_world_detector.py`
  - `src/semantic_perception/semantic_perception/clip_encoder.py`
  - `src/semantic_perception/semantic_perception/instance_tracker.py`

---

## 2) 已验证测试结果（本地复跑）

执行命令：

```bash
pytest src/semantic_planner/test -q
pytest src/semantic_perception/test -q
```

结果：

- `src/semantic_planner/test`: **113 passed, 4 skipped**
- `src/semantic_perception/test`: **48 passed**

说明：当前仓库状态下，planner/perception 两侧单测均可通过。

---

## 3) 关键差距与风险（代码走查结论）

### 3.1 Frontier 接入现状（已接入，待系统级验证）

- `frontier_scorer.py` 已接入 `planner_node.py` 探索主流程：`strategy=frontier` 时优先使用 Frontier 评分，失败后回退到 LLM 探索建议。
- 新增配置参数：`exploration.costmap_topic`、`exploration.frontier_*` 权重与阈值。
- 新增纯逻辑测试：`test_exploration_strategy.py`（Frontier 场景提取与目标生成）。

### 3.2 配置与实现存在漂移风险

- `config/semantic_planner.yaml` 中存在 `goal_resolution.vision_threshold`，但节点代码主要使用 `vision.verify_threshold`。
- 建议统一参数命名并清理未生效项，避免实验配置与运行行为不一致。

### 3.3 论文数据主张需要分级

以下数值在现有仓库中缺少可复现脚本或原始实验记录，暂不应写成“已实证结论”：

- Fast Path `0.17ms`
- `90%` Fast Path hit rate
- `92.5%` token reduction
- `82.3%` 端到端成功率
- `11 FPS` Jetson 实测

建议标注为：

- `Claimed`: 文档主张但未附复现实验
- `Measured`: 有脚本、日志、数据表
- `Published`: 有对外版本或论文附录可追溯

---

## 4) 论文写作建议（立刻可执行）

1. 在 `docs/09-paper/04_experiments.md` 增加“证据等级”列（Claimed/Measured/Published）。
2. 为每个核心指标补充复现实验入口（命令 + 输入数据 + 输出日志路径）。
3. 把“frontier 已接入但缺 E2E 指标闭环”写入 Limitations，避免 reviewer 高估当前验证强度。
4. 在附录明确：当前通过的是模块单测，不等价于真实机端到端成功率。

---

## 5) 建议的最小补充实验包（用于论文可审稿）

- **E1 Fast/Slow 路由统计**: 统计 Fast 命中率、平均延迟、P95 延迟。
- **E2 Selective Grounding 消融**: 对比过滤前后 token、目标保留率、最终成功率。
- **E3 Frontier 接入前后对比**: 探索步数、找到目标率、路径长度。
- **E4 实机性能**: Jetson 平台 FPS/CPU/GPU 占用与端到端任务成功率。

---

## 6) 结论

当前代码已具备“论文级系统框架”的主体实现和稳定单测基础，适合继续推进论文；但若要把文档中的性能数字作为正式结论，仍需补齐可复现实验证据与数据资产管理。
