# 深度缺口分析与继续工作建议

> 分析日期: 2026-02-13  
> 更新: 2026-02-17 — 完整论文初稿完成，与真实论文 (SG-Nav NeurIPS 2024, FSR-VLN) 深度对标  
> 更新: 2026-02-18 — 离线全流程验证完成 (66/66 测试通过), 实验章节用真实数据重写  
> 基于: HONEST_REVIEW.md, LITERATURE_RESEARCH.md, 实际代码审查, SG-Nav论文全文分析

---

## 0. 执行摘要

**当前状态**: 
- ✅ 代码实现: 5个创新点全部已实现 (HSG构建、Fast-Slow推理、Re-perception、Frontier视觉评分、边缘部署)
- ✅ 论文初稿: 完整5章结构已撰写 (Abstract+Intro, Related Work, Method, Experiments, Conclusion)
- ✅ 离线验证: 66/66 测试通过, 算法正确性已确认
- ✅ 量化数据: Fast Path 100% L1/L2 命中率, 多假设 100% SR, VoI 自适应调度已验证
- ❌ 真机实验: SR/SPL 指标需真机 (所有导航层面的数据)
- ❌ 论文图表: 7张图待制作 (系统图、定性可视化等)

**当前瓶颈**: 从离线验证→完整论文的关键缺口是 **真机导航实验** 和 **可视化图表**。

**方法学升级 (2026-02-17~18)**: 整合 reviewer 建议，将工程策略升级为可形式化的算法贡献：
- ✅ BA-HSG 信念模块 **[已实现]**: Beta(α,β) 存在性 + Gaussian σ² 位置不确定性 + 复合可信度 + 图扩散 (`instance_tracker.py`)
- ✅ 风险敏感多假设目标规划 **[已实现]**: `TargetBeliefManager` + 贝叶斯后验 + 期望代价选择 + 到达验证重选 (`goal_resolver.py`)
- ✅ VoI 推理调度 **[已实现]**: `VoIScheduler` 信息价值驱动的 continue/reperceive/slow_reason 调度 (`voi_scheduler.py`, `planner_node.py`)
- ✅ Beta-based 可信度评估 **[已实现]**: 融合 belief 字段的新证据模型 (`sgnav_reasoner.py`)
- 📝 新增论文文件: `03b_belief_graph.md` (完整形式化)
- 📝 新增代码文件: `voi_scheduler.py` (VoI 调度器独立模块)

---

## I. 代码实现进展 (已完成)

### 创新点 1: 在线层次场景图 (HSG)

| 模块 | 状态 | 文件 |
|------|------|------|
| DBSCAN 空间聚类 (ε=3.0m) | ✅ | `instance_tracker.py` → `compute_regions()` |
| 质量感知 EMA CLIP 融合 | ✅ | `instance_tracker.py` → `_fuse_feature()` |
| 规则推理房间类型 (8类) | ✅ | `instance_tracker.py` → `infer_room_type()` |
| 可选 LLM 房间命名 | ✅ | `perception_node.py` → `_make_room_llm_namer()` |
| 语义分组 (5个类别族) | ✅ | `instance_tracker.py` → `compute_groups()` |
| Room CLIP embedding | ✅ | `instance_tracker.py` → `compute_rooms()` |

### 创新点 2: Fast-Slow 层次推理

| 模块 | 状态 | 文件 |
|------|------|------|
| Fast Path 多源融合 (4因子) | ✅ | `goal_resolver.py` → `fast_resolve()` |
| Slow Path 选择性 grounding | ✅ | `goal_resolver.py` → `resolve()` |
| 层次 CoT Prompt (5步) | ✅ | `prompt_templates.py` → `build_goal_resolution_prompt()` |
| 中文 jieba 分词支持 | ✅ | `goal_resolver.py` → `_parse_instruction_roles()` |

### 创新点 3: 图式 Re-perception

| 模块 | 状态 | 文件 |
|------|------|------|
| 到达后 Re-perception | ✅ | `planner_node.py` → Nav2 result callback |
| 连续 Re-perception (每2m) | ✅ | `planner_node.py` → `_trigger_continuous_reperception()` |
| 可信度 EMA 追踪 | ✅ | `sgnav_reasoner.py` → `evaluate_target_credibility()` |
| 假阳性惩罚 + 拒绝机制 | ✅ | `sgnav_reasoner.py` (δ_fp=0.2, τ_reject=0.25) |

### 创新点 4: 视觉增强 Frontier 探索

| 模块 | 状态 | 文件 |
|------|------|------|
| 5因子评分 (距离/新颖度/语言/场景图/视觉) | ✅ | `frontier_scorer.py` → `score_frontiers()` |
| CLIP 方向观测缓存 (8 bins) | ✅ | `frontier_scorer.py` → `update_directional_observation()` |
| 子图-Frontier 插值 | ✅ | `sgnav_reasoner.py` → `interpolate_to_frontier()` |
| 房间级门控 | ✅ | `sgnav_reasoner.py` → `_apply_room_level_gating()` |

### 创新点 5: 四足边缘部署

| 模块 | 状态 | 文件 |
|------|------|------|
| Laplacian 模糊滤波 | ✅ | `perception_node.py` |
| YOLO-World 开放词汇检测 | ✅ | `perception_node.py` |
| Nav2 集成 + 连续反馈 | ✅ | `planner_node.py` |
| 多步指令分解 + 闭环 | ✅ | `task_decomposer.py`, `planner_node.py` |

---

## II. 论文撰写进展

### 完成的章节

| 章节 | 文件 | 页数 | 公式 | 表格 | 算法 |
|------|------|------|------|------|------|
| Abstract + Introduction | `01_abstract_intro.md` | ~2.5 | 0 | 0 | 0 |
| Related Work | `02_related_work.md` | ~2.5 | 0 | 1 (系统对比) | 0 |
| Method | `03_method.md` | ~4 | 15+ | 2 (参数表) | 1 (Algorithm 1) |
| Experiments + Results + Analysis | `04_experiments.md` | ~4 | 1 (SPL) | 10 | 0 |
| Conclusion + References | `05_conclusion_refs.md` | ~2 | 0 | 0 | 0 |
| **合计** | | **~15页** | **16+** | **13** | **1** |

### 与 SG-Nav (NeurIPS 2024) 的章节对标

| 要素 | SG-Nav | 我们 | 状态 |
|------|--------|------|------|
| 动机对比图 (Figure 1) | ✅ | ❌ | TODO: 需设计 FSR-VLN离线 vs 我们在线 对比图 |
| Pipeline 架构图 (Figure 2) | ✅ | ❌ | TODO: Perception→HSG→Planning→Execution 流程图 |
| 核心机制示意图 (Figure 3) | ✅ (边生成) | ❌ | TODO: Re-perception 可信度曲线图 |
| Per-category SR 分析 (Figure 4) | ✅ | ❌ | TODO: 需真机数据 |
| 效率分析图 (Figure 5) | ✅ (时间复杂度) | ❌ | TODO: Fast/Slow路由比例分析 |
| 定性导航案例 (Figure 6-8) | ✅ | ❌ | TODO: 需真机截图 + 场景图演变 |
| 主实验表 (Table 1) | ✅ (9个baseline) | ✅ 框架 (3 baseline + 5 ablation) | TBD: 需跑实验填数据 |
| 消融实验表 (Table 2-5) | ✅ (4个表) | ✅ 框架 (Table 4-6) | TBD: 需跑实验填数据 |
| 数学公式 (Eq. 1-4) | ✅ | ✅ (15+ 公式) | — |
| 算法伪代码 | ❌ | ✅ (Algorithm 1) | 我们更详细 |
| 参考文献 (30+) | ✅ | ✅ (35篇) | — |

---

## III. 关键缺口分析 (按优先级)

### P0: 必须做 — 决定能否投稿

| # | 缺口 | 当前状态 | 需要什么 | 预估工作量 |
|---|------|----------|----------|-----------|
| 1 | **真机 L1/L2/L3 实验** | 框架就绪，数据全部 TBD | Go2 真机 + 45条指令跑3轮 | 2-3天真机 |
| 2 | **论文图表制作** | 0张图 | 8张图 (系统图、场景图、定性案例等) | 2-3天设计 |
| 3 | **扩充测试环境** | 1个环境 (200m²) | 至少2-3个不同环境 | 1-2天布置 |
| 4 | **外部 Baseline 实现** | 设计了3个但未跑 | CLIP-Frontier, Flat-SG, SG-Nav-Heur | 2天实现 |

### P1: 应该做 — 提升论文竞争力

| # | 缺口 | 当前状态 | 需要什么 | 预估工作量 |
|---|------|----------|----------|-----------|
| 5 | **Fast/Slow 路由比例分析** | 设计了 Table 5 | 真机数据统计 | 1天 |
| 6 | **Per-category 检测分析** | 规划了 Fig.3 | 真机数据 + 绘图 | 1天 |
| 7 | **Jetson 性能 Benchmark** | 脚本就绪 (jetson_benchmark.py) | 在真机上跑 | 半天 |
| 8 | **动态场景测试** | 设计了 Table 6 | 真机实验 (移动物体) | 半天 |

### P2: 可选 — 锦上添花

| # | 缺口 | 需要什么 | 预估工作量 |
|---|------|----------|-----------|
| 9 | Habitat 仿真评测 | 移植到 Habitat API | 5-7天 |
| 10 | 更多真实论文数据对比 | 在相近设定下与 LOVON 跑同一场景 | 取决于可行性 |
| 11 | LaTeX 正式排版 | 转为 IEEE/ICRA 模板 | 1-2天 |
| 12 | View 节点 (关键帧图像) | 代码实现 + 评测 | 2-3天 |

---

## IV. 实验执行计划

### 第一轮: 基础数据收集 (2-3天)

```
Day 1: Env-A (办公走廊)
  - 跑 L1 × 20 instructions × 3 trials = 60 trials
  - 同时收集: 场景图 JSON dump, 视频录制, Nav2 log
  - 运行 jetson_benchmark.py

Day 2: Env-A 续 + Ablation
  - 跑 L2 × 15 × 3 = 45 trials (Full HSG-Nav)
  - 跑 L3 × 10 × 3 = 30 trials (Full HSG-Nav)
  - 动态场景测试: 5 × 3 = 15 trials (移动物体)

Day 3: Ablation + Baseline
  - w/o SceneGraph: L1 × 20 × 3 = 60 trials
  - w/o Hierarchy: L1 × 20 × 3 = 60 trials
  - w/o RePerception: L1 × 20 × 3 = 60 trials
  - CLIP-Frontier baseline: L1 × 20 × 3 = 60 trials
```

### 第二轮: 扩展环境 (1-2天)

```
Day 4: Env-B (不同布局)
  - L1 × 20 × 3 + L2 × 10 × 3 = 90 trials

Day 5: Env-C (更大规模)
  - L1 × 20 × 3 + L2 × 10 × 3 = 90 trials
```

### 第三轮: 图表制作 (2-3天)

```
Day 6: 系统图 + 场景图可视化
  - Fig 1: 系统架构图 (Perception → HSG → Planning → Execution)
  - Fig 2: HSG 四层结构示意图
  - Fig 4: Fast-Slow 路由流程图

Day 7: 数据图表
  - Fig 3: Per-category 检测 recall 条形图
  - Fig 5: Re-perception 可信度时间曲线
  - Fig 6: Frontier 评分因子分解可视化

Day 8: 定性案例
  - Fig 7: 完整导航案例 (场景图演变 + 推理链 + 路径轨迹)
  - Fig 8: Go2 平台照片 (带传感器标注)
```

---

## V. 论文定位与投稿建议

### 建议标题
> **HSG-Nav: Online Hierarchical Scene Graph Reasoning for Zero-Shot Object Navigation on Quadruped Robots**

### 5个核心贡献 (vs 竞品的超越点)

| # | 贡献 | 打谁的弱点 | 论文证据 |
|---|------|-----------|---------|
| 1 | 在线增量 HSG (4层，质量感知CLIP融合) | FSR-VLN 离线预建 (W1) | §3.2, Table 7 |
| 2 | Fast-Slow 层次 CoT 推理 | SG-Nav 每步都调LLM | §3.3, Table 5 |
| 3 | 图式连续 Re-perception | FSR-VLN 静态假设 (W2), SG-Nav 仅到达时 | §3.4, Table 6 |
| 4 | 视觉增强 Frontier 探索 | FSR-VLN 无探索 (W4) | §3.6, 消融 w/o Exploration |
| 5 | 四足真机边缘部署 | FSR-VLN 仅人形 (W5), SG-Nav 仅仿真 | §3.7, Table 8-9 |

### 建议投稿会议

| 会议 | 截止日期 | 匹配度 | 说明 |
|------|---------|--------|------|
| **ICRA 2027** | ~Sep 2026 | ★★★★★ | 机器人导航核心会议 |
| **IROS 2026** | ~Mar 2026 | ★★★★☆ | 时间可能紧张 |
| **CoRL 2026** | ~Jun 2026 | ★★★★☆ | 强调实机部署 |
| **RA-L** | 随时 | ★★★★☆ | 期刊，审稿周期较长 |

### 不应该声称

- ❌ 不要声称 "超越 FSR-VLN 92% SR" — 评测设定完全不同
- ❌ 不要声称 "端到端训练" — 我们是零样本 pipeline
- ❌ 不要声称 "SOTA on MP3D/HM3D" — 我们没有在标准 benchmark 上评测
- ✅ 可以声称 "首个在四足机器人上实现在线层次场景图引导探索的零样本系统"
- ✅ 可以声称 "在更具挑战性的条件下 (未知环境 + 动态 + 多步 + 边缘计算 + 四足) 实现可比性能"

---

## VI. 代码改动入口速查

| 改进项 | 主要文件 | 入口函数/位置 |
|--------|----------|---------------|
| HSG 构建 | `instance_tracker.py` | `compute_regions()`, `compute_rooms()`, `compute_groups()` |
| CLIP 融合 | `instance_tracker.py` | `_fuse_feature()` |
| Room 命名 | `instance_tracker.py`, `perception_node.py` | `infer_room_type()`, `_make_room_llm_namer()` |
| Fast Path | `goal_resolver.py` | `fast_resolve()`, `_multi_source_fusion()` |
| Slow Path | `goal_resolver.py`, `prompt_templates.py` | `resolve()`, `build_goal_resolution_prompt()` |
| 层次 CoT | `prompt_templates.py` | `SYSTEM_PROMPT_ZH`, `SYSTEM_PROMPT_EN` |
| Re-perception | `planner_node.py`, `sgnav_reasoner.py` | `_sgnav_reperception_check()`, `evaluate_target_credibility()` |
| 连续 Re-perception | `planner_node.py` | `_trigger_continuous_reperception()`, `_nav2_feedback_callback()` |
| Frontier 评分 | `frontier_scorer.py` | `score_frontiers()`, `_compute_vision_score()` |
| 子图推理 | `sgnav_reasoner.py` | `score_subgraphs()`, `interpolate_to_frontier()` |
| **BA-HSG 信念状态** | `instance_tracker.py` | `TrackedObject.belief_alpha/beta`, `existence_prob`, `propagate_beliefs()` |
| **Beta 可信度** | `sgnav_reasoner.py` | `evaluate_target_credibility()` — 融合 `belief.P_exist`, `belief.credibility` |
| **多假设目标规划** | `goal_resolver.py` | `TargetBeliefManager`, `verify_and_reselect()` |
| **VoI 调度** | `voi_scheduler.py`, `planner_node.py` | `VoIScheduler.decide()`, `_build_voi_state()`, `_trigger_voi_slow_reason()` |

---

## VII. 总结

**已完成**:
- ✅ 5个创新点的完整代码实现
- ✅ 完整论文初稿 (5章, ~15页, 16+公式, 13表, 1算法)
- ✅ 实验框架 (evaluation runner, benchmark scripts, instruction set, ablation configs)
- ✅ 35篇参考文献整理

**待完成** (按优先级):

### 代码实现 (方法学升级) — ✅ 全部已完成
1. ✅ **BA-HSG 信念模块** — Beta/Gaussian + 图扩散 + 负面证据 (`instance_tracker.py`)
2. ✅ **VoI 调度器** — 替代固定 2m 触发 (`voi_scheduler.py`, `planner_node.py`)
3. ✅ **多假设目标规划** — 贝叶斯后验 + 期望代价 + 到达重选 (`goal_resolver.py`)
4. ✅ **Beta-based 可信度评估** — 融合信念状态的证据模型 (`sgnav_reasoner.py`)

### 实验与评测
4. 🔴 **跑真机实验** (L1/L2/L3 × 3 trials, 填充所有 TBD 数据)
5. 🔴 **制作8张图表** (系统图、场景图、定性案例)
6. 🟡 **扩充测试环境** (至少2-3个不同环境)
7. 🟡 **实现外部 baseline** (Nav2-GT, CLIP-Frontier, Flat-SG, Stubborn, SENT-Map-style)
8. 🟡 **BA-HSG 消融实验** (w/o BeliefState, w/o VoI, w/o MultiHypothesis, w/o GraphDiffusion)
9. 🟡 **超参敏感性分析** (τ_reject, τ_fast, η_neg, λ_t, w_5)
10. 🟡 **统计显著性检验** (McNemar for SR, Wilcoxon for SPL, bootstrap CI)

### 论文定稿
11. 🟢 **LaTeX 排版** → 正式投稿版本
12. 🟢 **考虑 HM3DSEM 仿真评测** → 大规模统计验证

### 时间表建议 (来自 reviewer)
| 阶段 | 目标 | 工作 |
|------|------|------|
| 3个月 | 强系统论文 (ICRA/IROS) | 仿真主结果 + 真机规模化 + 强基线 + 完整指标 |
| 6个月 | 方法学模块落地 | BA-HSG + VoI 验证 + 动态扰动评测 + 投稿 |
| 12个月 | 强方法论文 (RSS/CoRL) | 统一框架 + 跨平台 + 开源基准 |
