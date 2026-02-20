# NaviMind Paper — 论文撰写进展

> 论文标题: **NaviMind: Online Hierarchical Scene Graph Reasoning for Zero-Shot Object Navigation on Quadruped Robots**

---

## 论文结构与文件

| 章节 | 文件 | 状态 | 说明 |
|------|------|------|------|
| Abstract + Introduction | `01_abstract_intro.md` | ✅ v2 | 5个贡献点升级为方法学贡献 (BA-HSG + VoI + Multi-Hypothesis) |
| Related Work | `02_related_work.md` | ✅ v2 | 5子方向 + DovSG/SENT-Map/不确定性建模，Table 1系统对比表 |
| Method (System) | `03_method.md` | ✅ 完成 | HSG构建 + Fast-Slow + Frontier + 实现细节 |
| **Method (Innovation)** | **`03b_belief_graph.md`** | **✅ 新增** | **BA-HSG: Beta信念 + 图扩散 + 风险规划 + VoI调度 + Algorithm 2-3** |
| Experiments + Results | `04_experiments.md` | ✅ v2 | 15+表格框架，6个baseline，10个ablation，统计检验方案 |
| Conclusion + References | `05_conclusion_refs.md` | ✅ 完成 | 结论 + 35篇参考文献 + 3个Appendix |

## 与SG-Nav (NeurIPS 2024) 论文结构对标

| SG-Nav 有什么 | 我们有什么 | 差距 |
|---------------|-----------|------|
| Abstract (100词) | ✅ Abstract (~200词) | — |
| Figure 1: 动机对比图 | ❌ 缺 | 需设计 |
| Introduction (2页) | ✅ 1.1-1.4 四节 | — |
| Related Work (2页, 2子方向) | ✅ 2.1-2.5 五子方向 + Table 1 | — |
| Method (3页, Eq.1-4, Fig.2-3) | ✅ 3.1-3.7 七节, 15+公式, Algorithm 1 | — |
| Experiments: 3 benchmarks, 6000+ episodes | ❌ 1环境, 45指令 | **核心差距** |
| Table 1: 9个baseline对比 | ✅ 3 baselines + 5 ablations | 需扩充 |
| Table 2-5: 消融实验 | ✅ Table 4-6 (数据TBD) | 需跑实验 |
| Figure 4: Per-category SR | ✅ 规划了 Fig.3 | 需数据 |
| Figure 5: 效率分析 | ✅ Table 8 性能表 | — |
| Figure 6-8: 定性可视化 | ❌ 缺 | 需真机截图 |
| Conclusion | ✅ Section 7 | — |
| 35 references | ✅ 35 references | — |

## 下一步工作优先级

### 必须 (P0) — 论文投稿前必须完成
1. **跑真机实验** → 填充所有 _TBD_ SR/SPL 数据 (Go2 + 办公走廊)
2. **制作 Figure 1-7** → 系统架构图、BA-HSG结构图、Fast-Slow路由图、TSG拓扑图、定性案例
3. **扩充测试环境** → 至少2-3个不同室内环境
4. **LaTeX排版** → IEEE/IROS/CoRL 模板

### 应该 (P1) — 显著提升论文质量
5. **实现3个外部baseline** → CLIP-Frontier, Flat-SG, SG-Nav-Heur
6. **消融实验** → w/o SceneGraph, w/o Hierarchy, w/o Belief, w/o VoI, w/o TSG
7. **Per-category分析图** → 按物体类别的SR条形图
8. **Habitat仿真** → 在MP3D/HM3D上评测 (与SG-Nav直接对比)

### 可选 (P2)
9. 补充更多qualitative examples
10. 扩展到100+指令
11. 动态场景实验 (物体移动)

## 方法学创新路线 (reviewer 建议整合)

| 路线 | 核心思想 | 整合状态 | 文件 |
|------|---------|---------|------|
| **UA-HSG** | Beta存在性信念 + 高斯位置不确定性 + 可信度分解 + 图扩散 | ✅ 已写入 | `03b_belief_graph.md` §3.4.1-3.4.2 |
| **Belief-GoalNav** | 多假设后验 + 贝叶斯验证 + 期望代价选择 | ✅ 已写入 | `03b_belief_graph.md` §3.4.3 |
| **VoI-Scheduler** | 信息价值驱动 continue/reperceive/slow-reason 调度 | ✅ 已写入 | `03b_belief_graph.md` §3.4.4 |
| **Graph Diffusion** | 信念在层次图中上下传播 | ✅ 已写入 | `03b_belief_graph.md` §3.4.2 |

## 版本历史

| 日期 | 变更 |
|------|------|
| 2026-02-13 | 初始实验框架 (experiments/) |
| 2026-02-17 | 完整论文重写，围绕5个创新点重构所有章节 |
| 2026-02-17 | **方法学升级**: 整合 reviewer 建议，新增 BA-HSG 形式化 (03b_belief_graph.md)，更新所有章节 |
| 2026-02-18 | **创新6: TSG 拓扑语义探索** — topology_graph.py (信息增益+Dijkstra+穿越记忆), 33测试全通过 |
| 2026-02-18 | **TSG + LLM 端到端验证** — Kimi-k2.5 实测3场景, TSG与LLM决策一致, 穿越记忆有效 |
| 2026-02-18 | **品牌更名**: HSG-Nav → **NaviMind (灵途)**, 全文档更新 |
| 2026-02-18 | 测试总数: 66 → **99** (新增33个TSG测试), 12个LLM端到端 + 6个TSG+LLM探索测试 |
