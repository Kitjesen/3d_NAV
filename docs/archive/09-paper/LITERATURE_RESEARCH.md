# 文献调研与系统对标分析

> 调研日期: 2026-02-17  
> 目的: 深入调研 VLN/ObjectNav/场景图/四足导航 领域前沿论文，逐篇与我们系统做详细对比，找出差距并给出可落地的提升方案  
> 覆盖范围: 2022–2026，共 35+ 篇核心论文  
> 重点: FSR-VLN 深度分析与超越策略

---

## 0. 我们系统的核心 Pipeline

```
YOLO-World 开放词汇检测 → CLIP 编码 → Depth + TF2 → 3D 投影
→ 实例追踪 (EMA) → 场景图构建 (Object + Group + Region)
→ GoalResolver (Fast: 关键词+CLIP匹配 / Slow: LLM推理)
→ TaskDecomposer (规则模板 or LLM 分解) → Nav2 执行
→ FrontierScorer (启发式) + SGNavReasoner (子图打分)
```

关键特征: 零样本（无训练）、模块化 pipeline、边缘部署(Jetson Orin NX)、四足机器人

---

## I. 层次场景图与导航推理

### 1. SG-Nav (NeurIPS 2024)

**论文**: "SG-Nav: Online 3D Scene Graph Prompting for LLM-based Zero-shot Object Navigation"  
**来源**: NeurIPS 2024 | [GitHub](https://github.com/bagh2178/SG-Nav) | [主页](https://bagh2178.github.io/SG-Nav/)

**核心方法**:
- 在线构建层次 3D 场景图 (object → group → room)
- 将场景图分为子图，每个子图用**层次 Chain-of-Thought prompt** 给 LLM 推理
- LLM 输出每个子图的相关概率 → **插值到 frontier** → 选最高分 frontier 探索
- **Re-perception 机制**: 持续置信度判断，纠正假阳性检测
- 零样本方法首次超越监督方法 (MP3D SR > 10%)

**性能**: MP3D / HM3D / RoboTHOR 上零样本 SOTA，超越所有之前零样本方法 10%+ SR

**与我们的对比**:

| 维度 | SG-Nav | 我们 | 差距 |
|------|--------|------|------|
| 场景图构建 | 在线增量 3D 场景图 + 占用栅格 | 检测 + 规则空间关系 + DBSCAN 聚类 | 概念类似，质量差距大 |
| 层次推理 | 层次 CoT prompt, LLM 对子图概率推理 | 子图启发式打分 + 距离插值 | **核心差距**: 推理深度不够 |
| Frontier 选择 | LLM 概率 → 插值到 frontier | 手写权重启发式 | LLM 参与度低 |
| Re-perception | 持续置信度 → 拒绝假阳性 | 框架存在但未激活 | **关键缺失** |
| 评测 | Habitat MP3D/HM3D/RoboTHOR | 无标准评测 | 无法比较 |

**可借鉴点**:
1. **层次 CoT prompt**: 改造 `build_sgnav_subgraph_prompt` 为真正的层次推理（Room→Group→Object 逐层缩小）
2. **Re-perception**: 激活已有的 `_target_credibility` 机制，在 APPROACH 后做近距离重检测
3. **子图概率插值**: 当前的距离衰减插值基本对齐，但子图评分应更多依赖 LLM 而非启发式

---

### 2. FSR-VLN (2025, Horizon Robotics)

**论文**: "FSR-VLN: Fast and Slow Reasoning for Vision-Language Navigation with Hierarchical Multi-modal Scene Graph"  
**来源**: arXiv 2509.13733 | [主页](https://horizonrobotics.github.io/robot_lab/fsr-vln/)

**核心方法**:
- **四层 HMSG**: Floor → Room (2D polygon + CLIP embedding + LLM 命名) → View (关键帧图像+位姿) → Object (3D bbox + 语义)
- **Fast-to-Slow 推理**: Fast 阶段用 CLIP 匹配逐层筛选 (Room→View→Object); Slow 阶段 VLM 看 View 图像精细判断
- 真机部署: Unitree G1 + Intel RealSense D455 + Mid360 LiDAR + FAST-LIVO2 SLAM

**性能**: SR = 92% (80/87), 响应时间比 VLM-only 降低 82%, 大幅超越 OK-Robot (60.9%) 和 HOVSG (51.7%)

**与我们的对比**:

| 维度 | FSR-VLN | 我们 | 差距 |
|------|---------|------|------|
| 层次层数 | 4 层 (Floor/Room/View/Object) | 3 层 (Region/Group/Object) | 缺 View 层, Room 无真实边界 |
| Room 定义 | 2D polygon 真实房间 + CLIP embedding + LLM 命名 | DBSCAN 3m 聚类 + 拼接标签名 | **Room 质量差距大** |
| View 层 | 关键帧图像 + 位姿，VLM 可直接看图 | **不存在** | **关键缺失** |
| Fast 路径 | CLIP 逐层筛选 (Room→View→Object) | 关键词匹配 + CLIP 单层匹配 | **没有逐层递进** |
| Slow 路径 | VLM 看 View 图像精细判断 | 把 JSON 发给 LLM 问坐标 | VLM 无法看图 |
| 硬件 | Unitree G1 + RealSense + Mid360 + FAST-LIVO2 | 相似 (Unitree + Orbbec + Livox + FAST-LIO2) | 硬件平台接近 |

**可借鉴点 (最高优先级)**:
1. **View 节点**: 保存每个 Region 的关键帧图像 + 位姿，Slow Path 时 VLM 直接看图 → 这是最大的提升点
2. **Room CLIP embedding**: 给每个 Region 计算 CLIP 语义向量 (区域内物体特征均值)
3. **Room LLM 命名**: 用 LLM 给区域命名 ("走廊"/"办公室"/"楼梯间")
4. **逐层 CLIP 筛选**: Fast Path 从 Room→Object 逐层缩小范围

**为何 FSR-VLN 是最值得对齐的**: 硬件平台几乎相同 (Unitree + LiDAR + RGBD)，方法论也是模块化 pipeline（不是端到端训练），且已发表，92% SR 是一个可信的对标目标。

---

### 3. OSG Navigator (2025)

**论文**: "Open Scene Graphs for Open-World Object-Goal Navigation"  
**来源**: [主页](https://open-scene-graphs.github.io/) | [OpenReview](https://openreview.net/forum?id=qGkMxAx588)

**核心方法**:
- **OSG Schema**: 描述环境类别 (home/supermarket/office) 的模板结构，可从语义标签自动生成
- 组合 LLM + VFM + GNM (通用导航模型)，零样本适配新环境
- 在 Fetch 和 Spot 机器人上真机部署

**与我们的对比**:

| 维度 | OSG Navigator | 我们 | 差距 |
|------|---------------|------|------|
| 场景先验 | OSG Schema 描述环境结构 | 无环境先验 | 我们没有利用环境类型 |
| 模型组合 | LLM + VFM + GNM | LLM + YOLO-World + CLIP | 类似思路 |
| 真机部署 | Fetch + Spot | 四足 (Go2) | 均有真机 |

**可借鉴点**: OSG Schema 的思想 — 给系统一个环境类型先验 (如 "这是办公楼")，可以约束 LLM 推理的空间

---

### 4. vS-Graphs (2025)

**论文**: "vS-Graphs: Tightly Coupling Visual SLAM and 3D Scene Graphs Exploiting Hierarchical Scene Understanding"  
**来源**: arXiv 2503.01783

**核心方法**:
- 将 Visual SLAM 与 3D 场景图紧耦合
- 从建筑组件 (墙壁/地面) 推断房间/楼层结构
- 场景图层次: 建筑 → 楼层 → 房间 → 物体
- 定位精度提升 15.22%

**与我们的对比**:

| 维度 | vS-Graphs | 我们 | 差距 |
|------|-----------|------|------|
| SLAM 耦合 | 场景图嵌入 SLAM 后端优化 | 场景图独立于 SLAM | 我们的场景图不参与 SLAM 优化 |
| 房间检测 | 从墙壁/地面结构推断 | DBSCAN 物体聚类 | **我们没有结构感知** |

**可借鉴点**: 利用 3D 点云/占用栅格的结构信息 (墙壁检测) 来辅助 Region 划分，而不是纯靠物体聚类

---

### 5. Hydra (RSS 2022 + IJRR)

**论文**: "Hydra: A Real-time Spatial Perception System for 3D Scene Graph Construction and Optimization"  
**来源**: [GitHub](https://github.com/MIT-SPARK/Hydra) | MIT SPARK Lab

**核心方法**:
- 实时增量构建 5 层场景图: 度量网格 → 物体 → 地点 → 房间 → 建筑
- 层次回环检测 + 嵌入变形图优化
- 从 3D 度量-语义网格分割出房间

**与我们的对比**: Hydra 的 5 层结构是场景图领域的标杆。我们的 3 层 (Region/Group/Object) 缺少"地点"层和真实的房间分割。

**可借鉴点**: "地点 (places)" 的概念 — 拓扑导航的路径点，比我们的 TopologicalMemory 更结构化

---

## II. 四足机器人 VLN (直接竞品)

### 6. LOVON (arXiv 2025.07)

**论文**: "LOVON: Legged Open-Vocabulary Object Navigator"  
**来源**: arXiv 2507.06747 | [GitHub](https://github.com/DaojiePENG/LOVON)

**核心方法**:
- LLM 层次任务规划 + 开放词汇视觉检测
- Laplacian 方差滤波解决视觉抖动
- 盲区处理 + 临时目标丢失恢复
- 功能执行逻辑保证自主导航 + 任务适应
- 多平台: Unitree Go2, B2, H1-2

**性能**: 真机长序列任务完成 (检测/搜索/导航)

**与我们的对比**:

| 维度 | LOVON | 我们 | 差距/优势 |
|------|-------|------|-----------|
| 检测 | 开放词汇 | YOLO-World 开放词汇 | 类似 |
| Laplacian 滤波 | 有 | 有 | 我们已实现 |
| 场景图 | **无** | 有 (3层层次) | **我们的优势** |
| 空间推理 | 无显式建模 | 空间关系图 | **我们的优势** |
| CLIP 语义 | 无 | 有 (text_text + text_image) | **我们的优势** |
| 盲区处理 | 有专门方案 | 无 | 我们缺失 |
| 目标丢失恢复 | 有 | 有 (backtrack 子目标) | 类似 |
| 真机实验 | 有完整数据 | **无** | **关键差距** |
| 多平台 | Go2/B2/H1-2 | 仅 Go2 | 他们覆盖更广 |

**关键结论**: LOVON 是**扁平 pipeline** (detect → plan → go)，没有场景图和层次推理。我们的场景图 + CLIP + 空间关系是相对 LOVON 的**核心差异化**，但必须用实验证明它带来了真实收益。

---

### 7. OrionNav (arXiv 2024.10)

**论文**: "OrionNav: Online Planning for Robot Autonomy with Context-Aware LLM and Open-Vocabulary Semantic Scene Graphs"  
**来源**: arXiv 2410.06239

**核心方法**:
- FC-CLIP 开放词汇语义分割 + RGB-D
- DBSCAN 聚类构建语义物体地图 → 生成场景图
- LLM 基于场景图实时规划，动态更新 plan
- **实时 onboard 运行**

**与我们的对比**:

| 维度 | OrionNav | 我们 | 差距/优势 |
|------|----------|------|-----------|
| 分割 vs 检测 | FC-CLIP 语义分割 | YOLO-World 检测框 | 分割更精细 |
| 场景图 | 层次场景图 + 空间关系 | 类似 | 概念对齐 |
| LLM 规划 | 实时更新 plan | 一次性分解 | **我们缺闭环规划** |
| Onboard | 实时 onboard | 依赖云端 LLM API | 离线能力弱 |

**可借鉴点**: 实时 LLM 规划更新 — 场景图变化时动态调整 plan

---

### 8. NaVILA (RSS 2025, NVIDIA)

**论文**: "NaVILA: Legged Robot Vision-Language-Action Model for Navigation"  
**来源**: RSS 2025 | [主页](https://navila-bot.github.io/)

**核心方法**:
- 两层架构: VLA 模型生成中层空间语言命令 (如 "forward 75cm") → RL 运动策略执行
- 训练数据: 真实人类视频 + 仿真导航 + QA
- VLN-CE-Isaac 物理仿真评测
- 平台: Unitree Go2, G1, Booster T1

**与我们的对比**:

| 维度 | NaVILA | 我们 | 差距 |
|------|--------|------|------|
| 架构 | 端到端 VLA | 模块化 pipeline | **根本不同** |
| 训练 | 大规模 (视频+仿真+QA) | 零样本, 无训练 | 不可比 |
| 动作空间 | 中层语言命令 → RL 策略 | PoseStamped → Nav2 | 更灵活 |
| 可解释性 | 黑盒 | 可解释 (场景图+推理) | **我们的优势** |

**关键结论**: NaVILA 是端到端训练路线，和我们的模块化零样本路线是**不同赛道**。不应直接对标，但可在论文中作为对比说明两种路线的 trade-off (训练成本 vs 可解释性)。

---

### 9. Helpful DoggyBot (2024)

**论文**: "Helpful DoggyBot: Open-World Object Fetching using Legged Robots and Vision-Language Models"  
**来源**: arXiv 2410.00231 | [主页](https://helpful-doggybot.github.io/)

**核心方法**:
- 四足 + 前置夹爪 → 物体抓取
- RL 训练低层控制 (攀爬/倾斜/避障)
- VLM 双摄像头 (第三人称鱼眼 + 自身 RGB) 语义理解
- 零样本泛化到新环境

**性能**: 60% SR 在未见环境, 无真实数据训练

**与我们的对比**:

| 维度 | Helpful DoggyBot | 我们 | 差距 |
|------|------------------|------|------|
| 任务 | 抓取 + 导航 | 纯导航 | 不同任务 |
| 低层控制 | RL 训练 (攀爬等) | 已有运动栈 | 我们用已有栈 |
| 语义 | VLM 双摄 | 场景图 + CLIP | 不同方法 |
| SR | 60% 零样本 | 未测 | 需真机评测 |

---

### 10. VLN-PE (ICCV 2025)

**论文**: "Rethinking the Embodied Gap in Vision-and-Language Navigation: A Holistic Study of Physical and Visual Disparities"  
**来源**: ICCV 2025

**核心方法**:
- 系统性评测 VLN 方法在不同机器人 (人形/四足/轮式) 上的性能差距
- 揭示仿真→真实的性能下降原因: 有限观测空间、光照变化、碰撞、跌倒
- 支持无缝集成 MP3D 之外的新场景

**与我们的启示**:
- **四足的特殊挑战**: 视角抖动、高度限制、可通行性判断
- **观测空间受限**: 固定前向摄像头 vs 仿真中的全景
- 这些都是我们真机实验必须面对的

---

## III. 探索策略与 Frontier

### 11. VLFM (ICRA 2024 Best Paper)

**论文**: "VLFM: Vision-Language Frontier Maps for Zero-Shot Semantic Navigation"  
**来源**: ICRA 2024 Best Paper (Cognitive Robotics) | [GitHub](https://github.com/bdaiinstitute/vlfm) | Boston Dynamics AI

**核心方法**:
- 从深度图构建占用栅格 → 识别 frontier
- 用预训练 VLM 生成 **language-grounded value map** → 给 frontier 语义评分
- 在 Boston Dynamics Spot 上真机部署

**性能**: Gibson/HM3D/MP3D SOTA SPL

**与我们的对比**:

| 维度 | VLFM | 我们 | 差距 |
|------|------|------|------|
| Frontier 评分 | VLM 语义 value map | 手写权重 (distance/novelty/language/grounding) | **核心差距** |
| Value map | VLM 为每个 frontier 方向生成语义热图 | 无 | 我们没有视觉语义评分 |
| 占用栅格 | 核心依赖 | 订阅了 costmap 但没用到 | **浪费了已有资源** |
| 真机 | Spot | 四足 (Go2) | 均有真机能力 |

**可借鉴点**:
1. **用 CLIP/VLM 给 frontier 视觉评分**: 对每个 frontier 方向拍照 → CLIP(指令, 图像) → 评分
2. **利用已订阅的 costmap**: 当前 perception_node 订阅了 `/nav/costmap` 但未传给 FrontierScorer
3. **Value map 思想**: 不一定要生成热图，但可以用 CLIP 给 frontier 方向的观测图像打分

---

### 12. L3MVN (IROS 2023)

**论文**: "L3MVN: Leveraging Large Language Models for Visual Target Navigation"  
**来源**: IROS 2023

**核心方法**:
- LLM 常识评估 frontier ("厨房附近可能有冰箱")
- 从语义地图提取 frontier 信息 → LLM 评分 → 选最高分 frontier

**与我们的对比**: 我们的 `TopologicalMemory` 只记录位置和物体标签，不参与 frontier 评分。L3MVN 的核心是让 LLM 参与 frontier 选择。

**可借鉴点**: 将拓扑记忆中的物体标签传给 LLM，让它推理 "这个方向之前看到了什么，目标更可能在哪里"

---

## IV. 多步规划与闭环执行

### 13. Inner Monologue (Google, CoRL 2023)

**论文**: "Inner Monologue: Embodied Reasoning through Planning with Language Models"  
**来源**: CoRL 2023 | [主页](https://innermonologue.github.io/)

**核心方法**:
- LLM 不是一次性规划，而是在执行循环中持续接收反馈
- 反馈类型: 成功检测、场景描述、物体识别
- 失败时 LLM 重新推理, 无需额外训练

**与我们的对比**:

| 维度 | Inner Monologue | 我们 | 差距 |
|------|-----------------|------|------|
| 规划模式 | 闭环: 执行→观察→反思→调整 | 开环: 一次分解→顺序执行 | **核心差距** |
| 反馈 | 多模态 (成功/场景/物体) | 无 | 我们完全没有执行反馈 |
| 失败处理 | LLM 分析原因 + 重规划 | retry_count >= max → FAILED | 我们不分析原因 |

**可借鉴点 (P1 优先级)**:
```
执行子目标 → 获取当前场景图快照 → 构建反馈文本:
  "已执行: NAVIGATE to door, 结果: 到达 (3.2, 1.5) 附近"
  "当前场景: 检测到 [chair, table, door], 距目标约 2.1m"
→ LLM 判断: 继续 / 修改目标 / 回退探索
```

---

### 14. T-A-L: Think, Act, Learn (2025)

**论文**: "Think, Act, Learn: A Framework for Autonomous Robotic Agents using Closed-Loop Large Language Models"  
**来源**: arXiv 2507.19854

**核心方法**:
- **Think**: LLM 分解高层指令为 plan
- **Act**: 执行 + 收集多模态反馈
- **Learn**: 处理反馈 → 因果分析 → 修正策略 → 存入经验记忆
- 经验记忆供后续规划参考

**性能**: 97%+ SR 复杂长序列任务, ~9 次试验收敛

**与我们的对比**: T-A-L 的"学习"模块（经验记忆 + 因果分析）远超我们的"重试 N 次后放弃"。

**可借鉴点**: 在 TopologicalMemory 中增加失败经验 — 记录 "在此位置执行X失败，原因Y"，下次规划时避免。

---

### 15. LERa: Look, Explain, Replan (2025)

**论文**: "LERa: Replanning with Visual Feedback in Instruction Following"  
**来源**: arXiv 2507.05135

**核心方法**:
- **Look**: RGB 图像 → VLM 描述场景
- **Explain**: VLM 识别执行异常 ("计划说要到门，但前方被椅子挡住")
- **Replan**: VLM 生成修改方案 ("绕过椅子再去门")

**性能**: 动态环境提升 40%, 操作任务提升 67%

**与我们的对比**: LERa 只需 RGB + 指令 + 初始 plan + 失败检测，不需要额外检测器或预定义条件。非常适合我们集成。

**可借鉴点 (非常实用)**: 在子目标执行超时或 Nav2 返回失败时:
1. 拍当前帧
2. 发给 VLM: "这是当前画面，我要去X，出了什么问题？"
3. VLM 回答 + 建议新 plan

---

### 16. Reflective Planning (2025)

**论文**: "Reflective Planning: Vision-Language Models for Multi-Stage Long-Horizon Robotic Manipulation"  
**来源**: arXiv 2502.16707 | [主页](https://reflect-vlm.github.io/)

**核心方法**:
- 用生成模型想象未来世界状态
- 基于想象引导动作选择
- 反思潜在次优性 → 修正推理

**与我们的启示**: "想象未来状态" 的思路可用于探索决策 — 给 LLM 描述 frontier 方向之前看到的物体，让它"想象"那边可能有什么

---

### 17. SayCan (Google, 2022)

**论文**: "Do As I Can, Not As I Say: Grounding Language in Robotic Affordances"  
**来源**: CoRL 2022 | [主页](https://say-can.github.io/)

**核心方法**:
- LLM 提出动作候选 → 价值函数评估可行性 → 选最可行的
- 后续 PaLM-SayCan: 84% 正确序列, 74% 执行 SR

**与我们的对比**: 我们的 TaskDecomposer 借鉴了 SayCan 的分解思想，但缺少 **affordance grounding** — 不评估子目标是否可执行。

**可借鉴点**: 在分解后，用场景图检查每个子目标的可行性 (目标是否在场景中? 路径是否可通行?) → 过滤不可行的子目标

---

## V. 场景图构建

### 18. ConceptGraphs (ICRA 2024)

**论文**: "ConceptGraphs: Open-Vocabulary 3D Scene Graphs for Perception and Planning"  
**来源**: ICRA 2024 | [主页](https://concept-graphs.github.io/) | [GitHub](https://github.com/concept-graphs/concept-graphs)

**核心方法**:
- SAM 实例分割 → CLIP 编码 → 3D 点云投影
- **多视角关联融合**: 同一物体多次观测在 3D 空间聚合
- **LLM 生成关系**: 不是硬编码规则
- 支持复杂语义查询 ("Michael Jordan 会玩的东西" → 篮球)

**与我们的对比**:

| 维度 | ConceptGraphs | 我们 | 差距 |
|------|---------------|------|------|
| 分割方法 | SAM 实例分割 | YOLO-World 检测框 | 检测框信息量少 |
| CLIP 融合 | 多视角加权融合 | 单帧 | **差距大** |
| 空间关系 | LLM 推理生成 | 硬编码距离/方位规则 | **完全不同** |
| 实例关联 | 视觉特征 + 空间 | 标签名 + 距离 | 简化很多 |

**可借鉴点**:
1. **多视角 CLIP 融合**: 同一 TrackedObject 的 CLIP 特征应取加权平均 (当前只保留最新帧)
2. **LLM 关系生成**: 定期把检测到的物体列表发给 LLM 生成关系描述 (而非硬编码 dx/dy 规则)

---

### 19. DYNEMO-SLAM (2025)

**论文**: "DYNEMO-SLAM: Dynamic Entity and Motion-Aware 3D Scene Graph SLAM"  
**来源**: arXiv 2503.02050

**核心方法**:
- 处理动态环境中的场景图 SLAM
- 建模运动实体 (人、移动物体) 的位姿
- 联合优化: 机器人轨迹 + 动态实体位姿 + 环境结构
- 轨迹误差减少 49.97%

**与我们的启示**: 当前实例追踪假设物体静止 (EMA 平滑)。真机场景中人会走动、门会开关 → 需要动态物体感知

---

## VI. 记忆与长程导航

### 20. Mem4Nav (ICLR 2026 submitted)

**论文**: "Mem4Nav: Boosting Vision-and-Language Navigation in Urban Environments with a Hierarchical Spatial-Cognition Long-Short Memory System"  
**来源**: [OpenReview](https://openreview.net/forum?id=El3Sitc09j) | [GitHub](https://github.com/TSYJ-He/Mem4Nav_VLN)

**核心方法**:
- **稀疏八叉树**: 细粒度体素索引 (局部空间记忆)
- **语义拓扑图**: 高层 landmark 连通性 (全局结构记忆)
- **双记忆**: 长期记忆 (LTM, 可逆 Transformer 压缩) + 短期记忆 (STM, 实时避障)
- 即插即用增强现有 VLN backbone

**性能**: Task Completion 提升 7-13.3pp, nDTW 提升 10-12pp

**与我们的对比**: 我们的 `TopologicalMemory` 是简单的位置+标签记录。Mem4Nav 的双层记忆 (体素+拓扑) 远更丰富。

**可借鉴点**: 在 TopologicalMemory 中区分短期 (最近 N 步的详细观测) 和长期 (历史访问过的区域摘要)

---

## VII. 补充论文 (零样本 ObjectNav / 场景表示 / 记忆 / 控制)

### 21. HOV-SG (RSS 2024)

**论文**: "Hierarchical Open-Vocabulary 3D Scene Graphs for Language-Grounded Robot Navigation"  
**来源**: RSS 2024 | [主页](https://hovsg.github.io/) | [GitHub](https://github.com/hovsg/HOV-SG)

**核心方法**:
- 从开放词汇视觉基础模型生成 segment-level 3D 地图
- 自顶向下构建三层场景图: Floor → Room → Object，每层加开放词汇特征
- 比稠密开放词汇地图体积**减少 75%**，语义精度更高
- 支持多层楼建筑 + 跨楼层 Voronoi 导航图

**性能**: 在真机多层楼环境中完成长程语言导航; FSR-VLN 对比中 SR = 51.7%

**与我们的对比**:

| 维度 | HOV-SG | 我们 | 差距 |
|------|--------|------|------|
| 分割方法 | 开放词汇分割模型 | YOLO-World 检测框 | 分割更精细 |
| 层次结构 | Floor → Room → Object | Region → Group → Object | 概念类似 |
| Room 特征 | 开放词汇 CLIP | DBSCAN 聚类 + 拼接标签 | 我们语义弱 |
| 多楼层 | 支持 | 不支持 | 我们是单层 |

---

### 22. OK-Robot (2024)

**论文**: "OK-Robot: What Really Matters in Integrating Open-Knowledge Models for Robotics"  
**来源**: arXiv 2401.12202 | [GitHub](https://github.com/ok-robot/ok-robot)

**核心方法**:
- 零样本模块化框架: VLM 检测 + 导航 + 抓取
- 10 个纽约公寓, 171 个 pick-and-drop 任务

**性能**: SR = 58.5% (杂乱环境), 82% (整洁环境); FSR-VLN 对比中 SR = 60.9%  
**失败分析**: 语义检索错误 9.3%, 操作位姿困难 8.0%, 硬件问题 7.5%

**与我们的启示**: OK-Robot 的失败分析方法值得借鉴 — 我们也应该做系统化失败归因

---

### 23. CoW: CLIP on Wheels (CVPR 2023)

**论文**: "CoWs on Pasture: Baselines and Benchmarks for Language-Driven Zero-Shot Object Navigation"  
**来源**: CVPR 2023 | [主页](https://cow.cs.columbia.edu/)

**核心方法**:
- 将零样本 ObjectNav 拆为: CLIP 物体定位 + 经典探索
- 无需训练，匹配在 Habitat 上训练 500M 步的方法

**性能**: RoboTHOR 上比学习 baseline SR +15.6-16.1pp  
**PASTURE 基准**: 不常见物体、属性描述、隐藏物体

**与我们的对比**: CoW 是零样本 ObjectNav 的经典 baseline, 我们的 Fast Path 本质上类似 CoW (CLIP 匹配 + 探索), 但我们多了场景图和空间关系

---

### 24. ESC (ICML 2023)

**论文**: "ESC: Exploration with Soft Commonsense Constraints for Zero-shot Object Navigation"  
**来源**: ICML 2023

**核心方法**:
- LLM 常识 → 软逻辑约束 → 指导探索
- 房间级 + 物体级双层常识推理
- 比 CoW 在 MP3D 上 SR 提升 288% (相对)

**与我们的对比**: ESC 是 SG-Nav 的前身。ESC 用 LLM 常识指导 frontier 探索, 我们的 FrontierScorer 也有类似思路但实现更简单 (手写启发式 vs 逻辑谓词)

---

### 25. VoroNav (ICML 2024)

**论文**: "VoroNav: Voronoi-based Zero-shot Object Navigation with Large Language Model"  
**来源**: ICML 2024 | [主页](https://voro-nav.github.io/)

**核心方法**:
- 从实时语义地图构建 **Reduced Voronoi Graph**
- 生成路径和远景文本描述 → LLM 选择最优 waypoint
- HM3D SR +2.8%, SPL +3.7%; HSSD SR +2.6%, SPL +3.8%

**与我们的对比**: VoroNav 用 Voronoi 图替代 frontier 方法选导航点。我们用 Nav2 + frontier, 但 VoroNav 的"远景描述"思路有启发 — 给 LLM 描述每个方向能看到什么

---

### 26. GaussNav (TPAMI 2024)

**论文**: "GaussNav: Gaussian Splatting for Visual Navigation"  
**来源**: TPAMI | [GitHub](https://github.com/XiaohanLei/GaussNav)

**核心方法**:
- 用 3D Gaussian Splatting 构建场景表示
- 比 BEV 地图保留更多纹理细节
- HM3D SPL 从 0.252 提升到 0.578

**与我们的启示**: 3DGS 是新的场景表示趋势, 但对边缘设备 (Jetson) 来说计算量过大。我们暂不考虑, 但值得关注。

---

### 27. ReMEmbR (ICRA 2025, NVIDIA)

**论文**: "ReMEmbR: Building and Reasoning Over Long-Horizon Spatio-Temporal Memory for Robot Navigation"  
**来源**: ICRA 2025 | [GitHub](https://github.com/NVIDIA-AI-IOT/remembr)

**核心方法**:
- 视频片段 → VILA 模型字幕 → 向量数据库 (MilvusDB) + 时间戳 + 坐标
- RAG 检索 + LLM 推理回答关于过去事件的问题
- Nova Carter 机器人 25 分钟导航 → 回答导航相关问题

**与我们的启示**: 当前 TopologicalMemory 只记录位置和标签, 不记录时间线和事件。ReMEmbR 的 RAG 方法可增强我们的长程记忆。

---

### 28. RoboEXP (CoRL 2024)

**论文**: "RoboEXP: Action-Conditioned Scene Graph via Interactive Exploration for Robotic Manipulation"  
**来源**: CoRL 2024 | [GitHub](https://github.com/Jianghanxiao/RoboEXP)

**核心方法**:
- LMM 推理探索什么/怎么探索 → 增量构建**动作条件场景图** (ACSG)
- ACSG 同时捕获几何/语义 (低层) 和动作关系 (高层)
- 零样本泛化到新环境

**与我们的对比**: 我们的场景图只有空间关系, 没有"动作关系" (如 "门可以打开", "抽屉可以拉出")。RoboEXP 的 ACSG 思路可扩展我们的场景图。

---

### 29. VLM-PC (RSS 2024, Stanford)

**论文**: "Commonsense Reasoning for Legged Robot Adaptation with Vision-Language Models"  
**来源**: RSS 2024 | [主页](https://anniesch.github.io/vlm-pc/)

**核心方法**:
- VLM + Model Predictive Control → 四足机器人避障
- 关键: 基于历史交互的 **in-context adaptation** + 多步技能规划
- Go1 四足, 无地图/无人引导, 处理死胡同/窄缝/攀爬等

**与我们的启示**: VLM-PC 展示了 VLM 在四足复杂地形中的实用性。我们的 Slow Path 可以借鉴其"基于历史的 in-context 推理" — 把之前走过的路和遇到的障碍作为上下文传给 LLM。

---

### 30. ThinkBot (2024)

**论文**: "ThinkBot: Embodied Instruction Following with Thought Chain Reasoning"  
**来源**: [主页](https://guanxinglu.github.io/thinkbot/)

**核心方法**:
- **Instruction Completer**: LLM 补全指令中缺失的动作描述
- **Object Localizer**: 语义地图推断物体位置
- ALFRED 基准上超越 SOTA

**与我们的对比**: ThinkBot 的"指令补全"思路有启发 — 用户说 "去门那里" 缺少很多隐含步骤, LLM 可以自动补全 "走到走廊 → 找到门 → 面向门 → 确认"

---

### 31. MFRA (2025)

**论文**: "Think Hierarchically, Act Dynamically: Hierarchical Multi-modal Fusion and Reasoning for Vision-and-Language Navigation"  
**来源**: arXiv 2504.16516

**核心方法**:
- 多层级特征融合: 低层视觉 → 高层语义, 跨模态聚合
- 指令引导注意力 + 动态上下文集成
- REVERIE / R2R / SOON 基准 SOTA

**与我们的对比**: MFRA 是端到端训练方法 (不同赛道), 但其"多层级融合"概念可以启发我们的 Fast Path — 在匹配时同时考虑低层 (标签匹配) 和高层 (语义相似度) 特征

---

## VIII. 综合对标表

### 按方法类别

| 类别 | 论文 | 我们的对齐程度 | 值得对齐程度 |
|------|------|---------------|-------------|
| **场景图 + 导航** | SG-Nav | 30-40% | 最高 |
| | FSR-VLN | 25-35% | **最高 (主要对标)** |
| | HOV-SG | 20-30% | 高 |
| | OSG Navigator | 15-20% | 中 |
| **零样本 ObjectNav** | ESC | 15-20% | 中 |
| | CoW | 20-25% | 中 (baseline) |
| | VoroNav | 10-15% | 中 |
| | GaussNav | 0% | 低 (趋势关注) |
| **四足 VLN** | LOVON | 竞品定位 | **必须超越** |
| | OrionNav | 30-40% | 高 |
| | NaVILA | 不同路线 | 作对比 |
| | Helpful DoggyBot | 不同任务 | 作参考 |
| | VLM-PC | 5% | 中 (四足避障) |
| **Frontier 探索** | VLFM | 10-15% | 高 |
| | L3MVN | 15-20% | 中 |
| **闭环规划** | Inner Monologue | 5-10% | **高** |
| | T-A-L | 0% | 高 |
| | LERa | 0% | **高** (最易集成) |
| | ThinkBot | 5% | 中 |
| | Reflective Planning | 0% | 中 |
| **场景图构建** | ConceptGraphs | 20-30% | 中 |
| | Hydra | 10% | 参考 |
| | RoboEXP | 5% | 低 |
| | DYNEMO-SLAM | 0% | 低 |
| **记忆系统** | Mem4Nav | 5% | 中 |
| | ReMEmbR | 0% | 中 |
| **其他** | OK-Robot | 15% | 参考 (失败分析) |
| | VLN-PE | 0% | 参考 (sim2real) |
| | MFRA | 0% | 低 (端到端) |
| | SayCan | 40-50% | 中 |

### 按论文发表的 SR 性能

| 系统 | 评测环境 | SR | SPL | 备注 |
|------|----------|----|-----|------|
| FSR-VLN | 真机 (G1) | **92%** | — | 模块化, **预建图**, 87条指令 |
| SG-Nav | Habitat MP3D | ~55% | ~25% | 零样本, 超越监督方法 |
| SG-Nav | Habitat HM3D | ~45% | ~22% | 零样本 |
| VLFM | HM3D/MP3D | SOTA | SOTA SPL | Frontier + VLM |
| ESC | MP3D | ~35% | — | CoW 的 288% 提升 |
| CoW | RoboTHOR | ~20% | — | 零样本 baseline |
| VoroNav | HM3D | +2.8% | +3.7% | Voronoi graph |
| GaussNav | HM3D | — | 0.578 | 3DGS |
| OK-Robot | 真机 | 58.5% | — | 导航+抓取 |
| HOV-SG | 真机 | 51.7% (FSR-VLN 评) | — | 多楼层 |
| Helpful DoggyBot | 真机 | 60% | — | 四足+抓取 |
| T-A-L | 真机 | **97%** | — | 闭环+经验学习 |
| NaVILA | VLN-CE-Isaac | SOTA | — | 端到端训练 |
| LOVON | 真机 | 有 (未公开) | — | 四足 pipeline |
| **我们** | **无评测** | **未知** | — | — |

---

## IX. FSR-VLN 深度分析与超越策略

### FSR-VLN 的精确工作流程

```
[离线阶段 - HMSG 构建]
  扫描环境 (G1 + RealSense + Mid360 + FAST-LIVO2)
  → 实例级开放词汇建图 → 自动提取 Floor/Room/Object 节点
  → 每个 Room: LLM (GPT-4o) 看图命名 + CLIP embedding
  → 每个 Object: 3D bbox + CLIP embedding
  → 每个 View: 关键帧图像 + 位姿 + CLIP embedding + VLM 字幕
  → 构建拓扑边: Room↔View↔Object, View↔View (相对位姿)

[在线阶段 - FSR 推理]
  用户指令 → LLM 解析 (推断目标 or 分解为 floor/room/object)
  → Fast Matching:
     CLIP(指令, Room_embedding) → 候选 Room
     CLIP(指令, View_embedding) → 候选 View
     CLIP(指令, Object_embedding) → 候选 Object
  → Slow Reasoning (仅在 Fast 不可靠时):
     Step 1: VLM 验证 Object 是否真的在其 best_view 图中
     Step 2: 若验证失败 → LLM 读 View 字幕重选 → VLM 对比两个 View
     Step 3: 确定最终 View → 重新 CLIP 匹配该 View 下的 Object
  → 输出: 目标 Object 的 3D 位置 → 导航
```

### FSR-VLN 的 5 大弱点 (论文明确承认或可推导)

| # | 弱点 | 论文原文/依据 | 影响 |
|---|------|--------------|------|
| **W1** | **HMSG 离线预建, 不支持在线增量** | "limitations include the time-consuming HMSG construction process" | 必须先扫描完整环境才能导航, 不能探索未知区域 |
| **W2** | **假设静态环境** | "the assumption of static environments, which are targeted for future work" | 门开关、人走动、物体移动都会导致失败 |
| **W3** | **仅单步指令 (go to X)** | 87 条指令均为单步目标定位, 无多步任务 | 不支持 "先去厨房再找红杯子" |
| **W4** | **无探索能力** | 依赖预建图, 无 frontier 探索 | 目标不在 HMSG 中则直接失败 |
| **W5** | **仅人形测试 (G1)** | 未在四足上验证 | 四足视角更低、抖动更大、通行性不同 |

### 我们超越 FSR-VLN 的 5 条路线

#### 路线 A: **在线增量** vs FSR-VLN 的离线预建 (打 W1)

FSR-VLN 需要提前扫描建图，我们的场景图是**实时增量构建**的:

```
FSR-VLN: 扫描 30min → 构建 HMSG → 然后才能导航
我们:     开机即导航 → 边走边建场景图 → 场景图持续增长
```

**这是最大的结构性优势**。论文中可以论述:

> "FSR-VLN 依赖离线预建的 HMSG，在未知环境中无法直接使用。我们的系统实时增量构建层次场景图, 支持即时部署和持续探索。"

**需要补强**: 当前增量场景图质量不如 FSR-VLN 的离线图。所以要:
- 加 Room CLIP embedding (均值聚合)
- 加 Room LLM 命名
- 加 View 节点 (关键帧保存)
- 逐层 CLIP 筛选

#### 路线 B: **探索能力** vs FSR-VLN 无探索 (打 W4)

FSR-VLN 如果目标不在 HMSG 中, 无法处理。我们有:

```
目标不在场景图 → FrontierScorer 评分 → SGNavReasoner 子图推理
→ 选择最有可能的 frontier → 前往探索 → 场景图增长 → 重新匹配
```

**需要补强**: Frontier 评分当前是手写启发式。应该:
- 用 CLIP 给 frontier 方向的观测图像评分 (VLFM 思路)
- 把拓扑记忆中已探索区域的物体信息传给 LLM 推理 (L3MVN/ESC 思路)

#### 路线 C: **多步指令 + 闭环反馈** vs FSR-VLN 单步 (打 W3)

FSR-VLN 只处理 "go to X" 类型, 不支持:
- "先去厨房, 然后找到红色杯子" (多步)
- "找到门旁边的灭火器" (空间条件)
- "如果走廊尽头没有, 就去二楼" (条件分支)

我们已有 TaskDecomposer + 子目标序列, 补强闭环后:

```
"先去门那里, 然后找附近的灭火器"
→ SubGoal 1: NAVIGATE to door  → 执行 → 成功
→ [场景图更新: 门附近检测到灭火器]
→ SubGoal 2: FIND fire extinguisher near door → 场景图匹配 → 成功
→ SubGoal 3: APPROACH + VERIFY → re-perception 确认
```

**需要补强**: 闭环执行反馈 (Inner Monologue / LERa 思路)

#### 路线 D: **四足真机** vs FSR-VLN 仅人形 (打 W5)

FSR-VLN 仅在 Unitree G1 人形上测试。四足机器人有独特挑战:
- 更低的视角 (~30cm vs ~150cm) → 视野更受限
- 行走时的视觉抖动 → Laplacian 滤波更重要
- 不同的通行性 (可以钻矮处, 不能上楼梯) → 需要地形感知

我们天然在四足平台上 (Go2), 这是论文差异化:

> "我们是首个在四足机器人上实现层次场景图引导探索的系统"

#### 路线 E: **Re-perception + 动态场景** vs FSR-VLN 静态假设 (打 W2)

FSR-VLN 的 Slow Reasoning 有 VLM 验证 Object, 但这是在预建 HMSG 中的一次性匹配。  
我们可以做**持续 re-perception**:

```
导航途中: 每 N 步更新场景图 → 检测目标是否仍在 → 如果消失则重新搜索
到达目标: 近距离重新检测 → 确认/拒绝 → 更新 credibility
环境变化: 新物体出现/旧物体消失 → 场景图自动更新
```

### 超越 FSR-VLN 的综合论述

```
FSR-VLN:  预建图 → 单步指令 → Fast/Slow CLIP匹配 → 静态环境 → 人形机器人
我们:      在线建图 → 多步指令 → 层次推理+探索+闭环 → 动态场景 → 四足机器人
          ↑ 路线A      ↑ 路线C    ↑ 路线B          ↑ 路线E    ↑ 路线D
```

**FSR-VLN 92% SR 的上下文**: 这是在预建图 + 单步指令 + 静态环境条件下的结果。在我们的设定 (在线建图 + 多步 + 动态 + 四足) 下, FSR-VLN 将**无法运行**。

**论文实验设计建议**:

| 实验 | 说明 | 验证的贡献 |
|------|------|-----------|
| 对照 1 | 纯 CLIP 匹配 (类似 CoW), 无场景图 | 场景图的价值 |
| 对照 2 | 扁平场景图 (无层次, 类似 LOVON) | 层次推理的价值 |
| 对照 3 | 无 Re-perception | Re-perception 的价值 |
| 对照 4 | 无闭环反馈 (当前开环) | 闭环的价值 |
| 对照 5 | 无探索 (仅匹配已知场景图) | 探索能力的价值 |
| 基准 | L1/L2/L3 三级难度, 每级 20+ 指令 | 整体 SR/SPL |

---

## X. 提升路线图 (按优先级, 更新版)

### P0: 必须做 — 超越 FSR-VLN 的核心差异化

| 改进 | 对标论文 | 具体改动 | 预估工作量 |
|------|----------|----------|-----------|
| **Room CLIP embedding** | FSR-VLN 路线A | Region 节点加 CLIP 向量 (内部物体特征均值) | 2h |
| **Room LLM 命名** | FSR-VLN 路线A | 定期把 Region 内物体列表发给 LLM 命名 ("走廊"/"办公室") | 2h |
| **逐层 CLIP 筛选** | FSR-VLN 路线A | Fast Path: Room CLIP → Object CLIP 两级筛选 | 4h |
| **Re-perception 激活** | SG-Nav / 路线E | 导航到目标后近距离重检测，更新 credibility | 3h |
| **闭环执行反馈** | Inner Monologue / 路线C | 子目标执行后场景+结果反馈 LLM, 支持重规划 | 1天 |
| **真机 L1/L2/L3 评测** | 全部 | 设计指令集 + 跑实验 + 记录 SR/SPL + ablation | 2-3天 |

### P1: 应该做 — 提升场景图质量与探索效率

| 改进 | 对标论文 | 具体改动 | 预估工作量 |
|------|----------|----------|-----------|
| **闭环执行反馈** | Inner Monologue / LERa | 子目标执行后把场景变化反馈给 LLM | 1天 |
| **View 节点** | FSR-VLN | 保存关键帧图像+位姿, Slow Path 时 VLM 看图 | 1天 |
| **Frontier 视觉评分** | VLFM | 对 frontier 方向观测图像做 CLIP 评分 | 4h |
| **多视角 CLIP 融合** | ConceptGraphs | TrackedObject 的 CLIP 特征取加权平均 | 2h |
| **利用 costmap** | VLFM | 把已订阅的 costmap 传给 FrontierScorer | 1h |

### P2: 可以做 — 锦上添花

| 改进 | 对标论文 | 具体改动 | 预估工作量 |
|------|----------|----------|-----------|
| 失败经验记忆 | T-A-L | TopologicalMemory 记录失败 + 原因 | 4h |
| LLM 空间关系 | ConceptGraphs | 用 LLM 替代硬编码距离规则生成关系 | 4h |
| 环境类型先验 | OSG Navigator | 支持指定 "办公楼"/"住宅" 类型 | 2h |
| 双记忆系统 | Mem4Nav | 短期 (最近观测) + 长期 (历史摘要) | 1天 |
| 动态物体感知 | DYNEMO-SLAM | 标记移动物体, 降低其位置权重 | 4h |

---

## XI. 论文写作定位建议

### 建议标题:

> **"NaviMind: Online Hierarchical Scene Graph Reasoning for Zero-Shot Multi-Step VLN on Quadruped Robots"**

### 核心贡献 (vs FSR-VLN 的 5 个超越点):

1. **在线增量场景图** (vs FSR-VLN 离线预建): 无需预扫描，开机即导航，支持未知环境探索 [路线A+B]
2. **多步指令 + 闭环执行**: 任务分解 → 子目标序列 → 执行反馈 → 重规划 (vs FSR-VLN 仅单步) [路线C]
3. **Re-perception + 动态场景适应**: 持续置信度追踪 + 假阳性抑制 (vs FSR-VLN 静态假设) [路线E]
4. **四足机器人验证**: 低视角、高抖动、地形约束下的系统级验证 (vs FSR-VLN 仅人形) [路线D]
5. **边缘部署**: Jetson Orin NX 16GB 完整运行 (vs FSR-VLN 未提及算力约束)

### 与各系统的定位关系:

| 系统 | 我们的关系 | 论文中的角色 |
|------|-----------|-------------|
| **FSR-VLN** | 主要对标, 打其 5 个弱点 | 最强 baseline, 指出其离线/单步/静态局限 |
| **SG-Nav** | 方法论灵感 | Related work: 层次推理 + re-perception 的来源 |
| **LOVON** | 直接竞品 (四足) | 实验对比: 层次场景图 vs 扁平 pipeline |
| **OrionNav** | 同领域 (四足+场景图) | Related work: 类似思路但无层次推理 |
| **VLFM** | Frontier 方法参考 | Related work: frontier 语义评分的启发 |
| **ConceptGraphs** | 场景图参考 | Related work: 增量场景图构建 |
| **Inner Monologue** | 闭环规划参考 | Related work: 闭环执行反馈 |
| **CoW / ESC** | 零样本 baseline | 实验 ablation 中的对照 |

### 不应该声称:

- 不要声称"端到端"或"训练出的 xxx" — 我们是零样本 pipeline
- 不要声称"对齐 VLingNav/OmniNav/AdaNav" — 那些是端到端训练模型
- 不要声称"超越 FSR-VLN 的 SR" — 评测设定不同, 应强调能力维度的超越
- 合理引用: SG-Nav, ConceptGraphs, VLFM, SayCan, Inner Monologue

---

## XII. 参考文献列表 (35 篇)

### 场景图 + 导航
1. SG-Nav — Yin et al., NeurIPS 2024
2. FSR-VLN — Horizon Robotics, arXiv 2509.13733, 2025
3. HOV-SG — RSS 2024
4. OSG Navigator — 2025
5. vS-Graphs — arXiv 2503.01783, 2025
6. Hydra — Hughes et al., RSS 2022 / IJRR

### 四足 / 移动机器人 VLN
7. LOVON — Peng et al., arXiv 2507.06747, 2025
8. OrionNav — arXiv 2410.06239, 2024
9. NaVILA — RSS 2025, NVIDIA
10. Helpful DoggyBot — arXiv 2410.00231, 2024
11. VLN-PE — ICCV 2025
12. VLM-PC — RSS 2024, Stanford

### 零样本 ObjectNav
13. VLFM — Yokoyama et al., ICRA 2024 Best Paper
14. ESC — Zhou et al., ICML 2023
15. CoW — Gadre et al., CVPR 2023
16. VoroNav — Wu et al., ICML 2024
17. GaussNav — Lei et al., TPAMI 2024
18. OK-Robot — arXiv 2401.12202, 2024

### Frontier 探索
19. L3MVN — Yu et al., IROS 2023

### 闭环规划 / 任务分解
20. Inner Monologue — Huang et al., CoRL 2023
21. T-A-L (Think, Act, Learn) — arXiv 2507.19854, 2025
22. LERa — arXiv 2507.05135, 2025
23. Reflective Planning — arXiv 2502.16707, 2025
24. SayCan — Ahn et al., CoRL 2022
25. ThinkBot — arXiv 2312.07062, 2024

### 场景图构建
26. ConceptGraphs — Gu et al., ICRA 2024
27. DYNEMO-SLAM — arXiv 2503.02050, 2025
28. RoboEXP — CoRL 2024

### 记忆系统
29. Mem4Nav — ICLR 2026 submitted
30. ReMEmbR — ICRA 2025, NVIDIA

### VLN 架构 / 训练方法
31. MFRA — arXiv 2504.16516, 2025
32. DivScene — EMNLP 2025 Findings
33. ActiveVLN — arXiv 2509.12618, 2025

### 数据集 / 评测
34. PASTURE (CoW) — CVPR 2023
35. VLN-CE-Isaac (NaVILA) — RSS 2025
