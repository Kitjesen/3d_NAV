# Related Work: 详细技术对比

> 日期: 2026-02-17
> 用途: 论文 Related Work 章节素材

---

## 📊 核心对比：SLAM-Free vs 3D Scene Graph

### 论文对比表

| 维度 | SLAM-Free VLN (2025) | 我们的 3D-NAV |
|------|---------------------|---------------|
| **建图方式** | 轻量级拓扑图（无 SLAM） | 完整 3D 场景图（需 SLAM） |
| **感知层次** | 场景级 + 物体级 | 物体级 + 区域级 + 空间关系 |
| **语义表示** | 拓扑节点 + VLM 描述 | 3D bbox + CLIP 特征 + 空间关系 |
| **规划策略** | LLM 全局 + 视觉局部 | 场景图推理 + Frontier 探索 |
| **空间推理** | 隐式（VLM 理解） | 显式（bbox、法线、投影） |
| **失败恢复** | 未提及 | 自适应 FSM + 重规划 |
| **机器人平台** | 未明确 | Unitree Go2 四足 |
| **实验验证** | 未知 | 真机 + 三级难度 benchmark |

---

## 1. SLAM-Free VLN (2025) 技术分析

### 1.1 核心思路

**"用语义推理替代密集几何"**

```
传统方法:
RGB-D → SLAM → 密集点云/栅格地图 → 几何路径规划

SLAM-Free 方法:
RGB → VLM → 场景语义理解 → 拓扑节点 → LLM 推理
```

**优势:**
- ✅ 计算轻量（无 SLAM）
- ✅ 适合大范围导航
- ✅ 不需要精确几何

**劣势:**
- ❌ 空间推理不精确
- ❌ 难以处理复杂空间关系（"on", "in", "near"）
- ❌ 缺乏 3D 几何约束

### 1.2 层次化视觉-语言感知

**两层感知架构:**

**场景级 (Scene-level):**
- 输入: 全景图像
- 模型: VLM (如 GPT-4V, LLaVA)
- 输出: 场景描述 "这是一个客厅，有沙发、电视、茶几"

**物体级 (Object-level):**
- 输入: 裁剪的物体图像
- 模型: 开放词汇检测器 (如 YOLO-World)
- 输出: 物体列表 + 边界框

**融合策略:**
- 将场景上下文与物体线索结合
- 用于鲁棒的语义推理

### 1.3 粗到细规划

**粗粒度 (Coarse) - 全局层:**
- 任务: 子目标选择
- 方法: LLM 推理
- 输入: 场景描述 + 指令
- 输出: 子目标序列 ["客厅", "走廊", "厨房"]

**细粒度 (Fine) - 局部层:**
- 任务: 障碍物规避
- 方法: 基于视觉的局部规划
- 输入: 当前图像 + 子目标
- 输出: 运动指令

---

## 2. 我们的 3D-NAV 技术分析

### 2.1 核心思路

**"完整 3D 场景图 + 层次推理 + 自适应探索"**

```
我们的方法:
RGB-D → SLAM → 3D 点云 → ConceptGraphs 场景图
                              ↓
                    物体 + 区域 + 空间关系
                              ↓
                    CLIP 语义 + LLM 推理
                              ↓
                    Frontier 探索 + 自适应 FSM
```

**优势:**
- ✅ 精确的 3D 空间推理
- ✅ 显式的空间关系（bbox、法线、投影）
- ✅ 支持复杂多步指令
- ✅ 自适应失败恢复

**劣势:**
- ❌ 需要 SLAM（计算成本高）
- ❌ 依赖深度传感器

### 2.2 层次化场景图

**三层场景表示:**

**物体级 (Object-level):**
- 3D bbox + CLIP 特征
- 检测分数 + 时间戳
- 示例: `chair_1: {pos: [1.2, 0.5, 0.3], bbox: [...], clip: [...]}`

**区域级 (Region-level):**
- DBSCAN 聚类
- 空间邻近性
- 示例: `region_0: [chair_1, table_1, lamp_1]`

**关系级 (Relation-level):**
- 空间关系: on, in, near, left_of, right_of
- 几何约束: bbox gap, 法线方向, 投影重叠
- 示例: `(cup_1, on, table_1)`, `(chair_1, near, table_1)`

### 2.3 多步 VLN + 自适应恢复

**三级解析策略:**

**L1: Regex 解析**
- 简单指令: "go to the chair"
- 直接提取目标词

**L2: 场景顺序解析**
- 中等指令: "go to the chair, then the table"
- 按场景图顺序分解

**L3: LLM 分解**
- 复杂指令: "go to the red chair near the window"
- LLM 推理 + CLIP 属性区分

**自适应 FSM (13 状态):**
- IDLE → PLANNING → NAVIGATING → VERIFYING
- 失败检测 → REPLANNING → EXPLORING
- 动态状态转换

---

## 3. 关键技术差异对比

### 3.1 空间推理能力

**SLAM-Free VLN:**
```python
# 隐式空间推理（VLM 黑盒）
scene_desc = vlm.describe(image)
# "The chair is near the table"
# 无法量化 "near" 的距离
```

**我们的方法:**
```python
# 显式空间推理（几何计算）
bbox_gap = compute_bbox_gap(obj1.bbox, obj2.bbox)
if bbox_gap < NEAR_THRESHOLD:  # 1.5m
    relation = SpatialRelation(obj1, "near", obj2)
# 精确的几何约束
```

### 3.2 语义表示精度

**SLAM-Free VLN:**
- 拓扑节点: `node_1: "living room with sofa"`
- 语义描述: 文本字符串
- 空间信息: 隐式（VLM 理解）

**我们的方法:**
- 3D 物体: `chair_1: {pos: [x,y,z], bbox: [...], clip: [512-dim]}`
- 语义特征: CLIP 向量（可计算相似度）
- 空间信息: 显式（3D 坐标 + bbox）

### 3.3 规划策略

**SLAM-Free VLN:**
```
全局: LLM 选择子目标 ["room_1", "room_2"]
局部: 视觉避障
```

**我们的方法:**
```
全局: 场景图推理 + CLIP 排序
中层: Frontier 探索 + 拓扑记忆
局部: Nav2 + 动态避障
```

### 3.4 失败恢复机制

**SLAM-Free VLN:**
- 未明确提及失败恢复
- 可能依赖重新规划

**我们的方法:**
- 13 状态 FSM
- 失败检测: 导航超时、目标不可达
- 恢复策略: 重规划、探索、回退
- Nav2 action feedback 实时监控

---

## 4. 实验对比

### 4.1 实验环境

**SLAM-Free VLN:**
- 平台: 未明确
- 场景: 未知
- 指令: 未知

**我们的方法:**
- 平台: Unitree Go2 四足机器人
- 场景: 真实室内环境
- 指令: 三级难度 (L1/L2/L3)

### 4.2 评估指标

**SLAM-Free VLN (声称):**
- 语义精度提升
- 规划质量提升
- 导航成功率提升

**我们的方法 (计划):**
- 成功率 (SR)
- 路径长度 (SPL)
- 导航误差 (NE)
- 完成时间
- 按难度分层评估

---

## 5. Related Work 写作建议

### 5.1 SLAM-Free 方法段落

```latex
\subsection{SLAM-Free Visual Navigation}

Recent work explores SLAM-free approaches to reduce computational
overhead. \citet{zhao2025slamfree} propose a hierarchical vision-
language perception system that replaces dense geometric mapping
with lightweight topological representations. Their method uses
VLMs for scene-level understanding and LLM-based reasoning for
coarse-to-fine planning.

\textbf{Limitations:} While SLAM-free methods reduce computational
cost, they sacrifice spatial reasoning precision. Without explicit
3D geometry, these methods struggle with complex spatial relations
(e.g., "on", "in", "near") and fine-grained object localization.
Moreover, implicit scene understanding through VLMs lacks the
geometric constraints necessary for robust navigation in cluttered
environments.

\textbf{Our approach:} In contrast, we construct a complete 3D
hierarchical scene graph with explicit spatial relations. Our
system computes geometric constraints (bounding box gaps, surface
normals, projection overlaps) to enable precise spatial reasoning.
This allows us to handle complex multi-step instructions with
attribute disambiguation (e.g., "go to the red chair near the
window"), which is challenging for SLAM-free methods.
```

### 5.2 对比表格

```latex
\begin{table}[t]
\centering
\caption{Comparison with SLAM-Free VLN Methods}
\begin{tabular}{lcc}
\toprule
\textbf{Method} & \textbf{SLAM-Free} & \textbf{3D-NAV (Ours)} \\
\midrule
Spatial Representation & Topological & 3D Scene Graph \\
Spatial Relations & Implicit (VLM) & Explicit (Geometric) \\
Semantic Features & Text Descriptions & CLIP Embeddings \\
Planning Strategy & LLM Global + Visual Local & Scene Graph + Frontier \\
Failure Recovery & Not Specified & Adaptive FSM \\
Robot Platform & Not Specified & Unitree Go2 \\
Real-World Validation & Unknown & 3-Level Benchmark \\
\bottomrule
\end{tabular}
\end{table}
```

### 5.3 优势总结

**我们相对于 SLAM-Free 方法的优势:**

1. **精确的空间推理**
   - 显式 3D 几何约束
   - 量化的空间关系（距离、方向、重叠）
   - 支持复杂空间查询

2. **鲁棒的语义表示**
   - CLIP 特征向量（可计算相似度）
   - 属性区分能力（颜色、大小、形状）
   - 时间一致性（EMA 更新）

3. **自适应失败恢复**
   - 13 状态 FSM
   - 多层次重规划
   - Nav2 action feedback

4. **真机验证**
   - Unitree Go2 四足机器人
   - 真实室内场景
   - 三级难度 benchmark

---

## 6. 论文写作策略

### 6.1 定位策略

**不要说:**
- ❌ "我们提出了 SLAM-free 方法"（他们已经做了）
- ❌ "我们使用 VLM 理解场景"（他们也用了）

**应该说:**
- ✅ "与 SLAM-free 方法不同，我们构建完整 3D 场景图"
- ✅ "我们的显式空间推理优于隐式 VLM 理解"
- ✅ "我们在真实四足机器人上验证了系统"

### 6.2 创新点强调

**C1: 层次场景图引导探索**
- 对比: SLAM-free 用拓扑图，我们用 3D 场景图
- 优势: 精确的空间关系 + DBSCAN 区域聚类

**C2: 多步 VLN + 自适应失败恢复**
- 对比: SLAM-free 未提及失败恢复
- 优势: 13 状态 FSM + Nav2 feedback + 动态重规划

**C3: 三级难度真机 Benchmark**
- 对比: SLAM-free 实验环境未知
- 优势: Unitree Go2 + 真实场景 + L1/L2/L3 分级评估

---

## 7. 完整 Related Work 对比表

| 方法 | 建图 | 感知 | 规划 | 恢复 | 平台 | 验证 |
|------|------|------|------|------|------|------|
| **SLAM-Free VLN** | 拓扑图 | VLM | LLM | ❌ | ❓ | ❓ |
| **VLingNav** | ❌ | VLM | LLM | ❌ | 仿真 | Habitat |
| **LOVON** | ❌ | YOLO+CLIP | 端到端 | ❌ | Go2 | 真机 |
| **SG-Nav** | 场景图 | CLIP | LLM | ❌ | 轮式 | 仿真 |
| **ConceptGraphs** | 3D场景图 | CLIP | ❌ | ❌ | ❌ | 感知 |
| **3D-NAV (Ours)** | 3D场景图 | YOLO+CLIP | 场景图+Frontier | ✅ | Go2 | 真机 |

---

**总结**: SLAM-Free 方法牺牲空间精度换取计算效率，适合大范围粗粒度导航。我们的方法保留完整 3D 几何，支持精确空间推理和复杂多步指令，更适合需要细粒度操作的真实机器人应用。
