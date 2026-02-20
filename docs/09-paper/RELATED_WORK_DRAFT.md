# Related Work 章节草稿

> 用于论文 Related Work 部分
> 日期: 2026-02-17

---

## 完整 Related Work 章节

### 2. Related Work

We review recent advances in vision-language navigation, focusing on semantic scene understanding, spatial reasoning, and robotic deployment.

#### 2.1 Vision-Language Navigation Systems

**Simulation-based VLN.** Early VLN work primarily focuses on simulation environments. VLingNav~\cite{vlingnav2026} proposes a Fast-Slow dual-process architecture for efficient instruction following, achieving 70\% fast-path hit rate in Habitat simulation. However, these methods lack real-world validation and struggle with the sim-to-real gap.

**Real-robot VLN.** LOVON~\cite{lovon2024} demonstrates VLN on the Unitree Go2 quadruped robot using end-to-end learning with 6 motion primitives. While pioneering in real-robot deployment, LOVON is limited to simple object navigation ("run to an object") without complex instruction understanding or spatial reasoning.

**Our contribution:** We present the first complete VLN system on a quadruped robot that handles complex multi-step instructions with hierarchical scene graph reasoning and adaptive failure recovery.

#### 2.2 Semantic Scene Understanding

**3D Scene Graphs.** ConceptGraphs~\cite{conceptgraphs2024} builds incremental 3D scene graphs with open-vocabulary detection and CLIP features. SG-Nav~\cite{sgnav2024} uses hierarchical scene graphs (room→object→relation) with LLM reasoning for navigation. However, these works either focus solely on perception (ConceptGraphs) or lack real-robot validation (SG-Nav).

**SLAM-Free Approaches.** Recent work explores SLAM-free navigation to reduce computational overhead. Zhao et al.~\cite{zhao2025slamfree} propose hierarchical vision-language perception with lightweight topological representations, replacing dense geometric mapping with VLM-based scene understanding and LLM-based coarse-to-fine planning.

**Limitations of SLAM-Free methods:** While computationally efficient, SLAM-free approaches sacrifice spatial reasoning precision. Without explicit 3D geometry, they struggle with:
- Complex spatial relations (e.g., "on", "in", "near")
- Fine-grained object localization
- Attribute disambiguation (e.g., "red chair" vs "blue chair")
- Geometric constraints in cluttered environments

**Our approach:** We construct a complete 3D hierarchical scene graph with explicit spatial relations. Our system computes geometric constraints (bounding box gaps, surface normals, projection overlaps) to enable precise spatial reasoning. This allows us to handle complex multi-step instructions with attribute disambiguation, which is challenging for SLAM-free methods. Table~\ref{tab:comparison} compares our approach with recent VLN systems.

\begin{table}[t]
\centering
\caption{Comparison with Recent VLN Systems}
\label{tab:comparison}
\small
\begin{tabular}{lccccc}
\toprule
\textbf{Method} & \textbf{3D Scene} & \textbf{Spatial} & \textbf{Multi-step} & \textbf{Failure} & \textbf{Real} \\
 & \textbf{Graph} & \textbf{Relations} & \textbf{VLN} & \textbf{Recovery} & \textbf{Robot} \\
\midrule
VLingNav~\cite{vlingnav2026} & \xmark & \xmark & \cmark & \xmark & \xmark \\
LOVON~\cite{lovon2024} & \xmark & \xmark & \xmark & \xmark & \cmark \\
SG-Nav~\cite{sgnav2024} & \cmark & \cmark & \cmark & \xmark & \xmark \\
ConceptGraphs~\cite{conceptgraphs2024} & \cmark & \cmark & \xmark & \xmark & \xmark \\
SLAM-Free VLN~\cite{zhao2025slamfree} & \xmark & Implicit & \cmark & \xmark & Unknown \\
\midrule
\textbf{3D-NAV (Ours)} & \cmark & \textbf{Explicit} & \cmark & \cmark & \cmark \\
\bottomrule
\end{tabular}
\end{table}

#### 2.3 Spatial Reasoning for Navigation

**Implicit spatial reasoning.** VLM-based methods~\cite{zhao2025slamfree,vlmnav2024} rely on vision-language models to implicitly understand spatial relations through natural language descriptions. While flexible, these methods lack quantitative geometric constraints and struggle with precise localization.

**Explicit spatial reasoning.** Our work follows ConceptGraphs~\cite{conceptgraphs2024} in building explicit 3D representations, but extends it with:
1. **Hierarchical clustering:** DBSCAN-based region discovery for room-level understanding
2. **Geometric spatial relations:** Quantitative computation of "on", "in", "near" using bounding box gaps, surface normals, and projection overlaps
3. **CLIP-based attribute disambiguation:** Distinguishing objects by color, size, and shape for L3-level instructions

#### 2.4 Exploration and Failure Recovery

**Frontier-based exploration.** Traditional frontier exploration~\cite{yamauchi1997} identifies unexplored regions for autonomous mapping. Recent work integrates semantic information: OmniNav~\cite{omninav2026} uses uncertainty estimation for adaptive exploration.

**Failure recovery.** Most VLN systems assume successful navigation without explicit failure handling. In contrast, we implement a 13-state adaptive FSM with:
- Nav2 action feedback for real-time monitoring
- Multi-level replanning (local → global → exploration)
- Topological memory for backtracking

**Our contribution:** We combine frontier exploration with scene graph reasoning, using grounding potential scores to prioritize frontiers likely to contain target objects. Our adaptive FSM enables robust failure recovery in real-world scenarios.

#### 2.5 Quadruped Robot Navigation

**Learning-based control.** LOVON~\cite{lovon2024} uses end-to-end learning (IOE + L2MM) for quadruped VLN with 6 motion primitives. While effective for simple tasks, this approach lacks interpretability and generalization to complex instructions.

**Modular systems.** Our work adopts a modular architecture separating perception, planning, and control. We leverage Nav2~\cite{nav2} for low-level navigation and focus on high-level semantic reasoning. This design enables:
- Easier debugging and maintenance
- Component-wise optimization
- Integration with existing robotics stacks

---

## 关键差异总结

### 与 SLAM-Free VLN 的核心差异

**技术层面:**
1. **空间表示**: 拓扑图 vs 3D 场景图
2. **空间推理**: 隐式 (VLM) vs 显式 (几何计算)
3. **语义特征**: 文本描述 vs CLIP 向量
4. **失败恢复**: 无 vs 13 状态 FSM

**实验层面:**
1. **机器人平台**: 未知 vs Unitree Go2
2. **验证环境**: 未知 vs 真实室内场景
3. **任务复杂度**: 未知 vs 三级难度 (L1/L2/L3)

### 与其他工作的差异

**vs VLingNav:**
- 他们: 仿真 + Fast-Slow 架构
- 我们: 真机 + 完整系统 + 失败恢复

**vs LOVON:**
- 他们: 真机 + 简单导航
- 我们: 真机 + 复杂多步指令 + 场景图推理

**vs SG-Nav:**
- 他们: 场景图 + LLM (仿真/轮式)
- 我们: 场景图 + LLM + 四足 + 真机

**vs ConceptGraphs:**
- 他们: 3D 场景图 (仅感知)
- 我们: 3D 场景图 + 完整导航系统

---

## 写作要点

### 强调我们的独特性

1. **唯一性组合:**
   - 真实四足机器人 + 完整 3D 场景图 + 多步 VLN + 失败恢复

2. **技术深度:**
   - 显式空间关系计算（bbox gap, 法线, 投影）
   - CLIP 属性区分（L3 级别指令）
   - 13 状态自适应 FSM

3. **实验验证:**
   - Unitree Go2 真机
   - 三级难度 benchmark
   - 真实室内场景

### 避免的陷阱

1. ❌ 不要声称发明了 SLAM-free（他们已经做了）
2. ❌ 不要声称首次使用 VLM（很多人用了）
3. ❌ 不要过度强调工程实现（强调系统集成的价值）

### 应该强调的

1. ✅ 首个在四足机器人上的完整 VLN 系统
2. ✅ 显式 3D 空间推理优于隐式 VLM 理解
3. ✅ 自适应失败恢复机制
4. ✅ 三级难度真机 benchmark

---

**下一步**: 根据这个对比分析，完善论文的 Related Work 章节，突出我们的独特贡献。
