# 语义导航论文仓库对比与改进路线（2026-02-16）

**对比对象**: 用户指定的 14 篇论文（arXiv）  
**本地克隆目录**: `D:\robot\code\3dnav\paper_compare`

---

## 1) 仓库可获得性（截至今日）

### 1.1 已找到并克隆

- `OmniNav (2509.25687)`: `paper_compare/omninav-amap`
- `CompassNav (2510.10154)`: `paper_compare/compassnav`
- `ESCA/SGCLIP (2510.15963)`: `paper_compare/esca`
- `MTU3D (2507.04047)`: `paper_compare/mtu3d`
- `MSGNav (2511.10376)`: `paper_compare/msgnav`
- `SG-Nav (2410.08189)`: `paper_compare/sg-nav`
- `L3MVN (2304.05501)`: `paper_compare/l3mvn`
- `VLMnav (2411.05755)`: `paper_compare/vlmnav`
- `ConceptGraphs (2309.16650)`: `paper_compare/concept-graphs`
- `YOLO-World (2401.17270)`: `paper_compare/yolo-world`
- `SayCan (2204.01691)`: `paper_compare/saycan-google-research.git`（Google monorepo bare clone，含 `saycan/`）

### 1.2 未找到稳定可用官方代码仓库

- `VLingNav (2601.08665)`：仅找到网页仓库候选 `paper_compare/vlingnav-web`，无可运行算法代码
- `AdaNav (2509.24387)`：arXiv 页面代码链接候选不可访问
- `OrionNav (2410.06239)`：未检索到明确官方仓库

---

## 2) 与我们项目的实现对齐（结论）

> 结论先行：**不是每篇都“完整实现”**。目前是“核心思想广覆盖 + 若干关键环节未闭环”。

| 论文 | 我们当前状态 | 说明 |
|---|---|---|
| VLingNav | 部分实现 | Fast/Slow 已有，但缺官方代码可逐项对齐，且缺标准基准复现实验 |
| OmniNav | 部分实现 | 有 Fast/Slow 思路，但缺其 benchmark pipeline 与训练/评测框架 |
| AdaNav | 部分实现 | 有置信度阈值路由，但缺“显式不确定性驱动推理深度”机制 |
| CompassNav | 未实质对齐 | 对方代码尚未公开（仓库 TODO），目前无法做方法级复现 |
| ESCA/SGCLIP | 部分实现 | selective grounding 已有，但缺其完整数据/训练管线 |
| MTU3D | 部分实现 | frontier scorer 已接入主流程，但缺基准级 E2E 指标对齐 |
| MSGNav | 未实质对齐 | 仓库 currently coming soon |
| OrionNav | 部分实现 | 有 OVG 场景图+LLM规划形态，但缺论文官方实现对照 |
| SG-Nav | 部分实现 | 有在线 scene graph + LLM 规划主线，但实现细节不完全一致 |
| L3MVN | 部分实现 | 有拓扑记忆/frontier 探索，但探索策略与原文仍未完全同构 |
| VLMnav | 部分实现 | 有 VLM 验证，不是其端到端动作选择范式 |
| ConceptGraphs | 部分实现 | 有实例追踪与关系图，但不是完整 ConceptGraphs pipeline |
| YOLO-World | 部分实现 | 用于检测推理已接入；其完整训练/微调生态未对齐 |
| SayCan | 部分实现 | 有任务分解思路，但缺 skill-affordance 打分闭环 |

---

## 3) 我们项目最该优先补齐的 5 件事

1. **补齐 Frontier 接入后的系统级评估**（最高优先）  
   现状：`frontier_scorer.py` 已接入 `planner_node.py`，但缺 benchmark 级别指标闭环。

2. **补“置信度→推理深度”控制器**（AdaNav 对齐）  
   现状：仅阈值切换 Fast/Slow，缺分层推理预算（step/token/tool budget）。

3. **建立统一 benchmark harness**（OmniNav/SG-Nav/VLMnav 对齐）  
   至少覆盖：成功率、SPL、平均步数、延迟、token、API 成本、P95。

4. **把论文指标改成证据分级**（Claimed/Measured/Published）  
   先去掉“无法复现实验支撑”的硬结论，再逐条补实验脚本和日志路径。

5. **补端到端真实机或仿真闭环实验包**  
   单测通过不等价论文结论，需要任务级/场景级完整闭环结果。

---

## 4) 代码定位（我们仓库）

- Fast/Slow + selective grounding: `src/semantic_planner/semantic_planner/goal_resolver.py`
- 规划状态机与 VLM verify: `src/semantic_planner/semantic_planner/planner_node.py`
- 任务分解: `src/semantic_planner/semantic_planner/task_decomposer.py`
- frontier scorer（已接入主闭环）: `src/semantic_planner/semantic_planner/frontier_scorer.py`
- 拓扑记忆: `src/semantic_planner/semantic_planner/topological_memory.py`
- 场景图感知链路: `src/semantic_perception/semantic_perception/perception_node.py`
