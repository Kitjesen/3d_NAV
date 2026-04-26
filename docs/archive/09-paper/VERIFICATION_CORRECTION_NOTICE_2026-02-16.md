# 论文数据校正声明（2026-02-16）

本项目已确认：早期论文草稿中存在“指标先写、证据后补”的问题。为避免虚标，现统一执行以下规则。

---

## 已撤回为“未验证（Claimed）”的数字

以下数字在当前仓库中缺少可复现实验脚本+原始日志+数据表三件套，不能作为论文结论：

- Fast Path 命中率 `90%`
- Fast Path 延迟 `0.17ms`
- ESCA token reduction `92.5%`
- 端到端成功率 `82.3%`
- Jetson 实时性能 `11 FPS`

---

## 当前可确认事实（Measured）

- 代码具备 Fast/Slow、Selective Grounding、Frontier 评分、拓扑记忆、动作原语等实现骨架。
- Frontier 评分已接入 planner 主探索流程，并保留 LLM 回退路径。
- 单测现状：
  - `pytest src/semantic_planner/test -q` → `113 passed, 4 skipped`
  - `pytest src/semantic_perception/test -q` → `48 passed`

---

## 论文写作硬规则（立即生效）

1. 所有性能结论必须标注证据等级：`Claimed / Measured / Published`。
2. `Claimed` 不得写成“我们达到/超过/显著优于”的定论句。
3. 任何对比 SOTA 的百分比提升，必须给出可复现命令、数据集版本、日志路径。

---

## 纠偏目标

短期先做到“描述真实、证据可追溯”；再逐步把 `Claimed` 转成 `Measured`。
