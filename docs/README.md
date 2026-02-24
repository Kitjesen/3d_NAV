# MapPilot (灵途) 文档中心

**项目**: MapPilot — 四足机器人户外自主导航系统
**平台**: Jetson Orin NX 16GB | ROS2 Humble | Ubuntu 22.04
**版本**: 1.5.0
**最后更新**: 2026-02-24

---

## 文档导航

### [01-getting-started](./01-getting-started/) — 快速开始
新用户从这里开始，了解如何构建和部署系统。

- **[QUICK_START.md](./01-getting-started/QUICK_START.md)** — 5分钟快速启动
- **[BUILD_GUIDE.md](./01-getting-started/BUILD_GUIDE.md)** — 构建指南
- **[DEPLOY.md](./01-getting-started/DEPLOY.md)** — 部署指南
- **[PROJECT_OVERVIEW_ZH.md](./01-getting-started/PROJECT_OVERVIEW_ZH.md)** — 项目总览（中文）

---

### [02-architecture](./02-architecture/) — 架构设计
系统架构、算法原理和通信协议。

- **[ARCHITECTURE.md](./02-architecture/ARCHITECTURE.md)** — 系统架构文档
- **[ALGORITHM_REFERENCE.md](./02-architecture/ALGORITHM_REFERENCE.md)** — 核心算法参考
- **[TOPIC_CONTRACT.md](./02-architecture/TOPIC_CONTRACT.md)** — ROS2话题契约
- **[geometry_enhanced_topology_design.md](./02-architecture/geometry_enhanced_topology_design.md)** — 几何增强拓扑设计
- **[polyhedron_expansion_algorithm.md](./02-architecture/polyhedron_expansion_algorithm.md)** — 多面体膨胀算法

---

### [03-development](./03-development/) — 开发指南
API参考、参数调优、故障排查和重构记录。

- **[API_REFERENCE.md](./03-development/API_REFERENCE.md)** — API参考文档
- **[PARAMETER_TUNING.md](./03-development/PARAMETER_TUNING.md)** — 参数调优指南
- **[TROUBLESHOOTING.md](./03-development/TROUBLESHOOTING.md)** — 故障排查手册
- **[CODE_REFACTOR.md](./03-development/CODE_REFACTOR.md)** — 代码重构记录
- **[API_REFACTORING_SUMMARY.md](./03-development/API_REFACTORING_SUMMARY.md)** — API重构总结
- **[hybrid_planner_usage.md](./03-development/hybrid_planner_usage.md)** — 混合规划器使用指南
- **[optimization_summary.md](./03-development/optimization_summary.md)** — 优化工作总结
- **[visualization_tools_summary.md](./03-development/visualization_tools_summary.md)** — 可视化工具总结

---

### [04-deployment](./04-deployment/) — 部署运维
Docker容器化、OTA升级和生产环境部署。

- **[DOCKER_GUIDE.md](./04-deployment/DOCKER_GUIDE.md)** — Docker部署指南
- **[OTA_GUIDE.md](./04-deployment/OTA_GUIDE.md)** — OTA升级指南
- **[OTA_V3_REPORT.md](./04-deployment/OTA_V3_REPORT.md)** — OTA V3技术报告

---

### [05-specialized](./05-specialized/) — 专项指南
WebRTC、网关、通信优化等专项技术文档。

- **[WEBRTC_GUIDE.md](./05-specialized/WEBRTC_GUIDE.md)** — WebRTC视频流指南
- **[gateway.md](./05-specialized/gateway.md)** — 网关设计文档
- **[PROTO_REGEN_PLAYBOOK.md](./05-specialized/PROTO_REGEN_PLAYBOOK.md)** — Protobuf重新生成手册
- **[COMM_OPTIMIZATION.md](./05-specialized/COMM_OPTIMIZATION.md)** — 通信优化方案
- **[DECOUPLING_ANALYSIS.md](./05-specialized/DECOUPLING_ANALYSIS.md)** — 解耦分析文档

---

### [06-semantic-nav](./06-semantic-nav/) — 语义导航（核心）
语义导航子系统的技术文档、升级计划和实现状态。

**概览文档**
- **[SEMANTIC_NAV_REPORT.md](./06-semantic-nav/SEMANTIC_NAV_REPORT.md)** — 语义导航技术报告
- **[SEMANTIC_NAV_STATUS.md](./06-semantic-nav/SEMANTIC_NAV_STATUS.md)** — 实现状态检查
- **[EXECUTIVE_SUMMARY.md](./06-semantic-nav/EXECUTIVE_SUMMARY.md)** — 执行摘要
- **[COMPREHENSIVE_STATUS.md](./06-semantic-nav/COMPREHENSIVE_STATUS.md)** — 综合状态报告

**实现文档**
- **[FAST_SLOW_IMPLEMENTATION.md](./06-semantic-nav/FAST_SLOW_IMPLEMENTATION.md)** — Fast-Slow双进程实现
- **[LOVON_IMPLEMENTATION.md](./06-semantic-nav/LOVON_IMPLEMENTATION.md)** — LOVON动作原语实现
- **[TASK_DECOMPOSITION_AND_GOAL_RESOLUTION.md](./06-semantic-nav/TASK_DECOMPOSITION_AND_GOAL_RESOLUTION.md)** — 任务分解与目标解析
- **[SCENE_GRAPH_AND_FSRVLN.md](./06-semantic-nav/SCENE_GRAPH_AND_FSRVLN.md)** — 场景图与FSR-VLN
- **[SEMANTIC_CONTROL_SYSTEM_V1.md](./06-semantic-nav/SEMANTIC_CONTROL_SYSTEM_V1.md)** — 语义控制系统 V1
- **[USS_NAV_README.md](./06-semantic-nav/USS_NAV_README.md)** — USS-Nav探索模式说明

**状态与计划**
- **[PROJECT_STATUS.md](./06-semantic-nav/PROJECT_STATUS.md)** — 项目状态跟踪
- **[UPGRADE_PLAN.md](./06-semantic-nav/UPGRADE_PLAN.md)** — 论文级升级计划
- **[README_UPGRADE.md](./06-semantic-nav/README_UPGRADE.md)** — 升级项目说明
- **[SEMANTIC_PERCEPTION_STATUS.md](./06-semantic-nav/SEMANTIC_PERCEPTION_STATUS.md)** — 感知模块状态
- **[SEMANTIC_PLANNER_STATUS.md](./06-semantic-nav/SEMANTIC_PLANNER_STATUS.md)** — 规划模块状态
- **[IMPLEMENTATION_VERIFICATION.md](./06-semantic-nav/IMPLEMENTATION_VERIFICATION.md)** — 实现验证报告
- **[FINAL_WORK_SUMMARY.md](./06-semantic-nav/FINAL_WORK_SUMMARY.md)** — 最终工作总结
- **[SEMANTIC_NAV_GUIDE.md](./06-semantic-nav/SEMANTIC_NAV_GUIDE.md)** — 语义导航使用指南
- **[PROJECT_QA.md](./06-semantic-nav/PROJECT_QA.md)** — 项目问答

**核心技术**:
- Fast-Slow双进程 (VLingNav 2026) | ESCA选择性Grounding (NeurIPS 2025)
- MTU3D Frontier评分 (ICCV 2025) | ConceptGraphs场景图 (ICRA 2024)
- LOVON动作原语 (2024) | AdaNav熵触发 | ReMEmbR情景记忆

---

### [07-testing](./07-testing/) — 测试
测试报告、性能基准和回归检查清单。

**测试报告**
- **[FINAL_TEST_REPORT.md](./07-testing/FINAL_TEST_REPORT.md)** — 最终测试报告
- **[TEST_REPORT_20260208.md](./07-testing/TEST_REPORT_20260208.md)** — 测试报告 (2026-02-08)
- **[FAST_SLOW_PERFORMANCE_REPORT.md](./07-testing/FAST_SLOW_PERFORMANCE_REPORT.md)** — Fast-Slow性能报告
- **[SLOW_PATH_LLM_TEST_REPORT.md](./07-testing/SLOW_PATH_LLM_TEST_REPORT.md)** — Slow Path LLM测试报告
- **[CODE_REVIEW_SPATIAL_REASONING.md](./07-testing/CODE_REVIEW_SPATIAL_REASONING.md)** — 空间推理代码审查
- **[server_test_report.md](./07-testing/server_test_report.md)** — 服务器测试报告
- **[testing_metrics_summary.md](./07-testing/testing_metrics_summary.md)** — 测试指标汇总
- **[FINAL_HONEST_SUMMARY.md](./07-testing/FINAL_HONEST_SUMMARY.md)** — 诚实评估总结

**计划与清单**
- **[REGRESSION_CHECKLIST_V1_3.md](./07-testing/REGRESSION_CHECKLIST_V1_3.md)** — V1.3回归测试清单
- **[TEST_SUPPLEMENT_PLAN.md](./07-testing/TEST_SUPPLEMENT_PLAN.md)** — 测试补充计划
- **[quantitative_experiments_guide.md](./07-testing/quantitative_experiments_guide.md)** — 量化实验指南

**参考论文** ([papers/](./07-testing/papers/))
- AdaNav, ESCA, OmniNav, SG-Nav, VLingNav (PDF + TXT)

---

### [08-project-management](./08-project-management/) — 项目管理
待办事项、变更日志、里程碑和交付记录。

**当前状态**
- **[TODO.md](./08-project-management/TODO.md)** — 待办事项
- **[CHANGELOG.md](./08-project-management/CHANGELOG.md)** — 变更日志
- **[PRODUCT_REVIEW.md](./08-project-management/PRODUCT_REVIEW.md)** — 产品评审
- **[GAP_ANALYSIS.md](./08-project-management/GAP_ANALYSIS.md)** — 深度缺口分析

**计划与路线**
- **[ROADMAP_V2_2025_PAPERS.md](./08-project-management/ROADMAP_V2_2025_PAPERS.md)** — V2路线图（基于2025论文）
- **[complete_implementation_plan.md](./08-project-management/complete_implementation_plan.md)** — 完整实现计划
- **[DOCS_REORGANIZATION_PLAN.md](./08-project-management/DOCS_REORGANIZATION_PLAN.md)** — 文档重组计划

**里程碑报告**
- **[phase1_completion_summary.md](./08-project-management/phase1_completion_summary.md)** — 第一阶段完成总结
- **[phase2_progress_summary.md](./08-project-management/phase2_progress_summary.md)** — 第二阶段进度总结
- **[stage1_completion_summary.md](./08-project-management/stage1_completion_summary.md)** — Stage 1完成总结
- **[stage2_completion_summary.md](./08-project-management/stage2_completion_summary.md)** — Stage 2完成总结
- **[stage2_progress_summary.md](./08-project-management/stage2_progress_summary.md)** — Stage 2进度总结
- **[project_progress_report.md](./08-project-management/project_progress_report.md)** — 项目进度报告
- **[project_progress_report_updated.md](./08-project-management/project_progress_report_updated.md)** — 项目进度报告（更新版）
- **[project_summary.md](./08-project-management/project_summary.md)** — 项目总结
- **[final_project_summary.md](./08-project-management/final_project_summary.md)** — 最终项目总结
- **[project_final_status.md](./08-project-management/project_final_status.md)** — 项目最终状态

**交付**
- **[PROJECT_COMPLETE.md](./08-project-management/PROJECT_COMPLETE.md)** — 项目完成报告
- **[DELIVERY_CONFIRMATION.md](./08-project-management/DELIVERY_CONFIRMATION.md)** — 交付确认书
- **[REORGANIZATION_SUMMARY.md](./08-project-management/REORGANIZATION_SUMMARY.md)** — 重组总结

---

### [09-paper](./09-paper/) — 论文材料
IEEE论文草稿、文献综述、创新评估。

**论文正文**
- **[IEEE_Paper_Complete.md](./09-paper/IEEE_Paper_Complete.md)** — 论文整合草稿
- **[01_abstract_intro.md](./09-paper/01_abstract_intro.md)** — 摘要与引言
- **[02_related_work.md](./09-paper/02_related_work.md)** — 相关工作
- **[03_method.md](./09-paper/03_method.md)** — 方法
- **[03b_belief_graph.md](./09-paper/03b_belief_graph.md)** — 信念图
- **[04_experiments.md](./09-paper/04_experiments.md)** — 实验
- **[05_conclusion_refs.md](./09-paper/05_conclusion_refs.md)** — 结论与参考文献

**评估与分析**
- **[HONEST_REVIEW.md](./09-paper/HONEST_REVIEW.md)** — 诚实论文对标审阅
- **[innovation_assessment.md](./09-paper/innovation_assessment.md)** — 创新性评估
- **[INNOVATION_REASSESSMENT.md](./09-paper/INNOVATION_REASSESSMENT.md)** — 创新性重新评估
- **[RELATED_WORK_COMPARISON.md](./09-paper/RELATED_WORK_COMPARISON.md)** — 相关工作对比
- **[ARCHITECTURE_COMPARISON.md](./09-paper/ARCHITECTURE_COMPARISON.md)** — 架构对比
- **[paper_level_comparison.md](./09-paper/paper_level_comparison.md)** — 论文级别对比
- **[PAPER_GRADE_EVALUATION.md](./09-paper/PAPER_GRADE_EVALUATION.md)** — 论文等级评估

**实验与缺口分析**
- **[EXPERIMENT_GAP_ANALYSIS.md](./09-paper/EXPERIMENT_GAP_ANALYSIS.md)** — 实验缺口分析
- **[GAP_FIX_REPORT_2026-02-17.md](./09-paper/GAP_FIX_REPORT_2026-02-17.md)** — 缺口修复报告
- **[CROSS_REPO_GAP_ANALYSIS_2026-02-16.md](./09-paper/CROSS_REPO_GAP_ANALYSIS_2026-02-16.md)** — 跨仓库缺口分析
- **[METHOD_CURRENT_IMPLEMENTATION_2026-02-17.md](./09-paper/METHOD_CURRENT_IMPLEMENTATION_2026-02-17.md)** — 方法当前实现状态
- **[EVIDENCE_BASED_STATUS_2026-02-16.md](./09-paper/EVIDENCE_BASED_STATUS_2026-02-16.md)** — 证据化现状

**文献与LOVON集成**
- **[LITERATURE_RESEARCH.md](./09-paper/LITERATURE_RESEARCH.md)** — 文献研究综述
- **[RELATED_WORK_DRAFT.md](./09-paper/RELATED_WORK_DRAFT.md)** — 相关工作草稿
- **[LOVON_INTEGRATION_ANALYSIS.md](./09-paper/LOVON_INTEGRATION_ANALYSIS.md)** — LOVON集成分析
- **[LOVON_INTEGRATION_DECISION.md](./09-paper/LOVON_INTEGRATION_DECISION.md)** — LOVON集成决策
- **[LOVON_SYNTHETIC_DATA_GENERATION.md](./09-paper/LOVON_SYNTHETIC_DATA_GENERATION.md)** — LOVON合成数据生成
- **[GOAL_RESOLVER_IMPLEMENTATION.md](./09-paper/GOAL_RESOLVER_IMPLEMENTATION.md)** — 目标解析器实现
- **[VERIFICATION_CORRECTION_NOTICE_2026-02-16.md](./09-paper/VERIFICATION_CORRECTION_NOTICE_2026-02-16.md)** — 数据校正声明

详细索引见 [09-paper/README.md](./09-paper/README.md)

---

### [presentations](./presentations/) — 演示材料
- **HSG-Nav_Presentation.pptx** — HSG-Nav技术演示
- **NaviMind_Pitch_Deck.pptx** — NaviMind商业路演

---

### [guides](./guides/) — 外部集成指南
- **[FEISHU_BOT_SETUP.md](./guides/FEISHU_BOT_SETUP.md)** — 飞书机器人配置
- **[TELEGRAM_BOT_SETUP.md](./guides/TELEGRAM_BOT_SETUP.md)** — Telegram机器人配置

---

### [archive](./archive/) — 归档
历史版本、过程文档和一次性会话记录。

---

## 快速链接

| 角色 | 入口 |
|---|---|
| 新用户 | [快速启动](./01-getting-started/QUICK_START.md) → [系统架构](./02-architecture/ARCHITECTURE.md) |
| 开发者 | [API参考](./03-development/API_REFERENCE.md) → [算法参考](./02-architecture/ALGORITHM_REFERENCE.md) → [故障排查](./03-development/TROUBLESHOOTING.md) |
| 运维 | [部署指南](./01-getting-started/DEPLOY.md) → [Docker](./04-deployment/DOCKER_GUIDE.md) → [OTA](./04-deployment/OTA_GUIDE.md) |
| 研究 | [Fast-Slow实现](./06-semantic-nav/FAST_SLOW_IMPLEMENTATION.md) → [论文草稿](./09-paper/IEEE_Paper_Complete.md) |

---

## 项目状态

**当前版本**: 1.5.0
**核心功能**: 几何导航 ✅ | 语义导航 ✅ | Flutter控制端 ✅
**论文状态**: 见 [09-paper/README.md](./09-paper/README.md)
**详细状态**: [06-semantic-nav/PROJECT_STATUS.md](./06-semantic-nav/PROJECT_STATUS.md)
