# 3D-NAV 文档中心

**项目**: 3D-NAV 四足机器人导航系统
**平台**: Jetson Orin NX 16GB | ROS2 Humble
**最后更新**: 2026-02-18

---

## 📚 文档导航

### 🚀 [01-getting-started](./01-getting-started/) - 快速开始
新用户从这里开始，了解如何构建和部署系统。

- **[QUICK_START.md](./01-getting-started/QUICK_START.md)** - 快速启动指南
- **[BUILD_GUIDE.md](./01-getting-started/BUILD_GUIDE.md)** - 构建指南
- **[DEPLOY.md](./01-getting-started/DEPLOY.md)** - 部署指南

---

### 🏗️ [02-architecture](./02-architecture/) - 架构设计
了解系统架构、算法原理和通信协议。

- **[ARCHITECTURE.md](./02-architecture/ARCHITECTURE.md)** - 系统架构文档
- **[ALGORITHM_REFERENCE.md](./02-architecture/ALGORITHM_REFERENCE.md)** - 核心算法参考
- **[TOPIC_CONTRACT.md](./02-architecture/TOPIC_CONTRACT.md)** - ROS2话题契约

---

### 💻 [03-development](./03-development/) - 开发指南
开发者必读，API参考、参数调优和故障排查。

- **[API_REFERENCE.md](./03-development/API_REFERENCE.md)** - API参考文档
- **[PARAMETER_TUNING.md](./03-development/PARAMETER_TUNING.md)** - 参数调优指南
- **[TROUBLESHOOTING.md](./03-development/TROUBLESHOOTING.md)** - 故障排查手册
- **[CODE_REFACTOR.md](./03-development/CODE_REFACTOR.md)** - 代码重构记录

---

### 🚢 [04-deployment](./04-deployment/) - 部署运维
Docker容器化、OTA升级和生产环境部署。

- **[DOCKER_GUIDE.md](./04-deployment/DOCKER_GUIDE.md)** - Docker部署指南
- **[OTA_GUIDE.md](./04-deployment/OTA_GUIDE.md)** - OTA升级指南
- **[OTA_V3_REPORT.md](./04-deployment/OTA_V3_REPORT.md)** - OTA V3技术报告

---

### 🔧 [05-specialized](./05-specialized/) - 专项指南
WebRTC、网关、通信优化等专项技术文档。

- **[WEBRTC_GUIDE.md](./05-specialized/WEBRTC_GUIDE.md)** - WebRTC视频流指南
- **[gateway.md](./05-specialized/gateway.md)** - 网关设计文档
- **[PROTO_REGEN_PLAYBOOK.md](./05-specialized/PROTO_REGEN_PLAYBOOK.md)** - Protobuf重新生成手册
- **[COMM_OPTIMIZATION.md](./05-specialized/COMM_OPTIMIZATION.md)** - 通信优化方案
- **[DECOUPLING_ANALYSIS.md](./05-specialized/DECOUPLING_ANALYSIS.md)** - 解耦分析文档

---

### 🤖 [06-semantic-nav](./06-semantic-nav/) - 语义导航（核心）
语义导航子系统的技术报告、升级计划和实现状态。

- **[SEMANTIC_NAV_REPORT.md](./06-semantic-nav/SEMANTIC_NAV_REPORT.md)** - 语义导航技术报告
- **[SEMANTIC_NAV_STATUS.md](./06-semantic-nav/SEMANTIC_NAV_STATUS.md)** - 实现状态检查
- **[UPGRADE_PLAN.md](./06-semantic-nav/UPGRADE_PLAN.md)** - 论文级升级计划
- **[PROJECT_STATUS.md](./06-semantic-nav/PROJECT_STATUS.md)** - 项目状态跟踪
- **[TASK_11_COMPLETION_REPORT.md](./06-semantic-nav/TASK_11_COMPLETION_REPORT.md)** - 任务完成报告
- **[README_UPGRADE.md](./06-semantic-nav/README_UPGRADE.md)** - 语义导航升级项目说明

**核心技术**:
- Fast-Slow双进程架构 (VLingNav 2026)
- ESCA选择性Grounding (NeurIPS 2025)
- MTU3D Frontier评分 (ICCV 2025)
- ConceptGraphs场景图 (ICRA 2024)
- LOVON动作原语 (2024)

---

### 🧪 [07-testing](./07-testing/) - 测试相关
测试报告和回归测试清单。

- **[TEST_REPORT_20260208.md](./07-testing/TEST_REPORT_20260208.md)** - 测试报告
- **[REGRESSION_CHECKLIST_V1_3.md](./07-testing/REGRESSION_CHECKLIST_V1_3.md)** - 回归测试清单

---

### 📋 [08-project-management](./08-project-management/) - 项目管理
待办事项、变更日志和产品评审。

- **[TODO.md](./08-project-management/TODO.md)** - 待办事项列表
- **[CHANGELOG.md](./08-project-management/CHANGELOG.md)** - 变更日志
- **[PRODUCT_REVIEW.md](./08-project-management/PRODUCT_REVIEW.md)** - 产品评审文档
- **[DOCS_REORGANIZATION_PLAN.md](./08-project-management/DOCS_REORGANIZATION_PLAN.md)** - 文档重组计划
- **[PROJECT_COMPLETE.md](./08-project-management/PROJECT_COMPLETE.md)** - 项目完成报告
- **[GAP_ANALYSIS.md](./08-project-management/GAP_ANALYSIS.md)** - 深度缺口分析
- **[DELIVERY_CONFIRMATION.md](./08-project-management/DELIVERY_CONFIRMATION.md)** - 交付确认书
- **[README_COMPLETION.md](./08-project-management/README_COMPLETION.md)** - 项目完成总结

---

### 📝 [09-paper](./09-paper/) - 论文材料
论文草稿、创新评估与证据化状态报告。

- **[README.md](./09-paper/README.md)** - 论文文档索引
- **[VERIFICATION_CORRECTION_NOTICE_2026-02-16.md](./09-paper/VERIFICATION_CORRECTION_NOTICE_2026-02-16.md)** - 数据校正声明
- **[EVIDENCE_BASED_STATUS_2026-02-16.md](./09-paper/EVIDENCE_BASED_STATUS_2026-02-16.md)** - 证据化现状
- **[IEEE_Paper_Complete.md](./09-paper/IEEE_Paper_Complete.md)** - 论文整合草稿
- **[HONEST_REVIEW.md](./09-paper/HONEST_REVIEW.md)** - 诚实论文对标审阅
- **[LITERATURE_RESEARCH.md](./09-paper/LITERATURE_RESEARCH.md)** - 文献研究综述

---

## 🎯 快速链接

### 新用户
1. [快速启动](./01-getting-started/QUICK_START.md) - 5分钟上手
2. [系统架构](./02-architecture/ARCHITECTURE.md) - 了解整体设计
3. [语义导航报告](./06-semantic-nav/SEMANTIC_NAV_REPORT.md) - 核心功能介绍

### 开发者
1. [API参考](./03-development/API_REFERENCE.md) - 接口文档
2. [算法参考](./02-architecture/ALGORITHM_REFERENCE.md) - 算法实现
3. [故障排查](./03-development/TROUBLESHOOTING.md) - 常见问题

### 运维人员
1. [部署指南](./01-getting-started/DEPLOY.md) - 生产环境部署
2. [Docker指南](./04-deployment/DOCKER_GUIDE.md) - 容器化部署
3. [OTA升级](./04-deployment/OTA_GUIDE.md) - 固件升级

---

## 📊 项目状态

**当前版本**: V1.3
**语义导航升级**: 进行中（以证据化报告为准）
**核心功能**: ✅ 几何导航 | 🔄 语义导航 | ✅ Flutter控制端

查看详细状态: [PROJECT_STATUS.md](./06-semantic-nav/PROJECT_STATUS.md)
论文状态入口: [09-paper/README.md](./09-paper/README.md)

---

## 🔗 相关资源

- **代码仓库**: `D:\robot\code\3dnav\3d_NAV`
- **配置文件**: `config/`
- **测试套件**: `tests/`
- **ROS2包**: `src/`

---

## 📞 联系方式

**项目路径**: D:\robot\code\3dnav\3d_NAV
**文档路径**: D:\robot\code\3dnav\3d_NAV\docs

---

**文档结构最后更新**: 2026-02-18
