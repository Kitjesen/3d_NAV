# 文档重组计划

**日期**: 2026-02-15
**当前问题**: 33个文档平铺，大量重复内容，缺少组织结构

---

## 📊 问题分析

### 1. 重复文档（需要合并）
以下5个文档内容高度重复，都在讲项目状态和进度：
- `PROJECT_STATUS.md` - 项目状态
- `FINAL_SUMMARY.md` - 最终总结
- `DELIVERY_CHECKLIST.md` - 交付清单
- `WORK_SUMMARY.md` - 工作总结
- `PROGRESS_REPORT.md` - 进度报告

**建议**: 合并为1个文档 `PROJECT_STATUS.md`

### 2. 文档分类混乱
- `TODO.md` 内容与其他文档不一致（讲Flutter App，而非语义导航）
- 缺少主README索引
- 没有子目录分类

### 3. 文档数量过多
33个文档全部平铺，难以查找和维护

---

## 🎯 重组方案

### 新的目录结构
```
docs/
├── README.md                          # 📖 文档总索引（新建）
│
├── 01-getting-started/                # 🚀 快速开始
│   ├── QUICK_START.md
│   ├── BUILD_GUIDE.md
│   └── DEPLOY.md
│
├── 02-architecture/                   # 🏗️ 架构设计
│   ├── ARCHITECTURE.md
│   ├── ALGORITHM_REFERENCE.md
│   └── TOPIC_CONTRACT.md
│
├── 03-development/                    # 💻 开发指南
│   ├── API_REFERENCE.md
│   ├── PARAMETER_TUNING.md
│   ├── TROUBLESHOOTING.md
│   └── CODE_REFACTOR.md
│
├── 04-deployment/                     # 🚢 部署运维
│   ├── DOCKER_GUIDE.md
│   ├── OTA_GUIDE.md
│   └── OTA_V3_REPORT.md
│
├── 05-specialized/                    # 🔧 专项指南
│   ├── WEBRTC_GUIDE.md
│   ├── gateway.md
│   ├── PROTO_REGEN_PLAYBOOK.md
│   ├── COMM_OPTIMIZATION.md
│   └── DECOUPLING_ANALYSIS.md
│
├── 06-semantic-nav/                   # 🤖 语义导航（核心）
│   ├── SEMANTIC_NAV_REPORT.md         # 原始技术报告
│   ├── SEMANTIC_NAV_STATUS.md         # 实现状态
│   ├── UPGRADE_PLAN.md                # 升级计划
│   ├── PROJECT_STATUS.md              # 项目状态（合并后）
│   ├── CHINESE_TOKENIZER_GUIDE.md     # 中文分词指南
│   └── TASK_11_COMPLETION_REPORT.md   # 任务报告
│
├── 07-testing/                        # 🧪 测试相关
│   ├── TEST_REPORT_20260208.md
│   └── REGRESSION_CHECKLIST_V1_3.md
│
├── 08-project-management/             # 📋 项目管理
│   ├── TODO.md
│   ├── CHANGELOG.md
│   └── PRODUCT_REVIEW.md
│
└── archive/                           # 📦 归档（删除的文档）
    ├── FINAL_SUMMARY.md               # 合并到PROJECT_STATUS.md
    ├── DELIVERY_CHECKLIST.md          # 合并到PROJECT_STATUS.md
    ├── WORK_SUMMARY.md                # 合并到PROJECT_STATUS.md
    └── PROGRESS_REPORT.md             # 合并到PROJECT_STATUS.md
```

---

## 📝 具体操作

### 阶段1: 创建目录结构
```bash
cd docs
mkdir -p 01-getting-started 02-architecture 03-development 04-deployment
mkdir -p 05-specialized 06-semantic-nav 07-testing 08-project-management archive
```

### 阶段2: 移动文件
```bash
# 快速开始
mv QUICK_START.md BUILD_GUIDE.md DEPLOY.md 01-getting-started/

# 架构设计
mv ARCHITECTURE.md ALGORITHM_REFERENCE.md TOPIC_CONTRACT.md 02-architecture/

# 开发指南
mv API_REFERENCE.md PARAMETER_TUNING.md TROUBLESHOOTING.md CODE_REFACTOR.md 03-development/

# 部署运维
mv DOCKER_GUIDE.md OTA_GUIDE.md OTA_V3_REPORT.md 04-deployment/

# 专项指南
mv WEBRTC_GUIDE.md gateway.md PROTO_REGEN_PLAYBOOK.md COMM_OPTIMIZATION.md DECOUPLING_ANALYSIS.md 05-specialized/

# 语义导航
mv SEMANTIC_NAV_REPORT.md SEMANTIC_NAV_STATUS.md UPGRADE_PLAN.md 06-semantic-nav/
mv CHINESE_TOKENIZER_GUIDE.md TASK_11_COMPLETION_REPORT.md 06-semantic-nav/

# 测试相关
mv TEST_REPORT_20260208.md REGRESSION_CHECKLIST_V1_3.md 07-testing/

# 项目管理
mv TODO.md CHANGELOG.md PRODUCT_REVIEW.md 08-project-management/

# 归档重复文档
mv FINAL_SUMMARY.md DELIVERY_CHECKLIST.md WORK_SUMMARY.md PROGRESS_REPORT.md archive/
```

### 阶段3: 合并重复文档
将以下4个文档的内容合并到 `PROJECT_STATUS.md`：
- FINAL_SUMMARY.md
- DELIVERY_CHECKLIST.md
- WORK_SUMMARY.md
- PROGRESS_REPORT.md

保留 `PROJECT_STATUS.md` 作为唯一的项目状态文档，移动到 `06-semantic-nav/`

### 阶段4: 创建README索引
创建 `docs/README.md` 作为文档导航入口

---

## 📊 文档数量对比

| 类别 | 重组前 | 重组后 | 减少 |
|------|--------|--------|------|
| 总文档数 | 33 | 29 | -4 |
| 顶层文档 | 33 | 1 (README) | -32 |
| 分类目录 | 0 | 8 | +8 |

---

## ✅ 预期效果

1. **清晰的层次结构**: 8个分类目录，一目了然
2. **消除重复**: 合并5个重复文档为1个
3. **易于导航**: README索引快速定位
4. **便于维护**: 相关文档集中管理

---

## 🚀 执行建议

1. **备份**: 先备份整个docs目录
2. **分步执行**: 按阶段逐步重组
3. **验证链接**: 检查文档间的相互引用
4. **更新引用**: 更新代码中的文档路径引用

---

**是否执行此重组计划？**
