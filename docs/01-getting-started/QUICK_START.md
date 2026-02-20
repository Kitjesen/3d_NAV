# 3D语义导航系统升级 - 快速启动指南

## 🚀 项目概览

本项目将3D-NAV语义导航系统从简易实现升级到论文级别，基于2024-2026年最新的VLN研究成果。

**项目路径**: `D:\robot\code\3dnav\3d_NAV`
**团队**: semantic-nav-upgrade (4名工程师)
**进度**: 1/15任务完成 (6.7%)

---

## 📋 快速检查清单

### 环境准备
```bash
# 1. 进入项目目录
cd D:\robot\code\3dnav\3d_NAV

# 2. 安装依赖
bash scripts/install_deps.sh

# 3. 验证安装
python3 -c "import jieba; print('✓ jieba installed')"
python3 -c "import ultralytics; print('✓ ultralytics installed')"
python3 -c "import open_clip; print('✓ open-clip-torch installed')"

# 4. 运行测试
cd tests
pytest test_chinese_tokenizer.py -v
```

### 配置API密钥（可选，用于LLM功能）
```bash
export OPENAI_API_KEY='your-key-here'
export ANTHROPIC_API_KEY='your-key-here'
export DASHSCOPE_API_KEY='your-key-here'
```

---

## 📚 核心文档导航

### 必读文档
1. **SEMANTIC_NAV_REPORT.md** - 原始技术报告（了解系统架构）
2. **UPGRADE_PLAN.md** - 升级计划（了解改进方案）
3. **ALGORITHM_REFERENCE.md** - 算法参考（含代码示例）

### 参考文档
4. **WORK_SUMMARY.md** - 工作总结（了解当前进展）
5. **PROJECT_STATUS.md** - 项目状态（跟踪任务进度）
6. **CHINESE_TOKENIZER_GUIDE.md** - 中文分词指南（已完成功能）

---

## 🎯 核心技术要点

### Fast-Slow双进程架构
```python
# Fast Path (System 1) - 无需LLM，~10ms
if fused_score > 0.75:
    return fast_path_result  # 70%的场景走这里

# Slow Path (System 2) - 调用LLM，~2s
else:
    filtered_graph = esca_filter(scene_graph)  # 200物体→15物体
    return llm_resolve(filtered_graph)
```

### 多源置信度融合
```python
fused_score = (0.35 × label_match +      # 文本匹配
               0.35 × CLIP_similarity +   # 视觉-语言相似度
               0.15 × detector_score +    # 检测器置信度
               0.15 × spatial_hint)       # 空间关系
```

### ESCA选择性Grounding
```
完整场景图 (200物体)
  ↓ 关键词匹配
  ↓ 1-hop关系扩展
  ↓ 区域扩展
过滤后场景图 (15物体) → 发送给LLM
```

---

## 🔧 使用已完成的功能

### 中文分词优化（已完成）

```python
# 方式1: 简单使用
from semantic_planner.chinese_tokenizer import extract_keywords

keywords = extract_keywords("去红色灭火器旁边")
# 结果: ["红色", "灭火器", "旁边"]

# 方式2: 高级配置
from semantic_planner.chinese_tokenizer import ChineseTokenizer

tokenizer = ChineseTokenizer(use_jieba=True)
keywords = tokenizer.extract_keywords(
    "请导航到会议室左边的红色灭火器旁边",
    min_length=2,
    filter_stopwords=True,
    keep_colors=True,
    keep_spatial=True
)
# 结果: ["导航", "会议室", "左边", "红色", "灭火器", "旁边"]

# 方式3: 在goal_resolver中自动使用
# goal_resolver._extract_keywords() 已自动集成jieba
```

---

## 📊 任务状态一览

### 已完成 ✅
- [x] 任务#11: 中文分词优化（jieba集成）

### 进行中 🔄
- [ ] 任务#1: 升级YOLO-World检测器（perception-engineer）
- [ ] 任务#4: 升级目标解析器（planner-engineer）
- [ ] 任务#8: 升级动作执行器（integration-engineer）
- [ ] 任务#10: 实现单元测试套件（test-engineer）

### 待开始 ⏳
- [ ] 任务#2: 升级CLIP编码器
- [ ] 任务#3: 升级实例跟踪器
- [ ] 任务#5: 升级Frontier评分器
- [ ] 任务#6: 升级拓扑记忆
- [ ] 任务#7: 升级任务分解器
- [ ] 任务#9: 升级LLM客户端
- [ ] 任务#12: TensorRT优化
- [ ] 任务#13: 视觉验证闭环
- [ ] 任务#14: 配置管理优化
- [ ] 任务#15: 技术文档编写

---

## 🎓 论文参考清单

### 核心参考（2025-2026）
1. **VLingNav** (arXiv 2601.08665, 2026)
   - Fast-Slow双进程架构
   - 70%场景用System 1完成

2. **ESCA/SGCLIP** (NeurIPS 2025)
   - 选择性Grounding
   - 200物体→15物体，tokens减少90%

3. **MTU3D** (ICCV 2025)
   - Frontier Grounding Potential
   - 探索效率提升14-23%

4. **AdaNav** (ICLR 2026)
   - 多源置信度融合
   - 不确定性自适应

5. **OmniNav** (ICLR 2026)
   - 统一Fast-Slow系统
   - 5Hz控制频率

### 基础参考（2023-2024）
6. **ConceptGraphs** (ICRA 2024) - 增量式场景图
7. **L3MVN** (ICRA 2024) - 语言引导拓扑图
8. **VLMnav** (2024) - 拓扑图+VLM验证
9. **LOVON** (2024) - 四足VLN动作原语
10. **SG-Nav** (NeurIPS 2024) - 层次场景图

---

## 🛠️ 开发工作流

### 1. 查看任务状态
```bash
# 查看任务列表
cat ~/.claude/tasks/semantic-nav-upgrade/*.json | grep -E "subject|status"
```

### 2. 运行测试
```bash
cd tests

# 运行单个测试
pytest test_chinese_tokenizer.py -v

# 运行所有测试
pytest -v

# 运行带覆盖率的测试
pytest --cov=semantic_planner --cov=semantic_perception -v
```

### 3. 查看代码
```bash
# 感知模块
ls -la src/semantic_perception/semantic_perception/

# 规划模块
ls -la src/semantic_planner/semantic_planner/

# 测试
ls -la tests/
```

### 4. 查看文档
```bash
# 所有文档
ls -la docs/

# 查看特定文档
cat docs/ALGORITHM_REFERENCE.md
cat docs/UPGRADE_PLAN.md
```

---

## 🎯 性能目标

| 指标 | 当前 | 目标 | 状态 |
|------|------|------|------|
| Fast Path响应 | ~2s | <200ms | 🔄 待实现 |
| 端到端成功率 | 未测 | >75% | 🔄 待测试 |
| API费用 | 高 | 降低90% | 🔄 10%完成 |
| 检测帧率 | 未优化 | >10 FPS | 🔄 待TensorRT |
| 内存占用 | 未测 | <4GB | 🔄 待测试 |

---

## 🐛 故障排查

### 问题1: jieba未安装
```bash
pip install jieba
# 或使用安装脚本
bash scripts/install_deps.sh
```

### 问题2: 测试失败
```bash
# 检查依赖
pip list | grep -E "jieba|pytest|numpy"

# 重新安装
pip install --upgrade jieba pytest numpy
```

### 问题3: 找不到模块
```bash
# 确保在正确的目录
cd D:\robot\code\3dnav\3d_NAV

# 设置PYTHONPATH
export PYTHONPATH=$PYTHONPATH:$(pwd)/src
```

---

## 📞 获取帮助

### 查看文档
- 技术问题: 查看 `ALGORITHM_REFERENCE.md`
- 使用问题: 查看 `CHINESE_TOKENIZER_GUIDE.md`
- 进度问题: 查看 `WORK_SUMMARY.md`

### 团队成员
- perception-engineer: 感知模块
- planner-engineer: 规划模块
- integration-engineer: 集成和基础设施
- test-engineer: 测试和文档

---

## 🎉 下一步

1. **查看文档**: 阅读 `UPGRADE_PLAN.md` 了解完整方案
2. **运行测试**: 验证已完成的中文分词功能
3. **等待进展**: 团队成员正在并行工作
4. **跟踪状态**: 查看 `WORK_SUMMARY.md` 了解最新进展

---

**最后更新**: 2026-02-15
**项目状态**: 进行中
**完成度**: 6.7% (1/15)
