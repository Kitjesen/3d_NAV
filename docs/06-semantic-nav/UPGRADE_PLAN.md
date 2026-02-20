# 3D语义导航系统论文级升级计划

## 项目状态
- **启动时间**: 2026-02-15
- **团队**: semantic-nav-upgrade (4名专业工程师)
- **总任务数**: 15个核心升级任务
- **当前状态**: 团队已启动，正在并行工作

## 升级目标

### 性能指标
| 指标 | 当前 | 目标 | 论文参考 |
|------|------|------|---------|
| 指令响应延迟 | ~2s | <200ms (Fast Path) | VLingNav 2026 |
| 端到端成功率 | 未知 | >75% | VLN基准 |
| API费用 | 高 | 降低90% | Fast Path优化 |
| 检测帧率 | 未优化 | >10 FPS | YOLO-World + TRT |
| 内存占用 | 未测 | <4GB额外 | Jetson预算 |

### 架构升级

#### 1. 感知层 (semantic_perception)
**当前问题**: 简易实现，缺少论文级优化

**升级方案**:
- **YOLO-World检测器** → CVPR 2024完整实现
  - TensorRT优化流程
  - 动态类别缓存
  - 批处理推理
  - RepVL-PAN架构

- **CLIP编码器** → OpenCLIP论文级
  - 多尺度特征提取
  - 特征缓存和索引
  - 批处理优化
  - 真实集成到Fast Path

- **实例跟踪器** → ConceptGraphs ICRA 2024
  - 增量式3D场景图
  - 多视角融合
  - 完整空间关系推理（8种）
  - 优化DBSCAN聚类

#### 2. 规划层 (semantic_planner)
**当前问题**: Fast-Slow架构未完全实现，缺少ESCA过滤

**升级方案**:
- **目标解析器** → VLingNav 2026 + ESCA NeurIPS 2025
  - 完整Fast-Slow双进程
  - 多源置信度融合: 0.35×标签 + 0.35×CLIP + 0.15×检测器 + 0.15×空间
  - ESCA选择性Grounding (200物体→15物体)
  - 自适应阈值调整
  - VLMnav视觉验证闭环

- **Frontier评分器** → MTU3D ICCV 2025
  - Grounding Potential计算
  - 空间梯度分析
  - 关系链延伸
  - 常识共现知识库

- **拓扑记忆** → VLMnav 2024 + L3MVN ICRA 2024
  - 完整拓扑图构建
  - CLIP特征匹配
  - 8扇区访问密度
  - 智能节点裁剪

- **任务分解器** → SayCan + Inner Monologue
  - 8种子目标类型
  - 可行性评分
  - 动态重规划
  - 失败恢复

#### 3. 执行层
**当前问题**: 动作原语实现不完整

**升级方案**:
- **动作执行器** → LOVON 2024
  - 6种动作原语精确控制
  - yaw自动对准
  - LOOK_AROUND精确360度
  - APPROACH减速策略
  - 碰撞避免

#### 4. 基础设施
**当前问题**: 缺少测试、中文支持弱、TensorRT未验证

**升级方案**:
- **单元测试**: 102个测试用例覆盖全模块
- **中文分词**: jieba替换简单regex
- **TensorRT**: Jetson Orin NX端到端优化
- **LLM客户端**: GPT-4o/Claude/Qwen多后端容错
- **视觉验证**: GPT-4o Vision集成
- **配置管理**: 动态参数重配置

## 团队分工

### 1. perception-engineer
**负责模块**: semantic_perception
**任务列表**:
- [#1] 升级YOLO-World检测器
- [#2] 升级CLIP编码器
- [#3] 升级实例跟踪器

**关键交付**:
- TensorRT优化达到>10 FPS
- CLIP真实集成到Fast Path
- ConceptGraphs增量式场景图

### 2. planner-engineer
**负责模块**: semantic_planner核心算法
**任务列表**:
- [#4] 升级目标解析器
- [#5] 升级Frontier评分器
- [#6] 升级拓扑记忆
- [#7] 升级任务分解器

**关键交付**:
- Fast-Slow双进程架构
- ESCA选择性Grounding
- MTU3D Grounding Potential
- 完整的8种子目标类型

### 3. integration-engineer
**负责模块**: 系统集成和基础设施
**任务列表**:
- [#8] 升级动作执行器
- [#9] 升级LLM客户端
- [#11] 中文分词优化
- [#12] TensorRT优化
- [#13] 视觉验证闭环
- [#14] 配置和ROS2集成

**关键交付**:
- LOVON 6种动作原语
- 多后端LLM容错
- jieba中文分词
- Jetson TensorRT验证
- GPT-4o Vision集成

### 4. test-engineer
**负责模块**: 测试和文档
**任务列表**:
- [#10] 实现102个单元测试
- [#15] 编写技术文档

**关键交付**:
- 7个测试文件，102个测试用例
- 算法原理文档
- API参考文档
- 部署指南

## 核心算法详解

### Fast-Slow双进程架构 (VLingNav 2026)

```
用户指令 → TaskDecomposer → GoalResolver
                                    ↓
                    ┌───────────────┴───────────────┐
                    ↓                               ↓
            Fast Path (System 1)            Slow Path (System 2)
            ├─ 场景图匹配                    ├─ ESCA过滤
            ├─ 多源置信度融合                ├─ LLM推理
            ├─ ~10ms, 免费                  ├─ ~2s, API费用
            └─ 置信度>0.75 ✓                └─ 可选VLM验证
                    ↓                               ↓
                    └───────────────┬───────────────┘
                                    ↓
                            ActionExecutor
```

**价值**: 70%+步骤用Fast Path，延迟降低99.5%，API费用降低90%

### 多源置信度融合 (AdaNav ICLR 2026)

```python
fused_score = 0.35 × label_match      # 文本匹配
            + 0.35 × CLIP_similarity   # 视觉-语言相似度
            + 0.15 × detector_score    # 检测器置信度 × 观测次数
            + 0.15 × spatial_hint      # 空间关系命中

if fused_score > 0.75:
    return fast_path_result  # 直接输出坐标
else:
    return slow_path_result  # 调用LLM
```

### ESCA选择性Grounding (NeurIPS 2025)

```
完整场景图 (200物体)
    ↓
关键词匹配 → 提取相关物体 (5-10个)
    ↓
1-hop关系扩展 → 添加邻居物体 (3-5个)
    ↓
区域扩展 → 添加同区域物体 (2-3个)
    ↓
过滤后场景图 (15物体) → 发送给LLM
```

**价值**: tokens减少90%，推理更精准，开源模型可超越GPT-4

### MTU3D Grounding Potential (ICCV 2025)

```python
frontier_score = 0.2 × distance       # 距离: 近的优先
               + 0.3 × novelty        # 新颖度: 未去过优先
               + 0.2 × language       # 语言: 附近有相关物体
               + 0.3 × grounding_pot  # 目标出现概率

grounding_pot = spatial_gradient      # 方向有相关物体
              + relation_chain        # 关联物体在该方向
              + commonsense_cooccur   # 常识共现知识
```

**价值**: 比VLFM基线提升14-23%

## P0优先级任务

### 1. 中文分词优化 (任务#11)
**问题**: 当前简单regex分词不准确
**方案**: 集成jieba分词器
**影响**: 中文指令解析准确率提升

### 2. TensorRT端到端测试 (任务#12)
**问题**: YOLO-World TRT导出未在Jetson验证
**方案**: Jetson Orin NX实测，优化到>10 FPS
**影响**: 实时性能达标

### 3. CLIP真实集成 (任务#2)
**问题**: Fast Path的CLIP分数用标签匹配近似
**方案**: 接入真实CLIP encoder
**影响**: Fast Path置信度更准确

## 论文参考清单

### 核心参考 (2025-2026)
1. **VLingNav** (arXiv 2601.08665, 2026) - Fast-Slow双进程
2. **OmniNav** (ICLR 2026) - 统一Fast-Slow系统
3. **AdaNav** (ICLR 2026) - 多源置信度融合
4. **ESCA/SGCLIP** (NeurIPS 2025) - 选择性Grounding
5. **MTU3D** (ICCV 2025) - Frontier Grounding Potential

### 基础参考 (2023-2024)
6. **SG-Nav** (NeurIPS 2024) - 层次场景图
7. **ConceptGraphs** (ICRA 2024) - 增量式场景图
8. **L3MVN** (ICRA 2024) - 语言引导拓扑图
9. **VLMnav** (2024) - 拓扑图+VLM验证
10. **LOVON** (2024) - 四足VLN动作原语
11. **VLFM** (2023) - Visual-Language Frontier Map
12. **SayCan** (Google, 2022) - LLM任务分解
13. **Inner Monologue** (2022) - Chain-of-thought分解

## 技术栈

### 核心依赖
```bash
# 检测和编码
pip install ultralytics          # YOLO-World
pip install open-clip-torch      # CLIP

# LLM客户端
pip install openai               # GPT-4o
pip install anthropic            # Claude
pip install dashscope            # Qwen

# 中文处理
pip install jieba                # 中文分词

# 测试
pip install pytest pytest-cov    # 单元测试
```

### 硬件要求
- **开发**: 任意GPU (CUDA 11.4+)
- **部署**: Jetson Orin NX 16GB (TensorRT 8.5+)
- **相机**: Orbbec RGB-D (640x480, 30fps)

## 验收标准

### 功能验收
- [ ] Fast Path置信度>0.75时直接输出坐标
- [ ] Slow Path正确调用ESCA过滤+LLM推理
- [ ] 102个单元测试全部通过
- [ ] 支持中文指令（jieba分词）
- [ ] 6种动作原语正确执行

### 性能验收
- [ ] Fast Path响应<200ms
- [ ] YOLO-World检测>10 FPS (Jetson TRT)
- [ ] 内存占用<4GB额外
- [ ] 端到端成功率>75% (室内场景)

### 文档验收
- [ ] 算法原理文档完整
- [ ] API参考文档完整
- [ ] 部署指南可执行
- [ ] 代码注释覆盖率>80%

## 风险和缓解

### 风险1: TensorRT优化困难
**影响**: 检测帧率达不到10 FPS
**缓解**:
- 降级到YOLO-World-M (更小模型)
- 使用INT8量化
- 调整输入分辨率

### 风险2: LLM API不稳定
**影响**: Slow Path失败率高
**缓解**:
- 多后端容错 (GPT-4o/Claude/Qwen)
- 自动重试机制
- 本地缓存常见查询

### 风险3: 中文分词效果不佳
**影响**: 中文指令解析错误
**缓解**:
- 自定义词典（机器人领域词汇）
- 结合规则和jieba
- 添加纠错机制

## 下一步行动

### 立即行动
1. 等待团队成员完成首批任务
2. 审查代码质量和测试覆盖
3. 在Jetson上验证TensorRT性能

### 短期计划 (1-2周)
1. 完成所有P0任务
2. 实现102个单元测试
3. 完成核心模块升级

### 中期计划 (1个月)
1. 完成所有15个任务
2. 端到端集成测试
3. 性能调优和文档完善

### 长期计划 (2-3个月)
1. 实际场景测试和优化
2. 多机器人协作扩展
3. 本地小模型替代LLM API

## 联系和协作

**项目路径**: D:\robot\code\3dnav\3d_NAV
**团队名称**: semantic-nav-upgrade
**文档路径**: docs/SEMANTIC_NAV_REPORT.md

**团队成员**:
- perception-engineer: 感知模块
- planner-engineer: 规划模块
- integration-engineer: 集成和基础设施
- test-engineer: 测试和文档

---

*本文档将随项目进展持续更新*
