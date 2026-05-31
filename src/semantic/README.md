# Semantic — L3/L4 语义感知与决策层

## 概述

Semantic 层负责视觉感知、语义理解、LLM 驱动的目标推理和自主导航规划，是 LingTu 实现"语义导航"能力的核心。

## perception/ — 语义感知

- **Detector**: 目标检测（YOLOE、YOLO-World、Grounding DINO、BPU 硬件加速）
- **Encoder**: 视觉编码（CLIP ViT-B/32、MobileCLIP 边缘端）
- **后端注册**: 通过 `@register("detector", name)` / `@register("encoder", name)` 插件化
- **SCG Builder**: 场景图构建 + 实例跟踪
- **Room Manager**: 房间分割与空间推理
- **Knowledge Graph**: 语义知识图谱

## planner/ — 语义规划与 LLM

- **SemanticPlannerModule**: 5 级降级目标解析（Tag -> Fast Path -> Vector -> Frontier -> Visual Servo）
- **LLM**: Kimi / OpenAI / Claude / Qwen / Mock 多后端支持
- **GoalResolver**: Fast-Slow 双系统 + AdaNav 熵触发 + LERa 失败恢复
- **AgentLoop**: 多轮 LLM 工具调用（7 个工具，最多 10 步 / 120s 超时）
- **VisualServoModule**: 视觉伺服（bbox 导航 + 人员跟随，远/近双通道）
- **TaskDecomposer**: 任务分解

## reconstruction/ — 3D 重建

- `reconstruction_module.py` — 在线 3D 重建
- `keyframe_exporter_module.py` — 关键帧导出
- `dataset_recorder_module.py` — 数据集录制
- `semantic_labeler.py` — 语义标注
- `server/` — 重建服务端
