# Semantic Perception API

**版本**: 1.8.0
**状态**: ✅ 生产就绪
**完成日期**: 2026-02-17 | **最后更新**: 2026-03-12

---

## 🎯 概述

Semantic Perception API是3D-NAV项目的核心感知模块，提供了一个**专业级的API接口层**，用于处理RGB-D图像、检测物体、提取特征、追踪实例和构建场景图。

### 核心特性

- ✅ **统一的API接口** - 清晰的抽象层
- ✅ **工厂模式** - 简化创建过程
- ✅ **依赖注入** - 松耦合设计
- ✅ **完整类型注解** - 类型安全
- ✅ **统一异常处理** - 易于调试
- ✅ **模块化设计** - 易于扩展

---

## 🚀 快速开始

### 安装依赖

```bash
pip install ultralytics open-clip-torch numpy
```

### 基本使用

```python
from semantic_perception.api import PerceptionFactory, CameraInfo
import numpy as np

# 1. 创建感知系统
perception = PerceptionFactory.create_perception(
    detector_type="yolo_world",
    encoder_type="clip"
)

# 2. 设置检测类别
perception.detector.set_classes(["chair", "table", "person"])

# 3. 准备输入
rgb_image = np.zeros((480, 640, 3), dtype=np.uint8)
depth_image = np.zeros((480, 640), dtype=np.float32)
camera_info = CameraInfo(
    fx=525.0, fy=525.0,
    cx=319.5, cy=239.5,
    width=640, height=480
)

# 4. 处理图像
detections = perception.process_frame(rgb_image, depth_image, camera_info)

# 5. 获取场景图
scene_graph = perception.get_scene_graph()

print(f"检测到 {len(detections)} 个物体")
print(f"场景图包含 {len(scene_graph.relations)} 个关系")
```

---

## 📚 API文档

### 核心接口

#### PerceptionAPI

完整的感知系统接口。

```python
from semantic_perception.api import PerceptionFactory

perception = PerceptionFactory.create_perception()

# 处理图像
detections = perception.process_frame(rgb, depth, camera_info)

# 获取场景图
scene_graph = perception.get_scene_graph()

# 查询物体
chairs = perception.query_objects(label="chair", min_confidence=0.5)

# 获取统计
stats = perception.get_statistics()

# 重置系统
perception.reset()
```

#### DetectorAPI

物体检测接口。

```python
from semantic_perception.api import PerceptionFactory

detector = PerceptionFactory.create_detector("yolo_world")

# 设置类别
detector.set_classes(["chair", "table", "person"])

# 检测物体
detections = detector.detect(image)

# 获取模型信息
info = detector.get_model_info()
```

#### EncoderAPI

特征编码接口。

```python
from semantic_perception.api import PerceptionFactory

encoder = PerceptionFactory.create_encoder("clip")

# 编码图像
image_feat = encoder.encode_image(image)

# 编码文本
text_feat = encoder.encode_text("a red chair")

# 计算相似度
similarity = encoder.compute_similarity(image_feat, text_feat)
```

#### TrackerAPI

实例追踪接口。

```python
from semantic_perception.api import PerceptionFactory

tracker = PerceptionFactory.create_tracker("instance")

# 更新追踪
tracked = tracker.update(detections_3d)

# 获取所有追踪
all_tracks = tracker.get_all_tracks()

# 重置追踪器
tracker.reset()
```

---

## 🏗️ 架构设计

### 层次结构

```
┌─────────────────────────────────────┐
│         PerceptionAPI               │  ← 顶层接口
│  (process_frame, get_scene_graph)  │
└─────────────────────────────────────┘
              ↓ 依赖注入
┌─────────────────────────────────────┐
│  DetectorAPI  EncoderAPI  TrackerAPI│  ← 组件接口
└─────────────────────────────────────┘
              ↓ 实现
┌─────────────────────────────────────┐
│ YOLOWorld    CLIP    InstanceTracker│  ← 具体实现
└─────────────────────────────────────┘
```

### 处理流程

```
RGB + Depth 图像
    ↓
1. 2D检测 (YOLO-World, DetectorAPI)
    ↓
2. CLIP编码 (HOV-SG三源融合: f_g+f_l+f_m, EncoderAPI)
    ↓
3. 3D投影 (内部实现)
    ↓
4. 实例追踪 (DBSCAN特征精炼, TrackerAPI)
    ↓
5. 场景图构建 (RoomNode视角嵌入K=10, 4层结构)
    ↓
Detection3D列表 + SceneGraph + TopologyGraph
```

### HOV-SG 感知升级 (NaviMind 论文)

- **三源 CLIP 融合** (`encode_three_source`): f_g 全局 + f_l 裁剪 + f_m 掩码裁剪, 权重 0.25/0.50/0.25
- **DBSCAN 特征精炼**: 每 5 次检测对 TrackedObject 历史做聚类
- **RoomNode 视角嵌入**: K=10 FIFO, `query_similarity()` 支持房间级语义检索
- **4 层场景图**: Object → ViewNode → RoomNode → BuildingNode

---

## 📦 目录结构

```
semantic_perception/
├── api/                    # API接口层
│   ├── __init__.py
│   ├── types.py           # 数据类型
│   ├── exceptions.py      # 异常定义
│   ├── detector_api.py    # 检测器接口
│   ├── encoder_api.py     # 编码器接口
│   ├── tracker_api.py     # 追踪器接口
│   ├── perception_api.py  # 感知系统接口
│   └── factory.py         # 工厂类
├── impl/                   # 实现层
│   ├── __init__.py
│   ├── yolo_world_detector.py
│   ├── clip_encoder.py
│   ├── instance_tracker.py
│   └── perception_impl.py
└── examples/               # 示例代码
    └── api_usage_examples.py
```

---

## 🎓 设计模式

### 1. 工厂模式

```python
# 使用工厂创建对象
perception = PerceptionFactory.create_perception(
    detector_type="yolo_world",
    encoder_type="clip"
)
```

### 2. 依赖注入

```python
# PerceptionImpl通过构造函数注入依赖
class PerceptionImpl(PerceptionAPI):
    def __init__(
        self,
        detector: DetectorAPI,
        encoder: EncoderAPI,
        tracker: TrackerAPI
    ):
        self.detector = detector
        self.encoder = encoder
        self.tracker = tracker
```

### 3. 策略模式

```python
# 不同的检测器实现可以互换
detector1 = PerceptionFactory.create_detector("yolo_world")
detector2 = PerceptionFactory.create_detector("grounding_dino")
```

---

## 🔧 配置

### 使用配置对象

```python
from semantic_perception.api import PerceptionConfig, PerceptionFactory

config = PerceptionConfig(
    detector_type="yolo_world",
    encoder_type="clip",
    confidence_threshold=0.3,
    iou_threshold=0.5,
    merge_distance=0.5,
    max_depth=6.0,
    min_depth=0.3
)

perception = PerceptionFactory.create_from_config(config)
```

### 运行时重配置

```python
# 修改配置
new_config = PerceptionConfig(
    confidence_threshold=0.5,
    merge_distance=0.8
)

perception.configure(new_config)
```

---

## 🐛 错误处理

### 异常层次

```
PerceptionAPIError
├── DetectorError
│   ├── DetectorInitError
│   └── DetectorInferenceError
├── EncoderError
│   ├── EncoderInitError
│   └── EncoderInferenceError
├── TrackerError
├── InvalidImageError
├── InvalidDepthError
└── ConfigurationError
```

### 使用示例

```python
from semantic_perception.api.exceptions import (
    InvalidImageError,
    PerceptionAPIError
)

try:
    detections = perception.process_frame(rgb, depth, camera_info)
except InvalidImageError as e:
    print(f"图像格式错误: {e}")
except PerceptionAPIError as e:
    print(f"处理失败: {e}")
```

---

## 📊 性能

### 基准测试

| 组件 | 延迟 | 吞吐量 |
|------|------|--------|
| YOLO-World检测 | ~50ms | 20 FPS |
| CLIP编码 | ~30ms | 33 FPS |
| 实例追踪 | ~5ms | 200 FPS |
| **总计** | ~85ms | **11 FPS** |

*测试环境: NVIDIA Jetson AGX Orin*

---

## 🧪 测试

### 运行示例

```bash
cd src/semantic_perception
python examples/api_usage_examples.py
```

### 运行单元测试

```bash
pytest src/semantic_perception/test -v
```

---

## 📖 更多示例

查看 `examples/api_usage_examples.py` 获取7个完整示例：

1. 基本使用
2. 单独使用组件
3. 查询物体
4. 获取统计信息
5. 配置和重配置
6. 错误处理
7. 工厂方法

---

## 🤝 贡献

### 添加新的检测器

1. 创建新类继承 `DetectorAPI`
2. 实现所有抽象方法
3. 在 `factory.py` 中注册

```python
class MyDetector(DetectorAPI):
    def detect(self, image: np.ndarray) -> List[Detection2D]:
        # 实现检测逻辑
        pass
```

### 添加新的编码器

1. 创建新类继承 `EncoderAPI`
2. 实现所有抽象方法
3. 在 `factory.py` 中注册

---

## 📝 文档

- **API方案**: `docs/03-development/API_REFACTORING_PLAN.md`
- **完成报告**: `docs/03-development/API_REFACTORING_COMPLETE.md`
- **总结报告**: `docs/03-development/API_REFACTORING_SUMMARY.md`

---

## 📄 许可证

MIT License

---

## 👥 作者

3D-NAV Team

---

## 🙏 致谢

感谢以下开源项目：

- [YOLO-World](https://github.com/AILab-CVC/YOLO-World) - 开放词汇检测
- [OpenCLIP](https://github.com/mlfoundations/open_clip) - CLIP实现
- [Ultralytics](https://github.com/ultralytics/ultralytics) - YOLO框架

---

**版本**: 1.8.0
**最后更新**: 2026-03-12
**状态**: ✅ 生产就绪
