# Semantic Perception — 模块组织索引

> 文件均位于 `src/semantic/perception/semantic_perception/`
> 共 54 个 .py 文件（含子目录），约 19 016 行
> 本文档说明各文件的功能归属，便于导航和维护。

---

## Module 层（核心框架接入）(3)

> 这三个文件是 Module-First 架构的入口，对接 `core.Module` 框架，其余算法文件不依赖框架。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `perception_module.py` | 531 | **PerceptionModule**：Module-First 感知模块，封装完整 detect→encode→track→scene_graph 流水线；无 ROS2 依赖 |
| `detector_module.py` | 154 | **DetectorModule**：可插拔检测器 Module；`encoder="yoloe"/"bpu"/"yolo_world"` 一行切换后端 |
| `encoder_module.py` | 154 | **EncoderModule**：可插拔编码器 Module；`encoder="clip"/"mobileclip"` 一行切换后端 |

---

## 根节点 / 纯算法编排 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `service.py` | ~200 | **PerceptionService**：纯算法编排（不依赖框架/ROS2），将 detector→projector→encoder→tracker 串联成流水线，可独立 mock 测试 |
| `perception_publishers.py` | 242 | ROS2 话题发布封装：Detection3DArray、scene_graph JSON、detections_3d 统一出口（仅在 ROS2 桥接模式下使用） |
| `sim_scene_observer.py` | ~80 | **SimSceneObserver**：仿真专用场景观察器，从 MuJoCo XML 元数据直接生成 SceneGraph，绕过真实视觉检测 |

---

## Detector — 目标检测后端 (5)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `detector_base.py` | 58 | 抽象基类：`DetectorBase`，定义 `detect(image) → List[Detection]` 接口 |
| `yolo_world_detector.py` | 364 | YOLO-World 检测器：开放词汇检测，125 类导航词汇表，10–15 FPS on Jetson |
| `bpu_detector.py` | 672 | Nash BPU YOLO11s-seg：HB_HBMRuntime API，NV12 输入，~45ms/帧，实例分割（S100P 专用） |
| `yoloe_detector.py` | 223 | YOLOE 检测器：轻量级开放集检测，默认仿真/开发后端 |
| `grounding_dino_detector.py` | 148 | Grounding DINO 检测器：高精度慢速，用于离线标注或 slow path 验证 |

---

## Encoder — 特征编码 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `clip_encoder.py` | 623 | CLIP 特征编码器：HOV-SG encode_three_source (f_g+f_l+f_m，权重 0.25/0.50/0.25)，LRU 缓存 |
| `mobileclip_encoder.py` | 273 | MobileCLIP 编码器：USS-Nav 风格纯文本编码，标签预编码缓存，~0.1ms/帧（torch 可选，无 torch 则 CPU 文本编码） |
| `projection.py` | 241 | 2D→3D 投影：像素坐标 + 深度 → 相机系 → 世界系，TF 变换封装 |

---

## Tracker — 实例跟踪与场景图构建 (4)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `instance_tracker.py` | 1 571 | **场景图核心构建器**：HOV-SG RoomNode view_embeddings (K=10)，DBSCAN 特征精化，TrackedObject 管理，BA-HSG 贝叶斯置信融合 |
| `tracked_objects.py` | 566 | 数据结构：TrackedObject，检测历史，CLIP 特征 FIFO，置信度衰减 |
| `keyframe_selector.py` | 219 | 关键帧选择：运动量 + 场景变化双触发，控制 CLIP 编码频率 |
| `bpu_tracker.py` | 276 | BPU 专用跟踪器：Nash BPU 输出 → TrackedObject，与 `bpu_detector` 配套 |

---

## SceneGraph — 场景图构建与路径规划 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `scg_builder.py` | 516 | 场景图构建器：物体节点 + 关系边 (near/on/in)，增量更新，JSON 序列化 |
| `scg_path_planner.py` | 572 | 场景图路径规划器：基于语义关系的可达性搜索，输出粗粒度 waypoint 序列 |
| `hybrid_planner.py` | 523 | 混合规划器：语义场景图 + 几何 costmap 联合规划，soft constraint 融合（内含直线路径回退，待接入 PCT A*） |

---

## Topology — 拓扑图与房间管理 (4)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `topology_graph.py` | 536 | 拓扑图：ViewNode + RoomNode，跨房间可达性，`query_by_position` / `query_by_label` |
| `topology_types.py` | 460 | 数据类型：ViewNode、RoomNode、TopoEdge，序列化/反序列化 |
| `room_manager.py` | 959 | 房间管理器：房间边界估计，房间-物体归属，动态合并/分裂 |
| `leiden_segmentation.py` | 366 | Leiden 图聚类：将场景图物体节点聚类为语义房间区域 |

---

## Belief — 贝叶斯置信网络 (2)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `belief_network.py` | 652 | 贝叶斯置信网络：物体存在概率推断，时序衰减，跨传感器融合 |
| `belief_propagation.py` | 601 | 置信传播算法：Loopy BP 在场景图上推断语义标签后验概率 |

---

## KnowledgeGraph — 知识图谱 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `knowledge_graph.py` | 568 | 知识图谱：室内场景本体，物体-属性-关系三元组，SPARQL 风格查询 |
| `knowledge_data.py` | 1 944 | 知识图谱数据：预定义物体分类、属性词典、房间-物体先验共现表 |
| `bpu_qp_bridge.py` | 314 | BPU 量化感知桥接：将 BPU 输出概率校准到知识图谱置信度空间 |

---

## Geometry — 几何处理 (4)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `geometry_extractor.py` | 400 | 几何特征提取：点云 PCA 主轴，包围盒，表面法向量 |
| `polyhedron_expansion.py` | 515 | 多面体膨胀：障碍物安全边界生成，用于 hybrid_planner 的碰撞代价 |
| `laplacian_filter.py` | 43 | 拉普拉斯点云滤波：去除激光点云离群噪声 |
| `local_rolling_grid.py` | 340 | 局部滚动栅格：以机器人为中心的滑动窗口占据栅格，供 `scg_path_planner` 使用 |

---

## Coverage — 覆盖与不确定性 (2)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `global_coverage_mask.py` | 354 | 全局覆盖掩码：记录已探索区域，引导 frontier 优先探索未知区域 |
| `uncertainty_model.py` | 464 | 不确定性模型：感知置信度 + 地图不确定性联合估计，输出 VoI 信号 |

---

## API 层（接口抽象）— `api/` (7)

> 为各算法组件定义 ABC 协议接口，解耦上层 Module 与下层实现，便于 mock 测试。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `api/types.py` | 219 | 公共数据类型：`BBox2D`、`Detection`、`TrackedObject`、`SceneGraphData` 等 dataclass |
| `api/exceptions.py` | 70 | 异常定义：`PerceptionAPIError` 及各子类 |
| `api/detector_api.py` | 111 | 检测器抽象接口：`DetectorAPI`（`detect` / `load_model` / `shutdown`）|
| `api/encoder_api.py` | 168 | 编码器抽象接口：`EncoderAPI`（`encode_image` / `encode_text` / `load_model`）|
| `api/tracker_api.py` | 124 | 追踪器抽象接口：`TrackerAPI`（`update` / `get_scene_graph_json` / `clear`）|
| `api/perception_api.py` | 193 | 感知系统顶层抽象接口：`PerceptionAPI`（完整流水线调用契约）|
| `api/factory.py` | 233 | 工厂类：按配置名称实例化 Detector / Encoder / Tracker 具体实现 |

---

## Impl 层（API 适配实现）— `impl/` (4)

> 将现有算法类适配到 `api/` 接口，是连接算法层与接口层的桥梁。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `impl/yolo_world_detector.py` | 328 | `YOLOWorldDetector` → `DetectorAPI` 适配 |
| `impl/clip_encoder.py` | 410 | `CLIPEncoder` → `EncoderAPI` 适配，含图像编码 + 文本编码 |
| `impl/instance_tracker.py` | 315 | `InstanceTracker` → `TrackerAPI` 适配 |
| `impl/perception_impl.py` | 533 | `PerceptionImpl`：整合三个适配器，实现 `PerceptionAPI` 完整流程 |

---

## Storage 层（存储转发）— `storage/` (3)

> 薄层转发，将存储接口统一指向 `memory.storage`，避免在 semantic_perception 包内重复实现持久化逻辑。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `storage/__init__.py` | 2 | re-export `memory.storage.*` |
| `storage/sqlite_store.py` | 2 | re-export `memory.storage.sqlite_store.*` |
| `storage/timeseries_store.py` | 2 | re-export `memory.storage.timeseries_store.*` |

---

## Evaluation — 评测框架 (4)

> 离线 benchmark 工具，不在机器人运行时加载。

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `evaluation_framework.py` | 464 | 评测框架：指标计算 (precision/recall/F1)，多后端对比，结果汇总 |
| `dataset_loader.py` | 479 | 数据集加载器：HM3D / ScanQA / 自定义格式，统一 Episode 接口 |
| `baseline_wrappers.py` | 429 | 基线封装：CLIP-Nav、ObjectNav-v1 等 baseline 的统一调用接口 |
| `end_to_end_evaluation.py` | 405 | 端到端评测：SR / SPL / SoftSPL 计算，NaviMind 论文 Table 2/3 数据来源 |

---

## Visualization — 可视化 (1)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `visualization_tools.py` | 604 | 可视化工具：场景图 RViz marker，检测框投影，点云着色，Perception Viewer MJPEG |

---

## 层级依赖关系

```
core.Module (框架层)
    ↓ 调用
perception_module.py  detector_module.py  encoder_module.py
    ↓ 调用
service.py  (PerceptionService — 纯算法编排)
    ↓ 实例化
api/  (接口抽象)
    ↓ 实现
impl/  (API 适配)
    ↓ 委托
算法层: instance_tracker.py / clip_encoder.py / yolo_world_detector.py / ...
    ↓ 持久化
storage/ → memory.storage (重定向到 memory 包)
```

> **设计原则**: Module 层只依赖 `service.py`，算法层不依赖 `core.Module`，测试可对任意层 mock。
