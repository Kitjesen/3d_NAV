# Semantic Perception — 模块组织索引

> 文件均位于 `src/semantic_perception/semantic_perception/`
> 共 39 个 .py 文件，19 035 行
> 本文档说明各文件的功能归属，便于导航和维护。

---

## 根节点 (2)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `perception_node.py` | 567 | ROS2 主节点：订阅相机 RGB-D，驱动检测→跟踪→场景图完整流水线，发布 /nav/semantic/scene_graph |
| `perception_publishers.py` | 242 | ROS2 话题发布封装：Detection3DArray、scene_graph JSON、detections_3d 统一出口 |

---

## Detector — 目标检测后端 (5)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `detector_base.py` | 58 | 抽象基类：DetectorBase，定义 detect(image) → List[Detection] 接口 |
| `yolo_world_detector.py` | 364 | YOLO-World 检测器：开放词汇检测，125 类导航词汇表，10–15 FPS on Jetson |
| `bpu_detector.py` | 672 | Nash BPU YOLO11s-seg：HB_HBMRuntime API，NV12 输入，~45ms/帧，实例分割 |
| `yoloe_detector.py` | 223 | YOLOE 检测器：轻量级开放集检测备选后端 |
| `grounding_dino_detector.py` | 148 | Grounding DINO 检测器：高精度但慢速，用于离线标注或 slow path 验证 |

---

## Encoder — 特征编码 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `clip_encoder.py` | 623 | CLIP 特征编码器：HOV-SG encode_three_source (f_g+f_l+f_m, 权重 0.25/0.50/0.25)，LRU 缓存 |
| `mobileclip_encoder.py` | 273 | MobileCLIP 编码器：轻量化 CLIP，适配 Jetson/BPU 低延迟场景 |
| `projection.py` | 241 | 2D→3D 投影：像素坐标 + 深度 → 相机系 → 世界系，TF 变换封装 |

---

## Tracker — 实例跟踪与场景图构建 (4)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `instance_tracker.py` | 1 571 | 场景图核心构建器：HOV-SG RoomNode view_embeddings (K=10)，DBSCAN 特征精化，TrackedObject 管理 |
| `tracked_objects.py` | 566 | 数据结构：TrackedObject，检测历史，CLIP 特征 FIFO，置信度衰减 |
| `keyframe_selector.py` | 219 | 关键帧选择：运动量 + 场景变化双触发，控制 CLIP 编码频率 |
| `bpu_tracker.py` | 276 | BPU 专用跟踪器：Nash BPU 输出 → TrackedObject，与 bpu_detector 配套 |

---

## SceneGraph — 场景图构建与路径规划 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `scg_builder.py` | 516 | 场景图构建器：物体节点 + 关系边 (near/on/in)，增量更新，JSON 序列化 |
| `scg_path_planner.py` | 572 | 场景图路径规划器：基于语义关系的可达性搜索，输出粗粒度 waypoint 序列 |
| `hybrid_planner.py` | 523 | 混合规划器：语义场景图 + 几何 costmap 联合规划，soft constraint 融合 |

---

## Topology — 拓扑图与房间管理 (4)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `topology_graph.py` | 536 | 拓扑图：ViewNode + RoomNode，跨房间可达性，query_by_position / query_by_label |
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
| `local_rolling_grid.py` | 340 | 局部滚动栅格：以机器人为中心的滑动窗口占据栅格，供 scg_path_planner 使用 |

---

## Coverage — 覆盖与不确定性 (3)

| 文件 | 行数 | 职责 |
|------|-----:|------|
| `global_coverage_mask.py` | 354 | 全局覆盖掩码：记录已探索区域，引导 frontier 优先探索未知区域 |
| `uncertainty_model.py` | 464 | 不确定性模型：感知置信度 + 地图不确定性联合估计，输出 VOI 信号 |
| `perception_pipeline.py` | 483 | 感知流水线编排器：检测 → 投影 → 跟踪 → 场景图的完整调用链封装 |

---

## Evaluation — 评测框架 (4)

> 这些文件用于离线 benchmark，不在机器人运行时加载。

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
