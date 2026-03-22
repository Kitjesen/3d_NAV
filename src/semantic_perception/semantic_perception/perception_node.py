"""
semantic_perception_node — 语义感知 ROS2 节点 (USS-Nav 风格 15Hz 管线)

管道 (每帧):
  1. 接收 RGB + Depth (message_filters 同步)
  2. Laplacian 模糊过滤 → 跳过模糊帧
  3. YOLO-E 实例分割 → mask + label + score (USS-Nav §IV-C)
  4. Mobile-CLIP 文本编码 → 语义向量 (仅编码标签, 缓存后零开销)
  5. Mask + Depth → 物体点云 (USS-Nav: mask→cloud projection)
  6. 双指标优先级融合 (语义+几何) → 实例匹配
  7. 发布场景图 (JSON over std_msgs/String)

USS-Nav vs 原实现:
  原: YOLO-World bbox → CLIP 图像裁剪编码 → 单点投影 → 质心+标签匹配
  新: YOLO-E mask → Mobile-CLIP 文本编码 → 完整点云 → 双指标融合

性能: 15Hz on Jetson Orin NX (vs 原 5Hz)

参考:
  - USS-Nav (2026): Unified Spatio-Semantic Scene Graph
  - YOLOE (2025): Real-Time Seeing Anything
  - MobileCLIP (CVPR 2024): Fast Image-Text Models
"""

import json
import os
import re
import time
import traceback
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from semantic_common import (
    validate_bgr,
    validate_depth,
    validate_depth_pair,
    validate_intrinsics,
    normalize_quaternion,
)

from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger
try:
    from semantic_perception.srv import QueryScene
except ImportError:
    QueryScene = None

import message_filters
from cv_bridge import CvBridge

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .laplacian_filter import is_blurry
from .clip_encoder import CLIPEncoder
from .mobileclip_encoder import MobileCLIPEncoder
from .projection import (
    CameraIntrinsics,
    Detection3D,
    bbox_center_depth,
    project_to_3d,
    transform_point,
    mask_to_pointcloud,
    pointcloud_centroid,
)
from .instance_tracker import InstanceTracker
from .perception_pipeline import PerceptionPipelineMixin
from .perception_publishers import PerceptionPublishersMixin


class SemanticPerceptionNode(PerceptionPipelineMixin, PerceptionPublishersMixin, Node):
    """語義感知 ROS2 節點 (論文級實現)。"""

    def __init__(self):
        super().__init__("semantic_perception_node")

        # ── 参数声明 ──
        self.declare_parameter("detector_type", "yoloe")  # yoloe | yolo_world | grounding_dino | bpu
        # YOLO-E 分割参数 (USS-Nav 默认)
        self.declare_parameter("yoloe.model_size", "l")
        self.declare_parameter("yoloe.confidence", 0.3)
        self.declare_parameter("yoloe.iou_threshold", 0.5)
        self.declare_parameter("yoloe.tensorrt", False)
        self.declare_parameter("yoloe.max_detections", 30)
        # YOLO-World 参数 (legacy fallback)
        self.declare_parameter("yolo_world.model_size", "l")
        self.declare_parameter("yolo_world.confidence", 0.3)
        self.declare_parameter("yolo_world.iou_threshold", 0.5)
        self.declare_parameter("yolo_world.tensorrt", False)
        # BPU YOLO 参数 (D-Robotics Nash, S100P 专用)
        self.declare_parameter("bpu.model_path", "")
        self.declare_parameter("bpu.confidence", 0.25)
        self.declare_parameter("bpu.iou_threshold", 0.45)
        self.declare_parameter("bpu.max_detections", 30)
        # 语义编码 (USS-Nav: text-only Mobile-CLIP)
        self.declare_parameter("clip.enable", True)
        self.declare_parameter("clip.model", "ViT-B/32")
        self.declare_parameter("clip.text_only", True)  # USS-Nav: 仅文本编码
        # GroundingDINO 参数 (legacy)
        self.declare_parameter("grounding_dino.config_path", "")
        self.declare_parameter("grounding_dino.weights_path", "")
        self.declare_parameter("grounding_dino.box_threshold", 0.35)
        self.declare_parameter("grounding_dino.text_threshold", 0.25)
        self.declare_parameter(
            "default_classes",
            "door . chair . fire extinguisher . person . desk . stairs . elevator . sign",
        )
        self.declare_parameter("laplacian_threshold", 100.0)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("max_depth", 6.0)
        self.declare_parameter("min_depth", 0.3)
        self.declare_parameter("iou_threshold", 0.3)
        self.declare_parameter("clip_similarity_threshold", 0.75)
        self.declare_parameter("merge_distance", 0.5)
        self.declare_parameter("scene_graph.enable", True)
        self.declare_parameter("scene_graph.publish_rate", 1.0)
        self.declare_parameter("semantic_map.enable", True)  # 点云到语义地图渲染
        self.declare_parameter("scene_graph.max_objects", 200)
        self.declare_parameter("target_fps", 15.0)   # USS-Nav: 15Hz on Jetson Orin NX
        self.declare_parameter("skip_frames", 1)      # USS-Nav: 处理每帧
        # TF2 参数 (A1 修复)
        self.declare_parameter("camera_frame", "camera_link")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.1)
        self.declare_parameter("tf_fallback_enable", True)  # 无 TF 时用默认静态变换
        # 实例追踪参数 (C10 参数化)
        self.declare_parameter("tracker.stale_timeout", 300.0)
        self.declare_parameter("tracker.ema_alpha", 0.3)
        self.declare_parameter("tracker.near_threshold", 1.5)
        self.declare_parameter("tracker.on_threshold", 0.3)
        self.declare_parameter("tracker.region_cluster_radius", 3.0)
        # 可配置阈值 (D13)
        self.declare_parameter("adaptive_blur_detection", False)
        # Room LLM 命名 (创新1 补强)
        self.declare_parameter("room_naming.llm_enable", False)
        self.declare_parameter("room_naming.api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("room_naming.model", "gpt-4o-mini")
        self.declare_parameter("room_naming.language", "zh")
        # 指令→检测器文本处理: 动态合并用户指令目标词到检测类别
        self.declare_parameter("instruction_topic", "instruction")
        self.declare_parameter("instruction_merge_enable", True)
        # 语义地图持久化: 非空路径表示启用跨会话物体位置保存/加载
        self.declare_parameter("semantic_map_path", "")

        # ── 读取参数 ──
        self._detector_type = self.get_parameter("detector_type").value
        self._laplacian_threshold = self.get_parameter("laplacian_threshold").value
        self._depth_scale = self.get_parameter("depth_scale").value
        self._max_depth = self.get_parameter("max_depth").value
        self._min_depth = self.get_parameter("min_depth").value
        self._default_classes = self.get_parameter("default_classes").value
        self._target_fps = self.get_parameter("target_fps").value
        self._skip_frames = self.get_parameter("skip_frames").value
        self._sg_enable = self.get_parameter("scene_graph.enable").value
        self._sg_publish_rate = self.get_parameter("scene_graph.publish_rate").value
        self._camera_frame = self.get_parameter("camera_frame").value
        self._world_frame = self.get_parameter("world_frame").value
        self._tf_timeout = self.get_parameter("tf_timeout_sec").value

        # ── 内部状态 ──
        self._bridge = CvBridge()
        self._intrinsics: Optional[CameraIntrinsics] = None
        self._frame_count = 0
        self._last_process_time = 0.0

        # 关键帧选择器 (OpenFly/DualMap 风格, 替代固定 skip_frames)
        try:
            from .keyframe_selector import KeyframeSelector, KeyframeConfig
            self._keyframe_selector = KeyframeSelector(KeyframeConfig(
                min_translation=0.3,
                min_rotation=10.0,
                max_interval=3.0,
                min_interval=1.0 / max(self._target_fps, 1),
                enable_visual=True,
                enable_curvature=True,
            ))
            self._use_keyframe_selector = True
        except Exception:
            self._keyframe_selector = None
            self._use_keyframe_selector = False
        self._min_interval = 1.0 / max(self._target_fps, 0.1)
        self._warned_no_camera_info = False
        self._warned_no_tf = False
        self._tf_fail_count = 0
        self._tf_total_count = 0
        # TF fallback: 无 SLAM 时使用默认静态变换 (map→camera_link)
        self._tf_fallback_enable = self.get_parameter("tf_fallback_enable").value
        self._tf_fallback_matrix = np.eye(4)
        self._tf_fallback_matrix[:3, 3] = [0.15, 0.0, 0.45]  # body(z=0.35) + camera(x=0.15, z=0.1)
        self._tf_fallback_warned = False

        # 指令→检测器: 缓存最新指令和合并后的类别
        self._instruction_merge_enable = self.get_parameter("instruction_merge_enable").value
        self._latest_instruction: str = ""
        self._merged_classes_cache: Optional[str] = None
        self._last_instruction_for_cache: str = ""

        # 实例追踪器
        self._tracker = InstanceTracker(
            merge_distance=self.get_parameter("merge_distance").value,
            iou_threshold=self.get_parameter("iou_threshold").value,
            clip_threshold=self.get_parameter("clip_similarity_threshold").value,
            max_objects=self.get_parameter("scene_graph.max_objects").value,
            stale_timeout=self.get_parameter("tracker.stale_timeout").value,
        )

        # 语义地图持久化 — 启动时加载已保存的物体场景图
        self._semantic_map_path = self.get_parameter("semantic_map_path").value
        if self._semantic_map_path:
            if os.path.exists(self._semantic_map_path):
                if self._tracker.load_from_file(self._semantic_map_path):
                    self.get_logger().info(
                        f"[SemanticMap] Loaded persistent scene graph from {self._semantic_map_path} "
                        f"({len(self._tracker._objects)} objects)"
                    )
                else:
                    self.get_logger().warning(
                        f"[SemanticMap] Failed to load scene graph from {self._semantic_map_path}"
                    )
            else:
                self.get_logger().info(
                    f"[SemanticMap] No existing map at {self._semantic_map_path}, will create on shutdown"
                )

        # 知识图谱注入 (ConceptBot / DovSG: 每个物体创建时查 KG 补属性)
        try:
            from .knowledge_graph import IndustrialKnowledgeGraph
            self._knowledge_graph = IndustrialKnowledgeGraph()
            self._tracker.set_knowledge_graph(self._knowledge_graph)
            kg_stats = self._knowledge_graph.get_stats()
            self.get_logger().info(
                f"KG injected: {kg_stats['total_concepts']} concepts, "
                f"{kg_stats['total_relations']} relations, "
                f"{kg_stats['total_safety_constraints']} safety constraints"
            )

            # 用 KG 词汇表扩展检测器类别 (开放词汇增强)
            kg_vocab = self._knowledge_graph.get_clip_vocabulary()
            if kg_vocab and self._default_classes:
                existing = set(c.strip() for c in self._default_classes.split(".") if c.strip())
                new_terms = [v for v in kg_vocab if v.lower() not in {e.lower() for e in existing}]
                if new_terms:
                    expanded = self._default_classes.rstrip(" .") + " . " + " . ".join(new_terms[:30])
                    self._default_classes = expanded
                    self.get_logger().info(
                        f"Detection classes expanded with {min(len(new_terms), 30)} KG terms "
                        f"(total: {len(expanded.split('.'))})"
                    )
        except Exception as e:
            self._knowledge_graph = None
            self.get_logger().warning(f"KG initialization failed (non-critical): {e}")

        # 场景图快照 (DovSG 动态更新)
        self._prev_scene_snapshot: dict = {}

        # 创新1 补强: Room LLM 命名 (可选)
        if self.get_parameter("room_naming.llm_enable").value:
            api_key_env = self.get_parameter("room_naming.api_key_env").value
            api_key = os.environ.get(api_key_env, "")
            if api_key:
                model = self.get_parameter("room_naming.model").value
                lang = self.get_parameter("room_naming.language").value
                self._tracker.set_room_namer(
                    self._make_room_llm_namer(api_key, model, lang)
                )
                self.get_logger().info("Room LLM naming enabled")
            else:
                self.get_logger().warn(
                    f"Room LLM naming requested but {api_key_env} not set"
                )

        # ── TF2 (A1 修复: 精确 camera→map 变换) ──
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 语义编码器 (USS-Nav: text-only Mobile-CLIP) ──
        self._clip_encoder = None
        self._clip_enabled = self.get_parameter("clip.enable").value
        self._text_only_mode = self.get_parameter("clip.text_only").value
        if self._clip_enabled:
            try:
                if self._text_only_mode:
                    self._clip_encoder = MobileCLIPEncoder(
                        model_name=self.get_parameter("clip.model").value.replace("/", "-"),
                    )
                    self._clip_encoder.load_model()
                    # 预编码默认检测类别
                    default_labels = [c.strip() for c in self._default_classes.split(".") if c.strip()]
                    if default_labels:
                        self._clip_encoder.precompute_labels(default_labels)
                    self.get_logger().info(
                        f"USS-Nav MobileCLIP text encoder loaded (text-only, {len(default_labels)} labels cached)"
                    )
                else:
                    self._clip_encoder = CLIPEncoder(
                        model_name=self.get_parameter("clip.model").value,
                    )
                    self._clip_encoder.load_model()
                    self.get_logger().info("CLIP encoder loaded (legacy image+text mode)")
            except Exception as e:
                self.get_logger().warn(
                    f"Semantic encoder load failed, using string matching only: {e}"
                )
                self._clip_encoder = None

        # ── 检测器 ──
        self._detector = None
        self._init_detector()

        # ── 订阅 ──
        self._sub_camera_info = self.create_subscription(
            CameraInfo, "camera_info",
            self._camera_info_callback,
            qos_profile_sensor_data,
        )

        # RGB + Depth 同步
        self._sub_color = message_filters.Subscriber(
            self, Image, "color_image", qos_profile=qos_profile_sensor_data
        )
        self._sub_depth = message_filters.Subscriber(
            self, Image, "depth_image", qos_profile=qos_profile_sensor_data
        )
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._sub_color, self._sub_depth],
            queue_size=5,
            slop=0.05,
        )
        self._sync.registerCallback(self._rgbd_callback)

        # Costmap (A5: 为 FrontierScorer 提供数据)
        self._latest_costmap: Optional[OccupancyGrid] = None
        self._sub_costmap = self.create_subscription(
            OccupancyGrid, "/nav/costmap",
            self._costmap_callback,
            10,
        )

        # 指令订阅 (开放词汇: 动态合并用户指令目标词到检测类别)
        if self._instruction_merge_enable:
            instruction_topic = self.get_parameter("instruction_topic").value
            self._sub_instruction = self.create_subscription(
                String, instruction_topic,
                self._instruction_callback,
                10,
            )
            self.get_logger().info(
                f"Instruction merge enabled: subscribing to '{instruction_topic}'"
            )

        # ── 发布 ──
        self._pub_detections = self.create_publisher(String, "detections_3d", 10)
        self._pub_scene_graph = self.create_publisher(String, "scene_graph", 10)
        self._pub_scene_diff = self.create_publisher(String, "scene_diff", 10)
        self._semantic_map_enable = self.get_parameter("semantic_map.enable").value
        self._pub_semantic_markers = (
            self.create_publisher(MarkerArray, "semantic_map_markers", 10)
            if self._semantic_map_enable
            else None
        )

        # ── 场景图定时发布 ──
        if self._sg_enable and self._sg_publish_rate > 0:
            period = 1.0 / self._sg_publish_rate
            self._sg_timer = self.create_timer(period, self._publish_scene_graph)

        # ── 查询服务 (需要 srv 编译, 可选) ──
        if QueryScene is not None:
            self._query_srv = self.create_service(
                QueryScene, "/nav/semantic/query", self._query_callback
            )

        self.get_logger().info(
            f"SemanticPerceptionNode started: detector={self._detector_type}, "
            f"TF: {self._camera_frame}→{self._world_frame}, "
            f"classes={self._default_classes}, "
            f"target_fps={self._target_fps}, skip_frames={self._skip_frames}"
        )

    def _rgbd_callback(self, color_msg: Image, depth_msg: Image):
        """RGB-D 同步回调 — 主处理管道。"""
        self._frame_count += 1
        now = time.time()

        # 关键帧选择 (OpenFly/DualMap 风格) 或 固定跳帧 (fallback)
        if self._use_keyframe_selector and self._keyframe_selector is not None:
            # 从 TF 获取机器人位置用于关键帧判定
            robot_x, robot_y, robot_yaw = 0.0, 0.0, 0.0
            try:
                tf_mat = self._lookup_tf_camera_to_world(color_msg.header.stamp)
                if tf_mat is not None:
                    robot_x = float(tf_mat[0, 3])
                    robot_y = float(tf_mat[1, 3])
                    import math
                    robot_yaw = math.atan2(float(tf_mat[1, 0]), float(tf_mat[0, 0]))
            except Exception:
                pass

            # 快速灰度缩略图用于视觉变化检测
            gray_thumb = None
            try:
                raw = np.frombuffer(color_msg.data, dtype=np.uint8)
                if color_msg.encoding in ("rgb8", "bgr8"):
                    img = raw.reshape(color_msg.height, color_msg.width, 3)
                    import cv2 as _cv2
                    gray_thumb = _cv2.cvtColor(
                        _cv2.resize(img, (160, 90)), _cv2.COLOR_BGR2GRAY
                    )
            except Exception:
                pass

            if not self._keyframe_selector.is_keyframe(
                robot_x, robot_y, robot_yaw, gray_thumb, now
            ):
                return

            # 每 100 帧打印一次关键帧统计
            if self._frame_count % 100 == 0:
                stats = self._keyframe_selector.stats
                self.get_logger().info(
                    f"Keyframe stats: {stats['keyframes']}/{stats['total']} "
                    f"({stats['keyframe_rate']}), saved {stats['savings']}"
                )
        else:
            # fallback: 固定跳帧
            if self._frame_count % max(self._skip_frames, 1) != 0:
                return
            if now - self._last_process_time < self._min_interval:
                return

        self._last_process_time = now

        # 前置条件检查
        if self._detector is None:
            return
        if self._intrinsics is None:
            if not self._warned_no_camera_info:
                self.get_logger().warn("Waiting for camera_info...")
                self._warned_no_camera_info = True
            return

        # A1 修复: TF2 查询精确变换
        self._tf_total_count += 1
        tf_camera_to_world = self._lookup_tf_camera_to_world(color_msg.header.stamp)
        if tf_camera_to_world is None:
            self._tf_fail_count += 1
            if self._tf_fallback_enable:
                # 降级模式: 使用默认静态变换, 检测仍然运行
                tf_camera_to_world = self._tf_fallback_matrix
                if not self._tf_fallback_warned:
                    self.get_logger().warn(
                        "TF unavailable — using static fallback transform. "
                        "3D positions approximate until SLAM/TF online."
                    )
                    self._tf_fallback_warned = True
            else:
                if self._tf_fail_count % 50 == 0:
                    fail_rate = self._tf_fail_count / max(self._tf_total_count, 1) * 100
                    self.get_logger().warn(
                        f"TF fail rate: {self._tf_fail_count}/{self._tf_total_count} "
                        f"({fail_rate:.1f}%) — 3D projection degraded"
                    )
                return

        try:
            self._process_frame(color_msg, depth_msg, tf_camera_to_world)
        except Exception as e:
            # A2 修复: 不再静默吞异常, 完整记录日志
            self.get_logger().error(
                f"Frame processing error (frame={self._frame_count}): {e}\n"
                f"{traceback.format_exc()}"
            )

    # ================================================================
    #  查询服务
    # ================================================================

    def _query_callback(self, request, response):
        """
        查询服务: 返回场景图 JSON。

        Args:
            request.query: 空字符串返回完整场景图，否则返回匹配的物体
        """
        try:
            if not request.query:
                # 返回完整场景图
                response.scene_graph = self._tracker.get_scene_graph_json()
            else:
                # 返回匹配的物体
                scene_data = json.loads(self._tracker.get_scene_graph_json())
                matched_objects = []
                query_lower = request.query.lower()

                for obj in scene_data.get("objects", []):
                    if query_lower in obj.get("label", "").lower():
                        matched_objects.append(obj)

                response.scene_graph = json.dumps({
                    "query": request.query,
                    "matched_count": len(matched_objects),
                    "objects": matched_objects
                })

            response.success = True
            response.message = ""
        except Exception as e:
            self.get_logger().error(f"Query service error: {e}")
            response.success = False
            response.scene_graph = ""
            response.message = str(e)
        return response

    # ================================================================
    #  生命周期
    # ================================================================

    def destroy_node(self):
        """清理资源，保存语义地图。"""
        # 语义地图持久化 — 关闭时保存物体场景图
        if getattr(self, "_semantic_map_path", "") and getattr(self, "_tracker", None):
            try:
                os.makedirs(os.path.dirname(os.path.abspath(self._semantic_map_path)), exist_ok=True)
                self._tracker.save_to_file(self._semantic_map_path)
                self.get_logger().info(
                    f"[SemanticMap] Saved {len(self._tracker._objects)} objects to {self._semantic_map_path}"
                )
            except Exception as e:
                self.get_logger().warn(f"[SemanticMap] Failed to save scene graph: {e}")

        if self._detector is not None:
            try:
                self._detector.shutdown()
            except Exception as e:
                self.get_logger().warn(f"Detector shutdown error: {e}")
        if self._clip_encoder is not None:
            try:
                self._clip_encoder.shutdown()
            except Exception as e:
                self.get_logger().warn(f"CLIP shutdown error: {e}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SemanticPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
