"""
perception_pipeline.py — 检测管线 Mixin (USS-Nav 风格)

提供 _process_frame 及其所有辅助方法。由 SemanticPerceptionNode 通过多继承引入。
"""

import json
import math
import time
import traceback
from typing import List, Optional

import numpy as np

import rclpy
import tf2_ros
from rclpy.duration import Duration

from semantic_common import normalize_quaternion

from .projection import (
    Detection3D,
    bbox_center_depth,
    mask_to_pointcloud,
    pointcloud_centroid,
    project_to_3d,
    transform_point,
)
from .mobileclip_encoder import MobileCLIPEncoder


class PerceptionPipelineMixin:
    """
    检测管线 Mixin — _process_frame 及 TF/辅助方法。

    需要宿主类提供以下属性 (均在 __init__ 中设置):
      self._bridge, self._intrinsics, self._detector, self._clip_encoder,
      self._text_only_mode, self._depth_scale, self._min_depth, self._max_depth,
      self._laplacian_threshold, self._tracker, self._knowledge_graph,
      self._frame_count, self._tf_buffer, self._tf_timeout,
      self._camera_frame, self._world_frame, self._tf_fallback_enable,
      self._tf_fallback_matrix, self._tf_fallback_warned,
      self._warned_no_tf, self._tf_fail_count, self._tf_total_count,
      self._pub_detections
    """

    # ================================================================
    #  TF2 变换查询 (A1 修复)
    # ================================================================

    def _lookup_tf_camera_to_world(self, stamp) -> Optional[np.ndarray]:
        """
        使用 TF2 查询 camera_link → map 精确变换矩阵。

        A1 修复: 替换之前用里程计近似 camera pose 的做法。
        TF2 会自动处理 camera_link → body → odom → map 的链路。

        Args:
            stamp: ROS2 Time (帧的时间戳)

        Returns:
            4x4 变换矩阵, 或 None (TF 不可用时)
        """
        try:
            # 非阻塞查询 (关键不变量 P1 #9: TF non-blocking)
            if not self._tf_buffer.can_transform(
                self._world_frame,
                self._camera_frame,
                stamp,
                timeout=Duration(seconds=0.0),
            ):
                # TF 尚未就绪 — 尝试用最新可用变换
                try:
                    transform = self._tf_buffer.lookup_transform(
                        self._world_frame,
                        self._camera_frame,
                        rclpy.time.Time(),  # 最新可用
                        timeout=Duration(seconds=self._tf_timeout),
                    )
                except Exception as e2:
                    if not self._warned_no_tf:
                        self.get_logger().warn(f"TF fallback also failed: {e2}")
                        self._warned_no_tf = True
                    return None
            else:
                transform = self._tf_buffer.lookup_transform(
                    self._world_frame,
                    self._camera_frame,
                    stamp,
                    timeout=Duration(seconds=self._tf_timeout),
                )

            # TransformStamped → 4x4 矩阵
            t = transform.transform.translation
            q = transform.transform.rotation
            quat = normalize_quaternion(q.x, q.y, q.z, q.w)
            if quat is None:
                return None  # Invalid quaternion
            qx, qy, qz, qw = quat
            rot = self._quat_to_rotation(qx, qy, qz, qw)
            tf_mat = np.eye(4)
            tf_mat[:3, :3] = rot
            tf_mat[:3, 3] = [t.x, t.y, t.z]
            return tf_mat

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            if not self._warned_no_tf:
                self.get_logger().warn(
                    f"TF2 lookup {self._camera_frame}→{self._world_frame} failed: {e}. "
                    f"3D projection disabled until TF available."
                )
                self._warned_no_tf = True
            return None

    @staticmethod
    def _quat_to_rotation(x, y, z, w) -> np.ndarray:
        """四元数 → 3x3 旋转矩阵。"""
        return np.array([
            [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
            [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
            [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])

    # ================================================================
    #  核心处理
    # ================================================================

    def _process_frame(
        self,
        color_msg,
        depth_msg,
        tf_camera_to_world: np.ndarray,
    ):
        """
        USS-Nav 风格处理单帧 RGB-D。

        Pipeline: YOLO-E mask → Mobile-CLIP text → mask+depth→点云 → 双指标融合
        """
        from semantic_common import validate_bgr, validate_depth, validate_depth_pair
        from .laplacian_filter import is_blurry
        from std_msgs.msg import String

        # 1. 转换为 numpy
        try:
            bgr = self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        bgr = validate_bgr(bgr, caller="perception")
        if bgr is None:
            return
        depth = validate_depth(depth, caller="perception")
        if depth is None:
            return
        depth = validate_depth_pair(bgr, depth, caller="perception")
        if depth is None:
            return

        # 2. Laplacian 模糊检测
        if is_blurry(bgr, threshold=self._laplacian_threshold):
            return

        # 3. 开放词汇检测 (USS-Nav: YOLO-E 实例分割 → mask + label)
        classes_to_detect = self._merge_detection_classes()
        detections_2d = self._detector.detect(bgr, classes_to_detect)
        if not detections_2d:
            return

        # 3.5 USS-Nav: Mobile-CLIP 文本编码 (仅编码标签, 缓存后零开销)
        if self._clip_encoder is not None and self._text_only_mode:
            try:
                unique_labels = list({d.label for d in detections_2d})
                if isinstance(self._clip_encoder, MobileCLIPEncoder):
                    self._clip_encoder.precompute_labels(unique_labels)
                    for det in detections_2d:
                        det.features = self._clip_encoder.encode_label(det.label)
                else:
                    label_feats = self._clip_encoder.encode_text(unique_labels)
                    label_map = {l: f for l, f in zip(unique_labels, label_feats)}
                    for det in detections_2d:
                        det.features = label_map.get(det.label, np.array([]))
            except Exception as e:
                self.get_logger().warn(f"Text encoding failed: {e}")
        elif self._clip_encoder is not None:
            # Legacy: 图像裁剪编码 (慢, 仅 YOLO-World fallback)
            try:
                bboxes = [d.bbox for d in detections_2d]
                clip_features = self._clip_encoder.encode_image_crops(bgr, bboxes)
                for det, feat in zip(detections_2d, clip_features):
                    det.features = feat
            except Exception as e:
                self.get_logger().warn(f"CLIP encoding failed: {e}")

        # 4. USS-Nav: mask + depth → 物体点云 + 质心投影
        detections_3d = self._run_detection_3d(detections_2d, depth, tf_camera_to_world)

        if not detections_3d:
            return

        # 5. USS-Nav 双指标融合实例追踪
        # 传入相机位姿用于 FOV 检查 (OneMap 理念: 只对视野内物体记录负面证据)
        cam_pos = tf_camera_to_world[:3, 3] if tf_camera_to_world is not None else None
        cam_fwd = tf_camera_to_world[:3, 2] if tf_camera_to_world is not None else None
        cam_fx = self._intrinsics.fx if self._intrinsics is not None else 0.0
        tracked_objs = self._tracker.update(
            detections_3d,
            camera_pos=cam_pos,
            camera_forward=cam_fwd,
            intrinsics_fx=cam_fx,
        )

        # 5.5 开放词汇: 未知物体 → KG 概念映射 (DovSG / LOVON)
        if self._knowledge_graph is not None:
            for obj in tracked_objs:
                if not obj.kg_concept_id and obj.detection_count <= 2:
                    mapped = self._knowledge_graph.map_unknown_to_concept(
                        obj.label,
                        clip_embedding=obj.features if obj.features.size > 0 else None,
                        clip_encoder=self._clip_encoder,
                    )
                    if mapped is not None:
                        props = self._knowledge_graph.enrich_object_properties(mapped.concept_id)
                        obj.kg_concept_id = props.get("concept_id", "")
                        obj.safety_level = props.get("safety_level", "safe")
                        obj.affordances = props.get("affordances", [])
                        obj.functional_properties = props

        # 5.6 记录关键视角 (view 节点)
        try:
            camera_pos = tf_camera_to_world[:3, 3]
            observed_ids = [o.object_id for o in tracked_objs]
            self._tracker.record_view(camera_pos, observed_ids)
        except Exception as e:
            self.get_logger().debug(f"record_view skipped: {e}")

        # 6. 发布本帧检测 (JSON)
        det_json = json.dumps({
            "timestamp": time.time(),
            "frame_id": self._frame_count,
            "detections": [
                {
                    "label": d.label,
                    "score": round(d.score, 3),
                    "position": {
                        "x": round(float(d.position[0]), 3),
                        "y": round(float(d.position[1]), 3),
                        "z": round(float(d.position[2]), 3),
                    },
                    "depth": round(d.depth, 3),
                }
                for d in detections_3d
            ],
        }, ensure_ascii=False)

        msg = String()
        msg.data = det_json
        self._pub_detections.publish(msg)

    # ================================================================
    #  Callbacks & 指令合并
    # ================================================================

    def _camera_info_callback(self, msg):
        """接收相机内参 (只需一次, 后续消息忽略 — 内参不变)。"""
        from semantic_common import validate_intrinsics
        from .projection import CameraIntrinsics
        if self._intrinsics is not None:
            return
        result = validate_intrinsics(msg.k, msg.width, msg.height, caller="perception")
        if result is None:
            return
        self._intrinsics = CameraIntrinsics(
            fx=result.fx, fy=result.fy,
            cx=result.cx, cy=result.cy,
            width=result.width, height=result.height,
        )
        self.get_logger().info(
            f"Camera intrinsics received: fx={self._intrinsics.fx:.1f}, "
            f"fy={self._intrinsics.fy:.1f}, "
            f"{self._intrinsics.width}x{self._intrinsics.height}"
        )

    def _costmap_callback(self, msg):
        """缓存最新 costmap (A5: 为 FrontierScorer 提供数据)。"""
        self._latest_costmap = msg

    def _instruction_callback(self, msg):
        """
        缓存最新用户指令 (开放词汇: 用于动态合并检测类别)。

        Planner 发布 JSON: {"instruction": "...", "language": "zh", ...}
        或纯文本。
        """
        import json
        try:
            data = json.loads(msg.data)
            inst = data.get("instruction", msg.data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            inst = msg.data if msg.data else ""
        if isinstance(inst, str) and inst.strip():
            self._latest_instruction = inst.strip()
            self._merged_classes_cache = None  # 失效缓存, 下次合并时重新计算

    def _extract_instruction_keywords(self, instruction: str) -> list:
        """
        从指令中提取目标词 (用于合并到检测类别)。

        轻量实现, 不依赖 semantic_planner, 支持中英文。
        """
        import re
        if not instruction or not instruction.strip():
            return []
        stop_words = {
            "the", "a", "an", "to", "go", "find", "get", "me", "for", "and", "or",
            "is", "at", "in", "on", "near", "next", "by", "of", "with", "from",
            "去", "到", "找", "拿", "的", "在", "旁边", "附近", "那个",
            "请", "帮", "我", "一个", "把", "了", "着", "过", "that", "this",
        }
        inst_lower = instruction.lower().strip()
        en_tokens = re.findall(r"[a-zA-Z]+", inst_lower)
        zh_tokens = re.findall(r"[\u4e00-\u9fff]{2,6}", instruction)
        keywords: list = []
        for w in en_tokens:
            if w not in stop_words and len(w) > 1:
                keywords.append(w)
        for w in zh_tokens:
            if w not in stop_words:
                keywords.append(w)
        seen: set = set()
        unique: list = []
        for k in keywords:
            k_lower = k.lower()
            if k_lower not in seen:
                seen.add(k_lower)
                unique.append(k)
        return unique

    def _merge_detection_classes(self) -> str:
        """
        合并 default_classes + 指令中的目标词。

        有缓存: 指令未变时复用, 避免每帧重复解析。
        """
        if not self._instruction_merge_enable or not self._latest_instruction:
            return self._default_classes
        if (
            self._merged_classes_cache is not None
            and self._last_instruction_for_cache == self._latest_instruction
        ):
            return self._merged_classes_cache
        default_set = {
            c.strip().lower()
            for c in self._default_classes.split(".")
            if c.strip()
        }
        keywords = self._extract_instruction_keywords(self._latest_instruction)
        for kw in keywords:
            if kw.lower() not in default_set:
                default_set.add(kw.lower())
        default_list = [c.strip() for c in self._default_classes.split(".") if c.strip()]
        for kw in keywords:
            if kw.lower() not in {c.lower() for c in default_list}:
                default_list.append(kw)
        merged = " . ".join(default_list)
        self._merged_classes_cache = merged
        self._last_instruction_for_cache = self._latest_instruction
        return merged

    # ================================================================
    #  初始化
    # ================================================================

    def _init_detector(self):
        """初始化检测器后端 (USS-Nav: 默认 YOLO-E 分割)。"""
        import traceback
        try:
            if self._detector_type == "yoloe":
                from .yoloe_detector import YOLOEDetector
                self._detector = YOLOEDetector(
                    model_size=self.get_parameter("yoloe.model_size").value,
                    confidence=self.get_parameter("yoloe.confidence").value,
                    iou_threshold=self.get_parameter("yoloe.iou_threshold").value,
                    tensorrt=self.get_parameter("yoloe.tensorrt").value,
                    max_detections=self.get_parameter("yoloe.max_detections").value,
                )
            elif self._detector_type == "yolo_world":
                from .yolo_world_detector import YOLOWorldDetector
                self._detector = YOLOWorldDetector(
                    model_size=self.get_parameter("yolo_world.model_size").value,
                    confidence=self.get_parameter("yolo_world.confidence").value,
                    iou_threshold=self.get_parameter("yolo_world.iou_threshold").value,
                    tensorrt=self.get_parameter("yolo_world.tensorrt").value,
                )
            elif self._detector_type == "bpu":
                from .bpu_detector import BPUDetector as BPUDet
                model_path = self.get_parameter("bpu.model_path").value
                self._detector = BPUDet(
                    confidence=self.get_parameter("bpu.confidence").value,
                    iou_threshold=self.get_parameter("bpu.iou_threshold").value,
                    model_path=model_path or None,
                    max_detections=self.get_parameter("bpu.max_detections").value,
                )
            elif self._detector_type == "grounding_dino":
                from .grounding_dino_detector import GroundingDINODetector
                config_path = self.get_parameter("grounding_dino.config_path").value
                weights_path = self.get_parameter("grounding_dino.weights_path").value
                self._detector = GroundingDINODetector(
                    config_path=config_path or None,
                    weights_path=weights_path or None,
                    box_threshold=self.get_parameter("grounding_dino.box_threshold").value,
                    text_threshold=self.get_parameter("grounding_dino.text_threshold").value,
                )
            else:
                self.get_logger().error(f"Unknown detector_type: {self._detector_type}")
                return

            self._detector.load_model()
            self.get_logger().info(f"Detector '{self._detector_type}' loaded")
        except Exception as e:
            self.get_logger().error(
                f"Failed to load detector '{self._detector_type}': {e}\n"
                f"{traceback.format_exc()}"
            )
            self._detector = None

    def _run_detection_3d(
        self,
        detections_2d,
        depth: np.ndarray,
        tf_camera_to_world: np.ndarray,
    ) -> list:
        """
        USS-Nav: 对每个 2D 检测做 mask+depth → 3D 点云 + 质心投影。

        Returns:
            List[Detection3D]
        """
        detections_3d = []
        for det2d in detections_2d:
            # 4a. 尝试 mask→点云 (USS-Nav 主路径)
            points = None
            centroid = None
            center_depth = None

            if det2d.mask is not None:
                points = mask_to_pointcloud(
                    mask=det2d.mask,
                    depth_image=depth,
                    intrinsics=self._intrinsics,
                    tf_camera_to_world=tf_camera_to_world,
                    depth_scale=self._depth_scale,
                    min_depth=self._min_depth,
                    max_depth=self._max_depth,
                )
                if points is not None and len(points) > 0:
                    centroid = pointcloud_centroid(points)
                    center_depth = float(np.linalg.norm(centroid - tf_camera_to_world[:3, 3]))

            # 4b. Fallback: bbox 中心深度投影 (无 mask 时)
            if centroid is None:
                d = bbox_center_depth(depth, det2d.bbox, depth_scale=self._depth_scale)
                if d is None or d < self._min_depth or d > self._max_depth:
                    continue
                cx = (det2d.bbox[0] + det2d.bbox[2]) / 2
                cy = (det2d.bbox[1] + det2d.bbox[3]) / 2
                p_camera = project_to_3d(cx, cy, d, self._intrinsics)
                centroid = transform_point(p_camera, tf_camera_to_world)
                center_depth = d

            detections_3d.append(Detection3D(
                position=centroid,
                label=det2d.label,
                score=det2d.score,
                bbox_2d=det2d.bbox,
                depth=center_depth,
                features=getattr(det2d, 'features', np.array([])),
                points=points if points is not None else np.empty((0, 3)),
            ))

        return detections_3d
