"""perception_module.py -- semantic perception Module wrapper (core framework).

Wraps the PURE ALGORITHM parts of SemanticPerceptionNode as a Module,
fully decoupled from ROS2.  No algorithm is rewritten -- all real work
is delegated to lazy imports from the semantic_perception package.

Port contract:
  In:  color_image, depth_image, camera_info, odometry
  Out: scene_graph, detections_3d

Lifecycle: __init__ -> setup() -> start() -> [color_image drives pipeline] -> stop()
"""

from __future__ import annotations

import json
import logging
import time
from typing import Any, Dict, List, Optional

import numpy as np

from core import Module, In, Out
from core.msgs.sensor import CameraIntrinsics, Image, ImageFormat
from core.msgs.semantic import (
    Detection3D as CoreDetection3D,
    Relation,
    Region,
    SceneGraph,
)
from core.msgs.nav import Odometry
from core.msgs.geometry import Vector3
from core.config import get_config

logger = logging.getLogger(__name__)


class PerceptionModule(Module, layer=3):
    """Semantic perception: YOLO detection + CLIP encoding + scene graph tracking.

    Thin Module shell around existing algorithms in semantic_perception.
    All heavy lifting is done by InstanceTracker, DetectorBase subclasses,
    CLIPEncoder / MobileCLIPEncoder, and the projection module.
    """

    # -- input ports --
    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraIntrinsics]
    odometry: In[Odometry]

    # -- output ports --
    scene_graph: Out[SceneGraph]
    detections_3d: Out[list]  # List[CoreDetection3D]

    # Backends that api/factory.py's PerceptionFactory supports natively.
    # All others (yoloe, bpu, sim_scene) fall back to the direct-import path.
    _FACTORY_DETECTORS = {"yolo_world"}
    _FACTORY_ENCODERS = {"clip"}

    def __init__(
        self,
        detector_type: str = "yoloe",
        encoder_type: str = "mobileclip",
        merge_distance: float = 0.5,
        confidence_threshold: float = 0.3,
        max_depth: float = 6.0,
        min_depth: float = 0.3,
        depth_scale: float = 0.001,
        laplacian_threshold: float = 100.0,
        max_objects: int = 200,
        default_classes: str = "door . chair . person . desk . stairs . elevator . sign",
        skip_frames: int = 1,
        world: str = "",
        **kw: Any,
    ) -> None:
        super().__init__(**kw)
        self._detector_type = detector_type
        self._encoder_type = encoder_type
        self._merge_distance = merge_distance
        self._confidence_threshold = confidence_threshold
        self._max_depth = max_depth
        self._min_depth = min_depth
        self._depth_scale = depth_scale
        self._laplacian_threshold = laplacian_threshold
        self._max_objects = max_objects
        self._default_classes = default_classes
        self._skip_frames = max(skip_frames, 1)
        self._world = world

        # Camera extrinsics from factory calibration
        cam_cfg = get_config().camera
        self._T_body_camera = cam_cfg.T_body_camera

        # runtime state (populated during setup)
        self._tracker = None
        self._detector = None
        self._clip_encoder = None
        self._sim_scene_observer = None
        self._frame_count: int = 0
        self._latest_depth: Optional[np.ndarray] = None
        self._latest_intrinsics: Optional[Any] = None
        self._latest_odom_matrix: Optional[np.ndarray] = None
        self._latest_core_detections: List[CoreDetection3D] = []

    # == Lifecycle =============================================================

    def setup(self) -> None:
        """Lazy-import real algorithms, register port subscriptions.

        Components are created through two paths depending on detector_type:
          - Factory path  (yolo_world / clip):  api/factory.py → impl/ adapters
          - Direct path   (yoloe / bpu / sim_scene / mobileclip): legacy lazy imports

        Both paths produce objects with the same duck-typed interface (detect /
        load_model / shutdown for detectors; encode_text / load_model / shutdown
        for encoders), so the rest of the module is unchanged.
        """
        if self._detector_type in self._FACTORY_DETECTORS:
            self._setup_via_factory()
        else:
            self._setup_direct()

        # Wire port callbacks
        self.color_image.subscribe(self._on_color_frame)
        self.depth_image.subscribe(self._on_depth)
        self.camera_info.subscribe(self._on_camera_info)
        self.odometry.subscribe(self._on_odometry)

        # Sanity-check factory calibration
        self._check_calibration()

    def _check_calibration(self) -> None:
        """Warn at startup if camera calibration looks wrong."""
        cfg = get_config().camera
        # Camera position sanity: should be within reasonable robot body bounds
        pos = (cfg.position_x, cfg.position_y, cfg.position_z)
        if all(v == 0.0 for v in pos):
            logger.warning(
                "Camera extrinsics position is all zeros — 3D projections will be wrong. "
                "Check config/robot_config.yaml camera section."
            )
        if abs(cfg.position_x) > 2.0 or abs(cfg.position_y) > 2.0 or abs(cfg.position_z) > 3.0:
            logger.warning(
                "Camera position (%.2f, %.2f, %.2f) looks too large for a quadruped. "
                "Check config/robot_config.yaml camera section.",
                cfg.position_x, cfg.position_y, cfg.position_z,
            )
        # Default intrinsics check
        if cfg.fx <= 0 or cfg.fy <= 0:
            logger.warning(
                "Camera default intrinsics have non-positive focal length (fx=%.1f, fy=%.1f). "
                "Runtime CameraInfo must provide valid values.",
                cfg.fx, cfg.fy,
            )
        # LiDAR sanity
        lidar_cfg = get_config().lidar
        offset_mag = (lidar_cfg.offset_x**2 + lidar_cfg.offset_y**2 + lidar_cfg.offset_z**2) ** 0.5
        if offset_mag > 1.0:
            logger.warning(
                "LiDAR offset magnitude %.3fm seems large for body-mounted sensor. "
                "Check config/robot_config.yaml lidar section.",
                offset_mag,
            )
        logger.info(
            "Calibration OK: camera at (%.3f, %.3f, %.3f), LiDAR offset %.4fm",
            cfg.position_x, cfg.position_y, cfg.position_z, offset_mag,
        )

    def _setup_via_factory(self) -> None:
        """Create detector + encoder + tracker through api/factory.py."""
        try:
            from semantic.perception.semantic_perception.api.factory import PerceptionFactory
            from semantic.perception.semantic_perception.api.types import PerceptionConfig

            cfg = PerceptionConfig(
                detector_type=self._detector_type,
                encoder_type=self._encoder_type if self._encoder_type in self._FACTORY_ENCODERS else "clip",
                confidence_threshold=self._confidence_threshold,
                iou_threshold=self._confidence_threshold,
                merge_distance=self._merge_distance,
                max_depth=self._max_depth,
                min_depth=self._min_depth,
            )

            # Tracker via factory (always "instance")
            self._tracker = PerceptionFactory.create_tracker("instance", cfg)
            logger.info("InstanceTracker created via factory")

            # Detector via factory
            self._detector = PerceptionFactory.create_detector(self._detector_type, cfg)
            logger.info("Detector %r created via factory", self._detector_type)

            # Encoder via factory (only "clip" supported; fall back to mobileclip otherwise)
            enc_type = self._encoder_type if self._encoder_type in self._FACTORY_ENCODERS else "clip"
            self._clip_encoder = PerceptionFactory.create_encoder(enc_type, cfg)
            logger.info("Encoder %r created via factory", enc_type)

        except Exception as e:
            logger.warning(
                "Factory setup failed (%s) — falling back to direct import path", e
            )
            self._setup_direct()

    def _setup_direct(self) -> None:
        """Create components via direct lazy imports (original path)."""
        # Instance tracker
        try:
            from semantic.perception.semantic_perception.instance_tracker import InstanceTracker
            self._tracker = InstanceTracker(
                merge_distance=self._merge_distance,
                iou_threshold=self._confidence_threshold,
                max_objects=self._max_objects,
            )
            logger.info("InstanceTracker initialized (merge_dist=%.2f)", self._merge_distance)
        except ImportError:
            logger.warning(
                "semantic_perception.instance_tracker not available -- "
                "scene graph tracking disabled"
            )

        # Detector backend (optional -- graceful degrade)
        self._detector = self._init_detector()

        # Encoder (optional)
        self._clip_encoder = self._init_clip_encoder()

    def stop(self) -> None:
        """Release GPU resources."""
        for attr, name in [("_detector", "Detector"), ("_clip_encoder", "Encoder")]:
            obj = getattr(self, attr, None)
            if obj is None:
                continue
            for method in ("shutdown", "close", "reset"):
                fn = getattr(obj, method, None)
                if callable(fn):
                    try:
                        fn()
                    except Exception as e:
                        logger.warning("%s %s() error: %s", name, method, e)
                    break
        super().stop()

    # == Port callbacks ========================================================

    def _on_depth(self, img: Image) -> None:
        """Cache latest depth frame."""
        self._latest_depth = img.data

    def _on_camera_info(self, intrinsics: CameraIntrinsics) -> None:
        """Cache intrinsics (one-shot, convert to projection.CameraIntrinsics)."""
        if self._latest_intrinsics is not None:
            return
        try:
            from semantic.perception.semantic_perception.projection import (
                CameraIntrinsics as ProjIntrinsics,
            )
            self._latest_intrinsics = ProjIntrinsics(
                fx=intrinsics.fx, fy=intrinsics.fy,
                cx=intrinsics.cx, cy=intrinsics.cy,
                width=intrinsics.width, height=intrinsics.height,
            )
        except ImportError:
            self._latest_intrinsics = intrinsics
        logger.info(
            "Camera intrinsics received: fx=%.1f, fy=%.1f, %dx%d",
            intrinsics.fx, intrinsics.fy, intrinsics.width, intrinsics.height,
        )

    def _on_odometry(self, odom: Odometry) -> None:
        """Build 4x4 transform matrix from odometry."""
        pos = odom.pose.position
        q = odom.pose.orientation
        rot = _quat_to_rotation(q.x, q.y, q.z, q.w)
        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = [pos.x, pos.y, pos.z]
        self._latest_odom_matrix = mat

    def _on_color_frame(self, img: Image) -> None:
        """Main callback -- each RGB frame triggers the full pipeline."""
        self._frame_count += 1
        if self._frame_count % self._skip_frames != 0:
            return
        if self._latest_intrinsics is None or self._latest_depth is None:
            return

        tf_body_world = self._latest_odom_matrix
        if tf_body_world is None:
            # Static fallback when no SLAM/TF — use camera extrinsics as world pose
            tf_camera_world = self._T_body_camera.copy()
        else:
            # Correct transform chain: T_camera_world = T_body_world @ T_body_camera
            tf_camera_world = tf_body_world @ self._T_body_camera

        try:
            self._process_frame(img, tf_camera_world)
        except Exception as e:
            logger.error("Frame processing error (frame=%d): %s", self._frame_count, e)

    # == Core pipeline =========================================================

    def _process_frame(self, color_img: Image, tf_camera_to_world: np.ndarray) -> None:
        """USS-Nav style single-frame processing: detect -> project -> track -> publish."""
        bgr = color_img.to_bgr().data if hasattr(color_img, "to_bgr") else color_img.data
        depth = self._latest_depth

        # Laplacian blur detection is useful for real RGB streams, but the sim-scene
        # backend derives detections from world metadata and should not be blocked by image sharpness.
        if self._sim_scene_observer is None:
            try:
                from semantic.perception.semantic_perception.laplacian_filter import is_blurry
                if is_blurry(bgr, threshold=self._laplacian_threshold):
                    return
            except ImportError:
                pass

        detections_3d = []
        if self._sim_scene_observer is not None:
            detections_3d = self._sim_scene_observer.observe(
                tf_camera_to_world=tf_camera_to_world,
                intrinsics=self._latest_intrinsics,
                text_prompt=self._default_classes,
            )
        else:
            detections_2d = self._run_detector(bgr)
            if not detections_2d:
                return
            detections_3d = self._project_to_3d(detections_2d, depth, tf_camera_to_world)
        if not detections_3d:
            return

        # Instance tracking
        if self._tracker is not None:
            cam_pos = tf_camera_to_world[:3, 3]
            cam_fwd = tf_camera_to_world[:3, 2]
            fx = getattr(self._latest_intrinsics, "fx", 0.0)
            self._tracker.update(
                detections_3d,
                camera_pos=cam_pos,
                camera_forward=cam_fwd,
                intrinsics_fx=fx,
            )

        # Publish detections
        core_dets = self._convert_detections(detections_3d)
        self._latest_core_detections = core_dets
        self.detections_3d.publish(core_dets)

        # Publish scene graph
        sg = self._build_scene_graph()
        self.scene_graph.publish(sg)

    # == Detector init =========================================================

    def _init_detector(self):
        """Lazy-import detector backend."""
        try:
            if self._detector_type == "yoloe":
                from semantic.perception.semantic_perception.yoloe_detector import YOLOEDetector
                det = YOLOEDetector(confidence=self._confidence_threshold)
                det.load_model()
                logger.info("YOLOEDetector loaded")
                return det
            elif self._detector_type == "yolo_world":
                from semantic.perception.semantic_perception.yolo_world_detector import YOLOWorldDetector
                det = YOLOWorldDetector(confidence=self._confidence_threshold)
                det.load_model()
                logger.info("YOLOWorldDetector loaded")
                return det
            elif self._detector_type == "sim_scene":
                from semantic.perception.semantic_perception.sim_scene_observer import SimSceneObserver
                self._sim_scene_observer = SimSceneObserver(world=self._world)
                logger.info("SimSceneObserver loaded (world=%s)", self._world or "")
                return self._sim_scene_observer
        except (ImportError, Exception) as e:
            logger.warning("Detector %r unavailable: %s", self._detector_type, e)
        return None

    def _init_clip_encoder(self):
        """Lazy-import encoder backend based on encoder_type."""
        try:
            if self._encoder_type == "clip":
                from semantic.perception.semantic_perception.clip_encoder import CLIPEncoder
                enc = CLIPEncoder()
                enc.load_model()
                logger.info("CLIPEncoder loaded")
                return enc
            else:
                # Default: MobileCLIP (USS-Nav style text-only, fast)
                from semantic.perception.semantic_perception.mobileclip_encoder import MobileCLIPEncoder
                enc = MobileCLIPEncoder()
                enc.load_model()
                logger.info("MobileCLIPEncoder loaded")
                return enc
        except (ImportError, Exception) as e:
            logger.warning("Encoder %r unavailable: %s", self._encoder_type, e)
        return None

    def _run_detector(self, bgr: np.ndarray) -> list:
        """Run detector, return Detection2D list."""
        if self._detector is None:
            return []
        try:
            return self._detector.detect(bgr, self._default_classes)
        except Exception as e:
            logger.warning("Detection failed: %s", e)
            return []

    # == 3D Projection =========================================================

    def _project_to_3d(
        self, detections_2d: list, depth: np.ndarray, tf_camera_to_world: np.ndarray,
    ) -> list:
        """Project 2D detections to 3D (delegates to projection module)."""
        try:
            from semantic.perception.semantic_perception.projection import (
                Detection3D as ProjDetection3D,
                bbox_center_depth,
                mask_to_pointcloud,
                pointcloud_centroid,
                project_to_3d,
                transform_point,
            )
        except ImportError:
            return self._project_to_3d_fallback(detections_2d, depth, tf_camera_to_world)

        intrinsics = self._latest_intrinsics
        results = []
        for det2d in detections_2d:
            centroid = None
            center_depth = None
            points = None

            # Try mask -> pointcloud (USS-Nav main path)
            if getattr(det2d, "mask", None) is not None:
                points = mask_to_pointcloud(
                    mask=det2d.mask, depth_image=depth,
                    intrinsics=intrinsics, tf_camera_to_world=tf_camera_to_world,
                    depth_scale=self._depth_scale,
                    min_depth=self._min_depth, max_depth=self._max_depth,
                )
                if points is not None and len(points) > 0:
                    centroid = pointcloud_centroid(points)
                    center_depth = float(
                        np.linalg.norm(centroid - tf_camera_to_world[:3, 3])
                    )

            # Fallback: bbox center depth
            if centroid is None:
                d = bbox_center_depth(depth, det2d.bbox, depth_scale=self._depth_scale)
                if d is None or d < self._min_depth or d > self._max_depth:
                    continue
                cx = (det2d.bbox[0] + det2d.bbox[2]) / 2
                cy = (det2d.bbox[1] + det2d.bbox[3]) / 2
                p_camera = project_to_3d(cx, cy, d, intrinsics)
                centroid = transform_point(p_camera, tf_camera_to_world)
                center_depth = d

            results.append(ProjDetection3D(
                position=centroid, label=det2d.label, score=det2d.score,
                bbox_2d=det2d.bbox, depth=center_depth,
                features=getattr(det2d, "features", np.array([])),
                points=points if points is not None else np.empty((0, 3)),
            ))
        return results

    def _project_to_3d_fallback(
        self, detections_2d: list, depth: np.ndarray, tf_camera_to_world: np.ndarray,
    ) -> list:
        """Simplified 3D projection when projection module is unavailable (Windows/tests)."""
        results = []
        intr = self._latest_intrinsics
        if intr is None:
            return results

        fx = getattr(intr, "fx", 600.0) or 600.0
        fy = getattr(intr, "fy", 600.0) or 600.0
        ccx = getattr(intr, "cx", 320.0)
        ccy = getattr(intr, "cy", 240.0)

        for det2d in detections_2d:
            bbox = det2d.bbox
            px = (bbox[0] + bbox[2]) / 2
            py = (bbox[1] + bbox[3]) / 2
            iy, ix = int(py), int(px)
            h, w = depth.shape[:2]
            if not (0 <= iy < h and 0 <= ix < w):
                continue
            d = float(depth[iy, ix]) * self._depth_scale
            if d < self._min_depth or d > self._max_depth:
                continue
            p_cam = np.array([(px - ccx) * d / fx, (py - ccy) * d / fy, d])
            p_world = (tf_camera_to_world @ np.array([*p_cam, 1.0]))[:3]

            class _FallbackDet3D:
                """Minimal duck-typed Detection3D for fallback path."""
                pass

            det3d = _FallbackDet3D()
            det3d.position = p_world
            det3d.label = det2d.label
            det3d.score = det2d.score
            det3d.bbox_2d = bbox
            det3d.depth = d
            det3d.features = getattr(det2d, "features", np.array([]))
            det3d.points = np.empty((0, 3))
            results.append(det3d)
        return results

    # == Format conversion =====================================================

    def _convert_detections(self, detections_3d: list) -> List[CoreDetection3D]:
        """projection.Detection3D -> core.msgs.Detection3D."""
        results = []
        for d in detections_3d:
            pos = d.position
            feat = getattr(d, "features", None)
            has_feat = feat is not None and hasattr(feat, "size") and feat.size > 0
            results.append(CoreDetection3D(
                label=d.label,
                confidence=d.score,
                position=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
                bbox_2d=[float(x) for x in d.bbox_2d],
                clip_feature=feat if has_feat else None,
            ))
        return results

    def _build_scene_graph(self) -> SceneGraph:
        """Build core SceneGraph message from InstanceTracker state."""
        if self._tracker is None:
            return self._build_detection_scene_graph()
        try:
            sg_json = self._tracker.get_scene_graph_json()
            data = json.loads(sg_json)
        except Exception:
            return self._build_detection_scene_graph()

        objects = []
        for obj in data.get("objects", []):
            pos = obj.get("position", [0, 0, 0])
            if isinstance(pos, dict):
                px, py, pz = float(pos.get("x", 0)), float(pos.get("y", 0)), float(pos.get("z", 0))
            elif isinstance(pos, (list, tuple)) and len(pos) >= 3:
                px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
            else:
                px, py, pz = 0.0, 0.0, 0.0
            label = str(obj.get("label", ""))
            matched_det = self._match_detection_metadata(label, px, py, pz)
            bbox_2d = []
            clip_feature = None
            if matched_det is not None:
                bbox_2d = [float(x) for x in matched_det.bbox_2d]
                if matched_det.clip_feature is not None:
                    clip_feature = np.array(matched_det.clip_feature, copy=True)
            objects.append(CoreDetection3D(
                id=str(obj.get("id", "")),
                label=label,
                confidence=float(obj.get("confidence", obj.get("score", 0))),
                position=Vector3(px, py, pz),
                bbox_2d=bbox_2d,
                clip_feature=clip_feature,
            ))

        relations = []
        for rel in data.get("relations", []):
            relations.append(Relation(
                subject_id=str(rel.get("subject_id", rel.get("subject", ""))),
                predicate=str(rel.get("predicate", rel.get("relation", ""))),
                object_id=str(rel.get("object_id", rel.get("object", ""))),
            ))

        regions = []
        for reg in data.get("rooms", data.get("regions", [])):
            regions.append(Region(
                name=str(reg.get("name", reg.get("room_type", ""))),
                object_ids=[str(oid) for oid in reg.get("object_ids", [])],
            ))

        return SceneGraph(
            objects=objects, relations=relations, regions=regions, frame_id="map",
        )

    def _build_detection_scene_graph(self) -> SceneGraph:
        """Fallback scene graph when tracker state is unavailable."""
        return SceneGraph(
            objects=[self._clone_core_detection(det) for det in self._latest_core_detections],
            frame_id="map",
        )

    def _match_detection_metadata(
        self, label: str, px: float, py: float, pz: float,
    ) -> Optional[CoreDetection3D]:
        best_det: Optional[CoreDetection3D] = None
        best_dist = 1.5
        label_lower = label.lower()

        for det in self._latest_core_detections:
            if label_lower and det.label.lower() != label_lower:
                continue
            dx = det.position.x - px
            dy = det.position.y - py
            dz = det.position.z - pz
            dist = float(np.sqrt(dx * dx + dy * dy + dz * dz))
            if dist < best_dist:
                best_dist = dist
                best_det = det

        return best_det

    @staticmethod
    def _clone_core_detection(det: CoreDetection3D) -> CoreDetection3D:
        clip_feature = None
        if det.clip_feature is not None:
            clip_feature = np.array(det.clip_feature, copy=True)
        return CoreDetection3D(
            id=det.id,
            label=det.label,
            confidence=det.confidence,
            position=Vector3(det.position.x, det.position.y, det.position.z),
            bbox_2d=[float(x) for x in det.bbox_2d],
            clip_feature=clip_feature,
            ts=det.ts,
        )

    # == Query API =============================================================

    @property
    def tracker(self):
        """Access underlying InstanceTracker (for external queries / tests)."""
        return self._tracker

    @property
    def frame_count(self) -> int:
        return self._frame_count


# == Utility ===================================================================

def _quat_to_rotation(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
