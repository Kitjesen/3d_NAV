"""Camera → YOLO → FusionMOT → 3D perception pipeline.

Full perception chain using the same production code as the real robot:
  1. MuJoCo camera render → RGB image
  2. qp_perception YoloDetector → person detections
  3. qp_perception FusionMOT → stable track IDs
  4. Depth deprojection → 3D world position

No ROS2 involved — direct Python calls to qp_perception.
"""
from __future__ import annotations

import logging
import math
from typing import Optional

import numpy as np

from sim.following.interfaces import PerceivedTarget, PersonState

logger = logging.getLogger(__name__)


class CameraYoloPipeline:
    """Full camera-based perception for person following simulation.

    Uses qp_perception's YoloDetector + FusionMOT + PersonFollowingSelector
    via direct Python import — identical code path to S100P real robot.
    """

    def __init__(
        self,
        camera_name: str = "front_camera",
        yolo_model: str = "yolo11n.pt",
        confidence: float = 0.3,
        render_width: int = 640,
        render_height: int = 480,
        throttle_steps: int = 3,
    ):
        self._camera_name = camera_name
        self._render_w = render_width
        self._render_h = render_height
        self._throttle = throttle_steps
        self._step_count = 0
        self._last_target: Optional[PerceivedTarget] = None

        # Lazy init (deferred to first update call)
        self._detector = None
        self._tracker = None
        self._selector = None
        self._renderer = None
        self._depth_renderer = None
        self._initialized = False
        self._yolo_model = yolo_model
        self._confidence = confidence

        # Camera intrinsics (set from MuJoCo model on init)
        self._fx = 0.0
        self._fy = 0.0
        self._cx = 0.0
        self._cy = 0.0

    def _lazy_init(self, engine):
        """Initialize detector, tracker, selector on first call."""
        if self._initialized:
            return

        import mujoco

        # Camera intrinsics from MuJoCo model
        cam_id = mujoco.mj_name2id(
            engine._model, mujoco.mjtObj.mjOBJ_CAMERA, self._camera_name
        )
        if cam_id < 0:
            logger.error("Camera '%s' not found in model", self._camera_name)
            self._initialized = True
            return

        fovy = engine._model.cam_fovy[cam_id]
        fovy_rad = math.radians(fovy)
        self._fy = self._render_h / (2.0 * math.tan(fovy_rad / 2.0))
        aspect = self._render_w / self._render_h
        self._fx = self._fy  # square pixels
        self._cx = self._render_w / 2.0
        self._cy = self._render_h / 2.0

        # MuJoCo renderer for RGB + depth
        self._renderer = mujoco.Renderer(
            engine._model, height=self._render_h, width=self._render_w
        )
        self._depth_renderer = mujoco.Renderer(
            engine._model, height=self._render_h, width=self._render_w
        )

        # qp_perception components
        try:
            from qp_perception.detection.yolo import YoloDetector
            from qp_perception.tracking.fusion import FusionMOT, FusionMOTConfig
            from qp_perception.selection.person_following import (
                FollowingConfig, PersonFollowingSelector,
            )

            self._detector = YoloDetector(
                model_path=self._yolo_model,
                confidence_threshold=self._confidence,
                class_whitelist=("person",),
            )
            self._tracker = FusionMOT(config=FusionMOTConfig())
            self._selector = PersonFollowingSelector(
                FollowingConfig(auto_lock=True, min_confidence=0.2)
            )
            logger.info("CameraYoloPipeline: YOLO + FusionMOT + Selector ready")
        except ImportError as e:
            logger.warning("qp_perception not available: %s", e)
            logger.warning("Falling back to ground truth perception")

        self._initialized = True

    def update(
        self,
        engine,
        person_gt: Optional[PersonState] = None,
        timestamp: float = 0.0,
    ) -> Optional[PerceivedTarget]:
        self._lazy_init(engine)
        self._step_count += 1

        # Throttle: only run YOLO every N steps
        if self._step_count % self._throttle != 0:
            return self._last_target

        # Fallback if no detector available
        if self._detector is None:
            if person_gt is not None and person_gt.visible:
                return PerceivedTarget(
                    position_world=person_gt.position.copy(),
                    confidence=1.0,
                    timestamp=timestamp,
                )
            return None

        import mujoco

        # 1. Render RGB
        self._renderer.update_scene(engine._data, camera=self._camera_name)
        rgb = self._renderer.render()  # (H, W, 3) uint8 RGB

        # 2. Render depth
        self._depth_renderer.update_scene(engine._data, camera=self._camera_name)
        self._depth_renderer.enable_depth_rendering()
        depth_raw = self._depth_renderer.render()  # (H, W) float32 [0, 1] NDC
        self._depth_renderer.disable_depth_rendering()

        # Linearize depth: NDC → meters
        znear = engine._model.vis.map.znear * engine._model.stat.extent
        zfar = engine._model.vis.map.zfar * engine._model.stat.extent
        depth_m = np.where(
            depth_raw < 1.0,
            znear * zfar / (zfar - depth_raw * (zfar - znear)),
            0.0,
        )

        # 3. YOLO detection (needs BGR)
        bgr = rgb[:, :, ::-1].copy()
        detections = self._detector.detect(bgr, timestamp)

        if not detections:
            self._last_target = None
            return None

        # 4. FusionMOT tracking
        bboxes = np.array(
            [[d.bbox.x, d.bbox.y, d.bbox.w, d.bbox.h] for d in detections],
            dtype=np.float32,
        )
        confs = np.array([d.confidence for d in detections], dtype=np.float32)
        tracks = self._tracker.update(bboxes, confs, None, timestamp)

        if not tracks:
            self._last_target = None
            return None

        # 5. Selector picks target
        from qp_perception.types import BoundingBox, Track as QPTrack
        qp_tracks = []
        for tid, bbox_arr, conf in tracks:
            x, y, w, h = bbox_arr
            qp_tracks.append(QPTrack(
                track_id=int(tid),
                bbox=BoundingBox(x=float(x), y=float(y), w=float(w), h=float(h)),
                confidence=float(conf),
                class_id="person",
                first_seen_ts=timestamp,
                last_seen_ts=timestamp,
            ))

        obs = self._selector.select(qp_tracks, timestamp)
        if obs is None:
            self._last_target = None
            return None

        # 6. Depth deprojection: bbox center → 3D world
        cx_px = obs.bbox.x + obs.bbox.w / 2
        cy_px = obs.bbox.y + obs.bbox.h / 2
        ix, iy = int(cx_px), int(cy_px)
        ix = max(0, min(ix, self._render_w - 1))
        iy = max(0, min(iy, self._render_h - 1))
        z_m = float(depth_m[iy, ix])

        if z_m <= 0.1 or z_m > 20.0:
            # Invalid depth, try median in bbox region
            bx1 = max(0, int(obs.bbox.x))
            by1 = max(0, int(obs.bbox.y))
            bx2 = min(self._render_w, int(obs.bbox.x + obs.bbox.w))
            by2 = min(self._render_h, int(obs.bbox.y + obs.bbox.h))
            roi = depth_m[by1:by2, bx1:bx2]
            valid = roi[(roi > 0.1) & (roi < 20.0)]
            if len(valid) > 0:
                z_m = float(np.median(valid))
            else:
                self._last_target = None
                return None

        # Camera frame: x_cam = (px - cx) * z / fx, y_cam = (py - cy) * z / fy
        x_cam = (cx_px - self._cx) * z_m / self._fx
        y_cam = (cy_px - self._cy) * z_m / self._fy

        # Camera → world transform
        cam_body_id = mujoco.mj_name2id(
            engine._model, mujoco.mjtObj.mjOBJ_CAMERA, self._camera_name
        )
        R_cam = engine._data.cam_xmat[cam_body_id].reshape(3, 3)
        t_cam = engine._data.cam_xpos[cam_body_id]

        # MuJoCo camera: -Z forward, Y up, X right
        point_cam = np.array([-x_cam, y_cam, -z_m])
        point_world = R_cam @ point_cam + t_cam

        self._last_target = PerceivedTarget(
            position_world=point_world,
            confidence=float(obs.confidence),
            bbox=(float(obs.bbox.x), float(obs.bbox.y),
                  float(obs.bbox.w), float(obs.bbox.h)),
            track_id=obs.track_id,
            timestamp=timestamp,
        )
        return self._last_target
