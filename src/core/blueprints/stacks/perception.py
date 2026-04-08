"""Perception stack: semantic perception + optional encoder + reconstruction."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)
_NATIVE_CAMERA_DRIVERS = {"MujocoDriverModule"}  # Only MuJoCo has built-in camera


def perception(detector: str = "yoloe", encoder: str = "mobileclip", **config) -> Blueprint:
    """Visual perception: RGB-D semantic perception + optional encoder + 3D reconstruction."""
    bp = Blueprint()
    try:
        from core.service_manager import get_service_manager
        svc = get_service_manager()
        svc.ensure("camera")
    except Exception:
        pass

    drv_name = config.get("_driver_cls_name", "")
    needs_camera_bridge = bool(config.get("force_camera_bridge")) or (
        drv_name not in _NATIVE_CAMERA_DRIVERS
    )

    try:
        from drivers.thunder.camera_bridge_module import CameraBridgeModule

        if needs_camera_bridge:
            bp.add(CameraBridgeModule)
    except ImportError:
        pass

    try:
        from semantic.perception.semantic_perception.encoder_module import EncoderModule
        from semantic.perception.semantic_perception.perception_module import PerceptionModule

        bp.add(
            PerceptionModule,
            detector_type=detector,
            confidence_threshold=config.get("confidence", 0.3),
            tracking_iou_threshold=config.get(
                "tracking_iou_threshold",
                config.get("iou_threshold", 0.3),
            ),
            detector_iou_threshold=config.get(
                "detector_iou_threshold",
                config.get("iou_threshold", 0.45),
            ),
            detector_max_detections=config.get(
                "detector_max_detections",
                config.get("max_detections", 64),
            ),
            detector_min_box_size_px=config.get(
                "detector_min_box_size_px",
                config.get("min_box_size_px", 12),
            ),
            detector_model_size=config.get("model_size", "l"),
            detector_device=config.get("device", ""),
            detector_model_path=config.get(
                "detector_model_path",
                config.get("model_path", ""),
            ),
            skip_frames=config.get("perception_skip_frames", 1),
            world=config.get("world", ""),
        )
        bp.add(EncoderModule, encoder=encoder)
    except ImportError as e:
        logger.warning("Perception modules not available: %s", e)

    try:
        from semantic.reconstruction.reconstruction_module import ReconstructionModule
        bp.add(ReconstructionModule)
    except ImportError:
        pass

    return bp
