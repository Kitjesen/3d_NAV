"""Perception stack: Detector + Encoder + Reconstruction."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def perception(detector: str = "yoloe", encoder: str = "mobileclip", **config) -> Blueprint:
    """Visual perception: object detection + CLIP encoding + 3D reconstruction."""
    bp = Blueprint()
    # Pull up camera service on demand
    # Camera service + bridge (only when perception is needed)
    try:
        from core.service_manager import get_service_manager
        svc = get_service_manager()
        svc.ensure("camera")
    except Exception:
        pass
    try:
        from core.registry import get as _get_drv
        drv_name = config.get("_driver_cls_name", "")
        # Add camera bridge if driver has no camera output
        from drivers.thunder.camera_bridge_module import CameraBridgeModule
        bp.add(CameraBridgeModule)
    except ImportError:
        pass
    try:
        from semantic.perception.semantic_perception.detector_module import DetectorModule
        from semantic.perception.semantic_perception.encoder_module import EncoderModule
        bp.add(DetectorModule, detector=detector,
               confidence=config.get("confidence", 0.3))
        bp.add(EncoderModule, encoder=encoder)
    except ImportError as e:
        logger.warning("Perception modules not available: %s", e)

    try:
        from semantic.reconstruction.reconstruction_module import ReconstructionModule
        bp.add(ReconstructionModule)
    except ImportError:
        pass

    return bp
