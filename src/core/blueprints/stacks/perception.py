"""Perception stack: Detector + Encoder + Reconstruction."""

from __future__ import annotations

import logging

from core.blueprint import Blueprint

logger = logging.getLogger(__name__)


def perception(detector: str = "yoloe", encoder: str = "mobileclip", **config) -> Blueprint:
    """Visual perception: object detection + CLIP encoding + 3D reconstruction."""
    bp = Blueprint()
    # Pull up camera service on demand
    try:
        from core.service_manager import get_service_manager, SERVICES_CAMERA
        svc = get_service_manager()
        svc.ensure(*SERVICES_CAMERA)
    except Exception:
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
