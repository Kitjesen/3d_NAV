"""DetectorModule — pluggable object detector as an independent Module.

Wraps any DetectorBase implementation (YOLO-World, YOLO-E, GroundingDINO,
BPU) into a core Module with typed In/Out ports. Swapping detectors is
a one-line Blueprint change.

Usage::

    from core.blueprints import Blueprint
    from semantic.perception.semantic_perception.semantic_perception.detector_module import DetectorModule

    # YOLO-World on GPU
    bp.add(DetectorModule, detector="yolo_world", model_size="l")

    # BPU on S100P
    bp.add(DetectorModule, detector="bpu", model_path="/opt/models/yolo.hbm")

    # Swap = change one argument, nothing else touches
"""

from __future__ import annotations

import logging
import time
from typing import Any, Dict, List, Optional

import numpy as np

from core.module import Module
from core.registry import register
from core.stream import In, Out

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Output type — framework-level detection result
# ---------------------------------------------------------------------------

class DetectionResult:
    """Detector output: list of 2D detections + metadata.

    Serializable, transport-friendly. Decoupled from detector internals.
    """
    __slots__ = ("detections", "detector_name", "inference_ms", "timestamp")

    def __init__(self, detections: list, timestamp: float = 0.0,
                 detector_name: str = "", inference_ms: float = 0.0):
        self.detections = detections
        self.timestamp = timestamp or time.time()
        self.detector_name = detector_name
        self.inference_ms = inference_ms

    def __repr__(self):
        return (f"DetectionResult(n={len(self.detections)}, "
                f"det={self.detector_name}, {self.inference_ms:.1f}ms)")


# ---------------------------------------------------------------------------
# DetectorModule
# ---------------------------------------------------------------------------

@register("detector", "pluggable", description="Pluggable detector module")
class DetectorModule(Module, layer=3):
    """Pluggable object detector Module.

    In:  image (np.ndarray)  — BGR uint8 HxWx3
    Out: detections (DetectionResult) — list of Detection2D + metadata

    Swap the detector backend via ``detector=`` config parameter:
      "yolo_world", "yoloe", "grounding_dino", "bpu"
    """

    image: In[np.ndarray]
    detections: Out[DetectionResult]

    def __init__(
        self,
        detector: str = "yoloe",
        text_prompt: str = "",
        confidence: float = 0.3,
        iou_threshold: float = 0.45,
        max_detections: int = 64,
        min_box_size_px: int = 12,
        model_size: str = "l",
        model_path: str = "",
        device: str = "",
        **kw,
    ):
        super().__init__(**kw)
        self._detector_name = detector
        self._text_prompt = text_prompt
        self._confidence = confidence
        self._iou_threshold = iou_threshold
        self._max_detections = max_detections
        self._min_box_size_px = min_box_size_px
        self._model_size = model_size
        self._model_path = model_path
        self._device = device
        self._backend = None
        self._frame_count = 0
        self._total_inference_ms = 0.0

    def setup(self):
        """Load the selected detector backend."""
        self._backend = self._create_backend()
        try:
            self._backend.load_model()
            logger.info("DetectorModule: loaded '%s' backend", self._detector_name)
        except Exception:
            logger.exception("DetectorModule: failed to load '%s'", self._detector_name)
            self._backend = None
        self.image.subscribe(self._on_image)

    def _create_backend(self):
        """Factory: instantiate the selected detector backend."""
        name = self._detector_name.lower()

        if name == "yolo_world":
            from .yolo_world_detector import YOLOWorldDetector
            return YOLOWorldDetector(
                model_size=self._model_size,
                confidence=self._confidence,
                iou_threshold=self._iou_threshold,
                device=self._device,
            )
        elif name == "yoloe":
            from .yoloe_detector import YOLOEDetector
            return YOLOEDetector(
                model_size=self._model_size,
                confidence=self._confidence,
                iou_threshold=self._iou_threshold,
                device=self._device,
                max_detections=self._max_detections,
            )
        elif name == "grounding_dino":
            from .grounding_dino_detector import GroundingDINODetector
            return GroundingDINODetector(confidence=self._confidence)
        elif name == "bpu":
            from .bpu_detector import BPUDetector
            return BPUDetector(
                model_path=self._model_path,
                confidence=self._confidence,
                iou_threshold=self._iou_threshold,
                max_detections=self._max_detections,
                min_box_size_px=self._min_box_size_px,
            )
        else:
            raise ValueError(
                f"Unknown detector '{name}'. "
                f"Available: yolo_world, yoloe, grounding_dino, bpu"
            )

    def _on_image(self, image: np.ndarray):
        """Run detection on incoming image, publish results."""
        if self._backend is None:
            return

        t0 = time.time()
        try:
            dets = self._backend.detect(image, self._text_prompt)
        except Exception:
            logger.exception("DetectorModule: detection failed")
            return
        inference_ms = (time.time() - t0) * 1000

        self._frame_count += 1
        self._total_inference_ms += inference_ms

        result = DetectionResult(
            detections=dets,
            timestamp=time.time(),
            detector_name=self._detector_name,
            inference_ms=inference_ms,
        )
        self.detections.publish(result)

    def set_prompt(self, text_prompt: str):
        """Update detection classes at runtime."""
        self._text_prompt = text_prompt

    def stop(self):
        """Shutdown the detector backend."""
        if self._backend and hasattr(self._backend, 'shutdown'):
            try:
                self._backend.shutdown()
            except Exception:
                pass
        super().stop()

    def health(self) -> dict[str, Any]:
        info = super().port_summary()
        avg_ms = (self._total_inference_ms / self._frame_count
                  if self._frame_count > 0 else 0.0)
        info["detector"] = {
            "backend": self._detector_name,
            "loaded": self._backend is not None,
            "frames": self._frame_count,
            "avg_ms": round(avg_ms, 1),
        }
        return info
