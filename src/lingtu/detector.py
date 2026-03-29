"""Detector — object detection, import and use."""

from __future__ import annotations

import logging
from typing import List, Optional

import numpy as np

logger = logging.getLogger(__name__)


class Detector:
    """Object detection — YOLO/BPU powered.

    Usage::

        detector = Detector(camera)
        detector.start()
        objects = detector.detect()        # [{"label": "chair", "confidence": 0.9, ...}]
        found = detector.find("red chair") # closest match
        detector.stop()
    """

    def __init__(self, camera=None, backend: str = "yoloe"):
        self._camera = camera
        self._backend = backend
        self._module = None
        self._latest_detections: List[dict] = []
        self._started = False

    def start(self) -> "Detector":
        if self._started:
            return self
        try:
            from semantic.perception.semantic_perception.detector_module import DetectorModule
            self._module = DetectorModule(detector=self._backend)
            self._module.setup()
            self._module.start()
            self._started = True
            logger.info("Detector started (backend=%s)", self._backend)
        except Exception as e:
            logger.error("Detector start failed: %s", e)
        return self

    def stop(self) -> None:
        if self._module:
            self._module.stop()
            self._module = None
        self._started = False

    def detect(self) -> List[dict]:
        """Run detection on latest camera frame. Returns list of detections."""
        if self._camera and self._module:
            color = self._camera.get_color()
            if color is not None and hasattr(self._module, "_backend") and self._module._backend:
                try:
                    results = self._module._backend.detect(color)
                    self._latest_detections = [
                        {"label": d.label, "confidence": d.confidence,
                         "bbox": list(d.bbox) if hasattr(d, "bbox") else []}
                        for d in results
                    ]
                except Exception as e:
                    logger.debug("Detection failed: %s", e)
        return self._latest_detections

    def find(self, label: str) -> Optional[dict]:
        """Find the best match for a label in current detections."""
        detections = self.detect()
        target = label.lower()
        best = None
        best_score = 0
        for d in detections:
            if target in d["label"].lower() or d["label"].lower() in target:
                if d["confidence"] > best_score:
                    best = d
                    best_score = d["confidence"]
        return best

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *args):
        self.stop()

    def __repr__(self):
        return "Detector(backend=%s, running=%s)" % (self._backend, self._started)
