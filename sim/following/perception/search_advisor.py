"""Search advisor — 3-tier scene understanding for person re-acquisition.

When the person disappears, this module analyzes the camera image to
decide WHERE to search, using a fast→medium→slow cascade:

  Fast   (26ms):  YOLO detects doors/corridors/exits in current view
  Medium (2-3s):  SmolVLM local model describes the scene
  Slow   (1-2s):  Kimi API full reasoning (online only)

Each tier runs asynchronously. Fast starts immediately and the robot
begins moving. Medium/Slow results update the search direction when ready.

Usage::

    advisor = SearchAdvisor()

    # When person disappears:
    advice = advisor.get_search_direction(
        rgb_image,
        last_seen_position,
        last_seen_velocity,
        robot_position,
    )
    # advice.direction = yaw angle to search
    # advice.description = "door on the left"
    # advice.confidence = 0.8
    # advice.source = "yolo" / "vlm" / "kimi"
"""
from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class SearchAdvice:
    """Where to search for the lost person."""
    direction: float = 0.0        # yaw angle in world frame (radians)
    position: Optional[np.ndarray] = None  # target position to walk toward
    description: str = ""         # human-readable explanation
    confidence: float = 0.0       # 0-1
    source: str = "none"          # "geometry" / "yolo" / "vlm" / "kimi"


class SearchAdvisor:
    """3-tier search direction advisor.

    Tier 1 (Fast): Geometric — use last velocity to predict direction.
        Always available, instant, low confidence.

    Tier 2 (Medium): SmolVLM — local VLM analyzes camera frame.
        2-3s on CPU, medium confidence, works offline.

    Tier 3 (Slow): Kimi API — cloud VLM for deep reasoning.
        1-2s with network, high confidence, needs internet.
    """

    def __init__(
        self,
        enable_vlm: bool = True,
        enable_kimi: bool = True,
        vlm_model_name: str = "HuggingFaceTB/SmolVLM-256M-Instruct",
    ):
        self._enable_vlm = enable_vlm
        self._enable_kimi = enable_kimi
        self._vlm_model_name = vlm_model_name

        # Lazy-loaded models
        self._vlm_model = None
        self._vlm_processor = None
        self._vlm_loaded = False
        self._vlm_loading = False

        # Async results
        self._vlm_result: Optional[SearchAdvice] = None
        self._kimi_result: Optional[SearchAdvice] = None
        self._lock = threading.Lock()

    # ── Public API ──

    def get_search_direction(
        self,
        rgb_image: Optional[np.ndarray],
        last_position: np.ndarray,
        last_velocity: np.ndarray,
        robot_position: np.ndarray,
        robot_yaw: float = 0.0,
        scene_objects: Optional[list] = None,
    ) -> SearchAdvice:
        """Get best available search direction (non-blocking).

        Returns immediately with geometric estimate. Kicks off VLM/Kimi
        in background threads. Call again later for updated results.
        """
        # Tier 1: Always available — geometric prediction
        fast = self._geometric_advice(last_position, last_velocity, robot_position)

        # Check for async results from previous calls
        with self._lock:
            if self._vlm_result is not None and self._vlm_result.confidence > fast.confidence:
                fast = self._vlm_result
            if self._kimi_result is not None and self._kimi_result.confidence > fast.confidence:
                fast = self._kimi_result

        # Tier 1.5: YOLO scene objects (if available)
        if scene_objects:
            yolo_advice = self._yolo_advice(scene_objects, last_position, last_velocity, robot_position)
            if yolo_advice.confidence > fast.confidence:
                fast = yolo_advice

        # Tier 2: Kick off VLM in background (if not already running)
        if self._enable_vlm and rgb_image is not None and not self._vlm_loading:
            self._run_vlm_async(rgb_image, last_position, last_velocity, robot_yaw)

        # Tier 3: Kick off Kimi in background
        if self._enable_kimi and rgb_image is not None:
            self._run_kimi_async(rgb_image, last_position, last_velocity)

        return fast

    def get_latest_advice(self) -> Optional[SearchAdvice]:
        """Get the latest async result (VLM or Kimi) if available."""
        with self._lock:
            best = None
            for result in [self._vlm_result, self._kimi_result]:
                if result is not None:
                    if best is None or result.confidence > best.confidence:
                        best = result
            return best

    def clear(self) -> None:
        """Clear async results (call when person is re-acquired)."""
        with self._lock:
            self._vlm_result = None
            self._kimi_result = None

    # ── Tier 1: Geometric ──

    def _geometric_advice(
        self, last_pos, last_vel, robot_pos,
    ) -> SearchAdvice:
        speed = np.linalg.norm(last_vel[:2])
        if speed > 0.05:
            direction = math.atan2(last_vel[1], last_vel[0])
            # Predict position 3 seconds ahead
            pred_pos = last_pos[:2] + last_vel[:2] * 3.0
            return SearchAdvice(
                direction=direction,
                position=np.array([pred_pos[0], pred_pos[1], 0.0]),
                description=f"last heading {math.degrees(direction):.0f}deg",
                confidence=0.3,
                source="geometry",
            )
        else:
            # Person was stationary — search from robot toward last position
            dx = last_pos[0] - robot_pos[0]
            dy = last_pos[1] - robot_pos[1]
            direction = math.atan2(dy, dx)
            return SearchAdvice(
                direction=direction,
                position=last_pos[:3].copy(),
                description="last known position (stationary)",
                confidence=0.2,
                source="geometry",
            )

    # ── Tier 1.5: YOLO scene objects ──

    def _yolo_advice(
        self, objects, last_pos, last_vel, robot_pos,
    ) -> SearchAdvice:
        """Use detected doors/corridors to infer search direction."""
        passage_keywords = {"door", "entrance", "exit", "corridor", "hallway", "gate", "opening"}

        candidates = []
        for obj in objects:
            label = (obj.get("label", "") or "").lower()
            if any(kw in label for kw in passage_keywords):
                pos = obj.get("position", [0, 0, 0])
                if isinstance(pos, dict):
                    pos = [pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)]
                # Score: closer to person's last trajectory = higher
                pos_arr = np.array(pos[:2])
                pred_line = last_pos[:2] + last_vel[:2] * 2.0
                dist_to_trajectory = np.linalg.norm(pos_arr - pred_line)
                score = max(0, 1.0 - dist_to_trajectory / 5.0)
                candidates.append((pos_arr, label, score))

        if not candidates:
            return SearchAdvice(confidence=0.0)

        best = max(candidates, key=lambda c: c[2])
        pos_arr, label, score = best
        direction = math.atan2(pos_arr[1] - robot_pos[1], pos_arr[0] - robot_pos[0])
        return SearchAdvice(
            direction=direction,
            position=np.array([pos_arr[0], pos_arr[1], 0.0]),
            description=f"YOLO: {label} near trajectory",
            confidence=min(0.7, score + 0.3),
            source="yolo",
        )

    # ── Tier 2: SmolVLM (async) ──

    def _run_vlm_async(self, rgb, last_pos, last_vel, robot_yaw):
        """Run SmolVLM in a background thread."""
        self._vlm_loading = True

        def _worker():
            try:
                result = self._vlm_infer(rgb, last_pos, last_vel, robot_yaw)
                with self._lock:
                    self._vlm_result = result
            except Exception as e:
                logger.debug("VLM inference failed: %s", e)
            finally:
                self._vlm_loading = False

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def _vlm_infer(self, rgb, last_pos, last_vel, robot_yaw) -> SearchAdvice:
        """SmolVLM inference — describe scene to infer person direction."""
        if not self._vlm_loaded:
            self._load_vlm()
        if self._vlm_model is None:
            return SearchAdvice(confidence=0.0)

        import torch
        from PIL import Image

        # Prepare image
        if rgb.shape[2] == 3:
            pil_image = Image.fromarray(rgb)
        else:
            pil_image = Image.fromarray(rgb[:, :, :3])

        prompt = (
            "Look at this robot camera image. A person was being followed "
            "but just disappeared from view. "
            "Describe what you see: are there doors, corridors, corners, "
            "or openings where someone could have gone? "
            "Answer in one short sentence: 'The person likely went [direction/location]'"
        )

        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "image", "image": pil_image},
                    {"type": "text", "text": prompt},
                ],
            }
        ]

        text = self._vlm_processor.apply_chat_template(messages, add_generation_prompt=True)
        inputs = self._vlm_processor(
            text=[text], images=[pil_image], return_tensors="pt"
        )

        with torch.no_grad():
            output_ids = self._vlm_model.generate(
                **inputs, max_new_tokens=64, do_sample=False,
            )
        response = self._vlm_processor.decode(output_ids[0], skip_special_tokens=True)

        # Parse direction from response
        direction, confidence = self._parse_vlm_response(response, robot_yaw)

        return SearchAdvice(
            direction=direction,
            description=f"VLM: {response[:100]}",
            confidence=confidence,
            source="vlm",
        )

    def _load_vlm(self):
        """Lazy-load SmolVLM model."""
        try:
            from transformers import AutoProcessor, AutoModelForImageTextToText
            import torch

            logger.info("Loading SmolVLM (%s)...", self._vlm_model_name)
            self._vlm_processor = AutoProcessor.from_pretrained(
                self._vlm_model_name, trust_remote_code=True,
            )
            self._vlm_model = AutoModelForImageTextToText.from_pretrained(
                self._vlm_model_name, trust_remote_code=True,
                torch_dtype=torch.float32,
            )
            self._vlm_model.eval()
            self._vlm_loaded = True
            params = sum(p.numel() for p in self._vlm_model.parameters())
            logger.info("SmolVLM loaded: %.0fM params", params / 1e6)
        except Exception as e:
            logger.warning("SmolVLM load failed: %s", e)
            self._vlm_model = None
            self._vlm_loaded = True  # don't retry

    def _parse_vlm_response(self, response: str, robot_yaw: float) -> tuple:
        """Extract direction from VLM text response."""
        response_lower = response.lower()
        # Simple keyword → relative direction mapping
        if "left" in response_lower:
            return robot_yaw + math.pi / 2, 0.6
        elif "right" in response_lower:
            return robot_yaw - math.pi / 2, 0.6
        elif "ahead" in response_lower or "forward" in response_lower or "front" in response_lower:
            return robot_yaw, 0.5
        elif "behind" in response_lower or "back" in response_lower:
            return robot_yaw + math.pi, 0.5
        elif "door" in response_lower or "corridor" in response_lower:
            return robot_yaw, 0.4  # some passage ahead
        return robot_yaw, 0.2  # no clear direction

    # ── Tier 3: Kimi API (async) ──

    def _run_kimi_async(self, rgb, last_pos, last_vel):
        """Run Kimi API in a background thread."""
        def _worker():
            try:
                result = self._kimi_infer(rgb, last_pos, last_vel)
                with self._lock:
                    self._kimi_result = result
            except Exception as e:
                logger.debug("Kimi inference failed: %s", e)

        t = threading.Thread(target=_worker, daemon=True)
        t.start()

    def _kimi_infer(self, rgb, last_pos, last_vel) -> SearchAdvice:
        """Kimi API call for deep scene reasoning."""
        import os
        import base64
        from PIL import Image
        import io

        api_key = os.environ.get("MOONSHOT_API_KEY")
        if not api_key:
            return SearchAdvice(confidence=0.0)

        # Encode image to base64
        pil = Image.fromarray(rgb[:, :, :3] if rgb.shape[2] >= 3 else rgb)
        pil = pil.resize((320, 240))  # shrink for fast upload
        buf = io.BytesIO()
        pil.save(buf, format="JPEG", quality=60)
        b64 = base64.b64encode(buf.getvalue()).decode()

        try:
            import httpx
            resp = httpx.post(
                "https://api.moonshot.cn/v1/chat/completions",
                headers={"Authorization": f"Bearer {api_key}"},
                json={
                    "model": "moonshot-v1-8k-vision-preview",
                    "messages": [
                        {
                            "role": "user",
                            "content": [
                                {
                                    "type": "image_url",
                                    "image_url": {"url": f"data:image/jpeg;base64,{b64}"},
                                },
                                {
                                    "type": "text",
                                    "text": (
                                        "This is a robot's camera view. A person being followed "
                                        "just disappeared. The person was moving at velocity "
                                        f"({last_vel[0]:.1f}, {last_vel[1]:.1f}) m/s. "
                                        "Where did they most likely go? "
                                        "Answer: LEFT, RIGHT, FORWARD, or BEHIND, "
                                        "and explain in one sentence."
                                    ),
                                },
                            ],
                        }
                    ],
                    "max_tokens": 100,
                },
                timeout=5.0,
            )
            data = resp.json()
            text = data["choices"][0]["message"]["content"]
            direction, confidence = self._parse_vlm_response(text, 0.0)
            return SearchAdvice(
                direction=direction,
                description=f"Kimi: {text[:100]}",
                confidence=min(0.9, confidence + 0.2),
                source="kimi",
            )
        except Exception as e:
            logger.debug("Kimi API error: %s", e)
            return SearchAdvice(confidence=0.0)
