"""
PersonTracker -- VLM 选人 + 实时视觉跟踪 (跟人走功能核心)

链路:
  1. VLM 一次性选人: "穿红衣服的人" → YOLO crops → VLM 选编号 → 锁定目标
  2. 帧间跟踪: IoU + 外观特征余弦相似度 + EMA 位置平滑 + 速度预测
  3. 丢失重识别: 外观特征搜索 → 兜底 VLM 重新选人

不依赖卡尔曼滤波库。
"""

import logging
import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class TrackedPerson:
    """被追踪的目标人物。"""
    position: list[float]                          # [x, y, z] 世界坐标
    velocity: list[float] = field(default_factory=lambda: [0.0, 0.0])  # [vx, vy] m/s
    last_seen: float = field(default_factory=time.time)
    confidence: float = 1.0
    # 外观特征 (用于遮挡后 Re-ID)
    appearance: np.ndarray | None = None        # CLIP 图像特征, shape (D,)
    bbox: list[int] | None = None               # 最近一帧的 [x1, y1, x2, y2]
    obj_id: str | None = None                   # 场景图 object ID (帧间匹配优先)


class PersonTracker:
    """
    VLM 辅助选人 + 实时视觉跟踪。

    使用方式:
      1. set_target(description, llm_fn, clip_fn) — 用 VLM 从候选人中选定目标
      2. update(scene_objects, rgb_frame) — 每帧更新跟踪状态
      3. get_follow_waypoint(robot_pos) — 获取跟随航点
    """

    LOST_TIMEOUT = 5.0          # 秒: 丢失判定
    REID_TIMEOUT = 15.0         # 秒: 超过此时间放弃 Re-ID, 需 VLM 重选
    EMA_ALPHA = 0.4             # 位置平滑系数
    APPEARANCE_ALPHA = 0.1      # 外观特征 EMA 更新系数
    MATCH_DIST_THRESHOLD = 2.0  # 米: 距离匹配阈值
    APPEARANCE_THRESHOLD = 0.6  # 外观相似度阈值 (Re-ID)

    def __init__(self, follow_distance: float = 1.5, lost_timeout: float = 5.0):
        self.follow_distance = follow_distance
        self.lost_timeout = lost_timeout
        self._person: TrackedPerson | None = None
        self._description: str = ""                # 用户描述 ("穿红衣服的人")
        self._target_selected: bool = False        # VLM 是否已选定目标
        self._vlm_selecting: bool = False          # VLM 选人中 (防重入)
        self._clip_encoder = None                  # CLIP 编码器 (外部注入)
        self._lock = threading.Lock()
        # FusionMOT backend (optional, enabled via enable_fusion_tracking)
        self._fusion_tracker = None
        self._reid_extractor = None
        self._target_track_id: int | None = None
        self._following_selector = None  # PersonFollowingSelector (optional)

    def set_clip_encoder(self, clip_encoder) -> None:
        """注入 CLIP 编码器，用于外观特征提取。"""
        self._clip_encoder = clip_encoder

    def enable_fusion_tracking(self) -> bool:
        """Enable FusionMOT + OSNet Re-ID backend for Kalman-smoothed tracking.

        When enabled, update() routes through FusionMOT which provides:
          - Kalman prediction between frames (smooth bbox trajectory)
          - OSNet Re-ID features (robust re-identification after occlusion)
          - Stable track IDs across frames

        Returns True if successfully initialized, False if qp_perception unavailable.
        """
        try:
            from qp_perception.reid.extractor import ReIDConfig, ReIDExtractor
            from qp_perception.tracking.fusion import FusionMOT, FusionMOTConfig

            reid_cfg = ReIDConfig(backbone="osnet_x1_0", device="")
            self._reid_extractor = ReIDExtractor(reid_cfg)

            mot_cfg = FusionMOTConfig()
            self._fusion_tracker = FusionMOT(
                config=mot_cfg,
                feature_dim=self._reid_extractor.feature_dim,
            )

            # PersonFollowingSelector: state machine for target lock lifecycle
            try:
                from qp_perception.selection.person_following import (
                    FollowingConfig,
                    PersonFollowingSelector,
                )
                self._following_selector = PersonFollowingSelector(
                    FollowingConfig(
                        search_timeout_s=self.REID_TIMEOUT,
                        auto_lock=True,
                        min_confidence=0.3,
                    )
                )
            except Exception:
                self._following_selector = None

            logger.info("PersonTracker: FusionMOT + OSNet Re-ID enabled")
            return True
        except Exception as e:
            logger.info(
                "PersonTracker: FusionMOT unavailable (%s), using classic tracking", e
            )
            self._fusion_tracker = None
            self._reid_extractor = None
            return False

    async def select_target_with_vlm(
        self,
        description: str,
        person_crops: list[np.ndarray],
        person_objects: list[dict],
        llm_chat_fn: Callable,
    ) -> int:
        """
        用 VLM 从候选人中选定目标 (一次性调用)。

        Args:
            description: 用户描述, 如 "穿红衣服背包的人"
            person_crops: 每个 person 的裁剪图像列表
            person_objects: 对应的场景图对象列表
            llm_chat_fn: async LLM 调用函数 (prompt) -> str

        Returns:
            选中的 person 索引 (0-based), -1 表示失败
        """
        if not person_crops:
            return -1

        self._description = description
        n = len(person_crops)

        # 构建 VLM prompt
        prompt = (
            f"画面中有 {n} 个人，编号 1 到 {n}。\n"
            f"用户想跟随的是: {description}\n"
            f"请只回答一个数字（1-{n}），表示最匹配的人的编号。\n"
            f"如果都不匹配，回答 0。"
        )

        # 如果有 CLIP，用 CLIP 做初步筛选辅助 VLM
        clip_scores = []
        if self._clip_encoder is not None:
            try:
                text_feat = self._clip_encoder.encode_text([description])
                if text_feat is not None and len(text_feat) > 0:
                    for crop in person_crops:
                        try:
                            img_feat = self._clip_encoder.encode_image(crop)
                            if img_feat is not None and img_feat.size > 0:
                                sim = float(np.dot(text_feat[0], img_feat) / (
                                    np.linalg.norm(text_feat[0]) * np.linalg.norm(img_feat) + 1e-9
                                ))
                                clip_scores.append(sim)
                            else:
                                clip_scores.append(0.0)
                        except Exception:
                            clip_scores.append(0.0)
            except Exception as e:
                logger.debug("CLIP scoring failed: %s", e)

        if clip_scores:
            prompt += f"\nCLIP 相似度参考: {[f'{s:.2f}' for s in clip_scores]}"

        # 调用 VLM
        try:
            response = await llm_chat_fn(prompt)
            # 解析数字
            import re
            numbers = re.findall(r'\d+', str(response))
            if numbers:
                idx = int(numbers[0])
                if 1 <= idx <= n:
                    selected = idx - 1  # 0-based
                    logger.info(
                        "VLM selected person #%d for '%s'", idx, description
                    )
                    # 锁定目标: 存储外观特征
                    self._lock_target(person_objects[selected], person_crops[selected])
                    return selected
                elif idx == 0:
                    logger.info("VLM: no matching person for '%s'", description)
                    return -1
        except Exception as e:
            logger.error("VLM selection failed: %s", e)

        # VLM 失败时，用 CLIP 分数选 (降级)
        if clip_scores:
            best_idx = int(np.argmax(clip_scores))
            if clip_scores[best_idx] > 0.2:
                logger.info(
                    "VLM failed, using CLIP fallback: person #%d (sim=%.2f)",
                    best_idx + 1, clip_scores[best_idx],
                )
                self._lock_target(person_objects[best_idx], person_crops[best_idx])
                return best_idx

        return -1

    def select_by_clip(
        self,
        description: str,
        person_crops: list[np.ndarray],
        person_objects: list[dict],
    ) -> int:
        """
        纯 CLIP 选人 (无 VLM 时的降级方案, 同步调用)。

        Returns:
            选中的 person 索引 (0-based), -1 表示失败
        """
        if not person_crops or self._clip_encoder is None:
            return -1

        self._description = description
        try:
            text_feat = self._clip_encoder.encode_text([description])
            if text_feat is None or len(text_feat) == 0:
                return -1

            best_idx = -1
            best_sim = 0.2  # 最低阈值
            for i, crop in enumerate(person_crops):
                try:
                    img_feat = self._clip_encoder.encode_image(crop)
                    if img_feat is not None and img_feat.size > 0:
                        sim = float(np.dot(text_feat[0], img_feat) / (
                            np.linalg.norm(text_feat[0]) * np.linalg.norm(img_feat) + 1e-9
                        ))
                        if sim > best_sim:
                            best_sim = sim
                            best_idx = i
                except Exception:
                    continue

            if best_idx >= 0:
                logger.info(
                    "CLIP selected person #%d for '%s' (sim=%.2f)",
                    best_idx + 1, description, best_sim,
                )
                self._lock_target(person_objects[best_idx], person_crops[best_idx])
                return best_idx
        except Exception as e:
            logger.error("CLIP selection failed: %s", e)
        return -1

    def _lock_target(self, obj: dict, crop: np.ndarray | None = None) -> None:
        """锁定目标，存储外观特征。"""
        pos = obj.get("position", [0, 0, 0])
        if isinstance(pos, dict):
            pos = [pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)]

        appearance = None
        if crop is not None and self._clip_encoder is not None:
            try:
                feat = self._clip_encoder.encode_image(crop)
                if feat is not None and feat.size > 0:
                    norm = np.linalg.norm(feat)
                    appearance = feat / norm if norm > 0 else feat
            except Exception:
                pass

        with self._lock:
            self._person = TrackedPerson(
                position=list(pos[:3]),
                last_seen=time.time(),
                confidence=obj.get("confidence", 1.0),
                appearance=appearance,
                bbox=obj.get("bbox"),
                obj_id=obj.get("id"),
            )
            self._target_selected = True

    def update(
        self,
        scene_objects: list[dict],
        rgb_frame: np.ndarray | None = None,
    ) -> bool:
        """
        每帧更新跟踪状态。

        When FusionMOT is enabled (via enable_fusion_tracking), routes through
        Kalman-smoothed tracking with OSNet Re-ID. Otherwise uses classic matching:
          1. obj_id 匹配 (instance_tracker 已赋予唯一 ID)
          2. 距离最近 (< MATCH_DIST_THRESHOLD)
          3. 外观特征 Re-ID (遮挡后恢复)

        Args:
            scene_objects: 场景图 objects (含 label, position, confidence, id, bbox)
            rgb_frame: 当前 RGB 帧 (用于外观特征更新)

        Returns:
            True 如果成功匹配到目标
        """
        persons = [
            o for o in scene_objects
            if o.get("label", "").lower() in ("person", "people", "human", "pedestrian")
        ]
        if not persons:
            self._fusion_tick_empty()
            return False

        with self._lock:
            # FusionMOT path: Kalman + OSNet Re-ID
            if self._fusion_tracker is not None and rgb_frame is not None:
                return self._update_fusion(persons, rgb_frame)

            # ── Classic path (unchanged) ──
            if self._person is None:
                # 未选定目标时，退化为跟最高置信度的 (兼容旧行为)
                best = max(persons, key=lambda p: p.get("confidence", 0))
                self._init_person(best)
                return True

            matched = self._match_person(persons, rgb_frame)
            if matched is not None:
                self._update_tracked(matched, rgb_frame)
                return True

            # 未经 VLM 选定时，退化为跟最近的 person (兼容旧行为)
            if not self._target_selected:
                nearest = min(persons, key=lambda p: math.hypot(
                    self._get_pos(p)[0] - self._person.position[0],
                    self._get_pos(p)[1] - self._person.position[1],
                ))
                self._update_tracked(nearest, rgb_frame)
                return True

        return False

    # ── FusionMOT backend ─────────────────────────────────────────────────────

    def _update_fusion(self, persons: list[dict], rgb_frame: np.ndarray) -> bool:
        """Tracking via FusionMOT: Kalman smoothing + OSNet Re-ID.

        Uses Selective Re-ID (Fast-Deep-OC-SORT): only extracts appearance
        features for detections that lack a high-IoU geometric match,
        saving ~60-70% of Re-ID computation in steady-state tracking.

        Flow:
          1. Convert person bboxes → FusionMOT input (xywh)
          2. FusionMOT.update_selective() → IoU pre-match → extract Re-ID
             only for ambiguous detections → full matching
          3. Match target by track_id (O(1) lookup) or fall back to classic
        """
        timestamp = time.time()

        # Convert person detections to FusionMOT format
        bboxes, confs, valid_persons = [], [], []
        for p in persons:
            bbox = p.get("bbox")
            if not bbox or len(bbox) < 4:
                continue
            if isinstance(bbox, np.ndarray):
                bbox = bbox.tolist()
            x1, y1 = float(bbox[0]), float(bbox[1])
            x2, y2 = float(bbox[2]), float(bbox[3])
            w, h = x2 - x1, y2 - y1
            if w <= 0 or h <= 0:
                continue
            bboxes.append([x1, y1, w, h])
            confs.append(float(p.get("confidence", 0.5)))
            valid_persons.append(p)

        if not bboxes:
            self._fusion_tick_empty()
            return False

        bboxes_np = np.array(bboxes, dtype=np.float32)
        confs_np = np.array(confs, dtype=np.float32)

        # Selective Re-ID: only extract for ambiguous/unmatched detections
        tracks = self._fusion_tracker.update_selective(
            bboxes_np, confs_np, timestamp,
            reid_extractor=self._reid_extractor,
            frame=rgb_frame,
        )
        if not tracks:
            return False

        # Map track_id → person object (by bbox center proximity)
        track_map = self._map_tracks_to_persons(tracks, valid_persons)

        # ── PersonFollowingSelector path ──
        if self._following_selector is not None:
            track_objects = self._raw_to_tracks(tracks, timestamp)

            # If VLM selected a person but selector not yet locked, associate
            if (
                not self._following_selector.is_locked
                and self._person is not None
                and self._target_selected
            ):
                matched = self._match_person(list(track_map.values()), rgb_frame)
                if matched is not None:
                    for tid, p in track_map.items():
                        if p is matched:
                            self._following_selector.lock_track(tid, self._description)
                            self._target_track_id = tid
                            break

            obs = self._following_selector.select(track_objects, timestamp)
            if obs is not None:
                person = track_map.get(obs.track_id)
                if person is not None:
                    self._target_track_id = obs.track_id
                    self._update_tracked(person, rgb_frame)
                    return True
            return False

        # ── Fallback: manual track_id matching (no selector) ──
        if self._target_track_id is not None and self._target_track_id in track_map:
            self._update_tracked(track_map[self._target_track_id], rgb_frame)
            return True

        if self._person is not None:
            matched = self._match_person(list(track_map.values()), rgb_frame)
            if matched is not None:
                for tid, p in track_map.items():
                    if p is matched:
                        self._target_track_id = tid
                        break
                self._update_tracked(matched, rgb_frame)
                return True
            return False

        if not self._target_selected:
            best_tid = max(track_map, key=lambda t: track_map[t].get("confidence", 0))
            self._init_person(track_map[best_tid])
            self._target_track_id = best_tid
            return True

        return False

    def _fusion_tick_empty(self) -> None:
        """Maintain FusionMOT state with no detections (keeps lost-track timers)."""
        if self._fusion_tracker is None:
            return
        try:
            self._fusion_tracker.update(
                np.empty((0, 4)), np.array([]), None, time.time()
            )
        except Exception:
            pass

    @staticmethod
    def _raw_to_tracks(raw_tracks: list, timestamp: float) -> list:
        """Convert FusionMOT raw output to qp_perception Track objects."""
        from qp_perception.types import BoundingBox, Track
        result = []
        for track_id, bbox_arr, conf in raw_tracks:
            x, y, w, h = bbox_arr
            result.append(Track(
                track_id=int(track_id),
                bbox=BoundingBox(x=float(x), y=float(y), w=float(w), h=float(h)),
                confidence=float(conf),
                class_id="person",
                first_seen_ts=timestamp,
                last_seen_ts=timestamp,
            ))
        return result

    @staticmethod
    def _map_tracks_to_persons(
        tracks: list, persons: list[dict],
    ) -> dict[int, dict]:
        """Map FusionMOT track_id to nearest input person by bbox center distance."""
        result: dict[int, dict] = {}
        for track_id, bbox_arr, _conf in tracks:
            tx, ty, tw, th = bbox_arr
            tcx, tcy = tx + tw / 2, ty + th / 2
            best_p, best_d = None, float("inf")
            for p in persons:
                pb = p.get("bbox", [0, 0, 0, 0])
                if isinstance(pb, np.ndarray):
                    pb = pb.tolist()
                if len(pb) < 4:
                    continue
                pcx = (float(pb[0]) + float(pb[2])) / 2
                pcy = (float(pb[1]) + float(pb[3])) / 2
                d = (tcx - pcx) ** 2 + (tcy - pcy) ** 2
                if d < best_d:
                    best_d = d
                    best_p = p
            if best_p is not None:
                result[int(track_id)] = best_p
        return result

    def _match_person(
        self, persons: list[dict], rgb_frame: np.ndarray | None
    ) -> dict | None:
        """在候选 persons 中匹配已锁定的目标。"""
        if self._person is None:
            return None

        # 策略 1: obj_id 精确匹配
        if self._person.obj_id:
            for p in persons:
                if p.get("id") == self._person.obj_id:
                    return p

        # 策略 2: 距离最近匹配
        best_dist_p = None
        best_dist = self.MATCH_DIST_THRESHOLD
        for p in persons:
            pos = self._get_pos(p)
            dist = math.hypot(
                pos[0] - self._person.position[0],
                pos[1] - self._person.position[1],
            )
            if dist < best_dist:
                best_dist = dist
                best_dist_p = p

        if best_dist_p is not None:
            return best_dist_p

        # 策略 3: 外观 Re-ID (距离匹配失败时)
        if self._person.appearance is not None and rgb_frame is not None and self._clip_encoder is not None:
            best_sim_p = None
            best_sim = self.APPEARANCE_THRESHOLD
            for p in persons:
                crop = self._crop_person(rgb_frame, p)
                if crop is None:
                    continue
                try:
                    feat = self._clip_encoder.encode_image(crop)
                    if feat is not None and feat.size > 0:
                        norm = np.linalg.norm(feat)
                        if norm > 0:
                            feat = feat / norm
                        sim = float(np.dot(self._person.appearance, feat))
                        if sim > best_sim:
                            best_sim = sim
                            best_sim_p = p
                except Exception:
                    continue
            if best_sim_p is not None:
                logger.info("Re-ID matched person (sim=%.2f)", best_sim)
                return best_sim_p

        return None

    def _update_tracked(self, obj: dict, rgb_frame: np.ndarray | None) -> None:
        """更新已匹配目标的位置和外观。"""
        new_pos = self._get_pos(obj)
        now = time.time()

        dt = now - self._person.last_seen
        old = self._person.position

        # 速度 EMA
        if dt > 0.01:
            self._person.velocity = [
                (new_pos[0] - old[0]) / dt * 0.3 + self._person.velocity[0] * 0.7,
                (new_pos[1] - old[1]) / dt * 0.3 + self._person.velocity[1] * 0.7,
            ]

        # 位置 EMA
        a = self.EMA_ALPHA
        self._person.position = [
            a * new_pos[0] + (1 - a) * old[0],
            a * new_pos[1] + (1 - a) * old[1],
            a * new_pos[2] + (1 - a) * old[2],
        ]
        self._person.last_seen = now
        self._person.confidence = obj.get("confidence", 1.0)
        self._person.obj_id = obj.get("id", self._person.obj_id)
        self._person.bbox = obj.get("bbox", self._person.bbox)

        # 外观特征 EMA 更新
        if rgb_frame is not None and self._clip_encoder is not None:
            crop = self._crop_person(rgb_frame, obj)
            if crop is not None:
                try:
                    feat = self._clip_encoder.encode_image(crop)
                    if feat is not None and feat.size > 0:
                        norm = np.linalg.norm(feat)
                        if norm > 0:
                            feat = feat / norm
                        if self._person.appearance is not None:
                            aa = self.APPEARANCE_ALPHA
                            self._person.appearance = aa * feat + (1 - aa) * self._person.appearance
                        else:
                            self._person.appearance = feat
                except Exception:
                    pass

    def _init_person(self, obj: dict) -> None:
        """初始化目标 (未经 VLM 选定时的退化路径)。"""
        pos = self._get_pos(obj)
        self._person = TrackedPerson(
            position=list(pos[:3]),
            last_seen=time.time(),
            confidence=obj.get("confidence", 1.0),
            obj_id=obj.get("id"),
            bbox=obj.get("bbox"),
        )

    @staticmethod
    def _get_pos(obj: dict) -> list[float]:
        """统一提取 position，兼容 list 和 dict 格式。"""
        pos = obj.get("position", [0, 0, 0])
        if isinstance(pos, dict):
            return [pos.get("x", 0), pos.get("y", 0), pos.get("z", 0)]
        return list(pos[:3]) if len(pos) >= 3 else list(pos) + [0.0] * (3 - len(pos))

    @staticmethod
    def _crop_person(rgb: np.ndarray, obj: dict) -> np.ndarray | None:
        """从 RGB 帧裁剪 person 区域。"""
        bbox = obj.get("bbox")
        if bbox is None:
            return None
        if isinstance(bbox, np.ndarray):
            bbox = bbox.astype(int).tolist()
        elif isinstance(bbox, dict):
            bbox = [int(bbox.get("x1", 0)), int(bbox.get("y1", 0)),
                    int(bbox.get("x2", 0)), int(bbox.get("y2", 0))]

        h, w = rgb.shape[:2]
        x1 = max(0, int(bbox[0]))
        y1 = max(0, int(bbox[1]))
        x2 = min(w, int(bbox[2]))
        y2 = min(h, int(bbox[3]))
        if x2 - x1 < 10 or y2 - y1 < 10:
            return None
        return rgb[y1:y2, x1:x2].copy()

    # ── 导航接口 (保持与旧版兼容) ──

    def get_follow_waypoint(
        self, robot_pos: list[float], predict_dt: float = 0.3,
    ) -> dict | None:
        """返回跟随航点 (目标后方 follow_distance 处)。"""
        with self._lock:
            if self._person is None or self.is_lost():
                return None

            px = self._person.position[0] + self._person.velocity[0] * predict_dt
            py = self._person.position[1] + self._person.velocity[1] * predict_dt
            pz = self._person.position[2] if len(self._person.position) > 2 else 0.0

            dx = robot_pos[0] - px
            dy = robot_pos[1] - py
            dist = math.hypot(dx, dy)
            if dist < 0.01:
                return {"x": px, "y": py, "z": pz}

            fx = px + (dx / dist) * self.follow_distance
            fy = py + (dy / dist) * self.follow_distance
            return {"x": fx, "y": fy, "z": pz}

    def is_lost(self) -> bool:
        if self._person is None:
            return True
        return (time.time() - self._person.last_seen) > self.lost_timeout

    def needs_vlm_reselect(self) -> bool:
        """是否需要 VLM 重新选人 (丢失超过 REID_TIMEOUT)。"""
        if self._following_selector is not None:
            return self._following_selector.needs_reselect
        if self._person is None:
            return True
        elapsed = time.time() - self._person.last_seen
        return elapsed > self.REID_TIMEOUT

    def get_person_position(self) -> list[float] | None:
        with self._lock:
            if self._person and not self.is_lost():
                return list(self._person.position)
        return None

    @property
    def target_selected(self) -> bool:
        return self._target_selected

    @property
    def description(self) -> str:
        return self._description

    def reset(self):
        with self._lock:
            self._person = None
            self._description = ""
            self._target_selected = False
            self._vlm_selecting = False
            self._target_track_id = None
            if self._following_selector is not None:
                self._following_selector.unlock()
