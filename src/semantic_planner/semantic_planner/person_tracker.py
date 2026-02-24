"""
PersonTracker -- 实时人体追踪模块 (跟人走功能核心)

不依赖卡尔曼滤波库, 使用简单的 EMA 位置平滑 + 速度估计。
"""
from dataclasses import dataclass, field
from typing import Optional, List, Dict
import time
import math


@dataclass
class TrackedPerson:
    position: List[float]          # [x, y, z]
    velocity: List[float] = field(default_factory=lambda: [0.0, 0.0])  # [vx, vy] m/s
    last_seen: float = field(default_factory=time.time)
    confidence: float = 1.0


class PersonTracker:
    LOST_TIMEOUT = 3.0      # 秒: 超时判定丢失
    EMA_ALPHA = 0.4         # 位置平滑系数
    FOLLOW_DISTANCE = 1.5   # 跟随距离 (米)
    MIN_DISTANCE = 0.8      # 最近允许距离

    def __init__(self, follow_distance: float = 1.5, lost_timeout: float = 3.0):
        self.follow_distance = follow_distance
        self.lost_timeout = lost_timeout
        self._person: Optional[TrackedPerson] = None

    def update(self, scene_objects: List[Dict]) -> bool:
        """
        从场景图 objects 中找 person, 更新追踪状态。
        返回 True 如果找到 person。
        scene_objects: List of {"label": str, "position": [x,y,z], "confidence": float}
        """
        persons = [
            o for o in scene_objects
            if o.get("label", "").lower() in ("person", "people", "human", "pedestrian")
        ]
        if not persons:
            return False

        # 选最高置信度的
        best = max(persons, key=lambda p: p.get("confidence", 0))
        new_pos = best.get("position", [0, 0, 0])
        now = time.time()

        if self._person is None:
            self._person = TrackedPerson(
                position=list(new_pos[:3]),
                last_seen=now,
                confidence=best.get("confidence", 1.0),
            )
        else:
            # EMA 位置平滑
            dt = now - self._person.last_seen
            old = self._person.position
            # 速度估计
            if dt > 0.01:
                self._person.velocity = [
                    (new_pos[0] - old[0]) / dt * 0.3 + self._person.velocity[0] * 0.7,
                    (new_pos[1] - old[1]) / dt * 0.3 + self._person.velocity[1] * 0.7,
                ]
            a = self.EMA_ALPHA
            self._person.position = [
                a * new_pos[0] + (1 - a) * old[0],
                a * new_pos[1] + (1 - a) * old[1],
                a * new_pos[2] + (1 - a) * old[2],
            ]
            self._person.last_seen = now
            self._person.confidence = best.get("confidence", 1.0)
        return True

    def get_follow_waypoint(
        self, robot_pos: List[float], predict_dt: float = 0.3
    ) -> Optional[Dict]:
        """
        返回机器人应该导航到的点 (目标后方 follow_distance 处)。
        predict_dt: 预测目标未来位置的时间窗 (秒)
        """
        if self._person is None or self.is_lost():
            return None

        # 预测目标位置
        px = self._person.position[0] + self._person.velocity[0] * predict_dt
        py = self._person.position[1] + self._person.velocity[1] * predict_dt
        pz = self._person.position[2] if len(self._person.position) > 2 else 0.0

        # 计算目标 -> 机器人方向, 在目标后方 follow_distance 处
        dx = robot_pos[0] - px
        dy = robot_pos[1] - py
        dist = math.hypot(dx, dy)
        if dist < 0.01:
            return {"x": px, "y": py, "z": pz}

        # 跟随点 = 目标位置 + 朝向机器人方向 follow_distance
        fx = px + (dx / dist) * self.follow_distance
        fy = py + (dy / dist) * self.follow_distance
        return {"x": fx, "y": fy, "z": pz}

    def is_lost(self) -> bool:
        if self._person is None:
            return True
        return (time.time() - self._person.last_seen) > self.lost_timeout

    def get_person_position(self) -> Optional[List[float]]:
        if self._person and not self.is_lost():
            return list(self._person.position)
        return None

    def reset(self):
        self._person = None
