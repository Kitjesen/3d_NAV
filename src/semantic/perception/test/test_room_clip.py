"""OneMap Room 级 CLIP 特征聚合测试。

验证:
1. compute_rooms 正确聚合物体 CLIP 特征到 RoomNode
2. query_rooms_by_embedding 余弦相似度匹配
3. 无特征物体不影响聚合
"""

import os
import sys

# 确保 semantic_common 和 semantic_perception 可导入
_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _pkg in ("semantic_common", "semantic_perception"):
    _p = os.path.join(_repo, _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import pytest

from semantic_perception.instance_tracker import (
    InstanceTracker,
    RoomNode,
    Region,
    TrackedObject,
)


def _make_feature(seed: int, dim: int = 512) -> np.ndarray:
    """生成确定性伪随机特征向量 (L2 归一化)。"""
    rng = np.random.RandomState(seed)
    f = rng.randn(dim).astype(np.float64)
    return f / np.linalg.norm(f)


def _make_tracker_with_objects(objects_per_room: dict) -> InstanceTracker:
    """构建 tracker 并注入物体。

    Args:
        objects_per_room: {room_id: [(label, feature_or_None), ...]}
    """
    tracker = InstanceTracker()
    oid = 0
    for room_id, items in objects_per_room.items():
        for label, feat in items:
            obj = TrackedObject(
                object_id=oid,
                label=label,
                position=np.array([float(room_id), float(oid), 0.0]),
                best_score=0.9,
            )
            if feat is not None:
                obj.features = feat.copy()
            obj.region_id = room_id
            tracker._objects[oid] = obj
            oid += 1
    tracker._next_id = oid
    return tracker


def _make_regions(objects_per_room: dict) -> list:
    """从 objects_per_room 构建 Region 列表。"""
    regions = []
    oid = 0
    for room_id, items in objects_per_room.items():
        obj_ids = list(range(oid, oid + len(items)))
        regions.append(
            Region(
                region_id=room_id,
                name=f"room_{room_id}",
                center=np.array([float(room_id), 0.0]),
                object_ids=obj_ids,
            )
        )
        oid += len(items)
    return regions


class TestRoomClipAggregation:
    """测试 compute_rooms 的 CLIP 特征聚合。"""

    def test_room_clip_feature_aggregated(self):
        """物体有 CLIP 特征时, room 应该有聚合特征。"""
        f1 = _make_feature(1)
        f2 = _make_feature(2)
        objects_per_room = {
            0: [("chair", f1), ("table", f2)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)

        rooms = tracker.compute_rooms(regions, [])
        assert len(rooms) == 1
        room = rooms[0]

        assert room.clip_feature is not None
        assert room.feature_count == 2
        # 聚合特征应为 L2 归一化
        norm = np.linalg.norm(room.clip_feature)
        assert abs(norm - 1.0) < 1e-6

    def test_room_without_features(self):
        """物体没有 CLIP 特征时, room.clip_feature 应为 None。"""
        objects_per_room = {
            0: [("chair", None), ("table", None)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)

        rooms = tracker.compute_rooms(regions, [])
        assert rooms[0].clip_feature is None
        assert rooms[0].feature_count == 0

    def test_mixed_features(self):
        """部分物体有特征, 部分没有 — 只聚合有特征的。"""
        f1 = _make_feature(10)
        objects_per_room = {
            0: [("chair", f1), ("table", None), ("lamp", None)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)

        rooms = tracker.compute_rooms(regions, [])
        room = rooms[0]
        assert room.clip_feature is not None
        assert room.feature_count == 1
        # 只有一个特征时, 聚合结果应等于该特征
        cos_sim = float(np.dot(room.clip_feature, f1 / np.linalg.norm(f1)))
        assert cos_sim > 0.999


class TestQueryRoomsByEmbedding:
    """测试 query_rooms_by_embedding 余弦相似度查询。"""

    def test_query_returns_most_similar(self):
        """查询应返回与 embedding 最相似的 room。"""
        # 3 个 room, 每个有不同语义的物体
        f_kitchen_1 = _make_feature(100)
        f_kitchen_2 = _make_feature(101)
        f_office_1 = _make_feature(200)
        f_office_2 = _make_feature(201)
        f_bedroom_1 = _make_feature(300)

        objects_per_room = {
            0: [("refrigerator", f_kitchen_1), ("microwave", f_kitchen_2)],
            1: [("computer", f_office_1), ("desk", f_office_2)],
            2: [("bed", f_bedroom_1)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)
        tracker.compute_rooms(regions, [])

        # 用 kitchen 的第一个物体特征查询 — 应该匹配 room 0
        results = tracker.query_rooms_by_embedding(f_kitchen_1, top_k=3)
        assert len(results) == 3
        best_room, best_score = results[0]
        assert best_room.room_id == 0
        assert best_score > 0.5  # 应有较高相似度

    def test_query_top_k(self):
        """top_k 应限制返回数量。"""
        objects_per_room = {
            i: [("obj", _make_feature(i * 10))] for i in range(5)
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)
        tracker.compute_rooms(regions, [])

        results = tracker.query_rooms_by_embedding(_make_feature(0), top_k=2)
        assert len(results) == 2

    def test_query_empty_rooms(self):
        """没有 rooms 时应返回空列表。"""
        tracker = InstanceTracker()
        results = tracker.query_rooms_by_embedding(_make_feature(0))
        assert results == []

    def test_query_rooms_no_features(self):
        """所有 room 都没有 CLIP 特征时应返回空列表。"""
        objects_per_room = {
            0: [("chair", None)],
            1: [("table", None)],
        }
        tracker = _make_tracker_with_objects(objects_per_room)
        regions = _make_regions(objects_per_room)
        tracker.compute_rooms(regions, [])

        results = tracker.query_rooms_by_embedding(_make_feature(0))
        assert results == []


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
