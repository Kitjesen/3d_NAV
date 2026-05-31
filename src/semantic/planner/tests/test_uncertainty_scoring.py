"""语义不确定性驱动 frontier 评分测试。

验证:
1. 高 entropy 方向的 frontier 得分更高
2. set_room_type_posteriors_from_json 正确解析
3. 无后验时 uncertainty_score 为 0
"""

import os
import sys

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
for _pkg in ("semantic_planner",):
    _p = os.path.join(_repo, _pkg)
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import pytest

from semantic.planner.semantic_planner.frontier_scorer import Frontier, FrontierScorer


def _make_scorer(**kwargs) -> FrontierScorer:
    return FrontierScorer(**kwargs)


def _make_frontier(fid: int, x: float, y: float) -> Frontier:
    return Frontier(frontier_id=fid, center_world=np.array([x, y]), size=10)


class TestUncertaintyScoring:
    """测试 _compute_uncertainty_score 和 semantic_uncertainty_weight。"""

    def test_high_entropy_scores_higher(self):
        """高 entropy 方向的 frontier 应该得分更高。"""
        scorer = _make_scorer()
        scorer.set_room_type_posteriors_from_json({
            "0": {
                "entropy": 1.5,
                "best_type": "unknown",
                "confidence": 0.3,
                "top3": [{"kitchen": 0.35}, {"office": 0.33}, {"bedroom": 0.32}],
            },
            "1": {
                "entropy": 0.1,
                "best_type": "corridor",
                "confidence": 0.95,
                "top3": [{"corridor": 0.95}, {"hallway": 0.03}, {"lobby": 0.02}],
            },
        })
        robot_pos = np.array([0.0, 0.0])
        scene_rooms = [
            {"room_id": 0, "center": {"x": 5.0, "y": 0.0}},
            {"room_id": 1, "center": {"x": -5.0, "y": 0.0}},
        ]

        # Frontier pointing towards high entropy room
        f_high = _make_frontier(0, 5.0, 0.0)
        score_high = scorer._compute_uncertainty_score(f_high, robot_pos, scene_rooms)

        # Frontier pointing towards low entropy room
        f_low = _make_frontier(1, -5.0, 0.0)
        score_low = scorer._compute_uncertainty_score(f_low, robot_pos, scene_rooms)

        assert score_high > score_low
        assert score_high > 0.3
        assert score_low < 0.2

    def test_no_posteriors_returns_zero(self):
        """没有后验时 uncertainty score 应为 0。"""
        scorer = _make_scorer()
        frontier = _make_frontier(0, 5.0, 0.0)
        robot_pos = np.array([0.0, 0.0])
        scene_rooms = [{"room_id": 0, "center": {"x": 5.0, "y": 0.0}}]

        score = scorer._compute_uncertainty_score(frontier, robot_pos, scene_rooms)
        assert score == 0.0

    def test_posteriors_from_json_parsing(self):
        """set_room_type_posteriors_from_json 应正确解析 JSON 格式。"""
        scorer = _make_scorer()
        json_data = {
            "0": {
                "entropy": 1.2,
                "best_type": "kitchen",
                "confidence": 0.5,
                "top3": [{"kitchen": 0.5}, {"office": 0.3}, {"bedroom": 0.2}],
            },
            "5": {
                "entropy": 0.0,
                "best_type": "corridor",
                "confidence": 1.0,
                "top3": [{"corridor": 1.0}],
            },
        }
        scorer.set_room_type_posteriors_from_json(json_data)
        assert 0 in scorer._room_type_posteriors
        assert 5 in scorer._room_type_posteriors
        assert scorer._room_type_posteriors[0].entropy == 1.2
        assert scorer._room_type_posteriors[5].entropy == 0.0
        assert "kitchen" in scorer._room_type_posteriors[0].hypotheses

    def test_default_weights(self):
        """默认权重应为 0.15。"""
        scorer = _make_scorer()
        assert scorer.semantic_prior_weight == 0.15
        assert scorer.semantic_uncertainty_weight == 0.15

    def test_uncertainty_weight_in_total(self):
        """semantic_uncertainty_weight 应参与总权重计算。"""
        scorer = _make_scorer(
            distance_weight=0.2,
            novelty_weight=0.3,
            language_weight=0.2,
            grounding_weight=0.3,
            semantic_uncertainty_weight=0.15,
        )
        total = (scorer.distance_weight + scorer.novelty_weight
                 + scorer.language_weight + scorer.grounding_weight
                 + scorer.vision_weight + scorer.semantic_prior_weight
                 + scorer.semantic_uncertainty_weight)
        assert total > 1.0  # 新权重加入后总和 > 1.0 (会被归一化)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
