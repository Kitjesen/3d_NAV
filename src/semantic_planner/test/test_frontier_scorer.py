"""
test_frontier_scorer.py — Frontier 评分器单元测试

覆盖:
  - Costmap 更新
  - Frontier 提取
  - 评分 (距离、新颖度、语言、grounding_potential)
  - 辅助方法
"""

import math
import unittest

import numpy as np

from semantic_planner.frontier_scorer import (
    FREE_CELL,
    OCCUPIED_CELL,
    UNKNOWN_CELL,
    Frontier,
    FrontierScorer,
)


class TestCostmapUpdate(unittest.TestCase):
    """Costmap 更新测试。"""

    def test_update_stores_params(self):
        scorer = FrontierScorer()
        grid = np.zeros((20, 20), dtype=np.int8)
        scorer.update_costmap(grid, resolution=0.1, origin_x=-1.0, origin_y=-1.0)
        self.assertIsNotNone(scorer._grid)
        self.assertAlmostEqual(scorer._resolution, 0.1)


class TestFrontierExtraction(unittest.TestCase):
    """Frontier 提取测试。"""

    def _make_grid(self, rows=20, cols=20):
        """创建一个上半部 free, 下半部 unknown 的栅格。"""
        grid = np.full((rows, cols), UNKNOWN_CELL, dtype=np.int8)
        grid[:rows // 2, :] = FREE_CELL
        return grid

    def test_extract_finds_frontiers(self):
        scorer = FrontierScorer(min_frontier_size=3)
        grid = self._make_grid(20, 20)
        scorer.update_costmap(grid, resolution=0.1, origin_x=0.0, origin_y=0.0)
        frontiers = scorer.extract_frontiers(np.array([0.5, 0.5]))
        self.assertGreater(len(frontiers), 0)
        # frontier 中心应在 free/unknown 交界处
        for f in frontiers:
            self.assertGreater(f.size, 0)
            self.assertGreater(f.distance, 0)

    def test_all_free_no_frontiers(self):
        scorer = FrontierScorer(min_frontier_size=3)
        grid = np.full((20, 20), FREE_CELL, dtype=np.int8)
        scorer.update_costmap(grid, resolution=0.1, origin_x=0.0, origin_y=0.0)
        frontiers = scorer.extract_frontiers(np.array([0.5, 0.5]))
        self.assertEqual(len(frontiers), 0)

    def test_all_unknown_no_frontiers(self):
        scorer = FrontierScorer(min_frontier_size=3)
        grid = np.full((20, 20), UNKNOWN_CELL, dtype=np.int8)
        scorer.update_costmap(grid, resolution=0.1, origin_x=0.0, origin_y=0.0)
        frontiers = scorer.extract_frontiers(np.array([0.5, 0.5]))
        self.assertEqual(len(frontiers), 0)

    def test_no_grid_returns_empty(self):
        scorer = FrontierScorer()
        frontiers = scorer.extract_frontiers(np.array([0.0, 0.0]))
        self.assertEqual(len(frontiers), 0)


class TestFrontierScoring(unittest.TestCase):
    """Frontier 评分测试。"""

    def _make_scorer_with_frontiers(self):
        scorer = FrontierScorer(min_frontier_size=3)
        grid = np.full((30, 30), UNKNOWN_CELL, dtype=np.int8)
        grid[:15, :] = FREE_CELL
        scorer.update_costmap(grid, resolution=0.1, origin_x=0.0, origin_y=0.0)
        scorer.extract_frontiers(np.array([0.5, 0.5]))
        return scorer

    def test_scoring_assigns_nonzero_scores(self):
        scorer = self._make_scorer_with_frontiers()
        if not scorer._frontiers:
            self.skipTest("No frontiers extracted")
        scored = scorer.score_frontiers(
            instruction="find the door",
            robot_position=np.array([0.5, 0.5]),
        )
        for f in scored:
            self.assertGreater(f.score, 0)

    def test_language_relevance_boosts_score(self):
        scorer = self._make_scorer_with_frontiers()
        if not scorer._frontiers:
            self.skipTest("No frontiers extracted")

        # 无物体
        scored_no_obj = scorer.score_frontiers(
            instruction="find door",
            robot_position=np.array([0.5, 0.5]),
        )
        max_score_no_obj = max(f.score for f in scored_no_obj)

        # 在 frontier 附近有 "door" 物体
        door_pos = scored_no_obj[0].center_world
        scene_objects = [{
            "id": 0,
            "label": "door",
            "position": {"x": float(door_pos[0]), "y": float(door_pos[1])},
        }]
        scored_with_obj = scorer.score_frontiers(
            instruction="find door",
            robot_position=np.array([0.5, 0.5]),
            scene_objects=scene_objects,
        )
        max_score_with_obj = max(f.score for f in scored_with_obj)
        self.assertGreaterEqual(max_score_with_obj, max_score_no_obj)

    def test_get_best_frontier(self):
        scorer = self._make_scorer_with_frontiers()
        if not scorer._frontiers:
            self.skipTest("No frontiers extracted")
        scorer.score_frontiers("test", np.array([0.5, 0.5]))
        best = scorer.get_best_frontier()
        self.assertIsNotNone(best)


class TestHelperMethods(unittest.TestCase):
    """辅助方法测试。"""

    def test_angle_diff(self):
        self.assertAlmostEqual(FrontierScorer._angle_diff(0, 0), 0)
        self.assertAlmostEqual(
            FrontierScorer._angle_diff(math.pi, -math.pi), 0, places=5
        )
        diff = FrontierScorer._angle_diff(0.1, -0.1)
        self.assertAlmostEqual(diff, 0.2, places=5)

    def test_angle_to_label(self):
        self.assertEqual(FrontierScorer._angle_to_label(0), "east")
        self.assertEqual(FrontierScorer._angle_to_label(math.pi / 2), "north")

    def test_cooccurrence_score(self):
        score = FrontierScorer._cooccurrence_score("find fire extinguisher", "door")
        self.assertGreater(score, 0)
        score_none = FrontierScorer._cooccurrence_score("find cat", "door")
        self.assertEqual(score_none, 0.0)

    def test_frontiers_summary_json(self):
        import json
        scorer = FrontierScorer(min_frontier_size=3)
        grid = np.full((20, 20), UNKNOWN_CELL, dtype=np.int8)
        grid[:10, :] = FREE_CELL
        scorer.update_costmap(grid, resolution=0.1, origin_x=0.0, origin_y=0.0)
        scorer.extract_frontiers(np.array([0.5, 0.5]))
        summary_str = scorer.get_frontiers_summary()
        summary = json.loads(summary_str)
        self.assertIn("frontier_count", summary)
        self.assertIn("frontiers", summary)


if __name__ == "__main__":
    unittest.main()
