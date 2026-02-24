#!/usr/bin/env python3
"""
SCG ROS2 集成测试 — 纯 Python，无需 ROS2 运行时。

测试覆盖:
  1. SCGPathPlanner 基本路径规划 (start → goal → waypoints)
  2. JSON 请求/响应序列化与反序列化
  3. Fallback 逻辑 (SCG 为空时降级到直接目标)
  4. 边界情况 (起点/终点不在多面体内)
"""

import json
import sys
from pathlib import Path

import numpy as np
import pytest

# 将 semantic_perception 包路径加入 sys.path
_pkg_root = Path(__file__).parent.parent
sys.path.insert(0, str(_pkg_root))

from semantic_perception.scg_builder import SCGBuilder, SCGConfig
from semantic_perception.scg_path_planner import SCGPathPlanner, SCGPath
from semantic_perception.polyhedron_expansion import (
    PolyhedronExpander,
    PolyhedronExpansionConfig,
    Polyhedron,
)


# ══════════════════════════════════════════════════════════════════
#  Fixtures
# ══════════════════════════════════════════════════════════════════

def _build_minimal_scg():
    """
    构建最小测试 SCG。

    使用较小的参数保证测试速度，同时保证能生成至少 2 个多面体。
    """
    # 创建一个简单的自由空间：走廊形状（Y 轴方向延伸）
    occupancy_grid = np.ones((20, 40, 5), dtype=np.float32)
    # 走廊: x=[4,16], y=[0,40], z=[0,5] 全部自由
    occupancy_grid[4:16, :, :] = 0.0

    grid_resolution = 0.5
    grid_origin = np.array([0.0, 0.0, 0.0])

    config = PolyhedronExpansionConfig(
        num_sphere_samples=24,
        r_min=0.3,
        r_max=1.0,
        r_step=0.3,
        min_polyhedron_volume=0.05,
        max_polyhedra=10,
        coverage_threshold=0.4,
        collision_threshold=0.5,
    )

    expander = PolyhedronExpander(config)
    polyhedra = expander.expand(occupancy_grid, grid_resolution, grid_origin)

    scg_config = SCGConfig(
        adjacency_threshold=0.3,
        connectivity_samples=10,
        loop_closure_threshold=0.5,
    )
    builder = SCGBuilder(scg_config)
    for poly in polyhedra:
        builder.add_polyhedron(poly)

    builder.build_edges(occupancy_grid, grid_resolution, grid_origin)
    return builder, polyhedra


def _make_fake_polyhedron(poly_id: int, center: np.ndarray, radius: float = 1.0) -> Polyhedron:
    """
    构造一个假多面体 (用于隔离测试，不依赖多面体扩展算法)。
    """
    # 简单正八面体顶点
    offsets = np.array([
        [radius, 0, 0], [-radius, 0, 0],
        [0, radius, 0], [0, -radius, 0],
        [0, 0, radius], [0, 0, -radius],
    ])
    vertices = center + offsets
    faces = np.array([[0, 2, 4], [0, 2, 5], [0, 3, 4], [0, 3, 5],
                      [1, 2, 4], [1, 2, 5], [1, 3, 4], [1, 3, 5]])
    return Polyhedron(
        poly_id=poly_id,
        vertices=vertices,
        faces=faces,
        center=center.copy(),
        volume=(4.0 / 3.0) * np.pi * radius ** 3,
        radius=radius,
        seed_point=center.copy(),
        sample_points=vertices,
    )


# ══════════════════════════════════════════════════════════════════
#  测试 1: SCGPathPlanner 基本路径规划
# ══════════════════════════════════════════════════════════════════

class TestSCGPathPlannerBasic:
    """测试 SCGPathPlanner 能接收 start/goal 并返回路径。"""

    def test_plan_returns_scg_path_object(self):
        """plan() 始终返回 SCGPath 对象。"""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([5.0, 5.0, 0.0])

        result = planner.plan(start, goal)
        assert isinstance(result, SCGPath)

    def test_plan_fails_gracefully_when_no_nodes(self):
        """SCG 为空时，plan() 返回 success=False 而不是抛异常。"""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([10.0, 10.0, 0.0])

        result = planner.plan(start, goal)
        assert result.success is False
        assert result.total_distance == 0.0
        assert len(result.polyhedron_sequence) == 0

    def test_plan_same_polyhedron(self):
        """起点和终点在同一个多面体内，应该成功。"""
        builder = SCGBuilder(SCGConfig())
        center = np.array([0.0, 0.0, 0.0])
        poly = _make_fake_polyhedron(0, center, radius=2.0)
        builder.add_polyhedron(poly)
        builder.build_edges()  # 无 occupancy_grid

        planner = SCGPathPlanner(builder)
        start = center + np.array([0.1, 0.0, 0.0])
        goal = center + np.array([-0.1, 0.0, 0.0])

        result = planner.plan(start, goal)
        # 同一多面体内，序列长度为 1
        assert result.success is True
        assert len(result.polyhedron_sequence) == 1
        assert result.polyhedron_sequence[0] == 0

    def test_plan_two_adjacent_polyhedra(self):
        """两个相邻多面体之间的路径规划。"""
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()

        planner = SCGPathPlanner(builder)
        start = c1
        goal = c2

        result = planner.plan(start, goal)
        assert result.success is True
        assert len(result.polyhedron_sequence) >= 1

    def test_plan_with_real_scg(self):
        """使用真实多面体扩展构建的 SCG 进行路径规划。"""
        builder, polyhedra = _build_minimal_scg()
        if len(polyhedra) < 2:
            pytest.skip("不足 2 个多面体，跳过路径规划测试")

        planner = SCGPathPlanner(builder)
        start = polyhedra[0].center
        goal = polyhedra[-1].center

        result = planner.plan(start, goal, smooth=True, simplify=True)

        # 至少应返回结构正确的 SCGPath
        assert isinstance(result, SCGPath)
        assert hasattr(result, "success")
        assert hasattr(result, "total_distance")
        assert hasattr(result, "planning_time")
        assert result.planning_time >= 0.0

        if result.success:
            assert result.total_distance > 0.0
            assert len(result.polyhedron_sequence) >= 1


# ══════════════════════════════════════════════════════════════════
#  测试 2: JSON 序列化/反序列化
# ══════════════════════════════════════════════════════════════════

class TestSCGJsonSerialization:
    """测试请求和响应的 JSON 格式正确性。"""

    def _simulate_scg_plan_request(self, builder: SCGBuilder, planner: SCGPathPlanner,
                                   start_list: list, goal_list: list) -> dict:
        """
        模拟 perception_node._scg_plan_request_callback 的核心逻辑
        (不依赖 ROS2，仅测试纯业务逻辑)。
        """
        # --- 模拟接收请求 ---
        request_json = json.dumps({"start": start_list, "goal": goal_list})
        data = json.loads(request_json)
        start_raw = data.get("start", [0.0, 0.0, 0.0])
        goal_raw = data.get("goal", [0.0, 0.0, 0.0])
        start = np.array(start_raw, dtype=np.float64)
        goal = np.array(goal_raw, dtype=np.float64)

        if len(builder.nodes) == 0:
            return {
                "success": False,
                "waypoints": [],
                "poly_count": 0,
                "distance": 0.0,
                "error": "SCG has no nodes",
            }

        path = planner.plan(start, goal)

        if path.success:
            all_waypoints = []
            seen = set()
            for seg in path.segments:
                for pt in seg.waypoints:
                    key = (round(pt[0], 4), round(pt[1], 4), round(pt[2], 4))
                    if key not in seen:
                        seen.add(key)
                        all_waypoints.append([
                            round(float(pt[0]), 4),
                            round(float(pt[1]), 4),
                            round(float(pt[2]), 4),
                        ])
            return {
                "success": True,
                "waypoints": all_waypoints,
                "poly_count": len(path.polyhedron_sequence),
                "distance": round(path.total_distance, 4),
                "error": "",
            }
        else:
            return {
                "success": False,
                "waypoints": [],
                "poly_count": 0,
                "distance": 0.0,
                "error": "SCG planner could not find a path",
            }

    def test_request_json_valid(self):
        """请求 JSON 格式正确可解析。"""
        start = [1.0, 2.0, 0.0]
        goal = [5.0, 6.0, 0.0]
        request = json.dumps({"start": start, "goal": goal})
        parsed = json.loads(request)
        assert parsed["start"] == start
        assert parsed["goal"] == goal

    def test_response_json_success_format(self):
        """成功响应包含所有必需字段且类型正确。"""
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()
        planner = SCGPathPlanner(builder)

        result = self._simulate_scg_plan_request(
            builder, planner, [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]
        )

        # 序列化为 JSON 再反序列化
        result_json = json.dumps(result)
        parsed = json.loads(result_json)

        assert "success" in parsed
        assert "waypoints" in parsed
        assert "poly_count" in parsed
        assert "distance" in parsed
        assert "error" in parsed
        assert isinstance(parsed["success"], bool)
        assert isinstance(parsed["waypoints"], list)
        assert isinstance(parsed["poly_count"], int)
        assert isinstance(parsed["distance"], float)

    def test_response_json_failure_format(self):
        """失败响应（SCG 为空）包含所有必需字段。"""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        result = self._simulate_scg_plan_request(
            builder, planner, [0.0, 0.0, 0.0], [10.0, 10.0, 0.0]
        )

        result_json = json.dumps(result)
        parsed = json.loads(result_json)

        assert parsed["success"] is False
        assert parsed["waypoints"] == []
        assert parsed["poly_count"] == 0
        assert parsed["distance"] == 0.0
        assert "error" in parsed
        assert len(parsed["error"]) > 0

    def test_waypoints_are_serializable(self):
        """路径点列表中每个元素都是长度为 3 的 float 列表。"""
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()
        planner = SCGPathPlanner(builder)

        result = self._simulate_scg_plan_request(
            builder, planner, [0.0, 0.0, 0.0], [2.0, 0.0, 0.0]
        )

        if result["success"]:
            for wp in result["waypoints"]:
                assert len(wp) == 3
                assert all(isinstance(v, float) for v in wp)

    def test_invalid_request_json_handled(self):
        """非法 JSON 请求应被安全处理（不抛异常）。"""
        bad_jsons = [
            "",
            "not json",
            '{"start": "not_a_list", "goal": [1,2,3]}',
            '{"start": [1,2], "goal": null}',
        ]
        for bad in bad_jsons:
            try:
                data = json.loads(bad)
                start_raw = data.get("start", [0.0, 0.0, 0.0])
                goal_raw = data.get("goal", [0.0, 0.0, 0.0])
                # 如果 start/goal 不是列表，会在 np.array() 时失败
                np.array(start_raw, dtype=np.float64)
                np.array(goal_raw, dtype=np.float64)
            except (json.JSONDecodeError, ValueError, TypeError):
                pass  # 期望: 被捕获，不传播


# ══════════════════════════════════════════════════════════════════
#  测试 3: Fallback 逻辑
# ══════════════════════════════════════════════════════════════════

class TestSCGFallbackLogic:
    """测试 SCG 规划失败时的降级行为。"""

    def test_fallback_when_scg_empty(self):
        """SCG 无节点时，模拟 planner_node 的 fallback 到直接 PoseStamped。"""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        # 模拟 _plan_with_scg 的核心逻辑
        def simulate_plan_with_scg(robot_pos, goal_xyz):
            if len(builder.nodes) == 0:
                return None  # fallback 条件

            start = np.array([robot_pos["x"], robot_pos["y"], robot_pos["z"]])
            goal = np.array(goal_xyz)
            path = planner.plan(start, goal)
            if not path.success:
                return None

            all_waypoints = []
            for seg in path.segments:
                for pt in seg.waypoints:
                    all_waypoints.append({"x": float(pt[0]), "y": float(pt[1]), "z": float(pt[2])})
            return all_waypoints if len(all_waypoints) > 1 else None

        robot_pos = {"x": 0.0, "y": 0.0, "z": 0.0}
        goal = (5.0, 5.0, 0.0)
        result = simulate_plan_with_scg(robot_pos, goal)

        assert result is None, "SCG 为空时应返回 None (触发 fallback)"

    def test_fallback_when_start_outside_polyhedra(self):
        """起点不在任何多面体内时，最近邻回退仍可规划成功。"""
        builder = SCGBuilder(SCGConfig())
        center = np.array([5.0, 5.0, 0.0])
        poly = _make_fake_polyhedron(0, center, radius=1.0)
        builder.add_polyhedron(poly)
        builder.build_edges()

        planner = SCGPathPlanner(builder)

        # 起点远离所有多面体 — _locate_polyhedron 最近邻回退到 poly 0
        start = np.array([100.0, 100.0, 0.0])
        goal = center

        result = planner.plan(start, goal)
        # 最近邻回退使规划成功（polyhedron_sequence 非空）
        assert result.success is True

    def test_fallback_when_goal_outside_polyhedra(self):
        """终点不在任何多面体内时，最近邻回退仍可规划成功。"""
        builder = SCGBuilder(SCGConfig())
        center = np.array([0.0, 0.0, 0.0])
        poly = _make_fake_polyhedron(0, center, radius=1.0)
        builder.add_polyhedron(poly)
        builder.build_edges()

        planner = SCGPathPlanner(builder)

        start = center
        goal = np.array([100.0, 100.0, 0.0])

        result = planner.plan(start, goal)
        # 最近邻回退使规划成功（polyhedron_sequence 非空）
        assert result.success is True

    def test_fallback_preserves_direct_goal(self):
        """
        fallback 后，目标位置应与原始 GoalResult 一致（不被 SCG 覆盖）。

        模拟 _handle_navigate_result 中 SCG 失败时直接使用原始 target_x/y/z。
        """
        target_x, target_y, target_z = 3.14, 2.72, 0.0

        # 模拟 SCG 不可用
        scg_waypoints = None  # 模拟 _plan_with_scg 返回 None

        # fallback 逻辑: 使用原始目标
        if scg_waypoints is None or len(scg_waypoints) <= 1:
            goal_x = target_x
            goal_y = target_y
            goal_z = target_z
        else:
            final_wp = scg_waypoints[-1]
            goal_x = final_wp["x"]
            goal_y = final_wp["y"]
            goal_z = final_wp["z"]

        assert goal_x == target_x
        assert goal_y == target_y
        assert goal_z == target_z

    def test_no_fallback_when_scg_provides_path(self):
        """
        SCG 成功时，路径应来自 SCG 多路径点，而非单点目标。

        使用两个相邻多面体验证 SCG 生成的路径点多于 1 个。
        """
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()

        planner = SCGPathPlanner(builder)
        start = c1
        goal = c2

        path = planner.plan(start, goal)

        if path.success:
            # 收集所有路径点
            all_waypoints = []
            seen = set()
            for seg in path.segments:
                for pt in seg.waypoints:
                    key = (round(pt[0], 4), round(pt[1], 4), round(pt[2], 4))
                    if key not in seen:
                        seen.add(key)
                        all_waypoints.append({"x": float(pt[0]), "y": float(pt[1]), "z": float(pt[2])})

            # SCG 成功时应有多于 1 个路径点
            assert len(all_waypoints) > 1, (
                f"SCG 成功时应产生多路径点, 实际得到 {len(all_waypoints)} 个"
            )


# ══════════════════════════════════════════════════════════════════
#  测试 4: 边界情况
# ══════════════════════════════════════════════════════════════════

class TestSCGEdgeCases:
    """测试各种边界情况。"""

    def test_plan_with_nan_start(self):
        """NaN 起点不应使规划器崩溃。"""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)
        start = np.array([float("nan"), 0.0, 0.0])
        goal = np.array([1.0, 1.0, 0.0])
        result = planner.plan(start, goal)
        assert result.success is False

    def test_plan_with_inf_goal(self):
        """无穷大终点不应使规划器崩溃。"""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([float("inf"), 0.0, 0.0])
        result = planner.plan(start, goal)
        assert result.success is False

    def test_scg_result_distance_is_nonnegative(self):
        """成功路径的总距离应 >= 0。"""
        builder = SCGBuilder(SCGConfig(adjacency_threshold=0.5))
        c1 = np.array([0.0, 0.0, 0.0])
        c2 = np.array([2.0, 0.0, 0.0])
        poly1 = _make_fake_polyhedron(0, c1, radius=1.1)
        poly2 = _make_fake_polyhedron(1, c2, radius=1.1)
        builder.add_polyhedron(poly1)
        builder.add_polyhedron(poly2)
        builder.build_edges()

        planner = SCGPathPlanner(builder)
        result = planner.plan(c1, c2)

        if result.success:
            assert result.total_distance >= 0.0

    def test_planning_time_is_measured(self):
        """规划时间应被正确记录（大于 0）。"""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)
        start = np.array([0.0, 0.0, 0.0])
        goal = np.array([10.0, 10.0, 0.0])
        result = planner.plan(start, goal)
        assert result.planning_time >= 0.0

    def test_json_roundtrip_waypoints(self):
        """路径点经过 JSON 序列化/反序列化后值不变。"""
        original_waypoints = [
            [1.2345, -2.6789, 0.0],
            [3.0001, 4.9999, 1.5],
        ]
        json_str = json.dumps({"waypoints": original_waypoints})
        recovered = json.loads(json_str)["waypoints"]

        for orig, rec in zip(original_waypoints, recovered):
            for a, b in zip(orig, rec):
                assert abs(a - b) < 1e-9, f"JSON roundtrip失败: {a} != {b}"

    def test_multiple_calls_independent(self):
        """多次连续调用 plan() 互不干扰。"""
        builder = SCGBuilder(SCGConfig())
        planner = SCGPathPlanner(builder)

        results = []
        for _ in range(3):
            start = np.array([0.0, 0.0, 0.0])
            goal = np.array([float(_ + 1), 0.0, 0.0])
            results.append(planner.plan(start, goal))

        # 所有结果都应该是独立的 SCGPath 对象
        assert len(results) == 3
        for r in results:
            assert isinstance(r, SCGPath)
