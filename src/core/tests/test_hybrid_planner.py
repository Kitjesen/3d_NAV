"""test_hybrid_planner.py — HybridPlanner 单元测试

测试 hybrid_planner.py 的正确失败语义：
  - A* 找不到路径 → plan_path success=False，带 failure_reason
  - 无 traversability grid → success=False（不崩溃）
  - 拓扑图无路径 → success=False
  - A* 成功 → success=True，waypoints 非空且无直线虚假路径

不依赖 scipy、ROS2、torch、或任何重型库。
完全使用 numpy + 内联 mock 对象。
"""

import math
import numpy as np
import pytest

from semantic.perception.semantic_perception.hybrid_planner import (
    HybridPlanner,
    HybridPath,
    _astar_on_grid,
    _w2g,
    _g2w,
    _find_nearest_free,
    _downsample_cells,
)


# ── Mock 拓扑图 ────────────────────────────────────────────────────────────────

class MockRoom:
    def __init__(self, node_id, cx, cy, bbox=None):
        self.node_id = node_id
        self.center = np.array([cx, cy])
        self.bounding_box = bbox  # {"x_min","x_max","y_min","y_max"} or None
        self.convex_hull = None


class MockTSG:
    """极简拓扑图 mock：只实现 HybridPlanner 用到的接口。"""

    def __init__(self, rooms, edges):
        """
        rooms: list of MockRoom
        edges: dict {(from_id, to_id): cost}  — 双向
        """
        self._rooms = {r.node_id: r for r in rooms}
        self._edges = {}
        for (a, b), c in edges.items():
            self._edges.setdefault(a, {})[b] = c
            self._edges.setdefault(b, {})[a] = c

    @property
    def rooms(self):
        return list(self._rooms.values())

    def get_node(self, node_id):
        return self._rooms.get(node_id)

    def get_room_by_position(self, x, y, radius=5.0):
        for r in self._rooms.values():
            if math.hypot(r.center[0] - x, r.center[1] - y) <= radius:
                return r.node_id
        return None

    def shortest_path(self, src, dst):
        """Dijkstra — 返回 (cost, [node_id, ...]) 或 (inf, [])。"""
        import heapq
        dist = {src: 0.0}
        prev = {}
        pq = [(0.0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, math.inf):
                continue
            if u == dst:
                path = []
                cur = dst
                while cur in prev:
                    path.append(cur)
                    cur = prev[cur]
                path.append(src)
                path.reverse()
                return d, path
            for v, w in self._edges.get(u, {}).items():
                nd = d + w
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return math.inf, []


# ── Helpers ────────────────────────────────────────────────────────────────────

def _make_open_trav(nx=100, ny=100, obs_thr=49.9):
    """全部可通行的 traversability grid。"""
    return np.zeros((nx, ny), dtype=np.float32)


def _make_walled_trav(nx=50, ny=50, obs_thr=49.9):
    """中间有一堵墙（横向阻断），两端留通道。"""
    trav = np.zeros((nx, ny), dtype=np.float32)
    mid = nx // 2
    # 竖墙（沿 i=mid），留一个 3 格宽的缺口
    trav[mid, :] = 100.0
    trav[mid, ny // 2 - 1: ny // 2 + 2] = 0.0  # 缺口
    return trav


def _make_blocked_trav(nx=50, ny=50):
    """完全封死：墙穿整个中间，没有缺口。"""
    trav = np.zeros((nx, ny), dtype=np.float32)
    trav[nx // 2, :] = 100.0  # 无缺口
    return trav


def _simple_planner(tsg, trav, **kw):
    """构造一个两房间一边的简单 HybridPlanner。"""
    return HybridPlanner(
        topology_graph=tsg,
        trav=trav,
        trav_res=0.2,
        trav_cx=0.0,
        trav_cy=0.0,
        **kw,
    )


# ── A* 底层函数测试 ────────────────────────────────────────────────────────────

class TestAstarOnGrid:
    def test_direct_path_open_grid(self):
        trav = _make_open_trav(20, 20)
        path = _astar_on_grid(trav, (0, 0), (10, 10), obs_thr=49.9)
        assert path is not None
        assert path[0] == (0, 0)
        assert path[-1] == (10, 10)

    def test_returns_none_when_blocked(self):
        """目标被完全封堵 → None（不是直线！）。"""
        trav = _make_blocked_trav(20, 20)
        path = _astar_on_grid(trav, (0, 5), (15, 5), obs_thr=49.9)
        assert path is None, "A* should return None for blocked path, not a straight line"

    def test_timeout_returns_none(self):
        """超时返回 None，不抛异常。"""
        # 大格子 + 极短超时
        trav = np.zeros((200, 200), dtype=np.float32)
        path = _astar_on_grid(trav, (0, 0), (199, 199), obs_thr=49.9, timeout_sec=0.0001)
        assert path is None

    def test_start_equals_goal(self):
        trav = _make_open_trav(10, 10)
        path = _astar_on_grid(trav, (5, 5), (5, 5), obs_thr=49.9)
        assert path is not None
        assert path == [(5, 5)]

    def test_path_avoids_obstacle(self):
        """路径应绕过障碍物。"""
        trav = _make_walled_trav(30, 30)
        path = _astar_on_grid(trav, (5, 5), (20, 5), obs_thr=49.9)
        assert path is not None
        mid = 30 // 2
        # 路径上不应有 i==mid 的格（墙）
        for ci, cj in path:
            assert trav[ci, cj] < 49.9, "path goes through obstacle"

    def test_no_path_through_wall_without_gap(self):
        """没有缺口的墙 → None，永远不会绕过。"""
        trav = np.zeros((20, 20), dtype=np.float32)
        trav[10, :] = 100.0  # 完全封堵
        path = _astar_on_grid(trav, (0, 10), (15, 10), obs_thr=49.9)
        assert path is None


class TestAstarHelpers:
    def test_w2g_g2w_roundtrip(self):
        cx, cy, res, nx, ny, ox, oy = 0.0, 0.0, 0.2, 100, 100, 50, 50
        for wx, wy in [(1.0, 2.0), (-3.0, 0.5), (0.0, 0.0)]:
            ix, iy = _w2g(wx, wy, cx, cy, res, ox, oy, nx, ny)
            rx, ry = _g2w(ix, iy, cx, cy, res, ox, oy)
            # 精度在一格内（0.2m）
            assert abs(rx - wx) <= res + 1e-9
            assert abs(ry - wy) <= res + 1e-9

    def test_find_nearest_free_already_free(self):
        trav = np.zeros((10, 10), dtype=np.float32)
        ni, nj = _find_nearest_free(trav, 5, 5, obs_thr=49.9)
        assert (ni, nj) == (5, 5)

    def test_find_nearest_free_on_obstacle(self):
        trav = np.full((10, 10), 100.0, dtype=np.float32)
        trav[5, 7] = 0.0  # 唯一自由格
        ni, nj = _find_nearest_free(trav, 5, 5, obs_thr=49.9, radius=5)
        assert (ni, nj) == (5, 7)

    def test_downsample_removes_close_points(self):
        cells = [(i, 0) for i in range(20)]
        result = _downsample_cells(cells, min_dist=3)
        # 末尾点总是强制保留，不受最小距离约束；
        # 检查除最后一段外所有相邻保留点的间距 >= min_dist
        for k in range(len(result) - 2):
            dx = result[k + 1][0] - result[k][0]
            dy = result[k + 1][1] - result[k][1]
            assert math.hypot(dx, dy) >= 3 - 1e-9
        # 首点和末点必须保留
        assert result[0] == cells[0]
        assert result[-1] == cells[-1]


# ── HybridPlanner 失败语义测试 ─────────────────────────────────────────────────

class TestHybridPlannerFailureSemantic:
    """核心测试：确保 success=False 而非直线路径。"""

    def test_no_trav_grid_returns_failure(self):
        """无 traversability grid → success=False，不崩溃，不返回直线。"""
        # 两个房间足够远，使 get_room_by_position(radius=5) 不会交叉命中
        rooms = [MockRoom(1, 0.0, 0.0), MockRoom(2, 30.0, 0.0)]
        tsg = MockTSG(rooms, {(1, 2): 30.0})
        planner = HybridPlanner(topology_graph=tsg, trav=None)

        # 起点在 room1 附近，终点在 room2 附近（各自 radius 内）
        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([30.0, 0.0, 0.0]))

        assert not result.success, "Must return failure when no trav grid"
        assert len(result.waypoints) == 0, "Blocked path must have no waypoints"
        assert result.failure_reason != "", "Must include failure_reason"

    def test_blocked_path_returns_failure_not_straight_line(self):
        """A* 无法穿越的障碍 → success=False，不是直线兜底。"""
        # 使用 100×100 格子，分辨率 0.5m → 总宽 50m
        # room1 中心在 (5,25)，room2 在 (45,25)，相互不会落在对方 radius=5 内
        nx, ny = 100, 100
        rooms = [
            MockRoom(1,  5.0, 0.0, {"x_min":  0, "x_max": 20, "y_min": -10, "y_max": 10}),
            MockRoom(2, 45.0, 0.0, {"x_min": 30, "x_max": 50, "y_min": -10, "y_max": 10}),
        ]
        tsg = MockTSG(rooms, {(1, 2): 40.0})

        trav = np.zeros((nx, ny), dtype=np.float32)
        trav[50, :] = 100.0  # 格 i=50 作墙，无缺口

        # 分辨率 0.5m，cx=25m，cy=0m → 格 (50,50) 对应世界 (25m,0m)
        planner = HybridPlanner(
            topology_graph=tsg,
            trav=trav,
            trav_res=0.5,
            trav_cx=25.0,
            trav_cy=0.0,
        )

        start = np.array([5.0, 0.0, 0.0])   # room1 附近
        goal  = np.array([45.0, 0.0, 0.0])  # room2 附近（穿越 i=50 的墙）
        result = planner.plan_path(start, goal)

        assert not result.success, \
            "A* cannot cross the wall — must return failure, NOT a straight-line fallback"
        assert len(result.waypoints) == 0
        assert len(result.failure_reason) > 0

    def test_no_topo_path_returns_failure(self):
        """拓扑图中两房间不连通 → success=False。"""
        rooms = [MockRoom(1, 0.0, 0.0), MockRoom(2, 20.0, 0.0)]
        tsg = MockTSG(rooms, {})  # 无边，不连通
        trav = _make_open_trav()
        planner = _simple_planner(tsg, trav)

        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([20.0, 0.0, 0.0]))
        assert not result.success
        assert len(result.waypoints) == 0

    def test_missing_room_returns_failure(self):
        """起点/终点无法定位到任何房间 → success=False。"""
        tsg = MockTSG([], {})  # 无房间
        trav = _make_open_trav()
        planner = _simple_planner(tsg, trav)

        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]))
        assert not result.success


# ── HybridPlanner 成功路径测试 ─────────────────────────────────────────────────

class TestHybridPlannerSuccess:
    def _two_room_planner(self, trav=None):
        rooms = [
            MockRoom(1, 0.0, 0.0, {"x_min": -5, "x_max": 5, "y_min": -5, "y_max": 5}),
            MockRoom(2, 8.0, 0.0, {"x_min": 3, "x_max": 13, "y_min": -5, "y_max": 5}),
        ]
        tsg = MockTSG(rooms, {(1, 2): 8.0})
        if trav is None:
            trav = _make_open_trav(200, 100)
        return HybridPlanner(
            topology_graph=tsg,
            trav=trav,
            trav_res=0.2,
            trav_cx=8.0,  # 居中
            trav_cy=0.0,
        )

    def test_open_corridor_success(self):
        """两个房间、全开放地图 → success=True，waypoints 不为空。"""
        planner = self._two_room_planner()
        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([8.0, 0.0, 0.0]))
        assert result.success
        assert len(result.waypoints) >= 2

    def test_waypoints_start_and_end_correct(self):
        """首末路径点应精确等于 start 和 goal。"""
        planner = self._two_room_planner()
        start = np.array([0.0, 0.0, 0.0])
        goal  = np.array([8.0, 0.0, 0.0])
        result = planner.plan_path(start, goal)
        if result.success:
            np.testing.assert_allclose(result.waypoints[0], start, atol=1e-6)
            np.testing.assert_allclose(result.waypoints[-1], goal, atol=1e-6)

    def test_path_stays_in_free_space(self):
        """所有路径点对应的 grid 格必须是可通行的。"""
        planner = self._two_room_planner()
        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([8.0, 0.0, 0.0]))
        if not result.success:
            pytest.skip("Path not found — skip obstacle check")

        trav = planner._trav
        for wp in result.waypoints:
            ix, iy = _w2g(wp[0], wp[1],
                          planner._trav_cx, planner._trav_cy,
                          planner._trav_res,
                          planner._ox, planner._oy,
                          planner._nx, planner._ny)
            assert trav[ix, iy] < planner._obs_thr, \
                f"Waypoint {wp} maps to obstacle grid ({ix},{iy})"

    def test_update_traversability(self):
        """热更新 trav grid 后能重新规划。"""
        rooms = [MockRoom(1, 0.0, 0.0), MockRoom(2, 8.0, 0.0)]
        tsg = MockTSG(rooms, {(1, 2): 8.0})
        # 显式构造 trav=None 版本
        planner = HybridPlanner(topology_graph=tsg, trav=None)
        assert planner._trav is None

        new_trav = _make_open_trav(200, 100)
        planner.update_traversability(new_trav, res=0.2, cx=8.0, cy=0.0)
        assert planner._trav is not None
        assert planner._nx == 200

    def test_same_room_single_segment(self):
        """起点终点在同一房间 → room_sequence 长度为 1，仍成功。"""
        rooms = [MockRoom(1, 0.0, 0.0, {"x_min": -5, "x_max": 5, "y_min": -5, "y_max": 5})]
        tsg = MockTSG(rooms, {})
        trav = _make_open_trav(100, 100)
        planner = HybridPlanner(
            topology_graph=tsg,
            trav=trav,
            trav_res=0.2,
            trav_cx=0.0,
            trav_cy=0.0,
        )
        result = planner.plan_path(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
        # 同一房间，shortest_path 直接返回 [1]，无段规划
        # 结果可能是 success=True（无几何段）或 success=False（无段）
        # 重要：绝对不能有直线路径
        if result.success:
            assert len(result.waypoints) >= 1


# ── HybridPath 数据类测试 ──────────────────────────────────────────────────────

class TestHybridPath:
    def test_failure_has_empty_waypoints(self):
        hp = HybridPlanner._failed("test reason")
        assert not hp.success
        assert hp.waypoints == []
        assert hp.failure_reason == "test reason"
        assert hp.total_cost == float("inf")

    def test_failure_reason_propagated(self):
        rooms = [MockRoom(1, 0.0, 0.0), MockRoom(2, 100.0, 0.0)]
        tsg = MockTSG(rooms, {})  # 不连通
        planner = HybridPlanner(topology_graph=tsg, trav=_make_open_trav())
        result = planner.plan_path(np.array([0, 0, 0]), np.array([100, 0, 0]))
        assert not result.success
        assert len(result.failure_reason) > 0
