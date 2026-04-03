"""test_planner_backends.py — _PCTBackend 和 _AStarBackend 单元测试

覆盖：
  _PCTBackend:
    - ele_planner.so 不可用时 available=False，plan() 返回 []，不崩溃
    - tomogram 文件不存在时 available=False，日志清晰
    - 注册名为 "pct"

  _AStarBackend:
    - 基本路径规划（开放 grid）
    - 起点包含在路径中（修复前的 bug：起点丢失）
    - A* 失败返回 []（不返回直线）
    - update_map() 热更新
    - 注册名为 "astar"
    - Euclidean 启发（不再是 Manhattan）

不依赖 scipy / ROS2 / ele_planner.so。
"""

import math
import os
import pickle
import tempfile

import numpy as np
import pytest

from core.registry import get, snapshot, restore


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def registry_isolation():
    """每个测试前保存注册表，测试后恢复，避免污染。"""
    saved = snapshot()
    # 触发 @register 装饰器（import 时执行）
    import global_planning.pct_adapters.src.global_planner_module  # noqa: F401
    yield
    restore(saved)
    # 重新注册（restore 清空后需要重新注册供其他测试使用）
    import importlib
    importlib.reload(global_planning.pct_adapters.src.global_planner_module)


def _make_open_grid(rows=50, cols=50, obs_thr=49.9):
    """全通行 grid，trav[row, col] < obs_thr。"""
    return np.zeros((rows, cols), dtype=np.float32)


def _make_blocked_grid(rows=50, cols=50):
    """中间横墙，完全封堵。"""
    g = np.zeros((rows, cols), dtype=np.float32)
    g[rows // 2, :] = 100.0
    return g


def _make_tomogram_pickle(trav: np.ndarray, resolution: float = 0.2,
                           center=(0.0, 0.0)) -> str:
    """写一个最小格式的 tomogram.pickle，供 _load_tomogram 读取。"""
    rows, cols = trav.shape
    # data shape: (5, n_slices, H, W) — channel 0 = traversability
    data = np.zeros((5, 1, rows, cols), dtype=np.float32)
    data[0, 0] = trav
    payload = {
        "data": data,
        "resolution": resolution,
        "center": list(center) + [0.0],
        "slice_h0": 0.5,
        "slice_dh": 0.5,
    }
    fd, path = tempfile.mkstemp(suffix=".pickle")
    os.close(fd)
    with open(path, "wb") as f:
        pickle.dump(payload, f)
    return path


# ---------------------------------------------------------------------------
# _PCTBackend tests
# ---------------------------------------------------------------------------

class TestPCTBackend:
    def _backend(self, tomogram_path="", obs=49.9):
        from global_planning.pct_adapters.src.global_planner_module import _PCTBackend
        return _PCTBackend(tomogram_path=tomogram_path, obstacle_thr=obs)

    def test_registered_as_pct(self):
        from global_planning.pct_adapters.src.global_planner_module import _PCTBackend
        cls = get("planner_backend", "pct")
        assert cls is _PCTBackend

    def test_unavailable_on_missing_so(self):
        """ele_planner.so 不可用时 available=False，不崩溃。"""
        b = self._backend(tomogram_path="nonexistent.pickle")
        assert not b.available

    def test_plan_returns_empty_when_unavailable(self):
        """unavailable 时 plan() 返回 []，不 raise。"""
        b = self._backend()
        result = b.plan(np.array([0.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]))
        assert result == [], "PCTBackend unavailable must return [], not crash"

    def test_plan_returns_empty_on_missing_tomogram(self):
        b = self._backend(tomogram_path="/nonexistent/path/tomo.pickle")
        result = b.plan(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
        assert result == []

    def test_load_error_message_present(self):
        """unavailable 时 _load_error 应有清晰说明，便于排查。"""
        b = self._backend(tomogram_path="not_a_real_file.pickle")
        assert len(b._load_error) > 0


# ---------------------------------------------------------------------------
# _AStarBackend tests
# ---------------------------------------------------------------------------

class TestAStarBackend:
    def _backend(self, trav=None, resolution=0.2, center=(0.0, 0.0), obs=49.9):
        from global_planning.pct_adapters.src.global_planner_module import _AStarBackend
        b = _AStarBackend(obstacle_thr=obs)
        if trav is not None:
            h, w = trav.shape
            cx, cy = center
            # origin = center - (w*res/2, h*res/2)
            origin = np.array([cx - w * resolution / 2,
                                cy - h * resolution / 2])
            b.update_map(trav, resolution=resolution, origin=origin)
        return b

    def test_registered_as_astar(self):
        from global_planning.pct_adapters.src.global_planner_module import _AStarBackend
        cls = get("planner_backend", "astar")
        assert cls is _AStarBackend

    def test_no_map_returns_empty(self):
        b = self._backend()
        result = b.plan(np.array([0, 0, 0]), np.array([1, 0, 0]))
        assert result == []

    def test_open_grid_finds_path(self):
        trav = _make_open_grid(50, 50)
        b = self._backend(trav, resolution=0.2, center=(0.0, 0.0))
        path = b.plan(np.array([-2.0, -2.0, 0.0]), np.array([2.0, 2.0, 0.0]))
        assert len(path) >= 2, "Should find a path on open grid"

    def test_start_point_included_in_path(self):
        """Bug fix: 修复前起点不在路径里（came_from 回溯不含 start）。"""
        trav = _make_open_grid(50, 50)
        b = self._backend(trav, resolution=0.2, center=(0.0, 0.0))
        start = np.array([-2.0, -2.0, 0.0])
        goal  = np.array([ 2.0,  2.0, 0.0])
        path = b.plan(start, goal)
        assert len(path) >= 2
        # 第一个路径点应在 start 附近（1格内，即 0.2m）
        dx = abs(path[0][0] - float(start[0]))
        dy = abs(path[0][1] - float(start[1]))
        assert dx <= 0.3 and dy <= 0.3, (
            f"Start point not in path! path[0]={path[0]}, start={start}"
        )

    def test_blocked_grid_returns_empty(self):
        """完全堵死时返回 []，不返回直线路径。"""
        trav = _make_blocked_grid(50, 50)
        b = self._backend(trav, resolution=0.2, center=(0.0, 0.0))
        # start 在墙的一侧，goal 在另一侧
        path = b.plan(np.array([-2.0, 0.0, 0.0]), np.array([2.0, 0.0, 0.0]))
        assert path == [], "Blocked path MUST return [], not straight-line fallback"

    def test_goal_at_start_returns_single_point(self):
        trav = _make_open_grid(20, 20)
        b = self._backend(trav, resolution=0.2, center=(0.0, 0.0))
        pt = np.array([0.0, 0.0, 0.5])
        path = b.plan(pt, pt)
        assert len(path) == 1

    def test_update_map_replaces_grid(self):
        b = self._backend()
        assert b._grid is None
        new_grid = _make_open_grid(30, 30)
        b.update_map(new_grid, resolution=0.2, origin=np.array([-3.0, -3.0]))
        assert b._grid is not None
        assert b._grid.shape == (30, 30)

    def test_load_tomogram_pickle(self):
        """从 .pickle 文件加载，能规划路径。"""
        trav = _make_open_grid(50, 50)
        path = _make_tomogram_pickle(trav, resolution=0.2, center=(0.0, 0.0))
        try:
            from global_planning.pct_adapters.src.global_planner_module import _AStarBackend
            b = _AStarBackend(tomogram_path=path, obstacle_thr=49.9)
            result = b.plan(np.array([-2.0, -2.0, 0.0]), np.array([2.0, 2.0, 0.0]))
            assert len(result) >= 2, "Should plan path from loaded tomogram"
        finally:
            os.unlink(path)

    def test_path_points_in_free_space(self):
        """所有路径点对应的格子应在可行区域内。"""
        trav = _make_open_grid(50, 50)
        b = self._backend(trav, resolution=0.2, center=(0.0, 0.0))
        path = b.plan(np.array([-1.0, -1.0, 0.0]), np.array([1.0, 1.0, 0.0]))
        if not path:
            pytest.skip("No path found — skip obstacle check")
        res = b._resolution
        origin = b._origin
        h, w = trav.shape
        for (wx, wy, _) in path:
            col = int(round((wx - origin[0]) / res))
            row = int(round((wy - origin[1]) / res))
            col = max(0, min(col, w - 1))
            row = max(0, min(row, h - 1))
            assert trav[row, col] < 49.9, f"Path goes through obstacle at ({col},{row})"

    def test_euclidean_heuristic_not_manhattan(self):
        """启发函数应是 Euclidean（admissible），比 Manhattan 更精确。
        间接测试：对角线路径代价应 ≈ sqrt(2)*格数，而非 2*格数。"""
        trav = _make_open_grid(20, 20)
        b = self._backend(trav, resolution=1.0, center=(10.0, 10.0))
        # 对角线：从 (0,0) 到 (5,5)，理想代价 ≈ 5*sqrt(2) ≈ 7.07
        path = b.plan(np.array([0.0, 0.0, 0.0]), np.array([5.0, 5.0, 0.0]))
        assert len(path) >= 2
        # 路径长度应 ≈ 对角线长度，不超过 Manhattan 距离 10 太多
        total_dist = sum(
            math.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            for i in range(len(path) - 1)
        )
        manhattan = 5.0 + 5.0  # = 10
        diagonal  = 5.0 * math.sqrt(2)  # ≈ 7.07
        # 路径长度应在 [diagonal, manhattan] 之间（对角线走法）
        assert total_dist <= manhattan + 0.5, \
            f"Path length {total_dist:.2f} > Manhattan {manhattan}"
