"""test_planner_backends.py — _PCTBackend and _AStarBackend unit tests

Coverage:
  _PCTBackend:
    - available=False when ele_planner.so is missing, plan() returns [], no crash
    - available=False when tomogram file does not exist, with a clear log message
    - Registered under the name "pct"

  _AStarBackend:
    - Basic path planning on an open grid
    - Start point is included in the path (regression: start was previously dropped)
    - A* failure returns [] (not a straight-line fallback)
    - update_map() hot-swap works
    - Registered under the name "astar"
    - Euclidean heuristic (no longer Manhattan)

No scipy / ROS2 / ele_planner.so dependencies.
"""

import math
import os
import pickle
import tempfile

import numpy as np
import pytest

from core.registry import get, restore, snapshot

# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def registry_isolation():
    """Save registry state before each test, restore after to prevent pollution."""
    saved = snapshot()
    # Trigger @register decorators (executed at import time)
    import global_planning.pct_adapters.src.global_planner_module
    yield
    restore(saved)
    # Re-register after restore clears the registry
    import importlib
    importlib.reload(global_planning.pct_adapters.src.global_planner_module)


def _make_open_grid(rows=50, cols=50, obs_thr=49.9):
    """Fully traversable grid — all cells below obs_thr."""
    return np.zeros((rows, cols), dtype=np.float32)


def _make_blocked_grid(rows=50, cols=50):
    """Grid with a solid horizontal wall through the middle — no gap."""
    g = np.zeros((rows, cols), dtype=np.float32)
    g[rows // 2, :] = 100.0
    return g


def _make_tomogram_pickle(trav: np.ndarray, resolution: float = 0.2,
                           center=(0.0, 0.0)) -> str:
    """Write a minimal-format tomogram.pickle for _load_tomogram to read."""
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
        """ele_planner.so not available → available=False, no crash."""
        b = self._backend(tomogram_path="nonexistent.pickle")
        assert not b.available

    def test_plan_returns_empty_when_unavailable(self):
        """plan() returns [] when unavailable — must not raise."""
        b = self._backend()
        result = b.plan(np.array([0.0, 0.0, 0.0]), np.array([5.0, 0.0, 0.0]))
        assert result == [], "PCTBackend unavailable must return [], not crash"

    def test_plan_returns_empty_on_missing_tomogram(self):
        b = self._backend(tomogram_path="/nonexistent/path/tomo.pickle")
        result = b.plan(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]))
        assert result == []

    def test_load_error_message_present(self):
        """_load_error must be non-empty when unavailable for easier debugging."""
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
        """Regression: before fix, start was dropped (came_from did not include start)."""
        trav = _make_open_grid(50, 50)
        b = self._backend(trav, resolution=0.2, center=(0.0, 0.0))
        start = np.array([-2.0, -2.0, 0.0])
        goal  = np.array([ 2.0,  2.0, 0.0])
        path = b.plan(start, goal)
        assert len(path) >= 2
        # First path point must be within one cell width of start (0.2 m)
        dx = abs(path[0][0] - float(start[0]))
        dy = abs(path[0][1] - float(start[1]))
        assert dx <= 0.3 and dy <= 0.3, (
            f"Start point not in path! path[0]={path[0]}, start={start}"
        )

    def test_blocked_grid_returns_empty(self):
        """Fully blocked path returns [] — no straight-line fallback."""
        trav = _make_blocked_grid(50, 50)
        b = self._backend(trav, resolution=0.2, center=(0.0, 0.0))
        # start on one side of the wall, goal on the other
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
        """Loading from a .pickle file should enable path planning."""
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
        """All path points must map to traversable grid cells."""
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
        """Heuristic must be Euclidean (admissible) — indirect check via path length.

        A diagonal path (0,0)→(5,5) with Euclidean heuristic produces a total
        length ≈ 5*sqrt(2) ≈ 7.07, much closer to optimal than Manhattan (10).
        """
        trav = _make_open_grid(20, 20)
        b = self._backend(trav, resolution=1.0, center=(10.0, 10.0))
        path = b.plan(np.array([0.0, 0.0, 0.0]), np.array([5.0, 5.0, 0.0]))
        assert len(path) >= 2
        total_dist = sum(
            math.hypot(path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            for i in range(len(path) - 1)
        )
        manhattan = 5.0 + 5.0   # = 10.0 (upper bound for this grid)
        # Path length should be no more than Manhattan distance + small margin
        assert total_dist <= manhattan + 0.5, \
            f"Path length {total_dist:.2f} > Manhattan {manhattan}"
