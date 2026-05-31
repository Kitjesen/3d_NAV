from __future__ import annotations

import pickle
import tempfile
from pathlib import Path

import numpy as np

from nav.global_planner_service import GlobalPlannerService
from sim.engine.scenarios.nav_corridor_assets import build_corridor_gap_assets


def test_corridor_gap_tomogram_forces_astar_detour(tmp_path):
    assets = build_corridor_gap_assets(tmp_path)

    with assets.tomogram.open("rb") as fh:
        tomo = pickle.load(fh)
    grid = tomo["data"][0, 0]
    assert grid.max() >= 100.0

    svc = GlobalPlannerService(
        planner_name="astar",
        tomogram=str(assets.tomogram),
        downsample_dist=0.2,
        obstacle_thr=49.9,
    )
    svc.setup()
    path, _plan_ms = svc.plan(
        np.asarray(assets.start, dtype=float),
        np.asarray(assets.goal, dtype=float),
        safe_goal_tolerance=0.0,
    )

    xy = np.asarray([[float(p[0]), float(p[1])] for p in path], dtype=float)
    assert len(xy) > 4
    assert np.max(xy[:, 1]) > 0.45
    assert np.linalg.norm(xy[-1] - np.asarray(assets.goal[:2], dtype=float)) < 1e-6

    direct = float(np.linalg.norm(np.asarray(assets.goal[:2]) - np.asarray(assets.start[:2])))
    routed = float(np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1)))
    assert routed > direct * 1.15


def test_astar_live_costmap_preserves_static_tomogram(tmp_path):
    assets = build_corridor_gap_assets(tmp_path)
    svc = GlobalPlannerService(
        planner_name="astar",
        tomogram=str(assets.tomogram),
        downsample_dist=0.2,
        obstacle_thr=49.9,
    )
    svc.setup()
    backend = svc._backend
    original_shape = backend._grid.shape
    original_obstacles = int(np.count_nonzero(backend._grid >= 49.9))

    live = np.zeros((8, 8), dtype=np.float32)
    live[2, 3] = 100.0
    svc.update_map(live, resolution=0.1, origin=np.asarray([-0.6, -1.5]))

    assert backend._grid.shape == original_shape
    assert int(np.count_nonzero(backend._grid >= 49.9)) >= original_obstacles
    assert backend._grid[2, 3] >= 100.0


def test_astar_transposes_official_builder_tomogram_axes():
    data = np.zeros((5, 1, 6, 4), dtype=np.float32)
    official = {
        "data": data,
        "resolution": 1.0,
        "center": [1.0, 2.0],
        "slice_h0": 0.5,
        "slice_dh": 0.5,
    }
    with tempfile.NamedTemporaryFile(suffix=".pickle", delete=False) as fh:
        pickle.dump(official, fh)
        path = fh.name

    try:
        from global_planning.pct_adapters.global_planner_module import _AStarBackend

        backend = _AStarBackend(tomogram_path=path)
        assert backend._grid.shape == (4, 6)
        np.testing.assert_allclose(backend._origin, np.asarray([-2.0, 0.0]))
    finally:
        Path(path).unlink(missing_ok=True)
