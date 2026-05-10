from __future__ import annotations

import numpy as np

from base_autonomy.modules.local_planner_module import LocalPlannerModule
from base_autonomy.modules.terrain_module import TerrainModule
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.sensor import PointCloud2


class _FakeTerrainResult:
    n_points = 2
    terrain_points = [
        1.0,
        0.0,
        0.35,
        0.35,
        1.5,
        0.2,
        0.60,
        0.60,
    ]
    map_width = 2
    map_resolution = 0.1
    elevation_map = [0.0, 0.1, 0.2, 0.3]


class _FakeTerrainCore:
    def process(self, _flat, _ts):
        return _FakeTerrainResult()


def test_terrain_nanobind_preserves_height_intensity_for_local_planner():
    module = TerrainModule(backend="nanobind")
    module._core = _FakeTerrainCore()
    published: list[PointCloud2] = []
    module.terrain_map._add_callback(published.append)

    module._process_nanobind(
        PointCloud2(
            points=np.array([[0.0, 0.0, 0.0]], dtype=np.float32),
            frame_id="body",
            ts=12.0,
        )
    )

    assert len(published) == 1
    terrain = published[0]
    assert terrain.frame_id == "map"
    assert terrain.points.shape == (2, 4)
    np.testing.assert_allclose(terrain.points[:, 3], np.array([0.35, 0.60], dtype=np.float32))
    assert [field.name for field in terrain.fields] == ["x", "y", "z", "intensity"]


def _run_cmu_py_local_plan(obstacle_half_width: float) -> tuple[np.ndarray, np.ndarray]:
    module = LocalPlannerModule(backend="cmu_py")
    module.setup()
    published: list[Path] = []
    module.local_path._add_callback(published.append)
    module._on_odom(
        Odometry(
            pose=Pose(
                position=Vector3(0.0, 0.0, 0.0),
                orientation=Quaternion.from_yaw(0.0),
            ),
            frame_id="map",
            child_frame_id="base_link",
            ts=1.0,
        )
    )

    obstacle_points = []
    for x in np.linspace(1.0, 1.8, 5):
        samples = max(3, int(obstacle_half_width * 20) + 1)
        for y in np.linspace(-obstacle_half_width, obstacle_half_width, samples):
            obstacle_points.append([x, y, 0.0, 200.0])
    obstacles = np.asarray(obstacle_points, dtype=np.float32)
    module._on_added_obstacles(PointCloud2(points=obstacles, frame_id="map", ts=1.0))
    module._latest_waypoint = PoseStamped(
        pose=Pose(
            position=Vector3(4.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        ),
        frame_id="map",
        ts=1.0,
    )

    module._run_cmu_py()
    assert published, "local planner must publish a path decision"
    path = published[-1]
    path_xy = np.asarray(
        [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses],
        dtype=float,
    )
    return path_xy, obstacles[:, :2].astype(float)


def test_cmu_py_local_planner_routes_around_feasible_added_obstacle():
    path_xy, obstacle_xy = _run_cmu_py_local_plan(obstacle_half_width=0.15)

    assert len(path_xy) > 2
    assert float(np.max(np.abs(path_xy[:, 1]))) > 0.25
    min_clearance = float(
        np.min(np.linalg.norm(path_xy[:, None, :] - obstacle_xy[None, :, :], axis=2))
    )
    assert min_clearance > 0.30


def test_cmu_py_local_planner_does_not_fallback_to_collision_line_when_blocked():
    path_xy, _obstacle_xy = _run_cmu_py_local_plan(obstacle_half_width=0.30)

    assert path_xy.shape == (0,)
