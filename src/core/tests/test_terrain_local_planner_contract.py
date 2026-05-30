from __future__ import annotations

import numpy as np

from base_autonomy.modules.local_planner_module import LocalPlannerModule
from base_autonomy.modules.terrain_module import TerrainModule
from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from core.msgs.nav import Odometry, Path
from core.msgs.sensor import PointCloud2
from core.runtime_interface import TOPICS, topic_default_frame_id


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


class _FakeLocalPathVertex:
    def __init__(self, x: float, y: float, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z


class _FakeLocalPlanResult:
    path = [
        _FakeLocalPathVertex(0.0, 0.0),
        _FakeLocalPathVertex(1.0, 0.0),
    ]
    slow_down = 2
    near_field_stop = True
    path_found = True
    recovery_state = 0


class _FakeLocalPlannerCore:
    def set_vehicle(self, *_args):
        pass

    def set_goal(self, *_args):
        pass

    def plan(self, *_args):
        return _FakeLocalPlanResult()


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
    assert terrain.frame_id == topic_default_frame_id(TOPICS.terrain_map)
    assert terrain.points.shape == (2, 4)
    np.testing.assert_allclose(terrain.points[:, 3], np.array([0.35, 0.60], dtype=np.float32))
    assert [field.name for field in terrain.fields] == ["x", "y", "z", "intensity"]


def test_local_planner_default_path_frame_uses_runtime_topic_contract():
    module = LocalPlannerModule(backend="simple")

    assert module._path_frame_id == topic_default_frame_id(TOPICS.local_path)


def test_local_planner_corridor_goal_prefers_current_floor_at_same_xy():
    module = LocalPlannerModule(backend="simple", corridor_lookahead_m=1.0)
    module._robot_pos = np.asarray([0.0, 0.0, 0.0], dtype=float)
    module._global_path_points = np.asarray(
        [
            [0.0, 0.0, 3.0],
            [1.0, 0.0, 3.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
        ],
        dtype=float,
    )

    goal = module._select_corridor_goal(np.asarray([4.0, 0.0, 0.0], dtype=float))

    np.testing.assert_allclose(goal, np.asarray([1.0, 0.0, 0.0], dtype=float))


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


def test_nanobind_local_planner_publishes_control_hint_for_near_field_stop():
    module = LocalPlannerModule(backend="nanobind")
    module._core = _FakeLocalPlannerCore()
    hints: list[dict] = []
    module.control_hint._add_callback(hints.append)
    module._latest_waypoint = PoseStamped(
        pose=Pose(position=Vector3(4.0, 0.0, 0.0)),
        frame_id="map",
        ts=1.0,
    )

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

    assert hints
    assert hints[-1]["slow_down"] == 2
    assert hints[-1]["near_field_stop"] is True
    assert hints[-1]["safety_stop"] is True
    assert hints[-1]["reason"] == "nanobind"
