from __future__ import annotations

import numpy as np

from core.msgs.geometry import Pose, PoseStamped, Twist, Vector3
from nav.cmd_vel_mux_module import CmdVelMux
from nav.global_planner_service import GlobalPlannerService
from nav.navigation_module import NavigationModule
from nav.traversability_cost_module import TraversabilityCostModule
from semantic.planner.semantic_planner.bbox_navigator import STATE_TRACKING
from semantic.planner.semantic_planner.visual_servo_module import (
    MODE_FIND,
    VisualServoModule,
)


class _FakePlanner:
    is_ready = True
    has_map = True

    def plan(self, start: np.ndarray, goal: np.ndarray):
        midpoint = np.array([2.5, 0.0, 0.0])
        return [start.copy(), midpoint, goal.copy()], 1.0

    def update_map(self, *args, **kwargs) -> None:
        pass


class _GridBackend:
    def __init__(self, grid: np.ndarray, resolution: float = 1.0):
        self._grid = grid
        self._resolution = resolution
        self._origin = np.array([0.0, 0.0])

    def plan(self, start: np.ndarray, goal: np.ndarray):
        return [start, goal]


def test_navigation_goal_publishes_global_path_and_first_waypoint():
    nav = NavigationModule(enable_ros2_bridge=False)
    nav._planner_svc = _FakePlanner()

    paths: list[list[np.ndarray]] = []
    waypoints: list[PoseStamped] = []
    states: list[dict] = []
    nav.global_path._add_callback(paths.append)
    nav.waypoint._add_callback(waypoints.append)
    nav.mission_status._add_callback(states.append)

    nav._robot_pos = np.array([0.0, 0.0, 0.0])
    nav._on_goal(PoseStamped(Pose(5.0, 0.0, 0.0)))

    assert len(paths) == 1
    assert np.allclose(paths[0][-1], [5.0, 0.0, 0.0])
    assert len(waypoints) == 1
    assert waypoints[0].pose.position.x == 2.5
    assert states[-1]["state"] == "EXECUTING"


def test_global_planner_safe_goal_bfs_adjusts_obstacle_goal():
    svc = GlobalPlannerService(obstacle_thr=49.9)
    grid = np.array(
        [
            [100.0, 100.0, 100.0],
            [100.0, 100.0, 0.0],
            [100.0, 100.0, 100.0],
        ],
        dtype=np.float32,
    )
    svc._backend = _GridBackend(grid)

    safe_goal = svc._find_safe_goal(np.array([1.0, 1.0, 0.0]), tolerance=2.0)

    assert safe_goal is not None
    assert np.allclose(safe_goal, [2.0, 1.0, 0.0])


def test_traversability_fusion_preserves_hard_costs_and_relays_esdf():
    module = TraversabilityCostModule(
        safe_distance=2.0,
        max_slope_deg=45.0,
        proximity_cap=50.0,
        publish_hz=1000.0,
    )
    fused: list[dict] = []
    relayed_esdf: list[dict] = []
    module.fused_cost._add_callback(fused.append)
    module.esdf_field._add_callback(relayed_esdf.append)

    esdf = {
        "distance_field": np.array([[0.0, 0.0], [0.0, 2.0]], dtype=np.float32),
        "resolution": 1.0,
        "origin": [0.0, 0.0],
    }
    costmap = {
        "grid": np.array([[0.0, 100.0], [99.0, 0.0]], dtype=np.float32),
        "resolution": 1.0,
        "origin": [0.0, 0.0],
    }

    module._on_esdf(esdf)
    module._on_costmap(costmap)

    assert relayed_esdf == [esdf]
    assert len(fused) == 1
    assert np.allclose(fused[0]["grid"], [[50.0, 100.0], [99.0, 0.0]])


def test_localization_degeneracy_scales_navigation_speed():
    nav = NavigationModule(enable_ros2_bridge=False)

    nav._on_localization_status({"state": "TRACKING", "degeneracy": "MILD"})
    assert nav._speed_scale == 0.7

    nav._on_localization_status({"state": "TRACKING", "degeneracy": "SEVERE"})
    assert nav._speed_scale == 0.4

    nav._on_localization_status({"state": "FALLBACK_GNSS_ONLY", "degeneracy": "NONE"})
    assert nav._speed_scale == 0.3

    nav._on_localization_status({"state": "TRACKING", "degeneracy": "NONE"})
    assert nav._speed_scale == 1.0


def test_visual_servo_near_target_publishes_cmd_vel_and_nav_stop():
    servo = VisualServoModule(servo_takeover_distance=3.0, servo_takeover_hysteresis=0.3)
    cmd_vel: list[Twist] = []
    goals: list[PoseStamped] = []
    stops: list[int] = []
    servo.cmd_vel._add_callback(cmd_vel.append)
    servo.goal_pose._add_callback(goals.append)
    servo.nav_stop._add_callback(stops.append)
    servo._mode = MODE_FIND
    servo._target_label = "crate"
    servo._latest_depth = np.ones((4, 4), dtype=np.float32)
    servo._intrinsics = (100.0, 100.0, 2.0, 2.0)
    servo._find_target_bbox = lambda: [1, 1, 2, 2]
    servo._bbox_nav.update = lambda **_: {
        "state": STATE_TRACKING,
        "distance": 2.0,
        "target_3d": np.array([2.0, 0.0, 0.0]),
        "linear_x": 0.2,
        "angular_z": -0.1,
    }

    servo._tick_find()

    assert stops == [1]
    assert len(cmd_vel) == 1
    assert cmd_vel[0].linear.x == 0.2
    assert cmd_vel[0].angular.z == -0.1
    assert goals == []


def test_visual_servo_far_target_publishes_goal_pose():
    servo = VisualServoModule(servo_takeover_distance=3.0, servo_takeover_hysteresis=0.3)
    cmd_vel: list[Twist] = []
    goals: list[PoseStamped] = []
    stops: list[int] = []
    servo.cmd_vel._add_callback(cmd_vel.append)
    servo.goal_pose._add_callback(goals.append)
    servo.nav_stop._add_callback(stops.append)
    servo._mode = MODE_FIND
    servo._target_label = "crate"
    servo._latest_depth = np.ones((4, 4), dtype=np.float32)
    servo._intrinsics = (100.0, 100.0, 2.0, 2.0)
    servo._find_target_bbox = lambda: [1, 1, 2, 2]
    servo._bbox_nav.update = lambda **_: {
        "state": STATE_TRACKING,
        "distance": 4.0,
        "target_3d": np.array([4.0, 0.0, 0.0]),
        "linear_x": 0.2,
        "angular_z": -0.1,
    }

    servo._tick_find()

    assert stops == []
    assert cmd_vel == []
    assert len(goals) == 1
    assert goals[0].pose.position.x == 4.0


def test_cmd_vel_mux_selects_highest_priority_active_source():
    mux = CmdVelMux(source_timeout=10.0)
    driver_cmds: list[Twist] = []
    active_sources: list[str] = []
    mux.driver_cmd_vel._add_callback(driver_cmds.append)
    mux.active_source._add_callback(active_sources.append)

    mux._on_source("path_follower", Twist(linear=Vector3(x=0.1)))
    mux._on_source("visual_servo", Twist(linear=Vector3(x=0.2)))
    mux._on_source("teleop", Twist(linear=Vector3(x=0.3)))

    assert [cmd.linear.x for cmd in driver_cmds] == [0.1, 0.2, 0.3]
    assert active_sources == ["path_follower", "visual_servo", "teleop"]
