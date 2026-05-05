from __future__ import annotations

import numpy as np

from core.msgs.geometry import Pose, PoseStamped, Quaternion, Vector3
from core.msgs.nav import Odometry
from nav.navigation_module import NavigationModule


class _RecordingPlanner:
    is_ready = True
    has_map = True

    def __init__(self) -> None:
        self.maps: list[dict] = []

    def plan(self, start: np.ndarray, goal: np.ndarray):
        return [start.copy(), goal.copy()], 0.0

    def update_map(self, grid, resolution=0.2, origin=None) -> None:
        self.maps.append({
            "grid": grid,
            "resolution": resolution,
            "origin": origin,
        })


def test_navigation_reports_frame_mismatch_when_odometry_is_not_in_planning_frame():
    nav = NavigationModule(enable_ros2_bridge=False)
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
    ))

    assert events[-1]["event"] == "frame_mismatch"
    assert events[-1]["source"] == "odometry"
    assert events[-1]["expected_frame"] == "map"
    assert events[-1]["received_frame"] == "odom"
    assert nav._odom_frame_id == "odom"
    np.testing.assert_array_equal(nav._robot_pos, np.zeros(3))


def test_navigation_reports_frame_mismatch_when_costmap_is_not_in_planning_frame():
    nav = NavigationModule(enable_ros2_bridge=False)
    nav._planner_svc = _RecordingPlanner()
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_costmap({
        "grid": np.zeros((2, 2), dtype=np.int8),
        "resolution": 0.5,
        "origin": [0.0, 0.0],
        "frame_id": "odom",
    })

    assert events[-1]["event"] == "frame_mismatch"
    assert events[-1]["source"] == "costmap"
    assert events[-1]["expected_frame"] == "map"
    assert events[-1]["received_frame"] == "odom"
    assert nav._costmap_frame_id == "odom"
    assert nav._planner_svc.maps == []


def test_navigation_accepts_map_frame_odometry_without_frame_mismatch_report():
    nav = NavigationModule(enable_ros2_bridge=False)
    events: list[dict] = []
    nav.adapter_status._add_callback(events.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert events == []
    assert nav._odom_frame_id == "map"


def test_navigation_blocks_goal_when_odometry_is_not_in_planning_frame():
    nav = NavigationModule(enable_ros2_bridge=False, allow_direct_goal_fallback=True)
    nav._planner_svc = _RecordingPlanner()
    events: list[dict] = []
    waypoints: list[PoseStamped] = []
    nav.adapter_status._add_callback(events.append)
    nav.waypoint._add_callback(waypoints.append)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
    ))
    nav._on_goal(PoseStamped(
        pose=Pose(position=Vector3(3.0, 4.0, 0.0), orientation=Quaternion()),
        frame_id="map",
    ))

    assert nav._state == "FAILED"
    assert waypoints == []
    assert nav._failure_reason == "unsupported odometry frame 'odom'; expected 'map'"
    assert events[-1]["event"] == "navigation_blocked"
    assert events[-1]["source"] == "odometry"


def test_navigation_clears_motion_when_active_odometry_frame_mismatches():
    nav = NavigationModule(enable_ros2_bridge=False)
    events: list[dict] = []
    clears: list[bool] = []
    zeros = []
    nav.adapter_status._add_callback(events.append)
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)

    nav._state = "EXECUTING"
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(2.0, 3.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
    ))

    assert nav._state == "FAILED"
    assert nav._failure_reason == "unsupported odometry frame 'odom'; expected 'map'"
    assert nav._tracker.path_length == 0
    np.testing.assert_array_equal(nav._robot_pos, np.zeros(3))
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0
    assert events[-1]["event"] == "navigation_blocked"
    assert events[-1]["source"] == "odometry"


def test_navigation_clears_motion_when_active_costmap_frame_mismatches():
    nav = NavigationModule(enable_ros2_bridge=False)
    nav._planner_svc = _RecordingPlanner()
    events: list[dict] = []
    clears: list[bool] = []
    zeros = []
    nav.adapter_status._add_callback(events.append)
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)

    nav._state = "EXECUTING"
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_costmap({
        "grid": np.zeros((2, 2), dtype=np.int8),
        "resolution": 0.5,
        "origin": [0.0, 0.0],
        "frame_id": "odom",
    })

    assert nav._state == "FAILED"
    assert nav._failure_reason == "unsupported costmap frame 'odom'; expected 'map'"
    assert nav._tracker.path_length == 0
    assert nav._planner_svc.maps == []
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0
    assert events[-1]["event"] == "navigation_blocked"
    assert events[-1]["source"] == "costmap"


def test_navigation_cancel_is_idempotent_motion_cleanup_while_idle():
    nav = NavigationModule(enable_ros2_bridge=False)
    clears: list[bool] = []
    zeros = []
    nav.clear_path._add_callback(clears.append)
    nav.recovery_cmd_vel._add_callback(zeros.append)
    nav._goal = np.array([5.0, 0.0, 0.0])
    nav._tracker.reset([np.array([5.0, 0.0, 0.0])], nav._robot_pos)

    nav._on_cancel("operator_cancel")

    assert nav._state == "IDLE"
    assert nav._goal is None
    assert nav._goal_frame_id is None
    assert nav._tracker.path_length == 0
    assert clears == [True]
    assert zeros[-1].linear.x == 0.0
    assert zeros[-1].angular.z == 0.0


def test_navigation_preview_rejects_non_map_odometry_frame():
    nav = NavigationModule(enable_ros2_bridge=False)
    nav._planner_svc = _RecordingPlanner()
    nav._on_odom(Odometry(
        pose=Pose(position=Vector3(1.0, 2.0, 0.0), orientation=Quaternion()),
        frame_id="odom",
    ))

    preview = nav.preview_plan(3.0, 4.0)

    assert preview["feasible"] is False
    assert preview["reasons"] == ["frame_mismatch"]
    assert preview["error"] == "unsupported odometry frame 'odom'; expected 'map'"
