"""Tests for PathAdapterModule and MissionArcModule -- planning Module conversions.

Covers:
- PathAdapterModule: downsample, waypoint progression, stuck detection,
  goal_reached, max_index_jump, max_first_waypoint_dist, NaN odom rejection
- MissionArcModule: FSM transitions (IDLE->PLANNING->EXECUTING->COMPLETE/FAILED),
  replan on failure, recovery timeout, planning timeout, mission timeout,
  patrol mode, cancel, NaN goal rejection, e-stop
"""

import math
import time
from unittest.mock import patch

import pytest

from core import In, Out
from core.msgs.nav import Odometry, Path
from core.msgs.geometry import Pose, PoseStamped, Vector3

# Import modules under test
from global_planning.pct_adapters.src.path_adapter_module import PathAdapterModule
from global_planning.pct_adapters.src.mission_arc_module import (
    MissionArcModule,
    MissionState,
)


# ============================================================================
# Helpers
# ============================================================================

def _make_path(points, frame_id="map"):
    """Create a Path from list of (x, y, z) tuples."""
    poses = []
    for x, y, z in points:
        ps = PoseStamped(
            pose=Pose(position=Vector3(x, y, z)),
            frame_id=frame_id,
        )
        poses.append(ps)
    return Path(poses=poses, frame_id=frame_id)


def _make_odom(x, y, z=0.0):
    """Create an Odometry at given position."""
    return Odometry(pose=Pose(position=Vector3(x, y, z)))


def _collect_events(module, port_name):
    """Attach a collector to an Out port and return the list."""
    events = []
    port = getattr(module, port_name)
    port._add_callback(events.append)
    return events


# ============================================================================
# TestPathAdapterModule -- Downsampling
# ============================================================================

class TestPathAdapterDownsample:
    """Path downsampling by 3D distance."""

    def test_empty_path_ignored(self):
        mod = PathAdapterModule()
        mod.setup()
        events = _collect_events(mod, "adapter_status")
        mod.global_path._deliver(Path())
        assert len(events) == 0
        assert mod.path_length == 0

    def test_single_point_path(self):
        mod = PathAdapterModule()
        mod.setup()
        events = _collect_events(mod, "adapter_status")
        path = _make_path([(1, 2, 0)])
        mod.global_path._deliver(path)
        assert mod.path_length == 1
        assert events[-1]["event"] == "path_received"

    def test_dense_path_downsampled(self):
        """100 points at 0.01m spacing with waypoint_distance=0.5 -> ~3 waypoints."""
        mod = PathAdapterModule(waypoint_distance=0.5)
        mod.setup()
        points = [(i * 0.01, 0, 0) for i in range(101)]
        path = _make_path(points)
        mod.global_path._deliver(path)
        assert mod.path_length == 3

    def test_sparse_path_preserved(self):
        """Points already spaced > waypoint_distance are all kept."""
        mod = PathAdapterModule(waypoint_distance=0.5)
        mod.setup()
        points = [(0, 0, 0), (1, 0, 0), (2, 0, 0), (3, 0, 0)]
        path = _make_path(points)
        mod.global_path._deliver(path)
        assert mod.path_length == 4

    def test_3d_distance_used(self):
        """Downsampling uses 3D distance, not just 2D."""
        mod = PathAdapterModule(waypoint_distance=1.0)
        mod.setup()
        points = [(0, 0, 0), (0.6, 0, 0.8), (1.2, 0, 1.6)]
        path = _make_path(points)
        mod.global_path._deliver(path)
        assert mod.path_length == 3

    def test_final_point_always_included(self):
        """Final goal point always included even if within waypoint_distance."""
        mod = PathAdapterModule(waypoint_distance=5.0)
        mod.setup()
        points = [(0, 0, 0), (1, 0, 0), (2, 0, 0)]
        path = _make_path(points)
        mod.global_path._deliver(path)
        assert mod.path_length == 2


# ============================================================================
# TestPathAdapterModule -- Waypoint Progression
# ============================================================================

class TestPathAdapterWaypointProgression:
    """Waypoint tracking and arrival detection."""

    def test_basic_progression(self):
        """Robot moves along path, waypoints advance."""
        mod = PathAdapterModule(waypoint_distance=0.5, arrival_threshold=0.3)
        mod.setup()
        events = _collect_events(mod, "adapter_status")

        mod.odometry._deliver(_make_odom(0, 0))
        path = _make_path([(0, 0, 0), (1, 0, 0), (2, 0, 0)])
        mod.global_path._deliver(path)
        assert mod.current_waypoint_idx == 0

        mod.odometry._deliver(_make_odom(0, 0))
        mod.tick()
        assert mod.current_waypoint_idx == 1
        reached_events = [e for e in events if e["event"] == "waypoint_reached"]
        assert len(reached_events) == 1

    def test_goal_reached(self):
        """goal_reached published when robot arrives at last waypoint."""
        mod = PathAdapterModule(waypoint_distance=5.0, arrival_threshold=0.5)
        mod.setup()
        events = _collect_events(mod, "adapter_status")

        mod.odometry._deliver(_make_odom(0, 0))
        path = _make_path([(0, 0, 0), (1, 0, 0)])
        mod.global_path._deliver(path)

        mod.odometry._deliver(_make_odom(0, 0))
        mod.tick()  # arrive at wp 0 -> advance to wp 1
        assert mod.current_waypoint_idx == 1

        mod.odometry._deliver(_make_odom(1, 0))
        mod.tick()
        assert mod.goal_reached is True
        goal_events = [e for e in events if e["event"] == "goal_reached"]
        assert len(goal_events) == 1

    def test_goal_reached_not_repeated(self):
        """goal_reached is published only once."""
        mod = PathAdapterModule(waypoint_distance=5.0, arrival_threshold=0.5)
        mod.setup()
        events = _collect_events(mod, "adapter_status")

        mod.odometry._deliver(_make_odom(0, 0))
        path = _make_path([(0, 0, 0), (1, 0, 0)])
        mod.global_path._deliver(path)

        mod.odometry._deliver(_make_odom(0, 0))
        mod.tick()
        mod.odometry._deliver(_make_odom(1, 0))
        mod.tick()
        mod.tick()
        mod.tick()

        goal_events = [e for e in events if e["event"] == "goal_reached"]
        assert len(goal_events) == 1

    def test_new_path_resets_state(self):
        """Receiving a new path resets waypoint index and goal_reached."""
        mod = PathAdapterModule(waypoint_distance=5.0, arrival_threshold=0.5)
        mod.setup()

        mod.odometry._deliver(_make_odom(0, 0))
        path = _make_path([(0, 0, 0), (1, 0, 0)])
        mod.global_path._deliver(path)

        mod.tick()
        mod.odometry._deliver(_make_odom(1, 0))
        mod.tick()
        assert mod.goal_reached is True

        path2 = _make_path([(0, 0, 0), (5, 0, 0)])
        mod.global_path._deliver(path2)
        assert mod.goal_reached is False
        assert mod.current_waypoint_idx == 0

    def test_no_tick_without_odom(self):
        """tick() is no-op without odometry."""
        mod = PathAdapterModule()
        mod.setup()
        waypoints = _collect_events(mod, "waypoint")

        path = _make_path([(0, 0, 0), (1, 0, 0)])
        mod.global_path._deliver(path)
        mod.tick()

        assert len(waypoints) == 0


# ============================================================================
# TestPathAdapterModule -- Stuck Detection
# ============================================================================

class TestPathAdapterStuckDetection:
    """Stuck detection with WARN_STUCK and STUCK states."""

    def test_warn_stuck_at_half_timeout(self):
        """WARN_STUCK fires at half the stuck_timeout."""
        mod = PathAdapterModule(
            waypoint_distance=5.0,
            arrival_threshold=0.3,
            stuck_timeout=2.0,
            stuck_dist_thre=0.15,
        )
        mod.setup()
        events = _collect_events(mod, "adapter_status")

        # Robot at (2,0) -- not at any waypoint (far from 10,0,0 target)
        mod.odometry._deliver(_make_odom(2, 0))
        path = _make_path([(0, 0, 0), (10, 0, 0)])
        mod.global_path._deliver(path)

        # First tick: robot at (2,0), wp0 at (0,0) dist=2.0 > 0.3, so no advance.
        # Advance past wp0 manually by placing robot near wp0 first.
        mod.odometry._deliver(_make_odom(0, 0))
        mod.tick()  # arrives at wp0, advances to wp1

        # Now robot at (0,0), target is wp1 at (10,0,0), dist=10 >> 0.3
        # Simulate time passing without robot moving
        base_time = time.monotonic()
        with patch("time.monotonic", return_value=base_time + 1.5):
            mod.tick()

        warn_events = [e for e in events if e["event"] == "warn_stuck"]
        assert len(warn_events) == 1

    def test_stuck_at_full_timeout(self):
        """STUCK fires at full stuck_timeout."""
        mod = PathAdapterModule(
            waypoint_distance=5.0,
            arrival_threshold=0.3,
            stuck_timeout=2.0,
            stuck_dist_thre=0.15,
        )
        mod.setup()
        events = _collect_events(mod, "adapter_status")

        # Advance past wp0 so we are tracking wp1 at (10,0,0)
        mod.odometry._deliver(_make_odom(0, 0))
        path = _make_path([(0, 0, 0), (10, 0, 0)])
        mod.global_path._deliver(path)
        mod.tick()  # arrives at wp0, advances to wp1

        base_time = time.monotonic()
        with patch("time.monotonic", return_value=base_time + 2.5):
            mod.tick()

        stuck_events = [e for e in events if e["event"] == "stuck"]
        assert len(stuck_events) == 1

    def test_movement_resets_stuck(self):
        """Robot moving beyond stuck_dist_thre resets the stuck timer."""
        mod = PathAdapterModule(
            waypoint_distance=5.0,
            arrival_threshold=0.3,
            stuck_timeout=2.0,
            stuck_dist_thre=0.15,
        )
        mod.setup()
        events = _collect_events(mod, "adapter_status")

        # Advance past wp0
        mod.odometry._deliver(_make_odom(0, 0))
        path = _make_path([(0, 0, 0), (10, 0, 0)])
        mod.global_path._deliver(path)
        mod.tick()  # arrives at wp0, advances to wp1

        # Robot moves significantly from (0,0) to (0.5,0)
        mod.odometry._deliver(_make_odom(0.5, 0))

        base_time = time.monotonic()
        with patch("time.monotonic", return_value=base_time + 1.5):
            mod.tick()

        warn_events = [e for e in events if e["event"] == "warn_stuck"]
        assert len(warn_events) == 0

    def test_no_stuck_after_goal_reached(self):
        """Stuck detection is suppressed after goal_reached."""
        mod = PathAdapterModule(
            waypoint_distance=5.0,
            arrival_threshold=0.5,
            stuck_timeout=0.001,
        )
        mod.setup()
        events = _collect_events(mod, "adapter_status")

        mod.odometry._deliver(_make_odom(0, 0))
        path = _make_path([(0, 0, 0), (0.1, 0, 0)])
        mod.global_path._deliver(path)

        mod.tick()
        mod.odometry._deliver(_make_odom(0.1, 0))
        mod.tick()

        assert mod.goal_reached is True
        base_time = time.monotonic()
        with patch("time.monotonic", return_value=base_time + 100):
            mod.tick()

        stuck_events = [e for e in events if e["event"] in ("warn_stuck", "stuck")]
        assert len(stuck_events) == 0


# ============================================================================
# TestPathAdapterModule -- max_first_waypoint_dist
# ============================================================================

class TestPathAdapterMaxFirstWaypointDist:

    def test_far_first_waypoint_rejected(self):
        """Path with first waypoint > max_first_waypoint_dist is rejected."""
        mod = PathAdapterModule(max_first_waypoint_dist=5.0)
        mod.setup()
        events = _collect_events(mod, "adapter_status")

        mod.odometry._deliver(_make_odom(0, 0))
        path = _make_path([(100, 0, 0), (200, 0, 0)])
        mod.global_path._deliver(path)

        assert mod.path_length == 0
        fail_events = [e for e in events if e["event"] == "failed"]
        assert len(fail_events) == 1
        assert fail_events[0]["reason"] == "first_waypoint_too_far"


# ============================================================================
# TestPathAdapterModule -- NaN resilience
# ============================================================================

class TestPathAdapterNaNResilience:

    def test_nan_odom_ignored(self):
        mod = PathAdapterModule()
        mod.setup()
        mod.odometry._deliver(_make_odom(1, 2))
        assert mod.robot_position is not None
        mod.odometry._deliver(Odometry(pose=Pose(position=Vector3(float("nan"), 0, 0))))
        assert mod.robot_position.x == 1.0

    def test_inf_odom_ignored(self):
        mod = PathAdapterModule()
        mod.setup()
        mod.odometry._deliver(_make_odom(1, 2))
        mod.odometry._deliver(Odometry(pose=Pose(position=Vector3(float("inf"), 0, 0))))
        assert mod.robot_position.x == 1.0


# ============================================================================
# TestPathAdapterModule -- Port scanning
# ============================================================================

class TestPathAdapterPortScanning:

    def test_ports_detected(self):
        mod = PathAdapterModule()
        assert "global_path" in mod.ports_in
        assert "odometry" in mod.ports_in
        assert "waypoint" in mod.ports_out
        assert "adapter_status" in mod.ports_out

    def test_port_types(self):
        mod = PathAdapterModule()
        assert mod.ports_in["global_path"].msg_type is Path
        assert mod.ports_in["odometry"].msg_type is Odometry
        assert mod.ports_out["waypoint"].msg_type is PoseStamped
        assert mod.ports_out["adapter_status"].msg_type is dict

    def test_layer(self):
        assert PathAdapterModule._layer == 5
        mod = PathAdapterModule()
        assert mod.layer == 5


# ============================================================================
# TestMissionArcModule -- FSM Transitions
# ============================================================================

class TestMissionArcFSMTransitions:

    def test_initial_state_idle(self):
        mod = MissionArcModule()
        mod.setup()
        assert mod.state == MissionState.IDLE

    def test_goal_starts_planning(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        assert mod.state == MissionState.PLANNING

    def test_planning_success_to_executing(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        assert mod.state == MissionState.EXECUTING

    def test_executing_goal_reached_to_complete(self):
        mod = MissionArcModule()
        mod.setup()
        statuses = _collect_events(mod, "mission_status")

        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        mod.adapter_status._deliver({"event": "goal_reached"})

        assert mod.state == MissionState.COMPLETE
        assert any(s["state"] == "COMPLETE" for s in statuses)

    def test_planning_failed(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("FAILED")
        assert mod.state == MissionState.FAILED

    def test_executing_stuck_to_recovering(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        mod.planner_status._deliver("WARN_STUCK")
        assert mod.state == MissionState.RECOVERING

    def test_executing_goal_reached_via_planner(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        mod.planner_status._deliver("GOAL_REACHED")
        assert mod.state == MissionState.COMPLETE

    def test_full_lifecycle(self):
        """IDLE -> PLANNING -> EXECUTING -> COMPLETE."""
        mod = MissionArcModule()
        mod.setup()
        assert mod.state == MissionState.IDLE

        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        assert mod.state == MissionState.PLANNING

        mod.planner_status._deliver("SUCCESS")
        assert mod.state == MissionState.EXECUTING

        mod.adapter_status._deliver({"event": "path_received", "total": 5})
        mod.adapter_status._deliver({"event": "waypoint_reached", "index": 2, "total": 5})
        mod.adapter_status._deliver({"event": "goal_reached"})
        assert mod.state == MissionState.COMPLETE


# ============================================================================
# TestMissionArcModule -- Replan
# ============================================================================

class TestMissionArcReplan:

    def test_stuck_triggers_recovery(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        mod.planner_status._deliver("STUCK")
        assert mod.state == MissionState.RECOVERING

    def test_recovery_timeout_triggers_replan(self):
        mod = MissionArcModule(recovery_timeout_sec=1.0)
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        mod.planner_status._deliver("STUCK")
        assert mod.state == MissionState.RECOVERING

        base = time.monotonic()
        with patch("time.monotonic", return_value=base + 2.0):
            mod.tick()
        assert mod.state == MissionState.REPLANNING

    def test_replan_success_back_to_executing(self):
        mod = MissionArcModule(recovery_timeout_sec=0.001)
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        mod.planner_status._deliver("STUCK")

        base = time.monotonic()
        with patch("time.monotonic", return_value=base + 1.0):
            mod.tick()
        assert mod.state == MissionState.REPLANNING

        mod.planner_status._deliver("SUCCESS")
        assert mod.state == MissionState.EXECUTING

    def test_max_replan_exceeded_fails(self):
        mod = MissionArcModule(max_replan_count=1, recovery_timeout_sec=0.001)
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")

        mod.planner_status._deliver("STUCK")
        base = time.monotonic()
        with patch("time.monotonic", return_value=base + 1.0):
            mod.tick()
        assert mod.state == MissionState.REPLANNING
        assert mod.replan_count == 1

        mod.planner_status._deliver("SUCCESS")
        assert mod.state == MissionState.EXECUTING

        mod.planner_status._deliver("STUCK")
        base2 = time.monotonic()
        with patch("time.monotonic", return_value=base2 + 1.0):
            mod.tick()
        assert mod.state == MissionState.FAILED
        assert "exceeded max replan" in mod.failure_reason


# ============================================================================
# TestMissionArcModule -- Timeouts
# ============================================================================

class TestMissionArcTimeouts:

    def test_planning_timeout(self):
        mod = MissionArcModule(planning_timeout_sec=1.0)
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        assert mod.state == MissionState.PLANNING

        base = time.monotonic()
        with patch("time.monotonic", return_value=base + 2.0):
            mod.tick()
        assert mod.state == MissionState.FAILED
        assert "planning timeout" in mod.failure_reason

    def test_mission_timeout(self):
        mod = MissionArcModule(mission_timeout_sec=5.0)
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        assert mod.state == MissionState.EXECUTING

        base = time.monotonic()
        with patch("time.monotonic", return_value=base + 10.0):
            mod.tick()
        assert mod.state == MissionState.FAILED
        assert "mission timeout" in mod.failure_reason


# ============================================================================
# TestMissionArcModule -- Cancel
# ============================================================================

class TestMissionArcCancel:

    def test_cancel_from_executing(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        assert mod.state == MissionState.EXECUTING

        mod.cancel._deliver({})
        assert mod.state == MissionState.IDLE

    def test_cancel_from_idle_noop(self):
        mod = MissionArcModule()
        mod.setup()
        statuses = _collect_events(mod, "mission_status")
        mod.cancel._deliver({})
        assert mod.state == MissionState.IDLE
        assert len(statuses) == 0


# ============================================================================
# TestMissionArcModule -- Patrol
# ============================================================================

class TestMissionArcPatrol:

    def test_patrol_starts(self):
        mod = MissionArcModule()
        mod.setup()
        goals = _collect_events(mod, "goal_pose_out")

        mod.patrol_goals._deliver({
            "waypoints": [
                {"x": 1, "y": 2, "name": "WP0"},
                {"x": 5, "y": 6, "name": "WP1"},
            ],
            "loop": False,
            "route_name": "test_route",
        })

        assert mod.state == MissionState.PLANNING
        assert len(goals) == 1
        assert goals[0].x == 1.0

    def test_patrol_advances(self):
        mod = MissionArcModule()
        mod.setup()
        goals = _collect_events(mod, "goal_pose_out")

        mod.patrol_goals._deliver({
            "waypoints": [
                {"x": 1, "y": 2, "name": "WP0"},
                {"x": 5, "y": 6, "name": "WP1"},
            ],
            "loop": False,
        })

        mod.planner_status._deliver("SUCCESS")
        mod.adapter_status._deliver({"event": "goal_reached"})
        assert mod.state == MissionState.PLANNING
        assert len(goals) == 2

    def test_patrol_loop(self):
        mod = MissionArcModule()
        mod.setup()
        goals = _collect_events(mod, "goal_pose_out")

        mod.patrol_goals._deliver({
            "waypoints": [{"x": 1, "y": 2}],
            "loop": True,
        })

        mod.planner_status._deliver("SUCCESS")
        mod.adapter_status._deliver({"event": "goal_reached"})
        assert mod.state == MissionState.PLANNING
        assert len(goals) == 2

    def test_patrol_empty_ignored(self):
        mod = MissionArcModule()
        mod.setup()
        mod.patrol_goals._deliver({"waypoints": []})
        assert mod.state == MissionState.IDLE

    def test_patrol_nan_waypoints_filtered(self):
        mod = MissionArcModule()
        mod.setup()
        mod.patrol_goals._deliver({
            "waypoints": [
                {"x": float("nan"), "y": 0},
                {"x": 1, "y": 2},
            ],
        })
        assert mod.state == MissionState.PLANNING


# ============================================================================
# TestMissionArcModule -- NaN goal / E-stop / Recovery / Ports
# ============================================================================

class TestMissionArcNaNGoal:

    def test_nan_goal_rejected(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(
            pose=Pose(position=Vector3(float("nan"), 10, 0)), frame_id="map"
        )
        mod.goal_pose._deliver(goal)
        assert mod.state == MissionState.IDLE

    def test_inf_goal_rejected(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(
            pose=Pose(position=Vector3(float("inf"), 10, 0)), frame_id="map"
        )
        mod.goal_pose._deliver(goal)
        assert mod.state == MissionState.IDLE


class TestMissionArcEStop:

    def test_estop_tracked(self):
        mod = MissionArcModule()
        mod.setup()
        assert mod.estop_active is False
        mod.stop_signal._deliver(2)
        assert mod.estop_active is True
        mod.stop_signal._deliver(0)
        assert mod.estop_active is False


class TestMissionArcRecoveryFromAdapter:

    def test_waypoint_reached_during_recovery(self):
        """waypoint_reached during RECOVERING -> back to EXECUTING."""
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        mod.planner_status._deliver("WARN_STUCK")
        assert mod.state == MissionState.RECOVERING

        mod.adapter_status._deliver({"event": "waypoint_reached", "index": 2, "total": 5})
        assert mod.state == MissionState.EXECUTING


class TestMissionArcPortScanning:

    def test_ports_detected(self):
        mod = MissionArcModule()
        assert "goal_pose" in mod.ports_in
        assert "planner_status" in mod.ports_in
        assert "adapter_status" in mod.ports_in
        assert "odometry" in mod.ports_in
        assert "stop_signal" in mod.ports_in
        assert "cancel" in mod.ports_in
        assert "patrol_goals" in mod.ports_in
        assert "mission_status" in mod.ports_out
        assert "goal_pose_out" in mod.ports_out

    def test_layer(self):
        assert MissionArcModule._layer == 5
        mod = MissionArcModule()
        assert mod.layer == 5

    def test_status_published_on_tick(self):
        mod = MissionArcModule()
        mod.setup()
        statuses = _collect_events(mod, "mission_status")
        mod.tick()
        assert len(statuses) >= 1
        assert statuses[0]["state"] == "IDLE"


class TestMissionArcAdapterEvents:

    def test_stuck_final_in_executing(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")
        assert mod.state == MissionState.EXECUTING

        mod.adapter_status._deliver({"event": "stuck_final"})
        assert mod.state == MissionState.REPLANNING

    def test_adapter_failed_in_executing(self):
        mod = MissionArcModule()
        mod.setup()
        goal = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal)
        mod.planner_status._deliver("SUCCESS")

        mod.adapter_status._deliver({"event": "failed", "reason": "test_fail"})
        assert mod.state == MissionState.FAILED
        assert mod.failure_reason == "test_fail"


class TestMissionArcGoalOverride:

    def test_goal_during_executing_overrides(self):
        mod = MissionArcModule()
        mod.setup()
        goal1 = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal1)
        mod.planner_status._deliver("SUCCESS")
        assert mod.state == MissionState.EXECUTING

        goal2 = PoseStamped(pose=Pose(position=Vector3(20, 30, 0)), frame_id="map")
        mod.goal_pose._deliver(goal2)
        assert mod.state == MissionState.PLANNING

    def test_goal_during_planning_overrides(self):
        mod = MissionArcModule()
        mod.setup()
        goal1 = PoseStamped(pose=Pose(position=Vector3(5, 10, 0)), frame_id="map")
        mod.goal_pose._deliver(goal1)
        assert mod.state == MissionState.PLANNING

        goal2 = PoseStamped(pose=Pose(position=Vector3(20, 30, 0)), frame_id="map")
        mod.goal_pose._deliver(goal2)
        assert mod.state == MissionState.PLANNING
