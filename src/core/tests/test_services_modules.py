"""Tests for nav_services and reconstruction Module conversions.

Covers:
- Port declarations for all 5 modules (MapManager, Patrol, Geofence, TaskScheduler, Reconstruction)
- MapManagerModule basic command handling (list, delete, poi)
- PatrolManagerModule port types (In[str], In[Odometry], Out[list], Out[str])
"""

import json
import tempfile
import os

import pytest

from src.core import In, Out, Module
from src.core.msgs.nav import Odometry
from src.core.msgs.sensor import Image, CameraIntrinsics
from src.core.msgs.semantic import SceneGraph

from nav.services.nav_services.map_manager_module import MapManagerModule
from nav.services.nav_services.patrol_manager_module import PatrolManagerModule
from nav.services.nav_services.geofence_manager_module import GeofenceManagerModule
from nav.services.nav_services.task_scheduler_module import TaskSchedulerModule
from src.reconstruction.reconstruction.reconstruction_module import ReconstructionModule


# ============================================================================
# Helpers
# ============================================================================

def _collect(module, port_name):
    """Attach a collector callback to an Out port and return the events list."""
    events = []
    port = getattr(module, port_name)
    port._add_callback(events.append)
    return events


# ============================================================================
# Port declaration tests
# ============================================================================

class TestPortDeclarations:

    def test_map_manager_ports(self):
        m = MapManagerModule(data_dir=tempfile.mkdtemp(), map_dir=tempfile.mkdtemp())
        assert "map_command" in m.ports_in
        assert "map_response" in m.ports_out
        assert m.ports_in["map_command"].msg_type is str
        assert m.ports_out["map_response"].msg_type is dict
        assert m.layer == 6

    def test_patrol_manager_ports(self):
        m = PatrolManagerModule(routes_dir=tempfile.mkdtemp())
        assert "patrol_command" in m.ports_in
        assert "odometry" in m.ports_in
        assert "patrol_goals" in m.ports_out
        assert "patrol_status" in m.ports_out
        assert m.ports_in["patrol_command"].msg_type is str
        assert m.ports_in["odometry"].msg_type is Odometry
        assert m.ports_out["patrol_goals"].msg_type is list
        assert m.ports_out["patrol_status"].msg_type is str
        assert m.layer == 6

    def test_geofence_manager_ports(self):
        m = GeofenceManagerModule(geofence_file=os.path.join(tempfile.mkdtemp(), "gf.yaml"))
        assert "odometry" in m.ports_in
        assert "geofence_command" in m.ports_in
        assert "geofence_alert" in m.ports_out
        assert m.ports_in["odometry"].msg_type is Odometry
        assert m.layer == 6

    def test_task_scheduler_ports(self):
        m = TaskSchedulerModule(schedule_file=os.path.join(tempfile.mkdtemp(), "sched.yaml"))
        assert "schedule_command" in m.ports_in
        assert "scheduled_task" in m.ports_out
        assert m.ports_in["schedule_command"].msg_type is str
        assert m.ports_out["scheduled_task"].msg_type is dict
        assert m.layer == 6

    def test_reconstruction_ports(self):
        m = ReconstructionModule()
        assert "color_image" in m.ports_in
        assert "depth_image" in m.ports_in
        assert "camera_info" in m.ports_in
        assert "scene_graph" in m.ports_in
        assert "odometry" in m.ports_in
        assert "semantic_cloud" in m.ports_out
        assert "reconstruction_stats" in m.ports_out
        assert m.ports_in["color_image"].msg_type is Image
        assert m.ports_in["scene_graph"].msg_type is SceneGraph
        assert m.ports_in["camera_info"].msg_type is CameraIntrinsics
        assert m.layer == 3


# ============================================================================
# MapManagerModule functional tests
# ============================================================================

class TestMapManagerModule:

    def test_list_empty(self):
        m = MapManagerModule(data_dir=tempfile.mkdtemp(), map_dir=tempfile.mkdtemp())
        m.setup()
        events = _collect(m, "map_response")
        m.map_command._deliver(json.dumps({"action": "list"}))
        assert len(events) == 1
        assert events[0]["success"] is True
        assert events[0]["maps"] == []

    def test_delete_nonexistent(self):
        m = MapManagerModule(data_dir=tempfile.mkdtemp(), map_dir=tempfile.mkdtemp())
        m.setup()
        events = _collect(m, "map_response")
        m.map_command._deliver(json.dumps({"action": "delete", "name": "ghost"}))
        assert events[0]["success"] is False

    def test_poi_crud(self):
        m = MapManagerModule(data_dir=tempfile.mkdtemp(), map_dir=tempfile.mkdtemp())
        m.setup()
        events = _collect(m, "map_response")

        m.map_command._deliver(json.dumps({"action": "poi_set", "name": "office", "x": 1.0, "y": 2.0}))
        assert events[-1]["success"] is True

        m.map_command._deliver(json.dumps({"action": "poi_list"}))
        assert "office" in events[-1]["pois"]

        m.map_command._deliver(json.dumps({"action": "poi_delete", "name": "office"}))
        assert events[-1]["success"] is True


# ============================================================================
# PatrolManagerModule port type tests
# ============================================================================

class TestPatrolManagerModule:

    def test_port_types_correct(self):
        m = PatrolManagerModule(routes_dir=tempfile.mkdtemp())
        assert isinstance(m.patrol_command, In)
        assert isinstance(m.odometry, In)
        assert isinstance(m.patrol_goals, Out)
        assert isinstance(m.patrol_status, Out)

    def test_list_empty_routes(self):
        m = PatrolManagerModule(routes_dir=tempfile.mkdtemp())
        m.setup()
        events = _collect(m, "patrol_status")
        m.patrol_command._deliver(json.dumps({"action": "list"}))
        resp = json.loads(events[0])
        assert resp["success"] is True
        assert resp["routes"] == []
