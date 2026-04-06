"""Tests for memory/ modules — SemanticMapperModule, VectorMemoryModule,
EpisodicMemoryModule, TaggedLocationsModule, TemporalMemoryModule, TopologicalMemoryModule.

All tests are pure-Python, no ROS2 / hardware / CLIP / ChromaDB required.
"""

from __future__ import annotations

import json
import os
import tempfile
import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from core.msgs.geometry import Pose, Quaternion, Vector3
from core.msgs.nav import Odometry
from core.msgs.semantic import Detection3D, Region, SceneGraph


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_odom(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Odometry:
    return Odometry(
        pose=Pose(position=Vector3(x, y, z), orientation=Quaternion(0, 0, 0, 1)),
    )


def _make_scene_graph(
    labels: list[str] | None = None,
    regions: list[Region] | None = None,
) -> SceneGraph:
    """Build a SceneGraph with named objects and optional regions."""
    objects = []
    if labels:
        for i, lbl in enumerate(labels):
            objects.append(
                Detection3D(
                    id=f"obj_{i}",
                    label=lbl,
                    confidence=0.9,
                    position=Vector3(float(i), float(i), 0.0),
                )
            )
    return SceneGraph(objects=objects, regions=regions or [])


# ---------------------------------------------------------------------------
# 1. SemanticMapperModule
# ---------------------------------------------------------------------------

class TestSemanticMapperModule(unittest.TestCase):

    def _make_module(self, **kw):
        # Mock the heavy backends so the module can be tested in isolation
        with patch(
            "memory.modules.semantic_mapper_module.SemanticMapperModule._init_backends"
        ):
            from memory.modules.semantic_mapper_module import SemanticMapperModule
            m = SemanticMapperModule(**kw)
            m._kg = None
            m._tsg = None
            m.setup()
        return m

    def test_odom_updates_robot_xy(self):
        m = self._make_module()
        m.odometry._deliver(_make_odom(3.5, -1.2))
        self.assertAlmostEqual(m._robot_xy[0], 3.5)
        self.assertAlmostEqual(m._robot_xy[1], -1.2)

    def test_scene_graph_no_regions_is_noop(self):
        """Empty regions should not increment sg_count."""
        m = self._make_module()
        sg = _make_scene_graph(labels=["chair"], regions=[])
        m.scene_graph._deliver(sg)
        self.assertEqual(m._sg_count, 0)

    def test_scene_graph_with_region_increments_count(self):
        """A SceneGraph with regions should increment sg_count."""
        m = self._make_module()
        # Provide a mock TSG so _update_tsg doesn't bail early
        mock_tsg = MagicMock()
        mock_tsg.to_prompt_context.return_value = "room summary"
        mock_tsg.to_dict.return_value = {}
        m._tsg = mock_tsg

        region = Region(
            name="living_room",
            object_ids=["obj_0"],
            center=Vector3(1.0, 1.0, 0.0),
        )
        sg = _make_scene_graph(labels=["sofa"], regions=[region])
        m.scene_graph._deliver(sg)
        self.assertEqual(m._sg_count, 1)

    def test_stable_room_id_consistency(self):
        m = self._make_module()
        id1 = m._stable_room_id("kitchen")
        id2 = m._stable_room_id("bedroom")
        id3 = m._stable_room_id("kitchen")
        self.assertEqual(id1, id3, "Same name should return same ID")
        self.assertNotEqual(id1, id2, "Different names should have different IDs")

    def test_health_returns_expected_keys(self):
        m = self._make_module()
        h = m.health()
        self.assertIn("semantic_mapper", h)
        sm = h["semantic_mapper"]
        for key in ("save_dir", "kg_rooms", "tsg_nodes", "sg_updates", "last_save"):
            self.assertIn(key, sm, f"Missing key: {key}")
        self.assertEqual(sm["sg_updates"], 0)


# ---------------------------------------------------------------------------
# 2. VectorMemoryModule
# ---------------------------------------------------------------------------

class TestVectorMemoryModule(unittest.TestCase):

    def _make_module(self, **kw):
        with patch(
            "memory.modules.vector_memory_module.VectorMemoryModule._init_encoder"
        ), patch(
            "memory.modules.vector_memory_module.VectorMemoryModule._init_store"
        ):
            from memory.modules.vector_memory_module import VectorMemoryModule
            defaults = dict(store_interval=0.0, max_np_entries=10)
            defaults.update(kw)
            m = VectorMemoryModule(**defaults)
            m._encoder = None
            m._use_chromadb = False
            m.setup()
        return m

    def test_store_and_query_hash_fallback(self):
        """Store a snapshot with hash embedding and query it back."""
        m = self._make_module()
        # Manually store a snapshot
        m._robot_xy = (5.0, 10.0)
        m._store_snapshot(["backpack", "bench"])
        self.assertEqual(m._store_count, 1)
        self.assertEqual(len(m._np_embeddings), 1)

        # Query should find it
        results = m._query("backpack")
        self.assertGreater(len(results), 0)
        self.assertAlmostEqual(results[0]["x"], 5.0)
        self.assertAlmostEqual(results[0]["y"], 10.0)

    def test_lru_eviction(self):
        """When np entries exceed max_np_entries, oldest are evicted."""
        m = self._make_module(max_np_entries=5)
        for i in range(8):
            m._robot_xy = (float(i), 0.0)
            m._store_snapshot([f"obj_{i}"])

        self.assertEqual(len(m._np_embeddings), 5)
        self.assertEqual(len(m._np_metadata), 5)
        # Oldest entries (0,1,2) should be gone; metadata[0] should be obj_3
        self.assertAlmostEqual(m._np_metadata[0]["x"], 3.0)

    def test_query_empty_returns_empty(self):
        m = self._make_module()
        results = m._query("anything")
        self.assertEqual(results, [])

    def test_hash_embedding_deterministic(self):
        from memory.modules.vector_memory_module import VectorMemoryModule
        v1 = VectorMemoryModule._hash_embedding("backpack bench")
        v2 = VectorMemoryModule._hash_embedding("backpack bench")
        np.testing.assert_array_equal(v1, v2)

    def test_hash_embedding_normalized(self):
        from memory.modules.vector_memory_module import VectorMemoryModule
        v = VectorMemoryModule._hash_embedding("chair table lamp")
        norm = float(np.linalg.norm(v))
        self.assertAlmostEqual(norm, 1.0, places=5)

    def test_get_memory_stats_numpy_backend(self):
        m = self._make_module()
        m._robot_xy = (1.0, 2.0)
        m._store_snapshot(["cup"])
        stats = m.get_memory_stats()
        self.assertEqual(stats["backend"], "numpy")
        self.assertEqual(stats["entries"], 1)
        self.assertEqual(stats["store_count"], 1)


# ---------------------------------------------------------------------------
# 3. EpisodicMemoryModule
# ---------------------------------------------------------------------------

class TestEpisodicMemoryModule(unittest.TestCase):

    def _make_module(self, **kw):
        from memory.modules.episodic_module import EpisodicMemoryModule
        defaults = dict(max_records=50, min_distance_m=1.0)
        defaults.update(kw)
        m = EpisodicMemoryModule(**defaults)
        m.setup()
        return m

    def test_stores_episode_on_scene_graph(self):
        m = self._make_module()
        m.odometry._deliver(_make_odom(5.0, 5.0))
        sg = _make_scene_graph(labels=["chair", "table"])
        m.scene_graph._deliver(sg)
        self.assertEqual(len(m.memory), 1)

    def test_position_dedup(self):
        """Two scene graphs at same position should only create one record."""
        m = self._make_module(min_distance_m=2.0)
        m.odometry._deliver(_make_odom(1.0, 1.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["a"]))
        m.scene_graph._deliver(_make_scene_graph(labels=["b"]))
        self.assertEqual(len(m.memory), 1, "Same position should dedup")

    def test_different_positions_create_records(self):
        m = self._make_module(min_distance_m=1.0)
        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["a"]))
        m.odometry._deliver(_make_odom(10.0, 10.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["b"]))
        self.assertEqual(len(m.memory), 2)

    def test_no_odom_no_record(self):
        """Without odometry, scene_graph should not create a record."""
        m = self._make_module()
        m.scene_graph._deliver(_make_scene_graph(labels=["x"]))
        self.assertEqual(len(m.memory), 0)

    def test_timestamp_ordering(self):
        m = self._make_module(min_distance_m=0.5)
        for i in range(5):
            m.odometry._deliver(_make_odom(float(i) * 2, 0.0))
            m.scene_graph._deliver(_make_scene_graph(labels=[f"obj_{i}"]))
        records = m.memory.recent_n(5)
        timestamps = [r.timestamp for r in records]
        self.assertEqual(timestamps, sorted(timestamps), "Records should be time-ordered")

    def test_memory_context_published(self):
        m = self._make_module()
        results = []
        m.memory_context._add_callback(lambda msg: results.append(msg))

        m.odometry._deliver(_make_odom(3.0, 4.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["bench"]))
        self.assertGreater(len(results), 0, "memory_context should be published")

    def test_health_returns_expected_keys(self):
        m = self._make_module()
        h = m.health()
        self.assertIn("record_count", h)
        self.assertEqual(h["record_count"], 0)
        self.assertIn("oldest_ts", h)
        self.assertIsNone(h["oldest_ts"])


# ---------------------------------------------------------------------------
# 4. TaggedLocationsModule
# ---------------------------------------------------------------------------

class TestTaggedLocationsModule(unittest.TestCase):

    def _make_module(self, **kw):
        from memory.modules.tagged_locations_module import TaggedLocationsModule
        m = TaggedLocationsModule(**kw)
        m.setup()
        return m

    def test_save_and_goto(self):
        m = self._make_module()
        statuses = []
        m.tag_status._add_callback(lambda s: statuses.append(s))

        # Supply odom then save
        m.odometry._deliver(_make_odom(10.0, 20.0, 0.5))
        m.tag_command._deliver("save:gym")
        self.assertIn("saved:gym", statuses)

        # Goto
        locations = []
        m.saved_location._add_callback(lambda loc: locations.append(loc))
        m.tag_command._deliver("goto:gym")
        self.assertIn("recalled:gym", statuses)
        self.assertEqual(len(locations), 1)

    def test_remove(self):
        m = self._make_module()
        m.odometry._deliver(_make_odom(1.0, 2.0))
        m.tag_command._deliver("save:cafe")

        statuses = []
        m.tag_status._add_callback(lambda s: statuses.append(s))
        m.tag_command._deliver("remove:cafe")
        self.assertIn("removed:cafe", statuses)

        # Goto should now fail
        m.tag_command._deliver("goto:cafe")
        self.assertTrue(
            any("not_found" in s for s in statuses),
            "After remove, goto should report not_found",
        )

    def test_fuzzy_matching(self):
        m = self._make_module()
        m.odometry._deliver(_make_odom(5.0, 5.0))
        m.tag_command._deliver("save:basketball court")

        statuses = []
        m.tag_status._add_callback(lambda s: statuses.append(s))
        locations = []
        m.saved_location._add_callback(lambda loc: locations.append(loc))

        # "basketball" is contained in "basketball court"
        m.tag_command._deliver("goto:basketball")
        self.assertTrue(
            any("recalled" in s for s in statuses),
            "Fuzzy match should recall the location",
        )

    def test_list_command(self):
        m = self._make_module()
        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.tag_command._deliver("save:alpha")
        m.tag_command._deliver("save:beta")

        statuses = []
        m.tag_status._add_callback(lambda s: statuses.append(s))
        m.tag_command._deliver("list")
        # The last status should list locations
        self.assertTrue(
            any(s.startswith("locations:") for s in statuses),
            f"Expected 'locations:...' status, got {statuses}",
        )

    def test_save_load_json(self):
        with tempfile.NamedTemporaryFile(suffix=".json", delete=False) as f:
            path = f.name
        try:
            m = self._make_module(json_path=path)
            m.odometry._deliver(_make_odom(7.0, 8.0))
            m.tag_command._deliver("save:park")
            m.store.save()

            # Create a new store from the same file
            from memory.spatial.tagged_locations import TaggedLocationStore
            store2 = TaggedLocationStore(json_path=path)
            result = store2.query("park")
            self.assertIsNotNone(result)
            self.assertAlmostEqual(result["position"][0], 7.0)
        finally:
            os.unlink(path)

    def test_health_returns_expected_keys(self):
        m = self._make_module()
        h = m.health()
        self.assertIn("location_count", h)
        self.assertEqual(h["location_count"], 0)


# ---------------------------------------------------------------------------
# 5. TemporalMemoryModule
# ---------------------------------------------------------------------------

class TestTemporalMemoryModule(unittest.TestCase):

    def _make_module(self, **kw):
        from memory.modules.temporal_memory_module import TemporalMemoryModule
        defaults = dict(
            max_records=20,
            min_observation_interval=0.0,
            summary_interval=9999.0,  # disable auto-summary timer
        )
        defaults.update(kw)
        m = TemporalMemoryModule(**defaults)
        m.setup()
        return m

    def test_buffer_stores_records(self):
        m = self._make_module()
        m.odometry._deliver(_make_odom(1.0, 2.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["cat", "dog"]))
        self.assertEqual(len(m._buffer), 1)
        rec = m._buffer[0]
        self.assertEqual(len(rec.objects), 2)

    def test_deque_overflow(self):
        """Buffer should not exceed max_records."""
        m = self._make_module(max_records=5, min_observation_interval=0.0)
        for i in range(10):
            m.odometry._deliver(_make_odom(float(i), 0.0))
            m.scene_graph._deliver(_make_scene_graph(labels=[f"obj_{i}"]))
        self.assertEqual(len(m._buffer), 5)
        # Oldest should be obj_5
        self.assertEqual(m._buffer[0].objects[0]["label"], "obj_5")

    def test_min_observation_interval_dedup(self):
        m = self._make_module(min_observation_interval=10.0)
        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["a"]))
        m.scene_graph._deliver(_make_scene_graph(labels=["b"]))
        # Second delivery should be ignored due to interval
        self.assertEqual(len(m._buffer), 1)

    def test_query_by_label(self):
        m = self._make_module()
        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["person", "car"]))
        m.scene_graph._deliver(_make_scene_graph(labels=["tree"]))

        results = m.query_by_label("person")
        self.assertEqual(len(results), 1)
        results2 = m.query_by_label("tree")
        self.assertEqual(len(results2), 1)

    def test_query_last_seen(self):
        m = self._make_module()
        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["person"]))
        m.scene_graph._deliver(_make_scene_graph(labels=["car"]))

        rec = m.query_last_seen("person")
        self.assertIsNotNone(rec)
        self.assertIsNone(m.query_last_seen("helicopter"))

    def test_memory_context_published(self):
        m = self._make_module()
        results = []
        m.memory_context._add_callback(lambda msg: results.append(msg))
        m.odometry._deliver(_make_odom(1.0, 1.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["lamp"]))
        self.assertGreater(len(results), 0)

    def test_health_returns_expected_keys(self):
        m = self._make_module()
        h = m.health()
        self.assertIn("buffer_size", h)
        self.assertEqual(h["buffer_size"], 0)
        self.assertIn("timer_active", h)

    def test_stop_cancels_timer(self):
        m = self._make_module(summary_interval=1.0)
        m.start()
        self.assertIsNotNone(m._summary_timer)
        m.stop()
        self.assertFalse(
            m._summary_timer is not None and m._summary_timer.is_alive(),
            "Timer should be cancelled after stop()",
        )


# ---------------------------------------------------------------------------
# 6. TopologicalMemoryModule
# ---------------------------------------------------------------------------

class TestTopologicalMemoryModule(unittest.TestCase):

    def _make_module(self, **kw):
        from memory.modules.topological_module import TopologicalMemoryModule
        defaults = dict(new_node_distance=2.0, max_nodes=100)
        defaults.update(kw)
        m = TopologicalMemoryModule(**defaults)
        m.setup()
        return m

    def test_node_creation_on_movement(self):
        m = self._make_module(new_node_distance=1.0)
        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["tree"]))
        # First update creates a node
        self.assertGreaterEqual(len(m.memory.nodes), 1)

    def test_edge_creation_between_nodes(self):
        m = self._make_module(new_node_distance=1.0)
        # Move to create first node
        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["a"]))

        # Move far enough to create second node
        m.odometry._deliver(_make_odom(5.0, 5.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["b"]))

        nodes = m.memory.nodes
        self.assertGreaterEqual(len(nodes), 2, "Should have at least 2 nodes")

        # Check that edges exist between them
        has_edge = False
        for n in nodes.values():
            if len(n.neighbors) > 0:
                has_edge = True
                break
        self.assertTrue(has_edge, "Nodes should be connected by edges")

    def test_room_transitions(self):
        """Nodes in different rooms should generate room transitions."""
        m = self._make_module(new_node_distance=1.0)

        # Create first node in room 0
        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["desk"]))
        # Manually set room on current node
        mem = m.memory
        node0 = mem.current_node
        if node0:
            node0.room_id = 0
            node0.room_name = "office"

        # Move far and create second node in room 1
        m.odometry._deliver(_make_odom(20.0, 20.0))
        # Directly use memory API to set room info
        new_node = mem.update_position(
            position=np.array([20.0, 20.0, 0.0]),
            visible_labels=["sofa"],
            room_id=1,
            room_name="living_room",
        )
        transitions = mem.get_room_transitions()
        self.assertGreater(
            len(transitions), 0,
            "Should record room transition from office to living_room",
        )

    def test_topo_summary_published(self):
        m = self._make_module(new_node_distance=1.0)
        results = []
        m.topo_summary._add_callback(lambda msg: results.append(msg))

        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["bench"]))
        # At least one summary should have been published
        self.assertGreater(len(results), 0)

    def test_topo_graph_published(self):
        m = self._make_module(new_node_distance=1.0)
        results = []
        m.topo_graph._add_callback(lambda msg: results.append(msg))

        m.odometry._deliver(_make_odom(0.0, 0.0))
        m.scene_graph._deliver(_make_scene_graph(labels=["sign"]))
        self.assertGreater(len(results), 0)
        self.assertIsInstance(results[0], dict)

    def test_health_returns_expected_keys(self):
        m = self._make_module()
        h = m.health()
        self.assertIn("node_count", h)
        self.assertIn("edge_count", h)
        self.assertEqual(h["node_count"], 0)

    def test_max_nodes_pruning(self):
        """Exceeding max_nodes should trigger pruning."""
        m = self._make_module(new_node_distance=0.5, max_nodes=5)
        mem = m.memory
        for i in range(10):
            mem.update_position(
                position=np.array([float(i) * 5.0, 0.0, 0.0]),
                visible_labels=[f"obj_{i}"],
            )
        self.assertLessEqual(len(mem.nodes), 5, "Nodes should be pruned to max_nodes")


if __name__ == "__main__":
    unittest.main()
