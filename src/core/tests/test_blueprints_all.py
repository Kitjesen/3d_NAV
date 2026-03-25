"""Tests for all three blueprint variants: navigation, simulation, stub.

Verifies:
- StubDogModule / SimDogModule port declarations match ThunderDriver interface
- stub_blueprint builds and starts (fully self-contained, no hardware)
- simulation_blueprint builds (may not start sim server)
- StubDogModule: feed cmd_vel -> get odometry back (dead reckoning)
- SimDogModule: dead-reckoning fallback when no sim server
- All three blueprints produce the same number of core modules
"""

import math
import time
import unittest

from core import Module, In, Out, autoconnect
from core.msgs.geometry import Twist, Vector3, Quaternion, Pose
from core.msgs.nav import Odometry

from core.blueprints.stub import StubDogModule, stub_blueprint
from core.blueprints.simulation import SimDogModule, simulation_blueprint
from core.blueprints.navigation import navigation_blueprint
from drivers.thunder.han_dog_module import ThunderDriver


# ── Port compatibility ────────────────────────────────────────────────

class TestPortCompatibility(unittest.TestCase):
    """StubDogModule and SimDogModule must expose the same core ports
    as ThunderDriver so they are drop-in replacements."""

    REQUIRED_IN = {"cmd_vel", "stop_signal"}
    REQUIRED_OUT = {"odometry", "alive"}

    def _check_ports(self, cls):
        mod = cls()
        in_names = set(mod.ports_in.keys())
        out_names = set(mod.ports_out.keys())
        self.assertTrue(self.REQUIRED_IN.issubset(in_names),
                        f"{cls.__name__} missing In ports: "
                        f"{self.REQUIRED_IN - in_names}")
        self.assertTrue(self.REQUIRED_OUT.issubset(out_names),
                        f"{cls.__name__} missing Out ports: "
                        f"{self.REQUIRED_OUT - out_names}")

    def test_stub_has_required_ports(self):
        self._check_ports(StubDogModule)

    def test_sim_has_required_ports(self):
        self._check_ports(SimDogModule)

    def test_han_dog_has_required_ports(self):
        self._check_ports(ThunderDriver)

    def test_stub_port_types_match(self):
        stub = StubDogModule()
        real = ThunderDriver()
        for name in self.REQUIRED_IN:
            self.assertEqual(
                stub.ports_in[name]._msg_type,
                real.ports_in[name]._msg_type,
                f"In port '{name}' type mismatch")
        for name in self.REQUIRED_OUT:
            self.assertEqual(
                stub.ports_out[name]._msg_type,
                real.ports_out[name]._msg_type,
                f"Out port '{name}' type mismatch")

    def test_sim_port_types_match(self):
        sim = SimDogModule()
        real = ThunderDriver()
        for name in self.REQUIRED_IN:
            self.assertEqual(
                sim.ports_in[name]._msg_type,
                real.ports_in[name]._msg_type,
                f"In port '{name}' type mismatch")
        for name in self.REQUIRED_OUT:
            self.assertEqual(
                sim.ports_out[name]._msg_type,
                real.ports_out[name]._msg_type,
                f"Out port '{name}' type mismatch")

    def test_all_layer_1(self):
        self.assertEqual(StubDogModule._layer, 1)
        self.assertEqual(SimDogModule._layer, 1)
        self.assertEqual(ThunderDriver._layer, 1)


# ── StubDogModule unit tests ─────────────────────────────────────────

class TestStubDogModule(unittest.TestCase):
    """StubDogModule: feed cmd_vel, receive odometry."""

    def test_cmd_vel_produces_odometry(self):
        mod = StubDogModule()
        mod.setup()
        mod._last_ts = time.time() - 0.1  # pretend 100ms ago

        received = []
        mod.odometry._add_callback(received.append)

        twist = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3())
        mod.cmd_vel._deliver(twist)

        self.assertEqual(len(received), 1)
        odom = received[0]
        self.assertIsInstance(odom, Odometry)
        # Moving forward at 1m/s for ~100ms -> ~0.1m
        self.assertGreater(odom.pose.position.x, 0.0)

    def test_zero_cmd_vel(self):
        mod = StubDogModule()
        mod.setup()
        mod._last_ts = time.time()

        received = []
        mod.odometry._add_callback(received.append)

        twist = Twist(linear=Vector3(0.0, 0.0, 0.0), angular=Vector3())
        mod.cmd_vel._deliver(twist)

        self.assertEqual(len(received), 1)
        odom = received[0]
        self.assertAlmostEqual(odom.pose.position.x, 0.0, places=3)
        self.assertAlmostEqual(odom.pose.position.y, 0.0, places=3)

    def test_stop_signal_zeroes_velocity(self):
        mod = StubDogModule()
        mod.setup()
        mod._vx = 1.0
        mod._vy = 0.5
        mod._wz = 0.3

        mod.stop_signal._deliver(2)  # hard stop

        self.assertAlmostEqual(mod._vx, 0.0)
        self.assertAlmostEqual(mod._vy, 0.0)
        self.assertAlmostEqual(mod._wz, 0.0)

    def test_alive_on_start_stop(self):
        mod = StubDogModule()
        mod.setup()
        alive_values = []
        mod.alive._add_callback(alive_values.append)

        mod.start()
        self.assertEqual(alive_values[-1], True)

        mod.stop()
        self.assertEqual(alive_values[-1], False)

    def test_dead_reckoning_yaw(self):
        """Turning should update the heading."""
        mod = StubDogModule()
        mod.setup()
        mod._last_ts = time.time() - 0.1  # 100ms ago

        received = []
        mod.odometry._add_callback(received.append)

        # Turn left at 1 rad/s for ~100ms -> yaw ~0.1 rad
        twist = Twist(linear=Vector3(0.0, 0.0, 0.0),
                      angular=Vector3(0.0, 0.0, 1.0))
        mod.cmd_vel._deliver(twist)

        self.assertEqual(len(received), 1)
        self.assertGreater(abs(mod._yaw), 0.0)

    def test_health_report(self):
        mod = StubDogModule()
        h = mod.health()
        self.assertIn("stub", h)
        self.assertIn("stopped", h["stub"])


# ── SimDogModule unit tests ──────────────────────────────────────────

class TestSimDogModule(unittest.TestCase):
    """SimDogModule: dead-reckoning fallback when no sim server."""

    def test_dead_reckoning_fallback(self):
        """Without a running sim server, cmd_vel should still update position."""
        mod = SimDogModule(sim_host="127.0.0.1", sim_port=19999)
        mod.setup()
        mod._last_ts = time.time() - 0.1

        received = []
        mod.odometry._add_callback(received.append)

        twist = Twist(linear=Vector3(1.0, 0.0, 0.0), angular=Vector3())
        mod.cmd_vel._deliver(twist)

        # Should have used dead reckoning
        self.assertGreater(mod._pos_x, 0.0)

    def test_stop_signal(self):
        mod = SimDogModule()
        mod.setup()
        mod._vx = 1.0
        mod.stop_signal._deliver(1)
        self.assertAlmostEqual(mod._vx, 0.0)

    def test_health_report(self):
        mod = SimDogModule(sim_host="test_host", sim_port=1234)
        h = mod.health()
        self.assertIn("sim", h)
        self.assertEqual(h["sim"]["host"], "test_host:1234")
        self.assertFalse(h["sim"]["connected"])

    def test_odometry_publish(self):
        mod = SimDogModule()
        mod.setup()
        mod._pos_x = 3.0
        mod._pos_y = 4.0
        mod._yaw = 0.5

        received = []
        mod.odometry._add_callback(received.append)

        mod._publish_odometry()

        self.assertEqual(len(received), 1)
        odom = received[0]
        self.assertAlmostEqual(odom.pose.position.x, 3.0)
        self.assertAlmostEqual(odom.pose.position.y, 4.0)


# ── Blueprint build tests ────────────────────────────────────────────

class TestBlueprintBuild(unittest.TestCase):
    """All three blueprints should build successfully."""

    def test_stub_blueprint_builds_and_starts(self):
        system = stub_blueprint().build()
        self.assertIn("StubDogModule", system.modules)
        system.start()
        self.assertTrue(system.started)
        system.stop()
        self.assertFalse(system.started)

    def test_simulation_blueprint_builds(self):
        system = simulation_blueprint(
            sim_host="127.0.0.1", sim_port=19999).build()
        self.assertIn("SimDogModule", system.modules)

    def test_navigation_blueprint_builds(self):
        system = navigation_blueprint(dog_host="stub").build()
        self.assertIn("ThunderDriver", system.modules)

    def test_all_blueprints_same_module_count(self):
        stub_sys = stub_blueprint().build()
        sim_sys = simulation_blueprint().build()
        nav_sys = navigation_blueprint().build()

        self.assertEqual(len(stub_sys.modules), len(sim_sys.modules),
                         "stub and simulation should have same module count")
        self.assertEqual(len(sim_sys.modules), len(nav_sys.modules),
                         "simulation and navigation should have same module count")

    def test_stub_system_data_flow(self):
        """Build stub system, send cmd_vel, verify odometry arrives."""
        system = stub_blueprint().build()
        system.start()

        stub = system.get_module("StubDogModule")
        stub._last_ts = time.time() - 0.1

        # After start(), the evaluator etc. subscribe to odometry via auto_wire.
        # We add an extra callback to capture it.
        received = []
        stub.odometry._add_callback(received.append)

        twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3())
        stub.cmd_vel._deliver(twist)

        self.assertGreaterEqual(len(received), 1)
        self.assertIsInstance(received[0], Odometry)

        system.stop()


# ── Blueprint __init__ exports ───────────────────────────────────────

class TestBlueprintExports(unittest.TestCase):
    """The blueprints package should re-export all three factories."""

    def test_imports(self):
        from core.blueprints import navigation_blueprint
        from core.blueprints import simulation_blueprint
        from core.blueprints import stub_blueprint
        self.assertTrue(callable(navigation_blueprint))
        self.assertTrue(callable(simulation_blueprint))
        self.assertTrue(callable(stub_blueprint))


if __name__ == "__main__":
    unittest.main(verbosity=2)
