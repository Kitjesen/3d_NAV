"""Tests for CmdVelMux and TeleopModule.

Verifies priority arbitration, idle timeout, and teleop state management.
All tests are pure-Python, no ROS2 / hardware required.
"""

from __future__ import annotations

import time
import unittest

from core.msgs.geometry import Twist, Vector3
from nav.cmd_vel_mux_module import CmdVelMux


def _twist(vx: float = 0.0, wz: float = 0.0) -> Twist:
    return Twist(linear=Vector3(x=vx, y=0.0, z=0.0),
                 angular=Vector3(x=0.0, y=0.0, z=wz))


# ---------------------------------------------------------------------------
# CmdVelMux tests
# ---------------------------------------------------------------------------

class TestCmdVelMux(unittest.TestCase):
    """Test priority-based cmd_vel arbitration."""

    def _make_mux(self, timeout: float = 1.0) -> CmdVelMux:
        mux = CmdVelMux(source_timeout=timeout)
        mux.setup()
        return mux

    def test_single_source_forwards(self):
        """A single source publishes → mux forwards to driver."""
        mux = self._make_mux()
        published = []
        mux.driver_cmd_vel.subscribe(lambda t: published.append(t))

        mux._on_source("path_follower", _twist(0.3))
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[0].linear.x, 0.3)
        self.assertEqual(mux._active, "path_follower")

    def test_higher_priority_preempts(self):
        """Teleop (100) preempts path_follower (40)."""
        mux = self._make_mux()
        published = []
        mux.driver_cmd_vel.subscribe(lambda t: published.append(t))

        # Path follower active
        mux._on_source("path_follower", _twist(0.3))
        self.assertEqual(mux._active, "path_follower")

        # Teleop takes over
        mux._on_source("teleop", _twist(0.5))
        self.assertEqual(mux._active, "teleop")
        self.assertAlmostEqual(published[-1].linear.x, 0.5)

    def test_lower_priority_blocked(self):
        """While teleop is active, path_follower cmd_vel is not forwarded."""
        mux = self._make_mux()
        published = []
        mux.driver_cmd_vel.subscribe(lambda t: published.append(t))

        mux._on_source("teleop", _twist(0.5))
        count_after_teleop = len(published)

        # Path follower publishes — should NOT be forwarded
        mux._on_source("path_follower", _twist(0.3))
        # Active is still teleop, path_follower's twist NOT forwarded
        self.assertEqual(mux._active, "teleop")
        self.assertEqual(len(published), count_after_teleop)

    def test_source_timeout_releases(self):
        """After source timeout, lower priority source takes over."""
        mux = self._make_mux(timeout=0.1)

        mux._on_source("teleop", _twist(0.5))
        self.assertEqual(mux._active, "teleop")

        time.sleep(0.15)  # teleop times out

        # Path follower publishes
        published = []
        mux.driver_cmd_vel.subscribe(lambda t: published.append(t))
        mux._on_source("path_follower", _twist(0.3))
        self.assertEqual(mux._active, "path_follower")
        self.assertAlmostEqual(published[-1].linear.x, 0.3)

    def test_visual_servo_preempts_path_follower(self):
        """Visual servo (80) preempts path follower (40)."""
        mux = self._make_mux()

        mux._on_source("path_follower", _twist(0.3))
        self.assertEqual(mux._active, "path_follower")

        mux._on_source("visual_servo", _twist(0.1, 0.5))
        self.assertEqual(mux._active, "visual_servo")

    def test_recovery_preempts_path_follower(self):
        """Recovery (60) preempts path follower (40)."""
        mux = self._make_mux()

        mux._on_source("path_follower", _twist(0.3))
        mux._on_source("recovery", _twist(-0.2))
        self.assertEqual(mux._active, "recovery")

    def test_teleop_preempts_everything(self):
        """Teleop (100) preempts all other sources."""
        mux = self._make_mux()

        mux._on_source("path_follower", _twist(0.3))
        mux._on_source("visual_servo", _twist(0.1))
        mux._on_source("recovery", _twist(-0.2))
        mux._on_source("teleop", _twist(0.5))
        self.assertEqual(mux._active, "teleop")

    def test_no_active_source(self):
        """Initially no source is active."""
        mux = self._make_mux()
        self.assertEqual(mux._active, "")

    def test_health_report(self):
        """Health report shows all sources and active state."""
        mux = self._make_mux()
        mux._on_source("teleop", _twist(0.5))
        h = mux.health()
        self.assertEqual(h["active_source"], "teleop")
        self.assertIn("teleop", h["sources"])
        self.assertTrue(h["sources"]["teleop"]["active"])

    def test_active_source_published(self):
        """active_source Out port publishes on source change."""
        mux = self._make_mux()
        sources = []
        mux.active_source.subscribe(lambda s: sources.append(s))

        mux._on_source("path_follower", _twist(0.3))
        mux._on_source("teleop", _twist(0.5))

        self.assertIn("path_follower", sources)
        self.assertIn("teleop", sources)


# ---------------------------------------------------------------------------
# TeleopModule tests
# ---------------------------------------------------------------------------

from drivers.teleop_module import TeleopModule


class TestTeleopModule(unittest.TestCase):
    """Test TeleopModule joystick processing and state management."""

    def _make_teleop(self) -> TeleopModule:
        tm = TeleopModule(
            max_speed=1.0, max_yaw_rate=2.0, release_timeout=0.2,
        )
        tm.setup()
        return tm

    def test_joy_input_scales_correctly(self):
        """Joystick input is scaled by max_speed/max_yaw_rate."""
        tm = self._make_teleop()
        published = []
        tm.cmd_vel.subscribe(lambda t: published.append(t))

        tm._on_joy({"lx": 0.5, "ly": 0.0, "az": -0.5})
        self.assertEqual(len(published), 1)
        self.assertAlmostEqual(published[0].linear.x, 0.5)   # 0.5 * 1.0
        self.assertAlmostEqual(published[0].angular.z, -1.0)  # -0.5 * 2.0

    def test_joy_clamps_input(self):
        """Out-of-range joystick values are clamped to [-1, 1]."""
        tm = self._make_teleop()
        published = []
        tm.cmd_vel.subscribe(lambda t: published.append(t))

        tm._on_joy({"lx": 5.0, "ly": 0.0, "az": 0.0})
        self.assertAlmostEqual(published[0].linear.x, 1.0)  # clamped to 1.0 * 1.0

    def test_teleop_active_on_joy(self):
        """First joy input sets teleop_active = True."""
        tm = self._make_teleop()
        active_states = []
        tm.teleop_active.subscribe(lambda a: active_states.append(a))

        self.assertFalse(tm._active)
        tm._on_joy({"lx": 0.1})
        self.assertTrue(tm._active)
        self.assertEqual(active_states, [True])

    def test_release_publishes_zero_and_inactive(self):
        """force_release() publishes zero twist + teleop_active=False."""
        tm = self._make_teleop()
        twists = []
        actives = []
        tm.cmd_vel.subscribe(lambda t: twists.append(t))
        tm.teleop_active.subscribe(lambda a: actives.append(a))

        tm._on_joy({"lx": 0.5})
        tm._release()
        self.assertFalse(tm._active)
        # Last twist should be zero
        self.assertAlmostEqual(twists[-1].linear.x, 0.0)
        self.assertEqual(actives[-1], False)

    def test_idle_release(self):
        """After release_timeout with no joy, teleop releases automatically."""
        tm = self._make_teleop()
        tm._running = True
        actives = []
        tm.teleop_active.subscribe(lambda a: actives.append(a))

        tm._on_joy({"lx": 0.3})
        self.assertTrue(tm._active)
        time.sleep(0.3)  # > release_timeout (0.2)
        # Simulate idle check (normally runs in background thread)
        if (tm._active
                and time.monotonic() - tm._last_joy_time > tm._release_timeout):
            tm._release()
        self.assertFalse(tm._active)

    def test_client_disconnect_releases(self):
        """When last client disconnects, teleop is released."""
        tm = self._make_teleop()
        actives = []
        tm.teleop_active.subscribe(lambda a: actives.append(a))

        tm.on_client_connect()
        tm._on_joy({"lx": 0.5})
        self.assertTrue(tm._active)

        tm.on_client_disconnect()  # last client
        self.assertFalse(tm._active)

    def test_multiple_clients_keep_active(self):
        """Teleop stays active if other clients remain."""
        tm = self._make_teleop()

        tm.on_client_connect()
        tm.on_client_connect()
        tm._on_joy({"lx": 0.5})
        self.assertTrue(tm._active)

        tm.on_client_disconnect()  # still 1 client
        self.assertTrue(tm._active)

        tm.on_client_disconnect()  # last client
        self.assertFalse(tm._active)

    def test_get_teleop_status(self):
        """Skill method returns correct status."""
        tm = self._make_teleop()
        tm._on_joy({"lx": 0.1})
        status = tm.get_teleop_status()
        self.assertTrue(status["active"])
        self.assertIsNotNone(status["last_joy_age_ms"])


if __name__ == "__main__":
    unittest.main()
