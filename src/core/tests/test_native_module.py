"""Tests for NativeModule — subprocess lifecycle management.

Uses mock subprocess to avoid needing real C++ binaries.
"""

import os
import sys
import time
import threading
import unittest
from unittest.mock import patch, MagicMock, PropertyMock

from core.native_module import NativeModule, NativeModuleConfig
from core.blueprint import Blueprint


# ---------------------------------------------------------------------------
# Helper: create a config pointing to a real executable (python itself)
# ---------------------------------------------------------------------------

PYTHON_EXE = sys.executable  # always exists and is executable


def _make_config(**overrides):
    defaults = dict(
        executable=PYTHON_EXE,
        name="test_node",
        parameters={"maxSpeed": 0.875, "check": True},
        remappings={"/cloud_map": "/nav/map_cloud"},
    )
    defaults.update(overrides)
    return NativeModuleConfig(**defaults)


# ---------------------------------------------------------------------------
# TestNativeModuleConfig
# ---------------------------------------------------------------------------

class TestNativeModuleConfig(unittest.TestCase):

    def test_name_defaults_to_basename(self):
        cfg = NativeModuleConfig(executable="/opt/nav/bin/terrainAnalysis")
        self.assertEqual(cfg.name, "terrainAnalysis")

    def test_to_ros_args_parameters(self):
        cfg = _make_config(parameters={"maxSpeed": 0.875, "check": True}, remappings={})
        args = cfg.to_ros_args()
        self.assertIn("--ros-args", args)
        self.assertIn("-p", args)
        self.assertIn("maxSpeed:=0.875", args)
        self.assertIn("check:=true", args)

    def test_to_ros_args_remappings(self):
        cfg = _make_config(parameters={}, remappings={"/old": "/new"})
        args = cfg.to_ros_args()
        self.assertIn("-r", args)
        self.assertIn("/old:=/new", args)

    def test_to_ros_args_empty(self):
        cfg = _make_config(parameters={}, remappings={})
        self.assertEqual(cfg.to_ros_args(), [])

    def test_build_cmd(self):
        cfg = _make_config()
        cmd = cfg.build_cmd()
        self.assertEqual(cmd[0], PYTHON_EXE)
        self.assertIn("--ros-args", cmd)

    def test_build_cmd_no_args(self):
        cfg = _make_config(parameters={}, remappings={})
        cmd = cfg.build_cmd()
        self.assertEqual(cmd, [PYTHON_EXE])


# ---------------------------------------------------------------------------
# TestNativeModuleLifecycle
# ---------------------------------------------------------------------------

class TestNativeModuleLifecycle(unittest.TestCase):

    def test_setup_validates_executable_exists(self):
        cfg = NativeModuleConfig(executable="/nonexistent/binary")
        mod = NativeModule(cfg)
        with self.assertRaises(FileNotFoundError):
            mod.setup()

    def test_setup_validates_executable_permission(self):
        # Create a file that exists but isn't executable (Unix only)
        if os.name == "nt":
            self.skipTest("Permission test not reliable on Windows")
        import tempfile
        with tempfile.NamedTemporaryFile(delete=False, suffix=".sh") as f:
            f.write(b"#!/bin/sh\n")
            path = f.name
        try:
            os.chmod(path, 0o444)  # read-only
            cfg = NativeModuleConfig(executable=path)
            mod = NativeModule(cfg)
            with self.assertRaises(PermissionError):
                mod.setup()
        finally:
            os.unlink(path)

    @patch("core.native_module.subprocess.Popen")
    def test_start_launches_process(self, mock_popen):
        proc = MagicMock()
        proc.pid = 12345
        proc.poll.return_value = None  # running
        proc.stdout = MagicMock()
        proc.stdout.__iter__ = MagicMock(return_value=iter([]))
        mock_popen.return_value = proc

        cfg = _make_config()
        mod = NativeModule(cfg)
        mod.start()

        mock_popen.assert_called_once()
        cmd = mock_popen.call_args[0][0]
        self.assertEqual(cmd[0], PYTHON_EXE)
        self.assertTrue(mod.running)

        mod._shutdown_event.set()
        mod._process = proc
        proc.poll.return_value = 0
        time.sleep(0.1)
        mod.stop()

    @patch("core.native_module.subprocess.Popen")
    def test_stop_sends_sigterm(self, mock_popen):
        proc = MagicMock()
        proc.pid = 99
        proc.poll.return_value = None
        proc.wait.return_value = 0
        proc.stdout = MagicMock()
        proc.stdout.__iter__ = MagicMock(return_value=iter([]))
        mock_popen.return_value = proc

        cfg = _make_config(shutdown_timeout=1.0)
        mod = NativeModule(cfg)
        mod.start()

        # stop should terminate
        proc.poll.return_value = 0  # simulate exit after SIGTERM
        mod.stop()
        self.assertFalse(mod.running)

    @patch("core.native_module.subprocess.Popen")
    def test_stop_is_idempotent(self, mock_popen):
        proc = MagicMock()
        proc.pid = 100
        proc.poll.return_value = 0
        proc.stdout = MagicMock()
        proc.stdout.__iter__ = MagicMock(return_value=iter([]))
        mock_popen.return_value = proc

        cfg = _make_config()
        mod = NativeModule(cfg)
        mod.start()
        mod.stop()
        mod.stop()  # should not raise

    @patch("core.native_module.subprocess.Popen")
    def test_watchdog_detects_crash(self, mock_popen):
        proc = MagicMock()
        proc.pid = 200
        proc.stdout = MagicMock()
        proc.stdout.__iter__ = MagicMock(return_value=iter([]))
        mock_popen.return_value = proc

        alive_values = []
        cfg = _make_config(watchdog_interval=0.05, auto_restart=False)
        mod = NativeModule(cfg)
        mod.alive._add_callback(alive_values.append)

        # Start with process running
        proc.poll.return_value = None
        mod.start()
        time.sleep(0.02)

        # Simulate crash
        proc.poll.return_value = 1
        proc.returncode = 1
        time.sleep(0.15)  # let watchdog detect

        # Should have published False
        self.assertIn(False, alive_values)
        mod._shutdown_event.set()
        time.sleep(0.1)

    @patch("core.native_module.subprocess.Popen")
    def test_watchdog_auto_restart(self, mock_popen):
        proc1 = MagicMock()
        proc1.pid = 300
        proc1.stdout = MagicMock()
        proc1.stdout.__iter__ = MagicMock(return_value=iter([]))

        proc2 = MagicMock()
        proc2.pid = 301
        proc2.poll.return_value = None
        proc2.stdout = MagicMock()
        proc2.stdout.__iter__ = MagicMock(return_value=iter([]))

        mock_popen.side_effect = [proc1, proc2]

        cfg = _make_config(watchdog_interval=0.05, auto_restart=True, max_restarts=2)
        mod = NativeModule(cfg)

        proc1.poll.return_value = None
        mod.start()
        time.sleep(0.02)

        # Simulate crash
        proc1.poll.return_value = 1
        proc1.returncode = 1
        time.sleep(0.2)

        # Should have restarted
        self.assertEqual(mock_popen.call_count, 2)
        self.assertEqual(mod._restart_count, 1)

        mod._shutdown_event.set()
        time.sleep(0.1)


# ---------------------------------------------------------------------------
# TestNativeModuleInBlueprint
# ---------------------------------------------------------------------------

class TestNativeModuleInBlueprint(unittest.TestCase):

    @patch("core.native_module.subprocess.Popen")
    def test_blueprint_accepts_native_module(self, mock_popen):
        proc = MagicMock()
        proc.pid = 400
        proc.poll.return_value = None
        proc.stdout = MagicMock()
        proc.stdout.__iter__ = MagicMock(return_value=iter([]))
        mock_popen.return_value = proc

        cfg = _make_config()
        mod = NativeModule(cfg)

        bp = Blueprint()
        bp.add(mod, alias="test_native")
        handle = bp.build()

        self.assertIn("test_native", handle.modules)

    @patch("core.native_module.subprocess.Popen")
    @patch("core.native_module.NativeModule.setup")
    def test_system_start_launches_native(self, mock_setup, mock_popen):
        proc = MagicMock()
        proc.pid = 500
        proc.poll.return_value = None
        proc.stdout = MagicMock()
        proc.stdout.__iter__ = MagicMock(return_value=iter([]))
        mock_popen.return_value = proc

        cfg = _make_config()
        mod = NativeModule(cfg)
        bp = Blueprint()
        bp.add(mod, alias="native_test")
        handle = bp.build()
        handle.start()

        mock_popen.assert_called_once()
        self.assertTrue(handle.started)

        proc.poll.return_value = 0
        handle.stop()

    @patch("core.native_module.subprocess.Popen")
    @patch("core.native_module.NativeModule.setup")
    def test_mixed_python_and_native(self, mock_setup, mock_popen):
        """Python Module + NativeModule coexist in one Blueprint."""
        from core.module import Module
        from core.stream import Out

        class PySource(Module, layer=1):
            data: Out[int]

        proc = MagicMock()
        proc.pid = 600
        proc.poll.return_value = None
        proc.stdout = MagicMock()
        proc.stdout.__iter__ = MagicMock(return_value=iter([]))
        mock_popen.return_value = proc

        cfg = _make_config()
        native = NativeModule(cfg)

        bp = Blueprint()
        bp.add(PySource)
        bp.add(native, alias="native_node")
        handle = bp.build()
        handle.start()

        self.assertIn("PySource", handle.modules)
        self.assertIn("native_node", handle.modules)

        proc.poll.return_value = 0
        handle.stop()


# ---------------------------------------------------------------------------
# TestNativeModuleHealth
# ---------------------------------------------------------------------------

class TestNativeModuleHealth(unittest.TestCase):

    def test_health_report_structure(self):
        cfg = _make_config()
        mod = NativeModule(cfg)
        h = mod.health()
        self.assertIn("native", h)
        self.assertEqual(h["native"]["name"], "test_node")
        self.assertFalse(h["native"]["running"])
        self.assertIsNone(h["native"]["pid"])

    def test_repr(self):
        cfg = _make_config()
        mod = NativeModule(cfg)
        self.assertIn("test_node", repr(mod))
        self.assertIn("stopped", repr(mod))


if __name__ == "__main__":
    unittest.main(verbosity=2)
