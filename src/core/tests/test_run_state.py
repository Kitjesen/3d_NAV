"""Unit tests for cli.run_state — save/read/update/compute_uptime."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import pytest

# Add project root so `cli` is importable
_ROOT = Path(__file__).resolve().parent.parent.parent.parent
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


@pytest.fixture(autouse=True)
def isolated_run_dir(tmp_path, monkeypatch):
    """Redirect run_dir() / pid_file() / run_json_file() to a tmp directory."""
    from cli import paths, run_state

    rd = tmp_path / ".lingtu"
    monkeypatch.setattr(paths, "run_dir", lambda: rd)
    monkeypatch.setattr(paths, "pid_file", lambda: rd / "run.pid")
    monkeypatch.setattr(paths, "run_json_file", lambda: rd / "run.json")
    monkeypatch.setattr(run_state, "run_dir", lambda: rd)
    monkeypatch.setattr(run_state, "pid_file", lambda: rd / "run.pid")
    monkeypatch.setattr(run_state, "run_json_file", lambda: rd / "run.json")
    yield rd


class TestSaveReadClear:
    def test_save_and_read(self, isolated_run_dir):
        from cli.run_state import save_run_state, read_run_state

        cfg = {"robot": "stub", "llm": "mock", "_desc": "ignored"}
        save_run_state("stub", cfg, str(isolated_run_dir), argv=["stub"], daemon=False)

        state = read_run_state()
        assert state is not None
        assert state["profile"] == "stub"
        assert state["pid"] == os.getpid()
        assert "_desc" not in state["config"]  # underscore keys filtered
        assert state["config"]["robot"] == "stub"

    def test_save_populates_new_fields(self, isolated_run_dir):
        from cli.run_state import save_run_state, read_run_state

        save_run_state(
            "stub", {"robot": "stub"}, str(isolated_run_dir),
            status="running", module_count=19, wire_count=48,
        )
        state = read_run_state()
        assert state["status"] == "running"
        assert state["module_count"] == 19
        assert state["wire_count"] == 48
        assert "start_ts" in state and isinstance(state["start_ts"], (int, float))
        assert "host" in state
        assert "version" in state

    def test_save_omits_none_counts(self, isolated_run_dir):
        from cli.run_state import save_run_state, read_run_state

        save_run_state("stub", {}, str(isolated_run_dir))
        state = read_run_state()
        assert "module_count" not in state
        assert "wire_count" not in state

    def test_read_missing_returns_none(self, isolated_run_dir):
        from cli.run_state import read_run_state

        assert read_run_state() is None

    def test_clear_removes_files(self, isolated_run_dir):
        from cli.run_state import save_run_state, read_run_state, clear_run_state

        save_run_state("stub", {}, str(isolated_run_dir))
        assert read_run_state() is not None
        clear_run_state()
        assert read_run_state() is None


class TestUpdateRunState:
    def test_update_patches_fields(self, isolated_run_dir):
        from cli.run_state import save_run_state, read_run_state, update_run_state

        save_run_state("stub", {}, str(isolated_run_dir), status="initializing")
        assert update_run_state(status="running", module_count=19) is True

        state = read_run_state()
        assert state["status"] == "running"
        assert state["module_count"] == 19
        assert state["profile"] == "stub"  # original fields preserved

    def test_update_missing_returns_false(self, isolated_run_dir):
        from cli.run_state import update_run_state

        assert update_run_state(status="running") is False

    def test_update_preserves_other_fields(self, isolated_run_dir):
        from cli.run_state import save_run_state, update_run_state, read_run_state

        save_run_state("nav", {"robot": "thunder"}, str(isolated_run_dir), argv=["nav"])
        update_run_state(status="running")
        state = read_run_state()
        assert state["profile"] == "nav"
        assert state["argv"] == ["nav"]
        assert state["config"]["robot"] == "thunder"


class TestComputeUptime:
    def test_none_state(self):
        from cli.run_state import compute_uptime

        assert compute_uptime(None) is None
        assert compute_uptime({}) is None

    def test_missing_start_ts(self):
        from cli.run_state import compute_uptime

        assert compute_uptime({"pid": 123}) is None

    def test_invalid_start_ts(self):
        from cli.run_state import compute_uptime

        assert compute_uptime({"start_ts": "not a number"}) is None

    def test_positive_uptime(self):
        from cli.run_state import compute_uptime

        state = {"start_ts": time.time() - 60.0}
        uptime = compute_uptime(state)
        assert uptime is not None
        assert 59 <= uptime <= 61  # tolerance for test execution time

    def test_clock_skew_never_negative(self):
        from cli.run_state import compute_uptime

        state = {"start_ts": time.time() + 100.0}  # future
        assert compute_uptime(state) == 0.0


class TestFormatUptime:
    def test_none(self):
        from cli.run_state import format_uptime

        assert format_uptime(None) == "?"

    def test_seconds_only(self):
        from cli.run_state import format_uptime

        assert format_uptime(0.0) == "0s"
        assert format_uptime(45.0) == "45s"

    def test_minutes(self):
        from cli.run_state import format_uptime

        assert format_uptime(125.0) == "2m 5s"

    def test_hours(self):
        from cli.run_state import format_uptime

        assert format_uptime(3665.0) == "1h 1m 5s"

    def test_days(self):
        from cli.run_state import format_uptime

        assert format_uptime(90061.0) == "1d 1h 1m 1s"


class TestLingtuVersion:
    def test_returns_string(self):
        from cli.run_state import _lingtu_version

        v = _lingtu_version()
        assert isinstance(v, str)
        assert len(v) > 0
