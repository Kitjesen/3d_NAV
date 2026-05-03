from __future__ import annotations

import subprocess
from dataclasses import dataclass


@dataclass
class _RunResult:
    returncode: int = 0
    stdout: str = ""
    stderr: bytes = b""


class _FakeSystemctl:
    def __init__(self, *, active: set[str] | None = None, loaded: set[str] | None = None):
        self.active = set(active or set())
        self.loaded = set(loaded or set())
        self.commands: list[list[str]] = []

    def __call__(self, cmd, **kwargs):
        command = list(cmd)
        self.commands.append(command)

        if command[:3] == ["systemctl", "is-active", "--quiet"]:
            return _RunResult(returncode=0 if command[3] in self.active else 3)

        if command[:5] == ["systemctl", "show", "-p", "LoadState", "--value"]:
            unit = command[5]
            state = "loaded" if unit in self.loaded else "not-found"
            return _RunResult(returncode=0, stdout=state)

        if command[:3] == ["sudo", "systemctl", "start"]:
            unit = command[3]
            if unit not in self.loaded:
                raise subprocess.CalledProcessError(
                    5,
                    command,
                    stderr=b"Unit not found",
                )
            self.active.add(unit)
            return _RunResult()

        if command[:3] == ["sudo", "systemctl", "stop"]:
            self.active.discard(command[3])
            return _RunResult()

        raise AssertionError(f"unexpected command: {command!r}")


def test_logical_status_detects_robot_service_aliases(monkeypatch):
    from core.service_manager import ServiceManager

    fake = _FakeSystemctl(
        active={
            "robot-lidar.service",
            "robot-fastlio2.service",
            "robot-localizer.service",
            "robot-super-lio.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.status("lidar", "slam", "slam_pgo", "localizer", "super_lio") == {
        "lidar": "running",
        "slam": "running",
        "slam_pgo": "stopped",
        "localizer": "running",
        "super_lio": "running",
    }


def test_logical_status_keeps_legacy_service_fallback(monkeypatch):
    from core.service_manager import ServiceManager

    fake = _FakeSystemctl(active={"slam.service", "localizer.service"})
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.status("slam", "localizer") == {
        "slam": "running",
        "localizer": "running",
    }


def test_canonical_robot_unit_wins_over_active_legacy_alias(monkeypatch):
    from core.service_manager import ServiceManager

    fake = _FakeSystemctl(
        active={"camera.service"},
        loaded={"robot-camera.service", "camera.service"},
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.status("camera") == {"camera": "stopped"}
    assert svc.start("camera") == ["camera"]
    assert "robot-camera.service" in fake.active
    assert ["sudo", "systemctl", "start", "robot-camera.service"] in fake.commands
    assert ["sudo", "systemctl", "start", "camera.service"] not in fake.commands


def test_start_prefers_robot_unit_when_installed(monkeypatch):
    from core.service_manager import ServiceManager

    fake = _FakeSystemctl(loaded={"robot-fastlio2.service", "slam.service"})
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("slam") == ["slam"]
    assert "robot-fastlio2.service" in fake.active
    assert ["sudo", "systemctl", "start", "robot-fastlio2.service"] in fake.commands
    assert svc._started == ["robot-fastlio2.service"]


def test_start_super_lio_prefers_robot_unit_when_installed(monkeypatch):
    from core.service_manager import ServiceManager

    fake = _FakeSystemctl(
        loaded={
            "robot-super-lio.service",
            "super_lio.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()

    assert svc.start("super_lio") == ["super_lio"]
    assert "robot-super-lio.service" in fake.active
    assert ["sudo", "systemctl", "start", "robot-super-lio.service"] in fake.commands
    assert svc._started == ["robot-super-lio.service"]


def test_stop_clears_new_and_legacy_aliases(monkeypatch):
    from core.service_manager import ServiceManager

    fake = _FakeSystemctl(
        active={
            "robot-fastlio2.service",
            "slam.service",
            "robot-localizer.service",
            "localizer.service",
            "robot-super-lio.service",
            "super_lio.service",
        }
    )
    monkeypatch.setattr(subprocess, "run", fake)

    svc = ServiceManager()
    svc._started = ["robot-fastlio2.service", "slam.service"]

    svc.stop("slam", "localizer", "super_lio")

    assert "robot-fastlio2.service" not in fake.active
    assert "slam.service" not in fake.active
    assert "robot-localizer.service" not in fake.active
    assert "localizer.service" not in fake.active
    assert "robot-super-lio.service" not in fake.active
    assert "super_lio.service" not in fake.active
    assert ["sudo", "systemctl", "stop", "robot-fastlio2.service"] in fake.commands
    assert ["sudo", "systemctl", "stop", "slam.service"] in fake.commands
    assert ["sudo", "systemctl", "stop", "robot-super-lio.service"] in fake.commands
    assert ["sudo", "systemctl", "stop", "super_lio.service"] in fake.commands
    assert svc._started == []
