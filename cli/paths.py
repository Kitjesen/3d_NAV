"""Project-root paths (set once from the entry script before other cli imports)."""

from __future__ import annotations

from pathlib import Path

_ROOT: Path | None = None


def set_project_root(p: Path) -> None:
    global _ROOT
    _ROOT = p.resolve()


def project_root() -> Path:
    if _ROOT is None:
        raise RuntimeError("cli.paths.set_project_root() not called")
    return _ROOT


def run_dir() -> Path:
    return project_root() / ".lingtu"


def pid_file() -> Path:
    return run_dir() / "run.pid"


def run_json_file() -> Path:
    return run_dir() / "run.json"


def logs_base_dir() -> Path:
    return project_root() / "logs"
