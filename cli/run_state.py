"""PID file and run.json for daemon / status / log / config introspection."""

from __future__ import annotations

import json
import os
import socket
import time
from pathlib import Path

from .paths import pid_file, run_dir, run_json_file


def _lingtu_version() -> str:
    """Best-effort version lookup: pyproject.toml → package metadata → 'unknown'."""
    try:
        from importlib.metadata import version as _pkg_version

        return _pkg_version("lingtu")
    except Exception:
        pass
    try:
        root = Path(__file__).resolve().parent.parent
        pyproject = root / "pyproject.toml"
        if pyproject.exists():
            for line in pyproject.read_text(encoding="utf-8").splitlines():
                stripped = line.strip()
                if stripped.startswith("version"):
                    _, _, rhs = stripped.partition("=")
                    return rhs.strip().strip('"').strip("'")
    except Exception:
        pass
    return "unknown"


def save_run_state(
    profile_name: str,
    cfg: dict,
    log_dir: str,
    log_format: str = "text",
    argv: list[str] | None = None,
    cwd: str | None = None,
    daemon: bool = False,
    status: str = "initializing",
    module_count: int | None = None,
    wire_count: int | None = None,
) -> None:
    run_dir().mkdir(exist_ok=True)
    if log_format == "json":
        log_file = str(Path(log_dir) / "lingtu.jsonl")
    else:
        log_file = str(Path(log_dir) / "lingtu.log")
    pid_file().write_text(str(os.getpid()))
    now = time.time()
    payload = {
        "pid": os.getpid(),
        "profile": profile_name,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(now)),
        "start_ts": now,
        "status": status,
        "cwd": cwd or os.getcwd(),
        "argv": argv or [],
        "daemon": bool(daemon),
        "log_dir": log_dir,
        "log_file": log_file,
        "log_format": log_format,
        "host": socket.gethostname(),
        "version": _lingtu_version(),
        "config": {k: v for k, v in cfg.items() if not k.startswith("_")},
    }
    if module_count is not None:
        payload["module_count"] = int(module_count)
    if wire_count is not None:
        payload["wire_count"] = int(wire_count)
    run_json_file().write_text(json.dumps(payload, indent=2))


def update_run_state(**fields) -> bool:
    """Patch selected fields in run.json without touching the rest.

    Returns True on success, False if run.json is missing or unreadable.
    Best-effort: failures are swallowed — never block the main lifecycle.
    """
    state = read_run_state()
    if state is None:
        return False
    try:
        state.update(fields)
        run_json_file().write_text(json.dumps(state, indent=2))
        return True
    except OSError:
        return False


def compute_uptime(state: dict | None) -> float | None:
    """Seconds since start_ts, or None if unknown (legacy state without start_ts)."""
    if not state:
        return None
    ts = state.get("start_ts")
    if not isinstance(ts, (int, float)):
        return None
    delta = time.time() - float(ts)
    return max(delta, 0.0)


def format_uptime(seconds: float | None) -> str:
    """Human-friendly uptime: `2h 13m 5s` / `45s` / `?` if unknown."""
    if seconds is None:
        return "?"
    total = int(seconds)
    d, rem = divmod(total, 86400)
    h, rem = divmod(rem, 3600)
    m, s = divmod(rem, 60)
    parts = []
    if d:
        parts.append(f"{d}d")
    if h:
        parts.append(f"{h}h")
    if m:
        parts.append(f"{m}m")
    if s or not parts:
        parts.append(f"{s}s")
    return " ".join(parts)


def clear_run_state() -> None:
    pid_file().unlink(missing_ok=True)
    run_json_file().unlink(missing_ok=True)


def read_run_state() -> dict | None:
    p = run_json_file()
    if not p.exists():
        return None
    try:
        return json.loads(p.read_text())
    except (json.JSONDecodeError, OSError):
        return None


def resolve_log_file(state: dict | None) -> Path | None:
    if not state:
        return None

    explicit = state.get("log_file")
    if explicit:
        candidate = Path(explicit)
        if candidate.exists():
            return candidate

    log_dir = state.get("log_dir")
    if not log_dir:
        return None

    base = Path(log_dir)
    fmt = state.get("log_format")
    preferred = []
    if fmt == "json":
        preferred = [base / "lingtu.jsonl", base / "lingtu.log"]
    else:
        preferred = [base / "lingtu.log", base / "lingtu.jsonl"]

    for candidate in preferred:
        if candidate.exists():
            return candidate
    return None


def is_pid_alive(pid: int) -> bool:
    try:
        if os.name == "nt":
            import ctypes

            kernel32 = ctypes.windll.kernel32
            handle = kernel32.OpenProcess(0x1000, False, pid)
            if handle:
                kernel32.CloseHandle(handle)
                return True
            return False
        os.kill(pid, 0)
        return True
    except (OSError, ProcessLookupError):
        return False
