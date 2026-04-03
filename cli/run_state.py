"""PID file and run.json for daemon / status."""

from __future__ import annotations

import json
import os
import time

from .paths import pid_file, run_dir, run_json_file


def save_run_state(profile_name: str, cfg: dict, log_path: str) -> None:
    run_dir().mkdir(exist_ok=True)
    pid_file().write_text(str(os.getpid()))
    run_json_file().write_text(
        json.dumps(
            {
                "pid": os.getpid(),
                "profile": profile_name,
                "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "log_dir": log_path,
                "config": {k: v for k, v in cfg.items() if not k.startswith("_")},
            },
            indent=2,
        )
    )


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
