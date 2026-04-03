"""Structured logging for CLI runs."""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

from .paths import logs_base_dir


def setup_logging(level: str, profile_name: str) -> str:
    """Stderr + per-run file. Returns log directory path."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_dir = logs_base_dir() / f"{ts}_{profile_name}"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "lingtu.log"

    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    stderr_h = logging.StreamHandler(sys.stderr)
    stderr_h.setFormatter(fmt)

    file_h = logging.FileHandler(str(log_file), encoding="utf-8")
    file_h.setFormatter(
        logging.Formatter(
            "%(asctime)s\t%(levelname)s\t%(name)s\t%(message)s",
        )
    )

    root = logging.getLogger()
    root.setLevel(getattr(logging, level.upper(), logging.INFO))
    root.addHandler(stderr_h)
    root.addHandler(file_h)

    return str(log_dir)
