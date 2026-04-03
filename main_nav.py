#!/usr/bin/env python3
"""Backward-compatible entry point — prefer ``python lingtu.py`` or ``lingtu``."""

from __future__ import annotations

from pathlib import Path

from cli.bootstrap import init
from cli.paths import set_project_root

_ROOT = Path(__file__).resolve().parent
set_project_root(_ROOT)
init(_ROOT)

from cli.main import main

if __name__ == "__main__":
    main()
