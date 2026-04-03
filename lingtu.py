#!/usr/bin/env python3
"""LingTu — single entry point (CLI profiles + interactive REPL).

Run::

    python lingtu.py              # interactive profile picker
    python lingtu.py nav          # navigation stack
    python lingtu.py --list       # list profiles

When installed via pip, the ``lingtu`` console script calls ``lingtu_cli`` (same stack).
"""

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
