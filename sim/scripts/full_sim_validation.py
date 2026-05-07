#!/usr/bin/env python3
"""CLI wrapper for server-side full-system simulation validation."""

from __future__ import annotations

import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.validation.full_system import main


if __name__ == "__main__":
    raise SystemExit(main())
