"""Shared helper to locate and import _nav_core nanobind extension.

_nav_core.so is built by scripts/build_nav_core.sh and placed (or symlinked)
under src/nav/core/build_nb/.  This helper adds that directory to sys.path
before attempting the import so the modules work without manually setting
PYTHONPATH.
"""

from __future__ import annotations

import importlib.util
import logging
import os
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

# Candidate directories where _nav_core*.so might live, in priority order.
def _candidate_dirs() -> list[str]:
    # Walk up from this file to find the repo root (contains lingtu.py or src/)
    here = Path(__file__).resolve()
    for parent in [here.parent, here.parent.parent, here.parent.parent.parent,
                   here.parent.parent.parent.parent]:
        if (parent / "lingtu.py").exists() or (parent / "src").is_dir():
            repo = parent
            break
    else:
        repo = here.parent.parent.parent  # best guess

    return [
        str(repo / "src"),                              # symlink installed here by build script
        str(repo / "src" / "nav" / "core" / "build_nb"),  # build output dir
        str(repo / "install" / "nav_core" / "lib"),     # colcon install (future)
    ]


def ensure_nav_core_on_path() -> None:
    """Add _nav_core build dir to sys.path if not already importable."""
    if importlib.util.find_spec("_nav_core") is not None:
        return  # already importable

    for d in _candidate_dirs():
        if not os.path.isdir(d):
            continue
        # Check if any _nav_core*.so exists in this dir
        has_so = any(
            f.startswith("_nav_core") and f.endswith(".so")
            for f in os.listdir(d)
        )
        if has_so and d not in sys.path:
            sys.path.insert(0, d)
            logger.debug("_nav_core_loader: added %s to sys.path", d)
            break


def try_import_nav_core():
    """Import and return _nav_core, or None if not available."""
    ensure_nav_core_on_path()
    try:
        import _nav_core
        return _nav_core
    except ImportError:
        return None


def nav_core_build_hint() -> str:
    """Return a human-readable hint for building _nav_core."""
    return (
        "Run:  bash scripts/build_nav_core.sh\n"
        "      (needs cmake, python3-dev, pip install nanobind)"
    )
