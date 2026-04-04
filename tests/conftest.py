"""tests/conftest.py — sys.path setup for root-level tests."""

import os
import sys

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_src = os.path.join(_repo, "src")

for _p in [
    _repo,
    _src,
    os.path.join(_src, "semantic", "planner"),
    os.path.join(_src, "semantic", "perception"),
    os.path.join(_src, "semantic", "common"),
]:
    if _p not in sys.path:
        sys.path.insert(0, _p)
