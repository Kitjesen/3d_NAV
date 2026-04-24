"""conftest.py for src/core/tests/ -- add semantic package paths to sys.path."""

import os
import sys

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_src = os.path.join(_repo, "src")

_paths = [
    _repo,
    _src,
    os.path.join(_src, "semantic", "planner"),
    os.path.join(_src, "semantic", "perception"),
]

for _p in _paths:
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Integration harnesses run module-level setup at import time.
# Both files now expose proper def test_*() functions and guard sys.exit()
# in `if __name__ == "__main__":`, so pytest can collect them safely.
