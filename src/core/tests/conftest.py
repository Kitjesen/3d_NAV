"""conftest.py for src/core/tests/ -- add semantic package paths to sys.path.

Required because some Module tests import from semantic_planner which
transitively depends on semantic_common (a ROS2 package not installed).
"""

import os
import sys

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
_src = os.path.join(_repo, "src")

_paths = [
    _repo,
    _src,
    os.path.join(_src, "semantic", "planner"),
    os.path.join(_src, "semantic", "perception"),
    os.path.join(_src, "semantic", "common"),
]

for _p in _paths:
    if _p not in sys.path:
        sys.path.insert(0, _p)

# Script-style integration harnesses that use sys.exit() at module scope
# are not pytest-compatible. Exclude them from collection so that
# `pytest src/core/tests/` doesn't raise INTERNALERROR on SystemExit.
# Run them directly:  python3 src/core/tests/test_cross_module_integration.py
collect_ignore = [
    "test_cross_module_integration.py",
    "test_integration_36.py",
]
