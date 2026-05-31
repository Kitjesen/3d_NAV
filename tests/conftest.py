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


def pytest_ignore_collect(collection_path, config):
    """Skip 3rdparty and vendored directories to avoid import noise and leaks."""
    _str_path = str(collection_path)
    _skip_patterns = [
        "/3rdparty/",
        "/third_party/",
        "/.venv/",
        "/venv/",
        "/.omc/",
        "/artifacts/",
        "/build/",
        "\\3rdparty\\",
        "\\third_party\\",
        "\\.venv\\",
        "\\venv\\",
        "\\.omc\\",
        "\\artifacts\\",
        "\\build\\",
    ]
    for pat in _skip_patterns:
        if pat in _str_path:
            return True
    return None
