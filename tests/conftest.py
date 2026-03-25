"""
tests/conftest.py — 统一 sys.path 配置

ROS2 包采用嵌套结构 (src/semantic_planner/semantic_planner/),
需要将每个 ROS2 包目录加到 sys.path 才能 import。
"""

import os
import sys

_repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
_src = os.path.join(_repo, "src")

_paths = [
    _repo,
    _src,
    os.path.join(_src, "semantic_planner"),
    os.path.join(_src, "semantic_perception"),
    os.path.join(_src, "semantic_common"),
]

for _p in _paths:
    if _p not in sys.path:
        sys.path.insert(0, _p)
