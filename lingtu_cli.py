"""LingTu CLI entry point — installed as `lingtu` command via pip install -e ."""

import os
import sys

# Ensure root, src/, and ROS2 sub-package paths are on sys.path
_root = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _root)
sys.path.insert(0, os.path.join(_root, "src"))
for _sub in ["src/semantic/perception", "src/semantic/planner", "src/semantic/common"]:
    _p = os.path.join(_root, _sub)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)


def main():
    from main_nav import main as _main
    _main()


if __name__ == "__main__":
    main()
