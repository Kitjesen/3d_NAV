"""LingTu CLI entry point — installed as ``lingtu`` command via pip install -e ."""

from __future__ import annotations

from pathlib import Path

from cli.bootstrap import init
from cli.paths import set_project_root


def main():
    root = Path(__file__).resolve().parent
    set_project_root(root)
    init(root)
    from cli.main import main as _main

    _main()


if __name__ == "__main__":
    main()
