"""TTY-aware ANSI formatting for the CLI."""

from __future__ import annotations

import sys

_IS_TTY = hasattr(sys.stdout, "isatty") and sys.stdout.isatty()
IS_TTY = _IS_TTY


def c(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m" if _IS_TTY else text


def bold(t: str) -> str:
    return c("1", t)


def green(t: str) -> str:
    return c("0;32", t)


def blue(t: str) -> str:
    return c("0;34", t)


def deep_blue(t: str) -> str:
    # 256-color "navy" like blue. Falls back to plain text when not TTY.
    return c("38;5;18", t)


def yellow(t: str) -> str:
    return c("1;33", t)


def red(t: str) -> str:
    return c("0;31", t)


def dim(t: str) -> str:
    return c("2", t)
