"""Banner, profile listing, stop/status helpers."""

from __future__ import annotations

import os
import signal
import sys
import time

from . import term as T
from .profiles_data import PROFILES
from .run_state import clear_run_state, is_pid_alive, read_run_state


def _panel(lines: list[str], *, color) -> None:
    """Print a compact config panel (dimos-style)."""
    if not lines:
        return
    width = max(len(x) for x in lines)
    top = f"┌{'─' * (width + 2)}┐"
    bot = f"└{'─' * (width + 2)}┘"
    print(f"  {color(top)}")
    for line in lines:
        print(f"  {color('│')}{' '}{line:<{width}}{' '}{color('│')}")
    print(f"  {color(bot)}")


def print_banner(profile_name, cfg, system, log_dir: str) -> None:
    n = len(system.modules)
    c = len(system.connections)
    desc = cfg.get("_desc", "custom")
    gw = cfg.get("gateway_port", 5050)

    robot = cfg.get("robot", "?")
    planner = cfg.get("planner", "?")
    detector = cfg.get("detector", "?")
    llm = cfg.get("llm", "?")

    teleop_port = None
    try:
        teleop_port = system.get_module("TeleopModule").get_teleop_status().get("port")
    except Exception:
        pass

    tomogram = cfg.get("tomogram", "")
    tomo_note = ""
    if tomogram and "building2_9.pickle" in str(tomogram):
        tomo_note = T.yellow(" (sample)")

    navy = getattr(T, "navy", T.blue)

    title = f"{T.bold('LingTu')} {T.dim('Navigation System')}"
    subtitle = f"{T.green(profile_name)} {T.dim('— ' + desc)}"

    lines: list[str] = []
    lines.append(f"{title}")
    lines.append(f"mode:    {subtitle}")
    lines.append(f"robot:   {robot:12s}   planner: {planner}")
    if cfg.get("enable_semantic"):
        lines.append(f"semantic:{detector:10s}   llm:     {llm}")
    else:
        lines.append(f"semantic:{T.dim('disabled')}")
    if tomogram:
        lines.append(f"map:     {T.dim(str(tomogram))}{tomo_note}")
    if cfg.get("enable_gateway"):
        lines.append(f"gateway: http://localhost:{gw}")
    if teleop_port:
        lines.append(f"teleop:  ws://0.0.0.0:{teleop_port}/teleop")
    lines.append(f"health:  {n} modules, {c} connections")
    lines.append(f"logs:    {T.dim(log_dir)}")

    print()
    _panel(lines, color=navy)
    print(f"  {T.dim('commands:')} {T.bold('help')}, {T.bold('status')}, {T.bold('teleop status')}, {T.bold('map list')}")
    print(f"  {T.dim('stop:')}     {T.bold('Ctrl+C')}\n")


def select_interactive():
    print(f"\n  {T.bold('LingTu — Select Profile')}\n")
    names = list(PROFILES.keys())
    for i, name in enumerate(names, 1):
        desc = PROFILES[name]["_desc"]
        print(f"  [{i}] {T.green(f'{name:10s}')} {desc}")
    print()
    while True:
        try:
            choice = input("  Select [1-{}] or name: ".format(len(names))).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if not choice:
            continue
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(names):
                return names[idx]
        if choice in PROFILES:
            return choice
        print(f"  {T.red('?')} Invalid: {choice}")


def ask_bool(prompt: str, *, default: bool | None = None) -> bool:
    """Ask a yes/no question in TTY, returning a boolean.

    Inputs accepted: y/yes, n/no, empty (uses default if provided).
    """
    if not sys.stdin.isatty():
        return bool(default) if default is not None else False

    suffix = ""
    if default is True:
        suffix = " [Y/n]"
    elif default is False:
        suffix = " [y/N]"
    else:
        suffix = " [y/n]"

    while True:
        try:
            raw = input(f"  {prompt}{suffix}: ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if raw == "" and default is not None:
            return default
        if raw in ("y", "yes"):
            return True
        if raw in ("n", "no"):
            return False
        print(f"  {T.red('?')} Please enter y or n")


def wizard_interactive(profile_name: str, cfg: dict) -> None:
    """TTY wizard: toggle semantic / gateway / teleop before build.

    Mutates cfg in place. Teleop requires gateway (HTTP stack).
    """
    if not sys.stdin.isatty():
        return

    print(f"\n  {T.bold('LingTu — startup options')}")
    print(f"  {T.dim('(press Enter to keep profile defaults)')}\n")

    sem_def = bool(cfg.get("enable_semantic", True))
    gw_def = bool(cfg.get("enable_gateway", True))
    tp_def = bool(cfg.get("enable_teleop", True))

    cfg["enable_semantic"] = ask_bool(
        "Enable semantic stack (perception, memory, LLM planner)?",
        default=sem_def,
    )
    cfg["enable_gateway"] = ask_bool(
        "Enable gateway (HTTP API + MCP)?",
        default=gw_def,
    )
    if cfg["enable_gateway"]:
        cfg["enable_teleop"] = ask_bool(
            "Enable teleop WebSocket (joystick + camera stream)?",
            default=tp_def,
        )
    else:
        cfg["enable_teleop"] = False


def list_profiles():
    print(f"\n  {T.bold('Available profiles:')}\n")
    for name, p in PROFILES.items():
        print(f"  {T.green(f'{name:10s}')} {p['_desc']}")
        parts = []
        if p.get("robot"):
            parts.append(f"robot={p['robot']}")
        if p.get("detector"):
            parts.append(f"detector={p['detector']}")
        if p.get("llm"):
            parts.append(f"llm={p['llm']}")
        if not p.get("enable_native"):
            parts.append("no-native")
        if not p.get("enable_semantic"):
            parts.append("no-semantic")
        print(f"  {T.dim('           ' + ', '.join(parts))}")
    print("\n  Override any flag: lingtu nav --llm mock\n")


def cmd_stop() -> None:
    state = read_run_state()
    if state is None:
        print("  No running instance found (.lingtu/run.json missing)")
        sys.exit(1)

    pid = state.get("pid")
    if not pid or not is_pid_alive(pid):
        print(f"  Stale PID {pid} (process not alive). Cleaning up.")
        clear_run_state()
        sys.exit(0)

    print(f"  Stopping PID {pid} (profile: {state.get('profile', '?')})...")
    try:
        os.kill(pid, signal.SIGTERM)
    except OSError as e:
        print(f"  Failed to stop: {e}")
        sys.exit(1)

    for _ in range(30):
        if not is_pid_alive(pid):
            print(f"  {T.green('Stopped.')}")
            clear_run_state()
            return
        time.sleep(0.5)

    print(f"  {T.yellow('Process still alive after 15s. Force kill?')}")
    try:
        os.kill(pid, signal.SIGKILL)
        clear_run_state()
        print(f"  {T.green('Force killed.')}")
    except OSError:
        print(f"  Could not kill PID {pid}. Manual cleanup needed.")


def cmd_status_external() -> None:
    state = read_run_state()
    if state is None:
        print("  No running instance")
        return

    pid = state.get("pid")
    alive = is_pid_alive(pid) if pid else False
    profile = state.get("profile", "?")
    started = state.get("started_at", "?")
    log_dir = state.get("log_dir", "?")

    status = T.green("running") if alive else T.red("dead (stale PID)")
    print(f"\n  Status:    {status}")
    print(f"  PID:       {pid}")
    print(f"  Profile:   {profile}")
    print(f"  Started:   {started}")
    print(f"  Logs:      {log_dir}")

    if not alive:
        print(f"\n  {T.yellow('Stale run state. Run `lingtu stop` to clean up.')}")
    print()
