"""Banner, profile listing, stop/status helpers."""

from __future__ import annotations

import os
import signal
import sys
import time

from . import term as T
from .profiles_data import PROFILES
from .run_state import clear_run_state, is_pid_alive, read_run_state


LOGO = r"""\
  _     _             _____
 | |   (_)           |_   _|
 | |    _ _ __   __ _  | |  _   _
 | |   | | '_ \ / _` | | | | | | |
 | |___| | | | | (_| |_| |_| |_| |
 \_____/_|_| |_|\__, |_____|\__,_|
                  __/ |
                 |___/
"""


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

    print()
    for line in LOGO.splitlines():
        if line.strip():
            print(f"{T.blue(line)}")
    print(f"{T.bold('=' * 56)}")
    print(f"  {T.bold('LingTu Navigation System')}")
    print(f"{T.bold('=' * 56)}")
    print(f"  Profile:   {T.green(profile_name)} — {desc}")
    print(f"  Robot:     {robot:12s}  Planner: {planner}")
    if cfg.get("enable_semantic"):
        print(f"  Semantic:  detector={detector:10s}  llm={llm}")
    else:
        print(f"  Semantic:  {T.dim('disabled')}")
    if cfg.get("enable_gateway"):
        print(f"  Gateway:   http://localhost:{gw}")
    if teleop_port:
        print(f"  Teleop:    ws://0.0.0.0:{teleop_port}/teleop")
    if tomogram:
        print(f"  Map:       {T.dim(str(tomogram))}{tomo_note}")
    if cfg.get("dog_host") and cfg["dog_host"] != "127.0.0.1":
        print(f"  Robot host:{cfg['dog_host']}:{cfg.get('dog_port', 13145)}")
    print(f"  Health:    {n} modules, {c} connections")
    print(f"  Logs:      {T.dim(log_dir)}")
    print(f"  PID:       {os.getpid()}")
    print(f"{T.bold('-' * 56)}")
    print(f"  Commands:  {T.bold('help')}, {T.bold('status')}, {T.bold('teleop status')}, {T.bold('map list')}")
    print(f"  Stop:      {T.bold('Ctrl+C')}\n")


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
