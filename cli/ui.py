"""Banner, profile listing, stop/status helpers."""

from __future__ import annotations

import os
import signal
import sys
import time

from . import term as T
from .profiles_data import PROFILES
from .run_state import clear_run_state, is_pid_alive, read_run_state


def print_banner(profile_name, cfg, system, log_dir: str) -> None:
    n = len(system.modules)
    c = len(system.connections)
    desc = cfg.get("_desc", "custom")
    gw = cfg.get("gateway_port", 5050)

    print(f"\n{T.bold('=' * 56)}")
    print(f"  {T.bold('LingTu Navigation System')}")
    print(f"{T.bold('=' * 56)}")
    print(f"  Profile:     {T.green(profile_name)} — {desc}")
    print(f"  Modules:     {n}    Connections: {c}")
    print(f"  Robot:       {cfg.get('robot', '?'):12s}  Planner: {cfg.get('planner', '?')}")
    print(f"  Detector:    {cfg.get('detector', '?'):12s}  LLM:     {cfg.get('llm', '?')}")
    if cfg.get("enable_gateway"):
        print(f"  Gateway:     http://localhost:{gw}")
    if cfg.get("enable_rerun"):
        print("  Rerun:       http://localhost:9090")
    if cfg.get("dog_host") and cfg["dog_host"] != "127.0.0.1":
        print(f"  Robot host:  {cfg['dog_host']}:{cfg.get('dog_port', 13145)}")
    print(f"  Logs:        {T.dim(log_dir)}")
    print(f"  PID:         {os.getpid()}")
    print(f"{T.bold('-' * 56)}")
    for name, mod in sorted(system.modules.items(), key=lambda x: x[1].layer or 0):
        layer = f"L{mod.layer}" if mod.layer is not None else "L?"
        print(f"  [{layer}] {name}")
    print(f"{T.bold('=' * 56)}")
    print(f"  Type {T.bold('help')} for commands, {T.bold('Ctrl+C')} to stop.\n")


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
