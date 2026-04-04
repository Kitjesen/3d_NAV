"""Banner, profile listing, stop/status helpers."""

from __future__ import annotations

import os
import re
import signal
import sys
import time

from . import term as T
from .profiles_data import PROFILES
from .run_state import clear_run_state, is_pid_alive, read_run_state


def _vlen(s: str) -> int:
    """Visible length of a string — strips ANSI escape codes."""
    return len(re.sub(r'\033\[[0-9;]*m', '', s))

# ── LOGO ──────────────────────────────────────────────────────────────────────
# figlet font: "ANSI Shadow"
_LOGO_LINES = [
    "  ██╗     ██╗███╗   ██╗ ██████╗     ████████╗██╗   ██╗",
    "  ██║     ██║████╗  ██║██╔════╝        ██╔══╝██║   ██║",
    "  ██║     ██║██╔██╗ ██║██║  ███╗       ██║   ██║   ██║",
    "  ██║     ██║██║╚██╗██║██║   ██║       ██║   ██║   ██║",
    "  ███████╗██║██║ ╚████║╚██████╔╝       ██║   ╚██████╔╝",
    "  ╚══════╝╚═╝╚═╝  ╚═══╝ ╚═════╝        ╚═╝    ╚═════╝ ",
]
_TAGLINE = "  Autonomous Navigation for Quadruped Robots"

# Profile icons and accent colors
_PROFILE_META = {
    "nav":     ("◉", "Navigate using a saved map",                  T.green),
    "explore": ("◎", "Explore unknown area",                         T.cyan),
    "map":     ("⊕", "Build a new map",                              T.yellow),
    "sim":     ("◈", "MuJoCo simulation",                            T.blue),
    "dev":     ("◇", "Test perception & planning without a robot",   T.navy),
    "stub":    ("○", "Framework testing only",                       T.dim),
}

# Which wizard questions are relevant per profile.
# Keys: "semantic", "gateway", "teleop"
_PROFILE_WIZARD: dict[str, tuple[bool, bool, bool]] = {
    #                  semantic  gateway  teleop
    "nav":     (True,  True,  True),   # full stack
    "explore": (True,  True,  False),  # exploring — no joystick needed
    "map":     (False, True,  False),  # mapping — no semantics, no joystick
    "sim":     (True,  True,  True),   # full stack in sim
    "dev":     (True,  True,  False),  # no robot → no teleop
    "stub":    (False, True,  False),  # bare framework
}


def _print_logo() -> None:
    for line in _LOGO_LINES:
        print(T.navy(line))
    print(T.dim(_TAGLINE))
    print()


def _panel(lines: list[str], *, color) -> None:
    """Print a compact config panel — ANSI-aware width."""
    if not lines:
        return
    width = max(_vlen(x) for x in lines)
    top = f"┌{'─' * (width + 2)}┐"
    bot = f"└{'─' * (width + 2)}┘"
    print(f"  {color(top)}")
    for line in lines:
        pad = width - _vlen(line)
        print(f"  {color('│')} {line}{' ' * pad} {color('│')}")
    print(f"  {color(bot)}")


def _local_ip() -> str:
    """Best-effort local LAN IP (not 127.0.0.1)."""
    import socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def print_banner(profile_name, cfg, system, log_dir: str) -> None:
    n   = len(system.modules)
    nc  = len(system.connections)
    desc = cfg.get("_desc", "custom")
    gw   = cfg.get("gateway_port", 5050)

    robot    = cfg.get("robot", "?")
    planner  = cfg.get("planner", "?")
    detector = cfg.get("detector", "?")
    llm      = cfg.get("llm", "?")

    # Wait briefly for TeleopModule to finish binding its port
    import time as _time
    teleop_port = None
    for _ in range(10):
        try:
            st = system.get_module("TeleopModule").get_teleop_status()
            if st.get("port"):
                teleop_port = st["port"]
                break
        except Exception:
            break
        _time.sleep(0.1)

    tomogram  = cfg.get("tomogram", "")
    tomo_note = T.yellow(" ⚠ sample map") if (tomogram and "building2_9.pickle" in str(tomogram)) else ""

    icon, _, color = _PROFILE_META.get(profile_name, ("·", desc, T.dim))
    W = 58

    # Resolve the machine's LAN IP so the user can copy-paste the URL
    lan_ip = _local_ip()

    _print_logo()

    print(T.navy(f"  ┌{'─' * W}┐"))
    print(T.navy("  │") + T.bold(f"{'  System Ready':^{W}}") + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))

    def _row(label: str, value: str) -> None:
        inner = f"  {T.dim(f'{label:<10}')} {value}"
        pad = max(0, W - _vlen(inner))
        print(T.navy("  │") + inner + " " * pad + T.navy("│"))

    _row("profile",  color(f"{icon} {profile_name}") + T.dim(f"  {desc}"))
    _row("robot",    f"{robot}  {T.dim('planner:')} {planner}")
    if cfg.get("enable_semantic"):
        _row("semantic", f"{T.dim('detector:')} {detector}  {T.dim('llm:')} {llm}")
    else:
        _row("semantic", T.dim("disabled"))
    if tomogram:
        tomo_short = os.path.basename(tomogram)
        _row("map",      T.dim(tomo_short) + tomo_note)
    if cfg.get("enable_gateway"):
        _row("gateway",  T.cyan(f"http://{lan_ip}:{gw}"))
    if teleop_port:
        _row("teleop",   T.cyan(f"ws://{lan_ip}:{teleop_port}/teleop"))
    _row("health",   T.green(f"✓ {n} modules") + T.dim(f"  {nc} connections"))
    _row("logs",     T.dim(log_dir))

    print(T.navy(f"  ├{'─' * W}┤"))
    hint_raw = "help · status · map list · teleop status · Ctrl+C to quit"
    hint_pad = max(0, W - len(hint_raw))
    lpad = hint_pad // 2
    rpad = hint_pad - lpad
    hint_line = T.dim(" " * lpad + hint_raw + " " * rpad)
    print(T.navy("  │") + hint_line + T.navy("│"))
    print(T.navy(f"  └{'─' * W}┘"))
    print()


def select_interactive() -> str:
    """Full-screen profile picker with LOGO and styled cards."""
    _print_logo()

    names = list(PROFILES.keys())
    W = 54  # card inner width

    print(T.navy(f"  ┌{'─' * W}┐"))
    print(T.navy(f"  │") + T.bold(f"{'  Select a profile to launch':^{W}}") + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))

    for i, name in enumerate(names, 1):
        icon, desc, color = _PROFILE_META.get(name, ("·", PROFILES[name].get("_desc", ""), T.dim))
        num  = T.dim(f" {i} ")
        tag  = color(f" {icon} {name:<9}")
        body = T.dim(f" {desc}")
        inner = f"{num}{tag}{body}"
        pad = max(0, W - _vlen(inner))
        print(T.navy("  │") + inner + " " * pad + T.navy("│"))

    print(T.navy(f"  └{'─' * W}┘"))
    print()
    print(T.dim("  ↑↓ type number or name  ·  Ctrl+C to quit"))
    print()

    while True:
        try:
            choice = input(T.cyan("  › ")).strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if not choice:
            continue
        if choice.isdigit():
            idx = int(choice) - 1
            if 0 <= idx < len(names):
                selected = names[idx]
                icon, _, color = _PROFILE_META.get(selected, ("·", "", T.dim))
                print(f"\n  {color(f'✓ {selected}')}  {T.dim(PROFILES[selected].get('_desc', ''))}\n")
                return selected
        if choice in PROFILES:
            icon, _, color = _PROFILE_META.get(choice, ("·", "", T.dim))
            print(f"\n  {color(f'✓ {choice}')}  {T.dim(PROFILES[choice].get('_desc', ''))}\n")
            return choice
        print(f"  {T.red('✗')} {T.dim(f'Unknown: {choice!r}  (try 1–{len(names)})')}")


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
    """TTY wizard: show only the toggles that make sense for this profile.

    Mutates cfg in place.
    """
    if not sys.stdin.isatty():
        return

    ask_sem, ask_gw, ask_tp = _PROFILE_WIZARD.get(
        profile_name, (True, True, True)
    )

    # Nothing to ask — skip the wizard entirely
    if not (ask_sem or ask_gw or ask_tp):
        return

    icon, _, color = _PROFILE_META.get(profile_name, ("·", "", T.dim))
    W = 54
    print(T.navy(f"  ┌{'─' * W}┐"))
    print(T.navy("  │") + T.bold(f"{'  Configure launch options':^{W}}") + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))
    profile_val = color(f"{icon} {profile_name}")
    profile_row = f"  profile: {profile_val}"
    print(T.navy("  │") + profile_row + " " * max(0, W - _vlen(profile_row)) + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))
    hint_row = T.dim("  Press Enter to accept defaults shown in [brackets]")
    print(T.navy("  │") + hint_row + " " * max(0, W - _vlen(hint_row)) + T.navy("│"))
    print(T.navy(f"  └{'─' * W}┘"))
    print()

    sem_def = bool(cfg.get("enable_semantic", True))
    gw_def  = bool(cfg.get("enable_gateway",  True))
    tp_def  = bool(cfg.get("enable_teleop",   True))

    if ask_sem:
        cfg["enable_semantic"] = ask_bool(
            f"  {T.cyan('◈')} Semantic  {T.dim('(object detection · memory · LLM reasoning)')}",
            default=sem_def,
        )

    if ask_gw:
        cfg["enable_gateway"] = ask_bool(
            f"  {T.cyan('◈')} Gateway   {T.dim('(HTTP API · MCP tools for external control)')}",
            default=gw_def,
        )

    if ask_tp and cfg.get("enable_gateway", gw_def):
        cfg["enable_teleop"] = ask_bool(
            f"  {T.cyan('◈')} Teleop    {T.dim('(joystick remote control · live camera)')}",
            default=tp_def,
        )
    else:
        cfg["enable_teleop"] = False

    print()


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
