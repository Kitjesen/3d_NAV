"""Banner, profile listing, stop/status helpers."""

from __future__ import annotations

import os
import signal
import sys
import time

from . import term as T
from .profiles_data import PROFILES
from .run_state import clear_run_state, is_pid_alive, read_run_state

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
    "nav":     ("◉", "Navigate with pre-built map",    T.green),
    "explore": ("◎", "Explore unknown terrain",         T.cyan),
    "map":     ("⊕", "Build a new map (SLAM + PGO)",    T.yellow),
    "sim":     ("◈", "MuJoCo simulation (full stack)",  T.blue),
    "dev":     ("◇", "Semantic pipeline (no hardware)", T.navy),
    "stub":    ("○", "Framework testing only",          T.dim),
}


def _print_logo() -> None:
    for line in _LOGO_LINES:
        print(T.navy(line))
    print(T.dim(_TAGLINE))
    print()


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
    n   = len(system.modules)
    nc  = len(system.connections)
    desc = cfg.get("_desc", "custom")
    gw   = cfg.get("gateway_port", 5050)

    robot    = cfg.get("robot", "?")
    planner  = cfg.get("planner", "?")
    detector = cfg.get("detector", "?")
    llm      = cfg.get("llm", "?")

    teleop_port = None
    try:
        teleop_port = system.get_module("TeleopModule").get_teleop_status().get("port")
    except Exception:
        pass

    tomogram  = cfg.get("tomogram", "")
    tomo_note = T.yellow(" ⚠ sample map") if (tomogram and "building2_9.pickle" in str(tomogram)) else ""

    icon, _, color = _PROFILE_META.get(profile_name, ("·", desc, T.dim))
    W = 56

    _print_logo()

    print(T.navy(f"  ┌{'─' * W}┐"))
    print(T.navy("  │") + T.bold(f"{'  System Ready':^{W}}") + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))

    def _row(label: str, value: str) -> None:
        inner = f"  {T.dim(f'{label:<10}')} {value}"
        import re as _re
        vis = _re.sub(r'\033\[[0-9;]*m', '', inner)
        pad = max(0, W - len(vis))
        print(T.navy("  │") + inner + " " * pad + T.navy("│"))

    _row("profile",  color(f"{icon} {profile_name}") + T.dim(f"  {desc}"))
    _row("robot",    f"{robot}  {T.dim('planner:')} {planner}")
    if cfg.get("enable_semantic"):
        _row("semantic", f"{T.dim('detector:')} {detector}  {T.dim('llm:')} {llm}")
    else:
        _row("semantic", T.dim("disabled"))
    if tomogram:
        import os as _os
        tomo_short = _os.path.basename(tomogram)
        _row("map",      T.dim(tomo_short) + tomo_note)
    if cfg.get("enable_gateway"):
        _row("gateway",  T.cyan(f"http://localhost:{gw}"))
    if teleop_port:
        _row("teleop",   T.cyan(f"ws://0.0.0.0:{teleop_port}/teleop"))
    _row("health",   T.green(f"✓ {n} modules") + T.dim(f"  {nc} connections"))
    _row("logs",     T.dim(log_dir))

    print(T.navy(f"  ├{'─' * W}┤"))
    hint = f"  help · status · map list · teleop status · Ctrl+C to quit"
    print(T.navy("  │") + T.dim(f"{hint:^{W}}") + T.navy("│"))
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
        # right-pad to fill card width
        inner = f"{num}{tag}{body}"
        # strip ANSI for length calc
        import re as _re
        visible = _re.sub(r'\033\[[0-9;]*m', '', inner)
        pad = max(0, W - len(visible))
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
    """TTY wizard: toggle semantic / gateway / teleop before build.

    Mutates cfg in place. Teleop requires gateway (HTTP stack).
    """
    if not sys.stdin.isatty():
        return

    icon, _, color = _PROFILE_META.get(profile_name, ("·", "", T.dim))
    W = 54
    print(T.navy(f"  ┌{'─' * W}┐"))
    print(T.navy("  │") + T.bold(f"{'  Configure launch options':^{W}}") + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))
    print(T.navy("  │") + f"  profile: {color(f'{icon} {profile_name}')}" + " " * (W - 12 - len(profile_name)) + T.navy("│"))
    print(T.navy(f"  ├{'─' * W}┤"))
    print(T.navy("  │") + T.dim(f"  Press Enter to accept defaults shown in [brackets]") + " " * 3 + T.navy("│"))
    print(T.navy(f"  └{'─' * W}┘"))
    print()

    sem_def = bool(cfg.get("enable_semantic", True))
    gw_def  = bool(cfg.get("enable_gateway",  True))
    tp_def  = bool(cfg.get("enable_teleop",   True))

    cfg["enable_semantic"] = ask_bool(
        f"  {T.cyan('◈')} Semantic stack  {T.dim('(perception · memory · LLM planner)')}",
        default=sem_def,
    )
    cfg["enable_gateway"] = ask_bool(
        f"  {T.cyan('◈')} Gateway         {T.dim('(HTTP API · MCP tools)')}",
        default=gw_def,
    )
    if cfg["enable_gateway"]:
        cfg["enable_teleop"] = ask_bool(
            f"  {T.cyan('◈')} Teleop          {T.dim('(WebSocket joystick · camera stream)')}",
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
