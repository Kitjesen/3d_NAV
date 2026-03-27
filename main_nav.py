#!/usr/bin/env python3
"""LingTu Navigation System — CLI with interactive REPL.

Usage:
    lingtu                          # interactive profile selector
    lingtu stub                     # no hardware, framework testing
    lingtu sim                      # MuJoCo kinematic simulation
    lingtu dev                      # semantic pipeline, no C++ nodes
    lingtu s100p                    # real S100P robot (BPU + Kimi)
    lingtu explore                  # exploration, no pre-built map
    lingtu s100p --daemon           # background daemon (S100P)
    lingtu stop                     # stop running daemon
    lingtu --list                   # list all profiles

REPL commands:
    navigate <x> <y> [z]     send navigation goal
    go <instruction>          semantic navigation ("go find the table")
    stop                      emergency stop — halt all motion
    cancel                    cancel current mission
    status / s                module overview + message counts
    health / h                detailed health + startup order
    watch [sec]               live status refresh (Ctrl+C to stop)
    module <name> / m         inspect module ports
    connections / c           show all wiring
    log <level>               change log level at runtime
    config                    show current configuration
    quit / q                  graceful shutdown
"""

from __future__ import annotations

import argparse
import cmd
import json
import logging
import os
import signal
import sys
import textwrap
import threading
import time
from pathlib import Path

_SRC = os.path.join(os.path.dirname(__file__), "src")
sys.path.insert(0, _SRC)

# ROS2-style sub-packages use bare imports (e.g. `from semantic_common import ...`).
# In colcon workspace these are installed; for Module-First CLI, add parent dirs.
for _sub in ("semantic/common", "semantic/planner", "semantic/perception"):
    _p = os.path.join(_SRC, _sub)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

logger = logging.getLogger("lingtu")

_PROJECT_ROOT = Path(__file__).resolve().parent
_RUN_DIR = _PROJECT_ROOT / ".lingtu"
_PID_FILE = _RUN_DIR / "run.pid"
_RUN_FILE = _RUN_DIR / "run.json"
_LOG_DIR = _PROJECT_ROOT / "logs"


# ── ANSI helpers (degrade on non-TTY) ─────────────────────────────────

_IS_TTY = hasattr(sys.stdout, "isatty") and sys.stdout.isatty()

def _c(code: str, text: str) -> str:
    return f"\033[{code}m{text}\033[0m" if _IS_TTY else text

def _bold(t: str) -> str:   return _c("1", t)
def _green(t: str) -> str:  return _c("0;32", t)
def _blue(t: str) -> str:   return _c("0;34", t)
def _yellow(t: str) -> str: return _c("1;33", t)
def _red(t: str) -> str:    return _c("0;31", t)
def _dim(t: str) -> str:    return _c("2", t)


# ── Profiles ──────────────────────────────────────────────────────────

PROFILES = {
    "stub": dict(
        _desc="No hardware, framework testing",
        robot="stub", detector="yoloe", encoder="mobileclip",
        llm="mock", planner="astar",
        enable_native=False, enable_semantic=False, enable_gateway=True,
        gateway_port=5050,
    ),
    "sim": dict(
        _desc="MuJoCo simulation (full algorithm stack)",
        robot="sim_ros2", detector="yoloe", encoder="mobileclip",
        llm="mock", planner="astar",
        enable_native=False, enable_semantic=True, enable_gateway=True,
        gateway_port=5050,
    ),
    "dev": dict(
        _desc="Semantic pipeline, no C++ nodes",
        robot="stub", detector="yoloe", encoder="mobileclip",
        llm="mock", planner="astar",
        enable_native=False, enable_semantic=True, enable_gateway=True,
        gateway_port=5050,
    ),
    "s100p": dict(
        _desc="Full S100P robot (BPU + Kimi)",
        robot="thunder", detector="bpu", encoder="mobileclip",
        llm="kimi", planner="astar",
        enable_native=True, enable_semantic=True, enable_gateway=True,
        gateway_port=5050,
        dog_host="192.168.66.190", dog_port=13145,
    ),
    "explore": dict(
        _desc="Exploration, no pre-built map needed",
        robot="thunder", detector="bpu", encoder="mobileclip",
        llm="kimi", planner="astar",
        enable_native=True, enable_semantic=True, enable_gateway=True,
        gateway_port=5050,
        dog_host="192.168.66.190", dog_port=13145,
    ),
}


# ── Run state (PID + registry) ───────────────────────────────────────

def _save_run_state(profile_name: str, cfg: dict, log_path: str):
    _RUN_DIR.mkdir(exist_ok=True)
    _PID_FILE.write_text(str(os.getpid()))
    _RUN_FILE.write_text(json.dumps({
        "pid": os.getpid(),
        "profile": profile_name,
        "started_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "log_dir": log_path,
        "config": {k: v for k, v in cfg.items() if not k.startswith("_")},
    }, indent=2))


def _clear_run_state():
    _PID_FILE.unlink(missing_ok=True)
    _RUN_FILE.unlink(missing_ok=True)


def _read_run_state() -> dict | None:
    if not _RUN_FILE.exists():
        return None
    try:
        return json.loads(_RUN_FILE.read_text())
    except (json.JSONDecodeError, OSError):
        return None


def _is_pid_alive(pid: int) -> bool:
    try:
        if os.name == "nt":
            import ctypes
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.OpenProcess(0x1000, False, pid)  # PROCESS_QUERY_LIMITED_INFORMATION
            if handle:
                kernel32.CloseHandle(handle)
                return True
            return False
        else:
            os.kill(pid, 0)
            return True
    except (OSError, ProcessLookupError):
        return False


# ── Structured logging ────────────────────────────────────────────────

def _setup_logging(level: str, profile_name: str) -> str:
    """Setup logging: stderr + per-run file. Returns log directory path."""
    ts = time.strftime("%Y%m%d_%H%M%S")
    log_dir = _LOG_DIR / f"{ts}_{profile_name}"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_file = log_dir / "lingtu.log"

    fmt = logging.Formatter(
        "%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # stderr handler (REPL-friendly)
    stderr_h = logging.StreamHandler(sys.stderr)
    stderr_h.setFormatter(fmt)

    # file handler (structured, persistent)
    file_h = logging.FileHandler(str(log_file), encoding="utf-8")
    file_h.setFormatter(logging.Formatter(
        "%(asctime)s\t%(levelname)s\t%(name)s\t%(message)s",
    ))

    root = logging.getLogger()
    root.setLevel(getattr(logging, level.upper(), logging.INFO))
    root.addHandler(stderr_h)
    root.addHandler(file_h)

    return str(log_dir)


# ── Pre-start health check ───────────────────────────────────────────

def _health_check(system) -> bool:
    """Verify all modules were built and have valid ports."""
    ok = True
    for name, mod in system.modules.items():
        if mod is None:
            logger.error("Health check: module %s is None", name)
            ok = False
            continue
        # Verify ports are initialized
        if not hasattr(mod, "ports_in") or not hasattr(mod, "ports_out"):
            logger.error("Health check: %s missing ports", name)
            ok = False
    return ok


# ── Interactive REPL ──────────────────────────────────────────────────

class LingTuREPL(cmd.Cmd):
    """Runtime control REPL with navigation commands."""

    prompt = _c("1;34", "lingtu> ") if _IS_TTY else "lingtu> "

    def __init__(self, system, profile_cfg):
        super().__init__()
        self._system = system
        self._cfg = profile_cfg

    def _get_module(self, name: str):
        try:
            return self._system.get_module(name)
        except KeyError:
            return None

    # ── Navigation commands ──

    def do_navigate(self, arg):
        """Send navigation goal: navigate <x> <y> [z]"""
        parts = arg.split()
        if len(parts) < 2:
            print("  Usage: navigate <x> <y> [z]")
            return
        try:
            x, y = float(parts[0]), float(parts[1])
            z = float(parts[2]) if len(parts) > 2 else 0.0
        except ValueError:
            print("  Error: coordinates must be numbers")
            return

        nav = self._get_module("NavigationModule")
        if not nav or not hasattr(nav, "goal_pose"):
            print("  NavigationModule not available")
            return

        from core.msgs.geometry import PoseStamped, Pose, Vector3, Quaternion
        goal = PoseStamped(pose=Pose(
            position=Vector3(x=x, y=y, z=z),
            orientation=Quaternion(x=0, y=0, z=0, w=1),
        ))
        nav.goal_pose._deliver(goal)
        print(f"  Goal -> ({x:.1f}, {y:.1f}, {z:.1f})")

    do_nav = do_navigate

    def do_go(self, arg):
        """Semantic navigation: go <natural language instruction>"""
        if not arg:
            print("  Usage: go <instruction>  (e.g. go find the table)")
            return
        nav = self._get_module("NavigationModule")
        if not nav or not hasattr(nav, "instruction"):
            print("  NavigationModule not available")
            return
        nav.instruction._deliver(arg)
        print(f"  Instruction -> {arg}")

    def do_stop(self, arg):
        """Emergency stop — halt all motion immediately."""
        count = 0
        for name, mod in self._system.modules.items():
            if hasattr(mod, "stop_signal") and hasattr(mod.stop_signal, "_deliver"):
                mod.stop_signal._deliver(2)  # 2 = hard stop
                count += 1
        print(f"  {_red('STOP')} sent to {count} module(s)")

    def do_cancel(self, arg):
        """Cancel current navigation mission."""
        nav = self._get_module("NavigationModule")
        if not nav or not hasattr(nav, "cancel"):
            print("  NavigationModule not available")
            return
        nav.cancel._deliver("user")
        print("  Mission cancelled")

    # ── Monitoring commands ──

    def do_status(self, arg):
        """Module overview with layer tags and message counts."""
        s = self._system
        if not s.started:
            print("  System not started")
            return
        mods = s.modules
        print(f"\n  {len(mods)} modules, {len(s.connections)} connections\n")
        for name, mod in sorted(mods.items(), key=lambda x: x[1].layer or 0):
            layer = f"L{mod.layer}" if mod.layer is not None else "L?"
            n_in = sum(p.msg_count for p in mod.ports_in.values())
            n_out = sum(p.msg_count for p in mod.ports_out.values())
            traffic = f"in:{n_in} out:{n_out}" if (n_in + n_out) > 0 else _dim("idle")
            print(f"  [{layer}] {name:30s} {traffic}")
        # Show mission state if available
        nav = self._get_module("NavigationModule")
        if nav and hasattr(nav, "_state"):
            print(f"\n  Mission: {_bold(nav._state)}")
        print()

    do_s = do_status

    def do_health(self, arg):
        """Detailed health — totals, startup order, violations."""
        s = self._system
        if not s.started:
            print("  System not started")
            return
        h = s.health()
        print(f"\n  Modules: {h['module_count']}  Connections: {h['connection_count']}")
        print(f"  Total in: {h['total_messages_in']}  Total out: {h['total_messages_out']}")
        if h["layer_violations"]:
            for v in h["layer_violations"]:
                print(f"  {_red('!')} {v}")
        print(f"\n  Startup order:")
        for i, name in enumerate(h["startup_order"], 1):
            print(f"    {i}. {name}")
        print()

    do_h = do_health

    def do_watch(self, arg):
        """Live status refresh: watch [interval_sec]. Ctrl+C to stop."""
        interval = float(arg) if arg else 2.0
        try:
            while True:
                # Clear screen
                os.system("cls" if os.name == "nt" else "clear")
                ts = time.strftime("%H:%M:%S")
                print(f"  {_dim(f'[{ts}]  refresh every {interval:.0f}s  Ctrl+C to stop')}\n")
                self.do_status("")
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n  Watch stopped")

    do_w = do_watch

    def do_module(self, arg):
        """Inspect a module: module <name>"""
        if not arg:
            print("  Usage: module <name>")
            return
        try:
            mod = self._system.get_module(arg)
        except KeyError:
            matches = [n for n in self._system.modules if arg.lower() in n.lower()]
            if matches:
                print(f"  Unknown '{arg}'. Did you mean: {', '.join(matches)}")
            else:
                print(f"  Unknown module: {arg}")
            return
        layer = f"L{mod.layer}" if mod.layer is not None else "L?"
        print(f"\n  {_bold(arg)} [{layer}]  ({type(mod).__name__})")
        if mod.ports_in:
            print("  Inputs:")
            for pname, port in mod.ports_in.items():
                policy = getattr(port, "_policy_name", "all")
                print(f"    {pname}: {port.msg_type.__name__}  "
                      f"count={port.msg_count}  policy={policy}")
        if mod.ports_out:
            print("  Outputs:")
            for pname, port in mod.ports_out.items():
                n_subs = len(getattr(port, "_callbacks", []))
                print(f"    {pname}: {port.msg_type.__name__}  "
                      f"count={port.msg_count}  subs={n_subs}")
        print()

    do_m = do_module

    def complete_module(self, text, line, begidx, endidx):
        return [n for n in self._system.modules if n.lower().startswith(text.lower())]

    complete_m = complete_module

    def do_connections(self, arg):
        """Show all port wiring."""
        conns = self._system.connections
        if not conns:
            print("  No connections")
            return
        print(f"\n  {len(conns)} connections:\n")
        for out_mod, out_port, in_mod, in_port in conns:
            print(f"  {out_mod}.{out_port} {_dim('->')} {in_mod}.{in_port}")
        print()

    do_c = do_connections

    def do_log(self, arg):
        """Change log level: log debug|info|warning|error"""
        if not arg:
            cur = logging.getLevelName(logging.getLogger().level)
            print(f"  Current: {cur}")
            print("  Usage: log debug|info|warning|error")
            return
        lvl = getattr(logging, arg.upper(), None)
        if lvl is None:
            print(f"  Invalid level: {arg}")
            return
        logging.getLogger().setLevel(lvl)
        print(f"  Log level -> {arg.upper()}")

    def do_config(self, arg):
        """Show current configuration."""
        print()
        for k, v in sorted(self._cfg.items()):
            if not k.startswith("_"):
                print(f"  {k:20s} {v}")
        print()

    def do_quit(self, arg):
        """Graceful shutdown."""
        return True

    do_q = do_quit
    do_exit = do_quit

    def do_EOF(self, arg):
        print()
        return True

    def default(self, line):
        if line.strip():
            print(f"  Unknown: {line}. Type {_bold('help')} for commands.")

    def emptyline(self):
        pass


# ── Banner ────────────────────────────────────────────────────────────

def _print_banner(profile_name, cfg, system, log_dir):
    n = len(system.modules)
    c = len(system.connections)
    desc = cfg.get("_desc", "custom")
    gw = cfg.get("gateway_port", 5050)

    print(f"\n{_bold('=' * 56)}")
    print(f"  {_bold('LingTu Navigation System')}")
    print(f"{_bold('=' * 56)}")
    print(f"  Profile:     {_green(profile_name)} — {desc}")
    print(f"  Modules:     {n}    Connections: {c}")
    print(f"  Robot:       {cfg.get('robot', '?'):12s}  Planner: {cfg.get('planner', '?')}")
    print(f"  Detector:    {cfg.get('detector', '?'):12s}  LLM:     {cfg.get('llm', '?')}")
    if cfg.get("enable_gateway"):
        print(f"  Gateway:     http://localhost:{gw}")
    if cfg.get("dog_host") and cfg["dog_host"] != "127.0.0.1":
        print(f"  Robot host:  {cfg['dog_host']}:{cfg.get('dog_port', 13145)}")
    print(f"  Logs:        {_dim(log_dir)}")
    print(f"  PID:         {os.getpid()}")
    print(f"{_bold('-' * 56)}")
    for name, mod in sorted(system.modules.items(), key=lambda x: x[1].layer or 0):
        layer = f"L{mod.layer}" if mod.layer is not None else "L?"
        print(f"  [{layer}] {name}")
    print(f"{_bold('=' * 56)}")
    print(f"  Type {_bold('help')} for commands, {_bold('Ctrl+C')} to stop.\n")


# ── Profile selector ─────────────────────────────────────────────────

def _select_interactive():
    print(f"\n  {_bold('LingTu — Select Profile')}\n")
    names = list(PROFILES.keys())
    for i, name in enumerate(names, 1):
        desc = PROFILES[name]["_desc"]
        print(f"  [{i}] {_green(f'{name:10s}')} {desc}")
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
        print(f"  {_red('?')} Invalid: {choice}")


def _list_profiles():
    print(f"\n  {_bold('Available profiles:')}\n")
    for name, p in PROFILES.items():
        print(f"  {_green(f'{name:10s}')} {p['_desc']}")
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
        print(f"  {_dim('           ' + ', '.join(parts))}")
    print(f"\n  Override any flag: lingtu s100p --llm mock\n")


# ── Special commands ──────────────────────────────────────────────────

def _cmd_stop():
    """Stop a running daemon by PID file."""
    state = _read_run_state()
    if state is None:
        print("  No running instance found (.lingtu/run.json missing)")
        sys.exit(1)

    pid = state.get("pid")
    if not pid or not _is_pid_alive(pid):
        print(f"  Stale PID {pid} (process not alive). Cleaning up.")
        _clear_run_state()
        sys.exit(0)

    print(f"  Stopping PID {pid} (profile: {state.get('profile', '?')})...")
    try:
        if os.name == "nt":
            os.kill(pid, signal.SIGTERM)
        else:
            os.kill(pid, signal.SIGTERM)
    except OSError as e:
        print(f"  Failed to stop: {e}")
        sys.exit(1)

    # Wait for process to exit
    for _ in range(30):
        if not _is_pid_alive(pid):
            print(f"  {_green('Stopped.')}")
            _clear_run_state()
            return
        time.sleep(0.5)

    print(f"  {_yellow('Process still alive after 15s. Force kill?')}")
    try:
        if os.name == "nt":
            os.kill(pid, signal.SIGTERM)
        else:
            os.kill(pid, signal.SIGKILL)
        _clear_run_state()
        print(f"  {_green('Force killed.')}")
    except OSError:
        print(f"  Could not kill PID {pid}. Manual cleanup needed.")


def _cmd_status_external():
    """Show status of a running daemon."""
    state = _read_run_state()
    if state is None:
        print("  No running instance")
        return

    pid = state.get("pid")
    alive = _is_pid_alive(pid) if pid else False
    profile = state.get("profile", "?")
    started = state.get("started_at", "?")
    log_dir = state.get("log_dir", "?")

    status = _green("running") if alive else _red("dead (stale PID)")
    print(f"\n  Status:    {status}")
    print(f"  PID:       {pid}")
    print(f"  Profile:   {profile}")
    print(f"  Started:   {started}")
    print(f"  Logs:      {log_dir}")

    if not alive:
        print(f"\n  {_yellow('Stale run state. Run `lingtu stop` to clean up.')}")
    print()


# ── Daemon ────────────────────────────────────────────────────────────

def _daemonize(log_file: str):
    """Unix: fork and detach. Windows: warn and stay foreground."""
    if os.name == "nt":
        print(f"  {_yellow('Daemon mode not supported on Windows. Running in foreground.')}")
        return False

    # First fork
    pid = os.fork()
    if pid > 0:
        # Parent: report and exit
        print(f"  Daemon started (PID {pid})")
        print(f"  Logs: {log_file}")
        print(f"  Stop: lingtu stop")
        sys.exit(0)

    # Child: new session
    os.setsid()

    # Second fork (prevent re-acquiring terminal)
    pid = os.fork()
    if pid > 0:
        sys.exit(0)

    # Redirect stdio
    sys.stdout.flush()
    sys.stderr.flush()
    devnull = open(os.devnull, "r")
    os.dup2(devnull.fileno(), sys.stdin.fileno())
    log_f = open(log_file, "a")
    os.dup2(log_f.fileno(), sys.stdout.fileno())
    os.dup2(log_f.fileno(), sys.stderr.fileno())

    return True


# ── Main ──────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="LingTu Navigation System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            profiles:
              stub      No hardware, framework testing
              sim       MuJoCo kinematic simulation
              dev       Semantic pipeline, no C++ nodes
              s100p     Full S100P robot (BPU + Kimi)
              explore   Exploration, no pre-built map

            special commands:
              stop      Stop running daemon
        """),
    )
    parser.add_argument("profile", nargs="?", default=None,
                        help="Profile name or 'stop'")
    parser.add_argument("--list", action="store_true", help="List profiles and exit")
    parser.add_argument("--daemon", "-d", action="store_true",
                        help="Run as background daemon (Unix)")
    parser.add_argument("--robot", default=None)
    parser.add_argument("--dog-host", default=None, dest="dog_host")
    parser.add_argument("--dog-port", type=int, default=None, dest="dog_port")
    parser.add_argument("--detector", default=None)
    parser.add_argument("--encoder", default=None)
    parser.add_argument("--llm", default=None)
    parser.add_argument("--planner", default=None)
    parser.add_argument("--tomogram", default=None)
    parser.add_argument("--gateway-port", type=int, default=None, dest="gateway_port")
    parser.add_argument("--no-semantic", action="store_true")
    parser.add_argument("--no-gateway", action="store_true")
    parser.add_argument("--no-native", action="store_true")
    parser.add_argument("--no-repl", action="store_true",
                        help="Foreground daemon (no interactive REPL)")
    parser.add_argument("--log-level", default="INFO", dest="log_level")
    args = parser.parse_args()

    # ── Special commands ──
    if args.list:
        _list_profiles()
        return

    if args.profile == "stop":
        _cmd_stop()
        return

    # ── Resolve profile ──
    profile_name = args.profile
    if profile_name is None:
        has_custom = any([
            args.robot, args.dog_host, args.detector,
            args.encoder, args.llm, args.planner,
        ])
        if has_custom:
            profile_name = "stub"
        elif not _IS_TTY:
            profile_name = "stub"
        else:
            profile_name = _select_interactive()

    if profile_name not in PROFILES:
        print(f"  {_red('Error')}: Unknown profile '{profile_name}'")
        print(f"  Available: {', '.join(PROFILES.keys())}")
        sys.exit(1)

    # Build config: profile defaults + CLI overrides
    cfg = dict(PROFILES[profile_name])
    overrides = {
        "robot": args.robot, "dog_host": args.dog_host,
        "dog_port": args.dog_port, "detector": args.detector,
        "encoder": args.encoder, "llm": args.llm,
        "planner": args.planner, "tomogram": args.tomogram,
        "gateway_port": args.gateway_port,
    }
    for k, v in overrides.items():
        if v is not None:
            cfg[k] = v
    if args.no_semantic:
        cfg["enable_semantic"] = False
    if args.no_gateway:
        cfg["enable_gateway"] = False
    if args.no_native:
        cfg["enable_native"] = False

    # Daemon implies no REPL
    if args.daemon:
        args.no_repl = True
    if not _IS_TTY:
        args.no_repl = True

    # ── Structured logging ──
    log_dir = _setup_logging(args.log_level, profile_name)

    # ── Daemon fork (before heavy imports) ──
    if args.daemon:
        log_file = str(Path(log_dir) / "lingtu.log")
        _daemonize(log_file)

    # ── Build system ──
    desc = cfg.pop("_desc", "custom")
    blueprint_cfg = dict(cfg)
    cfg["_desc"] = desc

    print(f"\n  Building system ({_green(profile_name)})...")

    from core.blueprints.full_stack import full_stack_blueprint

    try:
        system = full_stack_blueprint(**blueprint_cfg).build()
    except Exception as e:
        logger.error("Build failed: %s", e, exc_info=True)
        print(f"\n  {_red('Build failed')}: {e}")
        sys.exit(1)

    # ── Pre-start health check ──
    if not _health_check(system):
        print(f"  {_red('Health check failed')} — some modules did not build correctly")
        sys.exit(1)
    logger.info("Health check passed: %d modules OK", len(system.modules))

    # ── Start ──
    try:
        system.start()
    except Exception as e:
        logger.error("Start failed: %s", e, exc_info=True)
        print(f"\n  {_red('Start failed')}: {e}")
        sys.exit(1)

    # ── Save run state ──
    _save_run_state(profile_name, cfg, log_dir)

    _print_banner(profile_name, cfg, system, log_dir)

    # ── Signal handling ──
    shutdown = threading.Event()

    def _on_signal(signum, frame):
        shutdown.set()

    signal.signal(signal.SIGINT, _on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _on_signal)

    # ── Run ──
    if args.no_repl:
        shutdown.wait()
    else:
        # Reset SIGINT for REPL (let KeyboardInterrupt propagate)
        signal.signal(signal.SIGINT, signal.default_int_handler)
        repl = LingTuREPL(system, cfg)
        try:
            repl.cmdloop()
        except KeyboardInterrupt:
            print()

    # ── Shutdown ──
    print("  Stopping modules...")
    system.stop()
    _clear_run_state()
    print(f"  {_green('Done.')}\n")


if __name__ == "__main__":
    main()
