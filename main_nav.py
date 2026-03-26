#!/usr/bin/env python3
"""LingTu Navigation System — CLI with interactive REPL.

Usage:
    python main_nav.py                    # interactive profile selector
    python main_nav.py stub               # stub mode (no hardware)
    python main_nav.py dev                # dev mode (semantic, no C++)
    python main_nav.py s100p              # real S100P robot
    python main_nav.py explore            # exploration (no pre-built map)
    python main_nav.py --list             # list all profiles
    python main_nav.py stub --no-repl     # daemon mode (no interactive)
    python main_nav.py --robot thunder --llm kimi   # custom flags

After startup, interactive REPL:
    lingtu> status          module overview
    lingtu> health          message counts + connections
    lingtu> module <name>   inspect module ports
    lingtu> connections     show all wiring
    lingtu> log debug       change log level at runtime
    lingtu> quit            graceful shutdown
"""

from __future__ import annotations

import argparse
import cmd
import logging
import signal
import sys
import os
import textwrap
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

logger = logging.getLogger("lingtu")

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


# ── Interactive REPL ──────────────────────────────────────────────────

class LingTuREPL(cmd.Cmd):
    """Runtime control REPL."""

    prompt = _c("1;34", "lingtu> ") if _IS_TTY else "lingtu> "

    def __init__(self, system, profile_cfg):
        super().__init__()
        self._system = system
        self._cfg = profile_cfg

    # ── commands ──

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

    def do_module(self, arg):
        """Inspect a module: module <name>"""
        if not arg:
            print("  Usage: module <name>")
            return
        try:
            mod = self._system.get_module(arg)
        except KeyError:
            # fuzzy match
            matches = [n for n in self._system.modules if arg.lower() in n.lower()]
            if matches:
                print(f"  Unknown module '{arg}'. Did you mean: {', '.join(matches)}")
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
        """Handle Ctrl+D."""
        print()
        return True

    def default(self, line):
        if line.strip():
            print(f"  Unknown: {line}. Type {_bold('help')} for commands.")

    def emptyline(self):
        pass


# ── Banner ────────────────────────────────────────────────────────────

def _print_banner(profile_name, cfg, system):
    n = len(system.modules)
    c = len(system.connections)
    desc = cfg.get("_desc", "custom")
    gw = cfg.get("gateway_port", 5050)

    print(f"\n{_bold('=' * 56)}")
    print(f"  {_bold('LingTu Navigation System')}")
    print(f"{_bold('=' * 56)}")
    print(f"  Profile:     {_green(profile_name)} — {desc}")
    print(f"  Modules:     {n}    Connections: {c}")
    print(f"  Robot:       {cfg.get('robot', '?'):10s}  Planner: {cfg.get('planner', '?')}")
    print(f"  Detector:    {cfg.get('detector', '?'):10s}  LLM:     {cfg.get('llm', '?')}")
    if cfg.get("enable_gateway"):
        print(f"  Gateway:     http://localhost:{gw}")
    if cfg.get("dog_host") and cfg["dog_host"] != "127.0.0.1":
        print(f"  Robot host:  {cfg['dog_host']}:{cfg.get('dog_port', 13145)}")
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
    print(f"\n  Override any flag: python main_nav.py s100p --llm mock\n")


# ── Main ──────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="LingTu Navigation System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            profiles:
              stub      No hardware, framework testing
              dev       Semantic pipeline, no C++ nodes
              s100p     Full S100P robot (BPU + Kimi)
              explore   Exploration, no pre-built map
        """),
    )
    parser.add_argument("profile", nargs="?", default=None,
                        help="Profile (stub/dev/s100p/explore) or omit for menu")
    parser.add_argument("--list", action="store_true", help="List profiles and exit")
    parser.add_argument("--robot", default=None)
    parser.add_argument("--dog-host", default=None, dest="dog_host")
    parser.add_argument("--dog-port", type=int, default=None, dest="dog_port")
    parser.add_argument("--detector", default=None)
    parser.add_argument("--encoder", default=None)
    parser.add_argument("--llm", default=None)
    parser.add_argument("--planner", default=None)
    parser.add_argument("--tomogram", default=None)
    parser.add_argument("--gateway-port", type=int, default=None, dest="gateway_port")
    parser.add_argument("--no-semantic", action="store_true",
                        help="Disable perception + semantic planning")
    parser.add_argument("--no-gateway", action="store_true",
                        help="Disable HTTP gateway + MCP server")
    parser.add_argument("--no-native", action="store_true",
                        help="Disable C++ NativeModule nodes")
    parser.add_argument("--no-repl", action="store_true",
                        help="Daemon mode — no interactive REPL")
    parser.add_argument("--log-level", default="INFO", dest="log_level")
    args = parser.parse_args()

    # --list
    if args.list:
        _list_profiles()
        return

    # Logging — stderr so REPL stdout stays clean
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        stream=sys.stderr,
    )

    # Resolve profile
    profile_name = args.profile
    if profile_name is None:
        # If user passed any explicit flag, treat as custom on top of stub
        has_custom = any([
            args.robot, args.dog_host, args.detector,
            args.encoder, args.llm, args.planner,
        ])
        if has_custom:
            profile_name = "stub"  # base, overridden below
        elif not _IS_TTY:
            profile_name = "stub"  # non-interactive default
        else:
            profile_name = _select_interactive()

    if profile_name not in PROFILES:
        print(f"  {_red('Error')}: Unknown profile '{profile_name}'")
        print(f"  Available: {', '.join(PROFILES.keys())}")
        sys.exit(1)

    # Build config: profile defaults + CLI overrides
    cfg = dict(PROFILES[profile_name])
    overrides = {
        "robot": args.robot,
        "dog_host": args.dog_host,
        "dog_port": args.dog_port,
        "detector": args.detector,
        "encoder": args.encoder,
        "llm": args.llm,
        "planner": args.planner,
        "tomogram": args.tomogram,
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

    # Auto-detect: no TTY → no REPL
    if not _IS_TTY:
        args.no_repl = True

    # Separate metadata from blueprint kwargs
    desc = cfg.pop("_desc", "custom")
    blueprint_cfg = dict(cfg)
    cfg["_desc"] = desc  # keep for banner

    # ── Build system ──
    print(f"\n  Building system ({_green(profile_name)})...")

    from core.blueprints.full_stack import full_stack_blueprint

    try:
        system = full_stack_blueprint(**blueprint_cfg).build()
    except Exception as e:
        logger.error("Build failed: %s", e, exc_info=True)
        print(f"\n  {_red('Build failed')}: {e}")
        sys.exit(1)

    # ── Start ──
    try:
        system.start()
    except Exception as e:
        logger.error("Start failed: %s", e, exc_info=True)
        print(f"\n  {_red('Start failed')}: {e}")
        sys.exit(1)

    _print_banner(profile_name, cfg, system)

    # ── Run ──
    if args.no_repl:
        # Daemon mode — wait for signal
        shutdown = threading.Event()

        def _on_signal(signum, frame):
            shutdown.set()

        signal.signal(signal.SIGINT, _on_signal)
        if hasattr(signal, "SIGTERM"):
            signal.signal(signal.SIGTERM, _on_signal)

        shutdown.wait()
    else:
        # Interactive REPL
        repl = LingTuREPL(system, cfg)
        try:
            repl.cmdloop()
        except KeyboardInterrupt:
            print()

    # ── Shutdown ──
    print("  Stopping modules...")
    system.stop()
    print(f"  {_green('Done.')}\n")


if __name__ == "__main__":
    main()
