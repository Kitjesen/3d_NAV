"""Argument parsing and process lifecycle for the LingTu CLI."""

from __future__ import annotations

import argparse
import logging
import signal
import sys
import textwrap
import threading
from pathlib import Path

from . import term as T
from .logging_util import setup_logging
from .profiles_data import PROFILES, ROBOT_PRESETS
from .repl import LingTuREPL
from .run_state import (
    _lingtu_version,
    clear_run_state,
    save_run_state,
    update_run_state,
)
from .runtime_extra import daemonize, health_check, kill_residual_ports, preflight
from .term import IS_TTY
from .ui import (
    cmd_health_external,
    cmd_log_external,
    cmd_restart,
    cmd_show_config_external,
    cmd_status_external,
    cmd_stop,
    list_profiles,
    print_banner,
    select_interactive,
    wizard_interactive,
)

logger = logging.getLogger("lingtu")


_SPECIAL_COMMANDS = {"stop", "restart", "status", "show-config", "log", "health", "doctor", "rerun"}


def _runtime_endpoint(name: str):
    from core.blueprints.runtime_endpoint import runtime_endpoint

    return runtime_endpoint(name)


def _runtime_endpoint_names() -> tuple[str, ...]:
    from core.blueprints.runtime_endpoint import runtime_endpoint_names

    return runtime_endpoint_names()


def _apply_runtime_endpoint_config(profile_name: str, cfg: dict, endpoint_name: str) -> dict:
    from core.blueprints.runtime_endpoint import apply_runtime_endpoint_config

    return apply_runtime_endpoint_config(profile_name, cfg, endpoint_name)


def _is_runtime_endpoint_error(exc: Exception) -> bool:
    from core.blueprints.runtime_endpoint import RuntimeEndpointError

    return isinstance(exc, RuntimeEndpointError)


def _run_external_profile_launcher(
    profile_name: str,
    cfg: dict,
    args: argparse.Namespace,
    repo_root: Path,
) -> None:
    """Run a first-class profile that delegates to a simulator launcher."""

    import os
    import subprocess

    launcher = cfg.get("_external_launcher")
    if not launcher:
        raise RuntimeError("external profile missing _external_launcher")
    launcher_path = (repo_root / str(launcher)).resolve()
    if not launcher_path.exists():
        print(f"  {T.red('Error')}: launcher not found: {launcher_path}")
        sys.exit(1)
    if args.daemon:
        print(
            f"  {T.red('Error')}: --daemon is not supported for external simulation launchers; "
            "use the launcher's status/stop commands."
        )
        sys.exit(2)

    if args.extra:
        launcher_args = list(args.extra)
    elif getattr(args, "record", False):
        launcher_args = list(
            cfg.get("_external_record_args")
            or cfg.get("_external_default_args")
            or ("gate",)
        )
    else:
        launcher_args = list(cfg.get("_external_default_args") or ("gate",))
    env = os.environ.copy()
    env.setdefault("LINGTU_PROFILE", profile_name)
    runtime_endpoint_name = cfg.get("_runtime_endpoint")
    if runtime_endpoint_name:
        env.setdefault("LINGTU_ENDPOINT", str(runtime_endpoint_name))
    endpoint_data_source = cfg.get("_endpoint_data_source")
    if endpoint_data_source:
        env.setdefault("LINGTU_DATA_SOURCE", str(endpoint_data_source))
    runtime_contract = cfg.get("_runtime_contract")
    if runtime_contract:
        env.setdefault("LINGTU_RUNTIME_CONTRACT", str(runtime_contract))

    print(f"\n  Launching external simulation profile ({T.green(profile_name)})...", flush=True)
    print(f"  Launcher: {launcher}", flush=True)
    print(f"  Command:  bash {launcher} {' '.join(launcher_args)}", flush=True)
    try:
        proc = subprocess.run(
            ["bash", str(launcher), *launcher_args],
            cwd=str(repo_root),
            env=env,
            check=False,
        )
    except FileNotFoundError:
        print(f"  {T.red('Error')}: bash is required to run {launcher}")
        sys.exit(127)
    if proc.returncode:
        sys.exit(proc.returncode)


def _resolve_profile_name(explicit_profile: str | None, args: argparse.Namespace) -> str:
    profile_name = explicit_profile
    if profile_name is None:
        has_custom = any(
            [
                args.dog_host,
                args.detector,
                args.encoder,
                args.llm,
                args.planner,
            ]
        )
        if has_custom:
            profile_name = "stub"
        elif not IS_TTY:
            profile_name = "stub"
        else:
            profile_name = select_interactive()
    return profile_name


def _resolve_config(
    profile_name: str,
    args: argparse.Namespace,
    *,
    allow_wizard: bool = True,
) -> dict:
    if profile_name not in PROFILES:
        print(f"  {T.red('Error')}: Unknown profile '{profile_name}'")
        print(f"  Available: {', '.join(PROFILES.keys())}")
        sys.exit(1)

    endpoint_name = getattr(args, "endpoint", None)
    cfg = dict(PROFILES[profile_name])
    profile_default_robot = cfg.pop("_default_robot", "stub")
    if endpoint_name:
        try:
            endpoint = _runtime_endpoint(endpoint_name)
            endpoint.require_profile(profile_name)
        except Exception as exc:
            if not _is_runtime_endpoint_error(exc):
                raise
            print(f"  {T.red('Error')}: {exc}")
            sys.exit(2)
        robot_key = args.robot or endpoint.robot_preset
    else:
        robot_key = args.robot or profile_default_robot

    if robot_key in ROBOT_PRESETS:
        preset = ROBOT_PRESETS[robot_key]
        for k, v in preset.items():
            if k not in cfg:
                cfg[k] = v
            elif k == "robot":
                cfg[k] = v
    else:
        cfg["robot"] = robot_key

    if endpoint_name:
        cfg = _apply_runtime_endpoint_config(profile_name, cfg, endpoint_name)

    overrides = {
        "dog_host": args.dog_host,
        "dog_port": args.dog_port,
        "detector": args.detector,
        "encoder": args.encoder,
        "llm": args.llm,
        "planner": args.planner,
        "tomogram": args.tomogram,
        "plan_safety_policy": args.plan_safety_policy,
        "fallback_planner_name": args.fallback_planner_name,
        "gateway_port": args.gateway_port,
    }
    for k, v in overrides.items():
        if v is not None:
            cfg[k] = v
    if args.no_semantic:
        cfg["enable_semantic"] = False
    if args.no_gateway:
        cfg["enable_gateway"] = False
    if args.native:
        cfg["enable_native"] = True
    if args.no_native:
        cfg["enable_native"] = False
    if args.rerun:
        cfg["enable_rerun"] = True

    if allow_wizard and args.target is None and IS_TTY:
        wizard_interactive(profile_name, cfg)

    return cfg


def main() -> None:
    parser = argparse.ArgumentParser(
        description="LingTu Navigation System",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent(
            """\
            profiles:
              stub      No hardware, framework testing
              sim       MuJoCo kinematic simulation
              sim_mujoco_live MuJoCo raw MID-360 + Fast-LIO live simulation
              sim_gazebo Gazebo/GZ ROS-native simulation
              sim_industrial Gazebo industrial-yard delivery simulation
              sim_cmu_tare CMU Unity + external TARE simulation
              dev       Semantic pipeline, no C++ nodes
              nav       Navigation with pre-built map
              explore   Exploration, no pre-built map

            runtime endpoints:
              --endpoint mujoco_live  Run a product task against MuJoCo raw MID-360 + Fast-LIO
              --endpoint gazebo       Run a product task against Gazebo/GZ industrial adapter
              --endpoint cmu_unity    Run TARE task against CMU Unity external runtime

            product simulation examples:
              python lingtu.py explore --endpoint mujoco_live
              python lingtu.py tare_explore --endpoint mujoco_live
              python lingtu.py tare_explore --endpoint cmu_unity --record

            lifecycle commands:
              stop         Stop running daemon
              restart      Stop and relaunch with the same argv
              status       Show external run status (add --json for jq)
              show-config  Print resolved config (add --json for jq)
              log          Print the current run log (add -f to follow)
              health       Sensor and module health (add --json for jq)
              doctor       Run diagnostics
              rerun        Launch Rerun 3D viewer
        """
        ),
    )
    parser.add_argument("target", nargs="?", default=None, help="Profile name or command")
    parser.add_argument("extra", nargs="*", help=argparse.SUPPRESS)
    parser.add_argument("--list", action="store_true", help="List profiles and exit")
    parser.add_argument("--version", action="store_true", help="Print LingTu version and exit")
    parser.add_argument("--json", action="store_true",
                        help="Machine-readable JSON output (status / show-config)")
    parser.add_argument("--daemon", "-d", action="store_true", help="Run as background daemon (Unix)")
    parser.add_argument("--follow", "-f", action="store_true", help="Follow output for `lingtu log`")
    parser.add_argument("--lines", type=int, default=80, help="Number of lines for `lingtu log` (default: 80)")
    parser.add_argument("--force", action="store_true", help="Force action for commands like `lingtu stop`")
    parser.add_argument(
        "--endpoint",
        choices=_runtime_endpoint_names(),
        default=None,
        help="Runtime endpoint/connection layer for map/nav/explore/tare_explore",
    )
    parser.add_argument(
        "--record",
        action="store_true",
        help="Use the endpoint's visible recording/demo action when available",
    )
    parser.add_argument("--robot", default=None)
    parser.add_argument("--dog-host", default=None, dest="dog_host")
    parser.add_argument("--dog-port", type=int, default=None, dest="dog_port")
    parser.add_argument("--detector", default=None)
    parser.add_argument("--encoder", default=None)
    parser.add_argument("--llm", default=None)
    parser.add_argument("--planner", default=None)
    parser.add_argument("--tomogram", default=None)
    parser.add_argument(
        "--plan-safety-policy",
        default=None,
        choices=["off", "observe", "reject", "fallback_astar"],
        dest="plan_safety_policy",
        help="Global plan safety handling: observe by default, or fallback/reject unsafe plans",
    )
    parser.add_argument(
        "--fallback-planner",
        default=None,
        dest="fallback_planner_name",
        help="Planner backend used when --plan-safety-policy=fallback_astar",
    )
    parser.add_argument("--gateway-port", type=int, default=None, dest="gateway_port")
    parser.add_argument("--no-semantic", action="store_true")
    parser.add_argument("--no-gateway", action="store_true")
    parser.add_argument("--native", action="store_true", help="Force C++ autonomy stack (terrain+local_planner+pathFollower)")
    parser.add_argument("--no-native", action="store_true")
    parser.add_argument("--rerun", action="store_true", help="Enable Rerun 3D visualization on startup")
    parser.add_argument("--no-repl", action="store_true", help="Foreground daemon (no interactive REPL)")
    parser.add_argument("--log-level", default="INFO", dest="log_level")
    parser.add_argument("--log-format", default="text", choices=["text", "json"],
                        dest="log_format", help="Log file format: text (default) or json")
    args, trailing_extra = parser.parse_known_args()
    if trailing_extra:
        args.extra.extend(trailing_extra)

    if args.version:
        print(f"lingtu {_lingtu_version()}")
        return

    if args.list:
        list_profiles()
        return

    if args.target == "stop":
        cmd_stop(force=args.force)
        return

    if args.target == "restart":
        cmd_restart()
        return

    if args.target == "status":
        cmd_status_external(as_json=args.json)
        return

    if args.target == "health":
        cmd_health_external(as_json=args.json)
        return

    _repo = Path(__file__).resolve().parent.parent

    if args.target == "doctor":
        import subprocess as _sp

        _sp.run([sys.executable, str(_repo / "scripts" / "doctor.py")])
        return

    if args.target == "rerun":
        import subprocess as _sp

        _sp.run([sys.executable, str(_repo / "scripts" / "rerun_live.py")])
        return

    if args.target == "log":
        cmd_log_external(follow=args.follow, lines=args.lines)
        return

    if args.target == "show-config":
        profile_name = args.extra[0] if args.extra else None
        if len(args.extra) > 1:
            print("  Usage: lingtu show-config [profile] [overrides]")
            sys.exit(1)
        profile_name = _resolve_profile_name(profile_name, args)
        cfg = _resolve_config(profile_name, args, allow_wizard=False)
        cmd_show_config_external(profile_name, cfg, as_json=args.json)
        return

    profile_name = _resolve_profile_name(args.target, args)
    endpoint_external = False
    if args.endpoint:
        endpoint_external = bool(_runtime_endpoint(args.endpoint).external_launcher)
    external_profile = bool(
        PROFILES.get(profile_name, {}).get("_external_launcher") or endpoint_external
    )

    if args.target not in _SPECIAL_COMMANDS and args.extra and not external_profile:
        print(f"  {T.red('Error')}: Unexpected extra positional arguments: {' '.join(args.extra)}")
        sys.exit(1)

    cfg = _resolve_config(profile_name, args, allow_wizard=True)

    if cfg.get("_external_launcher"):
        _run_external_profile_launcher(profile_name, cfg, args, _repo)
        return

    if args.daemon:
        args.no_repl = True
    if not IS_TTY:
        args.no_repl = True

    log_dir = setup_logging(args.log_level, profile_name, args.log_format)

    if args.daemon:
        log_file = str(Path(log_dir) / "lingtu.log")
        daemonize(log_file)

    desc = cfg.pop("_desc", "custom")
    blueprint_cfg = dict(cfg)
    cfg["_desc"] = desc

    preflight(profile_name, cfg)

    print(f"\n  Building system ({T.green(profile_name)})...")

    from core.blueprints.full_stack import full_stack_blueprint

    try:
        system = full_stack_blueprint(**blueprint_cfg).build()
    except Exception as e:
        logger.error("Build failed: %s", e, exc_info=True)
        print(f"\n  {T.red('Build failed')}: {e}")
        sys.exit(1)

    if not health_check(system):
        print(f"  {T.red('Health check failed')} — some modules did not build correctly")
        sys.exit(1)
    logger.info("Health check passed: %d modules OK", len(system.modules))

    kill_residual_ports(blueprint_cfg)

    # In --no-repl mode, defer uvicorn startup so main thread can run it
    # (avoids GIL starvation from DDS callbacks in daemon threads).
    exit_code = 0
    if args.no_repl:
        try:
            gw = system.get_module("GatewayModule")
            gw._defer_server = True
        except (KeyError, AttributeError):
            pass

    try:
        system.start()
    except Exception as e:
        logger.error("Start failed: %s", e, exc_info=True)
        print(f"\n  {T.red('Start failed')}: {e}")
        sys.exit(1)

    # Inject SystemHandle so MCPServerModule can serve get_health / list_modules
    for _mod_name in ("MCPServerModule",):
        try:
            system.get_module(_mod_name).set_system_handle(system)
        except (KeyError, AttributeError):
            pass

    if cfg.get("enable_rerun"):
        rerun_mod = None
        try:
            rerun_mod = system.get_module("RerunBridgeModule")
        except KeyError:
            pass
        if rerun_mod:
            url = rerun_mod.start_rerun()
            logger.info("Rerun auto-started: %s", url)

    try:
        _mod_count = len(system.modules)
    except Exception:
        _mod_count = None
    try:
        _wire_count = len(getattr(system, "_connections", []) or [])
    except Exception:
        _wire_count = None

    save_run_state(
        profile_name,
        cfg,
        log_dir,
        log_format=args.log_format,
        argv=sys.argv[1:],
        cwd=str(Path.cwd()),
        daemon=args.daemon,
        status="running",
        module_count=_mod_count,
        wire_count=_wire_count,
    )

    print_banner(profile_name, cfg, system, log_dir)

    shutdown = threading.Event()

    def _on_signal(signum, frame):
        shutdown.set()

    signal.signal(signal.SIGINT, _on_signal)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, _on_signal)

    if args.no_repl:
        # Run uvicorn in the main thread so it gets reliable event-loop
        # scheduling.  Daemon threads (DDS/LiDAR callbacks) can starve a
        # threaded uvicorn of GIL time, making all HTTP endpoints hang.
        gw = None
        try:
            gw = system.get_module("GatewayModule")
        except (KeyError, AttributeError):
            pass
        if gw is not None and hasattr(gw, "_run_server"):
            if hasattr(gw, "_start_client_http_prewarm"):
                gw._start_client_http_prewarm(timeout_s=30.0)
            server_stopped_cleanly = bool(gw._run_server())  # blocks until shutdown
            if not server_stopped_cleanly and not shutdown.is_set():
                logger.error("Gateway server exited unexpectedly; returning failure for systemd")
                exit_code = 1
        else:
            shutdown.wait()
    else:
        signal.signal(signal.SIGINT, signal.default_int_handler)
        repl = LingTuREPL(system, cfg)
        try:
            repl.cmdloop()
        except KeyboardInterrupt:
            print()

    print("  Stopping modules...")
    try:
        update_run_state(status="stopping")
    except Exception:
        pass
    system.stop()
    try:
        from core.ros2_context import shutdown_shared_executor

        shutdown_shared_executor()
    except Exception:
        pass
    try:
        from core.service_manager import get_service_manager

        svc = get_service_manager()
        if svc._started:
            print("  Releasing services: %s" % ", ".join(svc._started))
            svc.stop_all_started()
    except Exception:
        pass
    clear_run_state()
    print(f"  {T.green('Done.')}\n")
    if exit_code:
        sys.exit(exit_code)
