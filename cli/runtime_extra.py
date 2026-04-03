"""Pre-flight checks, port cleanup, health, daemon."""

from __future__ import annotations

import logging
import os
import subprocess
import sys
from . import term as T


def preflight(profile_name: str, cfg: dict) -> None:
    slam = cfg.get("slam_profile", "none")
    tomogram = cfg.get("tomogram", "")

    if slam == "localizer" and tomogram:
        map_dir = os.path.dirname(tomogram)
        if not os.path.isdir(map_dir):
            print(f"  {T.yellow('WARN')}: No active map at {map_dir}")
            print("        Run 'lingtu map' first to build a map, then 'map use <name>'")
        elif tomogram and not os.path.isfile(tomogram):
            print(f"  {T.yellow('WARN')}: Tomogram not found: {tomogram}")
            print("        Run 'map build <name>' to generate it")

    if slam in ("fastlio2", "pointlio") and os.name != "nt":
        import shutil

        if not shutil.which("ros2"):
            print(f"  {T.yellow('WARN')}: ros2 not in PATH — SLAM won't start")
            print("        Source: source /opt/ros/humble/setup.bash")


def kill_residual_ports(cfg: dict) -> None:
    import platform

    if platform.system() != "Linux":
        return
    ports = [cfg.get("gateway_port", 5050), 8090, 5060]
    for port in ports:
        try:
            result = subprocess.run(
                ["fuser", "-k", "%d/tcp" % port],
                capture_output=True,
                timeout=3,
            )
            if result.returncode == 0:
                logging.getLogger(__name__).info("Killed residual process on port %d", port)
        except (FileNotFoundError, subprocess.TimeoutExpired):
            pass


def health_check(system) -> bool:
    ok = True
    for name, mod in system.modules.items():
        if mod is None:
            logging.getLogger("lingtu").error("Health check: module %s is None", name)
            ok = False
            continue
        if not hasattr(mod, "ports_in") or not hasattr(mod, "ports_out"):
            logging.getLogger("lingtu").error("Health check: %s missing ports", name)
            ok = False
    return ok


def daemonize(log_file: str) -> bool:
    """Unix: fork and detach. Windows: warn and stay foreground."""
    if os.name == "nt":
        print(f"  {T.yellow('Daemon mode not supported on Windows. Running in foreground.')}")
        return False

    pid = os.fork()
    if pid > 0:
        print(f"  Daemon started (PID {pid})")
        print(f"  Logs: {log_file}")
        print("  Stop: lingtu stop")
        sys.exit(0)

    os.setsid()

    pid = os.fork()
    if pid > 0:
        sys.exit(0)

    sys.stdout.flush()
    sys.stderr.flush()
    devnull = open(os.devnull, "r")
    os.dup2(devnull.fileno(), sys.stdin.fileno())
    log_f = open(log_file, "a")
    os.dup2(log_f.fileno(), sys.stdout.fileno())
    os.dup2(log_f.fileno(), sys.stderr.fileno())

    return True
