"""Pre-flight checks, port cleanup, health, daemon."""

from __future__ import annotations

import logging
import os
import subprocess
import sys
from . import term as T

# Built-in sample tomogram (relative to project root, ships in repo)
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_SAMPLE_TOMOGRAM = os.path.join(
    _REPO_ROOT, "src", "global_planning", "PCT_planner",
    "rsc", "tomogram", "building2_9.pickle",
)


def _scan_maps(map_dir: str) -> list:
    """Return sorted list of map names that have at least a tomogram or PCD."""
    if not os.path.isdir(map_dir):
        return []
    maps = []
    for d in sorted(os.listdir(map_dir)):
        if d == "active":
            continue
        full = os.path.join(map_dir, d)
        if not os.path.isdir(full):
            continue
        has_tomo = os.path.isfile(os.path.join(full, "tomogram.pickle"))
        has_pcd  = os.path.isfile(os.path.join(full, "map.pcd"))
        if has_tomo or has_pcd:
            maps.append((d, has_tomo, has_pcd))
    return maps


def _select_map_interactive(cfg: dict, map_dir: str) -> None:
    """If slam=localizer and no active tomogram, let the user pick a map
    or choose to build one / use the built-in sample.

    Mutates cfg['tomogram'] in place if the user selects a map.
    Returns immediately (no-op) when not in an interactive TTY.
    """
    if not sys.stdin.isatty():
        return

    # Current tomogram already valid → nothing to do
    current = cfg.get("tomogram", "")
    if current and os.path.isfile(current):
        return

    maps = _scan_maps(map_dir)
    active_link = os.path.join(map_dir, "active")
    active_name = (
        os.path.basename(os.readlink(active_link))
        if os.path.islink(active_link)
        else ""
    )

    print()
    print(f"  {T.yellow('No active map found.')} Select how to proceed:\n")

    options = []

    if maps:
        print(f"  {T.bold('Saved maps:')}")
        for name, has_tomo, has_pcd in maps:
            parts = []
            if has_tomo:
                parts.append("tomogram")
            if has_pcd:
                parts.append("pcd")
            marker = f"  {T.green('*')} (active)" if name == active_name else ""
            print(f"    [{len(options)+1}] {T.green(name):30s} [{', '.join(parts)}]{marker}")
            options.append(("use", name, os.path.join(map_dir, name, "tomogram.pickle")))
        print()

    # Built-in sample option
    sample_label = "Use built-in sample map (building2_9) — PCT test only, not your environment"
    print(f"  {T.bold('Other options:')}")
    idx_sample = len(options) + 1
    print(f"    [{idx_sample}] {sample_label}")
    options.append(("sample", "building2_9", _SAMPLE_TOMOGRAM))

    idx_build = len(options) + 1
    print(f"    [{idx_build}] Switch to 'map' profile — build a new map first")
    options.append(("build", "", ""))

    idx_skip = len(options) + 1
    print(f"    [{idx_skip}] Continue without a map (navigation will fail to plan paths)")
    options.append(("skip", "", ""))

    print()
    while True:
        try:
            raw = input(f"  Choice [1-{len(options)}]: ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            sys.exit(0)
        if not raw:
            continue
        if raw.isdigit():
            idx = int(raw) - 1
            if 0 <= idx < len(options):
                action, name, tomo_path = options[idx]
                break
        print(f"  {T.red('?')} Enter a number between 1 and {len(options)}")

    if action == "use":
        cfg["tomogram"] = tomo_path
        # Also update the active symlink so subsequent starts remember the choice
        try:
            map_path = os.path.join(map_dir, name)
            if os.path.islink(active_link):
                os.unlink(active_link)
            os.symlink(map_path, active_link)
            print(f"  Active map set to: {T.green(name)}")
        except OSError as e:
            print(f"  {T.yellow('WARN')}: Could not update active symlink: {e}")

    elif action == "sample":
        cfg["tomogram"] = _SAMPLE_TOMOGRAM
        print(f"  {T.yellow('Using sample map')} — results reflect demo environment, not yours.")
        print("  Run 'lingtu map' on your robot to build a real map.")

    elif action == "build":
        print()
        print(f"  Run:  {T.green('python lingtu.py map')}")
        print("  Then: drive the robot around to build the map.")
        print("  Then: map save <name>  →  map use <name>")
        print()
        sys.exit(0)

    # action == "skip": continue with current (possibly empty) tomogram
    print()


def preflight(profile_name: str, cfg: dict) -> None:
    slam = cfg.get("slam_profile", "none")

    if slam in ("fastlio2", "pointlio") and os.name != "nt":
        import shutil
        if not shutil.which("ros2"):
            print(f"  {T.yellow('WARN')}: ros2 not in PATH — SLAM won't start")
            print("        Source: source /opt/ros/humble/setup.bash")

    # For nav/explore profiles that use slam=localizer, offer interactive map selection.
    if slam == "localizer":
        map_dir = os.environ.get("NAV_MAP_DIR", os.path.expanduser("~/data/nova/maps"))
        _select_map_interactive(cfg, map_dir)

        # Post-selection warning if still no valid tomogram
        tomogram = cfg.get("tomogram", "")
        if not tomogram or not os.path.isfile(tomogram):
            print(f"  {T.yellow('WARN')}: Tomogram not found: {tomogram or '(none)'}")
            print("        Navigation will start but PCT planner will be unavailable.")


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
