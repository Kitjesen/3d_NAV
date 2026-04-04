"""Interactive cmd-loop REPL for LingTu."""

from __future__ import annotations

import cmd
import logging
import os
import sys
import time

from . import term as T
from .paths import project_root
from .term import IS_TTY, c


class LingTuREPL(cmd.Cmd):
    """Runtime control REPL with navigation commands."""

    prompt = c("1;34", "lingtu> ") if IS_TTY else "lingtu> "

    def __init__(self, system, profile_cfg):
        super().__init__()
        self._system = system
        self._cfg = profile_cfg

    def _get_module(self, name: str):
        try:
            return self._system.get_module(name)
        except KeyError:
            return None

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

        goal = PoseStamped(
            pose=Pose(
                position=Vector3(x=x, y=y, z=z),
                orientation=Quaternion(x=0, y=0, z=0, w=1),
            )
        )
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
                mod.stop_signal._deliver(2)
                count += 1
        print(f"  {T.red('STOP')} sent to {count} module(s)")

    def do_cancel(self, arg):
        """Cancel current navigation mission."""
        nav = self._get_module("NavigationModule")
        if not nav or not hasattr(nav, "cancel"):
            print("  NavigationModule not available")
            return
        nav.cancel._deliver("user")
        print("  Mission cancelled")

    def do_map(self, arg):
        """Map management: map list | map save <name> | map use <name> | map build <name> | map delete <name>"""
        parts = arg.split()
        if not parts:
            print("  Usage: map list | save <name> | use <name> | build <name> | delete <name>")
            return

        subcmd = parts[0]
        name = parts[1] if len(parts) > 1 else ""

        if subcmd == "list":
            self._map_list()
        elif subcmd == "save" and name:
            self._map_save(name)
        elif subcmd == "use" and name:
            self._map_use(name)
        elif subcmd == "build" and name:
            self._map_build_tomogram(name)
        elif subcmd == "delete" and name:
            self._map_delete(name)
        else:
            print("  Usage: map list | save <name> | use <name> | build <name> | delete <name>")

    def _map_list(self):
        import os

        from cli.profiles_data import _default_map_dir

        map_dir = _default_map_dir()
        if not os.path.isdir(map_dir):
            print(f"  No maps directory: {map_dir}")
            return
        active_link = os.path.join(map_dir, "active")
        active_name = ""
        if os.path.islink(active_link):
            active_name = os.path.basename(os.readlink(active_link))

        maps = sorted(
            d for d in os.listdir(map_dir) if os.path.isdir(os.path.join(map_dir, d)) and d != "active"
        )
        if not maps:
            print("  No maps found")
            return
        for m in maps:
            has_pcd = os.path.exists(os.path.join(map_dir, m, "map.pcd"))
            has_tomo = os.path.exists(os.path.join(map_dir, m, "tomogram.pickle"))
            marker = " *" if m == active_name else ""
            status = []
            if has_pcd:
                status.append("pcd")
            if has_tomo:
                status.append("tomogram")
            print(f"  {m}{marker}  [{', '.join(status) or 'empty'}]")

    def _map_save(self, name):
        import subprocess

        map_dir = os.path.join(
            os.environ.get("NAV_MAP_DIR") or os.path.expanduser("~/data/lingtu/maps"), name
        )
        os.makedirs(map_dir, exist_ok=True)
        pcd_path = os.path.join(map_dir, "map.pcd")
        print(f"  Saving map to {pcd_path}...")
        try:
            result = subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    "/pgo/save_maps",
                    "interface/srv/SaveMaps",
                    f"{{file_path: '{pcd_path}'}}",
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )
            if result.returncode == 0:
                print(f"  PCD saved: {pcd_path}")
                self._map_build_tomogram(name)
            else:
                print(f"  Save failed: {result.stderr[:200]}")
        except FileNotFoundError:
            print("  Error: ros2 not found. Run on robot with ROS2.")
        except subprocess.TimeoutExpired:
            print("  Error: save_maps service timed out")

    def _map_use(self, name):
        import os
        import shutil

        from cli.profiles_data import _default_map_dir

        map_dir = _default_map_dir()
        map_path = os.path.join(map_dir, name)
        if not os.path.isdir(map_path):
            print(f"  Map not found: {name}")
            return
        active_link = os.path.join(map_dir, "active")
        if os.path.islink(active_link):
            os.unlink(active_link)
        elif os.path.exists(active_link):
            shutil.rmtree(active_link)
        os.symlink(map_path, active_link)

        tomogram = os.path.join(map_path, "tomogram.pickle")
        nav = self._get_module("NavigationModule")
        if nav and os.path.exists(tomogram) and hasattr(nav, "_planner_backend"):
            if hasattr(nav._planner_backend, "_load_tomogram"):
                nav._planner_backend._load_tomogram(tomogram)
                print(f"  Active map: {name} (tomogram loaded)")
            elif hasattr(nav._planner_backend, "update_map"):
                import pickle

                with open(tomogram, "rb") as f:
                    data = pickle.load(f)
                if "grid" in data:
                    nav._planner_backend.update_map(data["grid"], data.get("resolution", 0.2))
                print(f"  Active map: {name} (costmap loaded)")
            else:
                print(f"  Active map: {name} (planner reload not supported)")
        else:
            if not os.path.exists(tomogram):
                print(f"  Active map: {name} (no tomogram — run: map build {name})")
            else:
                print(f"  Active map: {name}")

    def _map_build_tomogram(self, name):
        import os
        import sys

        from cli.profiles_data import _default_map_dir

        map_dir = _default_map_dir()
        pcd_path = os.path.join(map_dir, name, "map.pcd")
        tomogram_path = os.path.join(map_dir, name, "tomogram.pickle")
        if not os.path.exists(pcd_path):
            print(f"  No PCD file: {pcd_path}")
            return
        print(f"  Building tomogram from {pcd_path}...")
        try:
            tomo_dir = project_root() / "src" / "global_planning" / "PCT_planner" / "tomography" / "scripts"
            tds = str(tomo_dir)
            if tds not in sys.path:
                sys.path.insert(0, tds)
            from global_planning.PCT_planner.tomography.scripts.build_tomogram import (  # noqa: PLC0415
                build_tomogram_from_pcd,
            )

            build_tomogram_from_pcd(pcd_path, tomogram_path)
            print(f"  Tomogram built: {tomogram_path}")
        except Exception as e:
            print(f"  Build failed: {e}")

    def _map_delete(self, name):
        import os
        import shutil

        map_dir = os.path.join(
            os.environ.get("NAV_MAP_DIR") or os.path.expanduser("~/data/lingtu/maps"), name
        )
        if not os.path.isdir(map_dir):
            print(f"  Map not found: {name}")
            return
        shutil.rmtree(map_dir)
        print(f"  Deleted: {name}")

    def do_smap(self, arg):
        """Semantic map: smap status | rooms | save | load <dir> | query <text>"""
        parts = arg.split(None, 1)
        subcmd = parts[0] if parts else ""
        rest = parts[1] if len(parts) > 1 else ""

        if subcmd == "status":
            self._smap_status()
        elif subcmd == "rooms":
            self._smap_rooms()
        elif subcmd == "save":
            self._smap_save()
        elif subcmd == "load":
            self._smap_load(rest.strip())
        elif subcmd == "query" and rest:
            self._smap_query(rest.strip())
        else:
            print("  Usage: smap status | rooms | save | load <dir> | query <text>")

    def _smap_get(self):
        try:
            from memory.modules.semantic_mapper_module import SemanticMapperModule

            return self._get_module(SemanticMapperModule.__name__)
        except ImportError:
            return None

    def _smap_status(self):
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        s = mod.get_semantic_status()
        kg = s.get("kg", {})
        tsg = s.get("tsg", {})
        print(f"  Save dir : {s['save_dir']}")
        print(
            f"  KG       : {kg.get('room_types', 0)} room types, "
            f"{kg.get('unique_objects', 0)} object types, "
            f"{kg.get('total_observations', 0)} observations"
        )
        print(
            f"  TSG      : {tsg.get('rooms', 0)} rooms, "
            f"{tsg.get('frontiers', 0)} frontiers, "
            f"current={tsg.get('current_room', 'unknown')}"
        )
        print(f"  Updates  : {s['sg_updates']} scene-graph callbacks")

    def _smap_rooms(self):
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        summary = mod.get_room_summary()
        for line in summary.splitlines():
            print(f"  {line}")

    def _smap_save(self):
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        mod._save_now()
        print(f"  Saved to {mod._save_dir}")

    def _smap_load(self, directory):
        if not directory:
            print("  Usage: smap load <directory>")
            return
        import os

        directory = os.path.expanduser(directory)
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        ok = mod.load_from_dir(directory)
        if ok:
            print(f"  Loaded from {directory}")
        else:
            print(f"  Load failed (check path: {directory})")

    def _smap_query(self, text):
        mod = self._smap_get()
        if mod is None:
            print("  SemanticMapperModule not running")
            return
        result = mod.query_room_for_object(text)
        rooms = result.get("rooms", [])
        if not rooms:
            print(f"  No data for '{text}' (source: {result.get('source', '?')})")
            return
        print(f"  '{text}' most likely in:")
        for r in rooms:
            bar = "█" * int(r["probability"] * 20)
            print(f"    {r['room']:20s} {bar} {r['probability']:.2f}")

    def do_agent(self, arg):
        """Multi-turn agent: agent <instruction>"""
        if not arg.strip():
            print("  Usage: agent <instruction>")
            return
        mod = self._get_module("SemanticPlannerModule")
        if mod is None:
            print("  SemanticPlannerModule not running")
            return
        if not hasattr(mod, "agent_instruction"):
            print("  agent_instruction port not available")
            return
        mod.agent_instruction._deliver(arg.strip())
        print(f"  Agent loop started: '{arg.strip()}'")

    def do_vmem(self, arg):
        """Vector memory: vmem query <text> | vmem stats"""
        parts = arg.split(None, 1)
        subcmd = parts[0] if parts else ""
        rest = parts[1] if len(parts) > 1 else ""

        mod = self._get_module("VectorMemoryModule")
        if mod is None:
            print("  VectorMemoryModule not running")
            return

        if subcmd == "query" and rest:
            result = mod.query_location(rest.strip())
            if not result.get("found"):
                print(f"  No match for '{rest.strip()}'")
                return
            for r in result.get("results", []):
                bar = "█" * int(r["score"] * 20)
                print(
                    f"    ({r['x']:6.1f}, {r['y']:6.1f})  {bar} {r['score']:.2f}  {r.get('labels', '')[:40]}"
                )
        elif subcmd == "stats":
            s = mod.get_memory_stats()
            print(f"  Backend:  {s['backend']}")
            print(f"  Entries:  {s['entries']}")
            print(f"  Stored:   {s['store_count']} snapshots")
            print(f"  Dir:      {s['persist_dir']}")
        else:
            print("  Usage: vmem query <text> | vmem stats")

    def do_teleop(self, arg):
        """Teleop control: teleop status | teleop release"""
        mod = self._get_module("TeleopModule")
        if mod is None:
            print("  TeleopModule not running")
            return

        subcmd = arg.strip()
        if subcmd == "status":
            s = mod.get_teleop_status()
            active = "ACTIVE" if s["active"] else "idle"
            print(f"  Status:  {active}")
            print(f"  Clients: {s['clients']}")
            print(f"  Port:    ws://0.0.0.0:{s['port']}/teleop")
        elif subcmd == "release":
            print(f"  {mod.force_release()}")
        else:
            print("  Usage: teleop status | teleop release")

    def do_rerun(self, arg):
        """Rerun visualization: rerun on | rerun off | rerun status"""
        subcmd = arg.strip().lower()
        mod = self._get_module("RerunBridgeModule")

        if subcmd == "on":
            if mod is None:
                print("  RerunBridgeModule not in blueprint (start with --rerun)")
                return
            url = mod.start_rerun()
            print(f"  Rerun: {url}")
        elif subcmd == "off":
            if mod is None:
                print("  RerunBridgeModule not running")
                return
            result = mod.stop_rerun()
            print(f"  Rerun: {result}")
        elif subcmd == "status":
            if mod is None:
                print("  RerunBridgeModule not in blueprint")
                return
            s = mod.rerun_status()
            active = T.green("ACTIVE") if s["active"] else T.dim("inactive")
            print(f"  Status:  {active}")
            if s["url"]:
                print(f"  URL:     {s['url']}")
            print(f"  Counts:  odom={s['counts'].get('odom', 0)} cloud={s['counts'].get('cloud', 0)}")
        else:
            print("  Usage: rerun on | off | status")

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
            traffic = f"in:{n_in} out:{n_out}" if (n_in + n_out) > 0 else T.dim("idle")
            print(f"  [{layer}] {name:30s} {traffic}")
        nav = self._get_module("NavigationModule")
        if nav and hasattr(nav, "_state"):
            print(f"\n  Mission: {T.bold(nav._state)}")
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
                print(f"  {T.red('!')} {v}")
        print("\n  Startup order:")
        for i, name in enumerate(h["startup_order"], 1):
            print(f"    {i}. {name}")
        print()

    do_h = do_health

    def do_watch(self, arg):
        """Live status refresh: watch [interval_sec]. Ctrl+C to stop."""
        interval = float(arg) if arg else 2.0
        try:
            while True:
                os.system("cls" if os.name == "nt" else "clear")
                ts = time.strftime("%H:%M:%S")
                print(f"  {T.dim(f'[{ts}]  refresh every {interval:.0f}s  Ctrl+C to stop')}\n")
                self.do_status("")
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\n  Watch stopped")

    do_w = do_watch

    def do_live(self, arg):
        """Real-time dashboard with hotkeys."""
        interval = float(arg) if arg else 1.0
        start_time = time.time()
        import select

        if os.name != "nt":
            import termios
            import tty

            old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                os.system("cls" if os.name == "nt" else "clear")
                elapsed = int(time.time() - start_time)
                ts = time.strftime("%H:%M:%S")

                mode = self._cfg.get("slam_profile", "?")
                if mode in ("fastlio2", "pointlio"):
                    mode_str = T.green("MAP")
                elif mode == "localizer":
                    mode_str = T.green("NAV")
                else:
                    mode_str = T.green(mode.upper())

                print(f"  {T.bold('LingTu')} [{mode_str}]  {ts}  {elapsed}s  Ctrl+C to stop\n")

                nav = self._get_module("NavigationModule")
                x, y, z, yaw_deg = 0.0, 0.0, 0.0, 0.0
                if nav:
                    x, y, z = nav._robot_pos[0], nav._robot_pos[1], nav._robot_pos[2]
                    try:
                        import math

                        q = nav._latest_quat if hasattr(nav, "_latest_quat") else None
                        if q:
                            yaw_deg = math.degrees(
                                math.atan2(
                                    2 * (q[3] * q[2] + q[0] * q[1]),
                                    1 - 2 * (q[1] * q[1] + q[2] * q[2]),
                                )
                            )
                    except Exception:
                        pass

                print(f"  {T.bold('POSE')}:  x={x:7.2f}  y={y:7.2f}  z={z:5.2f}  yaw={yaw_deg:6.1f}°")
                print()

                slam = self._get_module("SlamBridgeModule") or self._get_module("SLAMModule")
                if slam:
                    odom = slam.odometry.msg_count if hasattr(slam, "odometry") else 0
                    cloud = slam.map_cloud.msg_count if hasattr(slam, "map_cloud") else 0
                    hz = odom / max(elapsed, 1)
                    print(f"  SLAM:  odom {T.bold(str(odom)):>6s} ({hz:.0f}Hz)  cloud {T.bold(str(cloud)):>6s}")

                if nav:
                    state = nav._state
                    wp = (
                        f"{nav._tracker.wp_index}/{nav._tracker.path_length}"
                        if hasattr(nav, "_tracker")
                        else "-"
                    )
                    color = T.green if state in ("IDLE", "SUCCESS") else T.yellow if state == "EXECUTING" else T.red
                    print(f"  NAV:   {color(state)}  waypoint {wp}")

                print()
                print(f"  {T.dim('[s]ave map  [g]o target  [x] stop  [n]av x,y  [q]uit')}")

                key = None
                if os.name != "nt":
                    deadline = time.time() + interval
                    while time.time() < deadline:
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            key = sys.stdin.read(1)
                            break
                else:
                    time.sleep(interval)

                if key == "q":
                    break
                elif key == "s":
                    print("\n  Enter map name: ", end="", flush=True)
                    if os.name != "nt":
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    name = input().strip()
                    if os.name != "nt":
                        tty.setcbreak(sys.stdin.fileno())
                    if name:
                        self._map_save(name)
                        time.sleep(2)
                elif key == "g":
                    print("\n  Go to: ", end="", flush=True)
                    if os.name != "nt":
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    target = input().strip()
                    if os.name != "nt":
                        tty.setcbreak(sys.stdin.fileno())
                    if target:
                        self.do_go(target)
                        time.sleep(1)
                elif key == "x":
                    self.do_stop("")
                elif key == "n":
                    print("\n  Navigate x y: ", end="", flush=True)
                    if os.name != "nt":
                        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    coords = input().strip()
                    if os.name != "nt":
                        tty.setcbreak(sys.stdin.fileno())
                    if coords:
                        self.do_navigate(coords)
                        time.sleep(1)

        except KeyboardInterrupt:
            pass
        finally:
            if os.name != "nt":
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            print("\n  Live stopped")

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
        print(f"\n  {T.bold(arg)} [{layer}]  ({type(mod).__name__})")
        if mod.ports_in:
            print("  Inputs:")
            for pname, port in mod.ports_in.items():
                policy = getattr(port, "_policy_name", "all")
                print(
                    f"    {pname}: {port.msg_type.__name__}  "
                    f"count={port.msg_count}  policy={policy}"
                )
        if mod.ports_out:
            print("  Outputs:")
            for pname, port in mod.ports_out.items():
                n_subs = len(getattr(port, "_callbacks", []))
                print(
                    f"    {pname}: {port.msg_type.__name__}  "
                    f"count={port.msg_count}  subs={n_subs}"
                )
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
            print(f"  {out_mod}.{out_port} {T.dim('->')} {in_mod}.{in_port}")
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
            print(f"  Unknown: {line}. Type {T.bold('help')} for commands.")

    def emptyline(self):
        pass
