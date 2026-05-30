#!/usr/bin/env python3
"""Runtime saved-map relocalization gate for the MuJoCo/Fast-LIO source path.

This gate proves the saved-map stage with live simulated sensors:

raw MuJoCo MID-360 + IMU -> Fast-LIO native cloud + odometry
  -> localizer loads same-source map.pcd
  -> canonical relocalize service succeeds
  -> canonical localization health locks and map->odom TF stays sane

It is simulation-only and never connects to robot hardware.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import shutil
import subprocess
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from core.runtime_interface import (
    FRAME_LINKS,
    TOPICS,
    adapter_source_for_target,
)

FASTLIO_REGISTERED_CLOUD_TOPIC = adapter_source_for_target(
    "fastlio2",
    TOPICS.registered_cloud,
)
FASTLIO_MAP_CLOUD_TOPIC = adapter_source_for_target("fastlio2", TOPICS.map_cloud)
FASTLIO_ODOMETRY_TOPIC = adapter_source_for_target("fastlio2", TOPICS.odometry)
LOCALIZER_MAP_CLOUD_INPUT = adapter_source_for_target(
    "localizer",
    TOPICS.saved_map_cloud,
)
LOCALIZER_QUALITY_INPUT = adapter_source_for_target(
    "localizer",
    TOPICS.localization_quality,
)
LOCALIZER_RELOCALIZE_INPUT = adapter_source_for_target(
    "localizer",
    TOPICS.relocalize_service,
)
LOCALIZER_RELOCALIZE_CHECK_INPUT = adapter_source_for_target(
    "localizer",
    TOPICS.relocalize_check_service,
)
LOCALIZER_GLOBAL_RELOCALIZE_INPUT = adapter_source_for_target(
    "localizer",
    TOPICS.global_relocalize_service,
)
MAP_TO_ODOM_LINK = FRAME_LINKS["map_to_odom"]
TF_TOPIC = "/tf"


def _resolve_latest_map() -> Path | None:
    for pattern in (
        "artifacts/server_sim_closure/cli_tare_endpoint_mujoco_live*/**/same_source_map/map.pcd",
        "artifacts/server_sim_closure/mujoco_tare_exploration*/**/same_source_map/map.pcd",
        "artifacts/server_sim_closure/cli_explore_endpoint_mujoco_live*/**/same_source_map/map.pcd",
        "artifacts/server_sim_closure/cli_sim_mujoco_live*/**/same_source_map/map.pcd",
        "artifacts/server_sim_closure/mujoco_fastlio2_live*/**/same_source_map/map.pcd",
        "artifacts/server_sim_closure/mujoco_fastlio2_live*/same_source_map/map.pcd",
        "artifacts/server_sim_closure/**/same_source_map/map.pcd",
    ):
        candidates = [path for path in ROOT.glob(pattern) if path.is_file()]
        if candidates:
            return max(candidates, key=lambda path: path.stat().st_mtime)
    return None


def _resolve_map_path(value: str) -> Path | None:
    if value.strip().lower() in {"", "latest"}:
        return _resolve_latest_map()
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = (ROOT / path).resolve()
    return path


def _map_metadata_path(map_pcd: Path) -> Path:
    return map_pcd.parent / "metadata.json"


def _load_map_metadata(map_pcd: Path | None) -> dict[str, Any]:
    if map_pcd is None:
        return {}
    metadata_path = _map_metadata_path(map_pcd)
    if not metadata_path.is_file():
        return {}
    try:
        payload = json.loads(metadata_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _resolve_live_world_arg(world: str, map_metadata: dict[str, Any]) -> str:
    value = str(world or "map_metadata").strip()
    if value.lower() not in {"", "map_metadata", "same_source"}:
        return value
    metadata_world = str(map_metadata.get("world") or "").strip()
    return metadata_world or "industrial_park"


def _resolve_scan_time_profile_arg(
    scan_time_profile: str,
    map_metadata: dict[str, Any],
) -> str:
    value = str(scan_time_profile or "map_metadata").strip().lower()
    valid = {"instantaneous", "synthetic_rolling", "physical_rolling"}
    if value in valid:
        return value
    if value not in {"", "map_metadata", "same_source"}:
        raise ValueError(f"unsupported scan_time_profile: {scan_time_profile}")
    metadata_value = str(map_metadata.get("scan_time_profile") or "").strip().lower()
    if metadata_value in valid:
        return metadata_value
    return "physical_rolling"


def _default_localizer_config_path() -> Path:
    config_path = ROOT / "src/slam/localizer/config/localizer.yaml"
    if not config_path.is_file():
        config_path = ROOT / "install/share/localizer/config/localizer.yaml"
    return config_path


def _resolve_localizer_config_path(value: str) -> Path:
    raw = str(value or "").strip()
    if not raw:
        return _default_localizer_config_path()
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = (ROOT / path).resolve()
    return path


def _parse_localizer_thresholds(config_path: Path) -> dict[str, float | None]:
    thresholds: dict[str, float | None] = {
        "rough_score_thresh": None,
        "refine_score_thresh": None,
    }
    if not config_path.is_file():
        return thresholds
    for line in config_path.read_text(encoding="utf-8", errors="replace").splitlines():
        body = line.split("#", 1)[0]
        if ":" not in body:
            continue
        key, value = body.split(":", 1)
        key = key.strip()
        if key not in thresholds:
            continue
        try:
            thresholds[key] = float(value.strip())
        except ValueError:
            thresholds[key] = None
    return thresholds


def _write_localizer_runtime_config(
    base_config_path: Path,
    run_dir: Path,
    *,
    rough_score_thresh: float | None,
    refine_score_thresh: float | None,
) -> Path:
    if rough_score_thresh is None and refine_score_thresh is None:
        return base_config_path
    text = base_config_path.read_text(encoding="utf-8", errors="replace")
    replacements = {
        "rough_score_thresh": rough_score_thresh,
        "refine_score_thresh": refine_score_thresh,
    }
    for key, value in replacements.items():
        if value is None:
            continue
        pattern = re.compile(rf"^(\s*{re.escape(key)}\s*:\s*)([^#\r\n]*)(.*)$", re.MULTILINE)
        replacement = rf"\g<1>{float(value):.6g}\g<3>"
        text, count = pattern.subn(replacement, text, count=1)
        if count == 0:
            text = text.rstrip() + f"\n{key}: {float(value):.6g}\n"
    runtime_config = run_dir / "localizer_runtime.yaml"
    runtime_config.write_text(text, encoding="utf-8")
    return runtime_config


def _terminate_process(proc: subprocess.Popen[Any] | None, *, timeout_s: float = 6.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except Exception:
        proc.terminate()
    try:
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        proc.kill()
    try:
        proc.wait(timeout=timeout_s)
    except Exception:
        pass


def _tail(path: Path, *, lines: int = 80) -> str:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        return ""
    return "\n".join(text.splitlines()[-lines:])


def _load_ros_modules():
    try:
        import rclpy  # type: ignore
        from interface.srv import Relocalize  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore
        from sensor_msgs.msg import PointCloud2  # type: ignore
        from std_msgs.msg import String  # type: ignore
        from std_srvs.srv import Trigger  # type: ignore
        from tf2_msgs.msg import TFMessage  # type: ignore
    except Exception as exc:  # pragma: no cover - requires ROS 2 env
        raise RuntimeError(
            "ROS 2 Python dependencies are unavailable. Source "
            "/opt/ros/humble/setup.bash and install/setup.bash first."
        ) from exc
    return rclpy, Relocalize, Trigger, Odometry, PointCloud2, String, TFMessage


class _RuntimeSampler:
    def __init__(self) -> None:
        self.counts: defaultdict[str, int] = defaultdict(int)
        self.point_counts: dict[str, int] = {}
        self.frames: dict[str, str] = {}
        self.health_states: list[str] = []
        self.health_latest = ""
        self.map_to_odom_latest: dict[str, float] | None = None
        self.first_odom: tuple[float, float, float] | None = None
        self.last_odom: tuple[float, float, float] | None = None

    def on_cloud(self, name: str, msg: Any) -> None:
        self.counts[name] += 1
        self.point_counts[name] = int(getattr(msg, "width", 0) * getattr(msg, "height", 0))
        self.frames[name] = str(getattr(getattr(msg, "header", None), "frame_id", "") or "")

    def on_odom(self, name: str, msg: Any) -> None:
        self.counts[name] += 1
        pose = msg.pose.pose.position
        xyz = (float(pose.x), float(pose.y), float(pose.z))
        if self.first_odom is None:
            self.first_odom = xyz
        self.last_odom = xyz
        self.frames[name] = str(msg.header.frame_id or "")
        self.frames[f"{name}_child"] = str(msg.child_frame_id or "")

    def on_health(self, msg: Any) -> None:
        self.counts["localization_health"] += 1
        text = str(getattr(msg, "data", "") or "")
        state = text.split("|", 1)[0].strip().upper() or "UNKNOWN"
        self.health_latest = text
        self.health_states.append(state)

    def on_tf(self, msg: Any) -> None:
        for transform in getattr(msg, "transforms", []) or []:
            if (
                transform.header.frame_id != MAP_TO_ODOM_LINK.parent
                or transform.child_frame_id != MAP_TO_ODOM_LINK.child
            ):
                continue
            self.counts["map_to_odom_tf"] += 1
            t = transform.transform.translation
            q = transform.transform.rotation
            self.map_to_odom_latest = {
                "x": float(t.x),
                "y": float(t.y),
                "z": float(t.z),
                "qx": float(q.x),
                "qy": float(q.y),
                "qz": float(q.z),
                "qw": float(q.w),
            }

    @property
    def odom_delta_m(self) -> float | None:
        if self.first_odom is None or self.last_odom is None:
            return None
        return float(math.dist(self.first_odom, self.last_odom))


def _wait_for_topic_samples(
    *,
    rclpy: Any,
    node: Any,
    sampler: _RuntimeSampler,
    keys: tuple[str, ...],
    deadline: float,
) -> bool:
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(sampler.counts.get(key, 0) > 0 for key in keys):
            return True
    return False


def _call_relocalize(
    *,
    rclpy: Any,
    node: Any,
    Relocalize: Any,
    pcd_path: Path,
    timeout_s: float,
) -> tuple[bool, dict[str, Any]]:
    client = node.create_client(Relocalize, TOPICS.relocalize_service)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if client.wait_for_service(timeout_sec=0.2):
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    else:
        return False, {"available": False, "success": False, "message": "service unavailable"}

    request = Relocalize.Request()
    request.pcd_path = str(pcd_path)
    request.x = 0.0
    request.y = 0.0
    request.z = 0.0
    request.yaw = 0.0
    request.pitch = 0.0
    request.roll = 0.0
    future = client.call_async(request)
    deadline = time.time() + timeout_s
    while time.time() < deadline and not future.done():
        rclpy.spin_once(node, timeout_sec=0.1)
    if not future.done():
        return False, {"available": True, "success": False, "message": "service timeout"}
    response = future.result()
    return bool(response and response.success), {
        "available": True,
        "success": bool(response and response.success),
        "message": str(getattr(response, "message", "") if response else ""),
    }


def _call_global_relocalize(
    *,
    rclpy: Any,
    node: Any,
    Trigger: Any,
    timeout_s: float,
) -> tuple[bool, dict[str, Any]]:
    client = node.create_client(Trigger, TOPICS.global_relocalize_service)
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if client.wait_for_service(timeout_sec=0.2):
            break
        rclpy.spin_once(node, timeout_sec=0.05)
    else:
        return False, {"available": False, "success": False, "message": "service unavailable"}

    last_response: dict[str, Any] = {
        "available": True,
        "success": False,
        "message": "not called",
    }
    while time.time() < deadline:
        future = client.call_async(Trigger.Request())
        call_deadline = min(deadline, time.time() + 5.0)
        while time.time() < call_deadline and not future.done():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not future.done():
            last_response = {
                "available": True,
                "success": False,
                "message": "service timeout",
            }
            continue
        response = future.result()
        message = str(getattr(response, "message", "") if response else "")
        success = bool(response and response.success)
        last_response = {
            "available": True,
            "success": success,
            "message": message,
        }
        if success or "already running" in message.lower():
            return True, last_response
        if "no scan received yet" in message.lower():
            time.sleep(0.5)
            continue
        return False, last_response
    return False, last_response


def _isolated_map_without_last_pose(map_pcd: Path, run_dir: Path) -> Path:
    map_dir = run_dir / "kidnap_map"
    map_dir.mkdir(parents=True, exist_ok=True)
    isolated = map_dir / "map.pcd"
    if map_pcd.resolve() != isolated.resolve():
        shutil.copy2(map_pcd, isolated)
    last_pose = map_dir / "last_pose.txt"
    if last_pose.exists():
        last_pose.unlink()
    return isolated


def _start_live_feed(args: argparse.Namespace, run_dir: Path) -> subprocess.Popen[str]:
    python = shutil.which("python3") or sys.executable
    drive_source = "nav_cmd_vel" if args.live_drive_source == "frontier" else "fixed"
    cmd = [
        python,
        str(ROOT / "sim/scripts/mujoco_fastlio2_live_gate.py"),
        "--world",
        args.world,
        "--duration",
        str(args.duration),
        "--duration-clock",
        args.duration_clock,
        "--drive-source",
        drive_source,
        "--drive-vx",
        str(args.drive_vx),
        "--drive-vy",
        str(args.drive_vy),
        "--drive-wz",
        str(args.drive_wz),
        "--nav-data-source",
        "fastlio2",
        "--work-dir",
        str(run_dir / "live_feed/work"),
        "--json-out",
        str(run_dir / "live_feed/report.json"),
        "--mid360-samples-per-frame",
        str(args.mid360_samples_per_frame),
        "--scan-time-profile",
        args.scan_time_profile,
        "--imu-acc-mode",
        args.imu_acc_mode,
        "--max-fastlio-z-drift-m",
        str(args.max_fastlio_z_drift_m),
        "--no-save-map-artifacts",
    ]
    if args.live_drive_source == "frontier":
        cmd.extend(
            [
                "--run-lingtu-frontier",
                "--frontier-min-goals",
                str(args.frontier_min_goals),
                "--frontier-goal-timeout",
                str(args.frontier_goal_timeout),
            ]
        )
    (run_dir / "live_feed").mkdir(parents=True, exist_ok=True)
    (run_dir / "live_feed/command.txt").write_text(
        " ".join(shlex_quote(item) for item in cmd) + "\n",
        encoding="utf-8",
    )
    log = (run_dir / "live_feed/gate.log").open("w", encoding="utf-8")
    return subprocess.Popen(
        cmd,
        cwd=str(ROOT),
        env=os.environ.copy(),
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def shlex_quote(value: str) -> str:
    import shlex

    return shlex.quote(str(value))


def _start_localizer(
    map_pcd: Path,
    run_dir: Path,
    *,
    config_path: Path,
    initial_x: float = 0.0,
    initial_y: float = 0.0,
    initial_z: float = 0.0,
    initial_yaw: float = 0.0,
    disable_auto_global_relocalize: bool = False,
    bbs3d_num_threads: int = 1,
    bbs3d_timeout_ms: int = 30000,
) -> subprocess.Popen[str]:
    ros2 = shutil.which("ros2")
    if not ros2:
        raise RuntimeError("ros2 CLI is unavailable")
    cmd = [
        ros2,
        "run",
        "localizer",
        "localizer_node",
        "--ros-args",
        "-p",
        f"config_path:={config_path}",
        "-p",
        f"static_map_path:={map_pcd}",
        "-p",
        f"initial_x:={initial_x}",
        "-p",
        f"initial_y:={initial_y}",
        "-p",
        f"initial_z:={initial_z}",
        "-p",
        f"initial_yaw:={initial_yaw}",
        "-r",
        f"{LOCALIZER_MAP_CLOUD_INPUT}:={TOPICS.saved_map_cloud}",
        "-r",
        f"{LOCALIZER_QUALITY_INPUT}:={TOPICS.localization_quality}",
        "-r",
        f"{LOCALIZER_RELOCALIZE_INPUT}:={TOPICS.relocalize_service}",
        "-r",
        f"{LOCALIZER_RELOCALIZE_CHECK_INPUT}:={TOPICS.relocalize_check_service}",
        "-r",
        f"{LOCALIZER_GLOBAL_RELOCALIZE_INPUT}:={TOPICS.global_relocalize_service}",
    ]
    if disable_auto_global_relocalize:
        cmd.extend(
            [
                "-p",
                "auto_global_relocalize_on_boot:=false",
                "-p",
                "auto_global_relocalize_on_lost:=false",
                "-p",
                f"bbs3d_num_threads:={bbs3d_num_threads}",
                "-p",
                f"bbs3d_timeout_ms:={bbs3d_timeout_ms}",
            ]
        )
    (run_dir / "localizer_command.txt").write_text(
        " ".join(shlex_quote(item) for item in cmd) + "\n",
        encoding="utf-8",
    )
    log = (run_dir / "localizer.log").open("w", encoding="utf-8")
    return subprocess.Popen(
        cmd,
        cwd=str(ROOT),
        env=os.environ.copy(),
        stdout=log,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def _live_feed_timeout_s(args: argparse.Namespace) -> float:
    if float(args.live_process_timeout_s) > 0.0:
        return float(args.live_process_timeout_s)
    if str(args.duration_clock) == "sim":
        return max(120.0, float(args.duration) * 12.0 + 60.0)
    return max(5.0, float(args.duration) + 30.0)


def _wait_for_process_with_spin(
    proc: subprocess.Popen[str],
    *,
    timeout_s: float,
    rclpy: Any,
    node: Any,
) -> bool:
    deadline = time.time() + max(0.0, timeout_s)
    while time.time() < deadline:
        if proc.poll() is not None:
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return proc.poll() is not None


def run_gate(args: argparse.Namespace) -> dict[str, Any]:
    started = time.time()
    run_dir = args.run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    map_pcd = _resolve_map_path(args.map_pcd)
    blockers: list[str] = []
    if map_pcd is None:
        blockers.append("same-source map.pcd not found")
    elif not map_pcd.is_file():
        blockers.append(f"map_pcd not found: {map_pcd}")
    map_metadata = _load_map_metadata(map_pcd)
    live_world = _resolve_live_world_arg(args.world, map_metadata)
    scan_time_profile = _resolve_scan_time_profile_arg(
        args.scan_time_profile,
        map_metadata,
    )
    localizer_config_source = _resolve_localizer_config_path(args.localizer_config)
    if not localizer_config_source.is_file():
        blockers.append(f"localizer config not found: {localizer_config_source}")
    localizer_config_path = localizer_config_source
    if localizer_config_source.is_file():
        localizer_config_path = _write_localizer_runtime_config(
            localizer_config_source,
            run_dir,
            rough_score_thresh=args.localizer_rough_score_thresh,
            refine_score_thresh=args.localizer_refine_score_thresh,
        )
    localizer_thresholds = _parse_localizer_thresholds(localizer_config_path)
    args.world = live_world
    args.scan_time_profile = scan_time_profile

    report_base: dict[str, Any] = {
        "schema_version": "lingtu.saved_map_relocalize_runtime.v1",
        "validation_level": "runtime_relocalization",
        "runtime_stage": "saved_map_relocalization",
        "map_dependency": "saved_map_required",
        "requires_saved_map": True,
        "requires_live_slam": True,
        "requires_tomogram": False,
        "runtime_relocalization_executed": False,
        "runtime_relocalization_validated": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "map_pcd": str(map_pcd) if map_pcd else "",
        "map_metadata": str(_map_metadata_path(map_pcd)) if map_pcd else "",
        "map_metadata_world": str(map_metadata.get("world") or ""),
        "run_dir": str(run_dir),
        "global_relocalization_requested": bool(args.check_global_relocalize),
        "world": live_world,
        "scan_time_profile": scan_time_profile,
        "localizer_config": {
            "source_path": str(localizer_config_source),
            "runtime_path": str(localizer_config_path),
            "rough_score_thresh": localizer_thresholds.get("rough_score_thresh"),
            "refine_score_thresh": localizer_thresholds.get("refine_score_thresh"),
            "overrides": {
                "rough_score_thresh": args.localizer_rough_score_thresh,
                "refine_score_thresh": args.localizer_refine_score_thresh,
            },
        },
    }
    if blockers:
        return {**report_base, "ok": False, "blockers": blockers}

    if args.check_global_relocalize and map_pcd is not None and map_pcd.is_file():
        map_pcd = _isolated_map_without_last_pose(map_pcd, run_dir)

    rclpy, Relocalize, Trigger, Odometry, PointCloud2, String, TFMessage = _load_ros_modules()
    sampler = _RuntimeSampler()
    live_proc: subprocess.Popen[str] | None = None
    localizer_proc: subprocess.Popen[str] | None = None
    service: dict[str, Any] = {"available": False, "success": False, "message": "not called"}
    service_ok = False
    live_report: dict[str, Any] = {}
    live_report_path = run_dir / "live_feed/report.json"

    rclpy.init(args=None)
    node = rclpy.create_node("saved_map_relocalize_runtime_gate")
    qos = 10
    node.create_subscription(
        PointCloud2,
        FASTLIO_REGISTERED_CLOUD_TOPIC,
        lambda msg: sampler.on_cloud("cloud_registered", msg),
        qos,
    )
    node.create_subscription(
        PointCloud2,
        FASTLIO_MAP_CLOUD_TOPIC,
        lambda msg: sampler.on_cloud("cloud_map", msg),
        qos,
    )
    node.create_subscription(
        PointCloud2,
        TOPICS.saved_map_cloud,
        lambda msg: sampler.on_cloud("saved_map_cloud", msg),
        qos,
    )
    node.create_subscription(
        Odometry,
        FASTLIO_ODOMETRY_TOPIC,
        lambda msg: sampler.on_odom("raw_odometry", msg),
        qos,
    )
    node.create_subscription(String, TOPICS.localization_health, sampler.on_health, qos)
    node.create_subscription(TFMessage, TF_TOPIC, sampler.on_tf, qos)

    try:
        live_proc = _start_live_feed(args, run_dir)
        live_ready = _wait_for_topic_samples(
            rclpy=rclpy,
            node=node,
            sampler=sampler,
            keys=("cloud_registered", "cloud_map", "raw_odometry"),
            deadline=time.time() + args.topic_timeout_s,
        )
        if not live_ready:
            blockers.append("live Fast-LIO topics did not become ready")

        localizer_proc = _start_localizer(
            map_pcd,
            run_dir,
            config_path=localizer_config_path,
            initial_x=args.kidnap_initial_x if args.check_global_relocalize else 0.0,
            initial_y=args.kidnap_initial_y if args.check_global_relocalize else 0.0,
            initial_z=args.kidnap_initial_z if args.check_global_relocalize else 0.0,
            initial_yaw=args.kidnap_initial_yaw if args.check_global_relocalize else 0.0,
            disable_auto_global_relocalize=bool(args.check_global_relocalize),
            bbs3d_num_threads=args.bbs3d_num_threads,
            bbs3d_timeout_ms=args.bbs3d_timeout_ms,
        )
        if args.check_global_relocalize:
            service_ok, service = _call_global_relocalize(
                rclpy=rclpy,
                node=node,
                Trigger=Trigger,
                timeout_s=args.service_timeout_s,
            )
            if not service_ok:
                blockers.append(
                    f"{TOPICS.global_relocalize_service} failed: {service.get('message')}"
                )
        else:
            service_ok, service = _call_relocalize(
                rclpy=rclpy,
                node=node,
                Relocalize=Relocalize,
                pcd_path=map_pcd,
                timeout_s=args.service_timeout_s,
            )
            if not service_ok:
                blockers.append(
                    f"{TOPICS.relocalize_service} failed: {service.get('message')}"
                )

        monitor_deadline = time.time() + args.monitor_after_service_s
        while time.time() < monitor_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        live_done = _wait_for_process_with_spin(
            live_proc,
            timeout_s=_live_feed_timeout_s(args),
            rclpy=rclpy,
            node=node,
        )
        if not live_done:
            blockers.append("live feed process timed out")
            _terminate_process(live_proc)
        if live_report_path.is_file():
            live_report = json.loads(live_report_path.read_text(encoding="utf-8"))
        else:
            blockers.append("live feed report missing")
    finally:
        _terminate_process(localizer_proc)
        _terminate_process(live_proc)
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()

    tracking_states = {"LOCKED", "RECOVERED"}
    tracking_health_samples = sum(1 for state in sampler.health_states if state in tracking_states)
    lost_health_samples = sum(1 for state in sampler.health_states if state == "LOST")
    latest_health_state = sampler.health_states[-1] if sampler.health_states else ""
    map_to_odom = sampler.map_to_odom_latest or {}
    map_to_odom_xy = math.hypot(
        float(map_to_odom.get("x", 0.0)),
        float(map_to_odom.get("y", 0.0)),
    ) if map_to_odom else None
    map_to_odom_z_abs = abs(float(map_to_odom.get("z", 0.0))) if map_to_odom else None

    localizer_tail = _tail(run_dir / "localizer.log")
    bbs3d_ok = "BBS3D: ok" in localizer_tail or "BBS3D boot OK" in localizer_tail
    bbs3d_disabled = (
        "BBS3D disabled at build time" in localizer_tail
        or "bbs3d map not loaded" in str(service.get("message", "")).lower()
    )

    if live_report.get("ok") is not True:
        blockers.append("live feed report is not ok")
    if (live_report.get("fastlio2_z_consistency") or {}).get("ok") is not True:
        blockers.append("live feed Fast-LIO Z consistency is not ok")
    if sampler.counts.get("saved_map_cloud", 0) <= 0:
        blockers.append(f"{TOPICS.saved_map_cloud} missing")
    if sampler.point_counts.get("saved_map_cloud", 0) < int(args.min_saved_map_points):
        blockers.append(f"{TOPICS.saved_map_cloud} point count below threshold")
    if tracking_health_samples < int(args.min_tracking_health_samples):
        blockers.append("localizer tracking health samples below threshold")
    if latest_health_state not in tracking_states:
        blockers.append("latest localization health is not LOCKED/RECOVERED")
    if sampler.counts.get("map_to_odom_tf", 0) <= 0:
        blockers.append("map->odom TF missing")
    if map_to_odom_xy is not None and map_to_odom_xy > float(args.max_map_odom_xy_m):
        blockers.append(
            f"map->odom XY correction {map_to_odom_xy:.3f}m exceeds "
            f"{float(args.max_map_odom_xy_m):.3f}m"
        )
    if map_to_odom_z_abs is not None and map_to_odom_z_abs > float(args.max_map_odom_z_abs_m):
        blockers.append(
            f"map->odom Z correction {map_to_odom_z_abs:.3f}m exceeds "
            f"{float(args.max_map_odom_z_abs_m):.3f}m"
        )
    if args.check_global_relocalize:
        if bbs3d_disabled:
            blockers.append("BBS3D is disabled or did not load its map")
        if not bbs3d_ok:
            blockers.append("BBS3D global relocalize success was not observed")
        if lost_health_samples <= 0 and "Localization health → LOST" not in localizer_tail:
            blockers.append("kidnapped localizer did not report LOST before recovery")
        min_global_xy = float(args.min_global_map_odom_xy_m)
        if min_global_xy > 0.0 and map_to_odom_xy is not None and map_to_odom_xy < min_global_xy:
            blockers.append(
                f"kidnapped map->odom XY correction {map_to_odom_xy:.3f}m is below "
                f"{min_global_xy:.3f}m"
            )

    ok = not blockers
    return {
        **report_base,
        "ok": ok,
        "runtime_relocalization_executed": bool(service.get("available")),
        "runtime_relocalization_validated": ok,
        "global_relocalization_requested": bool(args.check_global_relocalize),
        "global_relocalization_validated": bool(args.check_global_relocalize and ok),
        "blockers": blockers,
        "service": service,
        "live_feed": {
            "ok": live_report.get("ok"),
            "report": str(live_report_path),
            "process_returncode": live_proc.returncode if live_proc is not None else None,
            "base_blockers": live_report.get("base_blockers") or [],
            "outputs": live_report.get("outputs") or {},
            "fastlio2_z_consistency": live_report.get("fastlio2_z_consistency") or {},
            "fastlio2_motion_consistency": live_report.get("fastlio2_motion_consistency") or {},
            "frames": live_report.get("frames") or {},
            "scan_time_profile": live_report.get("scan_time_profile") or "",
        },
        "localizer": {
            "process_returncode": localizer_proc.returncode if localizer_proc is not None else None,
            "health_samples": int(sampler.counts.get("localization_health", 0)),
            "tracking_health_samples": int(tracking_health_samples),
            "lost_health_samples": int(lost_health_samples),
            "health_states_seen": sorted(set(sampler.health_states)),
            "latest_health_state": latest_health_state,
            "latest_health": sampler.health_latest,
            "saved_map_cloud_samples": int(sampler.counts.get("saved_map_cloud", 0)),
            "saved_map_cloud_points_latest": int(sampler.point_counts.get("saved_map_cloud", 0)),
            "map_to_odom_tf_samples": int(sampler.counts.get("map_to_odom_tf", 0)),
            "map_to_odom_latest": map_to_odom,
            "map_to_odom_xy_m": map_to_odom_xy,
            "map_to_odom_z_abs_m": map_to_odom_z_abs,
            "odom_delta_m": sampler.odom_delta_m,
            "frames": sampler.frames,
            "bbs3d_success_observed": bbs3d_ok,
            "bbs3d_disabled_observed": bbs3d_disabled,
            "kidnap_initial_pose": {
                "x": float(args.kidnap_initial_x),
                "y": float(args.kidnap_initial_y),
                "z": float(args.kidnap_initial_z),
                "yaw": float(args.kidnap_initial_yaw),
            } if args.check_global_relocalize else None,
        },
        "thresholds": {
            "min_saved_map_points": int(args.min_saved_map_points),
            "min_tracking_health_samples": int(args.min_tracking_health_samples),
            "max_map_odom_xy_m": float(args.max_map_odom_xy_m),
            "max_map_odom_z_abs_m": float(args.max_map_odom_z_abs_m),
            "max_fastlio_z_drift_m": float(args.max_fastlio_z_drift_m),
            "localizer_rough_score_thresh": localizer_thresholds.get("rough_score_thresh"),
            "localizer_refine_score_thresh": localizer_thresholds.get("refine_score_thresh"),
            "min_global_map_odom_xy_m": (
                float(args.min_global_map_odom_xy_m)
                if args.check_global_relocalize
                else None
            ),
            "bbs3d_num_threads": (
                int(args.bbs3d_num_threads) if args.check_global_relocalize else None
            ),
            "bbs3d_timeout_ms": (
                int(args.bbs3d_timeout_ms) if args.check_global_relocalize else None
            ),
        },
        "logs": {
            "localizer": str(run_dir / "localizer.log"),
            "localizer_tail": localizer_tail,
            "live_feed": str(run_dir / "live_feed/gate.log"),
            "live_feed_tail": _tail(run_dir / "live_feed/gate.log"),
        },
        "wall_time_s": round(time.time() - started, 3),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map-pcd", default="latest")
    parser.add_argument(
        "--run-dir",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/saved_map_relocalize_runtime",
    )
    parser.add_argument(
        "--json-out",
        type=Path,
        default=ROOT / "artifacts/server_sim_closure/saved_map_relocalize_runtime/report.json",
    )
    parser.add_argument(
        "--world",
        default="map_metadata",
        help=(
            "MuJoCo world for the live feed. The default map_metadata uses "
            "same_source_map/metadata.json next to --map-pcd and falls back "
            "to industrial_park only when metadata is unavailable."
        ),
    )
    parser.add_argument("--duration", type=float, default=45.0)
    parser.add_argument(
        "--duration-clock",
        choices=["wall", "sim"],
        default="sim",
        help="Interpret the live-feed duration as wall-clock or MuJoCo simulation seconds.",
    )
    parser.add_argument(
        "--live-drive-source",
        choices=["frontier", "fixed"],
        default="frontier",
        help=(
            "frontier runs the same LingTu-driven motion used by the visible "
            "demo; fixed is useful only for diagnostics because stationary "
            "Fast-LIO can drift badly in this synthetic setup."
        ),
    )
    parser.add_argument("--drive-vx", type=float, default=0.25)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.06)
    parser.add_argument("--frontier-min-goals", type=int, default=0)
    parser.add_argument("--frontier-goal-timeout", type=float, default=240.0)
    parser.add_argument("--mid360-samples-per-frame", type=int, default=24000)
    parser.add_argument(
        "--localizer-config",
        default="",
        help=(
            "Optional localizer YAML. Defaults to the source-tree localizer.yaml "
            "or the installed share config."
        ),
    )
    parser.add_argument(
        "--localizer-rough-score-thresh",
        type=float,
        default=None,
        help="Optional runtime YAML override for localizer rough_score_thresh.",
    )
    parser.add_argument(
        "--localizer-refine-score-thresh",
        type=float,
        default=None,
        help="Optional runtime YAML override for localizer refine_score_thresh.",
    )
    parser.add_argument(
        "--scan-time-profile",
        choices=["map_metadata", "instantaneous", "synthetic_rolling", "physical_rolling"],
        default="map_metadata",
        help=(
            "Per-point time model passed to the MuJoCo Fast-LIO live feed. "
            "The default reads same_source_map/metadata.json and falls back "
            "to the strict physical_rolling profile when unavailable."
        ),
    )
    parser.add_argument("--imu-acc-mode", choices=["gravity_only", "finite_difference"], default="finite_difference")
    parser.add_argument("--max-fastlio-z-drift-m", type=float, default=1.0)
    parser.add_argument("--topic-timeout-s", type=float, default=25.0)
    parser.add_argument("--service-timeout-s", type=float, default=25.0)
    parser.add_argument("--monitor-after-service-s", type=float, default=18.0)
    parser.add_argument(
        "--check-global-relocalize",
        action="store_true",
        help="Use the canonical global relocalize service instead of seeded relocalize.",
    )
    parser.add_argument("--kidnap-initial-x", type=float, default=3.0)
    parser.add_argument("--kidnap-initial-y", type=float, default=2.0)
    parser.add_argument("--kidnap-initial-z", type=float, default=0.0)
    parser.add_argument("--kidnap-initial-yaw", type=float, default=1.2)
    parser.add_argument(
        "--min-global-map-odom-xy-m",
        type=float,
        default=0.0,
        help="Optional minimum map->odom XY correction expected in kidnapped global relocalization.",
    )
    parser.add_argument("--bbs3d-num-threads", type=int, default=4)
    parser.add_argument("--bbs3d-timeout-ms", type=int, default=90000)
    parser.add_argument(
        "--live-process-timeout-s",
        type=float,
        default=0.0,
        help="Override live feed process wait timeout. Default accounts for sim-time RTF.",
    )
    parser.add_argument("--min-saved-map-points", type=int, default=1000)
    parser.add_argument("--min-tracking-health-samples", type=int, default=3)
    parser.add_argument("--max-map-odom-xy-m", type=float, default=5.0)
    parser.add_argument("--max-map-odom-z-abs-m", type=float, default=2.0)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(args)
    text = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    args.json_out.parent.mkdir(parents=True, exist_ok=True)
    args.json_out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
