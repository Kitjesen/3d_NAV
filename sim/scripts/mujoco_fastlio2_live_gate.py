#!/usr/bin/env python3
"""Run Fast-LIO2 on live MuJoCo LiDAR/IMU and verify LingTu bridge outputs.

This gate is simulation-only. It does not connect to robot services and does
not publish hardware commands. A headless MuJoCo engine generates raw
`/points_raw` and `/imu_raw`; Fast-LIO2 consumes those topics and publishes
real algorithm outputs, which are then fed into `SlamBridgeModule`.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import signal
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from sim.scripts.fastlio2_rosbag_replay_gate import _tail, _write_fastlio2_config


def _load_ros_modules():
    try:
        import rclpy  # type: ignore
        from nav_msgs.msg import Odometry  # type: ignore
        from sensor_msgs.msg import Imu, PointCloud2, PointField  # type: ignore
    except Exception as exc:  # pragma: no cover - depends on ROS2 env
        raise RuntimeError(
            "ROS2 Python modules are unavailable. Source /opt/ros/humble/setup.bash "
            "and the LingTu install/setup.bash before running this gate."
        ) from exc
    return rclpy, Imu, Odometry, PointCloud2, PointField


def _resolve_world(world: str) -> Path:
    from drivers.sim.mujoco_driver_module import WORLDS, _WORLDS_DIR

    candidate = Path(world)
    if candidate.exists():
        return candidate.resolve()
    mapped = WORLDS.get(world, world)
    path = (_WORLDS_DIR / mapped).resolve()
    if not path.exists():
        raise FileNotFoundError(f"MuJoCo world not found: {world} -> {path}")
    return path


def _scene_start(scene_xml: Path) -> list[float] | None:
    from drivers.sim.mujoco_driver_module import _scene_placeholder_start

    start = _scene_placeholder_start(scene_xml)
    if start is not None:
        return start
    try:
        import xml.etree.ElementTree as ET

        root = ET.fromstring(scene_xml.read_text(encoding="utf-8", errors="ignore"))
        worldbody = root.find("worldbody")
        if worldbody is None:
            return None
        for body in worldbody.findall("body"):
            if body.attrib.get("name") != "base_link":
                continue
            parts = [float(v) for v in body.attrib.get("pos", "").split()]
            if len(parts) >= 3:
                return parts[:3]
    except Exception:
        return None
    return None


def _parse_start(value: str) -> list[float] | None:
    if not value:
        return None
    parts = [float(item.strip()) for item in value.replace(";", ",").split(",") if item.strip()]
    if len(parts) != 3:
        raise ValueError("--start must be formatted as x,y,z")
    return parts


def _scene_with_memory(scene_xml: Path, memory: str) -> Path:
    if not memory:
        return scene_xml
    text = scene_xml.read_text(encoding="utf-8", errors="ignore")
    if "<size " in text:
        return scene_xml
    marker_start = text.find("<mujoco")
    if marker_start < 0:
        return scene_xml
    marker_end = text.find(">", marker_start)
    if marker_end < 0:
        return scene_xml
    patched = text[: marker_end + 1] + f'\n  <size memory="{memory}"/>' + text[marker_end + 1 :]
    tmp = tempfile.NamedTemporaryFile(
        suffix=".xml",
        prefix=f"{scene_xml.stem}_memory_",
        dir=str(scene_xml.parent),
        mode="w",
        encoding="utf-8",
        delete=False,
    )
    tmp.write(patched)
    tmp.close()
    return Path(tmp.name)


def _build_engine(
    *,
    world: Path,
    drive_mode: str,
    n_rays: int,
    start: list[float] | None,
    mujoco_memory: str,
    camera_configs: list[Any] | None = None,
    robot_xml: Path | None = None,
    base_body_name: str = "base_link",
    lidar_body_name: str = "lidar_link",
    leg_joint_names: list[str] | None = None,
):
    from sim.engine.core.robot import RobotConfig
    from sim.engine.core.sensor import LidarConfig
    from sim.engine.core.world import WorldConfig
    from sim.engine.mujoco.engine import MuJoCoEngine

    robot_cfg = RobotConfig.default_thunder_v3()
    robot_cfg.resolve_paths(base_dir=str(ROOT / "sim"))
    if robot_xml is not None:
        robot_cfg.robot_xml = str(robot_xml)
    robot_cfg.base_body_name = str(base_body_name)
    robot_cfg.lidar_body_name = str(lidar_body_name)
    if leg_joint_names is not None:
        robot_cfg.leg_joint_names = list(leg_joint_names)
    start = start or _scene_start(world)
    if start is not None:
        robot_cfg.init_position = [float(v) for v in start[:3]]

    load_world = _scene_with_memory(world, mujoco_memory)
    engine = MuJoCoEngine(
        robot_config=robot_cfg,
        world_config=WorldConfig(scene_xml=str(load_world)),
        lidar_config=LidarConfig(
            body_name=robot_cfg.lidar_body_name,
            n_rays=int(n_rays),
            geom_group=0,
            add_noise=False,
        ),
        camera_configs=camera_configs or [],
        headless=True,
        drive_mode=drive_mode,
    )
    try:
        engine.load(str(load_world))
        engine.reset()
    finally:
        if load_world != world:
            load_world.unlink(missing_ok=True)
    return engine


def _stamp_from_seconds(stamp_s: float, time_cls: Any):
    msg = time_cls()
    sec = math.floor(stamp_s)
    msg.sec = int(sec)
    msg.nanosec = int(round((stamp_s - sec) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


def _quat_xyzw_to_matrix(q: np.ndarray) -> np.ndarray:
    x, y, z, w = [float(v) for v in q[:4]]
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _world_xyzi_to_sensor_xyzi(engine: Any, pts_xyzi_world: np.ndarray) -> np.ndarray:
    """Convert MuJoCo world-frame XYZI points into the LiDAR/body frame."""

    pts = np.asarray(pts_xyzi_world, dtype=np.float32)
    if pts.size == 0:
        return np.zeros((0, 4), dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError(f"expected point cloud shape (N, >=3), got {pts.shape}")

    data = getattr(engine, "_data", None)
    lidar_id = int(getattr(engine, "_lidar_body_id", 0))
    if data is not None and lidar_id >= 0:
        sensor_pos = np.asarray(data.xpos[lidar_id], dtype=np.float64)
        sensor_rmat = np.asarray(data.xmat[lidar_id], dtype=np.float64).reshape(3, 3)
    else:
        state = engine.get_robot_state()
        sensor_pos = np.asarray(state.position, dtype=np.float64)
        sensor_rmat = _quat_xyzw_to_matrix(np.asarray(state.orientation, dtype=np.float64))

    xyz_sensor = (pts[:, :3].astype(np.float64) - sensor_pos) @ sensor_rmat
    intensity = (
        pts[:, 3:4].astype(np.float32)
        if pts.shape[1] >= 4
        else np.full((len(pts), 1), 100.0, dtype=np.float32)
    )
    return np.hstack([xyz_sensor.astype(np.float32), intensity]).astype(np.float32, copy=False)


def _make_pointcloud2(
    *,
    points_xyzi: np.ndarray,
    stamp: Any,
    frame_id: str,
    pointcloud_cls: Any,
    pointfield_cls: Any,
):
    pts = np.asarray(points_xyzi, dtype=np.float32)
    if pts.ndim != 2 or pts.shape[1] != 4:
        raise ValueError(f"expected XYZI point cloud shape (N, 4), got {pts.shape}")

    msg = pointcloud_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(len(pts))
    msg.is_dense = False
    msg.is_bigendian = False
    msg.fields = [
        pointfield_cls(name="x", offset=0, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="y", offset=4, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="z", offset=8, datatype=pointfield_cls.FLOAT32, count=1),
        pointfield_cls(name="intensity", offset=12, datatype=pointfield_cls.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.row_step = 16 * int(len(pts))
    msg.data = pts.tobytes()
    return msg


def _specific_force_body(state: Any, prev_velocity: np.ndarray | None, dt: float) -> np.ndarray:
    velocity = np.asarray(state.linear_velocity, dtype=np.float64)
    if prev_velocity is None or dt <= 0.0:
        world_acc = np.zeros(3, dtype=np.float64)
    else:
        world_acc = (velocity - prev_velocity) / dt
    gravity_world = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    rot_body_to_world = _quat_xyzw_to_matrix(np.asarray(state.orientation, dtype=np.float64))
    return rot_body_to_world.T @ (world_acc - gravity_world)


def _make_imu_msg(*, state: Any, prev_velocity: np.ndarray | None, dt: float, stamp: Any, imu_cls: Any):
    msg = imu_cls()
    msg.header.stamp = stamp
    msg.header.frame_id = "body"
    gyro = np.asarray(state.imu_gyro, dtype=np.float64)
    acc = _specific_force_body(state, prev_velocity, dt)
    msg.angular_velocity.x = float(gyro[0])
    msg.angular_velocity.y = float(gyro[1])
    msg.angular_velocity.z = float(gyro[2])
    msg.linear_acceleration.x = float(acc[0])
    msg.linear_acceleration.y = float(acc[1])
    msg.linear_acceleration.z = float(acc[2])
    return msg


def _odom_xyz(msg: Any) -> list[float] | None:
    try:
        p = msg.pose.pose.position
        return [float(p.x), float(p.y), float(p.z)]
    except Exception:
        return None


def run_gate(
    *,
    world: Path,
    duration: float,
    drive_vx: float,
    drive_vy: float,
    drive_wz: float,
    n_rays: int,
    start: list[float] | None,
    mujoco_memory: str,
    startup_sleep: float,
    settle_sleep: float,
    work_dir: Path,
    backend_profile: str,
    drive_mode: str,
) -> dict[str, Any]:
    rclpy, Imu, Odometry, PointCloud2, PointField = _load_ros_modules()
    from builtin_interfaces.msg import Time as RosTime  # type: ignore
    from sim.engine.core.engine import VelocityCommand
    from slam.slam_bridge_module import SlamBridgeModule

    work_dir.mkdir(parents=True, exist_ok=True)
    config_path = work_dir / "fastlio2_mujoco_live.yaml"
    log_path = work_dir / "fastlio2_node.log"
    _write_fastlio2_config(config_path, imu_topic="/imu_raw", lidar_topic="/points_raw")

    ros2 = shutil.which("ros2")
    if not ros2:
        raise RuntimeError("ros2 CLI is not available")

    engine = _build_engine(
        world=world,
        drive_mode=drive_mode,
        n_rays=n_rays,
        start=start,
        mujoco_memory=mujoco_memory,
    )

    rclpy.init(args=None)
    node = rclpy.create_node("mujoco_fastlio2_live_gate")
    bridge = SlamBridgeModule(
        backend_profile=backend_profile,
        odom_timeout=2.0,
        cloud_timeout=2.0,
        localizer_health_timeout=2.0,
        watchdog_hz=20,
    )

    odom_out: list[Any] = []
    registered_cloud_out: list[Any] = []
    map_cloud_out: list[Any] = []
    bridge_statuses: list[dict[str, Any]] = []
    first_odom_xyz: list[float] | None = None
    last_odom_xyz: list[float] | None = None
    first_sim_xyz: list[float] | None = None
    last_sim_xyz: list[float] | None = None
    source_end_bridge_status: dict[str, Any] | None = None

    def on_odom(msg: Any) -> None:
        nonlocal first_odom_xyz, last_odom_xyz
        odom_out.append(msg)
        xyz = _odom_xyz(msg)
        if xyz is not None:
            first_odom_xyz = first_odom_xyz or xyz
            last_odom_xyz = xyz
        bridge._on_rclpy_odom(msg)
        worker = getattr(bridge, "_odom_worker_thread", None)
        if worker is not None:
            worker.join(timeout=0.1)

    def on_registered_cloud(msg: Any) -> None:
        registered_cloud_out.append(msg)

    def on_map_cloud(msg: Any) -> None:
        map_cloud_out.append(msg)
        bridge._process_rclpy_cloud(msg)

    bridge.localization_status._add_callback(bridge_statuses.append)
    node.create_subscription(Odometry, "/Odometry", on_odom, 100)
    node.create_subscription(PointCloud2, "/cloud_registered", on_registered_cloud, 10)
    node.create_subscription(PointCloud2, "/cloud_map", on_map_cloud, 10)
    imu_pub = node.create_publisher(Imu, "/imu_raw", 100)
    cloud_pub = node.create_publisher(PointCloud2, "/points_raw", 10)

    process: subprocess.Popen[str] | None = None
    log_file = log_path.open("w", encoding="utf-8")
    counts = {
        "imu_published": 0,
        "cloud_published": 0,
        "empty_cloud_frames": 0,
        "sim_steps": 0,
    }
    point_counts: list[int] = []
    subscription_counts = {"imu": 0, "cloud": 0}
    started = time.time()
    prev_velocity: np.ndarray | None = None
    prev_lidar_pub_sim_time = -1e9
    try:
        bridge.start()
        process = subprocess.Popen(
            [
                ros2,
                "run",
                "fastlio2",
                "lio_node",
                "--ros-args",
                "-p",
                f"config_path:={config_path}",
            ],
            stdout=log_file,
            stderr=subprocess.STDOUT,
            text=True,
            env=os.environ.copy(),
            start_new_session=True,
        )
        deadline = time.time() + startup_sleep
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if process.poll() is not None:
                break
            subscription_counts = {
                "imu": imu_pub.get_subscription_count(),
                "cloud": cloud_pub.get_subscription_count(),
            }
            if subscription_counts["imu"] > 0 and subscription_counts["cloud"] > 0:
                break

        sim_start_wall = time.time()
        while time.time() - sim_start_wall < duration:
            if process.poll() is not None:
                break

            loop_wall = time.time()
            state = engine.step(
                VelocityCommand(
                    linear_x=drive_vx,
                    linear_y=drive_vy,
                    angular_z=drive_wz,
                )
            )
            counts["sim_steps"] += 1
            first_sim_xyz = first_sim_xyz or [float(v) for v in state.position[:3]]
            last_sim_xyz = [float(v) for v in state.position[:3]]

            stamp = _stamp_from_seconds(time.time(), RosTime)
            imu_pub.publish(
                _make_imu_msg(
                    state=state,
                    prev_velocity=prev_velocity,
                    dt=float(engine.control_dt),
                    stamp=stamp,
                    imu_cls=Imu,
                )
            )
            counts["imu_published"] += 1
            prev_velocity = np.asarray(state.linear_velocity, dtype=np.float64)

            lidar_period = 1.0 / 10.0
            if engine.sim_time - prev_lidar_pub_sim_time >= lidar_period - 1e-6:
                cloud_world = engine.get_lidar_points()
                if cloud_world is None or len(cloud_world) == 0:
                    counts["empty_cloud_frames"] += 1
                else:
                    cloud_sensor = _world_xyzi_to_sensor_xyzi(engine, cloud_world)
                    point_counts.append(int(len(cloud_sensor)))
                    cloud_pub.publish(
                        _make_pointcloud2(
                            points_xyzi=cloud_sensor,
                            stamp=stamp,
                            frame_id="body",
                            pointcloud_cls=PointCloud2,
                            pointfield_cls=PointField,
                        )
                    )
                    counts["cloud_published"] += 1
                prev_lidar_pub_sim_time = float(engine.sim_time)

            rclpy.spin_once(node, timeout_sec=0.0)
            target_dt = float(engine.control_dt)
            elapsed = time.time() - loop_wall
            if elapsed < target_dt:
                deadline = time.time() + (target_dt - elapsed)
                while time.time() < deadline:
                    rclpy.spin_once(node, timeout_sec=min(0.005, max(0.0, deadline - time.time())))

        source_end_bridge_status = bridge_statuses[-1] if bridge_statuses else None
        deadline = time.time() + settle_sleep
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
        bridge.stop()
        node.destroy_node()
        rclpy.shutdown()
        log_file.close()
        engine.close()

    states = [str(item.get("state")) for item in bridge_statuses]
    source_end_state = str(source_end_bridge_status.get("state")) if source_end_bridge_status else ""
    shutdown_bridge_status = bridge_statuses[-1] if bridge_statuses else {}
    moved_m = math.dist(first_odom_xyz, last_odom_xyz) if first_odom_xyz and last_odom_xyz else None
    sim_moved_m = math.dist(first_sim_xyz, last_sim_xyz) if first_sim_xyz and last_sim_xyz else None
    process_returncode = process.returncode if process is not None else None
    algorithm_verified = len(odom_out) > 0 and len(map_cloud_out) > 0
    bridge_verified = source_end_state == "TRACKING" or any(state == "TRACKING" for state in states)
    ok = bool(
        counts["imu_published"] > 0
        and counts["cloud_published"] > 0
        and algorithm_verified
        and bridge_verified
    )
    return {
        "ok": ok,
        "algorithm": "fastlio2",
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "simulation_motion": abs(drive_vx) > 0.0 or abs(drive_vy) > 0.0 or abs(drive_wz) > 0.0,
        "live_mujoco_lidar_verified": counts["cloud_published"] > 0 and bool(point_counts),
        "live_mujoco_imu_verified": counts["imu_published"] > 0,
        "slam_algorithm_output_verified": algorithm_verified,
        "bridge_verified": bridge_verified,
        "true_mapping_input_path": "/points_raw + /imu_raw -> fastlio2 -> /Odometry + /cloud_map",
        "frames": {
            "mujoco_world": "world",
            "published_lidar": "body",
            "published_imu": "body",
            "fastlio2_body_frame": "body",
            "fastlio2_world_frame": "odom",
            "fastlio2_odometry": "odom",
            "fastlio2_cloud_map": "odom",
            "slam_bridge_output": "map",
            "cmd_vel": "not_published",
        },
        "world": str(world),
        "start_position": start or _scene_start(world),
        "mujoco_memory": mujoco_memory,
        "drive_mode": drive_mode,
        "commanded_sim_velocity": {
            "linear_x": drive_vx,
            "linear_y": drive_vy,
            "angular_z": drive_wz,
        },
        "counts": counts,
        "point_count": {
            "min": min(point_counts) if point_counts else 0,
            "max": max(point_counts) if point_counts else 0,
            "mean": round(float(np.mean(point_counts)), 1) if point_counts else 0.0,
        },
        "subscription_counts_at_start": subscription_counts,
        "outputs": {
            "fastlio2_odometry": len(odom_out),
            "fastlio2_cloud_registered": len(registered_cloud_out),
            "fastlio2_cloud_map": len(map_cloud_out),
            "bridge_localization_status": len(bridge_statuses),
        },
        "states_seen": states,
        "final_bridge_status": source_end_bridge_status or shutdown_bridge_status,
        "shutdown_bridge_status": shutdown_bridge_status,
        "first_odom_xyz": first_odom_xyz,
        "last_odom_xyz": last_odom_xyz,
        "fastlio2_moved_m": moved_m,
        "first_sim_xyz": first_sim_xyz,
        "last_sim_xyz": last_sim_xyz,
        "sim_moved_m": sim_moved_m,
        "wall_time_s": round(time.time() - started, 3),
        "process_returncode": process_returncode,
        "config_path": str(config_path),
        "log_path": str(log_path),
        "log_tail": _tail(log_path),
    }


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", default="building_scene")
    parser.add_argument("--start", default="", help="Optional start pose x,y,z; defaults to scene marker")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--drive-vx", type=float, default=0.25)
    parser.add_argument("--drive-vy", type=float, default=0.0)
    parser.add_argument("--drive-wz", type=float, default=0.06)
    parser.add_argument("--drive-mode", choices=["kinematic", "policy"], default="kinematic")
    parser.add_argument("--n-rays", type=int, default=6400)
    parser.add_argument("--mujoco-memory", default="64M")
    parser.add_argument("--startup-sleep", type=float, default=2.0)
    parser.add_argument("--settle-sleep", type=float, default=1.0)
    parser.add_argument("--backend-profile", default="fastlio2")
    parser.add_argument("--work-dir", default="artifacts/mujoco_fastlio2_live")
    parser.add_argument("--json-out", default="")
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    report = run_gate(
        world=_resolve_world(args.world),
        duration=args.duration,
        drive_vx=args.drive_vx,
        drive_vy=args.drive_vy,
        drive_wz=args.drive_wz,
        n_rays=args.n_rays,
        start=_parse_start(args.start),
        mujoco_memory=args.mujoco_memory,
        startup_sleep=args.startup_sleep,
        settle_sleep=args.settle_sleep,
        work_dir=Path(args.work_dir),
        backend_profile=args.backend_profile,
        drive_mode=args.drive_mode,
    )
    text = json.dumps(report, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        out = Path(args.json_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text + "\n", encoding="utf-8")
    return 0 if report.get("ok") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
