"""Fast-LIO2 live bridge helpers used by simulation and runtime gates.

This module owns the reusable Fast-LIO2 process/config boundary. Validation
scripts may start and observe it, but they should not keep their own copy of
the Fast-LIO launch/config behavior.
"""

from __future__ import annotations

import os
import signal
import shutil
import subprocess
from pathlib import Path
from typing import TextIO


def write_fastlio2_config(
    path: Path,
    *,
    imu_topic: str,
    lidar_topic: str,
    lidar_type: int = 3,
    scan_line: int = 4,
    timestamp_unit: int = 3,
    livox_scan_window: float | None = None,
    r_il: tuple[float, ...] | list[float] | None = None,
    t_il: tuple[float, ...] | list[float] | None = None,
    imu_static_acc_thresh: float = 0.04,
    imu_static_gyro_thresh: float = 0.001,
    zupt_min_static_frames: int = 5,
    lidar_filter_num: int = 4,
    scan_resolution: float = 0.15,
    map_resolution: float = 0.3,
    near_search_num: int = 5,
    ieskf_max_iter: int = 5,
    degeneracy_max_update_dof: int = 2,
    degeneracy_max_condition: float = 50_000.0,
    max_update_translation_m: float = 0.5,
    max_update_rotation_rad: float = 0.35,
    reject_nonconverged_update: bool = True,
    reject_degenerate_nonconverged_update: bool = True,
    lidar_cov_inv: float = 1000.0,
    time_diff_lidar_to_imu: float = 0.0,
    vertical_velocity_constraint_enabled: bool = False,
    vertical_velocity_sigma_v: float = 0.05,
) -> None:
    """Write a Fast-LIO2 YAML config for a live LiDAR/IMU source."""

    r_il_values = (
        list(r_il)
        if r_il is not None
        else [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    )
    t_il_values = list(t_il) if t_il is not None else [0.0, 0.0, 0.0]
    if len(r_il_values) != 9:
        raise ValueError("r_il must contain 9 row-major rotation values")
    if len(t_il_values) != 3:
        raise ValueError("t_il must contain 3 translation values")

    r_il_text = ", ".join(f"{float(value):.9g}" for value in r_il_values)
    t_il_text = ", ".join(f"{float(value):.9g}" for value in t_il_values)
    lines = [
        f"imu_topic: {imu_topic}",
        f"lidar_topic: {lidar_topic}",
        "body_frame: body",
        "world_frame: odom",
        "print_time_cost: false",
        "",
        "# Generic PointCloud2/Livox live source. The caller owns source fidelity;",
        "# this config describes the algorithm I/O contract for Fast-LIO2.",
        f"lidar_type: {int(lidar_type)}",
        f"scan_line: {int(scan_line)}",
        f"timestamp_unit: {int(timestamp_unit)}",
        "acc_scale: 1.0",
    ]
    if livox_scan_window is not None:
        lines.append(f"livox_scan_window: {float(livox_scan_window):.6f}")
    lines.extend(
        [
            "",
            f"lidar_filter_num: {int(lidar_filter_num)}",
            "lidar_min_range: 0.5",
            "lidar_max_range: 30.0",
            f"scan_resolution: {float(scan_resolution):.9g}",
            f"map_resolution: {float(map_resolution):.9g}",
            "",
            "cube_len: 2000",
            "det_range: 60",
            "move_thresh: 1.5",
            "max_map_points: 1000000",
            "stationary_thresh: 0.05",
            "",
            "na: 0.01",
            "ng: 0.01",
            "nba: 0.0001",
            "nbg: 0.0001",
            "",
            "imu_init_num: 20",
            f"near_search_num: {int(near_search_num)}",
            f"ieskf_max_iter: {int(ieskf_max_iter)}",
            f"degeneracy_max_update_dof: {int(degeneracy_max_update_dof)}",
            f"degeneracy_max_condition: {float(degeneracy_max_condition):.9g}",
            f"max_update_translation_m: {float(max_update_translation_m):.9g}",
            f"max_update_rotation_rad: {float(max_update_rotation_rad):.9g}",
            "reject_nonconverged_update: "
            f"{str(bool(reject_nonconverged_update)).lower()}",
            "reject_degenerate_nonconverged_update: "
            f"{str(bool(reject_degenerate_nonconverged_update)).lower()}",
            f"time_diff_lidar_to_imu: {float(time_diff_lidar_to_imu):.9g}",
            "",
            "gravity_align: true",
            "esti_il: false",
            f"r_il: [{r_il_text}]",
            f"t_il: [{t_il_text}]",
            f"lidar_cov_inv: {float(lidar_cov_inv):.9g}",
            "",
            f"imu_static_acc_thresh: {float(imu_static_acc_thresh):.9g}",
            f"imu_static_gyro_thresh: {float(imu_static_gyro_thresh):.9g}",
            f"zupt_min_static_frames: {int(zupt_min_static_frames)}",
            "zupt_sigma_v: 0.02",
            "zupt_sigma_pos: 0.1",
            "vertical_velocity_constraint_enabled: "
            f"{str(bool(vertical_velocity_constraint_enabled)).lower()}",
            f"vertical_velocity_sigma_v: {float(vertical_velocity_sigma_v):.9g}",
            "",
        ]
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines), encoding="utf-8")


def ensure_ros2_cli() -> str:
    ros2 = shutil.which("ros2")
    if not ros2:
        raise RuntimeError("ros2 CLI is not available")
    return ros2


def tail_file(path: Path, limit: int = 4000) -> str:
    if not path.exists():
        return ""
    data = path.read_bytes()
    return data[-limit:].decode("utf-8", errors="replace")


class FastLio2Process:
    """Small owner for the Fast-LIO2 ROS process lifecycle."""

    def __init__(
        self,
        *,
        ros2_executable: str,
        config_path: Path,
        log_path: Path,
        env: dict[str, str] | None = None,
    ) -> None:
        self.ros2_executable = str(ros2_executable)
        self.config_path = Path(config_path)
        self.log_path = Path(log_path)
        self.env = dict(env or os.environ)
        self._process: subprocess.Popen[str] | None = None
        self._log_file: TextIO | None = None

    @property
    def returncode(self) -> int | None:
        if self._process is None:
            return None
        return self._process.returncode

    def start(self) -> None:
        if self._process is not None:
            raise RuntimeError("Fast-LIO2 process already started")
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_file = self.log_path.open("w", encoding="utf-8")
        self._process = subprocess.Popen(
            [
                self.ros2_executable,
                "run",
                "fastlio2",
                "lio_node",
                "--ros-args",
                "-p",
                f"config_path:={self.config_path}",
            ],
            stdout=self._log_file,
            stderr=subprocess.STDOUT,
            text=True,
            env=self.env,
            start_new_session=True,
        )

    def poll(self) -> int | None:
        if self._process is None:
            return None
        return self._process.poll()

    def stop(self, *, timeout_s: float = 3.0) -> None:
        process = self._process
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=timeout_s)
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = None
