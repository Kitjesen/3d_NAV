#!/usr/bin/env python3
"""One-click calibration verification.

Checks all sensor calibration parameters in robot_config.yaml and SLAM configs
for sanity. Reports pass/warn/fail for each sensor.

Usage:
    python calibration/verify.py
    python calibration/verify.py --verbose
    python calibration/verify.py --config /path/to/robot_config.yaml
"""

import argparse
import logging
import sys
from pathlib import Path

import numpy as np
import yaml

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)

REPO_ROOT = Path(__file__).resolve().parent.parent
ROBOT_CONFIG = REPO_ROOT / "config" / "robot_config.yaml"
FASTLIO2_CONFIG = REPO_ROOT / "src" / "slam" / "fastlio2" / "config" / "lio.yaml"
POINTLIO_CONFIG = REPO_ROOT / "config" / "pointlio.yaml"

# ANSI colors
GREEN = "\033[92m"
YELLOW = "\033[93m"
RED = "\033[91m"
RESET = "\033[0m"
BOLD = "\033[1m"


def load_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class CheckResult:
    def __init__(self):
        self.passed = 0
        self.warned = 0
        self.failed = 0

    def ok(self, msg: str) -> None:
        self.passed += 1
        logger.info(f"  {GREEN}PASS{RESET}  {msg}")

    def warn(self, msg: str) -> None:
        self.warned += 1
        logger.info(f"  {YELLOW}WARN{RESET}  {msg}")

    def fail(self, msg: str) -> None:
        self.failed += 1
        logger.info(f"  {RED}FAIL{RESET}  {msg}")

    @property
    def total(self) -> int:
        return self.passed + self.warned + self.failed

    @property
    def success(self) -> bool:
        return self.failed == 0


def check_camera(cfg: dict, result: CheckResult, verbose: bool) -> None:
    """Verify camera calibration parameters."""
    logger.info(f"\n{BOLD}== Camera =={RESET}")
    cam = cfg.get("camera", {})

    if not cam:
        result.fail("No camera section in robot_config.yaml")
        return

    # Intrinsics
    fx = cam.get("fx", 0)
    fy = cam.get("fy", 0)
    cx = cam.get("cx", 0)
    cy = cam.get("cy", 0)
    w = cam.get("width", 0)
    h = cam.get("height", 0)

    if fx <= 0 or fy <= 0:
        result.fail(f"Invalid focal length: fx={fx}, fy={fy}")
    elif abs(fx - fy) / max(fx, fy) > 0.1:
        result.warn(f"fx ({fx:.1f}) and fy ({fy:.1f}) differ by >10%")
    else:
        result.ok(f"Focal length: fx={fx:.1f}, fy={fy:.1f}")

    if w > 0 and h > 0:
        # Principal point should be near image center
        if abs(cx - w / 2) > w * 0.15 or abs(cy - h / 2) > h * 0.15:
            result.warn(f"Principal point ({cx:.1f}, {cy:.1f}) far from center ({w/2}, {h/2})")
        else:
            result.ok(f"Principal point: ({cx:.1f}, {cy:.1f}) in {w}x{h}")
    else:
        result.fail(f"Invalid image size: {w}x{h}")

    # Distortion
    dk1 = cam.get("dist_k1", 0)
    dk2 = cam.get("dist_k2", 0)
    if abs(dk1) > 2.0 or abs(dk2) > 2.0:
        result.warn(f"Large distortion coefficients: k1={dk1:.4f}, k2={dk2:.4f}")
    else:
        has_dist = any(abs(cam.get(k, 0)) > 1e-10
                       for k in ["dist_k1", "dist_k2", "dist_p1", "dist_p2", "dist_k3"])
        if has_dist:
            result.ok(f"Distortion calibrated: k1={dk1:.6f}, k2={dk2:.6f}")
        else:
            result.warn("No distortion coefficients (all zero) — may need calibration")

    # Extrinsics
    px = cam.get("position_x", 0)
    py = cam.get("position_y", 0)
    pz = cam.get("position_z", 0)

    if px == 0 and py == 0 and pz == 0:
        result.fail("Camera position is all zeros — extrinsics not calibrated")
    elif abs(px) > 2 or abs(py) > 2 or abs(pz) > 3:
        result.warn(f"Camera position ({px:.3f}, {py:.3f}, {pz:.3f}) seems too large")
    else:
        result.ok(f"Camera position: ({px:.3f}, {py:.3f}, {pz:.3f}) m")

    depth_scale = cam.get("depth_scale", 1.0)
    if depth_scale <= 0 or depth_scale > 1.0:
        result.warn(f"Unusual depth_scale: {depth_scale} (expected 0.001 for mm)")
    elif verbose:
        result.ok(f"Depth scale: {depth_scale}")


def check_lidar(cfg: dict, result: CheckResult, verbose: bool) -> None:
    """Verify LiDAR calibration parameters."""
    logger.info(f"\n{BOLD}== LiDAR =={RESET}")
    lidar = cfg.get("lidar", {})

    if not lidar:
        result.fail("No lidar section in robot_config.yaml")
        return

    ox = lidar.get("offset_x", 0)
    oy = lidar.get("offset_y", 0)
    oz = lidar.get("offset_z", 0)
    mag = (ox**2 + oy**2 + oz**2) ** 0.5

    if mag > 0.5:
        result.warn(f"LiDAR offset magnitude {mag:.4f}m seems large")
    elif mag < 0.001:
        result.warn(f"LiDAR offset near zero ({mag:.4f}m) — may not be calibrated")
    else:
        result.ok(f"LiDAR offset: ({ox:.5f}, {oy:.5f}, {oz:.5f}) m = {mag:.4f}m")

    roll = lidar.get("roll", 0)
    pitch = lidar.get("pitch", 0)
    yaw = lidar.get("yaw", 0)
    rot_mag = (roll**2 + pitch**2 + yaw**2) ** 0.5
    if np.degrees(rot_mag) > 30:
        result.warn(f"LiDAR rotation {np.degrees(rot_mag):.1f} deg seems large")
    elif verbose:
        result.ok(f"LiDAR rotation: [{roll:.4f}, {pitch:.4f}, {yaw:.4f}] rad")


def check_imu(result: CheckResult, verbose: bool) -> None:
    """Verify IMU noise parameters in SLAM configs."""
    logger.info(f"\n{BOLD}== IMU Noise (SLAM configs) =={RESET}")

    checked = False

    if FASTLIO2_CONFIG.exists():
        lio = load_yaml(FASTLIO2_CONFIG)
        na = lio.get("na", 0.01)
        ng = lio.get("ng", 0.01)
        nba = lio.get("nba", 0.0001)
        nbg = lio.get("nbg", 0.0001)

        # Typical ranges for MEMS IMU
        if na <= 0 or ng <= 0:
            result.fail(f"Non-positive noise: na={na}, ng={ng}")
        elif na > 0.1 or ng > 0.1:
            result.warn(f"High noise values: na={na}, ng={ng} — consider re-calibrating")
        else:
            result.ok(f"Fast-LIO2: na={na}, ng={ng}, nba={nba}, nbg={nbg}")

        # Check LiDAR-IMU extrinsics in SLAM config
        r_il = lio.get("r_il")
        t_il = lio.get("t_il")
        if t_il:
            result.ok(f"Fast-LIO2 t_il: {t_il}")
        if r_il and verbose:
            R = np.array(r_il).reshape(3, 3)
            det = np.linalg.det(R)
            if abs(det - 1.0) > 0.01:
                result.fail(f"r_il determinant = {det:.4f} (expected 1.0)")
            else:
                result.ok(f"Fast-LIO2 r_il determinant: {det:.6f}")

        # Gravity alignment
        gravity_align = lio.get("gravity_align", False)
        if gravity_align:
            result.ok("Gravity alignment: enabled")
        elif verbose:
            result.warn("Gravity alignment: disabled — IMU init may be inaccurate")

        checked = True

    if POINTLIO_CONFIG.exists():
        pio = load_yaml(POINTLIO_CONFIG)
        acc_cov = pio.get("imu_meas_acc_cov", 0.01)
        omg_cov = pio.get("imu_meas_omg_cov", 0.01)
        if acc_cov > 0 and omg_cov > 0:
            result.ok(f"Point-LIO: acc_cov={acc_cov}, omg_cov={omg_cov}")
        checked = True

    if not checked:
        result.warn("No SLAM config files found — skipping IMU checks")


def check_consistency(cfg: dict, result: CheckResult) -> None:
    """Cross-check between different calibration parameters."""
    logger.info(f"\n{BOLD}== Cross-Checks =={RESET}")

    cam = cfg.get("camera", {})
    lidar = cfg.get("lidar", {})

    if cam and lidar:
        cam_z = cam.get("position_z", 0)
        lidar_z = lidar.get("offset_z", 0)
        # Camera should be above LiDAR typically
        if cam_z > 0 and lidar_z > 0:
            result.ok(f"Camera Z ({cam_z:.3f}m) and LiDAR Z ({lidar_z:.4f}m) both positive")
        elif cam_z == 0:
            result.warn("Camera Z is 0 — likely uncalibrated")

    # Check SLAM configs match robot_config
    if FASTLIO2_CONFIG.exists() and lidar:
        lio = load_yaml(FASTLIO2_CONFIG)
        t_il_lio = lio.get("t_il")
        if t_il_lio:
            t_cfg = [lidar.get("offset_x", 0), lidar.get("offset_y", 0),
                     lidar.get("offset_z", 0)]
            if np.allclose(t_il_lio, t_cfg, atol=0.001):
                result.ok("LiDAR offsets match between robot_config.yaml and lio.yaml")
            else:
                result.warn(
                    f"LiDAR offset mismatch: robot_config={t_cfg} vs lio.yaml={t_il_lio}"
                )


def main():
    parser = argparse.ArgumentParser(description="Verify sensor calibration")
    parser.add_argument("--config", default=str(ROBOT_CONFIG),
                        help="Path to robot_config.yaml")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Show additional checks")
    args = parser.parse_args()

    cfg = load_yaml(Path(args.config))
    if not cfg:
        logger.error("Failed to load config from %s", args.config)
        sys.exit(1)

    result = CheckResult()

    check_camera(cfg, result, args.verbose)
    check_lidar(cfg, result, args.verbose)
    check_imu(result, args.verbose)
    check_consistency(cfg, result)

    # Summary
    logger.info(f"\n{BOLD}== Summary =={RESET}")
    logger.info(f"  {GREEN}{result.passed} passed{RESET}, "
                f"{YELLOW}{result.warned} warnings{RESET}, "
                f"{RED}{result.failed} failed{RESET}")

    if result.failed > 0:
        logger.info(f"\n{RED}Some checks failed. Run calibration for affected sensors.{RESET}")
        sys.exit(1)
    elif result.warned > 0:
        logger.info(f"\n{YELLOW}All critical checks passed, but some warnings need attention.{RESET}")
    else:
        logger.info(f"\n{GREEN}All calibration checks passed.{RESET}")


if __name__ == "__main__":
    main()
