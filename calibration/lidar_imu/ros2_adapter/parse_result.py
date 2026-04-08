#!/usr/bin/env python3
"""Parse LiDAR_IMU_Init output (Initialization_result.txt) into structured YAML.

LI-Init writes a result file containing:
  - LiDAR-IMU rotation (3x3 matrix)
  - LiDAR-IMU translation (3x1 vector)
  - Time offset (lidar-to-imu)
  - IMU bias (accelerometer + gyroscope)
  - Gravity vector

This script parses that file and outputs a clean YAML that
apply_calibration.py can consume.

Usage:
    python calibration/lidar_imu/ros2_adapter/parse_result.py \
        --input calibration/lidar_imu/LiDAR_IMU_Init/result/Initialization_result.txt \
        --output calibration/lidar_imu/output/lidar_imu_calib.yaml
"""

import argparse
import logging
import re
import sys

import numpy as np
import yaml

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO, format="%(message)s")


def parse_initialization_result(filepath: str) -> dict:
    """Parse LI-Init Initialization_result.txt.

    The file format contains lines like:
        Rotation LiDAR to IMU =
        0.999  -0.001   0.002
        0.001   0.999  -0.003
        -0.002  0.003   0.999
        Translation LiDAR to IMU =
        -0.011  -0.023   0.044
        Time offset (lidar to imu) =    0.001
        Accelerometer bias =    0.01  0.02  -0.03
        Gyroscope bias =    0.001  -0.002  0.003
        Gravity =    0.00  0.00  -9.81
    """
    with open(filepath) as f:
        content = f.read()

    result = {}

    # Parse rotation matrix
    rot_match = re.search(
        r"Rotation\s+LiDAR\s+to\s+IMU\s*[=:]\s*\n"
        r"\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s*\n"
        r"\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s*\n"
        r"\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)",
        content, re.IGNORECASE,
    )
    if rot_match:
        vals = [float(rot_match.group(i)) for i in range(1, 10)]
        R = np.array(vals).reshape(3, 3)
        result["r_il"] = R.flatten().tolist()
        # Convert to roll/pitch/yaw for robot_config.yaml
        try:
            import cv2
            rvec, _ = cv2.Rodrigues(R)
            result["rotation_rodrigues"] = rvec.flatten().tolist()
        except ImportError:
            pass
    else:
        logger.warning("Could not parse rotation matrix")

    # Parse translation
    trans_match = re.search(
        r"Translation\s+LiDAR\s+to\s+IMU\s*[=:]\s*\n?"
        r"\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)",
        content, re.IGNORECASE,
    )
    if trans_match:
        t = [float(trans_match.group(i)) for i in range(1, 4)]
        result["t_il"] = t
    else:
        logger.warning("Could not parse translation")

    # Parse time offset
    time_match = re.search(
        r"[Tt]ime\s+offset.*?[=:]\s*([-\d.e+]+)",
        content,
    )
    if time_match:
        result["time_offset"] = float(time_match.group(1))

    # Parse accelerometer bias
    acc_bias_match = re.search(
        r"Accelerometer\s+bias\s*[=:]\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)",
        content, re.IGNORECASE,
    )
    if acc_bias_match:
        result["acc_bias"] = [float(acc_bias_match.group(i)) for i in range(1, 4)]

    # Parse gyroscope bias
    gyr_bias_match = re.search(
        r"Gyroscope\s+bias\s*[=:]\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)",
        content, re.IGNORECASE,
    )
    if gyr_bias_match:
        result["gyr_bias"] = [float(gyr_bias_match.group(i)) for i in range(1, 4)]

    # Parse gravity
    grav_match = re.search(
        r"Gravity\s*[=:]\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)",
        content, re.IGNORECASE,
    )
    if grav_match:
        result["gravity"] = [float(grav_match.group(i)) for i in range(1, 4)]

    # Try to parse 4x4 transformation matrix if present
    tf_match = re.search(
        r"(?:Transformation|T_LI|Extrinsic)\s*[=:]?\s*\n"
        r"\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s*\n"
        r"\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s*\n"
        r"\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s*\n"
        r"\s*([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)\s+([-\d.e+]+)",
        content, re.IGNORECASE,
    )
    if tf_match:
        vals = [float(tf_match.group(i)) for i in range(1, 17)]
        T = np.array(vals).reshape(4, 4)
        result["T_lidar_imu"] = T.tolist()

    return result


def validate_result(result: dict) -> list:
    """Sanity-check calibration result. Returns list of warnings."""
    warnings = []

    if "t_il" in result:
        t = np.array(result["t_il"])
        mag = np.linalg.norm(t)
        if mag > 0.5:
            warnings.append(
                f"LiDAR-IMU translation magnitude {mag:.3f}m seems large "
                f"for co-located sensors"
            )

    if "r_il" in result:
        R = np.array(result["r_il"]).reshape(3, 3)
        det = np.linalg.det(R)
        if abs(det - 1.0) > 0.01:
            warnings.append(f"Rotation matrix determinant {det:.4f} != 1.0")
        # Check rotation angle
        angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
        if np.degrees(angle) > 30:
            warnings.append(
                f"Rotation angle {np.degrees(angle):.1f} deg seems too large"
            )

    if "gravity" in result:
        g = np.linalg.norm(result["gravity"])
        if abs(g - 9.81) > 0.5:
            warnings.append(f"Gravity magnitude {g:.3f} m/s^2 deviates from 9.81")

    if "time_offset" in result:
        dt = abs(result["time_offset"])
        if dt > 0.1:
            warnings.append(f"Time offset {dt:.4f}s seems large (>100ms)")

    return warnings


def save_yaml(result: dict, output_path: str) -> None:
    """Save parsed result as clean YAML."""
    with open(output_path, "w") as f:
        yaml.dump(result, f, default_flow_style=False, sort_keys=False)
    logger.info("Saved to %s", output_path)


def main():
    parser = argparse.ArgumentParser(description="Parse LiDAR_IMU_Init result")
    parser.add_argument("--input", required=True,
                        help="Path to Initialization_result.txt")
    parser.add_argument("--output", default="calibration/lidar_imu/output/lidar_imu_calib.yaml",
                        help="Output YAML path")
    args = parser.parse_args()

    logger.info("Parsing %s ...", args.input)
    result = parse_initialization_result(args.input)

    if not result:
        logger.error("Failed to parse any calibration data")
        sys.exit(1)

    # Validate
    warnings = validate_result(result)
    for w in warnings:
        logger.warning("  WARNING: %s", w)

    # Print summary
    if "t_il" in result:
        t = result["t_il"]
        logger.info("  Translation (LiDAR→IMU): [%.5f, %.5f, %.5f]", *t)
    if "r_il" in result:
        R = np.array(result["r_il"]).reshape(3, 3)
        angle = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
        logger.info("  Rotation angle: %.2f deg", angle)
    if "time_offset" in result:
        logger.info("  Time offset: %.6f s", result["time_offset"])
    if "gravity" in result:
        g = result["gravity"]
        logger.info("  Gravity: [%.4f, %.4f, %.4f] (norm=%.4f)",
                     g[0], g[1], g[2], np.linalg.norm(g))

    save_yaml(result, args.output)

    if not warnings:
        logger.info("  All checks passed")


if __name__ == "__main__":
    main()
