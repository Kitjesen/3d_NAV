"""SLAM simulation evaluation primitives.

This package is intentionally independent from ROS runtime code. It provides
small, deterministic building blocks that can be used by future replay scripts,
simulation jobs, and CI checks.
"""

from .manifest import (
    SlamEvalBackend,
    SlamEvalCase,
    SlamEvalRobot,
    SlamEvalSensorSuite,
    case_from_dict,
    load_case,
)
from .metrics import (
    MatchedPose,
    TrajectoryErrorSummary,
    associate_by_timestamp,
    evaluate_trajectory,
    path_length,
)
from .tum import TumPose, parse_tum_line, read_tum_trajectory, write_tum_trajectory

__all__ = [
    "MatchedPose",
    "SlamEvalBackend",
    "SlamEvalCase",
    "SlamEvalRobot",
    "SlamEvalSensorSuite",
    "TrajectoryErrorSummary",
    "TumPose",
    "associate_by_timestamp",
    "case_from_dict",
    "evaluate_trajectory",
    "load_case",
    "parse_tum_line",
    "path_length",
    "read_tum_trajectory",
    "write_tum_trajectory",
]
