"""Trajectory metrics for SLAM simulation evaluation."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from math import sqrt
from typing import Iterable, Sequence

from .tum import TumPose


@dataclass(frozen=True)
class MatchedPose:
    """Timestamp-associated reference and estimated pose pair."""

    reference: TumPose
    estimate: TumPose
    dt: float


@dataclass(frozen=True)
class TrajectoryErrorSummary:
    """Translation-only trajectory error summary."""

    sample_count_reference: int
    sample_count_estimate: int
    matched_count: int
    duration_s: float
    coverage_ratio: float
    mean_translation_error_m: float
    rmse_translation_error_m: float
    max_translation_error_m: float
    final_translation_error_m: float
    path_length_reference_m: float
    path_length_estimate_m: float
    drift_per_meter: float
    max_time_offset_s: float

    def to_dict(self) -> dict[str, float | int]:
        return asdict(self)


def _sorted_by_timestamp(poses: Sequence[TumPose]) -> list[TumPose]:
    return sorted(poses, key=lambda pose: pose.timestamp)


def associate_by_timestamp(
    reference: Sequence[TumPose],
    estimate: Sequence[TumPose],
    *,
    max_dt: float = 0.02,
) -> list[MatchedPose]:
    """Greedily associate poses by nearest timestamp within ``max_dt`` seconds."""

    if max_dt < 0.0:
        raise ValueError("max_dt must be non-negative")

    ref_sorted = _sorted_by_timestamp(reference)
    est_sorted = _sorted_by_timestamp(estimate)
    matches: list[MatchedPose] = []
    est_index = 0

    for ref in ref_sorted:
        while est_index < len(est_sorted) and est_sorted[est_index].timestamp < ref.timestamp - max_dt:
            est_index += 1
        best_index = -1
        best_dt = max_dt + 1.0
        for candidate_index in (est_index, est_index + 1):
            if candidate_index >= len(est_sorted):
                continue
            candidate = est_sorted[candidate_index]
            dt = abs(candidate.timestamp - ref.timestamp)
            if dt <= max_dt and dt < best_dt:
                best_index = candidate_index
                best_dt = dt
        if best_index >= 0:
            estimate_pose = est_sorted[best_index]
            matches.append(MatchedPose(reference=ref, estimate=estimate_pose, dt=best_dt))
            est_index = best_index + 1

    return matches


def path_length(poses: Iterable[TumPose]) -> float:
    """Compute path length from consecutive translation samples."""

    total = 0.0
    previous: TumPose | None = None
    for pose in poses:
        if previous is not None:
            total += translation_distance(previous, pose)
        previous = pose
    return total


def translation_distance(a: TumPose, b: TumPose) -> float:
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    return sqrt(dx * dx + dy * dy + dz * dz)


def evaluate_trajectory(
    reference: Sequence[TumPose],
    estimate: Sequence[TumPose],
    *,
    max_dt: float = 0.02,
) -> TrajectoryErrorSummary:
    """Evaluate a trajectory without spatial alignment.

    This intentionally reports the raw closed-loop error. Alignment can hide
    frame and timestamp wiring mistakes, so it should live in a separate layer.
    """

    if not reference:
        raise ValueError("reference trajectory is empty")
    if not estimate:
        raise ValueError("estimated trajectory is empty")

    matches = associate_by_timestamp(reference, estimate, max_dt=max_dt)
    if not matches:
        raise ValueError("no trajectory samples matched within max_dt")

    errors = [translation_distance(match.reference, match.estimate) for match in matches]
    mean_error = sum(errors) / len(errors)
    rmse_error = sqrt(sum(error * error for error in errors) / len(errors))
    max_error = max(errors)
    final_error = errors[-1]
    reference_length = path_length(_sorted_by_timestamp(reference))
    estimate_length = path_length(_sorted_by_timestamp(estimate))
    drift_per_meter = final_error / reference_length if reference_length > 0.0 else 0.0
    ref_sorted = _sorted_by_timestamp(reference)
    duration = max(0.0, ref_sorted[-1].timestamp - ref_sorted[0].timestamp)

    return TrajectoryErrorSummary(
        sample_count_reference=len(reference),
        sample_count_estimate=len(estimate),
        matched_count=len(matches),
        duration_s=duration,
        coverage_ratio=len(matches) / len(reference),
        mean_translation_error_m=mean_error,
        rmse_translation_error_m=rmse_error,
        max_translation_error_m=max_error,
        final_translation_error_m=final_error,
        path_length_reference_m=reference_length,
        path_length_estimate_m=estimate_length,
        drift_per_meter=drift_per_meter,
        max_time_offset_s=max(match.dt for match in matches),
    )
