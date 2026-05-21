#!/usr/bin/env python3
"""Summarize fixed-drive Fast-LIO speed/config control reports."""

from __future__ import annotations

import argparse
import glob
import json
import math
from pathlib import Path
from typing import Any, Sequence


ROOT = Path(__file__).resolve().parents[2]


def _load_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(payload, dict):
        return payload
    return {}


def _safe_float(value: Any, *, default: float | None = None) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def _safe_int(value: Any, *, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _is_tuned_config(config: dict[str, Any]) -> bool:
    default_values = {
        "lidar_filter_num": 4.0,
        "scan_resolution": 0.15,
        "map_resolution": 0.30,
        "near_search_num": 5.0,
        "ieskf_max_iter": 5.0,
        "lidar_cov_inv": 1000.0,
    }
    for key, default in default_values.items():
        if key not in config:
            continue
        value = _safe_float(config.get(key))
        if value is not None and not math.isclose(value, default, rel_tol=1e-6, abs_tol=1e-9):
            return True
    return False


def _config_label(config: dict[str, Any]) -> str:
    return "tuned" if _is_tuned_config(config) else "default"


def _last_time_alignment(report: dict[str, Any]) -> dict[str, Any]:
    direct = report.get("time_alignment")
    if isinstance(direct, dict) and direct:
        return direct
    events = report.get("runtime_fault_events")
    if not isinstance(events, list):
        return {}
    for event in reversed(events):
        if not isinstance(event, dict):
            continue
        alignment = event.get("time_alignment")
        if isinstance(alignment, dict) and alignment:
            return alignment
    return {}


def _remaining_gaps(report: dict[str, Any]) -> list[str]:
    gaps = report.get("remaining_gaps")
    if not isinstance(gaps, list):
        return []
    return [str(gap) for gap in gaps if str(gap)]


def _has_wall_timeout(gaps: Sequence[str]) -> bool:
    return any("wall timeout" in gap.lower() for gap in gaps)


def _case_from_report(path: Path, report: dict[str, Any]) -> dict[str, Any]:
    commanded = (
        report.get("commanded_sim_velocity")
        if isinstance(report.get("commanded_sim_velocity"), dict)
        else {}
    )
    config = (
        report.get("fastlio2_sim_config")
        if isinstance(report.get("fastlio2_sim_config"), dict)
        else {}
    )
    z_check = (
        report.get("fastlio2_z_consistency")
        if isinstance(report.get("fastlio2_z_consistency"), dict)
        else {}
    )
    motion_check = (
        report.get("fastlio2_motion_consistency")
        if isinstance(report.get("fastlio2_motion_consistency"), dict)
        else {}
    )
    yaw_check = (
        report.get("fastlio2_yaw_consistency")
        if isinstance(report.get("fastlio2_yaw_consistency"), dict)
        else {}
    )
    speed = _safe_float(commanded.get("linear_x"), default=0.0) or 0.0
    lateral_y = _safe_float(commanded.get("linear_y"), default=0.0) or 0.0
    angular_z = _safe_float(commanded.get("angular_z"), default=0.0) or 0.0
    planar_speed = math.hypot(float(speed), float(lateral_y))
    source = str(commanded.get("source") or report.get("drive_source") or "")
    remaining_gaps = _remaining_gaps(report)
    blockers: list[str] = []
    if str(report.get("nav_data_source") or "").lower() != "fastlio2":
        blockers.append("nav_data_source is not fastlio2")
    if source != "fixed":
        blockers.append("commanded_sim_velocity.source is not fixed")
    if z_check.get("checked") is not True:
        blockers.append("fastlio2_z_consistency.checked is not true")
    if motion_check.get("checked") is not True:
        blockers.append("fastlio2_motion_consistency.checked is not true")
    if yaw_check.get("checked") is not True:
        blockers.append("fastlio2_yaw_consistency.checked is not true")

    return {
        "path": str(path),
        "ok": report.get("ok") is True,
        "control_valid": not blockers,
        "blockers": blockers,
        "speed_mps": round(float(speed), 6),
        "lateral_y_mps": round(float(lateral_y), 6),
        "abs_lateral_y_mps": round(abs(float(lateral_y)), 6),
        "planar_speed_mps": round(float(planar_speed), 6),
        "angular_z_radps": round(float(angular_z), 6),
        "abs_angular_z_radps": round(abs(float(angular_z)), 6),
        "drive_source": source,
        "sim_time_s": _safe_float(report.get("sim_time_s")),
        "wall_time_s": _safe_float(report.get("wall_time_s")),
        "duration_clock": str(report.get("duration_clock") or ""),
        "remaining_gaps": remaining_gaps,
        "wall_timeout": _has_wall_timeout(remaining_gaps),
        "tuned": _is_tuned_config(config),
        "config_label": _config_label(config),
        "config": config,
        "z_ok": z_check.get("ok") is True,
        "z_delta_error_m": _safe_float(z_check.get("z_delta_error_m")),
        "z_limit_m": _safe_float(z_check.get("max_allowed_z_drift_m")),
        "motion_ok": motion_check.get("ok") is True,
        "sim_moved_m": _safe_float(motion_check.get("sim_moved_m")),
        "fastlio2_moved_m": _safe_float(motion_check.get("fastlio2_moved_m")),
        "yaw_ok": yaw_check.get("ok") is True,
        "yaw_delta_error_rad": _safe_float(yaw_check.get("yaw_delta_error_rad")),
        "runtime_fault_count": _safe_int(len(report.get("runtime_fault_events") or [])),
        "time_alignment": _last_time_alignment(report),
    }


def _fixed_cases(cases: Sequence[dict[str, Any]], *, tuned: bool | None = None) -> list[dict[str, Any]]:
    result = [
        case
        for case in cases
        if case.get("control_valid") is True and str(case.get("drive_source")) == "fixed"
    ]
    if tuned is not None:
        result = [case for case in result if case.get("tuned") is tuned]
    return sorted(
        result,
        key=lambda case: (
            float(case.get("abs_angular_z_radps") or 0.0),
            float(case.get("speed_mps") or 0.0),
        ),
    )


def _straight_fixed_cases(cases: Sequence[dict[str, Any]], *, tuned: bool | None = None) -> list[dict[str, Any]]:
    return [
        case
        for case in _fixed_cases(cases, tuned=tuned)
        if abs(float(case.get("angular_z_radps") or 0.0)) <= 1e-6
    ]


def _blocking_subsystem(case: dict[str, Any]) -> str:
    if case.get("z_ok") is not True or case.get("motion_ok") is not True or case.get("yaw_ok") is not True:
        return "slam_localization"
    if case.get("wall_timeout") is True:
        return "validation_runtime"
    return "unknown"


def _minimal_red_defect(case: dict[str, Any] | None) -> dict[str, Any]:
    if not case:
        return {}
    primary_metric = "fastlio2_z_consistency"
    metric_value = case.get("z_delta_error_m")
    metric_threshold = case.get("z_limit_m")
    if case.get("wall_timeout") is True and _blocking_subsystem(case) == "validation_runtime":
        primary_metric = "gate_wall_timeout"
        metric_value = case.get("wall_time_s")
        metric_threshold = None
    elif case.get("motion_ok") is not True:
        primary_metric = "fastlio2_motion_consistency"
        metric_value = case.get("fastlio2_moved_m")
        metric_threshold = None
    return {
        "path": case.get("path"),
        "speed_mps": case.get("speed_mps"),
        "lateral_y_mps": case.get("lateral_y_mps"),
        "abs_lateral_y_mps": case.get("abs_lateral_y_mps"),
        "planar_speed_mps": case.get("planar_speed_mps"),
        "angular_z_radps": case.get("angular_z_radps"),
        "abs_angular_z_radps": case.get("abs_angular_z_radps"),
        "config_label": case.get("config_label"),
        "blocking_subsystem": _blocking_subsystem(case),
        "primary_metric": primary_metric,
        "metric_value": metric_value,
        "metric_threshold": metric_threshold,
        "sim_time_s": case.get("sim_time_s"),
        "wall_time_s": case.get("wall_time_s"),
        "z_delta_error_m": case.get("z_delta_error_m"),
        "motion_ok": case.get("motion_ok"),
        "yaw_delta_error_rad": case.get("yaw_delta_error_rad"),
        "remaining_gaps": case.get("remaining_gaps"),
        "time_alignment": case.get("time_alignment"),
    }


def _classification(case: dict[str, Any]) -> str:
    if case.get("ok") is True:
        return "green_control"
    if case.get("control_valid") is not True:
        return "invalid_control"
    return "red_defect"


def _speed_boundary(cases: Sequence[dict[str, Any]]) -> dict[str, Any]:
    baseline = _straight_fixed_cases(cases, tuned=False)
    passing = [case for case in baseline if case.get("ok") is True]
    failing = [case for case in baseline if case.get("ok") is not True]
    highest = max((float(case["speed_mps"]) for case in passing), default=None)
    first_failure_case = min(
        failing,
        key=lambda case: float(case.get("speed_mps") or 0.0),
        default=None,
    )
    first_failure = None if first_failure_case is None else float(first_failure_case["speed_mps"])
    state = "unknown"
    if highest is not None and first_failure is not None:
        state = "known_red_boundary" if first_failure > highest else "non_monotonic_or_unstable"
    elif highest is not None:
        state = "no_failure_observed"
    elif first_failure is not None:
        state = "no_passing_speed_observed"
    return {
        "state": state,
        "green_speed_mps": highest,
        "first_red_speed_mps": first_failure,
        "red_speeds_mps": sorted({float(case["speed_mps"]) for case in failing}),
        "minimal_red_defect": _minimal_red_defect(first_failure_case),
        "baseline_case_count": len(baseline),
        "baseline_passing_count": len(passing),
        "baseline_failing_count": len(failing),
    }


def _angular_rate_group(case_group: Sequence[dict[str, Any]]) -> dict[str, Any]:
    cases = sorted(case_group, key=lambda case: float(case.get("speed_mps") or 0.0))
    passing = [case for case in cases if case.get("ok") is True]
    failing = [case for case in cases if case.get("ok") is not True]
    first_failure_case = min(
        failing,
        key=lambda case: float(case.get("speed_mps") or 0.0),
        default=None,
    )
    exemplar = cases[0] if cases else {}
    return {
        "angular_z_radps": exemplar.get("angular_z_radps"),
        "abs_angular_z_radps": exemplar.get("abs_angular_z_radps"),
        "case_count": len(cases),
        "passing_count": len(passing),
        "failing_count": len(failing),
        "green_speed_mps": max((float(case["speed_mps"]) for case in passing), default=None),
        "first_red_speed_mps": None
        if first_failure_case is None
        else float(first_failure_case["speed_mps"]),
        "red_speeds_mps": sorted({float(case["speed_mps"]) for case in failing}),
        "minimal_red_defect": _minimal_red_defect(first_failure_case),
    }


def _turning_boundary(cases: Sequence[dict[str, Any]]) -> dict[str, Any]:
    baseline = _fixed_cases(cases, tuned=False)
    groups: dict[float, list[dict[str, Any]]] = {}
    for case in baseline:
        angular_z = float(case.get("angular_z_radps") or 0.0)
        groups.setdefault(round(angular_z, 6), []).append(case)
    angular_rate_groups = [
        _angular_rate_group(group)
        for _, group in sorted(groups.items(), key=lambda item: abs(item[0]))
    ]
    turning_cases = [
        case
        for case in baseline
        if abs(float(case.get("angular_z_radps") or 0.0)) > 1e-6
    ]
    turning_failures = [case for case in turning_cases if case.get("ok") is not True]
    turning_slam_failures = [
        case
        for case in turning_failures
        if _blocking_subsystem(case) == "slam_localization"
    ]
    first_turning_failure = min(
        turning_failures,
        key=lambda case: (
            float(case.get("speed_mps") or 0.0),
            float(case.get("abs_angular_z_radps") or 0.0),
        ),
        default=None,
    )
    first_turning_slam_failure = min(
        turning_slam_failures,
        key=lambda case: (
            float(case.get("speed_mps") or 0.0),
            float(case.get("abs_angular_z_radps") or 0.0),
        ),
        default=None,
    )
    return {
        "turning_case_count": len(turning_cases),
        "turning_passing_count": len([case for case in turning_cases if case.get("ok") is True]),
        "turning_failing_count": len(turning_failures),
        "turning_slam_failing_count": len(turning_slam_failures),
        "first_red_turning_case": _minimal_red_defect(first_turning_failure),
        "first_red_turning_slam_case": _minimal_red_defect(first_turning_slam_failure),
        "angular_rate_groups": angular_rate_groups,
    }


def _config_findings(cases: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    tuned_cases = _fixed_cases(cases, tuned=True)
    if not tuned_cases:
        return []
    best = sorted(
        tuned_cases,
        key=lambda case: (
            float(case.get("z_delta_error_m") or 999.0),
            float(case.get("yaw_delta_error_rad") or 999.0),
        ),
    )[0]
    return [
        {
            "config_label": best.get("config_label"),
            "speed_mps": best.get("speed_mps"),
            "classification": _classification(best),
            "changed_boundary": best.get("ok") is True,
            "z_delta_error_m": best.get("z_delta_error_m"),
            "path": best.get("path"),
            "recommendation": (
                "Continue scan timing / IMU / degeneracy diagnostics; "
                "current tuning does not clear the speed boundary."
            ),
        }
    ]


def evaluate_reports(report_paths: Sequence[Path | str]) -> dict[str, Any]:
    cases: list[dict[str, Any]] = []
    for path_like in report_paths:
        path = Path(path_like)
        try:
            cases.append(_case_from_report(path, _load_json(path)))
        except Exception as exc:  # pragma: no cover - CLI diagnostics path.
            cases.append(
                {
                    "path": str(path),
                    "ok": False,
                    "control_valid": False,
                    "blockers": [f"failed to load report: {exc}"],
                }
            )
    fixed_controls = _fixed_cases(cases)
    blockers: list[str] = []
    if not fixed_controls:
        blockers.append("no fixed-drive Fast-LIO controls provided")
    boundary = _speed_boundary(cases)
    turning_boundary = _turning_boundary(cases)
    config_findings = _config_findings(cases)
    boundary_characterized = bool(fixed_controls and not blockers)
    return {
        "schema_version": "lingtu.fastlio2_speed_config_boundary_gate.v1",
        "ok": False,
        "boundary_characterized": boundary_characterized,
        "gate_purpose": "diagnostic_boundary_only",
        "algorithm_pass": False,
        "claim_allowed": False,
        "simulation_only": True,
        "real_robot_motion": False,
        "cmd_vel_sent_to_hardware": False,
        "case_count": len(cases),
        "fixed_control_count": len(fixed_controls),
        "boundary": boundary,
        "turning_boundary": turning_boundary,
        "config_findings": config_findings,
        "blocking_subsystems": ["slam_localization"] if boundary_characterized else [],
        "recommended_next_step": (
            "Focus Fast-LIO speed/yaw-rate/lateral command sensitivity: scan timing, "
            "IMU assumptions, and motion-degeneracy diagnostics before claiming "
            "navigation health."
        ),
        "blockers": blockers,
        "cases": [
            {
                **case,
                "classification": _classification(case),
                "blocking_subsystem": _blocking_subsystem(case)
                if case.get("ok") is not True
                else "",
            }
            for case in cases
        ],
    }


def _discover_reports(reports: Sequence[Path], patterns: Sequence[str]) -> list[Path]:
    discovered: list[Path] = []
    seen: set[Path] = set()

    def add(path: Path) -> None:
        resolved = path if path.is_absolute() else ROOT / path
        if resolved.is_file() and resolved not in seen:
            seen.add(resolved)
            discovered.append(resolved)

    for report in reports:
        add(report)
    for pattern in patterns:
        glob_pattern = pattern if Path(pattern).is_absolute() else str(ROOT / pattern)
        for item in glob.glob(glob_pattern, recursive=True):
            add(Path(item))
    return sorted(discovered, key=lambda path: str(path))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Summarize fixed-drive Fast-LIO speed/config boundary reports."
    )
    parser.add_argument("--report", type=Path, action="append", default=[])
    parser.add_argument("--report-glob", action="append", default=[])
    parser.add_argument("--json-out", type=Path, default=None)
    parser.add_argument("--strict", action="store_true")
    return parser


def main() -> int:
    args = _build_parser().parse_args()
    summary = evaluate_reports(_discover_reports(args.report, args.report_glob))
    text = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True)
    print(text)
    if args.json_out:
        output = args.json_out if args.json_out.is_absolute() else ROOT / args.json_out
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(text + "\n", encoding="utf-8")
    return 0 if summary.get("boundary_characterized") or not args.strict else 1


if __name__ == "__main__":
    raise SystemExit(main())
