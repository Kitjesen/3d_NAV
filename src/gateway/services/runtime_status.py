"""App/Web runtime status builders for localization and navigation."""

from __future__ import annotations

import time
from collections.abc import Mapping
from typing import Any


LOCALIZATION_STATUS_SCHEMA_VERSION = 1
NAVIGATION_STATUS_SCHEMA_VERSION = 1

PATH_ENDPOINT = "/api/v1/path"

MISSION_ACTIVE_STATES = {"PLANNING", "EXECUTING", "PATROLLING"}
MISSION_TERMINAL_STATES = {"SUCCESS", "FAILED", "CANCELLED"}

CONTROL_SOURCE_META: dict[str, dict[str, Any]] = {
    "teleop": {
        "label": "Teleop joystick",
        "category": "manual",
        "owner": "teleop",
        "preempts_autonomy": True,
    },
    "visual_servo": {
        "label": "Visual servo",
        "category": "autonomy_assist",
        "owner": "visual_servo",
        "preempts_autonomy": True,
    },
    "recovery": {
        "label": "Navigation recovery",
        "category": "autonomy_recovery",
        "owner": "navigation",
        "preempts_autonomy": False,
    },
    "path_follower": {
        "label": "Path follower",
        "category": "autonomy",
        "owner": "navigation",
        "preempts_autonomy": False,
    },
}

TRACKING_STATES = {"TRACKING", "OK", "READY"}
LOST_STATES = {"LOST", "UNINIT", "UNINITIALIZED"}
DEGRADED_STATES = {"DEGRADED", "FALLBACK_GNSS_ONLY"}
BAD_DEGENERACY = {"MILD", "MODERATE", "SEVERE", "CRITICAL"}
GOOD_LOCALIZER_HEALTH = {
    "",
    "UNKNOWN",
    "LOCKED",
    "RECOVERED",
    "OK",
    "READY",
    "LIO_TRACKING",
    "LIO_RECOVERED",
}
POSE_FRESH_MAX_ODOM_AGE_MS = 2000.0


def _mapping(value: Any) -> dict[str, Any]:
    if isinstance(value, Mapping):
        return dict(value)
    return {}


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def _as_float(value: Any, default: float | None = None) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def _as_optional_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _as_optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "y"}:
            return True
        if lowered in {"false", "0", "no", "n"}:
            return False
    return None


def backend_capability_defaults(backend_name: str | None) -> dict[str, Any]:
    backend = str(backend_name or "").strip().lower()
    backend = {
        "super-lio": "super_lio",
        "superlio": "super_lio",
        "super_lio_reloc": "super_lio_relocation",
        "super-lio-reloc": "super_lio_relocation",
        "superlio-reloc": "super_lio_relocation",
        "super-lio-relocation": "super_lio_relocation",
        "superlio-relocation": "super_lio_relocation",
        "relocation": "super_lio_relocation",
    }.get(backend, backend)
    if backend == "super_lio":
        return {
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio",
        }
    if backend == "super_lio_relocation":
        return {
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_super_lio_relocation",
        }
    if backend == "localizer":
        return {
            "relocalization_supported": True,
            "saved_map_relocalization_supported": True,
            "restart_recovery_supported": True,
            "recovery_method": "relocalize_service",
        }
    if backend in {"fastlio2", "slam"}:
        return {
            "relocalization_supported": False,
            "saved_map_relocalization_supported": False,
            "restart_recovery_supported": True,
            "recovery_method": "restart_slam",
        }
    return {
        "relocalization_supported": True,
        "saved_map_relocalization_supported": True,
        "restart_recovery_supported": False,
        "recovery_method": "relocalize_service",
    }


def _reason_code_from_text(prefix: str, text: str) -> str:
    words: list[str] = []
    current: list[str] = []
    for ch in text.lower():
        if ch.isalnum():
            current.append(ch)
        elif current:
            words.append("".join(current))
            current = []
    if current:
        words.append("".join(current))
    if not words:
        return prefix
    return f"{prefix}_{'_'.join(words[:6])}"


def safe_session(gw: Any) -> dict[str, Any]:
    try:
        snapshot = gw._session_snapshot()
        if isinstance(snapshot, Mapping):
            return dict(snapshot)
    except Exception:
        pass
    return {
        "mode": getattr(gw, "_session_mode", "unknown"),
        "active_map": getattr(gw, "_session_map", None),
        "pending": bool(getattr(gw, "_session_pending", False)),
        "error": "session_snapshot_unavailable",
        "can_start_mapping": False,
        "can_start_navigating": False,
        "can_start_exploring": False,
        "can_end": False,
    }


def safe_lease(gw: Any) -> dict[str, Any]:
    lease = getattr(gw, "_lease", None)
    if hasattr(lease, "to_dict"):
        try:
            data = lease.to_dict()
            if isinstance(data, Mapping):
                return dict(data)
        except Exception:
            pass
    return {}


def _reported_state(raw: Any) -> str:
    state = str(raw or "").strip()
    return state.upper() if state else ""


def localizer_algorithm_healthy(
    diagnostics: Mapping[str, Any],
    icp_quality: float,
) -> bool:
    reported = _reported_state(diagnostics.get("state"))
    degeneracy = _reported_state(diagnostics.get("degeneracy"))
    localizer_health = _reported_state(diagnostics.get("localizer_health"))
    health_source = str(
        diagnostics.get("health_source")
        or diagnostics.get("localizer_health_source")
        or ""
    ).strip().lower()
    icp_fitness = _as_float(diagnostics.get("icp_fitness"), icp_quality)
    health_fitness = _as_float(diagnostics.get("localizer_health_fitness"), None)
    icp_ok = any(
        value is not None and 0.0 < value < 0.5
        for value in (icp_fitness, health_fitness)
    )
    pose_fresh, _ = _pose_freshness(diagnostics)
    cloud_fresh = _cloud_fresh(diagnostics)
    odom_cloud_ok = (
        health_source == "odom_map_cloud"
        and pose_fresh is not False
        and cloud_fresh
        and reported in {"", *TRACKING_STATES}
    )

    return (
        reported in {"", *TRACKING_STATES}
        and degeneracy not in BAD_DEGENERACY
        and localizer_health in GOOD_LOCALIZER_HEALTH
        and cloud_fresh
        and (icp_ok or odom_cloud_ok)
    )


def _localizer_algorithm_healthy(
    diagnostics: Mapping[str, Any],
    icp_quality: float,
) -> bool:
    return localizer_algorithm_healthy(diagnostics, icp_quality)


def classify_pose_freshness(diagnostics: Mapping[str, Any]) -> tuple[bool | None, str]:
    reported = _reported_state(diagnostics.get("state"))
    explicit = _as_optional_bool(diagnostics.get("pose_fresh"))
    odom_age_ms = _as_float(diagnostics.get("odom_age_ms"), None)
    confidence = diagnostics.get("confidence")

    if reported in LOST_STATES:
        return False, "lost"
    if explicit is not None:
        return explicit, "fresh" if explicit else "stale"
    if odom_age_ms is not None and odom_age_ms >= 0.0:
        fresh = odom_age_ms <= POSE_FRESH_MAX_ODOM_AGE_MS
        return fresh, "fresh" if fresh else "stale"
    if isinstance(confidence, (int, float)):
        return confidence >= 0.5, "fresh" if confidence >= 0.5 else "stale"
    return None, "unknown"


def _pose_freshness(diagnostics: Mapping[str, Any]) -> tuple[bool | None, str]:
    return classify_pose_freshness(diagnostics)


def _cloud_fresh(diagnostics: Mapping[str, Any]) -> bool:
    explicit = _as_optional_bool(diagnostics.get("map_cloud_fresh"))
    if explicit is not None:
        return explicit
    cloud_age_ms = _as_float(diagnostics.get("cloud_age_ms"), None)
    if cloud_age_ms is not None and cloud_age_ms >= 0.0:
        return cloud_age_ms <= POSE_FRESH_MAX_ODOM_AGE_MS
    return True


def _localization_state(
    odometry: Any,
    session: Mapping[str, Any],
    icp_quality: float,
    diagnostics: Mapping[str, Any],
) -> tuple[str, list[str]]:
    reasons: list[str] = []
    mode = str(session.get("mode", "unknown"))
    ready = bool(session.get("localizer_ready", False))
    pending = bool(session.get("pending", False))
    reported = _reported_state(diagnostics.get("state"))
    degeneracy = _reported_state(diagnostics.get("degeneracy"))
    localizer_health = _reported_state(diagnostics.get("localizer_health"))
    confidence = diagnostics.get("confidence")
    algorithm_healthy = _localizer_algorithm_healthy(diagnostics, icp_quality)
    pose_fresh, _ = _pose_freshness(diagnostics)

    if odometry is None:
        reasons.append("odometry_missing")
        return "no_odometry", reasons

    if "RELOCAL" in reported or (pending and mode == "navigating"):
        reasons.append("relocalization_pending")
        return "relocalizing", reasons

    if reported in LOST_STATES:
        reasons.append(f"reported_state:{reported.lower()}")
        return "lost", reasons

    if localizer_health == "LOST":
        reasons.append("localizer_health:lost")
        return "lost", reasons

    if (
        reported in DEGRADED_STATES
        or degeneracy in BAD_DEGENERACY
        or localizer_health == "DEGRADED"
        or pose_fresh is False
    ):
        if reported:
            reasons.append(f"reported_state:{reported.lower()}")
        if degeneracy and degeneracy != "NONE":
            reasons.append(f"degeneracy:{degeneracy.lower()}")
        if localizer_health == "DEGRADED":
            reasons.append("localizer_health:degraded")
        if pose_fresh is False:
            reasons.append("stale_odometry" if algorithm_healthy else "low_confidence")
        return "degraded", reasons

    if ready:
        return "ready", reasons

    if mode == "navigating":
        reasons.append("localizer_not_ready")
        return "initializing" if icp_quality <= 0.0 else "degraded", reasons

    if reported in TRACKING_STATES:
        return "tracking", reasons

    return "tracking", reasons


def build_localization_status_from_parts(
    odometry: Any,
    session: Mapping[str, Any],
    icp_quality: float,
    status: Any,
) -> dict[str, Any]:
    diagnostics = _mapping(status)
    diag_received_mono = _as_float(diagnostics.get("_gateway_received_mono"))
    diag_age_ms = (
        round(max(0.0, time.monotonic() - diag_received_mono) * 1000.0, 1)
        if diag_received_mono is not None
        else None
    )
    algorithm_healthy = _localizer_algorithm_healthy(diagnostics, float(icp_quality))
    pose_fresh, pose_freshness = _pose_freshness(diagnostics)
    state, reasons = _localization_state(
        odometry,
        session,
        float(icp_quality),
        diagnostics,
    )
    ready = state == "ready"
    backend = (
        diagnostics.get("backend")
        or diagnostics.get("slam_profile")
        or session.get("slam_profile")
    )
    backend_name = str(backend or "").strip().lower()
    capability_defaults = backend_capability_defaults(backend_name)
    relocalization_supported = _as_optional_bool(
        diagnostics.get("relocalization_supported")
    )
    if relocalization_supported is None:
        relocalization_supported = bool(
            capability_defaults["relocalization_supported"]
        )
    saved_map_relocalization_supported = _as_optional_bool(
        diagnostics.get("saved_map_relocalization_supported")
    )
    if saved_map_relocalization_supported is None:
        saved_map_relocalization_supported = relocalization_supported
    restart_recovery_supported = _as_optional_bool(
        diagnostics.get("restart_recovery_supported")
    )
    if restart_recovery_supported is None:
        restart_recovery_supported = bool(
            capability_defaults["restart_recovery_supported"]
        )
    recovery_method = diagnostics.get("recovery_method")
    if not recovery_method:
        recovery_method = capability_defaults["recovery_method"]
    map_save_supported = _as_optional_bool(diagnostics.get("map_save_supported"))
    return {
        "schema_version": LOCALIZATION_STATUS_SCHEMA_VERSION,
        "state": state,
        "ready": ready,
        "has_odometry": odometry is not None,
        "session_mode": session.get("mode"),
        "active_map": session.get("active_map"),
        "icp_quality": float(icp_quality),
        "reported_state": diagnostics.get("state"),
        "confidence": diagnostics.get("confidence"),
        "algorithm_healthy": algorithm_healthy,
        "backend": backend,
        "health_source": diagnostics.get("health_source"),
        "pose_fresh": pose_fresh,
        "pose_freshness": pose_freshness,
        "stale_odometry": pose_fresh is False and algorithm_healthy,
        "odom_age_ms": _as_float(diagnostics.get("odom_age_ms")),
        "cloud_age_ms": _as_float(diagnostics.get("cloud_age_ms")),
        "degeneracy": diagnostics.get("degeneracy"),
        "icp_fitness": _as_float(diagnostics.get("icp_fitness")),
        "effective_ratio": _as_float(diagnostics.get("effective_ratio")),
        "condition_number": _as_float(diagnostics.get("condition_number")),
        "degenerate_dof_count": _as_optional_int(
            diagnostics.get("degenerate_dof_count")
        ),
        "pos_cov_trace": _as_float(diagnostics.get("pos_cov_trace")),
        "ieskf_iter_num": _as_optional_int(diagnostics.get("ieskf_iter_num")),
        "ieskf_converged": _as_optional_bool(diagnostics.get("ieskf_converged")),
        "map_cloud_fresh": _as_optional_bool(diagnostics.get("map_cloud_fresh")),
        "map_state": diagnostics.get("map_state"),
        "map_save_supported": map_save_supported,
        "map_save_source": diagnostics.get("map_save_source"),
        "relocalization_supported": relocalization_supported,
        "saved_map_relocalization_supported": saved_map_relocalization_supported,
        "restart_recovery_supported": restart_recovery_supported,
        "recovery_method": recovery_method,
        "relocalization_state": diagnostics.get("relocalization_state"),
        "recovery_signal": diagnostics.get("recovery_signal"),
        "recovery_action": diagnostics.get("recovery_action"),
        "localizer_health": diagnostics.get("localizer_health"),
        "localizer_health_raw": diagnostics.get("localizer_health_raw"),
        "localizer_health_source": diagnostics.get("localizer_health_source"),
        "localizer_health_topic_age_ms": _as_float(
            diagnostics.get("localizer_health_topic_age_ms")
        ),
        "localizer_health_fitness": _as_float(
            diagnostics.get("localizer_health_fitness")
        ),
        "localizer_health_iter": _as_optional_int(
            diagnostics.get("localizer_health_iter")
        ),
        "localizer_health_cov_trace": _as_float(
            diagnostics.get("localizer_health_cov_trace")
        ),
        "ts": diagnostics.get("ts"),
        "diag_received_ts": _as_float(diagnostics.get("_gateway_received_ts")),
        "diag_age_ms": diag_age_ms,
        "can_relocalize": (
            saved_map_relocalization_supported
            and state in {"degraded", "lost"}
            and odometry is not None
        ),
        "reasons": reasons,
        "raw": diagnostics,
    }


def build_localization_status(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        odometry = gw._odom
        localization_status = getattr(gw, "_localization_status", None)

    session = safe_session(gw)
    return build_localization_status_from_parts(
        odometry,
        session,
        float(getattr(gw, "_icp_quality", 0.0)),
        localization_status,
    )


def _cmd_vel_health(gw: Any) -> dict[str, Any]:
    modules = getattr(gw, "_all_modules", None) or {}
    mux = modules.get("CmdVelMux")
    if mux is None:
        return {"active_source": "unknown", "sources": {}, "available": False}
    try:
        health = mux.health() if hasattr(mux, "health") else {}
        if isinstance(health, Mapping):
            data = dict(health)
            data["available"] = True
            data.setdefault("active_source", "unknown")
            data.setdefault("sources", {})
            return data
    except Exception as exc:
        return {
            "active_source": "unknown",
            "sources": {},
            "available": False,
            "error": str(exc),
        }
    return {"active_source": "unknown", "sources": {}, "available": False}


def _control_summary(
    *,
    mode: str,
    lease: Mapping[str, Any],
    mission_state: str,
    cmd_vel: Mapping[str, Any],
) -> dict[str, Any]:
    active_source = str(cmd_vel.get("active_source") or "none")
    sources = _mapping(cmd_vel.get("sources"))
    active_source_health = _mapping(sources.get(active_source))
    meta = CONTROL_SOURCE_META.get(active_source, {})
    if active_source == "none":
        meta = {
            "label": "No active command source",
            "category": "none",
            "owner": "none",
            "preempts_autonomy": False,
        }
    elif not meta:
        meta = {
            "label": active_source,
            "category": "unknown",
            "owner": "unknown",
            "preempts_autonomy": False,
        }

    autonomy_requested = mission_state in MISSION_ACTIVE_STATES
    manual_override = active_source == "teleop"
    preempting_autonomy = bool(
        autonomy_requested and meta.get("preempts_autonomy", False)
    )

    return {
        "mode": mode,
        "lease": dict(lease),
        "active_cmd_source": active_source,
        "command_owner": meta["owner"],
        "source_category": meta["category"],
        "manual_override": manual_override,
        "autonomy_requested": autonomy_requested,
        "preempting_autonomy": preempting_autonomy,
        "mux_available": bool(cmd_vel.get("available", False)),
        "active_source": {
            "name": active_source,
            "label": meta["label"],
            "category": meta["category"],
            "owner": meta["owner"],
            "priority": active_source_health.get("priority"),
            "active": active_source_health.get("active", active_source != "none"),
            "age_ms": active_source_health.get("age_ms"),
        },
        "sources": sources,
        "cmd_vel_mux": dict(cmd_vel),
    }


def _progress_summary(
    *,
    state: str,
    wp_index: int,
    wp_total: int,
    path_points: int,
    replan_count: int,
) -> dict[str, Any]:
    if state == "SUCCESS":
        fraction = 1.0
    elif wp_total > 0:
        fraction = max(0.0, min(1.0, wp_index / wp_total))
    else:
        fraction = 0.0

    return {
        "wp_index": wp_index,
        "wp_total": wp_total,
        "fraction": round(fraction, 4),
        "path_points": path_points,
        "replan_count": replan_count,
        "active": state in MISSION_ACTIVE_STATES,
        "terminal": state in MISSION_TERMINAL_STATES,
    }


def _navigation_reason_codes(
    *,
    state: str,
    failure_reason: str,
    has_odometry: bool,
    mode: str,
    session: Mapping[str, Any],
    localization: Mapping[str, Any],
    control: Mapping[str, Any],
) -> list[str]:
    codes: list[str] = []
    if not has_odometry:
        codes.append("odometry_missing")
    if mode == "estop":
        codes.append("estop_active")
    if bool(session.get("pending", False)):
        codes.append("session_transition_pending")

    localization_state = str(localization.get("state") or "unknown")
    if localization_state in {"degraded", "lost", "relocalizing", "initializing"}:
        codes.append(f"localization_{localization_state}")
    if (
        localization.get("pose_fresh") is False
        and bool(localization.get("algorithm_healthy", False))
    ):
        codes.append("pose_stale")

    if state == "STUCK":
        codes.append("mission_stuck")
    elif state == "FAILED":
        codes.append("mission_failed")
    elif state == "CANCELLED":
        codes.append("mission_cancelled")

    if failure_reason:
        codes.append(_reason_code_from_text("failure", failure_reason))

    if control.get("preempting_autonomy"):
        codes.append(f"control_preempted_by_{control.get('active_cmd_source')}")
    if not control.get("mux_available", False):
        codes.append("cmd_vel_mux_unavailable")

    return list(dict.fromkeys(codes))


_NAVIGATION_BLOCKER_CODES = {
    "odometry_missing",
    "estop_active",
    "session_transition_pending",
    "localization_lost",
    "localization_relocalizing",
    "localization_initializing",
    "pose_stale",
}


def _navigation_blockers(reason_codes: list[str]) -> list[str]:
    return [code for code in reason_codes if code in _NAVIGATION_BLOCKER_CODES]


def _readiness_summary(
    *,
    can_accept_goal: bool,
    reason_codes: list[str],
    localization: Mapping[str, Any],
    control: Mapping[str, Any],
) -> dict[str, Any]:
    blockers = _navigation_blockers(reason_codes)
    advisories = [code for code in reason_codes if code not in blockers]
    return {
        "can_accept_goal": can_accept_goal,
        "can_execute_autonomy": not blockers,
        "blockers": blockers,
        "advisories": advisories,
        "localization_ready": bool(localization.get("ready", False)),
        "control_owner": control.get("command_owner", "unknown"),
    }


def build_navigation_status(gw: Any) -> dict[str, Any]:
    with gw._state_lock:
        mission = _mapping(gw._mission)
        odometry = gw._odom
        path_len = len(gw._last_path)
        mode = gw._mode

    session = safe_session(gw)
    lease = safe_lease(gw)
    localization = build_localization_status(gw)
    cmd_vel = _cmd_vel_health(gw)
    state = str(mission.get("state", "IDLE"))
    wp_index = _as_int(mission.get("wp_index"), 0)
    wp_total = _as_int(mission.get("wp_total"), 0)
    replan_count = _as_int(mission.get("replan_count"), 0)
    speed_scale = _as_float(mission.get("speed_scale"), None)
    failure_reason = str(mission.get("failure_reason", "") or "")
    degraded = localization.get("state") in {
        "no_odometry",
        "initializing",
        "degraded",
        "lost",
        "relocalizing",
    }
    base_can_accept_goal = (
        mode != "estop"
        and odometry is not None
        and not bool(session.get("pending", False))
    )
    control = _control_summary(
        mode=mode,
        lease=lease,
        mission_state=state,
        cmd_vel=cmd_vel,
    )
    progress = _progress_summary(
        state=state,
        wp_index=wp_index,
        wp_total=wp_total,
        path_points=path_len,
        replan_count=replan_count,
    )
    reason_codes = _navigation_reason_codes(
        state=state,
        failure_reason=failure_reason,
        has_odometry=odometry is not None,
        mode=mode,
        session=session,
        localization=localization,
        control=control,
    )
    can_accept_goal = base_can_accept_goal and not _navigation_blockers(reason_codes)
    readiness = _readiness_summary(
        can_accept_goal=can_accept_goal,
        reason_codes=reason_codes,
        localization=localization,
        control=control,
    )

    return {
        "schema_version": NAVIGATION_STATUS_SCHEMA_VERSION,
        "state": state,
        "has_odometry": odometry is not None,
        "can_accept_goal": can_accept_goal,
        "wp_index": wp_index,
        "wp_total": wp_total,
        "replan_count": replan_count,
        "speed_scale": speed_scale,
        "failure_reason": failure_reason,
        "reason_codes": reason_codes,
        "readiness": readiness,
        "progress": progress,
        "path": {
            "points": path_len,
            "endpoint": PATH_ENDPOINT,
        },
        "control": control,
        "localization": {
            "state": localization.get("state"),
            "ready": localization.get("ready"),
            "degraded": degraded,
            "algorithm_healthy": localization.get("algorithm_healthy"),
            "pose_fresh": localization.get("pose_fresh"),
            "pose_freshness": localization.get("pose_freshness"),
            "speed_scale": speed_scale,
            "reasons": localization.get("reasons", []),
        },
        "diagnostics": {
            "reason_codes": reason_codes,
            "failure_reason": failure_reason,
            "localization_reasons": localization.get("reasons", []),
            "cmd_vel_mux_available": control.get("mux_available", False),
        },
        "mission": {
            "state": state,
            "raw": mission,
        },
        "ts": float(mission.get("ts", time.time()) or time.time()),
    }
