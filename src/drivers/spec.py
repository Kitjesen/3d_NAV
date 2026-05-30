"""Driver / sensor-source port contracts.

NAV COMPUTE CONTRACT companion (docs/architecture/NAVIGATION_COMPUTE_CONTRACT.md
and docs/superpowers/plans/2026-05-30-repo-structure-redesign.md §4).

Why this file exists
--------------------
LingTu motion drivers (real `thunder`, sim `mujoco`/`ros2`, `stub`) are
pluggable backends resolved via ``core.registry.get("driver", name)``.
Historically the "what ports must a robot motion driver expose" contract lived
only as duck-typed annotations scattered across each module, so there was no
single place to answer "is the sim driver shaped like the real driver?".

This module makes the contract explicit (dimos-style ``spec`` Protocols) and
machine-checkable, so sim/real/stub backends can be verified to share one shape.

Contract tiers
--------------
* MotionDriver (REQUIRED for every profile-selectable motion driver):
    In  : cmd_vel: In[Twist], stop_signal: In[int]
    Out : odometry: Out[Odometry]
* CameraSource (OPTIONAL capability):
    Out : (camera_image | color_image): Out[Image], depth_image, camera_info
* PointcloudSource (OPTIONAL capability):
    Out : map_cloud: Out[PointCloud2]

A backend is a "full sensor driver" (mujoco / ros2_sim) when it satisfies
MotionDriver + CameraSource + PointcloudSource. A "minimal driver" (stub /
thunder) satisfies only MotionDriver and delegates sensors to separate
camera-bridge / lidar modules.
"""

from __future__ import annotations

from typing import Any, Protocol, get_origin, runtime_checkable

from core.stream import In, Out

# ── Declarative port contract ─────────────────────────────────────────────

#: Required input ports every driver backend must declare.
REQUIRED_INPUTS: frozenset[str] = frozenset({"cmd_vel", "stop_signal"})
#: Required output ports every driver backend must declare.
REQUIRED_OUTPUTS: frozenset[str] = frozenset({"odometry"})

#: A camera source must expose one color stream plus depth + intrinsics.
CAMERA_COLOR_OUTPUTS: frozenset[str] = frozenset({"camera_image", "color_image"})
CAMERA_EXTRA_OUTPUTS: frozenset[str] = frozenset({"depth_image", "camera_info"})

#: A pointcloud source must expose the map cloud feeding maps/terrain.
POINTCLOUD_OUTPUTS: frozenset[str] = frozenset({"map_cloud"})


# ── Protocols (documentation + optional runtime isinstance) ────────────────


@runtime_checkable
class MotionDriver(Protocol):
    """Minimal driver contract: take velocity/stop, emit odometry."""

    cmd_vel: Any
    stop_signal: Any
    odometry: Any


@runtime_checkable
class PointcloudSource(Protocol):
    map_cloud: Any


@runtime_checkable
class CameraSource(Protocol):
    depth_image: Any
    camera_info: Any


# ── Port introspection ─────────────────────────────────────────────────────


def _port_direction(annotation: Any) -> str | None:
    """Return 'in' / 'out' / None for an In[...] / Out[...] port annotation.

    LingTu modules use ``from __future__ import annotations``, so class
    ``__annotations__`` hold *strings* like ``"In[Twist]"`` rather than the
    real generic alias. Handle both the stringized and the live-generic form.
    """
    if isinstance(annotation, str):
        text = annotation.strip()
        if text.startswith("In[") or text == "In":
            return "in"
        if text.startswith("Out[") or text == "Out":
            return "out"
        return None
    origin = get_origin(annotation)
    if origin is In:
        return "in"
    if origin is Out:
        return "out"
    return None


def collect_ports(module_cls: type) -> tuple[dict[str, Any], dict[str, Any]]:
    """Return (inputs, outputs) port-name -> annotation for a Module class.

    Walks the full MRO so subclasses inherit base-declared ports. Distinguishes
    direction via the In[...] / Out[...] annotation (string or live generic).
    """
    inputs: dict[str, Any] = {}
    outputs: dict[str, Any] = {}
    for cls in reversed(module_cls.__mro__):
        for name, annotation in getattr(cls, "__annotations__", {}).items():
            direction = _port_direction(annotation)
            if direction == "in":
                inputs[name] = annotation
            elif direction == "out":
                outputs[name] = annotation
    return inputs, outputs


def driver_contract_issues(module_cls: type) -> list[str]:
    """Return a list of contract violations; empty list means the class is a
    valid MotionDriver."""
    inputs, outputs = collect_ports(module_cls)
    issues: list[str] = []
    for port in sorted(REQUIRED_INPUTS):
        if port not in inputs:
            issues.append(f"missing required In port '{port}'")
    for port in sorted(REQUIRED_OUTPUTS):
        if port not in outputs:
            issues.append(f"missing required Out port '{port}'")
    return issues


def is_motion_driver(module_cls: type) -> bool:
    return not driver_contract_issues(module_cls)


def is_camera_source(module_cls: type) -> bool:
    _inputs, outputs = collect_ports(module_cls)
    has_color = bool(CAMERA_COLOR_OUTPUTS & outputs.keys())
    has_extra = CAMERA_EXTRA_OUTPUTS <= outputs.keys()
    return has_color and has_extra


def is_pointcloud_source(module_cls: type) -> bool:
    _inputs, outputs = collect_ports(module_cls)
    return POINTCLOUD_OUTPUTS <= outputs.keys()


def driver_capabilities(module_cls: type) -> dict[str, Any]:
    """Summarise a driver backend's contract compliance + optional capabilities."""
    return {
        "motion_driver": is_motion_driver(module_cls),
        "issues": driver_contract_issues(module_cls),
        "camera_source": is_camera_source(module_cls),
        "pointcloud_source": is_pointcloud_source(module_cls),
    }
