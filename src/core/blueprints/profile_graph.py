"""Profile graph compilation helpers.

This is the first engineering boundary around profiles: a profile can be
compiled into a stable module/wire graph without constructing runtime modules.
The graph is intentionally read-only and snapshot-friendly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from cli.profiles_data import PROFILES, ROBOT_PRESETS
from core.blueprint import Blueprint
from core.blueprints.full_stack import full_stack_blueprint
from core.blueprints.runtime_endpoint import (
    apply_runtime_endpoint_config,
    runtime_endpoint_robot_preset,
)


SIMULATION_PROFILES = (
    "stub",
    "dev",
    "sim",
    "sim_mujoco_live",
    "sim_gazebo",
    "sim_industrial",
    "sim_cmu_tare",
    "sim_nav",
)

PRODUCT_PROFILES = (
    "map",
    "nav",
    "explore",
    "tare_explore",
    "super_lio",
    "super_lio_relocation",
)

# Offline graph snapshots exclude profiles that require a locally built native
# external binary at blueprint construction time. They remain product profiles
# and are checked through profile-level contracts.
OPTIONAL_NATIVE_PRODUCT_PROFILES = ("tare_explore",)

PROFILE_SNAPSHOT_TARGETS = (
    *SIMULATION_PROFILES,
    *(profile for profile in PRODUCT_PROFILES if profile not in OPTIONAL_NATIVE_PRODUCT_PROFILES),
)


@dataclass(frozen=True, order=True)
class WireEdge:
    """A directed edge in the profile dependency graph between two module ports."""
    out_module: str
    out_port: str
    in_module: str
    in_port: str
    transport: str | None = None

    @classmethod
    def from_blueprint_spec(cls, spec: Any) -> "WireEdge":
        transport = getattr(spec, "transport", None)
        if transport is not None:
            transport = getattr(transport, "__class__", type(transport)).__name__
        return cls(
            out_module=spec.out_module,
            out_port=spec.out_port,
            in_module=spec.in_module,
            in_port=spec.in_port,
            transport=transport,
        )

    def as_snapshot(self) -> str:
        wire = f"{self.out_module}.{self.out_port}->{self.in_module}.{self.in_port}"
        if self.transport:
            wire = f"{wire}[{self.transport}]"
        return wire


@dataclass(frozen=True)
class ProfileGraph:
    """DAG representation of a runtime profile: which modules and explicit wires it includes."""
    profile: str
    modules: tuple[str, ...]
    explicit_wires: tuple[WireEdge, ...]

    def as_snapshot(self) -> dict[str, list[str]]:
        return {
            "modules": sorted(self.modules),
            "explicit_wires": sorted(wire.as_snapshot() for wire in self.explicit_wires),
        }

    def dangling_wires(self) -> tuple[WireEdge, ...]:
        module_set = set(self.modules)
        return tuple(
            wire
            for wire in self.explicit_wires
            if wire.out_module not in module_set or wire.in_module not in module_set
        )


def resolve_profile_config(
    profile: str,
    *,
    runtime_endpoint: str | None = None,
    **overrides: Any,
) -> dict[str, Any]:
    """Return the full full_stack_blueprint kwargs for a CLI profile."""

    if profile not in PROFILES:
        raise KeyError(f"unknown profile: {profile}")
    profile_data = PROFILES[profile]
    if runtime_endpoint:
        preset_name = runtime_endpoint_robot_preset(profile, runtime_endpoint)
    else:
        preset_name = profile_data.get("_default_robot", "stub")
    if preset_name not in ROBOT_PRESETS:
        raise KeyError(f"unknown robot preset for profile {profile}: {preset_name}")

    config = dict(ROBOT_PRESETS[preset_name])
    config.update({k: v for k, v in profile_data.items() if not k.startswith("_")})
    if runtime_endpoint:
        config = apply_runtime_endpoint_config(profile, config, runtime_endpoint)
    config.update(overrides)
    return config


def blueprint_for_profile(
    profile: str,
    *,
    run_startup_checks: bool = False,
    manage_external_services: bool = False,
    **overrides: Any,
) -> Blueprint:
    """Compile a profile into a Blueprint without starting modules."""

    config = resolve_profile_config(profile, **overrides)
    config["run_startup_checks"] = run_startup_checks
    config["manage_external_services"] = manage_external_services
    return full_stack_blueprint(**config)


def graph_for_profile(
    profile: str,
    *,
    run_startup_checks: bool = False,
    manage_external_services: bool = False,
    **overrides: Any,
) -> ProfileGraph:
    """Compile a profile into a stable module/wire graph."""

    bp = blueprint_for_profile(
        profile,
        run_startup_checks=run_startup_checks,
        manage_external_services=manage_external_services,
        **overrides,
    )
    modules = tuple(entry.name for entry in bp._entries)
    wires = tuple(WireEdge.from_blueprint_spec(spec) for spec in bp._wires)
    return ProfileGraph(profile=profile, modules=modules, explicit_wires=wires)


def snapshot_profile_graphs(
    profiles: tuple[str, ...] = PROFILE_SNAPSHOT_TARGETS,
) -> dict[str, dict[str, list[str]]]:
    return {profile: graph_for_profile(profile).as_snapshot() for profile in profiles}
