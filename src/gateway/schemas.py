"""FastAPI response schemas for app/web Gateway contracts."""

from __future__ import annotations

from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field


class GatewayResponseModel(BaseModel):
    """Base model that documents known fields without stripping future additions."""

    model_config = ConfigDict(extra="allow")


class GatewayErrorResponse(GatewayResponseModel):
    error: str
    message: str | None = None
    detail: Any = None


class BitrateRequest(BaseModel):
    bps: int


class ServerInfo(GatewayResponseModel):
    api_version: str
    time: float


class EndpointSpec(GatewayResponseModel):
    method: str
    path: str
    operation_id: str | None = None
    request_schema: str | None = None
    response_schema: str | None = None
    response_content_types: list[str] = Field(default_factory=list)
    status_codes: list[str] = Field(default_factory=list)


class TeleopSummary(GatewayResponseModel):
    active: bool
    clients: int


class ReadinessModuleStatus(GatewayResponseModel):
    ok: bool
    detail: dict[str, Any] | None = None
    error: str | None = None


class CommandReceipt(GatewayResponseModel):
    name: str
    request_id: str | None = None
    client_id: str
    accepted: bool
    replay: bool
    ts: float


class ReadinessResponse(GatewayResponseModel):
    schema_version: int
    status: str
    ready: bool
    modules: dict[str, ReadinessModuleStatus]
    module_count: int
    failed_modules: list[str]
    reasons: list[str]
    ts: float


class LivenessResponse(GatewayResponseModel):
    status: str
    ts: float


class DevicesResponse(GatewayResponseModel):
    devices: list[Any] = Field(default_factory=list)
    manager: str
    spec_count: int = 0
    opened_count: int = 0
    error: str | None = None


class HealthResponse(GatewayResponseModel):
    status: str
    modules_ok: int = 0
    modules_fail: int = 0
    gateway: dict[str, Any] = Field(default_factory=dict)
    teleop: TeleopSummary
    sensors: dict[str, Any] = Field(default_factory=dict)
    slam_hz: float = 0.0
    map_points: int = 0
    has_odom: bool = False
    modules: dict[str, str] = Field(default_factory=dict)
    brainstem: dict[str, Any] = Field(default_factory=dict)


class ControlCommandResponse(GatewayResponseModel):
    status: str
    command: CommandReceipt
    goal: list[float] | None = None
    instruction: str | None = None
    mode: str | None = None


class AuthLoginResponse(GatewayResponseModel):
    ok: bool
    message: str


class AuthCheckResponse(GatewayResponseModel):
    auth_required: bool


class LeaseResponse(GatewayResponseModel):
    status: str
    holder: str | None = None
    active: bool | None = None
    expires_in: float | None = None


class SceneGraphResponse(GatewayResponseModel):
    scene_graph: Any = None


class LocationEntry(GatewayResponseModel):
    name: str
    x: float
    y: float
    z: float


class LocationsResponse(GatewayResponseModel):
    locations: list[LocationEntry] = Field(default_factory=list)


class PathResponse(GatewayResponseModel):
    path: list[Any] = Field(default_factory=list)
    robot: Any = None


class SessionResponse(GatewayResponseModel):
    mode: str
    active_map: str | None = None
    map_has_pcd: bool = False
    map_has_tomogram: bool = False
    since: float | None = None
    pending: bool = False
    error: str = ""
    icp_quality: float | None = None
    localizer_ready: bool = False
    can_start_mapping: bool = False
    can_start_navigating: bool = False
    can_start_exploring: bool = False
    can_end: bool = False
    explorer_available: bool = False


class SessionTransitionResponse(GatewayResponseModel):
    success: bool
    session: SessionResponse | None = None
    message: str | None = None


class MapInfo(GatewayResponseModel):
    name: str
    has_pcd: bool = False
    has_tomogram: bool = False
    is_active: bool = False
    size_mb: float | None = None
    patch_count: int = 0


class MapListResponse(GatewayResponseModel):
    maps: list[MapInfo] = Field(default_factory=list)
    active: str = ""
    map_dir: str = ""


class MapLifecycleResponse(GatewayResponseModel):
    success: bool | None = None
    message: str | None = None
    name: str | None = None
    active: str | None = None
    old_name: str | None = None
    new_name: str | None = None
    path: str | None = None
    size: str | None = None
    restored_size: int | None = None
    replaced_backups_kept: int | None = None
    replaced_backups_pruned: int | None = None
    note: str | None = None
    errors: list[Any] | None = None
    dynamic_filter: Any = None
    maps: list[Any] | None = None


class MapPointsResponse(GatewayResponseModel):
    count: int
    layout: Literal["flat_xyz", "xyz_rows"] = "xyz_rows"
    points: list[float] | list[tuple[float, float, float]] = Field(
        default_factory=list
    )
    bounds: dict[str, list[float]] | None = None


class TemporalMemoryResponse(GatewayResponseModel):
    observations: list[Any] = Field(default_factory=list)
    count: int = 0


class ExplorationCommandResponse(GatewayResponseModel):
    status: Any = None


class ExplorationStatusResponse(GatewayResponseModel):
    available: bool
    exploring: bool = False
    frontier_count: int = 0


class SlamStatusResponse(GatewayResponseModel):
    mode: str
    services: dict[str, str] = Field(default_factory=dict)


class SlamOperationResponse(GatewayResponseModel):
    success: bool
    profile: str | None = None
    message: str | None = None
    quality: float | None = None


class BagOperationResponse(GatewayResponseModel):
    status: str | None = None
    path: str | None = None
    pid: int | None = None
    duration: int | None = None
    prefix: str | None = None
    error: str | None = None
    detail: Any = None


class BagStatusResponse(GatewayResponseModel):
    recording: bool
    path: str | None = None
    duration_s: float = 0.0
    size_bytes: int = 0
    pid: int | None = None
    exit_code: int | None = None
    disk_free: int = 0
    disk_total: int = 0


class WebRTCStatsResponse(GatewayResponseModel):
    enabled: bool = False
    active_peers: int = 0
    error: str | None = None


class Go2RTCStatusResponse(GatewayResponseModel):
    available: bool
    reason: str | None = None
    status: int | None = None
    streams: list[str] = Field(default_factory=list)


class WebRTCControlResponse(GatewayResponseModel):
    error: str | None = None
    enabled: bool | None = None
    sdp: str | None = None
    type: str | None = None
    bps: int | None = None


class StateResponse(GatewayResponseModel):
    schema_version: int
    server: ServerInfo
    odometry: Any = None
    safety: Any = None
    mission: Any = None
    eval: Any = None
    dialogue: Any = None
    mode: str
    lease: dict[str, Any]
    teleop: TeleopSummary
    session: dict[str, Any]
    localization: dict[str, Any]
    map: dict[str, Any]
    scene: dict[str, Any]
    path: dict[str, Any]
    links: dict[str, str]


class AppBootstrapResponse(GatewayResponseModel):
    schema_version: int
    server: ServerInfo
    robot: dict[str, Any]
    session: dict[str, Any]
    mission: dict[str, Any]
    safety: dict[str, Any]
    localization: dict[str, Any]
    control: dict[str, Any]
    map: dict[str, Any]
    scene: dict[str, Any]
    path: dict[str, Any]
    media: dict[str, Any]
    traffic: dict[str, Any]
    capabilities: dict[str, bool]
    capabilities_endpoint: str
    links: dict[str, str]


class AppCapabilitiesResponse(GatewayResponseModel):
    schema_version: int
    server: ServerInfo
    auth: dict[str, Any]
    features: dict[str, bool]
    endpoints: dict[str, dict[str, EndpointSpec]]
    probes: dict[str, EndpointSpec]
    realtime: dict[str, Any]
    client_policy: dict[str, Any]
    links: dict[str, str]
