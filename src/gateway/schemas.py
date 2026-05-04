"""FastAPI request/response schemas for app/web Gateway contracts."""

from __future__ import annotations

import time
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator


class GatewayResponseModel(BaseModel):
    """Base model that documents known fields without stripping future additions."""

    model_config = ConfigDict(extra="allow")


class CommandReceipt(GatewayResponseModel):
    name: str
    request_id: str | None = None
    client_id: str
    accepted: bool
    replay: bool
    ts: float


class GatewayErrorResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: Literal[False] = False
    error: str
    message: str | None = None
    detail: Any = None
    command: CommandReceipt | None = None


class BitrateRequest(BaseModel):
    bps: int


class GoalRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    instruction: str | None = None
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class ClickNavRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class PlanPreviewRequest(BaseModel):
    x: float
    y: float
    z: float = 0.0
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("x", "y", "z")
    @classmethod
    def finite(cls, v: float) -> float:
        import math

        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class CmdVelRequest(BaseModel):
    vx: float
    vy: float = 0.0
    wz: float
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("vx", "wz")
    @classmethod
    def finite(cls, v: float) -> float:
        import math

        if not math.isfinite(v):
            raise ValueError("must be finite")
        return v


class InstructionRequest(BaseModel):
    text: str = Field(min_length=1, max_length=1024)
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class StopRequest(BaseModel):
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)


class ModeRequest(BaseModel):
    mode: str
    request_id: str | None = Field(default=None, max_length=128)
    client_id: str = Field(default="unknown", max_length=128)

    @field_validator("mode")
    @classmethod
    def valid_mode(cls, v: str) -> str:
        if v not in ("manual", "autonomous", "estop"):
            raise ValueError(f"mode must be manual|autonomous|estop, got {v!r}")
        return v


class LeaseRequest(BaseModel):
    action: str
    client_id: str = Field(default="unknown", max_length=128)
    request_id: str | None = Field(default=None, max_length=128)
    ttl: float = Field(default=30.0, gt=0, le=3600)

    @field_validator("action")
    @classmethod
    def valid_action(cls, v: str) -> str:
        if v not in ("acquire", "release", "renew"):
            raise ValueError(f"action must be acquire|release|renew, got {v!r}")
        return v


class MapRequest(BaseModel):
    action: str
    name: str | None = None
    new_name: str | None = None

    @field_validator("action")
    @classmethod
    def valid_action(cls, v: str) -> str:
        allowed = {
            "list",
            "save",
            "use",
            "build",
            "delete",
            "rename",
            "set_active",
            "build_tomogram",
        }
        if v not in allowed:
            raise ValueError(f"action must be one of {allowed}")
        return v


class SessionStartRequest(BaseModel):
    mode: str | None = Field(default=None, max_length=32)
    map_name: str | None = Field(default=None, max_length=128)
    map: str | None = Field(default=None, max_length=128)
    slam_profile: str | None = Field(default=None, max_length=64)
    slam_backend: str | None = Field(default=None, max_length=64)


class MapNameRequest(BaseModel):
    name: str | None = Field(default=None, max_length=128)


class MapRenameRequest(BaseModel):
    old_name: str | None = Field(default=None, max_length=128)
    new_name: str | None = Field(default=None, max_length=128)


class MapSaveRequest(BaseModel):
    name: str | None = Field(default=None, max_length=128)


class WebRTCOfferRequest(BaseModel):
    model_config = ConfigDict(extra="allow")

    sdp: str | None = None
    type: str | None = None


class TemporalSemanticRequest(BaseModel):
    embedding: list[float] | None = None
    since: str | None = Field(default=None, max_length=64)
    top_k: int = Field(default=10, ge=1, le=1000)
    label: str | None = Field(default=None, max_length=128)


class SlamSwitchRequest(BaseModel):
    profile: str | None = Field(default=None, max_length=64)


class SlamRelocalizeRequest(BaseModel):
    map_name: str | None = Field(default=None, max_length=128)
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class BagStartRequest(BaseModel):
    duration: int = Field(default=600, ge=1, le=86400)
    prefix: str = Field(default="web", max_length=40)


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


class SSEEventEnvelope(GatewayResponseModel):
    schema_version: int = 1
    event_id: int | None = None
    type: str
    ts: float
    data: Any = None


class TeleopSummary(GatewayResponseModel):
    active: bool
    clients: int


class ReadinessModuleStatus(GatewayResponseModel):
    ok: bool
    detail: dict[str, Any] | None = None
    error: str | None = None


class ReadinessResponse(GatewayResponseModel):
    schema_version: int
    status: str
    ready: bool
    modules: dict[str, ReadinessModuleStatus]
    module_count: int
    failed_modules: list[str]
    reasons: list[str]
    runtime: dict[str, Any] = Field(default_factory=dict)
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
    schema_version: int = 1
    ok: bool = True
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
    schema_version: int = 1
    ok: bool = True
    status: str
    command: CommandReceipt
    holder: str | None = None
    active: bool | None = None
    expires_in: float | None = None


class SceneGraphObject(GatewayResponseModel):
    id: str | None = None
    label: str = ""
    x: float | None = None
    y: float | None = None
    z: float | None = None
    confidence: float | None = None
    distance: float | None = None
    bbox: Any = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class SceneGraphRelation(GatewayResponseModel):
    source: str | None = None
    target: str | None = None
    relation: str | None = None
    confidence: float | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class SceneGraphRegion(GatewayResponseModel):
    id: str | None = None
    name: str | None = None
    label: str | None = None
    x: float | None = None
    y: float | None = None
    z: float | None = None
    polygon: Any = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class SceneGraphResponse(GatewayResponseModel):
    schema_version: int = 1
    frame_id: str = "map"
    ts: float | None = None
    objects: list[SceneGraphObject] = Field(default_factory=list)
    relations: list[SceneGraphRelation] = Field(default_factory=list)
    regions: list[SceneGraphRegion] = Field(default_factory=list)
    count: int = 0
    scene_graph: Any = None


class LocationEntry(GatewayResponseModel):
    name: str
    x: float
    y: float
    z: float
    yaw: float | None = None
    tags: list[str] = Field(default_factory=list)
    source: str | None = None
    ts: float | None = None


class LocationsResponse(GatewayResponseModel):
    schema_version: int = 1
    locations: list[LocationEntry] = Field(default_factory=list)
    count: int = 0
    frame_id: str = "map"
    ts: float | None = None
    source: str = "tagged_locations"


class PathPoint(GatewayResponseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    frame_id: str | None = None
    ts: float | None = None
    metadata: dict[str, Any] = Field(default_factory=dict)


class RobotPoseSummary(GatewayResponseModel):
    x: float
    y: float
    z: float = 0.0
    yaw: float | None = None
    vx: float | None = None
    vy: float | None = None
    wz: float | None = None
    frame_id: str | None = None
    ts: float | None = None


class PathResponse(GatewayResponseModel):
    schema_version: int = 1
    path: list[PathPoint] = Field(default_factory=list)
    robot: RobotPoseSummary | None = None
    count: int = 0
    frame_id: str = "map"
    ts: float | None = None
    source: str = "gateway_cache"


class PlanPreviewResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool = True
    feasible: bool = False
    frame_id: str = "map"
    start: PathPoint | None = None
    goal: PathPoint
    adjusted_goal: PathPoint | None = None
    path: list[PathPoint] = Field(default_factory=list)
    count: int = 0
    distance_m: float | None = None
    plan_ms: float | None = None
    planner: str | None = None
    source: str = "navigation_preview"
    reasons: list[str] = Field(default_factory=list)
    error: str | None = None
    ts: float


class LocalizationStatusResponse(GatewayResponseModel):
    schema_version: int
    state: str
    ready: bool
    has_odometry: bool
    session_mode: str | None = None
    active_map: str | None = None
    icp_quality: float = 0.0
    reported_state: Any = None
    confidence: float | None = None
    algorithm_healthy: bool = False
    backend: str | None = None
    health_source: str | None = None
    pose_fresh: bool | None = None
    pose_freshness: str = "unknown"
    stale_odometry: bool = False
    odom_age_ms: float | None = None
    cloud_age_ms: float | None = None
    degeneracy: str | None = None
    icp_fitness: float | None = None
    effective_ratio: float | None = None
    condition_number: float | None = None
    degenerate_dof_count: int | None = None
    pos_cov_trace: float | None = None
    ieskf_iter_num: int | None = None
    ieskf_converged: bool | None = None
    map_cloud_fresh: bool | None = None
    map_state: str | None = None
    map_save_supported: bool | None = None
    map_save_source: str | None = None
    relocalization_supported: bool = True
    saved_map_relocalization_supported: bool | None = None
    restart_recovery_supported: bool | None = None
    recovery_method: str | None = None
    relocalization_state: str | None = None
    recovery_signal: str | None = None
    recovery_action: str | None = None
    localizer_health: str | None = None
    localizer_health_raw: str | None = None
    localizer_health_source: str | None = None
    localizer_health_topic_age_ms: float | None = None
    localizer_health_fitness: float | None = None
    localizer_health_iter: int | None = None
    localizer_health_cov_trace: float | None = None
    ts: float | None = None
    diag_received_ts: float | None = None
    diag_age_ms: float | None = None
    can_relocalize: bool = False
    reasons: list[str] = Field(default_factory=list)
    raw: dict[str, Any] = Field(default_factory=dict)


class NavigationPathSummary(GatewayResponseModel):
    points: int = 0
    endpoint: str


class NavigationControlActiveSource(GatewayResponseModel):
    name: str
    label: str
    category: str
    owner: str
    priority: int | None = None
    active: bool | None = None
    age_ms: int | None = None


class NavigationControlSummary(GatewayResponseModel):
    mode: str
    lease: dict[str, Any] = Field(default_factory=dict)
    active_cmd_source: str
    command_owner: str
    source_category: str
    manual_override: bool
    autonomy_requested: bool
    preempting_autonomy: bool
    mux_available: bool
    active_source: NavigationControlActiveSource
    sources: dict[str, Any] = Field(default_factory=dict)
    cmd_vel_mux: dict[str, Any] = Field(default_factory=dict)


class NavigationLocalizationSummary(GatewayResponseModel):
    state: str | None = None
    ready: bool | None = None
    degraded: bool = False
    algorithm_healthy: bool | None = None
    pose_fresh: bool | None = None
    pose_freshness: str | None = None
    speed_scale: float | None = None
    reasons: list[str] = Field(default_factory=list)


class NavigationReadinessSummary(GatewayResponseModel):
    can_accept_goal: bool
    can_execute_autonomy: bool
    blockers: list[str] = Field(default_factory=list)
    advisories: list[str] = Field(default_factory=list)
    localization_ready: bool
    control_owner: str


class NavigationProgressSummary(GatewayResponseModel):
    wp_index: int = 0
    wp_total: int = 0
    fraction: float = 0.0
    path_points: int = 0
    replan_count: int = 0
    active: bool = False
    terminal: bool = False


class NavigationDiagnosticsSummary(GatewayResponseModel):
    reason_codes: list[str] = Field(default_factory=list)
    failure_reason: str = ""
    localization_reasons: list[str] = Field(default_factory=list)
    cmd_vel_mux_available: bool = False


class NavigationMissionSummary(GatewayResponseModel):
    state: str
    raw: dict[str, Any] = Field(default_factory=dict)


class NavigationStatusResponse(GatewayResponseModel):
    schema_version: int
    state: str
    has_odometry: bool
    can_accept_goal: bool
    wp_index: int = 0
    wp_total: int = 0
    replan_count: int = 0
    speed_scale: float | None = None
    failure_reason: str = ""
    reason_codes: list[str] = Field(default_factory=list)
    readiness: NavigationReadinessSummary
    progress: NavigationProgressSummary
    path: NavigationPathSummary
    control: NavigationControlSummary
    localization: NavigationLocalizationSummary
    diagnostics: NavigationDiagnosticsSummary
    mission: NavigationMissionSummary
    ts: float


class SessionResponse(GatewayResponseModel):
    mode: str
    slam_profile: str = "stopped"
    localization_backend: str | None = None
    health_source: str | None = None
    active_map: str | None = None
    map_has_pcd: bool = False
    map_has_tomogram: bool = False
    since: float | None = None
    pending: bool = False
    error: str = ""
    icp_quality: float | None = None
    localizer_ready: bool = False
    localizer_algorithm_healthy: bool = False
    pose_fresh: bool | None = None
    pose_freshness: str = "unknown"
    map_state: str | None = None
    map_save_supported: bool = False
    map_save_source: str | None = None
    relocalization_supported: bool = True
    saved_map_relocalization_supported: bool | None = None
    restart_recovery_supported: bool | None = None
    recovery_method: str | None = None
    relocalization_state: str | None = None
    recovery_signal: str | None = None
    recovery_action: str | None = None
    can_start_mapping: bool = False
    can_start_navigating: bool = False
    can_start_exploring: bool = False
    can_end: bool = False
    explorer_available: bool = False
    explorer_unavailable_reason: str | None = None
    explorer_required_profile: str | None = None


class SessionTransitionResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    success: bool
    session: SessionResponse | None = None
    message: str | None = None
    ts: float = Field(default_factory=time.time)


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
    schema_version: int = 1
    ok: bool
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
    warnings: list[Any] | None = None
    slam_profile: str | None = None
    source: str | None = None
    map_save_source: str | None = None
    relocalization_supported: bool | None = None
    saved_map_relocalization_supported: bool | None = None
    restart_recovery_supported: bool | None = None
    recovery_method: str | None = None
    dynamic_filter: Any = None
    maps: list[Any] | None = None
    ts: float = Field(default_factory=time.time)


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
    reason: str | None = None
    required_profile: str | None = None
    action: str | None = None


class SlamStatusResponse(GatewayResponseModel):
    mode: str
    services: dict[str, str] = Field(default_factory=dict)


class SlamOperationResponse(GatewayResponseModel):
    schema_version: int = 1
    ok: bool
    success: bool
    profile: str | None = None
    message: str | None = None
    quality: float | None = None
    ts: float = Field(default_factory=time.time)


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


class ClientLinks(GatewayResponseModel):
    bootstrap: str | None = None
    capabilities: str | None = None
    traffic: str | None = None
    state: str | None = None
    scene_graph: str | None = None
    locations: str | None = None
    path: str | None = None
    localization_status: str | None = None
    navigation_status: str | None = None
    devices: str | None = None
    events: str | None = None
    teleop_ws: str | None = None
    camera_ws: str | None = None
    cloud_ws: str | None = None
    camera_snapshot: str | None = None
    webrtc_stats: str | None = None
    webrtc_offer: str | None = None
    webrtc_bitrate: str | None = None
    webrtc_whep: str | None = None
    go2rtc_status: str | None = None
    health: str | None = None
    session: str | None = None
    session_start: str | None = None
    session_end: str | None = None
    navigation_plan: str | None = None
    goal: str | None = None
    navigate_click: str | None = None
    stop: str | None = None
    instruction: str | None = None
    mode: str | None = None
    lease: str | None = None
    maps: str | None = None
    map_lifecycle: str | None = None
    map_activate: str | None = None
    map_rename: str | None = None
    map_save: str | None = None
    map_restore_predufo: str | None = None
    map_cloud_reset: str | None = None
    map_points: str | None = None
    saved_map_points: str | None = None
    explore_status: str | None = None
    explore_start: str | None = None
    explore_stop: str | None = None
    slam_status: str | None = None
    slam_switch: str | None = None
    slam_auto_relocalize: str | None = None
    slam_relocalize: str | None = None
    bag_start: str | None = None
    bag_stop: str | None = None
    bag_status: str | None = None
    memory_temporal: str | None = None
    memory_temporal_semantic: str | None = None
    diagnostic_pack: str | None = None


class AppMediaLinks(GatewayResponseModel):
    events: str
    teleop_ws: str
    camera_ws: str
    cloud_ws: str
    camera_snapshot: str
    webrtc_available: bool
    webrtc_offer: str | None = None


class RealtimeEventsCapability(GatewayResponseModel):
    path: str
    transport: Literal["sse"]
    initial_snapshot: bool
    heartbeat_s: float
    schema_version: int | None = None
    event_schema: str | None = None
    event_id_field: str | None = None
    timestamp_field: str | None = None
    heartbeat_type: str | None = None
    snapshot_type: str | None = None
    event_types: list[str] = Field(default_factory=list)
    diagnostic_event_types: list[str] = Field(default_factory=list)
    legacy_event_types: list[str] = Field(default_factory=list)
    named_events: bool | None = None
    browser_handler: str | None = None
    retry_ms: int | None = None
    replay_supported: bool | None = None
    last_event_id_header: str | None = None
    drop_policy: str | None = None
    large_event_policy: dict[str, Any] = Field(default_factory=dict)


class RealtimeTeleopCapability(GatewayResponseModel):
    path: str
    transport: Literal["websocket"]
    control_messages: list[str] = Field(default_factory=list)
    binary_camera_frames: bool
    legacy_camera_query: str | None = None


class RealtimeCameraCapability(GatewayResponseModel):
    path: str
    transport: Literal["websocket"]
    binary_camera_frames: bool
    explicit_subscription: bool


class RealtimeCloudCapability(GatewayResponseModel):
    path: str
    transport: Literal["websocket"]
    binary_point_cloud_frames: bool
    drop_policy: str | None = None


class AppRealtimeCapabilities(GatewayResponseModel):
    events: RealtimeEventsCapability
    teleop: RealtimeTeleopCapability
    camera: RealtimeCameraCapability
    cloud: RealtimeCloudCapability


class TrafficSSEStats(GatewayResponseModel):
    clients: int = 0
    queue_maxsize: int | None = None
    queue_depths: list[int] = Field(default_factory=list)
    max_depth_seen: int = 0
    latest_event_id: int = 0
    published_events: int = 0
    dropped_events: int = 0
    suppressed_events: dict[str, int] = Field(default_factory=dict)
    raster_min_interval_s: float | None = None
    slope_grid_inline: bool = False
    drop_policy: str | None = None


class TrafficCloudStats(GatewayResponseModel):
    clients: int = 0
    queue_maxsize: int | None = None
    queue_depths: list[int] = Field(default_factory=list)
    max_depth_seen: int = 0
    published_frames: int = 0
    dropped_frames: int = 0
    drop_policy: str | None = None
    latest_seq: int = 0


class AppTrafficResponse(GatewayResponseModel):
    schema_version: int
    ts: float
    server: ServerInfo
    status: Literal["ok", "degraded"]
    sse: TrafficSSEStats
    cloud: TrafficCloudStats
    recommended_client_rates_hz: dict[str, float] = Field(default_factory=dict)
    client_policy: dict[str, Any]
    warnings: list[str] = Field(default_factory=list)
    links: ClientLinks


class StateResponse(GatewayResponseModel):
    schema_version: int
    ts: float
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
    navigation: dict[str, Any] = Field(default_factory=dict)
    map: dict[str, Any]
    scene: dict[str, Any]
    path: dict[str, Any]
    links: ClientLinks


class AppBootstrapResponse(GatewayResponseModel):
    schema_version: int
    ts: float
    server: ServerInfo
    robot: dict[str, Any]
    session: dict[str, Any]
    mission: dict[str, Any]
    safety: dict[str, Any]
    localization: dict[str, Any]
    navigation: dict[str, Any]
    control: dict[str, Any]
    map: dict[str, Any]
    scene: dict[str, Any]
    path: dict[str, Any]
    media: AppMediaLinks
    traffic: dict[str, Any]
    capabilities: dict[str, bool]
    capabilities_endpoint: str
    links: ClientLinks


class AppCapabilitiesResponse(GatewayResponseModel):
    schema_version: int
    ts: float
    server: ServerInfo
    auth: dict[str, Any]
    features: dict[str, bool]
    endpoints: dict[str, dict[str, EndpointSpec]]
    probes: dict[str, EndpointSpec]
    realtime: AppRealtimeCapabilities
    client_policy: dict[str, Any]
    links: ClientLinks
