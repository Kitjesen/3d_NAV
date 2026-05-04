// Shared TypeScript interfaces for LingTu web dashboard

export interface MapInfo {
  name: string
  has_pcd: boolean
  has_tomogram: boolean
  is_active: boolean
  size_mb?: number
  patch_count?: number
}

export interface MapListResponse {
  schema_version: number
  maps: MapInfo[]
  count: number
  active: string
  map_dir: string
  ts: number
}

export interface MapPointsResponse {
  schema_version: number
  count: number
  layout: 'flat_xyz' | 'xyz_rows'
  frame_id: string
  source: string
  name?: string | null
  points: number[] | Array<[number, number, number]>
  bounds?: Record<string, number[]> | null
  ts: number
}

export interface ServerInfo {
  api_version: string
  time: number
}

export interface EndpointSpec {
  method: string
  path: string
  operation_id?: string | null
  request_schema?: string | null
  response_schema?: string | null
  response_content_types: string[]
  status_codes: string[]
}

export interface SceneGraphObject {
  id?: string | null
  label: string
  x?: number | null
  y?: number | null
  z?: number | null
  confidence?: number | null
  distance?: number | null
  bbox?: unknown
  metadata: Record<string, unknown>
}

export interface SceneGraphRelation {
  source?: string | null
  target?: string | null
  relation?: string | null
  confidence?: number | null
  metadata: Record<string, unknown>
}

export interface SceneGraphRegion {
  id?: string | null
  name?: string | null
  label?: string | null
  x?: number | null
  y?: number | null
  z?: number | null
  polygon?: unknown
  metadata: Record<string, unknown>
}

export interface SceneGraphResponse {
  schema_version: number
  frame_id: string
  ts?: number | null
  objects: SceneGraphObject[]
  relations: SceneGraphRelation[]
  regions: SceneGraphRegion[]
  count: number
  scene_graph?: unknown
}

export interface LocationEntry {
  name: string
  x: number
  y: number
  z: number
  yaw?: number | null
  tags: string[]
  source?: string | null
  ts?: number | null
}

export interface LocationsResponse {
  schema_version: number
  locations: LocationEntry[]
  count: number
  frame_id: string
  ts?: number | null
  source: string
}

export interface StateResponse {
  schema_version: number
  ts: number
  server?: ServerInfo
  odometry?: Record<string, unknown> | null
  safety?: Record<string, unknown> | null
  mission?: Record<string, unknown> | null
  eval?: Record<string, unknown> | null
  dialogue?: Record<string, unknown> | null
  mode?: string | null
  lease?: Record<string, unknown> | null
  teleop?: Record<string, unknown> | null
  session?: SessionEvent['data'] | Record<string, unknown> | null
  localization?: Record<string, unknown> | null
  navigation?: Record<string, unknown> | null
  map?: Record<string, unknown> | null
  scene?: Record<string, unknown> | null
  path?: Record<string, unknown> | null
  links?: Record<string, string>
}

export interface HealthResponse {
  status: string
  modules_ok: number
  modules_fail: number
  gateway: Record<string, unknown>
  teleop: {
    active: boolean
    clients: number
  }
  sensors: Record<string, unknown>
  slam_hz: number
  map_points: number
  has_odom: boolean
  modules: Record<string, string>
  brainstem: Record<string, unknown>
  [key: string]: unknown
}

export interface DeviceEntry {
  [key: string]: unknown
}

export interface DevicesResponse {
  devices: DeviceEntry[]
  manager: string
  spec_count: number
  opened_count: number
  error?: string | null
  [key: string]: unknown
}

export interface AuthLoginResponse {
  ok: boolean
  message?: string | null
  [key: string]: unknown
}

export interface AuthCheckResponse {
  auth_required: boolean
  [key: string]: unknown
}

export interface RobotPoseSummary {
  x: number
  y: number
  z: number
  yaw?: number | null
  vx?: number | null
  vy?: number | null
  wz?: number | null
  frame_id?: string | null
  ts?: number | null
}

export interface PathResponse {
  schema_version: number
  path: PathPoint[]
  robot: RobotPoseSummary | null
  count: number
  frame_id: string
  ts?: number | null
  source: string
}

export interface PlanPreviewRequest {
  x: number
  y: number
  z?: number
  client_id?: string
}

export interface PlanPreviewResponse {
  schema_version: number
  ok: boolean
  feasible: boolean
  frame_id: string
  start: PathPoint | null
  goal: PathPoint
  adjusted_goal?: PathPoint | null
  path: PathPoint[]
  count: number
  distance_m?: number | null
  plan_ms?: number | null
  planner?: string | null
  source: string
  reasons: string[]
  error?: string | null
  ts: number
}

export interface ClientLinks {
  bootstrap?: string
  capabilities?: string
  traffic?: string
  state?: string
  scene_graph?: string
  locations?: string
  path?: string
  localization_status?: string
  navigation_status?: string
  devices?: string
  auth_login?: string
  auth_check?: string
  events?: string
  teleop_ws?: string
  camera_ws?: string
  cloud_ws?: string
  camera_snapshot?: string
  webrtc_stats?: string
  webrtc_offer?: string
  webrtc_bitrate?: string
  webrtc_whep?: string
  go2rtc_status?: string
  health?: string
  session?: string
  session_start?: string
  session_end?: string
  navigation_plan?: string
  goal?: string
  navigate_click?: string
  stop?: string
  instruction?: string
  mode?: string
  lease?: string
  maps?: string
  map_lifecycle?: string
  map_activate?: string
  map_rename?: string
  map_save?: string
  map_restore_predufo?: string
  map_cloud_reset?: string
  map_points?: string
  saved_map_points?: string
  explore_status?: string
  explore_start?: string
  explore_stop?: string
  slam_status?: string
  slam_switch?: string
  slam_auto_relocalize?: string
  slam_relocalize?: string
  bag_start?: string
  bag_stop?: string
  bag_status?: string
  memory_temporal?: string
  memory_temporal_semantic?: string
  diagnostic_pack?: string
  [key: string]: string | undefined
}

export interface AppMediaLinks {
  events: string
  teleop_ws: string
  camera_ws: string
  cloud_ws: string
  camera_snapshot: string
  webrtc_available: boolean
  webrtc_stats?: string
  webrtc_offer?: string
  webrtc_bitrate?: string
  webrtc_whep?: string
  go2rtc_status?: string
  [key: string]: unknown
}

export interface RealtimeEventsCapability {
  path: string
  transport: 'sse'
  initial_snapshot: boolean
  heartbeat_s: number
  schema_version?: number
  event_schema?: string
  event_id_field?: string
  timestamp_field?: string
  heartbeat_type?: string
  snapshot_type?: string
  event_types?: string[]
  diagnostic_event_types?: string[]
  legacy_event_types?: string[]
  named_events?: boolean
  browser_handler?: string
  retry_ms?: number
  replay_supported?: boolean
  last_event_id_header?: string
  drop_policy?: string | null
  large_event_policy?: Record<string, unknown>
}

export interface RealtimeTeleopCapability {
  path: string
  transport: 'websocket'
  control_messages: string[]
  binary_camera_frames: boolean
  legacy_camera_query?: string
}

export interface RealtimeCameraCapability {
  path: string
  transport: 'websocket'
  binary_camera_frames: boolean
  explicit_subscription: boolean
}

export interface RealtimeCloudCapability {
  path: string
  transport: 'websocket'
  binary_point_cloud_frames: boolean
  drop_policy?: string | null
}

export interface AppRealtimeCapabilities {
  events: RealtimeEventsCapability
  teleop: RealtimeTeleopCapability
  camera: RealtimeCameraCapability
  cloud: RealtimeCloudCapability
}

export interface TrafficSSEStats {
  clients: number
  queue_maxsize?: number | null
  queue_depths: number[]
  max_depth_seen: number
  latest_event_id: number
  published_events: number
  dropped_events: number
  suppressed_events: Record<string, number>
  raster_min_interval_s?: number | null
  slope_grid_inline: boolean
  drop_policy?: string | null
  [key: string]: unknown
}

export interface TrafficCloudStats {
  clients: number
  queue_maxsize?: number | null
  queue_depths: number[]
  max_depth_seen: number
  published_frames: number
  dropped_frames: number
  drop_policy?: string | null
  latest_seq: number
  [key: string]: unknown
}

export interface AppTrafficResponse {
  schema_version: number
  ts: number
  server: ServerInfo
  status: 'ok' | 'degraded'
  sse: TrafficSSEStats
  cloud: TrafficCloudStats
  recommended_client_rates_hz: Record<string, number>
  client_policy: {
    usage: string
    poll_rates_hz: Record<string, number>
    events_endpoint: string
    traffic_endpoint: string
    cloud_endpoint: string
    large_event_policy: Record<string, unknown>
    backpressure: Record<string, unknown>
    [key: string]: unknown
  }
  warnings: string[]
  links: ClientLinks
}

export interface AppBootstrapResponse {
  schema_version: number
  ts: number
  server: ServerInfo
  robot: Record<string, unknown>
  session: SessionEvent['data'] | Record<string, unknown>
  mission: Record<string, unknown>
  safety: Record<string, unknown>
  localization: Record<string, unknown>
  navigation: Record<string, unknown>
  control: Record<string, unknown>
  map: Record<string, unknown>
  scene: { available: boolean; endpoint: string }
  path: { points: number; endpoint: string }
  media: AppMediaLinks
  traffic: Record<string, unknown> & {
    client_policy?: {
      usage: string
      poll_rates_hz: Record<string, number>
      events_endpoint: string
      traffic_endpoint?: string
      cloud_endpoint?: string
      large_event_policy?: Record<string, unknown>
      backpressure?: Record<string, unknown>
    }
  }
  capabilities: Record<string, boolean>
  capabilities_endpoint: string
  links: ClientLinks
}

export interface AppCapabilitiesResponse {
  schema_version: number
  ts: number
  server: ServerInfo
  auth: Record<string, unknown>
  features: Record<string, boolean>
  endpoints: Record<string, Record<string, EndpointSpec>>
  probes: Record<string, EndpointSpec>
  realtime: AppRealtimeCapabilities
  client_policy: Record<string, unknown>
  links: ClientLinks
}

export interface CommandReceipt {
  name: string
  request_id?: string | null
  client_id: string
  accepted: boolean
  replay: boolean
  ts: number
}

export interface GatewayErrorResponse {
  schema_version?: number
  ok?: false
  error: string
  message?: string | null
  detail?: unknown
  command?: CommandReceipt
}

export interface ControlCommandResponse {
  schema_version: number
  ok: boolean
  status: string
  command: CommandReceipt
  goal?: number[] | null
  instruction?: string | null
  mode?: string | null
}

export type LeaseAction = 'acquire' | 'release' | 'renew'

export interface LeaseRequest {
  action: LeaseAction
  client_id?: string
  request_id?: string | null
  ttl?: number
}

export interface LeaseResponse {
  schema_version: number
  ok: boolean
  status: string
  command: CommandReceipt
  holder?: string | null
  active?: boolean | null
  expires_in?: number | null
}

export interface OdometryEvent {
  type: 'odometry'
  x: number
  y: number
  yaw: number
  vx: number
}

export interface MissionStatusEvent {
  type: 'mission_status'
  state: string
  goal: string | null
  progress: number
}

export interface SafetyStateEvent {
  type: 'safety_state'
  estop: boolean
  level: string
}

export interface SceneGraphEvent {
  type: 'scene_graph'
  objects: Array<{ id: string; label: string; x: number; y: number; confidence: number }>
}

export interface HeartbeatEvent {
  type: 'heartbeat'
}

export interface PingEvent {
  type: 'ping'
}

export interface SnapshotEventData {
  odometry?: Record<string, unknown>
  safety?: Record<string, unknown>
  mission?: Record<string, unknown>
  mode?: string
  session?: Record<string, unknown>
}

export interface SnapshotEvent {
  type: 'snapshot'
  data?: SnapshotEventData
}

export interface MissionEvent {
  type: 'mission'
  data?: Record<string, unknown>
}

export interface SafetyEvent {
  type: 'safety'
  data?: Record<string, unknown>
}

export interface EvalEvent {
  type: 'eval'
  data?: Record<string, unknown>
}

export interface DialogueEvent {
  type: 'dialogue'
  data?: Record<string, unknown>
}

export interface GnssFusionEvent {
  type: 'gnss_fusion'
  data?: Record<string, unknown>
}

export interface SlamDiagnosticEvent {
  type: 'slam_diag'
  data?: Record<string, unknown>
}

export interface SlamDriftEvent {
  type: 'slam_drift'
  level?: string
  xy?: number
  v?: number
  action?: string
  count?: number
  dump_path?: string
  data?: Record<string, unknown>
}

export interface TareStatsEvent {
  type: 'tare_stats'
  data?: Record<string, unknown>
}

export interface ExplorationSupervisorEvent {
  type: 'exploration_supervisor'
  data?: Record<string, unknown>
}

export interface ExploringEvent {
  type: 'exploring'
  active?: boolean
}

export interface SlamStatusEvent {
  type: 'slam_status'
  slam_hz: number
  mode: string
  map_points: number
  degeneracy_count: number
}

export interface RobotStatusEvent {
  type: 'robot_status'
  battery: number
  temperature: number
}

export interface PathPoint {
  x: number
  y: number
  z: number
  yaw?: number | null
  frame_id?: string | null
  ts?: number | null
  metadata?: Record<string, unknown>
}

export interface GlobalPathEvent {
  type: 'global_path'
  points: PathPoint[]
}

export interface LocalPathEvent {
  type: 'local_path'
  points: PathPoint[]
}

export interface MapCloudEvent {
  type: 'map_cloud'
  points?: number[]  // flat [x,y,z, x,y,z, ...] when streamed inline
  count: number
  seq?: number
  bytes?: number
}

export interface SavedMapEvent {
  type: 'saved_map'  // localizer-refined static map, map frame, the 底图
  points: number[]
  count: number
}

export interface SessionEvent {
  type: 'session'
  data: {
    mode: 'idle' | 'mapping' | 'navigating' | 'exploring'
    slam_profile?: string | null
    localization_backend?: string | null
    health_source?: string | null
    active_map: string | null
    map_has_pcd: boolean
    map_has_tomogram: boolean
    since: number
    pending: boolean
    error: string
    icp_quality: number
    localizer_ready: boolean
    localizer_algorithm_healthy?: boolean
    pose_fresh?: boolean | null
    pose_freshness?: string | null
    map_state?: string | null
    map_save_supported?: boolean
    map_save_source?: string | null
    relocalization_supported?: boolean
    saved_map_relocalization_supported?: boolean
    restart_recovery_supported?: boolean
    recovery_method?: string | null
    relocalization_state?: string | null
    recovery_signal?: string | null
    recovery_action?: string | null
    can_start_mapping: boolean
    can_start_navigating: boolean
    can_start_exploring: boolean
    can_end: boolean
    explorer_available: boolean
    explorer_unavailable_reason?: string | null
    explorer_required_profile?: string | null
  }
}

export interface SessionTransitionResponse {
  schema_version: number
  ok: boolean
  success: boolean
  session?: SessionEvent['data'] | null
  message?: string | null
  ts: number
  [key: string]: unknown
}

export interface DynamicFilterResult {
  success: boolean
  orig_count?: number
  clean_count?: number
  dropped?: number
  elapsed_s?: number
  error?: string
  skipped?: boolean
  [key: string]: unknown
}

export interface MapLifecycleResponse {
  schema_version: number
  ok: boolean
  success?: boolean | null
  message?: string | null
  name?: string | null
  active?: string | null
  old_name?: string | null
  new_name?: string | null
  path?: string | null
  size?: string | null
  slam_profile?: string | null
  source?: string | null
  map_save_source?: string | null
  relocalization_supported?: boolean | null
  saved_map_relocalization_supported?: boolean | null
  restart_recovery_supported?: boolean | null
  recovery_method?: string | null
  warnings?: unknown[] | null
  errors?: unknown[] | null
  dynamic_filter?: DynamicFilterResult | null
  ts: number
  [key: string]: unknown
}

export interface CostmapEvent {
  type: 'costmap'
  grid_b64: string   // base64-encoded uint8 flat array (0=free, 100=occupied)
  rows: number       // number of rows (Y axis)
  cols: number       // number of columns (X axis)
  resolution: number // meters per cell
  origin: [number, number]  // world [x, y] of bottom-left corner in map frame
  yaw?: number       // rad, map→odom yaw applied to grid orientation (navigating mode)
}

export interface SlopeGridEvent {
  type: 'slope_grid'
  grid_b64?: string  // optional base64-encoded uint8 (0-90 deg mapped to 0-255)
  payload?: 'inline' | 'omitted'
  available?: boolean
  reason?: string
  encoding?: string
  cols: number
  rows?: number
  resolution: number
  origin: [number, number]
  yaw?: number
}

export interface AgentMessageEvent {
  type: 'agent_message'
  role: 'thinking' | 'assistant' | 'tool'
  text: string
  phase?: string
  ts: number
}

export interface SSEEnvelopeFields {
  schema_version?: number
  event_id?: number
  ts?: number
}

export type SSEEvent = SSEEnvelopeFields & (
  | OdometryEvent
  | MissionStatusEvent
  | SafetyStateEvent
  | SceneGraphEvent
  | HeartbeatEvent
  | PingEvent
  | SnapshotEvent
  | MissionEvent
  | SafetyEvent
  | EvalEvent
  | DialogueEvent
  | GnssFusionEvent
  | SlamDiagnosticEvent
  | SlamDriftEvent
  | TareStatsEvent
  | ExplorationSupervisorEvent
  | ExploringEvent
  | SlamStatusEvent
  | RobotStatusEvent
  | GlobalPathEvent
  | MapCloudEvent
  | SavedMapEvent
  | SessionEvent
  | CostmapEvent
  | SlopeGridEvent
  | AgentMessageEvent
  | LocalPathEvent
)

export interface SSEState {
  odometry: OdometryEvent | null
  missionStatus: MissionStatusEvent | null
  safetyState: SafetyStateEvent | null
  sceneGraph: SceneGraphEvent | null
  slamStatus: SlamStatusEvent | null
  robotStatus: RobotStatusEvent | null
  globalPath: GlobalPathEvent | null
  localPath: LocalPathEvent | null
  mapCloud: MapCloudEvent | null
  savedMap: SavedMapEvent | null
  session: SessionEvent['data'] | null
  locations: LocationsResponse | null
  stateSnapshot: StateResponse | null
  traffic: AppTrafficResponse | null
  costmap: CostmapEvent | null
  slopeGrid: SlopeGridEvent | null
  agentMessage: AgentMessageEvent | null  // latest agent chat message (ts dedups)
  gnssFusion: GnssFusionEvent | null
  slamDiag: SlamDiagnosticEvent | null
  slamDrift: SlamDriftEvent | null
  tareStats: TareStatsEvent | null
  explorationSupervisor: ExplorationSupervisorEvent | null
  exploring: ExploringEvent | null
  evalEvent: EvalEvent | null
  dialogue: DialogueEvent | null
  lastHeartbeat: number | null
  lastEventId: number | null
  missedEvents: number
  reconnects: number
  lastError: string | null
  lastRefreshAt: number | null
  lastRefreshReason: string | null
  refreshError: string | null
  connected: boolean
  events: SSEEvent[]
}

export type ToastKind = 'success' | 'error' | 'info'

export interface Toast {
  id: number
  message: string
  kind: ToastKind
}

export type Tab = 'console' | 'scene' | 'map' | 'slam'

export type SlamProfile = 'fastlio2' | 'localizer' | 'super_lio' | 'super_lio_relocation' | 'stop'

export interface SlamOperationResponse {
  schema_version: number
  ok: boolean
  success: boolean
  profile?: string | null
  message?: string | null
  quality?: number | null
  ts: number
  [key: string]: unknown
}

export interface BagStatusResponse {
  recording: boolean
  path?: string | null
  duration_s: number
  size_bytes: number
  pid?: number | null
  exit_code?: number | null
  disk_free: number
  disk_total: number
}

export interface BagOperationResponse {
  status?: string | null
  path?: string | null
  pid?: number | null
  duration?: number | null
  prefix?: string | null
  error?: string | null
  detail?: unknown
}
