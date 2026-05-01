// Shared TypeScript interfaces for LingTu web dashboard

export interface MapInfo {
  name: string
  has_pcd: boolean
  has_tomogram: boolean
  is_active: boolean
  size_mb?: number
  patch_count?: number
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

export interface AppBootstrapResponse {
  schema_version: number
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
  media: Record<string, unknown>
  traffic: Record<string, unknown> & {
    client_policy?: {
      usage: string
      poll_rates_hz: Record<string, number>
      events_endpoint: string
    }
  }
  capabilities: Record<string, boolean>
  capabilities_endpoint: string
  links: Record<string, string>
}

export interface AppCapabilitiesResponse {
  schema_version: number
  server: ServerInfo
  auth: Record<string, unknown>
  features: Record<string, boolean>
  endpoints: Record<string, Record<string, EndpointSpec>>
  probes: Record<string, EndpointSpec>
  realtime: Record<string, unknown>
  client_policy: Record<string, unknown>
  links: Record<string, string>
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
  points: number[]   // flat [x,y,z, x,y,z, …]
  count: number
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
    active_map: string | null
    map_has_pcd: boolean
    map_has_tomogram: boolean
    since: number
    pending: boolean
    error: string
    icp_quality: number
    localizer_ready: boolean
    can_start_mapping: boolean
    can_start_navigating: boolean
    can_start_exploring: boolean
    can_end: boolean
    explorer_available: boolean
  }
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
  grid_b64: string   // base64-encoded uint8 (0-90° mapped to 0-255)
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

export type SSEEvent =
  | OdometryEvent
  | MissionStatusEvent
  | SafetyStateEvent
  | SceneGraphEvent
  | HeartbeatEvent
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
  costmap: CostmapEvent | null
  slopeGrid: SlopeGridEvent | null
  agentMessage: AgentMessageEvent | null  // latest agent chat message (ts dedups)
  lastHeartbeat: number | null
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

export type SlamProfile = 'fastlio2' | 'localizer' | 'stop'
