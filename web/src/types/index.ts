// Shared TypeScript interfaces for LingTu web dashboard

export interface MapInfo {
  name: string
  has_pcd: boolean
  has_tomogram: boolean
  is_active: boolean
  size_mb?: number
  patch_count?: number
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
