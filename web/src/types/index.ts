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

export type SSEEvent =
  | OdometryEvent
  | MissionStatusEvent
  | SafetyStateEvent
  | SceneGraphEvent
  | HeartbeatEvent
  | SlamStatusEvent
  | RobotStatusEvent
  | GlobalPathEvent

export interface SSEState {
  odometry: OdometryEvent | null
  missionStatus: MissionStatusEvent | null
  safetyState: SafetyStateEvent | null
  sceneGraph: SceneGraphEvent | null
  slamStatus: SlamStatusEvent | null
  robotStatus: RobotStatusEvent | null
  globalPath: GlobalPathEvent | null
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

export type Tab = 'console' | 'map' | 'slam' | 'path'

export type SlamProfile = 'fastlio2' | 'localizer' | 'stop'
