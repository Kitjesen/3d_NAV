// Centralized API service layer for LingTu web dashboard
// All fetch() calls in one place.

import type {
  AppBootstrapResponse,
  AppCapabilitiesResponse,
  ControlCommandResponse,
  GatewayErrorResponse,
  LeaseAction,
  LeaseResponse,
  LocationsResponse,
  MapInfo,
  PathResponse,
  SceneGraphResponse,
  SlamProfile,
} from '../types'

const WEB_CLIENT_ID = 'web-dashboard'

async function fetchJson<T>(url: string): Promise<T> {
  const res = await fetch(url)
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
  return res.json() as Promise<T>
}

function makeRequestId(prefix: string): string {
  if (globalThis.crypto?.randomUUID) {
    return `${prefix}-${globalThis.crypto.randomUUID()}`
  }
  return `${prefix}-${Date.now().toString(36)}-${Math.random().toString(36).slice(2)}`
}

function commandBody<T extends Record<string, unknown>>(
  prefix: string,
  body: T,
): T & { client_id: string; request_id: string } {
  return {
    ...body,
    client_id: WEB_CLIENT_ID,
    request_id: makeRequestId(prefix),
  }
}

async function postJson<T>(url: string, body?: unknown): Promise<T> {
  const res = await fetch(url, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  })
  const text = await res.text()
  const data = text ? JSON.parse(text) : undefined
  if (!res.ok) {
    const error = data as GatewayErrorResponse | undefined
    throw new Error(error?.message || error?.error || `HTTP ${res.status}`)
  }
  return data as T
}

// --- App bootstrap / read APIs ---

export async function fetchAppBootstrap(): Promise<AppBootstrapResponse> {
  return fetchJson<AppBootstrapResponse>('/api/v1/app/bootstrap')
}

export async function fetchAppCapabilities(): Promise<AppCapabilitiesResponse> {
  return fetchJson<AppCapabilitiesResponse>('/api/v1/app/capabilities')
}

export async function fetchSceneGraph(): Promise<SceneGraphResponse> {
  return fetchJson<SceneGraphResponse>('/api/v1/scene_graph')
}

export async function fetchPath(): Promise<PathResponse> {
  return fetchJson<PathResponse>('/api/v1/path')
}

export async function fetchLocations(): Promise<LocationsResponse> {
  return fetchJson<LocationsResponse>('/api/v1/locations')
}

// --- Navigation ---

export async function sendInstruction(text: string): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    '/api/v1/instruction',
    commandBody('instruction', { text }),
  )
}

export async function sendGoal(x: number, y: number): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    '/api/v1/goal',
    commandBody('goal', { x, y }),
  )
}

export async function sendStop(): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    '/api/v1/stop',
    commandBody('stop', {}),
  )
}

export async function sendMode(mode: 'manual' | 'autonomous' | 'estop'): Promise<ControlCommandResponse> {
  return postJson<ControlCommandResponse>(
    '/api/v1/mode',
    commandBody('mode', { mode }),
  )
}

export async function updateLease(action: LeaseAction, ttl = 30): Promise<LeaseResponse> {
  return postJson<LeaseResponse>(
    '/api/v1/lease',
    commandBody('lease', { action, ttl }),
  )
}

// --- SLAM ---

export async function switchSlamMode(profile: SlamProfile): Promise<void> {
  const res = await fetch('/api/v1/slam/switch', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ profile }),
  })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
}

// --- Maps ---

export async function fetchMaps(): Promise<MapInfo[]> {
  const data = await fetchJson<{ maps?: MapInfo[] }>('/api/v1/slam/maps')
  return data.maps || []
}

export async function activateMap(name: string): Promise<void> {
  const res = await fetch('/api/v1/map/activate', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
}

export async function deleteMap(name: string): Promise<void> {
  const res = await fetch('/api/v1/maps', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ action: 'delete', name }),
  })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
}

export async function renameMap(oldName: string, newName: string): Promise<void> {
  const res = await fetch('/api/v1/map/rename', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ old_name: oldName, new_name: newName }),
  })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
}

export interface SaveMapResult {
  success: boolean
  name: string
  path?: string
  size?: string
  dynamic_filter?: {
    success: boolean
    orig_count?: number
    clean_count?: number
    dropped?: number
    elapsed_s?: number
    error?: string
  }
}

export async function saveMap(name: string): Promise<SaveMapResult> {
  // Save can take up to ~2 min on a busy robot because PGO + DUFOMap
  // run synchronously. Default fetch has no timeout which is what we want.
  const res = await fetch('/api/v1/map/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
  return res.json() as Promise<SaveMapResult>
}

// --- Session state machine ---

export interface SessionState {
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

export async function fetchSession(): Promise<SessionState> {
  const res = await fetch('/api/v1/session')
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
  return res.json()
}

export async function startSession(mode: 'mapping' | 'navigating' | 'exploring', mapName?: string): Promise<SessionState> {
  const res = await fetch('/api/v1/session/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ mode, map_name: mapName ?? '' }),
  })
  const data = await res.json()
  if (!res.ok || !data.success) throw new Error(data.message || `HTTP ${res.status}`)
  return data.session as SessionState
}

export async function endSession(): Promise<SessionState> {
  const res = await fetch('/api/v1/session/end', { method: 'POST' })
  const data = await res.json()
  if (!res.ok || !data.success) throw new Error(data.message || `HTTP ${res.status}`)
  return data.session as SessionState
}

export async function resetMapCloud(): Promise<void> {
  const res = await fetch('/api/v1/map_cloud/reset', { method: 'POST' })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
}

export async function fetchSavedMapPoints(name: string): Promise<number[]> {
  const res = await fetch(`/api/v1/maps/${encodeURIComponent(name)}/points?max_points=30000`)
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
  const data = await res.json()
  return data.points as number[]
}

export async function relocalize(mapName: string, x: number, y: number, yaw: number): Promise<void> {
  const res = await fetch('/api/v1/slam/relocalize', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ map_name: mapName, x, y, yaw }),
  })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
}

// Global (no-guess) relocalize via 3D-BBS. Fires the worker in localizer;
// the 2-4 s scan runs async, so the response is immediate. Fitness appears
// on /localization_quality after the worker finishes.
export async function autoRelocalize(): Promise<{ success: boolean; message: string }> {
  const res = await fetch('/api/v1/slam/auto_relocalize', { method: 'POST' })
  const data = await res.json()
  if (!res.ok) throw new Error(data.message || `HTTP ${res.status}`)
  return data
}

// --- Auth ---

export async function login(key: string): Promise<{ ok: boolean; message?: string }> {
  const res = await fetch('/api/v1/auth/login', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ key }),
  })
  return res.json()
}

export async function checkAuth(): Promise<{ auth_required: boolean }> {
  const res = await fetch('/api/v1/auth/check')
  return res.json()
}
