// Centralized API service layer for LingTu web dashboard
// All fetch() calls in one place.

import type {
  AppBootstrapResponse,
  AppCapabilitiesResponse,
  CommandReceipt,
  ControlCommandResponse,
  GatewayErrorResponse,
  LeaseAction,
  LeaseResponse,
  LocationsResponse,
  MapInfo,
  PathResponse,
  SceneGraphResponse,
  SessionEvent,
  SlamProfile,
  StateResponse,
} from '../types'

const WEB_CLIENT_ID = 'web-dashboard'

type CommandResponse = ControlCommandResponse | LeaseResponse

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null
}

function shortRequestId(id?: string | null): string {
  return id ? id.slice(0, 8) : ''
}

function detailStrings(detail: unknown, key: string): string[] {
  if (!isRecord(detail)) return []
  const value = detail[key]
  if (!Array.isArray(value)) return []
  return value.filter((item): item is string => typeof item === 'string' && item.length > 0)
}

function commandSuffix(command?: CommandReceipt): string {
  const parts: string[] = []
  if (command?.replay) parts.push('重复请求已确认')
  const requestId = shortRequestId(command?.request_id)
  if (requestId) parts.push(`#${requestId}`)
  return parts.length ? `（${parts.join(' ')}）` : ''
}

export class GatewayApiError extends Error {
  readonly statusCode: number
  readonly body?: GatewayErrorResponse
  readonly command?: CommandReceipt
  readonly errorCode?: string
  readonly detail?: unknown

  constructor(statusCode: number, body?: GatewayErrorResponse) {
    super(body?.message || body?.error || `HTTP ${statusCode}`)
    this.name = 'GatewayApiError'
    this.statusCode = statusCode
    this.body = body
    this.command = body?.command
    this.errorCode = body?.error
    this.detail = body?.detail
  }
}

export function isGatewayApiError(error: unknown): error is GatewayApiError {
  return error instanceof GatewayApiError
}

export function formatCommandAck(response: CommandResponse, label = '命令'): string {
  if (!response.ok) {
    return `${label}未接受${commandSuffix(response.command)}`
  }
  return `${label}已提交${commandSuffix(response.command)}`
}

export function formatCommandError(error: unknown, label = '命令失败'): string {
  if (isGatewayApiError(error)) {
    const message = error.body?.message || error.body?.error || error.message || `HTTP ${error.statusCode}`
    const blockers = detailStrings(error.detail, 'blockers')
    const advisories = detailStrings(error.detail, 'advisories')
    const reasonParts = [...blockers, ...advisories].slice(0, 3)
    const reason = reasonParts.length ? `；原因：${reasonParts.join('，')}` : ''
    const rejected = error.command?.accepted === false ? '（未接受）' : ''
    return `${label}${rejected}: ${message}${reason}${commandSuffix(error.command)}`
  }
  if (error instanceof Error) {
    return `${label}: ${error.message}`
  }
  return `${label}: ${String(error)}`
}

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
  let data: unknown
  try {
    data = text ? JSON.parse(text) : undefined
  } catch {
    data = undefined
  }
  if (!res.ok) {
    const error = data as GatewayErrorResponse | undefined
    throw new GatewayApiError(res.status, error)
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

export async function fetchState(): Promise<StateResponse> {
  return fetchJson<StateResponse>('/api/v1/state')
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
  slam_profile?: string
  source?: string
  map_save_source?: string
  relocalization_supported?: boolean
  saved_map_relocalization_supported?: boolean
  restart_recovery_supported?: boolean
  recovery_method?: string
  warnings?: string[]
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

export type SessionState = SessionEvent['data']
export type SessionMode = 'mapping' | 'navigating' | 'exploring'
export interface StartSessionOptions {
  mapName?: string
  slamProfile?: Exclude<SlamProfile, 'stop'>
}

export async function fetchSession(): Promise<SessionState> {
  const res = await fetch('/api/v1/session')
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
  return res.json()
}

export async function startSession(mode: SessionMode, mapNameOrOptions?: string | StartSessionOptions): Promise<SessionState> {
  const options: StartSessionOptions = typeof mapNameOrOptions === 'string'
    ? { mapName: mapNameOrOptions }
    : (mapNameOrOptions ?? {})
  const body: Record<string, string> = {
    mode,
    map_name: options.mapName ?? '',
  }
  if (options.slamProfile) {
    body.slam_profile = options.slamProfile
  }
  const res = await fetch('/api/v1/session/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
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
