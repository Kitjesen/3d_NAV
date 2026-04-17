// Centralized API service layer for LingTu web dashboard
// All fetch() calls in one place.

import type { MapInfo, SlamProfile } from '../types'

// --- Navigation ---

export async function sendInstruction(text: string): Promise<Response> {
  return fetch('/api/v1/instruction', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ text }),
  })
}

export async function sendGoal(x: number, y: number): Promise<Response> {
  return fetch('/api/v1/goal', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ x, y }),
  })
}

export async function sendStop(): Promise<Response> {
  return fetch('/api/v1/stop', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: '{}',
  })
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
  const res = await fetch('/api/v1/slam/maps')
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
  const data = await res.json()
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

export async function saveMap(name: string): Promise<void> {
  const res = await fetch('/api/v1/map/save', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
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
