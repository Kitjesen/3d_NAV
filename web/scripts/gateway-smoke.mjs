#!/usr/bin/env node

const host = process.env.ROBOT_HOST || '127.0.0.1:5050'
const base = (process.env.GATEWAY_URL || `http://${host}`).replace(/\/+$/, '')
const allowDegradedReady = process.env.ALLOW_DEGRADED_READY === '1'

const requiredBootstrapKeys = [
  'capabilities',
  'control',
  'links',
  'localization',
  'map',
  'media',
  'mission',
  'navigation',
  'robot',
  'safety',
  'scene',
  'session',
]

const requiredLinks = [
  'bootstrap',
  'events',
  'health',
  'session',
  'session_start',
  'session_end',
  'navigation_status',
  'navigation_plan',
  'navigation_cancel',
  'goal',
  'stop',
  'maps',
  'map_activate',
  'map_save',
  'map_points',
  'slam_status',
  'slam_switch',
  'slam_relocalize',
]

async function request(path, { json = true } = {}) {
  const url = new URL(path, `${base}/`)
  const res = await fetch(url)
  const text = await res.text()
  let body = text
  if (json && text) {
    try {
      body = JSON.parse(text)
    } catch (err) {
      throw new Error(`${path} returned non-JSON: ${err.message}`)
    }
  }
  return { status: res.status, body }
}

function missingKeys(obj, keys) {
  return keys.filter(key => obj == null || !(key in obj))
}

function ensure(condition, failures, message) {
  if (!condition) failures.push(message)
}

const failures = []

try {
  const root = await request('/', { json: false })
  ensure(root.status === 200, failures, `GET / expected 200, got ${root.status}`)

  const bootstrap = await request('/api/v1/app/bootstrap')
  ensure(bootstrap.status === 200, failures, `GET /api/v1/app/bootstrap expected 200, got ${bootstrap.status}`)

  const bootstrapBody = bootstrap.body
  const missingBootstrap = missingKeys(bootstrapBody, requiredBootstrapKeys)
  ensure(missingBootstrap.length === 0, failures, `bootstrap missing keys: ${missingBootstrap.join(', ')}`)

  const links = bootstrapBody.links || {}
  const missingLinks = missingKeys(links, requiredLinks)
  ensure(missingLinks.length === 0, failures, `bootstrap.links missing keys: ${missingLinks.join(', ')}`)

  const health = await request('/api/v1/health')
  ensure(health.status === 200, failures, `GET /api/v1/health expected 200, got ${health.status}`)

  const ready = await request('/ready', { json: false })
  if (!allowDegradedReady) {
    ensure(ready.status === 200, failures, `GET /ready expected 200, got ${ready.status}`)
  } else {
    ensure([200, 503].includes(ready.status), failures, `GET /ready expected 200 or 503, got ${ready.status}`)
  }

  const session = bootstrapBody.session || {}
  const localization = bootstrapBody.localization || {}
  const navigation = bootstrapBody.navigation || {}

  const summary = {
    base,
    ready: ready.status,
    session_mode: session.mode ?? null,
    active_map: session.active_map ?? null,
    localization_state: localization.state ?? null,
    localization_backend: session.localization_backend ?? session.slam_profile ?? null,
    navigation_ready: navigation.can_accept_goal ?? navigation.readiness?.can_accept_goal ?? null,
    link_count: Object.keys(links).length,
  }

  console.log(JSON.stringify({ ok: failures.length === 0, summary, failures }, null, 2))
} catch (err) {
  failures.push(err instanceof Error ? err.message : String(err))
  console.log(JSON.stringify({ ok: false, base, failures }, null, 2))
}

if (failures.length > 0) {
  process.exitCode = 1
}
