interface Layout {
  x: number
  y: number
  w: number
  h: number
  z: number
}

const LS_KEY = 'lingtu-widget-layouts-v5'
const LS_OLD_KEYS = [
  'lingtu-widget-layouts-v1',
  'lingtu-widget-layouts-v2',
  'lingtu-widget-layouts-v3',
  'lingtu-widget-layouts-v4',
]

export type { Layout }

export function cleanupOldLayouts() {
  try {
    for (const k of LS_OLD_KEYS) localStorage.removeItem(k)
  } catch { /* noop */ }
}

export function loadLayouts(): Record<string, Layout> {
  try {
    const raw = localStorage.getItem(LS_KEY)
    return raw ? JSON.parse(raw) : {}
  } catch {
    return {}
  }
}

export function saveLayouts(layouts: Record<string, Layout>) {
  try {
    localStorage.setItem(LS_KEY, JSON.stringify(layouts))
  } catch { /* quota or unavailable */ }
}

export function resetAllLayouts() {
  try {
    localStorage.removeItem(LS_KEY)
    for (const k of LS_OLD_KEYS) localStorage.removeItem(k)
  } catch { /* noop */ }
  window.location.reload()
}
