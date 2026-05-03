export interface WidgetLayout {
  x: number
  y: number
  w: number
  h: number
  z: number
}

const LS_KEY = 'lingtu-widget-layouts-v4'
const LS_OLD_KEYS = [
  'lingtu-widget-layouts-v1',
  'lingtu-widget-layouts-v2',
  'lingtu-widget-layouts-v3',
]

export function cleanupOldLayouts() {
  try {
    for (const key of LS_OLD_KEYS) localStorage.removeItem(key)
  } catch { /* noop */ }
}

export function loadLayouts(): Record<string, WidgetLayout> {
  try {
    const raw = localStorage.getItem(LS_KEY)
    return raw ? JSON.parse(raw) : {}
  } catch {
    return {}
  }
}

export function saveLayouts(layouts: Record<string, WidgetLayout>) {
  try {
    localStorage.setItem(LS_KEY, JSON.stringify(layouts))
  } catch { /* quota or unavailable */ }
}

export function resetAllLayouts() {
  try {
    localStorage.removeItem(LS_KEY)
    for (const key of LS_OLD_KEYS) localStorage.removeItem(key)
  } catch { /* noop */ }
  window.location.reload()
}
