import { useState, useEffect, useRef, useCallback } from 'react'
import styles from './FloatingWidget.module.css'

// ── Global z-index counter — shared across all widgets ──────────
let globalZ = 10

// ── Layout persistence ──────────────────────────────────────────
interface Layout {
  x: number
  y: number
  w: number
  h: number
  z: number
}

const LS_KEY = 'lingtu-widget-layouts-v1'

function loadLayouts(): Record<string, Layout> {
  try {
    const raw = localStorage.getItem(LS_KEY)
    return raw ? JSON.parse(raw) : {}
  } catch {
    return {}
  }
}

function saveLayouts(layouts: Record<string, Layout>) {
  try {
    localStorage.setItem(LS_KEY, JSON.stringify(layouts))
  } catch { /* quota or unavailable */ }
}

export function resetAllLayouts() {
  try { localStorage.removeItem(LS_KEY) } catch { /* noop */ }
  window.location.reload()
}

// ── Props ───────────────────────────────────────────────────────

export interface FloatingWidgetProps {
  id: string
  title: React.ReactNode
  titleIcon?: React.ReactNode
  defaultPos: { x: number; y: number }
  defaultSize: { w: number; h: number }
  minSize?: { w: number; h: number }
  children: React.ReactNode
  headerActions?: React.ReactNode
}

type DragMode = null | 'move' | 'e' | 's' | 'n' | 'w' | 'se' | 'sw' | 'ne' | 'nw'

export function FloatingWidget({
  id,
  title,
  titleIcon,
  defaultPos,
  defaultSize,
  minSize = { w: 260, h: 180 },
  children,
  headerActions,
}: FloatingWidgetProps) {
  // Load persisted layout or use defaults
  const [layout, setLayout] = useState<Layout>(() => {
    const stored = loadLayouts()[id]
    if (stored) return stored
    return {
      x: defaultPos.x,
      y: defaultPos.y,
      w: defaultSize.w,
      h: defaultSize.h,
      z: ++globalZ,
    }
  })

  const [dragMode, setDragMode] = useState<DragMode>(null)
  const [focused, setFocused] = useState(false)
  const dragStartRef = useRef<{
    mouseX: number
    mouseY: number
    startX: number
    startY: number
    startW: number
    startH: number
  } | null>(null)

  // Persist layout whenever it changes
  useEffect(() => {
    const all = loadLayouts()
    all[id] = layout
    saveLayouts(all)
  }, [id, layout])

  // Bring widget to front when clicked
  const bringToFront = useCallback(() => {
    setLayout(prev => ({ ...prev, z: ++globalZ }))
    setFocused(true)
  }, [])

  // Start a drag or resize operation
  const startAction = useCallback((mode: DragMode, e: React.MouseEvent) => {
    e.preventDefault()
    e.stopPropagation()
    dragStartRef.current = {
      mouseX: e.clientX,
      mouseY: e.clientY,
      startX: layout.x,
      startY: layout.y,
      startW: layout.w,
      startH: layout.h,
    }
    setDragMode(mode)
    bringToFront()
  }, [layout, bringToFront])

  // Global mouse move/up listeners while dragging
  useEffect(() => {
    if (!dragMode) return

    const onMove = (e: MouseEvent) => {
      const start = dragStartRef.current
      if (!start) return
      const dx = e.clientX - start.mouseX
      const dy = e.clientY - start.mouseY

      setLayout(prev => {
        const next = { ...prev }

        if (dragMode === 'move') {
          next.x = start.startX + dx
          next.y = start.startY + dy
        } else {
          // Resize modes
          if (dragMode.includes('e')) {
            next.w = Math.max(minSize.w, start.startW + dx)
          }
          if (dragMode.includes('w')) {
            const newW = Math.max(minSize.w, start.startW - dx)
            next.x = start.startX + (start.startW - newW)
            next.w = newW
          }
          if (dragMode.includes('s')) {
            next.h = Math.max(minSize.h, start.startH + dy)
          }
          if (dragMode.includes('n')) {
            const newH = Math.max(minSize.h, start.startH - dy)
            next.y = start.startY + (start.startH - newH)
            next.h = newH
          }
        }

        // Clamp to viewport
        const maxX = window.innerWidth - next.w
        const maxY = window.innerHeight - next.h
        next.x = Math.max(0, Math.min(maxX, next.x))
        next.y = Math.max(0, Math.min(maxY, next.y))

        return next
      })
    }

    const onUp = () => {
      setDragMode(null)
      dragStartRef.current = null
    }

    window.addEventListener('mousemove', onMove)
    window.addEventListener('mouseup', onUp)
    return () => {
      window.removeEventListener('mousemove', onMove)
      window.removeEventListener('mouseup', onUp)
    }
  }, [dragMode, minSize.w, minSize.h])

  // Lose focus when clicking outside (any other widget/anywhere)
  useEffect(() => {
    const onClick = (e: MouseEvent) => {
      const widgetEl = document.getElementById(`widget-${id}`)
      if (widgetEl && !widgetEl.contains(e.target as Node)) {
        setFocused(false)
      }
    }
    window.addEventListener('mousedown', onClick)
    return () => window.removeEventListener('mousedown', onClick)
  }, [id])

  const wrapperClass = [
    styles.widget,
    focused ? styles.widgetFocused : '',
    dragMode === 'move' ? styles.widgetDragging : '',
    dragMode && dragMode !== 'move' ? styles.widgetResizing : '',
  ].filter(Boolean).join(' ')

  return (
    <div
      id={`widget-${id}`}
      className={wrapperClass}
      style={{
        left: layout.x,
        top: layout.y,
        width: layout.w,
        height: layout.h,
        zIndex: layout.z,
      }}
      onMouseDown={bringToFront}
    >
      <div
        className={styles.header}
        onMouseDown={(e) => {
          // Only start drag if mousedown was directly on header, not on action buttons
          if ((e.target as HTMLElement).closest('button')) return
          startAction('move', e)
        }}
      >
        <span className={styles.dots}>
          <span className={`${styles.dot} ${styles.dotRed}`} />
          <span className={`${styles.dot} ${styles.dotYellow}`} />
          <span className={`${styles.dot} ${styles.dotGreen}`} />
        </span>
        <span className={styles.title}>
          {titleIcon && <span className={styles.titleIcon}>{titleIcon}</span>}
          {title}
        </span>
        {headerActions && <span className={styles.actions}>{headerActions}</span>}
      </div>

      <div className={styles.body}>{children}</div>

      {/* Resize handles */}
      <div className={styles.resizeN}  onMouseDown={(e) => startAction('n',  e)} />
      <div className={styles.resizeS}  onMouseDown={(e) => startAction('s',  e)} />
      <div className={styles.resizeE}  onMouseDown={(e) => startAction('e',  e)} />
      <div className={styles.resizeW}  onMouseDown={(e) => startAction('w',  e)} />
      <div className={styles.resizeNE} onMouseDown={(e) => startAction('ne', e)} />
      <div className={styles.resizeNW} onMouseDown={(e) => startAction('nw', e)} />
      <div className={styles.resizeSE} onMouseDown={(e) => startAction('se', e)} />
      <div className={styles.resizeSW} onMouseDown={(e) => startAction('sw', e)} />
    </div>
  )
}
