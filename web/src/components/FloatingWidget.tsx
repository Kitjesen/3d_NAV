import { useState, useEffect, useLayoutEffect, useRef, useCallback, type CSSProperties } from 'react'
import {
  cleanupOldLayouts,
  loadLayouts,
  saveLayouts,
  type Layout,
} from './floatingWidgetLayout'
import styles from './FloatingWidget.module.css'

// ── Global z-index counter — shared across all widgets ──────────
let globalZ = 10

// Clean up any stale keys from previous layout generations
cleanupOldLayouts()

// ── Props ───────────────────────────────────────────────────────

export interface FloatingWidgetProps {
  id: string
  defaultPos: { x: number; y: number }
  defaultSize: { w: number; h: number }
  minSize?: { w: number; h: number }
  responsiveLayout?: ResponsiveLayout
  dragHandleLeft?: string
  children: React.ReactNode
}

type DragMode = null | 'move' | 'e' | 's' | 'n' | 'w' | 'se' | 'sw' | 'ne' | 'nw'

interface Bounds {
  width: number
  height: number
}

type ResponsiveLayout = (bounds: Bounds) => Omit<Layout, 'z'>

function clampLayout(layout: Layout, bounds: Bounds, minSize: { w: number; h: number }): Layout {
  const maxWidth = Math.max(1, bounds.width)
  const maxHeight = Math.max(1, bounds.height)
  const minW = Math.min(minSize.w, maxWidth)
  const minH = Math.min(minSize.h, maxHeight)
  const w = Math.min(Math.max(layout.w, minW), maxWidth)
  const h = Math.min(Math.max(layout.h, minH), maxHeight)
  const maxX = Math.max(0, maxWidth - w)
  const maxY = Math.max(0, maxHeight - h)

  return {
    ...layout,
    x: Math.max(0, Math.min(maxX, layout.x)),
    y: Math.max(0, Math.min(maxY, layout.y)),
    w,
    h,
  }
}

function isSameLayout(a: Layout, b: Layout): boolean {
  return a.x === b.x && a.y === b.y && a.w === b.w && a.h === b.h && a.z === b.z
}

export function FloatingWidget({
  id,
  defaultPos,
  defaultSize,
  minSize = { w: 260, h: 180 },
  responsiveLayout,
  dragHandleLeft,
  children,
}: FloatingWidgetProps) {
  const minW = minSize.w
  const minH = minSize.h

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
  const widgetRef = useRef<HTMLDivElement>(null)
  const dragStartRef = useRef<{
    mouseX: number
    mouseY: number
    startX: number
    startY: number
    startW: number
    startH: number
  } | null>(null)

  const getBounds = useCallback((): Bounds => {
    const parent = widgetRef.current?.parentElement
    if (parent) {
      return {
        width: parent.clientWidth,
        height: parent.clientHeight,
      }
    }
    return {
      width: window.innerWidth,
      height: window.innerHeight,
    }
  }, [])

  const clampToCurrentBounds = useCallback((current: Layout) => (
    clampLayout(current, getBounds(), { w: minW, h: minH })
  ), [getBounds, minW, minH])

  const applyResponsiveLayout = useCallback(() => {
    if (!responsiveLayout) return

    const bounds = getBounds()
    const desired = responsiveLayout(bounds)
    setLayout(prev => {
      const next = clampLayout({ ...prev, ...desired }, bounds, { w: minW, h: minH })
      return isSameLayout(prev, next) ? prev : next
    })
  }, [getBounds, minW, minH, responsiveLayout])

  useLayoutEffect(() => {
    const clamp = () => {
      if (responsiveLayout) {
        applyResponsiveLayout()
        return
      }

      setLayout(prev => {
        const next = clampToCurrentBounds(prev)
        return isSameLayout(prev, next) ? prev : next
      })
    }

    clamp()

    const parent = widgetRef.current?.parentElement
    const ro = parent ? new ResizeObserver(clamp) : null
    if (parent) ro?.observe(parent)
    window.addEventListener('resize', clamp)

    return () => {
      ro?.disconnect()
      window.removeEventListener('resize', clamp)
    }
  }, [applyResponsiveLayout, clampToCurrentBounds, responsiveLayout])

  // Persist layout whenever it changes
  useEffect(() => {
    const all = loadLayouts()
    all[id] = layout
    saveLayouts(all)
  }, [id, layout])

  // Bring widget to front when clicked
  const bringToFront = useCallback(() => {
    setLayout(prev => ({ ...prev, z: ++globalZ }))
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
            next.w = Math.max(minW, start.startW + dx)
          }
          if (dragMode.includes('w')) {
            const newW = Math.max(minW, start.startW - dx)
            next.x = start.startX + (start.startW - newW)
            next.w = newW
          }
          if (dragMode.includes('s')) {
            next.h = Math.max(minH, start.startH + dy)
          }
          if (dragMode.includes('n')) {
            const newH = Math.max(minH, start.startH - dy)
            next.y = start.startY + (start.startH - newH)
            next.h = newH
          }
        }

        return clampToCurrentBounds(next)
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
  }, [dragMode, minW, minH, clampToCurrentBounds])

  const wrapperClass = [
    styles.widget,
    dragMode === 'move' ? styles.widgetDragging : '',
    dragMode && dragMode !== 'move' ? styles.widgetResizing : '',
  ].filter(Boolean).join(' ')
  const widgetStyle: CSSProperties & { '--drag-handle-left'?: string } = {
    left: layout.x,
    top: layout.y,
    width: layout.w,
    height: layout.h,
    zIndex: layout.z,
    '--drag-handle-left': dragHandleLeft,
  }

  return (
    <div
      ref={widgetRef}
      id={`widget-${id}`}
      className={wrapperClass}
      style={widgetStyle}
      onMouseDown={bringToFront}
    >
      {/* Invisible drag strip at the top — only visible on hover */}
      <div
        className={styles.dragStrip}
      >
        <span
          className={styles.dragGrabber}
          onMouseDown={(e) => startAction('move', e)}
          title="拖动移动"
        />
      </div>

      <div className={styles.body}>{children}</div>

      {/* Resize handles (invisible, hover cursors) */}
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
