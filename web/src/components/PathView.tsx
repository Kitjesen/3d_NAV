import { useRef, useEffect, useCallback, useState } from 'react'
import type { SSEState, PathPoint } from '../types'
import * as api from '../services/api'
import styles from './PathView.module.css'

interface PathViewProps {
  sseState: SSEState
  showToast: (message: string, kind?: 'success' | 'error' | 'info') => void
}

// Canvas drawing constants
const BG_COLOR      = '#0a0e17'
const GRID_COLOR    = 'rgba(255,255,255,0.03)'
const PATH_COLOR    = '#00ff88'
const ROBOT_COLOR   = '#00ff88'
const GOAL_COLOR    = '#ef4444'
const TRAIL_COLOR   = 'rgba(0,255,136,0.20)'
const ARROW_INTERVAL = 8  // draw direction arrow every N path segments
const TRAIL_MAX      = 200

interface ViewTransform {
  scale: number       // pixels per metre
  originX: number     // canvas px for world (0,0)
  originY: number     // canvas px for world (0,0)
}

function worldToCanvas(wx: number, wy: number, t: ViewTransform): [number, number] {
  return [t.originX + wx * t.scale, t.originY - wy * t.scale]
}

function canvasToWorld(cx: number, cy: number, t: ViewTransform): [number, number] {
  return [(cx - t.originX) / t.scale, -(cy - t.originY) / t.scale]
}

function computeTransform(
  points: PathPoint[],
  robotX: number,
  robotY: number,
  width: number,
  height: number,
): ViewTransform {
  // Collect all world points to auto-fit
  const xs: number[] = [robotX]
  const ys: number[] = [robotY]
  for (const p of points) {
    xs.push(p.x)
    ys.push(p.y)
  }

  const minX = Math.min(...xs)
  const maxX = Math.max(...xs)
  const minY = Math.min(...ys)
  const maxY = Math.max(...ys)

  const spanX = maxX - minX || 10
  const spanY = maxY - minY || 10

  const padding = 0.15  // 15% padding on each side
  const scaleX = (width  * (1 - 2 * padding)) / spanX
  const scaleY = (height * (1 - 2 * padding)) / spanY
  const scale  = Math.min(scaleX, scaleY)

  const midWorldX = (minX + maxX) / 2
  const midWorldY = (minY + maxY) / 2
  const originX = width  / 2 - midWorldX * scale
  const originY = height / 2 + midWorldY * scale

  return { scale, originX, originY }
}

function drawGrid(ctx: CanvasRenderingContext2D, t: ViewTransform, w: number, h: number) {
  ctx.strokeStyle = GRID_COLOR
  ctx.lineWidth = 1

  // 1-metre grid spacing; ensure at least 4 lines visible
  const step = 1  // metres per grid line
  const [wMinX] = canvasToWorld(0, 0, t)
  const [wMaxX] = canvasToWorld(w, 0, t)
  const [, wMinY] = canvasToWorld(0, h, t)
  const [, wMaxY] = canvasToWorld(0, 0, t)

  ctx.beginPath()
  for (let x = Math.floor(wMinX); x <= Math.ceil(wMaxX); x += step) {
    const [cx] = worldToCanvas(x, 0, t)
    ctx.moveTo(cx, 0)
    ctx.lineTo(cx, h)
  }
  for (let y = Math.floor(wMinY); y <= Math.ceil(wMaxY); y += step) {
    const [, cy] = worldToCanvas(0, y, t)
    ctx.moveTo(0, cy)
    ctx.lineTo(w, cy)
  }
  ctx.stroke()
}

function drawScaleBar(ctx: CanvasRenderingContext2D, t: ViewTransform, h: number) {
  // Draw a 1-metre scale bar at bottom-left
  const barMetres = 1
  const barPx = barMetres * t.scale
  const x0 = 16, y0 = h - 16
  ctx.strokeStyle = 'rgba(255,255,255,0.4)'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(x0, y0)
  ctx.lineTo(x0 + barPx, y0)
  ctx.moveTo(x0, y0 - 4)
  ctx.lineTo(x0, y0 + 4)
  ctx.moveTo(x0 + barPx, y0 - 4)
  ctx.lineTo(x0 + barPx, y0 + 4)
  ctx.stroke()
  ctx.fillStyle = 'rgba(255,255,255,0.5)'
  ctx.font = '10px "JetBrains Mono", monospace'
  ctx.fillText('1 m', x0 + barPx + 6, y0 + 4)
}

function drawPath(
  ctx: CanvasRenderingContext2D,
  path: PathPoint[],
  t: ViewTransform,
) {
  if (path.length < 2) return

  // Path line
  ctx.strokeStyle = PATH_COLOR
  ctx.lineWidth = 2
  ctx.setLineDash([])
  ctx.beginPath()
  const [x0, y0] = worldToCanvas(path[0].x, path[0].y, t)
  ctx.moveTo(x0, y0)
  for (let i = 1; i < path.length; i++) {
    const [px, py] = worldToCanvas(path[i].x, path[i].y, t)
    ctx.lineTo(px, py)
  }
  ctx.stroke()

  // Waypoint circles
  ctx.fillStyle = PATH_COLOR
  for (const p of path) {
    const [cx, cy] = worldToCanvas(p.x, p.y, t)
    ctx.beginPath()
    ctx.arc(cx, cy, 4, 0, Math.PI * 2)
    ctx.fill()
  }

  // Direction arrows every ARROW_INTERVAL segments
  ctx.fillStyle = PATH_COLOR
  for (let i = ARROW_INTERVAL; i < path.length; i += ARROW_INTERVAL) {
    const prev = path[i - 1]
    const curr = path[i]
    const [ax, ay] = worldToCanvas(curr.x, curr.y, t)
    const angle = Math.atan2(-(curr.y - prev.y), curr.x - prev.x)
    ctx.save()
    ctx.translate(ax, ay)
    ctx.rotate(-angle)
    ctx.beginPath()
    ctx.moveTo(6, 0)
    ctx.lineTo(-4, -4)
    ctx.lineTo(-4, 4)
    ctx.closePath()
    ctx.fill()
    ctx.restore()
  }
}

function drawGoal(ctx: CanvasRenderingContext2D, path: PathPoint[], t: ViewTransform) {
  if (path.length === 0) return
  const last = path[path.length - 1]
  const [cx, cy] = worldToCanvas(last.x, last.y, t)
  ctx.strokeStyle = GOAL_COLOR
  ctx.lineWidth = 2
  ctx.fillStyle = GOAL_COLOR
  ctx.beginPath()
  ctx.arc(cx, cy, 6, 0, Math.PI * 2)
  ctx.fill()
  // Crosshair rings
  ctx.beginPath()
  ctx.arc(cx, cy, 12, 0, Math.PI * 2)
  ctx.stroke()
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  rx: number,
  ry: number,
  yaw: number,
  t: ViewTransform,
) {
  const [cx, cy] = worldToCanvas(rx, ry, t)
  // Body circle
  ctx.fillStyle = ROBOT_COLOR
  ctx.shadowColor = 'rgba(0,255,136,0.4)'
  ctx.shadowBlur = 8
  ctx.beginPath()
  ctx.arc(cx, cy, 6, 0, Math.PI * 2)
  ctx.fill()
  ctx.shadowBlur = 0

  // Heading triangle
  ctx.save()
  ctx.translate(cx, cy)
  ctx.rotate(-yaw)
  ctx.fillStyle = ROBOT_COLOR
  ctx.beginPath()
  ctx.moveTo(12, 0)
  ctx.lineTo(6, -5)
  ctx.lineTo(6, 5)
  ctx.closePath()
  ctx.fill()
  ctx.restore()
}

function drawTrail(
  ctx: CanvasRenderingContext2D,
  trail: Array<[number, number]>,
  t: ViewTransform,
) {
  ctx.fillStyle = TRAIL_COLOR
  for (const [wx, wy] of trail) {
    const [cx, cy] = worldToCanvas(wx, wy, t)
    ctx.beginPath()
    ctx.arc(cx, cy, 2, 0, Math.PI * 2)
    ctx.fill()
  }
}

export function PathView({ sseState, showToast }: PathViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const wrapperRef = useRef<HTMLDivElement>(null)
  const trailRef = useRef<Array<[number, number]>>([])
  const transformRef = useRef<ViewTransform>({ scale: 40, originX: 0, originY: 0 })
  const [hoverCoords, setHoverCoords] = useState<[number, number] | null>(null)
  const [pathLength, setPathLength] = useState(0)

  const rawPath = sseState.globalPath?.points ?? []
  const path = rawPath.filter(
    (p): p is PathPoint =>
      p != null && typeof p.x === 'number' && typeof p.y === 'number'
  )
  const odom   = sseState.odometry
  const robotX = typeof odom?.x === 'number' ? odom.x : 0
  const robotY = typeof odom?.y === 'number' ? odom.y : 0
  const yaw    = typeof odom?.yaw === 'number' ? odom.yaw : 0

  // Accumulate position trail
  useEffect(() => {
    if (odom == null) return
    trailRef.current.push([odom.x, odom.y])
    if (trailRef.current.length > TRAIL_MAX) {
      trailRef.current.shift()
    }
  }, [odom])

  // Update path point count for stats
  useEffect(() => {
    setPathLength(path.length)
  }, [path.length])

  // Main render loop
  const render = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const w = canvas.width
    const h = canvas.height
    if (w === 0 || h === 0) return

    // Compute auto-fit transform
    const t = computeTransform(path, robotX, robotY, w, h)
    transformRef.current = t

    // Clear
    ctx.clearRect(0, 0, w, h)
    ctx.fillStyle = BG_COLOR
    ctx.fillRect(0, 0, w, h)

    drawGrid(ctx, t, w, h)
    drawTrail(ctx, trailRef.current, t)
    drawPath(ctx, path, t)
    if (path.length > 0) drawGoal(ctx, path, t)
    drawRobot(ctx, robotX, robotY, yaw, t)
    drawScaleBar(ctx, t, h)
  }, [path, robotX, robotY, yaw])

  // Resize canvas to match wrapper and re-render
  useEffect(() => {
    const wrapper = wrapperRef.current
    const canvas  = canvasRef.current
    if (!wrapper || !canvas) return

    const observer = new ResizeObserver(entries => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect
        canvas.width  = Math.round(width)
        canvas.height = Math.round(height)
        render()
      }
    })
    observer.observe(wrapper)
    return () => observer.disconnect()
  }, [render])

  // Re-render on state changes
  useEffect(() => {
    render()
  }, [render])

  // Click on canvas → send goal to that world position
  const handleCanvasClick = useCallback(async (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const cx = (e.clientX - rect.left) * (canvas.width  / rect.width)
    const cy = (e.clientY - rect.top)  * (canvas.height / rect.height)
    const [wx, wy] = canvasToWorld(cx, cy, transformRef.current)
    try {
      await api.sendGoal(wx, wy)
      showToast(`导航目标: (${wx.toFixed(2)}, ${wy.toFixed(2)})`, 'success')
    } catch {
      showToast('发送目标失败', 'error')
    }
  }, [showToast])

  const handleMouseMove = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const cx = (e.clientX - rect.left) * (canvas.width  / rect.width)
    const cy = (e.clientY - rect.top)  * (canvas.height / rect.height)
    setHoverCoords(canvasToWorld(cx, cy, transformRef.current))
  }, [])

  const handleMouseLeave = useCallback(() => setHoverCoords(null), [])

  const handleClearTrail = useCallback(() => {
    trailRef.current = []
    render()
  }, [render])

  return (
    <div className={styles.container}>
      <div className={styles.toolbar}>
        <span className={styles.toolbarTitle}>轨迹视图</span>
        <span className={pathLength > 0 ? styles.badge : `${styles.badge} ${styles.badgeEmpty}`}>
          {pathLength > 0 ? `路径 ${pathLength} 点` : '无路径'}
        </span>
        <button className={styles.btnGhost} onClick={handleClearTrail}>清除轨迹</button>
      </div>

      <div className={styles.canvasWrapper} ref={wrapperRef}>
        <canvas
          ref={canvasRef}
          className={styles.canvas}
          onClick={handleCanvasClick}
          onMouseMove={handleMouseMove}
          onMouseLeave={handleMouseLeave}
        />
        {hoverCoords && (
          <div className={styles.coords}>
            {hoverCoords[0].toFixed(2)}, {hoverCoords[1].toFixed(2)} m
          </div>
        )}
        <div className={styles.hint}>点击地图发送导航目标</div>
      </div>

      <div className={styles.statsRow}>
        <div className={styles.stat}>
          <span className={styles.statLabel}>机器人位置</span>
          <span className={styles.statValue}>
            {robotX.toFixed(2)}, {robotY.toFixed(2)} m
          </span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>航向</span>
          <span className={styles.statValue}>
            {((yaw * 180) / Math.PI).toFixed(1)}°
          </span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>路径点数</span>
          <span className={styles.statValue}>{pathLength}</span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>历史轨迹</span>
          <span className={styles.statValue}>{trailRef.current.length}</span>
        </div>
      </div>
    </div>
  )
}
