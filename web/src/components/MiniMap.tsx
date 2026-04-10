import { useRef, useEffect, useCallback } from 'react'
import type { SSEState, PathPoint } from '../types'
import styles from './MiniMap.module.css'

interface MiniMapProps {
  sseState: SSEState
}

// Keep trail history across renders via module-level ref pattern
const TRAIL_MAX = 200

// World → canvas for the mini map (fixed viewport, no pan/zoom interaction)
function worldToCanvas(
  wx: number, wy: number,
  originX: number, originY: number,
  scale: number,
): [number, number] {
  return [originX + wx * scale, originY - wy * scale]
}

function computeTransform(
  robotX: number, robotY: number,
  trail: Array<[number, number]>,
  path: PathPoint[],
  w: number, h: number,
) {
  const xs = [robotX]
  const ys = [robotY]
  for (const [tx, ty] of trail) { xs.push(tx); ys.push(ty) }
  for (const p of path) { xs.push(p.x); ys.push(p.y) }

  const minX = Math.min(...xs)
  const maxX = Math.max(...xs)
  const minY = Math.min(...ys)
  const maxY = Math.max(...ys)
  const spanX = Math.max(maxX - minX, 6)
  const spanY = Math.max(maxY - minY, 6)

  const pad = 0.18
  const scale = Math.min(
    (w * (1 - 2 * pad)) / spanX,
    (h * (1 - 2 * pad)) / spanY,
  )
  return {
    scale,
    originX: w / 2 - ((minX + maxX) / 2) * scale,
    originY: h / 2 + ((minY + maxY) / 2) * scale,
  }
}

function drawScene(
  ctx: CanvasRenderingContext2D,
  w: number, h: number,
  robotX: number, robotY: number, yaw: number,
  trail: Array<[number, number]>,
  path: PathPoint[],
) {
  ctx.clearRect(0, 0, w, h)

  const { scale, originX, originY } = computeTransform(robotX, robotY, trail, path, w, h)

  const toC = (wx: number, wy: number) =>
    worldToCanvas(wx, wy, originX, originY, scale)

  // ── Grid (subtle, 2m spacing at this scale) ──
  const cellWorld = 2
  const startX = Math.floor((0 - originX) / scale / cellWorld) * cellWorld
  const endX = Math.ceil((w - originX) / scale / cellWorld) * cellWorld
  const startY = Math.floor(-(originY / scale) / cellWorld) * cellWorld
  const endY = Math.ceil(((h - originY) / scale) / cellWorld) * cellWorld

  ctx.strokeStyle = 'rgba(255,255,255,0.04)'
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let x = startX; x <= endX; x += cellWorld) {
    const [cx] = toC(x, 0)
    ctx.moveTo(cx, 0); ctx.lineTo(cx, h)
  }
  for (let y = startY; y <= endY; y += cellWorld) {
    const [, cy] = toC(0, y)
    ctx.moveTo(0, cy); ctx.lineTo(w, cy)
  }
  ctx.stroke()

  // ── Trail ──
  if (trail.length >= 2) {
    ctx.strokeStyle = 'rgba(94, 234, 212, 0.5)'
    ctx.lineWidth = 1.5
    ctx.lineCap = 'round'
    ctx.lineJoin = 'round'
    ctx.shadowColor = 'rgba(94, 234, 212, 0.3)'
    ctx.shadowBlur = 3
    ctx.beginPath()
    const [x0, y0] = toC(trail[0][0], trail[0][1])
    ctx.moveTo(x0, y0)
    for (let i = 1; i < trail.length; i++) {
      const [px, py] = toC(trail[i][0], trail[i][1])
      ctx.lineTo(px, py)
    }
    ctx.stroke()
    ctx.shadowBlur = 0
  }

  // ── Planned path ──
  if (path.length >= 2) {
    const [x0, y0] = toC(path[0].x, path[0].y)
    const last = path[path.length - 1]
    const [xn, yn] = toC(last.x, last.y)
    const grad = ctx.createLinearGradient(x0, y0, xn, yn)
    grad.addColorStop(0, 'rgba(99,102,241,0.85)')
    grad.addColorStop(0.5, 'rgba(168,85,247,0.85)')
    grad.addColorStop(1, 'rgba(236,72,153,0.85)')
    ctx.strokeStyle = grad
    ctx.lineWidth = 2
    ctx.lineCap = 'round'
    ctx.lineJoin = 'round'
    ctx.shadowColor = 'rgba(168,85,247,0.4)'
    ctx.shadowBlur = 6
    ctx.beginPath()
    ctx.moveTo(x0, y0)
    for (let i = 1; i < path.length; i++) {
      const [px, py] = toC(path[i].x, path[i].y)
      ctx.lineTo(px, py)
    }
    ctx.stroke()
    ctx.shadowBlur = 0

    // Goal dot
    ctx.strokeStyle = 'rgba(251,191,36,0.5)'
    ctx.lineWidth = 1
    ctx.beginPath(); ctx.arc(xn, yn, 5, 0, Math.PI * 2); ctx.stroke()
    ctx.fillStyle = 'rgba(251,191,36,0.9)'
    ctx.beginPath(); ctx.arc(xn, yn, 2.5, 0, Math.PI * 2); ctx.fill()
  }

  // ── Robot arrow ──
  const [rx, ry] = toC(robotX, robotY)
  ctx.save()
  ctx.translate(rx, ry)
  ctx.rotate(-yaw) // canvas Y-inverted, so negate yaw
  const sz = 7
  ctx.shadowColor = 'rgba(99,102,241,0.7)'
  ctx.shadowBlur = 8
  ctx.fillStyle = '#6366F1'
  ctx.beginPath()
  ctx.moveTo(0, -sz)
  ctx.lineTo(sz * 0.6, sz * 0.7)
  ctx.lineTo(0, sz * 0.3)
  ctx.lineTo(-sz * 0.6, sz * 0.7)
  ctx.closePath()
  ctx.fill()
  ctx.shadowBlur = 0
  ctx.restore()
}

export function MiniMap({ sseState }: MiniMapProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const trailRef = useRef<Array<[number, number]>>([])
  const prevOdomRef = useRef<{ x: number; y: number } | null>(null)

  const render = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const dpr = window.devicePixelRatio || 1
    const w = canvas.clientWidth
    const h = canvas.clientHeight
    if (w === 0 || h === 0) return

    if (canvas.width !== Math.round(w * dpr) || canvas.height !== Math.round(h * dpr)) {
      canvas.width = Math.round(w * dpr)
      canvas.height = Math.round(h * dpr)
    }

    const ctx = canvas.getContext('2d')
    if (!ctx) return
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0)

    const odom = sseState.odometry
    const robotX = odom?.x ?? 0
    const robotY = odom?.y ?? 0
    const yaw = odom?.yaw ?? 0
    const path = sseState.globalPath?.points ?? []

    drawScene(ctx, w, h, robotX, robotY, yaw, trailRef.current, path)
  }, [sseState])

  // Append trail point when robot moves
  useEffect(() => {
    const odom = sseState.odometry
    if (!odom) return
    const prev = prevOdomRef.current
    const dx = prev ? Math.abs(odom.x - prev.x) : Infinity
    const dy = prev ? Math.abs(odom.y - prev.y) : Infinity
    // Only record if moved >0.1 m
    if (dx > 0.1 || dy > 0.1) {
      trailRef.current.push([odom.x, odom.y])
      if (trailRef.current.length > TRAIL_MAX) {
        trailRef.current.shift()
      }
      prevOdomRef.current = { x: odom.x, y: odom.y }
    }
  }, [sseState.odometry])

  // Re-render on any relevant state change
  useEffect(() => {
    render()
  }, [render])

  // ResizeObserver for DPR-aware redraws
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ro = new ResizeObserver(() => render())
    ro.observe(canvas)
    return () => ro.disconnect()
  }, [render])

  return (
    <div className={styles.card}>
      <div className={styles.header}>
        <span className={styles.label}>Mini Map</span>
        <span className={styles.sub}>
          {sseState.odometry
            ? `${sseState.odometry.x.toFixed(1)}, ${sseState.odometry.y.toFixed(1)} m`
            : '-- m'}
        </span>
      </div>
      <div className={styles.canvasWrap}>
        <canvas ref={canvasRef} className={styles.canvas} />
        {!sseState.odometry && (
          <div className={styles.offline}>无定位数据</div>
        )}
      </div>
    </div>
  )
}
