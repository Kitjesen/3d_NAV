import { useRef, useEffect, useCallback, useState } from 'react'
import {
  Compass, Grid3x3, Navigation, Route, Target, Bot,
  PanelLeftClose, PanelLeftOpen, Save, Trash2, StopCircle,
  MapPinned, Cloud,
} from 'lucide-react'
import type { SSEState, MapInfo, PathPoint, ToastKind } from '../types'
import * as api from '../services/api'
import { PromptModal } from './Modal'
import styles from './SceneView.module.css'

interface SceneViewProps {
  sseState: SSEState
  showToast: (msg: string, kind?: ToastKind) => void
}

interface ViewTransform {
  scale: number
  originX: number
  originY: number
}

// ── Layer flags ────────────────────────────────────────────────
interface Layers {
  grid:       boolean
  cloud:      boolean
  trail:      boolean
  path:       boolean
  goal:       boolean
  robot:      boolean
}

// ── World ↔ canvas helpers ─────────────────────────────────────
const worldToCanvas = (wx: number, wy: number, t: ViewTransform): [number, number] => [
  t.originX + wx * t.scale,
  t.originY - wy * t.scale,
]

const canvasToWorld = (cx: number, cy: number, t: ViewTransform): [number, number] => [
  (cx - t.originX) / t.scale,
  -(cy - t.originY) / t.scale,
]

function computeTransform(
  points: PathPoint[],
  robotX: number,
  robotY: number,
  trail: Array<[number, number]>,
  w: number,
  h: number,
): ViewTransform {
  const xs = [robotX]
  const ys = [robotY]
  for (const p of points) { xs.push(p.x); ys.push(p.y) }
  for (const [tx, ty] of trail) { xs.push(tx); ys.push(ty) }

  const minX = Math.min(...xs)
  const maxX = Math.max(...xs)
  const minY = Math.min(...ys)
  const maxY = Math.max(...ys)
  const spanX = Math.max(maxX - minX, 8)
  const spanY = Math.max(maxY - minY, 8)

  const padding = 0.15
  const scaleX = (w * (1 - 2 * padding)) / spanX
  const scaleY = (h * (1 - 2 * padding)) / spanY
  const scale = Math.min(scaleX, scaleY)

  const midX = (minX + maxX) / 2
  const midY = (minY + maxY) / 2
  return {
    scale,
    originX: w / 2 - midX * scale,
    originY: h / 2 + midY * scale,
  }
}

// ── Drawing ────────────────────────────────────────────────────

function drawCloud(
  ctx: CanvasRenderingContext2D,
  flat: number[],
  t: ViewTransform,
) {
  if (flat.length < 3) return
  ctx.fillStyle = 'rgba(56, 189, 248, 0.55)'  // sky-blue tint
  for (let i = 0; i + 2 < flat.length; i += 3) {
    const [cx, cy] = worldToCanvas(flat[i], flat[i + 1], t)
    ctx.fillRect(cx - 0.5, cy - 0.5, 1, 1)
  }
}

function drawGrid(ctx: CanvasRenderingContext2D, t: ViewTransform, w: number, h: number) {
  // Major grid every 5m, minor every 1m
  const [wMinX] = canvasToWorld(0, 0, t)
  const [wMaxX] = canvasToWorld(w, 0, t)
  const [, wMinY] = canvasToWorld(0, h, t)
  const [, wMaxY] = canvasToWorld(0, 0, t)

  // Minor grid (1 m)
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)'
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let x = Math.floor(wMinX); x <= Math.ceil(wMaxX); x++) {
    const [cx] = worldToCanvas(x, 0, t)
    ctx.moveTo(cx, 0); ctx.lineTo(cx, h)
  }
  for (let y = Math.floor(wMinY); y <= Math.ceil(wMaxY); y++) {
    const [, cy] = worldToCanvas(0, y, t)
    ctx.moveTo(0, cy); ctx.lineTo(w, cy)
  }
  ctx.stroke()

  // Major grid (5 m)
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.07)'
  ctx.lineWidth = 1
  ctx.beginPath()
  for (let x = Math.ceil(wMinX / 5) * 5; x <= wMaxX; x += 5) {
    const [cx] = worldToCanvas(x, 0, t)
    ctx.moveTo(cx, 0); ctx.lineTo(cx, h)
  }
  for (let y = Math.ceil(wMinY / 5) * 5; y <= wMaxY; y += 5) {
    const [, cy] = worldToCanvas(0, y, t)
    ctx.moveTo(0, cy); ctx.lineTo(w, cy)
  }
  ctx.stroke()

  // Origin axes — indigo X, magenta Y
  const [ox, oy] = worldToCanvas(0, 0, t)
  ctx.lineWidth = 1.5
  // X (world +X → right)
  ctx.strokeStyle = 'rgba(99, 102, 241, 0.5)'
  ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox + 30, oy); ctx.stroke()
  // Y (world +Y → up on canvas)
  ctx.strokeStyle = 'rgba(236, 72, 153, 0.5)'
  ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox, oy - 30); ctx.stroke()
}

function drawTrail(
  ctx: CanvasRenderingContext2D,
  trail: Array<[number, number]>,
  t: ViewTransform,
) {
  if (trail.length < 2) return
  // Gradient stroke
  ctx.strokeStyle = 'rgba(94, 234, 212, 0.55)'
  ctx.lineWidth = 2
  ctx.lineCap = 'round'
  ctx.lineJoin = 'round'
  ctx.shadowColor = 'rgba(94, 234, 212, 0.35)'
  ctx.shadowBlur = 4
  ctx.beginPath()
  const [x0, y0] = worldToCanvas(trail[0][0], trail[0][1], t)
  ctx.moveTo(x0, y0)
  for (let i = 1; i < trail.length; i++) {
    const [px, py] = worldToCanvas(trail[i][0], trail[i][1], t)
    ctx.lineTo(px, py)
  }
  ctx.stroke()
  ctx.shadowBlur = 0
}

function drawPath(
  ctx: CanvasRenderingContext2D,
  path: PathPoint[],
  t: ViewTransform,
) {
  if (path.length < 2) return
  // Glowing gradient line (indigo → pink)
  const [x0, y0] = worldToCanvas(path[0].x, path[0].y, t)
  const last = path[path.length - 1]
  const [xn, yn] = worldToCanvas(last.x, last.y, t)

  const grad = ctx.createLinearGradient(x0, y0, xn, yn)
  grad.addColorStop(0, 'rgba(99, 102, 241, 0.95)')
  grad.addColorStop(0.5, 'rgba(168, 85, 247, 0.95)')
  grad.addColorStop(1, 'rgba(236, 72, 153, 0.95)')

  ctx.strokeStyle = grad
  ctx.lineWidth = 3
  ctx.lineCap = 'round'
  ctx.lineJoin = 'round'
  ctx.shadowColor = 'rgba(168, 85, 247, 0.5)'
  ctx.shadowBlur = 10
  ctx.beginPath()
  ctx.moveTo(x0, y0)
  for (let i = 1; i < path.length; i++) {
    const [px, py] = worldToCanvas(path[i].x, path[i].y, t)
    ctx.lineTo(px, py)
  }
  ctx.stroke()
  ctx.shadowBlur = 0

  // Waypoint dots every N points
  const step = Math.max(1, Math.floor(path.length / 20))
  ctx.fillStyle = '#A78BFA'
  for (let i = 0; i < path.length; i += step) {
    const [cx, cy] = worldToCanvas(path[i].x, path[i].y, t)
    ctx.beginPath()
    ctx.arc(cx, cy, 2, 0, Math.PI * 2)
    ctx.fill()
  }
}

function drawGoal(ctx: CanvasRenderingContext2D, path: PathPoint[], t: ViewTransform) {
  if (path.length === 0) return
  const last = path[path.length - 1]
  const [cx, cy] = worldToCanvas(last.x, last.y, t)

  // Outer pulse ring (static render — CSS handles real animation elsewhere)
  ctx.strokeStyle = 'rgba(251, 191, 36, 0.3)'
  ctx.lineWidth = 1
  ctx.beginPath(); ctx.arc(cx, cy, 18, 0, Math.PI * 2); ctx.stroke()

  ctx.strokeStyle = 'rgba(251, 191, 36, 0.6)'
  ctx.lineWidth = 1.5
  ctx.beginPath(); ctx.arc(cx, cy, 11, 0, Math.PI * 2); ctx.stroke()

  // Glowing core
  ctx.shadowColor = 'rgba(251, 191, 36, 0.8)'
  ctx.shadowBlur = 14
  ctx.fillStyle = '#FBBF24'
  ctx.beginPath(); ctx.arc(cx, cy, 5, 0, Math.PI * 2); ctx.fill()
  ctx.shadowBlur = 0

  // Crosshair
  ctx.strokeStyle = 'rgba(251, 191, 36, 0.9)'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(cx - 16, cy); ctx.lineTo(cx - 8, cy)
  ctx.moveTo(cx + 8, cy);  ctx.lineTo(cx + 16, cy)
  ctx.moveTo(cx, cy - 16); ctx.lineTo(cx, cy - 8)
  ctx.moveTo(cx, cy + 8);  ctx.lineTo(cx, cy + 16)
  ctx.stroke()
}

/**
 * Quadruped robot silhouette seen from above.
 * Body is a rounded rectangle with 4 leg dots at corners and
 * a gradient heading arrow. Far more polished than a triangle.
 */
function drawRobot(
  ctx: CanvasRenderingContext2D,
  rx: number,
  ry: number,
  yaw: number,
  t: ViewTransform,
) {
  const [cx, cy] = worldToCanvas(rx, ry, t)

  ctx.save()
  ctx.translate(cx, cy)
  ctx.rotate(-yaw) // canvas Y is flipped relative to world Y

  // Glow base
  const glow = ctx.createRadialGradient(0, 0, 2, 0, 0, 28)
  glow.addColorStop(0, 'rgba(244, 114, 182, 0.45)')
  glow.addColorStop(1, 'rgba(244, 114, 182, 0)')
  ctx.fillStyle = glow
  ctx.beginPath(); ctx.arc(0, 0, 28, 0, Math.PI * 2); ctx.fill()

  // Body — rounded rectangle (robot looks forward along +X)
  const bodyL = 18
  const bodyW = 10
  const r = 3.5
  ctx.shadowColor = 'rgba(244, 114, 182, 0.6)'
  ctx.shadowBlur = 6

  const bodyGrad = ctx.createLinearGradient(-bodyL / 2, 0, bodyL / 2, 0)
  bodyGrad.addColorStop(0, '#6366F1')
  bodyGrad.addColorStop(0.5, '#A855F7')
  bodyGrad.addColorStop(1, '#EC4899')
  ctx.fillStyle = bodyGrad

  ctx.beginPath()
  ctx.moveTo(-bodyL / 2 + r, -bodyW / 2)
  ctx.lineTo(bodyL / 2 - r, -bodyW / 2)
  ctx.arcTo(bodyL / 2, -bodyW / 2, bodyL / 2, -bodyW / 2 + r, r)
  ctx.lineTo(bodyL / 2, bodyW / 2 - r)
  ctx.arcTo(bodyL / 2, bodyW / 2, bodyL / 2 - r, bodyW / 2, r)
  ctx.lineTo(-bodyL / 2 + r, bodyW / 2)
  ctx.arcTo(-bodyL / 2, bodyW / 2, -bodyL / 2, bodyW / 2 - r, r)
  ctx.lineTo(-bodyL / 2, -bodyW / 2 + r)
  ctx.arcTo(-bodyL / 2, -bodyW / 2, -bodyL / 2 + r, -bodyW / 2, r)
  ctx.closePath()
  ctx.fill()
  ctx.shadowBlur = 0

  // Inner highlight stroke
  ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)'
  ctx.lineWidth = 0.8
  ctx.stroke()

  // Four legs (dots at corners)
  ctx.fillStyle = '#FFFFFF'
  const legs: Array<[number, number]> = [
    [-bodyL / 2 + 3, -bodyW / 2 - 1],
    [ bodyL / 2 - 3, -bodyW / 2 - 1],
    [-bodyL / 2 + 3,  bodyW / 2 + 1],
    [ bodyL / 2 - 3,  bodyW / 2 + 1],
  ]
  for (const [lx, ly] of legs) {
    ctx.beginPath(); ctx.arc(lx, ly, 1.6, 0, Math.PI * 2); ctx.fill()
  }

  // Heading indicator — small triangle at front
  ctx.fillStyle = '#FFFFFF'
  ctx.beginPath()
  ctx.moveTo(bodyL / 2 + 5, 0)
  ctx.lineTo(bodyL / 2, -3)
  ctx.lineTo(bodyL / 2, 3)
  ctx.closePath()
  ctx.fill()

  ctx.restore()

  // Label under robot (unrotated)
  ctx.fillStyle = 'rgba(244, 114, 182, 0.7)'
  ctx.font = '500 9px Inter, system-ui, sans-serif'
  ctx.textAlign = 'center'
  ctx.fillText('LingTu', cx, cy + 24)
}

// ── Main component ─────────────────────────────────────────────

const MAP_GROUPS: Array<{ label: string; filter: (m: MapInfo) => boolean }> = [
  { label: '语义地图', filter: m => m.has_pcd && m.has_tomogram },
  { label: '三维点云', filter: m => m.has_pcd && !m.has_tomogram },
  { label: '空地图',   filter: m => !m.has_pcd },
]

const TRAIL_MAX = 300

export function SceneView({ sseState, showToast }: SceneViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const wrapperRef = useRef<HTMLDivElement>(null)
  const trailRef = useRef<Array<[number, number]>>([])
  const transformRef = useRef<ViewTransform>({ scale: 40, originX: 0, originY: 0 })

  const [hoverCoords, setHoverCoords] = useState<[number, number] | null>(null)
  const [drawerOpen, setDrawerOpen] = useState(true)
  const [saveModalOpen, setSaveModalOpen] = useState(false)
  const [layers, setLayers] = useState<Layers>({
    grid: true, cloud: true, trail: true, path: true, goal: true, robot: true,
  })
  const [maps, setMaps] = useState<MapInfo[]>([])

  const rawPath = sseState.globalPath?.points ?? []
  const path = rawPath.filter(
    (p): p is PathPoint =>
      p != null && typeof p.x === 'number' && typeof p.y === 'number'
  )
  const odom   = sseState.odometry
  const robotX = typeof odom?.x   === 'number' ? odom.x   : 0
  const robotY = typeof odom?.y   === 'number' ? odom.y   : 0
  const yaw    = typeof odom?.yaw === 'number' ? odom.yaw : 0
  const vx     = typeof odom?.vx  === 'number' ? odom.vx  : 0

  const missionState = sseState.missionStatus?.state ?? 'IDLE'
  const missionGoal  = sseState.missionStatus?.goal
  const hasGoal      = missionState === 'EXECUTING' || missionState === 'PLANNING'
  const slamMode     = sseState.slamStatus?.mode ?? '—'
  const slamHz       = sseState.slamStatus?.slam_hz ?? 0
  const cloudFlat    = sseState.mapCloud?.points ?? []

  // ── Load map list ─────────────────────────────────────────────
  const loadMaps = useCallback(async () => {
    try {
      const data = await api.fetchMaps()
      setMaps(data)
    } catch { /* noop */ }
  }, [])

  useEffect(() => { loadMaps() }, [loadMaps])

  // ── Trail tracking ────────────────────────────────────────────
  useEffect(() => {
    if (odom == null) return
    const trail = trailRef.current
    const last = trail[trail.length - 1]
    // Only push if robot moved at least 5cm
    if (!last || Math.hypot(odom.x - last[0], odom.y - last[1]) > 0.05) {
      trail.push([odom.x, odom.y])
      if (trail.length > TRAIL_MAX) trail.shift()
    }
  }, [odom])

  // ── Render loop ───────────────────────────────────────────────
  const render = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const w = canvas.width
    const h = canvas.height
    if (w === 0 || h === 0) return

    const t = computeTransform(path, robotX, robotY, trailRef.current, w, h)
    transformRef.current = t

    ctx.clearRect(0, 0, w, h)

    if (layers.grid)  drawGrid(ctx, t, w, h)
    if (layers.cloud) drawCloud(ctx, cloudFlat, t)
    if (layers.trail) drawTrail(ctx, trailRef.current, t)
    if (layers.path)  drawPath(ctx, path, t)
    if (layers.goal)  drawGoal(ctx, path, t)
    if (layers.robot) drawRobot(ctx, robotX, robotY, yaw, t)
  }, [path, robotX, robotY, yaw, layers, cloudFlat])

  // Resize & re-render on state change
  useEffect(() => {
    const wrapper = wrapperRef.current
    const canvas  = canvasRef.current
    if (!wrapper || !canvas) return

    const observer = new ResizeObserver(entries => {
      for (const entry of entries) {
        const dpr = window.devicePixelRatio || 1
        const { width, height } = entry.contentRect
        canvas.width  = Math.round(width * dpr)
        canvas.height = Math.round(height * dpr)
        canvas.style.width  = `${width}px`
        canvas.style.height = `${height}px`
        const ctx = canvas.getContext('2d')
        if (ctx) ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
        render()
      }
    })
    observer.observe(wrapper)
    return () => observer.disconnect()
  }, [render])

  useEffect(() => { render() }, [render])

  // ── Interaction handlers ──────────────────────────────────────

  const handleCanvasClick = useCallback(async (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const dpr = window.devicePixelRatio || 1
    const cx = (e.clientX - rect.left) * (canvas.width  / rect.width)  / dpr
    const cy = (e.clientY - rect.top)  * (canvas.height / rect.height) / dpr
    const [wx, wy] = canvasToWorld(cx, cy, transformRef.current)
    try {
      await api.sendGoal(wx, wy)
      showToast(`目标已发送: (${wx.toFixed(2)}, ${wy.toFixed(2)})`, 'success')
    } catch {
      showToast('发送目标失败', 'error')
    }
  }, [showToast])

  const handleMouseMove = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const dpr = window.devicePixelRatio || 1
    const cx = (e.clientX - rect.left) * (canvas.width  / rect.width)  / dpr
    const cy = (e.clientY - rect.top)  * (canvas.height / rect.height) / dpr
    setHoverCoords(canvasToWorld(cx, cy, transformRef.current))
  }, [])

  const handleMouseLeave = useCallback(() => setHoverCoords(null), [])

  const handleClearTrail = useCallback(() => {
    trailRef.current = []
    render()
  }, [render])

  const handleSaveMap = () => setSaveModalOpen(true)

  const confirmSaveMap = async (name: string) => {
    setSaveModalOpen(false)
    try {
      await api.saveMap(name)
      showToast(`已保存: ${name}`, 'success')
      loadMaps()
    } catch { showToast('保存失败', 'error') }
  }

  const handleActivate = async (name: string) => {
    try {
      await api.activateMap(name)
      showToast(`已激活: ${name}`, 'success')
      loadMaps()
    } catch { showToast('激活失败', 'error') }
  }

  const handleStop = async () => {
    try {
      await api.sendStop()
      showToast('已停止', 'info')
    } catch { /* noop */ }
  }

  const toggleLayer = (key: keyof Layers) =>
    setLayers(prev => ({ ...prev, [key]: !prev[key] }))

  // Layer button helper
  const LayerBtn = ({
    k, icon, label, colorClass,
  }: { k: keyof Layers; icon: React.ReactNode; label: string; colorClass?: string }) => (
    <button
      className={[
        layers[k] ? styles.layerBtnActive : styles.layerBtn,
        layers[k] && colorClass ? styles[colorClass] : '',
      ].filter(Boolean).join(' ')}
      onClick={() => toggleLayer(k)}
    >
      {icon} {label}
    </button>
  )

  return (
    <div className={styles.sceneView}>
      {/* Toolbar */}
      <div className={styles.toolbar}>
        <span className={styles.toolbarTitle}>
          <span className={styles.toolbarTitleIcon}><Compass size={13} /></span>
          场景视图
        </span>

        <span className={styles.divider} />

        <div className={styles.layerGroup}>
          <LayerBtn k="grid"  icon={<Grid3x3 size={11} />}   label="网格"  colorClass="layerGrid" />
          <LayerBtn k="cloud" icon={<Cloud size={11} />}      label="点云"  colorClass="layerCloud" />
          <LayerBtn k="trail" icon={<Route size={11} />}      label="轨迹"  colorClass="layerTrail" />
          <LayerBtn k="path"  icon={<Navigation size={11} />} label="路径" colorClass="layerPath" />
          <LayerBtn k="goal"  icon={<Target size={11} />}     label="目标" colorClass="layerGoal" />
          <LayerBtn k="robot" icon={<Bot size={11} />}        label="本机" colorClass="layerRobot" />
        </div>

        <div className={styles.toolbarSpacer} />

        <button className={styles.toolbarBtn} onClick={handleClearTrail}>
          <Trash2 size={12} /> 清除轨迹
        </button>
        <button className={styles.toolbarBtnPrimary} onClick={handleSaveMap}>
          <Save size={12} /> 保存地图
        </button>
      </div>

      {/* Workspace */}
      <div className={[styles.workspace, drawerOpen ? '' : styles.drawerClosed].filter(Boolean).join(' ')}>
        {/* Left: map drawer */}
        <div className={styles.drawer}>
          <div className={styles.drawerHeader}>
            <span className={styles.drawerTitle}>地图</span>
            <button
              className={styles.drawerToggle}
              onClick={() => setDrawerOpen(!drawerOpen)}
              title={drawerOpen ? '收起' : '展开'}
            >
              {drawerOpen ? <PanelLeftClose size={14} /> : <PanelLeftOpen size={14} />}
            </button>
          </div>
          <div className={styles.drawerBody}>
            {maps.length === 0 && (
              <div className={styles.emptyState}>
                <MapPinned size={32} className={styles.emptyIcon} strokeWidth={1.4} />
                <div className={styles.emptyTitle}>暂无地图</div>
                <div className={styles.emptyHint}>保存当前场景来创建第一张地图</div>
                <button
                  type="button"
                  className={styles.emptyCta}
                  onClick={handleSaveMap}
                >
                  <Save size={11} /> 保存地图
                </button>
              </div>
            )}
            {MAP_GROUPS.map(g => {
              const groupMaps = maps.filter(g.filter)
              if (groupMaps.length === 0) return null
              return (
                <div key={g.label} className={styles.mapGroup}>
                  <div className={styles.mapGroupTitle}>{g.label}</div>
                  {groupMaps.map(m => (
                    <button
                      key={m.name}
                      className={m.is_active ? styles.mapItemActive : styles.mapItem}
                      onClick={() => handleActivate(m.name)}
                      title={m.is_active ? '已激活' : '点击激活'}
                    >
                      <span>{m.name}</span>
                      {m.has_tomogram && <span className={styles.mapBadge}>T</span>}
                    </button>
                  ))}
                </div>
              )
            })}
          </div>
        </div>

        {/* Center: canvas */}
        <div className={styles.canvasArea}>
          <div className={styles.canvasWrap} ref={wrapperRef}>
            <canvas
              ref={canvasRef}
              className={styles.canvas}
              onClick={handleCanvasClick}
              onMouseMove={handleMouseMove}
              onMouseLeave={handleMouseLeave}
            />
            <div className={styles.canvasOverlayTop}>
              <span className={styles.hintBadge}>点击任意位置发送导航目标</span>
              <span className={styles.scaleLabel}>
                1 格 = 1 m  ·  5 格 = 5 m
              </span>
            </div>
            <div className={styles.canvasOverlayBottom}>
              <span className={styles.scaleLabel}>
                +X → 前进  ·  +Y → 左侧
              </span>
              {hoverCoords && (
                <span className={styles.coords}>
                  {hoverCoords[0].toFixed(2)}, {hoverCoords[1].toFixed(2)} m
                </span>
              )}
            </div>
          </div>
        </div>

        {/* Right: side panel */}
        <div className={styles.sidePanel}>
          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>机器人状态</div>
            <div className={styles.statGrid}>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>X</span>
                <span className={styles.statValueHl}>{robotX.toFixed(2)}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>Y</span>
                <span className={styles.statValueHl}>{robotY.toFixed(2)}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>航向</span>
                <span className={styles.statValue}>
                  {((yaw * 180) / Math.PI).toFixed(0)}°
                </span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>线速度</span>
                <span className={styles.statValue}>{vx.toFixed(2)} m/s</span>
              </div>
            </div>
          </div>

          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>导航任务</div>
            <div className={styles.statGrid}>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>状态</span>
                <span className={hasGoal ? styles.goalBadgeActive : styles.goalBadgeIdle}>
                  {missionState}
                </span>
              </div>
              <div className={styles.statItem} style={{ gridColumn: '1 / span 2' }}>
                <span className={styles.statLabel}>目标</span>
                <span className={styles.statValueDim}>
                  {missionGoal ?? '无'}
                </span>
              </div>
            </div>
          </div>

          <div className={styles.statCard}>
            <div className={styles.statCardTitle}>SLAM</div>
            <div className={styles.statGrid}>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>模式</span>
                <span className={styles.statValueDim}>{slamMode}</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>频率</span>
                <span className={styles.statValue}>{slamHz.toFixed(1)} Hz</span>
              </div>
            </div>
          </div>

          <button className={styles.eStopBtn} onClick={handleStop}>
            <StopCircle size={16} />
            紧急停止
          </button>
        </div>
      </div>

      <PromptModal
        open={saveModalOpen}
        title="保存当前地图"
        message="保存当前 SLAM 建图结果。系统会自动生成导航所需的 tomogram 和 occupancy 数据。"
        placeholder="例如 building_2f"
        confirmLabel="保存"
        icon={<Save size={18} />}
        validate={(v) => {
          if (!/^[a-zA-Z0-9_-]+$/.test(v)) return '仅支持字母、数字、下划线和横线'
          if (v.length > 32) return '名称过长 (最多 32 字符)'
          return null
        }}
        onConfirm={confirmSaveMap}
        onCancel={() => setSaveModalOpen(false)}
      />
    </div>
  )
}
