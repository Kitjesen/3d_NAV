import { useRef, useEffect, useCallback, useState, memo } from 'react'
import {
  Compass, Grid3x3, Navigation, Route, Target, Bot, Layers as LayersIcon, Mountain,
  PanelLeftClose, PanelLeftOpen, Save, Trash2, StopCircle, Pencil, X,
  MapPinned, Cloud, Maximize2, Radio, Activity, LocateFixed, VideoOff,
} from 'lucide-react'
import type { SSEState, MapInfo, PathPoint, ToastKind, SlamProfile } from '../types'
import * as api from '../services/api'
import { useCamera } from '../hooks/useCamera'
import { PromptModal, ConfirmModal } from './Modal'
import { Scene3D, type Scene3DHandle } from './Scene3D'
import styles from './SceneView.module.css'

interface SceneViewProps {
  sseState:  SSEState
  showToast: (msg: string, kind?: ToastKind) => void
}

// ── Layer flags ────────────────────────────────────────────────
interface Layers {
  grid:    boolean
  cloud:   boolean
  trail:   boolean
  path:    boolean
  goal:    boolean
  robot:   boolean
  costmap: boolean
  slope:   boolean
}

const TRAIL_MAX = 300

const MAP_GROUPS: Array<{ label: string; filter: (m: MapInfo) => boolean }> = [
  { label: '语义地图', filter: m => m.has_pcd && m.has_tomogram },
  { label: '三维点云', filter: m => m.has_pcd && !m.has_tomogram },
  { label: '空地图',   filter: m => !m.has_pcd },
]

function SceneViewComponent({ sseState, showToast }: SceneViewProps) {
  const scene3DRef = useRef<Scene3DHandle>(null)

  // Trail: state so Scene3D re-renders on movement
  const [trail, setTrail] = useState<Array<[number, number]>>([])
  const prevTrailEndRef = useRef<[number, number] | null>(null)

  const [slamPending, setSlamPending] = useState<SlamProfile | null>(null)
  const [drawerOpen, setDrawerOpen] = useState(true)
  const [saveModalOpen, setSaveModalOpen] = useState(false)
  const [layers, setLayers] = useState<Layers>({
    grid: true, cloud: true, trail: true, path: true, goal: true, robot: true, costmap: false, slope: false,
  })
  const [maps, setMaps] = useState<MapInfo[]>([])
  // Default 0.12 (12cm sphere per point) — with ~18k points in a room-scale
  // scene this gives a visually dense cloud. User can shrink via slider.
  const [pointSize, setPointSize] = useState(0.12)
  const [savedMapFlat, setSavedMapFlat] = useState<number[] | undefined>(undefined)
  const [relocOpen, setRelocOpen] = useState(false)
  const [relocDropOpen, setRelocDropOpen] = useState(false)
  const [relocMap, setRelocMap] = useState('')
  const relocDropRef = useRef<HTMLDivElement>(null)
  const [relocX, setRelocX] = useState('0')
  const [relocY, setRelocY] = useState('0')
  const [relocYaw, setRelocYaw] = useState('0')
  const [relocPending, setRelocPending] = useState(false)
  const [pendingGoal, setPendingGoal] = useState<{ x: number; y: number } | null>(null)

  // Map management modals
  const [mapContextMenu, setMapContextMenu] = useState<{ name: string; x: number; y: number } | null>(null)
  const [deleteTarget, setDeleteTarget] = useState<string | null>(null)
  const [renameTarget, setRenameTarget] = useState<string | null>(null)
  const [loadTarget, setLoadTarget] = useState<string | null>(null)
  // SLAM switch confirmation
  const [slamSwitchTarget, setSlamSwitchTarget] = useState<SlamProfile | null>(null)

  const { imgSrc: cameraImgSrc, connected: cameraConnected } = useCamera()

  const rawPath = sseState.globalPath?.points ?? []
  const path = rawPath.filter(
    (p): p is PathPoint =>
      p != null && typeof p.x === 'number' && typeof p.y === 'number'
  )
  const rawLocalPath = sseState.localPath?.points ?? []
  const localPathPts = rawLocalPath.filter(
    (p): p is PathPoint =>
      p != null && typeof p.x === 'number' && typeof p.y === 'number'
  )
  const odom   = sseState.odometry
  // Sanity filter: reject absurd values (SLAM may emit garbage when lost).
  // Reasonable bounds: |pos| < 10km, |vel| < 10 m/s (quadruped max ~3 m/s).
  const sanePos = (v: unknown): number =>
    typeof v === 'number' && Number.isFinite(v) && Math.abs(v) < 10000 ? v : 0
  const saneVel = (v: unknown): number =>
    typeof v === 'number' && Number.isFinite(v) && Math.abs(v) < 10 ? v : 0
  const saneYaw = (v: unknown): number =>
    typeof v === 'number' && Number.isFinite(v) ? v : 0

  const robotX = sanePos(odom?.x)
  const robotY = sanePos(odom?.y)
  const yaw    = saneYaw(odom?.yaw)
  const vx     = saneVel(odom?.vx)
  const odomValid = odom != null && sanePos(odom.x) === odom.x && sanePos(odom.y) === odom.y

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

  // Close dropdown on outside click
  useEffect(() => {
    if (!relocDropOpen) return
    const handler = (e: MouseEvent) => {
      if (relocDropRef.current && !relocDropRef.current.contains(e.target as Node)) {
        setRelocDropOpen(false)
      }
    }
    document.addEventListener('mousedown', handler)
    return () => document.removeEventListener('mousedown', handler)
  }, [relocDropOpen])

  // ── Trail tracking ────────────────────────────────────────────
  useEffect(() => {
    if (odom == null) return
    const last = prevTrailEndRef.current
    if (!last || Math.hypot(odom.x - last[0], odom.y - last[1]) > 0.05) {
      prevTrailEndRef.current = [odom.x, odom.y]
      setTrail(prev => {
        const next = [...prev, [odom.x, odom.y] as [number, number]]
        return next.length > TRAIL_MAX ? next.slice(next.length - TRAIL_MAX) : next
      })
    }
  }, [odom])

  // ── Handlers ──────────────────────────────────────────────────
  const handlePendingGoal = useCallback((x: number, y: number) => {
    setPendingGoal({ x, y })
  }, [])

  const handleConfirmGoal = useCallback(async () => {
    if (!pendingGoal) return
    const { x, y } = pendingGoal
    setPendingGoal(null)
    try {
      await api.sendGoal(x, y)
      showToast(`目标已发送: (${x.toFixed(2)}, ${y.toFixed(2)})`, 'success')
    } catch {
      showToast('发送目标失败', 'error')
    }
  }, [pendingGoal, showToast])

  const handleClearTrail = useCallback(() => {
    setTrail([])
    prevTrailEndRef.current = null
  }, [])

  const handleClearCloud = useCallback(async () => {
    try {
      await api.resetMapCloud()
      showToast('已清除累积点云', 'success')
    } catch (e) {
      showToast(`清除失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }, [showToast])

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
    // Left click = select/activate only (highlight in list), no relocalization, no toast
    try {
      await api.activateMap(name)
      loadMaps()
    } catch { /* silent */ }
  }

  const confirmLoadMap = async () => {
    if (!loadTarget) return
    const name = loadTarget
    setLoadTarget(null)
    // Clear old saved map first — don't show stale data
    setSavedMapFlat(undefined)
    try {
      await api.activateMap(name)
      await api.relocalize(name, 0, 0, 0)
      showToast(`已加载: ${name}，重定位到原点`, 'success')
      loadMaps()
      try {
        const pts = await api.fetchSavedMapPoints(name)
        setSavedMapFlat(pts)
      } catch { /* PCD not available */ }
    } catch (e: unknown) {
      showToast(`加载失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }

  const handleDeleteMap = async () => {
    if (!deleteTarget) return
    const name = deleteTarget
    setDeleteTarget(null)
    try {
      await api.deleteMap(name)
      showToast(`已删除: ${name}`, 'success')
      loadMaps()
      if (savedMapFlat !== undefined) setSavedMapFlat(undefined)
    } catch (e: unknown) {
      showToast(`删除失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }

  const confirmRenameMap = async (newName: string) => {
    if (!renameTarget) return
    const oldName = renameTarget
    setRenameTarget(null)
    try {
      await api.renameMap(oldName, newName)
      showToast(`已重命名: ${oldName} → ${newName}`, 'success')
      loadMaps()
    } catch (e: unknown) {
      showToast(`重命名失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    }
  }

  const handleSwitchSlam = (profile: SlamProfile) => {
    if (slamPending) return
    setSlamSwitchTarget(profile)
  }

  const confirmSwitchSlam = async () => {
    if (!slamSwitchTarget) return
    const profile = slamSwitchTarget
    setSlamSwitchTarget(null)
    setSlamPending(profile)
    try {
      await api.switchSlamMode(profile)
      showToast(`已切换: ${profile === 'fastlio2' ? 'SLAM建图' : '导航巡航'}`, 'success')
    } catch (e: unknown) {
      showToast(`切换失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setSlamPending(null)
    }
  }

  const handleStop = async () => {
    try {
      await api.sendStop()
      showToast('已停止', 'info')
    } catch { /* noop */ }
  }

  const handleRelocalize = async () => {
    if (!relocMap) { showToast('请先选择地图', 'error'); return }
    setRelocPending(true)
    try {
      await api.relocalize(relocMap, parseFloat(relocX) || 0, parseFloat(relocY) || 0, parseFloat(relocYaw) || 0)
      showToast(`重定位已发起: ${relocMap}`, 'success')
      setRelocOpen(false)
      // Load saved map cloud only after relocalization (coordinate frames now aligned)
      try {
        const pts = await api.fetchSavedMapPoints(relocMap)
        setSavedMapFlat(pts)
      } catch { /* PCD not available — ignore */ }
    } catch (e: unknown) {
      showToast(`重定位失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setRelocPending(false)
    }
  }

  const toggleLayer = (key: keyof Layers) =>
    setLayers(prev => ({ ...prev, [key]: !prev[key] }))

  const LayerBtn = ({
    k, icon, label,
  }: { k: keyof Layers; icon: React.ReactNode; label: string }) => (
    <button
      className={layers[k] ? styles.layerBtnActive : styles.layerBtn}
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
          <LayerBtn k="grid"  icon={<Grid3x3 size={11} />}   label="网格"  />
          <LayerBtn k="cloud" icon={<Cloud size={11} />}      label="点云"  />
          <LayerBtn k="trail" icon={<Route size={11} />}      label="轨迹"  />
          <LayerBtn k="path"  icon={<Navigation size={11} />} label="路径"  />
          <LayerBtn k="goal"  icon={<Target size={11} />}     label="目标"  />
          <LayerBtn k="robot"   icon={<Bot size={11} />}        label="本机"  />
          <LayerBtn k="costmap" icon={<LayersIcon size={11} />} label="代价"  />
          <LayerBtn k="slope"   icon={<Mountain size={11} />}   label="坡度"  />
        </div>

        <span className={styles.divider} />

        <div className={styles.pointSizeRow} title="点云粒子大小">
          <Cloud size={10} className={styles.pointSizeIcon} />
          <input
            type="range" min={0.02} max={0.4} step={0.01}
            value={pointSize}
            onChange={e => setPointSize(parseFloat(e.target.value))}
            className={styles.pointSlider}
          />
          <span className={styles.pointSizeVal}>{pointSize.toFixed(2)}</span>
        </div>

        <div className={styles.toolbarSpacer} />

        <button className={styles.toolbarBtn} onClick={() => scene3DRef.current?.resetCamera()} title="重置到自动视野">
          <Maximize2 size={12} /> 重置视野
        </button>
        <button className={styles.toolbarBtn} onClick={handleClearTrail}>
          <Trash2 size={12} /> 清除轨迹
        </button>
        <button className={styles.toolbarBtn} onClick={handleClearCloud} title="仅清浏览器可视化，不动 SLAM ikd-tree">
          <Cloud size={12} /> 清除点云
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
            <div style={{ display: 'flex', gap: 4 }}>
              {savedMapFlat !== undefined && (
                <button
                  className={styles.drawerToggle}
                  onClick={() => setSavedMapFlat(undefined)}
                  title="清除已加载地图"
                >
                  <X size={14} />
                </button>
              )}
              <button
                className={styles.drawerToggle}
                onClick={() => setDrawerOpen(!drawerOpen)}
                title={drawerOpen ? '收起' : '展开'}
              >
                {drawerOpen ? <PanelLeftClose size={14} /> : <PanelLeftOpen size={14} />}
              </button>
            </div>
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
                      onContextMenu={(e) => {
                        e.preventDefault()
                        const rect = (e.currentTarget as HTMLElement).getBoundingClientRect()
                        setMapContextMenu({ name: m.name, x: rect.right + 4, y: rect.top })
                      }}
                      title="左键选中 · 右键管理"
                    >
                      <span>{m.name}</span>
                      {m.has_tomogram && <span className={styles.mapBadge}>T</span>}
                    </button>
                  ))}
                </div>
              )
            })}
            {/* Context menu */}
            {mapContextMenu && (
              <div
                className={styles.contextMenu}
                style={{ position: 'fixed', left: mapContextMenu.x, top: mapContextMenu.y, zIndex: 100 }}
                onClick={() => setMapContextMenu(null)}
              >
                <button className={styles.contextMenuItem} onClick={() => { setLoadTarget(mapContextMenu.name); setMapContextMenu(null) }}>
                  <LocateFixed size={12} /> 加载并重定位
                </button>
                <button className={styles.contextMenuItem} onClick={() => { setRenameTarget(mapContextMenu.name); setMapContextMenu(null) }}>
                  <Pencil size={12} /> 重命名
                </button>
                <button className={[styles.contextMenuItem, styles.contextMenuDanger].join(' ')} onClick={() => { setDeleteTarget(mapContextMenu.name); setMapContextMenu(null) }}>
                  <Trash2 size={12} /> 删除
                </button>
              </div>
            )}
          </div>
        </div>

        {/* Center: 3D scene */}
        <div className={styles.canvasArea}>
          <div className={styles.canvasWrap}>
            <Scene3D
              ref={scene3DRef}
              cloudFlat={cloudFlat}
              savedMapFlat={savedMapFlat ?? sseState.savedMap?.points}
              costmap={sseState.costmap ?? null}
              slopeGrid={sseState.slopeGrid ?? null}
              sceneGraph={sseState.sceneGraph ?? null}
              robotX={robotX}
              robotY={robotY}
              yaw={yaw}
              trail={trail}
              path={path}
              localPath={localPathPts}
              layers={layers}
              pointSize={pointSize}
              onPendingGoal={handlePendingGoal}
              pendingGoal={pendingGoal}
            />
            <div className={styles.canvasOverlayTop}>
              <span className={styles.scaleLabel}>3D 场景视图  ·  拖拽旋转  ·  滚轮缩放  ·  点击放置目标</span>
            </div>
            <div className={styles.robotOverlay}>
              <span>位置 {odomValid ? `(${robotX.toFixed(2)}, ${robotY.toFixed(2)})` : '(--, --)'}</span>
              <span>航向 {odomValid ? `${((yaw * 180) / Math.PI).toFixed(1)}°` : '--°'}</span>
              <span>速度 {odomValid ? `${vx.toFixed(2)} m/s` : '-- m/s'}</span>
              <span>{slamMode === '—' || slamMode === 'stop' ? '⚠ SLAM 离线' : missionState}</span>
            </div>
            {pendingGoal && (
              <div className={styles.goalConfirmPanel}>
                <span className={styles.goalConfirmLabel}>导航目标</span>
                <span className={styles.goalConfirmCoords}>
                  ({pendingGoal.x.toFixed(2)}, {pendingGoal.y.toFixed(2)})
                </span>
                <button className={styles.goalConfirmBtn} onClick={handleConfirmGoal}>
                  <Navigation size={12} /> 发送
                </button>
                <button className={styles.goalCancelBtn} onClick={() => setPendingGoal(null)}>
                  取消
                </button>
              </div>
            )}
            {/* Camera PiP — draggable */}
            <div
              className={styles.cameraPip}
              onMouseDown={(e) => {
                const pip = e.currentTarget
                const rect = pip.getBoundingClientRect()
                const ox = e.clientX - rect.left
                const oy = e.clientY - rect.top
                const onMove = (ev: MouseEvent) => {
                  const parent = pip.parentElement!.getBoundingClientRect()
                  pip.style.left = `${ev.clientX - parent.left - ox}px`
                  pip.style.top = `${ev.clientY - parent.top - oy}px`
                  pip.style.right = 'auto'
                  pip.style.bottom = 'auto'
                }
                const onUp = () => {
                  document.removeEventListener('mousemove', onMove)
                  document.removeEventListener('mouseup', onUp)
                  pip.style.cursor = 'grab'
                }
                pip.style.cursor = 'grabbing'
                document.addEventListener('mousemove', onMove)
                document.addEventListener('mouseup', onUp)
              }}
            >
              <div className={styles.cameraPipHeader}>
                <span className={cameraConnected ? styles.camDotLive : styles.camDotOff} />
                {cameraConnected ? '摄像头' : '无信号'}
              </div>
              {cameraImgSrc
                ? <img src={cameraImgSrc} className={styles.cameraPipImg} alt="camera" draggable={false} />
                : <div className={styles.cameraPipEmpty}><VideoOff size={18} opacity={0.35} /></div>
              }
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
            <div className={styles.statCardTitle}>
              <Radio size={11} style={{ marginRight: 4, verticalAlign: 'middle' }} />
              SLAM 模式
            </div>
            <div className={styles.slamModeGroup}>
              <button
                className={slamMode === 'fastlio2' ? styles.slamModeBtnActive : styles.slamModeBtn}
                onClick={() => handleSwitchSlam('fastlio2')}
                disabled={slamPending !== null}
              >
                <Navigation size={11} />
                建图
                {slamMode === 'fastlio2' && <span className={styles.slamDot} />}
              </button>
              <button
                className={slamMode === 'localizer' ? styles.slamModeBtnActive : styles.slamModeBtn}
                onClick={() => handleSwitchSlam('localizer')}
                disabled={slamPending !== null}
              >
                <Activity size={11} />
                巡航
                {slamMode === 'localizer' && <span className={styles.slamDot} />}
              </button>
            </div>
            <div className={styles.statGrid}>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>频率</span>
                <span className={styles.statValue}>{slamHz.toFixed(1)} Hz</span>
              </div>
              <div className={styles.statItem}>
                <span className={styles.statLabel}>点云</span>
                <span className={styles.statValue}>
                  {sseState.slamStatus?.map_points?.toLocaleString() ?? '—'}
                </span>
              </div>
            </div>

            <button
              className={styles.relocBtn}
              onClick={() => setRelocOpen(v => !v)}
            >
              <LocateFixed size={11} />
              重定位
            </button>

            {relocOpen && (
              <div className={styles.relocPanel}>
                {/* Custom dropdown */}
                <div className={styles.customSelect} ref={relocDropRef}>
                  <button
                    type="button"
                    className={styles.customSelectTrigger}
                    onClick={() => setRelocDropOpen(v => !v)}
                  >
                    <span className={relocMap ? styles.customSelectValue : styles.customSelectPlaceholder}>
                      {relocMap || '— 选择地图 —'}
                    </span>
                    <svg width="10" height="6" viewBox="0 0 10 6" fill="none" className={relocDropOpen ? styles.customSelectArrowOpen : styles.customSelectArrow}>
                      <path d="M1 1L5 5L9 1" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                    </svg>
                  </button>
                  {relocDropOpen && (
                    <div className={styles.customSelectList}>
                      {maps.length === 0 && (
                        <div className={styles.customSelectEmpty}>暂无地图</div>
                      )}
                      {maps.map(m => (
                        <button
                          key={m.name}
                          type="button"
                          className={m.name === relocMap ? styles.customSelectItemActive : styles.customSelectItem}
                          onClick={() => { setRelocMap(m.name); setRelocDropOpen(false) }}
                        >
                          {m.name}
                        </button>
                      ))}
                    </div>
                  )}
                </div>
                <div className={styles.relocInputRow}>
                  <label>X</label>
                  <input className={styles.relocInput} type="number" step="0.1"
                    value={relocX} onChange={e => setRelocX(e.target.value)} />
                  <label>Y</label>
                  <input className={styles.relocInput} type="number" step="0.1"
                    value={relocY} onChange={e => setRelocY(e.target.value)} />
                  <label>Yaw</label>
                  <input className={styles.relocInput} type="number" step="0.1"
                    value={relocYaw} onChange={e => setRelocYaw(e.target.value)} />
                </div>
                <button
                  className={styles.relocConfirmBtn}
                  onClick={handleRelocalize}
                  disabled={relocPending || !relocMap}
                >
                  {relocPending ? '定位中…' : '确认重定位'}
                </button>
              </div>
            )}
          </div>

          <button className={styles.eStopBtn} onClick={handleStop}>
            <StopCircle size={16} />
            紧急停止
          </button>
        </div>
      </div>

      {/* Click-away to close context menu */}
      {mapContextMenu && (
        <div style={{ position: 'fixed', inset: 0, zIndex: 99 }} onClick={() => setMapContextMenu(null)} />
      )}

      {/* Load map confirm */}
      <ConfirmModal
        open={loadTarget !== null}
        title="加载地图"
        message={`加载「${loadTarget ?? ''}」并重定位到此地图？\n当前实时点云将对齐到此地图的坐标系。`}
        confirmLabel="加载并重定位"
        onConfirm={confirmLoadMap}
        onCancel={() => setLoadTarget(null)}
      />

      {/* Delete map confirm */}
      <ConfirmModal
        open={deleteTarget !== null}
        title="删除地图"
        message={`确认删除「${deleteTarget ?? ''}」？此操作不可恢复。`}
        confirmLabel="删除"
        danger
        onConfirm={handleDeleteMap}
        onCancel={() => setDeleteTarget(null)}
      />

      {/* Rename map */}
      <PromptModal
        open={renameTarget !== null}
        title="重命名地图"
        message={`当前名称: ${renameTarget ?? ''}`}
        placeholder="新名称"
        initialValue={renameTarget ?? ''}
        confirmLabel="重命名"
        icon={<Pencil size={18} />}
        validate={(v) => {
          if (!/^[a-zA-Z0-9_-]+$/.test(v)) return '仅支持字母、数字、下划线和横线'
          if (v === renameTarget) return '名称未变'
          return null
        }}
        onConfirm={confirmRenameMap}
        onCancel={() => setRenameTarget(null)}
      />

      {/* SLAM switch confirm */}
      <ConfirmModal
        open={slamSwitchTarget !== null}
        title="切换 SLAM 模式"
        message={slamSwitchTarget === 'fastlio2'
          ? '切换到建图模式？当前导航巡航将停止，开始实时建图。'
          : '切换到巡航模式？需要先加载一张已有地图用于定位。'}
        confirmLabel={slamSwitchTarget === 'fastlio2' ? '开始建图' : '切换巡航'}
        onConfirm={confirmSwitchSlam}
        onCancel={() => setSlamSwitchTarget(null)}
      />

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

export const SceneView = memo(SceneViewComponent)
