import { useState, useEffect, useCallback, useRef } from 'react'
import { Map, FolderOpen, Trash2, Star, RefreshCw, Save, Pencil } from 'lucide-react'
import type { MapInfo, ToastKind } from '../types'
import * as api from '../services/api'
import { PointCloudViewer } from './PointCloudViewer'
import styles from './MapView.module.css'

interface MapViewProps {
  showToast: (msg: string, kind?: ToastKind) => void
}

// ── Map type classification ────────────────────────────────────
// 语义地图  — has PCD + Tomogram (PCT planner ready)
// 三维点云  — has PCD, no tomogram (raw LiDAR map)
// 空地图    — no PCD

interface Group { label: string; hint: string; maps: MapInfo[] }

function groupMaps(maps: MapInfo[]): Group[] {
  return [
    {
      label: '语义地图',
      hint: '含 Tomogram，可直接用于导航规划',
      maps: maps.filter(m => m.has_pcd && m.has_tomogram),
    },
    {
      label: '三维点云',
      hint: '原始 LiDAR 点云，需建 Tomogram 后才能导航',
      maps: maps.filter(m => m.has_pcd && !m.has_tomogram),
    },
    {
      label: '空地图',
      hint: '尚未采集数据',
      maps: maps.filter(m => !m.has_pcd),
    },
  ].filter(g => g.maps.length > 0)
}

// ── Map card ───────────────────────────────────────────────────
interface CardProps {
  m: MapInfo
  selected: boolean
  onPreview:  (name: string) => void
  onActivate: (name: string) => void
  onRename:   (name: string) => void
  onDelete:   (name: string) => void
}
function MapCard({ m, selected, onPreview, onActivate, onRename, onDelete }: CardProps) {
  return (
    <div className={[
      m.is_active ? styles.cardActive : styles.card,
      selected    ? styles.cardSelected : '',
    ].filter(Boolean).join(' ')}>
      <div className={styles.cardInfo}>
        <span className={styles.mapName}>
          {m.is_active && <Star size={12} className={styles.starIcon} />}
          {m.name}
        </span>
        <span className={styles.meta}>
          {m.has_pcd      && <span className={styles.tagPcd}>PCD</span>}
          {m.has_tomogram && <span className={styles.tagTomo}>Tomogram</span>}
          {!m.has_pcd && !m.has_tomogram && <span className={styles.tagEmpty}>空</span>}
          {!!m.patch_count && <span className={styles.tagEmpty}>{m.patch_count} patches</span>}
          {!!m.size_mb    && <span className={styles.tagEmpty}>{m.size_mb.toFixed(1)} MB</span>}
        </span>
      </div>
      <div className={styles.cardActions}>
        {m.has_pcd && (
          <button
            className={selected ? styles.btnTinyActive : styles.btnTiny}
            onClick={() => onPreview(m.name)}
            title="3D 点云预览"
          >预览</button>
        )}
        <button className={styles.btnTiny} onClick={() => onRename(m.name)} title="重命名">
          <Pencil size={11} />
        </button>
        {!m.is_active && (
          <button className={styles.btnTinyAccent} onClick={() => onActivate(m.name)} title="激活">
            <Star size={11} /> 激活
          </button>
        )}
        <button className={styles.btnTinyDanger} onClick={() => onDelete(m.name)} title="删除">
          <Trash2 size={11} />
        </button>
      </div>
    </div>
  )
}

// ── Main ───────────────────────────────────────────────────────
export function MapView({ showToast }: MapViewProps) {
  const [maps,        setMaps       ] = useState<MapInfo[]>([])
  const [loading,     setLoading    ] = useState(true)
  const [error,       setError      ] = useState('')
  const [selectedMap, setSelectedMap] = useState<string | null>(null)
  const [splitPct,    setSplitPct   ] = useState(42)

  // Resizable divider
  const containerRef = useRef<HTMLDivElement>(null)
  const divDragRef   = useRef<{ startX: number; startPct: number } | null>(null)

  useEffect(() => {
    const onMove = (e: MouseEvent) => {
      if (!divDragRef.current || !containerRef.current) return
      const rect = containerRef.current.getBoundingClientRect()
      const pct  = ((e.clientX - rect.left) / rect.width) * 100
      setSplitPct(Math.max(20, Math.min(72, pct)))
    }
    const onUp = () => { divDragRef.current = null }
    window.addEventListener('mousemove', onMove)
    window.addEventListener('mouseup',   onUp)
    return () => {
      window.removeEventListener('mousemove', onMove)
      window.removeEventListener('mouseup',   onUp)
    }
  }, [])

  const onDividerDown = (e: React.MouseEvent) => {
    e.preventDefault()
    divDragRef.current = { startX: e.clientX, startPct: splitPct }
  }

  // Data
  const loadMaps = useCallback(async () => {
    setLoading(true); setError('')
    try { setMaps(await api.fetchMaps()) }
    catch (e: unknown) {
      setError(`无法获取地图列表: ${e instanceof Error ? e.message : String(e)}`)
      setMaps([])
    } finally { setLoading(false) }
  }, [])

  useEffect(() => { loadMaps() }, [loadMaps])

  const handleActivate = async (name: string) => {
    try { await api.activateMap(name); showToast(`已激活: ${name}`, 'success'); loadMaps() }
    catch { showToast(`激活失败: ${name}`, 'error') }
  }
  const handleDelete = async (name: string) => {
    if (!confirm(`确定删除地图 "${name}"？`)) return
    try {
      await api.deleteMap(name); showToast(`已删除: ${name}`, 'success')
      if (selectedMap === name) setSelectedMap(null); loadMaps()
    } catch { showToast(`删除失败: ${name}`, 'error') }
  }
  const handleRename = async (oldName: string) => {
    const n = prompt(`重命名地图 "${oldName}":`, oldName)
    if (!n || n === oldName) return
    try {
      await api.renameMap(oldName, n); showToast(`已重命名: ${n}`, 'success')
      if (selectedMap === oldName) setSelectedMap(n); loadMaps()
    } catch { showToast('重命名失败', 'error') }
  }
  const handleSave = async () => {
    const n = prompt('输入地图名称:'); if (!n) return
    try { await api.saveMap(n); showToast(`保存成功: ${n}`, 'success'); loadMaps() }
    catch { showToast('保存失败', 'error') }
  }

  const togglePreview = (name: string) =>
    setSelectedMap(prev => prev === name ? null : name)

  const groups = groupMaps(maps)

  return (
    <div className={styles.mapTab} ref={containerRef}>
      {/* Left: 3D viewer */}
      <div className={styles.viewerPanel} style={{ width: `${splitPct}%` }}>
        <PointCloudViewer mapName={selectedMap} />
      </div>

      {/* Drag divider */}
      <div
        className={styles.divider}
        onMouseDown={onDividerDown}
        title="拖拽调整宽度"
      />

      {/* Right: categorized map list */}
      <div className={styles.listPanel}>
        <div className={styles.listHeader}>
          <h2 className={styles.listTitle}><Map size={15} /> 地图管理</h2>
          <div className={styles.listActions}>
            <button className={styles.btnSmallAccent} onClick={handleSave}>
              <Save size={13} /> 保存当前地图
            </button>
            <button className={styles.btnSmall} onClick={loadMaps}>
              <RefreshCw size={13} /> 刷新
            </button>
          </div>
        </div>

        <div className={styles.listScroll}>
          {loading && <div className={styles.stateMsg}>加载中...</div>}
          {error   && (
            <div className={styles.stateMsg}>
              <p>{error}</p>
              <button className={styles.btnSmall} onClick={loadMaps}>
                <RefreshCw size={13} /> 重试
              </button>
            </div>
          )}
          {!loading && !error && maps.length === 0 && (
            <div className={styles.stateMsg}>
              <FolderOpen size={40} strokeWidth={1} />
              <p>暂无保存的地图</p>
            </div>
          )}

          {!loading && groups.map(g => (
            <div key={g.label} className={styles.group}>
              <div className={styles.groupHeader}>
                <span className={styles.groupLabel}>{g.label}</span>
                <span className={styles.groupCount}>{g.maps.length}</span>
                <span className={styles.groupHint}>{g.hint}</span>
              </div>
              <div className={styles.list}>
                {g.maps.map(m => (
                  <MapCard
                    key={m.name} m={m} selected={selectedMap === m.name}
                    onPreview={togglePreview} onActivate={handleActivate}
                    onRename={handleRename}   onDelete={handleDelete}
                  />
                ))}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  )
}
