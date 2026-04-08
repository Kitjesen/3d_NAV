import { useState, useEffect, useCallback } from 'react'
import { Map, FolderOpen, Trash2, Star, RefreshCw, Save, Pencil, Box } from 'lucide-react'
import type { ToastKind } from '../hooks/useToast'

interface MapInfo {
  name: string
  has_pcd: boolean
  has_tomogram: boolean
  is_active: boolean
  size_mb?: number
  patch_count?: number
}

interface MapViewProps {
  showToast: (msg: string, kind?: ToastKind) => void
}

export function MapView({ showToast }: MapViewProps) {
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')

  const fetchMaps = useCallback(async () => {
    setLoading(true)
    setError('')
    try {
      const res = await fetch('/api/v1/slam/maps')
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      const data = await res.json()
      setMaps(data.maps || [])
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e)
      setError(`无法获取地图列表: ${msg}`)
      setMaps([])
    } finally {
      setLoading(false)
    }
  }, [])

  useEffect(() => { fetchMaps() }, [fetchMaps])

  const activateMap = async (name: string) => {
    try {
      const res = await fetch('/api/v1/map/activate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name }),
      })
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      showToast(`已激活: ${name}`, 'success')
      fetchMaps()
    } catch {
      showToast(`激活失败: ${name}`, 'error')
    }
  }

  const deleteMap = async (name: string) => {
    if (!confirm(`确定删除地图 "${name}"？此操作不可恢复。`)) return
    try {
      const res = await fetch('/api/v1/maps', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ action: 'delete', name }),
      })
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      showToast(`已删除: ${name}`, 'success')
      fetchMaps()
    } catch {
      showToast(`删除失败: ${name}`, 'error')
    }
  }

  const renameMap = async (oldName: string) => {
    const newName = prompt(`重命名地图 "${oldName}":`, oldName)
    if (!newName || newName === oldName) return
    try {
      const res = await fetch('/api/v1/map/rename', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ old_name: oldName, new_name: newName }),
      })
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      showToast(`已重命名: ${newName}`, 'success')
      fetchMaps()
    } catch {
      showToast(`重命名失败`, 'error')
    }
  }

  const previewMap3D = (name: string) => {
    window.open(`/map/viewer?map=${encodeURIComponent(name)}`, '_blank', 'noopener,noreferrer')
  }

  const saveMap = async () => {
    const name = prompt('输入地图名称:')
    if (!name) return
    try {
      const res = await fetch('/api/v1/map/save', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name }),
      })
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      showToast(`保存成功: ${name}`, 'success')
      fetchMaps()
    } catch {
      showToast('保存失败', 'error')
    }
  }

  return (
    <div className="map-manager">
      <div className="map-header">
        <h2><Map size={18} /> 地图管理</h2>
        <div className="map-actions">
          <button className="btn-small btn-accent" onClick={saveMap}>
            <Save size={14} /> 保存当前地图
          </button>
          <button className="btn-small" onClick={fetchMaps}>
            <RefreshCw size={14} /> 刷新
          </button>
        </div>
      </div>

      {loading && <div className="map-loading">加载中...</div>}

      {error && (
        <div className="map-error">
          <p>{error}</p>
          <p className="map-hint">请确保后端 GatewayModule 已启动</p>
          <button className="btn-small" onClick={fetchMaps}>
            <RefreshCw size={14} /> 重试
          </button>
        </div>
      )}

      {!loading && !error && maps.length === 0 && (
        <div className="map-empty">
          <FolderOpen size={48} strokeWidth={1} />
          <p>暂无保存的地图</p>
          <p className="map-hint">使用建图模式 (python lingtu.py map) 创建地图</p>
        </div>
      )}

      {!loading && maps.length > 0 && (
        <div className="map-list">
          {maps.map((m) => (
            <div key={m.name} className={`map-card ${m.is_active ? 'map-card--active' : ''}`}>
              <div className="map-card-info">
                <span className="map-name">
                  {m.is_active && <Star size={14} className="star-icon" />}
                  {m.name}
                </span>
                <span className="map-meta">
                  {m.has_pcd && <span className="tag tag-pcd">PCD</span>}
                  {m.has_tomogram && <span className="tag tag-tomo">Tomogram</span>}
                  {!m.has_pcd && !m.has_tomogram && <span className="tag tag-empty">空</span>}
                  {m.patch_count != null && (
                    <span className="tag tag-empty">{m.patch_count} patches</span>
                  )}
                  {m.size_mb != null && (
                    <span className="tag tag-empty">{m.size_mb.toFixed(1)} MB</span>
                  )}
                </span>
              </div>
              <div className="map-card-actions">
                <button className="btn-tiny" onClick={() => previewMap3D(m.name)} title="3D 预览">
                  <Box size={12} /> 3D 预览
                </button>
                <button className="btn-tiny" onClick={() => renameMap(m.name)} title="重命名">
                  <Pencil size={12} /> 重命名
                </button>
                {!m.is_active && (
                  <button className="btn-tiny btn-accent" onClick={() => activateMap(m.name)} title="设为当前地图">
                    <Star size={12} /> 激活
                  </button>
                )}
                <button className="btn-tiny btn-danger" onClick={() => deleteMap(m.name)} title="删除地图">
                  <Trash2 size={12} />
                </button>
              </div>
            </div>
          ))}
        </div>
      )}
    </div>
  )
}
