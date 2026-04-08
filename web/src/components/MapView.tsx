import { useState, useEffect, useCallback } from 'react'
import { Map, FolderOpen, Trash2, Star, RefreshCw, Save } from 'lucide-react'

interface MapInfo {
  name: string
  has_pcd: boolean
  has_tomogram: boolean
  is_active: boolean
  size_mb?: number
}

export function MapView() {
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')
  const [message, setMessage] = useState('')

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

  const showMsg = (text: string) => {
    setMessage(text)
    setTimeout(() => setMessage(''), 3000)
  }

  const activateMap = async (name: string) => {
    try {
      const res = await fetch('/api/v1/map/activate', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ name }),
      })
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      showMsg(`已激活: ${name}`)
      fetchMaps()
    } catch {
      showMsg(`激活失败: ${name}`)
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
      showMsg(`已删除: ${name}`)
      fetchMaps()
    } catch {
      showMsg(`删除失败: ${name}`)
    }
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
      showMsg(`保存成功: ${name}`)
      fetchMaps()
    } catch {
      showMsg('保存失败')
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

      {message && <div className="map-message">{message}</div>}

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
                </span>
              </div>
              <div className="map-card-actions">
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
