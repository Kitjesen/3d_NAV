import { useState, useEffect, useCallback } from 'react'
import { Map, FolderOpen, Trash2, Star, RefreshCw, Save, Pencil, Box } from 'lucide-react'
import type { MapInfo, ToastKind } from '../types'
import * as api from '../services/api'
import styles from './MapView.module.css'

interface MapViewProps {
  showToast: (msg: string, kind?: ToastKind) => void
}

export function MapView({ showToast }: MapViewProps) {
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')

  const loadMaps = useCallback(async () => {
    setLoading(true)
    setError('')
    try {
      const result = await api.fetchMaps()
      setMaps(result)
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e)
      setError(`无法获取地图列表: ${msg}`)
      setMaps([])
    } finally {
      setLoading(false)
    }
  }, [])

  useEffect(() => { loadMaps() }, [loadMaps])

  const handleActivate = async (name: string) => {
    try {
      await api.activateMap(name)
      showToast(`已激活: ${name}`, 'success')
      loadMaps()
    } catch {
      showToast(`激活失败: ${name}`, 'error')
    }
  }

  const handleDelete = async (name: string) => {
    if (!confirm(`确定删除地图 "${name}"？此操作不可恢复。`)) return
    try {
      await api.deleteMap(name)
      showToast(`已删除: ${name}`, 'success')
      loadMaps()
    } catch {
      showToast(`删除失败: ${name}`, 'error')
    }
  }

  const handleRename = async (oldName: string) => {
    const newName = prompt(`重命名地图 "${oldName}":`, oldName)
    if (!newName || newName === oldName) return
    try {
      await api.renameMap(oldName, newName)
      showToast(`已重命名: ${newName}`, 'success')
      loadMaps()
    } catch {
      showToast(`重命名失败`, 'error')
    }
  }

  const previewMap3D = (name: string) => {
    window.open(`/map/viewer?map=${encodeURIComponent(name)}`, '_blank', 'noopener,noreferrer')
  }

  const handleSave = async () => {
    const name = prompt('输入地图名称:')
    if (!name) return
    try {
      await api.saveMap(name)
      showToast(`保存成功: ${name}`, 'success')
      loadMaps()
    } catch {
      showToast('保存失败', 'error')
    }
  }

  return (
    <div className={styles.mapManager}>
      <div className={styles.header}>
        <h2><Map size={18} /> 地图管理</h2>
        <div className={styles.actions}>
          <button className={styles.btnSmallAccent} onClick={handleSave}>
            <Save size={14} /> 保存当前地图
          </button>
          <button className={styles.btnSmall} onClick={loadMaps}>
            <RefreshCw size={14} /> 刷新
          </button>
        </div>
      </div>

      {loading && <div className={styles.loading}>加载中...</div>}

      {error && (
        <div className={styles.error}>
          <p>{error}</p>
          <p className={styles.mapHint}>请确保后端 GatewayModule 已启动</p>
          <button className={styles.btnSmall} onClick={loadMaps}>
            <RefreshCw size={14} /> 重试
          </button>
        </div>
      )}

      {!loading && !error && maps.length === 0 && (
        <div className={styles.empty}>
          <FolderOpen size={48} strokeWidth={1} />
          <p>暂无保存的地图</p>
          <p className={styles.mapHint}>使用建图模式 (python lingtu.py map) 创建地图</p>
        </div>
      )}

      {!loading && maps.length > 0 && (
        <div className={styles.list}>
          {maps.map((m) => (
            <div key={m.name} className={m.is_active ? styles.cardActive : styles.card}>
              <div className={styles.cardInfo}>
                <span className={styles.mapName}>
                  {m.is_active && <Star size={14} className={styles.starIcon} />}
                  {m.name}
                </span>
                <span className={styles.meta}>
                  {m.has_pcd && <span className={styles.tagPcd}>PCD</span>}
                  {m.has_tomogram && <span className={styles.tagTomo}>Tomogram</span>}
                  {!m.has_pcd && !m.has_tomogram && <span className={styles.tagEmpty}>空</span>}
                  {m.patch_count != null && (
                    <span className={styles.tagEmpty}>{m.patch_count} patches</span>
                  )}
                  {m.size_mb != null && (
                    <span className={styles.tagEmpty}>{m.size_mb.toFixed(1)} MB</span>
                  )}
                </span>
              </div>
              <div className={styles.cardActions}>
                <button className={styles.btnTiny} onClick={() => previewMap3D(m.name)} title="3D 预览">
                  <Box size={12} /> 3D 预览
                </button>
                <button className={styles.btnTiny} onClick={() => handleRename(m.name)} title="重命名">
                  <Pencil size={12} /> 重命名
                </button>
                {!m.is_active && (
                  <button className={styles.btnTinyAccent} onClick={() => handleActivate(m.name)} title="设为当前地图">
                    <Star size={12} /> 激活
                  </button>
                )}
                <button className={styles.btnTinyDanger} onClick={() => handleDelete(m.name)} title="删除地图">
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
