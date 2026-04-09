import { useState, useEffect, useCallback } from 'react'
import { Map, FolderOpen, Trash2, Star, RefreshCw, Save, Pencil } from 'lucide-react'
import type { MapInfo, ToastKind } from '../types'
import * as api from '../services/api'
import { PointCloudViewer } from './PointCloudViewer'
import styles from './MapView.module.css'

interface MapViewProps {
  showToast: (msg: string, kind?: ToastKind) => void
}

export function MapView({ showToast }: MapViewProps) {
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')
  const [selectedMap, setSelectedMap] = useState<string | null>(null)

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
      if (selectedMap === name) setSelectedMap(null)
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
      if (selectedMap === oldName) setSelectedMap(newName)
      loadMaps()
    } catch {
      showToast('重命名失败', 'error')
    }
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

  const togglePreview = (name: string, hasPcd: boolean) => {
    if (!hasPcd) return
    setSelectedMap(prev => prev === name ? null : name)
  }

  return (
    <div className={styles.mapTab}>
      {/* Left: point cloud viewer */}
      <div className={styles.viewerPanel}>
        <PointCloudViewer mapName={selectedMap} />
      </div>

      {/* Right: map list */}
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

          {error && (
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

          {!loading && maps.length > 0 && (
            <div className={styles.list}>
              {maps.map((m) => (
                <div
                  key={m.name}
                  className={[
                    m.is_active ? styles.cardActive : styles.card,
                    selectedMap === m.name ? styles.cardSelected : '',
                  ].filter(Boolean).join(' ')}
                >
                  <div className={styles.cardInfo}>
                    <span className={styles.mapName}>
                      {m.is_active && <Star size={12} className={styles.starIcon} />}
                      {m.name}
                    </span>
                    <span className={styles.meta}>
                      {m.has_pcd && <span className={styles.tagPcd}>PCD</span>}
                      {m.has_tomogram && <span className={styles.tagTomo}>Tomogram</span>}
                      {!m.has_pcd && !m.has_tomogram && <span className={styles.tagEmpty}>空</span>}
                      {!!m.patch_count && (
                        <span className={styles.tagEmpty}>{m.patch_count} patches</span>
                      )}
                      {!!m.size_mb && (
                        <span className={styles.tagEmpty}>{m.size_mb.toFixed(1)} MB</span>
                      )}
                    </span>
                  </div>

                  <div className={styles.cardActions}>
                    {m.has_pcd && (
                      <button
                        className={selectedMap === m.name ? styles.btnTinyActive : styles.btnTiny}
                        onClick={() => togglePreview(m.name, m.has_pcd)}
                        title="点云预览"
                      >
                        预览
                      </button>
                    )}
                    <button
                      className={styles.btnTiny}
                      onClick={() => handleRename(m.name)}
                      title="重命名"
                    >
                      <Pencil size={11} />
                    </button>
                    {!m.is_active && (
                      <button
                        className={styles.btnTinyAccent}
                        onClick={() => handleActivate(m.name)}
                        title="设为当前地图"
                      >
                        <Star size={11} /> 激活
                      </button>
                    )}
                    <button
                      className={styles.btnTinyDanger}
                      onClick={() => handleDelete(m.name)}
                      title="删除地图"
                    >
                      <Trash2 size={11} />
                    </button>
                  </div>
                </div>
              ))}
            </div>
          )}
        </div>
      </div>
    </div>
  )
}
