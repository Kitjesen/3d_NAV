import { useState, useCallback } from 'react'
import { Map, RefreshCw } from 'lucide-react'

export function MapView() {
  const [error, setError] = useState(false)
  const [loading, setLoading] = useState(true)

  const handleLoad = useCallback(() => {
    setLoading(false)
  }, [])

  const handleError = useCallback(() => {
    setError(true)
    setLoading(false)
  }, [])

  const retry = useCallback(() => {
    setError(false)
    setLoading(true)
  }, [])

  if (error) {
    return (
      <div className="map-view map-placeholder">
        <Map size={48} strokeWidth={1} />
        <p>地图查看器不可用</p>
        <p className="map-hint">请确保 GatewayModule 已启动且 /map/viewer 路由可用</p>
        <button className="btn-reconnect" onClick={retry}>
          <RefreshCw size={14} /> 重试
        </button>
      </div>
    )
  }

  return (
    <div className="map-view">
      {loading && (
        <div className="map-placeholder">
          <Map size={48} strokeWidth={1} />
          <p>加载地图查看器...</p>
        </div>
      )}
      <iframe
        className="map-iframe"
        src="/map/viewer"
        title="点云地图查看器"
        allowFullScreen
        onLoad={handleLoad}
        onError={handleError}
        style={{ display: loading ? 'none' : 'block' }}
      />
    </div>
  )
}
