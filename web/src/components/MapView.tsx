export function MapView() {
  return (
    <div className="map-view">
      <iframe
        className="map-iframe"
        src="/map/viewer"
        title="点云地图查看器"
        allowFullScreen
      />
    </div>
  )
}
