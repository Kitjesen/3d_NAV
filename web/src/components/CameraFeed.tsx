import { useRef, useEffect } from 'react'
import { Camera, StopCircle, RefreshCw } from 'lucide-react'
import { useCamera } from '../hooks/useCamera'

interface CameraFeedProps {
  onStop: () => void
  estop: boolean
}

export function CameraFeed({ onStop, estop }: CameraFeedProps) {
  const { imgSrc, connected, reconnect } = useCamera('/ws/teleop')
  const imgRef = useRef<HTMLImageElement>(null)

  // Revoke old object URL is handled inside the hook.
  // We just need to update the img src.
  useEffect(() => {
    if (imgRef.current && imgSrc) {
      imgRef.current.src = imgSrc
    }
  }, [imgSrc])

  return (
    <div className="camera-feed">
      {/* Video area */}
      <div className="camera-viewport">
        {imgSrc ? (
          <img
            ref={imgRef}
            className="camera-img"
            alt="机器人相机画面"
            src={imgSrc}
          />
        ) : (
          <div className="camera-placeholder">
            <Camera size={48} strokeWidth={1} className="placeholder-icon" />
            <span className="placeholder-label">
              {connected ? '等待画面帧…' : '无相机画面'}
            </span>
            {!connected && (
              <button className="btn-ghost reconnect-btn" onClick={reconnect}>
                <RefreshCw size={14} />
                重新连接
              </button>
            )}
          </div>
        )}

        {/* Connection badge */}
        <div className={`cam-badge ${connected ? 'cam-badge--live' : 'cam-badge--off'}`}>
          <span className="cam-badge-dot" />
          {connected ? '直播' : '离线'}
        </div>

        {/* ESTOP overlay — full-screen flash when active */}
        {estop && <div className="estop-overlay">急停激活</div>}
      </div>

      {/* Controls strip */}
      <div className="camera-controls">
        <button
          className="btn-stop"
          onClick={onStop}
          aria-label="紧急停止"
        >
          <StopCircle size={18} />
          紧急停止
        </button>
        <span className="camera-hint">640 × 480 · MJPEG</span>
      </div>
    </div>
  )
}
