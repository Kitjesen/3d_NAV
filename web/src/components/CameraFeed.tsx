import { useRef, useEffect } from 'react'
import { Camera, StopCircle, RefreshCw } from 'lucide-react'
import { useCamera } from '../hooks/useCamera'
import styles from './CameraFeed.module.css'

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
    <div className={styles.cameraFeed}>
      {/* Video area */}
      <div className={styles.viewport}>
        {imgSrc ? (
          <img
            ref={imgRef}
            className={styles.img}
            alt="机器人相机画面"
            src={imgSrc}
          />
        ) : (
          <div className={styles.placeholder}>
            <Camera size={48} strokeWidth={1} className={styles.placeholderIcon} />
            <span className={styles.placeholderLabel}>
              {connected ? '等待画面帧…' : '无相机画面'}
            </span>
            {!connected && (
              <button className={styles.reconnectBtn} onClick={reconnect}>
                <RefreshCw size={14} />
                重新连接
              </button>
            )}
          </div>
        )}

        {/* Connection badge */}
        <div className={connected ? styles.camBadgeLive : styles.camBadgeOff}>
          <span className={styles.camBadgeDot} />
          {connected ? '直播' : '离线'}
        </div>

        {/* ESTOP overlay — full-screen flash when active */}
        {estop && <div className={styles.estopOverlay}>急停激活</div>}
      </div>

      {/* Controls strip */}
      <div className={styles.controls}>
        <button
          className={styles.btnStop}
          onClick={onStop}
          aria-label="紧急停止"
        >
          <StopCircle size={18} />
          紧急停止
        </button>
        <span className={styles.hint}>640 × 480 · MJPEG</span>
      </div>
    </div>
  )
}
