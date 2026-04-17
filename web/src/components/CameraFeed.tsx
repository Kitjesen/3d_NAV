import { useEffect, useRef } from 'react'
import { Camera, StopCircle, RefreshCw } from 'lucide-react'
import { useCamera } from '../hooks/useCamera'
import { useWebRTC } from '../hooks/useWebRTC'
import { CameraHud } from './CameraHud'
import type { SSEState } from '../types'
import styles from './CameraFeed.module.css'

interface CameraFeedProps {
  onStop:   () => void
  estop:    boolean
  sseState: SSEState
}

export function CameraFeed({ onStop, estop, sseState }: CameraFeedProps) {
  // Prefer WebRTC (H.264, ~100 ms glass-to-glass).  If the gateway
  // returns 503 (aiortc not installed) we fall through to the legacy
  // JPEG-over-WS stream without any user action.
  const rtc = useWebRTC()
  const fallback = rtc.error === 'unavailable'
  const jpeg = useCamera(fallback ? '/ws/teleop' : '')  // empty URL → idle

  const videoRef = useRef<HTMLVideoElement>(null)
  useEffect(() => {
    if (!videoRef.current) return
    if (rtc.stream && videoRef.current.srcObject !== rtc.stream) {
      videoRef.current.srcObject = rtc.stream
    }
  }, [rtc.stream])

  const usingRtc     = !fallback
  const hasVideo     = usingRtc ? rtc.connected : jpeg.imgSrc != null
  const isConnected  = usingRtc ? rtc.connected : jpeg.connected
  const sourceLabel  = usingRtc ? '720p · H.264 · WebRTC' : '640 × 480 · MJPEG'
  const onReconnect  = usingRtc ? rtc.reconnect : jpeg.reconnect

  return (
    <div className={styles.cameraFeed}>
      <div className={styles.viewport}>
        {usingRtc ? (
          <video
            ref={videoRef}
            className={styles.img}
            autoPlay
            muted
            playsInline
          />
        ) : jpeg.imgSrc ? (
          <img className={styles.img} alt="机器人相机画面" src={jpeg.imgSrc} />
        ) : null}

        {!hasVideo && (
          <div className={styles.placeholder}>
            <Camera size={48} strokeWidth={1} className={styles.placeholderIcon} />
            <span className={styles.placeholderLabel}>
              {isConnected ? '等待画面帧…' : '无相机画面'}
            </span>
            {!isConnected && (
              <button className={styles.reconnectBtn} onClick={onReconnect}>
                <RefreshCw size={14} />
                重新连接
              </button>
            )}
          </div>
        )}

        <div className={isConnected ? styles.camBadgeLive : styles.camBadgeOff}>
          <span className={styles.camBadgeDot} />
          {isConnected ? '直播' : '离线'}
        </div>

        <CameraHud sseState={sseState} />

        {estop && <div className={styles.estopOverlay}>急停激活</div>}
      </div>

      <div className={styles.controls}>
        <button
          className={styles.btnStop}
          onClick={onStop}
          aria-label="紧急停止"
        >
          <StopCircle size={18} />
          紧急停止
        </button>
        <span className={styles.hint}>{sourceLabel}</span>
      </div>
    </div>
  )
}
