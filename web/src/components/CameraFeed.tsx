import { useEffect, useRef, useState } from 'react'
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

// If WebRTC doesn't produce a video frame within this window, fall back
// to MJPEG. ICE failures on certain networks (VPN, NAT) can leave peers
// stuck in 'checking' forever — we can't wait.
const WEBRTC_TIMEOUT_MS = 8_000

export function CameraFeed({ onStop, estop, sseState }: CameraFeedProps) {
  // Prefer WebRTC (H.264, ~100 ms glass-to-glass).  Fall back to JPEG when:
  //   - gateway says 'unavailable' (aiortc missing)
  //   - WebRTC fails to produce a stream within WEBRTC_TIMEOUT_MS (ICE stuck)
  const rtc = useWebRTC()
  const [timedOut, setTimedOut] = useState(false)
  useEffect(() => {
    if (rtc.connected || rtc.error) { setTimedOut(false); return }
    const t = setTimeout(() => setTimedOut(true), WEBRTC_TIMEOUT_MS)
    return () => clearTimeout(t)
  }, [rtc.connected, rtc.error])
  const fallback = rtc.error === 'unavailable' || timedOut
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
  const onReconnect  = usingRtc ? rtc.reconnect : jpeg.reconnect

  // Live HUD label: WebRTC exposes getStats, JPEG path doesn't.  Prefer
  // real numbers over the static "720p · H.264" caption when available.
  let sourceLabel: string
  if (!usingRtc) {
    sourceLabel = '640 × 480 · MJPEG'
  } else if (rtc.stats && rtc.stats.active_peers > 0) {
    const mbps = (rtc.stats.bitrate_bps / 1_000_000).toFixed(2)
    const fps  = rtc.stats.fps.toFixed(0)
    const enc  = rtc.stats.encode_avg_ms
    const encStr = typeof enc === 'number' ? ` · ${enc.toFixed(0)}ms enc` : ''
    sourceLabel = `H.264 · ${fps}fps · ${mbps} Mbit/s${encStr}`
  } else {
    sourceLabel = 'H.264 · WebRTC (建立中…)'
  }

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
