import { useEffect, useRef } from 'react'
import { Camera, StopCircle, RefreshCw } from 'lucide-react'
import { useCamera } from '../hooks/useCamera'
import { useWebRTC } from '../hooks/useWebRTC'
import { useWHEP } from '../hooks/useWHEP'
import { CameraHud } from './CameraHud'
import type { SSEState } from '../types'
import styles from './CameraFeed.module.css'

interface CameraFeedProps {
  onStop:   () => void
  estop:    boolean
  sseState: SSEState
}

type Source = 'whep' | 'rtc' | 'jpeg'

export function CameraFeed({ onStop, estop, sseState }: CameraFeedProps) {
  // Three-tier fallback, fastest-first:
  //   1. go2rtc WHEP sidecar (native Go, ~30–60 ms LAN)
  //   2. aiortc Python WebRTC (~80–150 ms LAN, same protocol)
  //   3. JPEG-over-WebSocket (~250 ms, universal fallback)
  // Each hook idles (no WS / no PC) until the previous tier reports an
  // "unavailable" error, so we never open three connections at once.
  const whep = useWHEP()
  const rtcIdle = whep.error !== 'unavailable'  // still trying WHEP
  const rtc = useWebRTC(rtcIdle ? '' : '/api/v1/webrtc/offer')
  const jpegIdle = rtc.error !== 'unavailable' || whep.error !== 'unavailable'
  const jpeg = useCamera(jpegIdle ? '' : '/ws/teleop')

  // Pick the tier that is actually producing video.  WHEP "connecting"
  // blocks fallback — if go2rtc is present but slow, we wait; if it's
  // absent the status probe fails fast and the hook emits 'unavailable'.
  let source: Source
  if (whep.stream) source = 'whep'
  else if (whep.error === 'unavailable' && rtc.stream) source = 'rtc'
  else if (whep.error === 'unavailable' && rtc.error === 'unavailable') source = 'jpeg'
  else source = whep.error === 'unavailable' ? 'rtc' : 'whep'  // still building

  const videoRef = useRef<HTMLVideoElement>(null)
  const liveStream = source === 'whep' ? whep.stream : source === 'rtc' ? rtc.stream : null
  useEffect(() => {
    if (!videoRef.current) return
    if (liveStream && videoRef.current.srcObject !== liveStream) {
      videoRef.current.srcObject = liveStream
    }
  }, [liveStream])

  const hasVideo =
    source === 'whep' ? whep.connected :
    source === 'rtc'  ? rtc.connected  :
    jpeg.imgSrc != null
  const isConnected =
    source === 'whep' ? whep.connected :
    source === 'rtc'  ? rtc.connected  :
    jpeg.connected
  const onReconnect =
    source === 'whep' ? whep.reconnect :
    source === 'rtc'  ? rtc.reconnect  :
    jpeg.reconnect

  let sourceLabel: string
  if (source === 'jpeg') {
    sourceLabel = '640 × 480 · MJPEG'
  } else if (source === 'whep') {
    sourceLabel = whep.connected ? '图传 · Go2RTC · H.264' : '图传 · Go2RTC (建立中…)'
  } else if (rtc.stats && rtc.stats.active_peers > 0) {
    const mbps = (rtc.stats.bitrate_bps / 1_000_000).toFixed(2)
    const fps  = rtc.stats.fps.toFixed(0)
    const enc  = rtc.stats.encode_avg_ms
    const encStr = typeof enc === 'number' ? ` · ${enc.toFixed(0)}ms enc` : ''
    sourceLabel = `aiortc · H.264 · ${fps}fps · ${mbps} Mbit/s${encStr}`
  } else {
    sourceLabel = 'aiortc · WebRTC (建立中…)'
  }

  return (
    <div className={styles.cameraFeed}>
      <div className={styles.viewport}>
        {(source === 'whep' || source === 'rtc') ? (
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
