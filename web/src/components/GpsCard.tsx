import type { SSEState } from '../types'
import styles from './GpsCard.module.css'

interface GpsCardProps {
  sseState: SSEState
}

/**
 * GpsCard — rotating globe with live robot position.
 *
 * Uses SLAM odometry as the source of truth (not real GPS — outdoor
 * robots in LingTu localize against a pre-built map via Fast-LIO2 +
 * ICP localizer). The "GPS" framing is just for operator familiarity.
 */
export function GpsCard({ sseState }: GpsCardProps) {
  const odom = sseState.odometry
  const connected = sseState.connected
  const slam = sseState.slamStatus

  const posX = typeof odom?.x === 'number' ? odom.x.toFixed(2) : '--'
  const posY = typeof odom?.y === 'number' ? odom.y.toFixed(2) : '--'
  const yawDeg =
    typeof odom?.yaw === 'number'
      ? ((odom.yaw * 180) / Math.PI).toFixed(1)
      : '--'
  const vx = typeof odom?.vx === 'number' ? odom.vx.toFixed(2) : '--'
  const mode = slam?.mode ?? '离线'
  const hasFix = connected && odom != null

  return (
    <div className={styles.card}>
      <div className={styles.header}>
        <span className={styles.label}>
          <span className={styles.labelDot} />
          定位
        </span>
        <span className={hasFix ? styles.status : styles.statusOffline}>
          {hasFix ? '已锁定' : '未锁定'}
        </span>
      </div>

      <div className={styles.globeWrap}>
        <div className={styles.orbit} />
        <div className={styles.globe} />
        <div className={styles.marker}>
          <div className={styles.markerPulse} />
          <div className={styles.markerPulse2} />
          <div className={styles.markerDot} />
        </div>
      </div>

      <div className={styles.stats}>
        <div className={styles.stat}>
          <span className={styles.statLabel}>X · Y (m)</span>
          <span className={styles.statValue}>
            {posX}, {posY}
          </span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>航向</span>
          <span className={styles.statValue}>{yawDeg}°</span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>线速度</span>
          <span className={styles.statValue}>{vx} m/s</span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>SLAM 模式</span>
          <span className={styles.statValueDim}>{mode}</span>
        </div>
      </div>
    </div>
  )
}
