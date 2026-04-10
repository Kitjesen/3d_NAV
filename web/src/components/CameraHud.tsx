import type { SSEState } from '../types'
import styles from './CameraHud.module.css'

interface CameraHudProps {
  sseState: SSEState
}

function fmt(v: number | undefined, dec: number, unit: string, fallback = '--') {
  return typeof v === 'number' ? `${v.toFixed(dec)} ${unit}` : fallback
}

export function CameraHud({ sseState }: CameraHudProps) {
  const slam = sseState.slamStatus
  const robot = sseState.robotStatus
  const odom = sseState.odometry

  // Latency from last heartbeat
  const latencyMs = sseState.lastHeartbeat
    ? Math.max(0, Date.now() - sseState.lastHeartbeat)
    : null

  const slamHz = typeof slam?.slam_hz === 'number' ? `${slam.slam_hz.toFixed(0)} Hz` : '--'
  const battery = typeof robot?.battery === 'number'
    ? `${robot.battery.toFixed(0)}%`
    : '--'
  const temp = fmt(robot?.temperature, 1, '°C')
  const latency = latencyMs != null ? `${latencyMs} ms` : '--'
  const speed = fmt(odom?.vx, 2, 'm/s')

  const batteryLow = typeof robot?.battery === 'number' && robot.battery < 20
  const tempHigh = typeof robot?.temperature === 'number' && robot.temperature > 60

  return (
    <div className={styles.hud}>
      <div className={styles.row}>
        <span className={styles.key}>SLAM</span>
        <span className={styles.val}>{slamHz}</span>
      </div>
      <div className={styles.row}>
        <span className={styles.key}>SPD</span>
        <span className={styles.val}>{speed}</span>
      </div>
      <div className={styles.row}>
        <span className={styles.key}>BAT</span>
        <span className={batteryLow ? styles.valWarn : styles.val}>{battery}</span>
      </div>
      <div className={styles.row}>
        <span className={styles.key}>TMP</span>
        <span className={tempHigh ? styles.valWarn : styles.val}>{temp}</span>
      </div>
      <div className={styles.row}>
        <span className={styles.key}>LAT</span>
        <span className={styles.val}>{latency}</span>
      </div>
    </div>
  )
}
