import { Activity } from 'lucide-react'
import type { SSEState } from '../types'
import styles from './StatusBar.module.css'

interface StatusBarProps {
  sseState: SSEState
  uptimeSeconds: number
}

function formatUptime(s: number) {
  const h = Math.floor(s / 3600)
  const m = Math.floor((s % 3600) / 60)
  const sec = s % 60
  if (h > 0) return `${h}h${String(m).padStart(2, '0')}m`
  if (m > 0) return `${m}m${String(sec).padStart(2, '0')}s`
  return `${sec}s`
}

function rad2deg(r: number | undefined) {
  if (typeof r !== 'number') return '--'
  return ((r * 180) / Math.PI).toFixed(1)
}

function num(v: unknown, digits = 2): string {
  return typeof v === 'number' && isFinite(v) ? v.toFixed(digits) : '--'
}

const NAV_STATE_ZH: Record<string, string> = {
  IDLE: '空闲',
  EXECUTING: '执行中',
  PLANNING: '规划中',
  ARRIVED: '已到达',
  FAILED: '失败',
  CANCELLED: '已取消',
}

export function StatusBar({ sseState, uptimeSeconds }: StatusBarProps) {
  const odom = sseState.odometry
  const mission = sseState.missionStatus
  const safety = sseState.safetyState

  const x = num(odom?.x)
  const y = num(odom?.y)
  const yaw = odom ? rad2deg(odom.yaw) + '°' : '--'
  const vx = num(odom?.vx) + (typeof odom?.vx === 'number' ? ' m/s' : '')
  const navState = mission?.state ?? 'IDLE'
  const estopActive = safety?.estop ?? false
  const navStateZh = NAV_STATE_ZH[navState] ?? navState

  return (
    <div className={styles.statusBar}>
      <Activity size={13} className={styles.icon} />

      <span className={styles.item}>
        <span className={styles.label}>位置</span>
        <span className={styles.value}>({x}, {y})</span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item}>
        <span className={styles.label}>航向</span>
        <span className={styles.value}>{yaw}</span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item}>
        <span className={styles.label}>速度</span>
        <span className={styles.value}>{vx}</span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item}>
        <span className={styles.label}>导航</span>
        <span className={`${styles.value} ${navState === 'EXECUTING' ? styles.navActive : navState === 'FAILED' ? styles.navFail : ''}`}>
          {navStateZh}
        </span>
      </span>

      <span className={styles.sep}>·</span>

      {estopActive && (
        <>
          <span className={`${styles.item} ${styles.estop}`}>急停</span>
          <span className={styles.sep}>·</span>
        </>
      )}

      <span className={styles.item}>
        <span className={styles.label}>运行时长</span>
        <span className={styles.value}>{formatUptime(uptimeSeconds)}</span>
      </span>

      <span className={styles.sep}>·</span>

      <span className={styles.item}>
        <span className={styles.label}>版本</span>
        <span className={styles.value}>1.8</span>
      </span>

      {sseState.slamStatus && (
        <>
          <span className={styles.sep}>·</span>
          <span className={styles.item}>
            <span className={styles.label}>SLAM</span>
            <span className={styles.value}>{sseState.slamStatus.slam_hz.toFixed(1)} Hz</span>
          </span>
          <span className={styles.sep}>·</span>
          <span className={styles.item}>
            <span className={styles.label}>退化</span>
            <span className={`${styles.value} ${sseState.slamStatus.degeneracy_count > 0 ? styles.navFail : ''}`}>
              {sseState.slamStatus.degeneracy_count}
            </span>
          </span>
        </>
      )}

      {sseState.robotStatus && (
        <>
          <span className={styles.sep}>·</span>
          <span className={styles.item}>
            <span className={styles.label}>电量</span>
            <span className={styles.value}>{sseState.robotStatus.battery.toFixed(0)}%</span>
          </span>
          <span className={styles.sep}>·</span>
          <span className={styles.item}>
            <span className={styles.label}>温度</span>
            <span className={styles.value}>{sseState.robotStatus.temperature.toFixed(1)}°C</span>
          </span>
        </>
      )}

      <span className={styles.right}>
        <span className={sseState.lastHeartbeat && Date.now() - sseState.lastHeartbeat < 5000 ? styles.hbDotAlive : styles.hbDot} title="Heartbeat" />
      </span>
    </div>
  )
}
