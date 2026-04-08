import { Navigation, Settings, Radio } from 'lucide-react'
import type { SSEState } from '../types'
import styles from './Topbar.module.css'

const SLAM_MODE_ZH: Record<string, string> = {
  fastlio2:  '建图',
  localizer: '导航',
  stop:      '停止',
}

const NAV_STATE_ZH: Record<string, string> = {
  IDLE: '空闲',
  EXECUTING: '执行中',
  PLANNING: '规划中',
  ARRIVED: '已到达',
  FAILED: '失败',
  CANCELLED: '已取消',
}

const NAV_STATE_CLASS: Record<string, string> = {
  executing: styles.navExecuting,
  failed:    styles.navFailed,
  planning:  styles.navPlanning,
}

const SLAM_MODE_CLASS: Record<string, string> = {
  fastlio2:  styles.slamFastlio2,
  localizer: styles.slamLocalizer,
  stop:      styles.slamStop,
}

interface TopbarProps {
  sseState: SSEState
}

export function Topbar({ sseState }: TopbarProps) {
  const estop = sseState.safetyState?.estop ?? false
  const odom = sseState.odometry
  const posLabel =
    typeof odom?.x === 'number' && typeof odom?.y === 'number'
      ? `(${odom.x.toFixed(1)}, ${odom.y.toFixed(1)})`
      : '--'
  const navState = sseState.missionStatus?.state ?? 'IDLE'
  const navStateZh = NAV_STATE_ZH[navState] ?? navState
  const slamMode = sseState.slamStatus?.mode ?? null
  const slamModeZh = slamMode ? (SLAM_MODE_ZH[slamMode] ?? slamMode) : null

  return (
    <header className={styles.topbar}>
      <div className={styles.left}>
        <span className={styles.logo}>
          <Navigation size={18} className={styles.logoIcon} />
          LingTu
        </span>
        <span className={sseState.connected ? styles.badgeOnline : styles.badgeOffline}>
          {sseState.connected ? '在线' : '离线'}
        </span>
      </div>
      <div className={styles.center}>
        <span className={styles.stat}>
          <span className={styles.statLabel}>位置</span>
          <span className={styles.statValue}>{posLabel}</span>
        </span>
        <span className={styles.divider} />
        <span className={styles.stat}>
          <span className={styles.statLabel}>导航</span>
          <span className={`${styles.statValue} ${NAV_STATE_CLASS[navState.toLowerCase()] ?? ''}`}>{navStateZh}</span>
        </span>
        {slamModeZh && (
          <>
            <span className={styles.divider} />
            <span className={styles.stat}>
              <span className={styles.statLabel}><Radio size={10} style={{ display: 'inline', verticalAlign: 'middle', marginRight: 2 }} />SLAM</span>
              <span className={`${styles.statValue} ${SLAM_MODE_CLASS[slamMode!] ?? ''}`}>{slamModeZh}</span>
            </span>
          </>
        )}
        {estop && (
          <>
            <span className={styles.divider} />
            <span className={styles.estop}>急停</span>
          </>
        )}
      </div>
      <div className={styles.right}>
        <button className={styles.btnIcon} aria-label="设置">
          <Settings size={16} />
        </button>
      </div>
    </header>
  )
}
