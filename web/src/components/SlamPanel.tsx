import { useState } from 'react'
import { Radio, Navigation, OctagonX, Activity } from 'lucide-react'
import type { SSEState, SlamProfile, ToastKind } from '../types'
import * as api from '../services/api'
import styles from './SlamPanel.module.css'

interface SlamPanelProps {
  sseState: SSEState
  showToast: (msg: string, kind?: ToastKind) => void
}

const PROFILE_LABELS: Record<SlamProfile, string> = {
  fastlio2:  'SLAM 建图',
  localizer: '导航巡航',
  stop:      '停止',
}

export function SlamPanel({ sseState, showToast }: SlamPanelProps) {
  const [pending, setPending] = useState<SlamProfile | null>(null)

  const slamStatus = sseState.slamStatus
  const currentMode: string = slamStatus?.mode ?? 'stop'

  const handle = async (profile: SlamProfile) => {
    if (pending) return
    setPending(profile)
    try {
      await api.switchSlamMode(profile)
      const label = PROFILE_LABELS[profile]
      showToast(`已切换: ${label}`, 'success')
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e)
      showToast(`切换失败: ${msg}`, 'error')
    } finally {
      setPending(null)
    }
  }

  const modeActive = (p: SlamProfile) => currentMode === p || (p === 'stop' && currentMode === 'stop')

  return (
    <div className={styles.slamPanel}>
      <div className={styles.panelHeader}>
        <span className={styles.panelTitle}>
          <Radio size={15} />
          SLAM 模式
        </span>
      </div>

      <div className={styles.section}>
        <p className={styles.sectionLabel}>运行模式</p>
        <div className={styles.modeGroup}>
          <button
            className={modeActive('fastlio2') ? styles.modeBtnActive : styles.modeBtn}
            onClick={() => handle('fastlio2')}
            disabled={pending !== null}
            aria-pressed={modeActive('fastlio2')}
          >
            <span className={styles.modeIcon}><Navigation size={14} /></span>
            <span className={styles.modeText}>SLAM 建图</span>
            {modeActive('fastlio2') && <span className={styles.modeDot} />}
            {pending === 'fastlio2' && <span className={styles.modeSpinner} />}
          </button>

          <button
            className={modeActive('localizer') ? styles.modeBtnActive : styles.modeBtn}
            onClick={() => handle('localizer')}
            disabled={pending !== null}
            aria-pressed={modeActive('localizer')}
          >
            <span className={styles.modeIcon}><Activity size={14} /></span>
            <span className={styles.modeText}>导航巡航</span>
            {modeActive('localizer') && <span className={styles.modeDot} />}
            {pending === 'localizer' && <span className={styles.modeSpinner} />}
          </button>
        </div>
      </div>

      <div className={styles.section}>
        <p className={styles.sectionLabel}>统计信息</p>
        <div className={styles.statsGrid}>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>SLAM 频率</span>
            <span className={styles.statValue}>
              {slamStatus ? `${slamStatus.slam_hz.toFixed(1)} Hz` : '--'}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>地图点云</span>
            <span className={styles.statValue}>
              {slamStatus ? slamStatus.map_points.toLocaleString() : '--'}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>退化次数</span>
            <span className={`${styles.statValue} ${slamStatus && slamStatus.degeneracy_count > 0 ? styles.statWarn : ''}`}>
              {slamStatus ? slamStatus.degeneracy_count : '--'}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>当前模式</span>
            <span className={`${styles.statValue} ${styles.statMode}`}>
              {slamStatus?.mode ?? '--'}
            </span>
          </div>
        </div>
      </div>

      <div className={styles.sectionEstop}>
        <button
          className={styles.estopBtn}
          onClick={() => handle('stop')}
          disabled={pending !== null}
          aria-label="紧急停止 SLAM"
        >
          <OctagonX size={16} />
          紧急停止
        </button>
        <p className={styles.estopHint}>停止所有 SLAM 进程</p>
      </div>
    </div>
  )
}
