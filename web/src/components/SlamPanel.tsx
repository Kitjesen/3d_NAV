import { useState } from 'react'
import { Radio, Navigation, OctagonX, Activity } from 'lucide-react'
import type { SSEState } from '../hooks/useSSE'
import type { ToastKind } from '../hooks/useToast'

interface SlamPanelProps {
  sseState: SSEState
  showToast: (msg: string, kind?: ToastKind) => void
}

type SlamProfile = 'fastlio2' | 'localizer' | 'stop'

const PROFILE_LABELS: Record<SlamProfile, string> = {
  fastlio2:  'SLAM 建图',
  localizer: '导航巡航',
  stop:      '停止',
}

async function switchProfile(profile: SlamProfile): Promise<void> {
  const res = await fetch('/api/v1/slam/switch', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ profile }),
  })
  if (!res.ok) throw new Error(`HTTP ${res.status}`)
}

export function SlamPanel({ sseState, showToast }: SlamPanelProps) {
  const [pending, setPending] = useState<SlamProfile | null>(null)

  const slamStatus = sseState.slamStatus
  const currentMode: string = slamStatus?.mode ?? 'stop'

  const handle = async (profile: SlamProfile) => {
    if (pending) return
    setPending(profile)
    try {
      await switchProfile(profile)
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
    <div className="slam-panel">
      <div className="slam-panel-header">
        <span className="slam-panel-title">
          <Radio size={15} />
          SLAM 模式
        </span>
      </div>

      <div className="slam-section">
        <p className="slam-section-label">运行模式</p>
        <div className="slam-mode-group">
          <button
            className={`slam-mode-btn ${modeActive('fastlio2') ? 'slam-mode-btn--active' : ''}`}
            onClick={() => handle('fastlio2')}
            disabled={pending !== null}
            aria-pressed={modeActive('fastlio2')}
          >
            <span className="slam-mode-icon"><Navigation size={14} /></span>
            <span className="slam-mode-text">SLAM 建图</span>
            {modeActive('fastlio2') && <span className="slam-mode-dot" />}
            {pending === 'fastlio2' && <span className="slam-mode-spinner" />}
          </button>

          <button
            className={`slam-mode-btn ${modeActive('localizer') ? 'slam-mode-btn--active' : ''}`}
            onClick={() => handle('localizer')}
            disabled={pending !== null}
            aria-pressed={modeActive('localizer')}
          >
            <span className="slam-mode-icon"><Activity size={14} /></span>
            <span className="slam-mode-text">导航巡航</span>
            {modeActive('localizer') && <span className="slam-mode-dot" />}
            {pending === 'localizer' && <span className="slam-mode-spinner" />}
          </button>
        </div>
      </div>

      <div className="slam-section">
        <p className="slam-section-label">统计信息</p>
        <div className="slam-stats-grid">
          <div className="slam-stat-card">
            <span className="slam-stat-label">SLAM 频率</span>
            <span className="slam-stat-value">
              {slamStatus ? `${slamStatus.slam_hz.toFixed(1)} Hz` : '--'}
            </span>
          </div>
          <div className="slam-stat-card">
            <span className="slam-stat-label">地图点云</span>
            <span className="slam-stat-value">
              {slamStatus ? slamStatus.map_points.toLocaleString() : '--'}
            </span>
          </div>
          <div className="slam-stat-card">
            <span className="slam-stat-label">退化次数</span>
            <span className={`slam-stat-value ${slamStatus && slamStatus.degeneracy_count > 0 ? 'slam-stat--warn' : ''}`}>
              {slamStatus ? slamStatus.degeneracy_count : '--'}
            </span>
          </div>
          <div className="slam-stat-card">
            <span className="slam-stat-label">当前模式</span>
            <span className="slam-stat-value slam-stat--mode">
              {slamStatus?.mode ?? '--'}
            </span>
          </div>
        </div>
      </div>

      <div className="slam-section slam-section--estop">
        <button
          className="slam-estop-btn"
          onClick={() => handle('stop')}
          disabled={pending !== null}
          aria-label="紧急停止 SLAM"
        >
          <OctagonX size={16} />
          紧急停止
        </button>
        <p className="slam-estop-hint">停止所有 SLAM 进程</p>
      </div>
    </div>
  )
}
