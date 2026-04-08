import { Activity } from 'lucide-react'
import type { SSEState } from '../hooks/useSSE'

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

function rad2deg(r: number) {
  return ((r * 180) / Math.PI).toFixed(1)
}

export function StatusBar({ sseState, uptimeSeconds }: StatusBarProps) {
  const odom = sseState.odometry
  const mission = sseState.missionStatus
  const safety = sseState.safetyState

  const x = odom ? odom.x.toFixed(2) : '--'
  const y = odom ? odom.y.toFixed(2) : '--'
  const yaw = odom ? rad2deg(odom.yaw) + '°' : '--'
  const vx = odom ? odom.vx.toFixed(2) + ' m/s' : '--'
  const navState = mission?.state ?? 'IDLE'
  const estopActive = safety?.estop ?? false

  const NAV_STATE_ZH: Record<string, string> = {
    IDLE: '空闲',
    EXECUTING: '执行中',
    PLANNING: '规划中',
    ARRIVED: '已到达',
    FAILED: '失败',
    CANCELLED: '已取消',
  }
  const navStateZh = NAV_STATE_ZH[navState] ?? navState

  return (
    <div className="status-bar">
      <Activity size={13} className="statusbar-icon" />

      <span className="status-item">
        <span className="status-label">位置</span>
        <span className="status-value">({x}, {y})</span>
      </span>

      <span className="status-sep">·</span>

      <span className="status-item">
        <span className="status-label">航向</span>
        <span className="status-value">{yaw}</span>
      </span>

      <span className="status-sep">·</span>

      <span className="status-item">
        <span className="status-label">速度</span>
        <span className="status-value">{vx}</span>
      </span>

      <span className="status-sep">·</span>

      <span className="status-item">
        <span className="status-label">导航</span>
        <span className={`status-value status-nav ${navState === 'EXECUTING' ? 'status-nav--active' : navState === 'FAILED' ? 'status-nav--fail' : ''}`}>
          {navStateZh}
        </span>
      </span>

      <span className="status-sep">·</span>

      {estopActive && (
        <>
          <span className="status-item status-estop">急停</span>
          <span className="status-sep">·</span>
        </>
      )}

      <span className="status-item">
        <span className="status-label">运行时长</span>
        <span className="status-value">{formatUptime(uptimeSeconds)}</span>
      </span>

      <span className="status-sep">·</span>

      <span className="status-item">
        <span className="status-label">版本</span>
        <span className="status-value">1.8</span>
      </span>

      {sseState.slamStatus && (
        <>
          <span className="status-sep">·</span>
          <span className="status-item">
            <span className="status-label">SLAM</span>
            <span className="status-value">{sseState.slamStatus.slam_hz.toFixed(1)} Hz</span>
          </span>
          <span className="status-sep">·</span>
          <span className="status-item">
            <span className="status-label">退化</span>
            <span className={`status-value ${sseState.slamStatus.degeneracy_count > 0 ? 'status-nav--fail' : ''}`}>
              {sseState.slamStatus.degeneracy_count}
            </span>
          </span>
        </>
      )}

      {sseState.robotStatus && (
        <>
          <span className="status-sep">·</span>
          <span className="status-item">
            <span className="status-label">电量</span>
            <span className="status-value">{sseState.robotStatus.battery.toFixed(0)}%</span>
          </span>
          <span className="status-sep">·</span>
          <span className="status-item">
            <span className="status-label">温度</span>
            <span className="status-value">{sseState.robotStatus.temperature.toFixed(1)}°C</span>
          </span>
        </>
      )}

      <span className="statusbar-right">
        <span className={`hb-dot ${sseState.lastHeartbeat && Date.now() - sseState.lastHeartbeat < 5000 ? 'hb-dot--alive' : ''}`} title="Heartbeat" />
      </span>
    </div>
  )
}
