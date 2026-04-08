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

  return (
    <div className="status-bar">
      <Activity size={13} className="statusbar-icon" />

      <span className="status-item">
        <span className="status-label">pos</span>
        <span className="status-value">({x}, {y})</span>
      </span>

      <span className="status-sep">·</span>

      <span className="status-item">
        <span className="status-label">hdg</span>
        <span className="status-value">{yaw}</span>
      </span>

      <span className="status-sep">·</span>

      <span className="status-item">
        <span className="status-label">vel</span>
        <span className="status-value">{vx}</span>
      </span>

      <span className="status-sep">·</span>

      <span className="status-item">
        <span className="status-label">nav</span>
        <span className={`status-value status-nav ${navState === 'EXECUTING' ? 'status-nav--active' : navState === 'FAILED' ? 'status-nav--fail' : ''}`}>
          {navState}
        </span>
      </span>

      <span className="status-sep">·</span>

      {estopActive && (
        <>
          <span className="status-item status-estop">E-STOP</span>
          <span className="status-sep">·</span>
        </>
      )}

      <span className="status-item">
        <span className="status-label">uptime</span>
        <span className="status-value">{formatUptime(uptimeSeconds)}</span>
      </span>

      <span className="status-sep">·</span>

      <span className="status-item">
        <span className="status-label">v</span>
        <span className="status-value">1.8</span>
      </span>

      <span className="statusbar-right">
        <span className={`hb-dot ${sseState.lastHeartbeat && Date.now() - sseState.lastHeartbeat < 5000 ? 'hb-dot--alive' : ''}`} title="Heartbeat" />
      </span>
    </div>
  )
}
