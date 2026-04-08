import { useState, useEffect, useCallback } from 'react'
import { Navigation, Settings } from 'lucide-react'
import { useSSE } from './hooks/useSSE'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { StatusBar } from './components/StatusBar'
import './App.css'

function App() {
  const sseState = useSSE('/api/v1/events')
  const [uptimeSeconds, setUptimeSeconds] = useState(0)

  // Uptime counter — increments every second from page load
  useEffect(() => {
    const t = setInterval(() => setUptimeSeconds(s => s + 1), 1000)
    return () => clearInterval(t)
  }, [])

  const handleStop = useCallback(async () => {
    try {
      await fetch('/api/v1/stop', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
    } catch {
      // best-effort — if the robot is unreachable, stop is already moot
    }
  }, [])

  const estop = sseState.safetyState?.estop ?? false
  const odom = sseState.odometry
  const posLabel = odom ? `(${odom.x.toFixed(1)}, ${odom.y.toFixed(1)})` : '--'
  const navState = sseState.missionStatus?.state ?? 'IDLE'

  return (
    <div className="app">
      {/* Top bar */}
      <header className="topbar">
        <div className="topbar-left">
          <span className="topbar-logo">
            <Navigation size={18} className="logo-icon" />
            LingTu
          </span>
          <span className={`topbar-badge ${sseState.connected ? 'badge--online' : 'badge--offline'}`}>
            {sseState.connected ? 'Online' : 'Offline'}
          </span>
        </div>

        <div className="topbar-center">
          <span className="topbar-stat">
            <span className="stat-label">pos</span>
            <span className="stat-value">{posLabel}</span>
          </span>
          <span className="topbar-divider" />
          <span className="topbar-stat">
            <span className="stat-label">nav</span>
            <span className={`stat-value nav-state--${navState.toLowerCase()}`}>{navState}</span>
          </span>
          {estop && (
            <>
              <span className="topbar-divider" />
              <span className="topbar-estop">E-STOP</span>
            </>
          )}
        </div>

        <div className="topbar-right">
          <button className="btn-icon" aria-label="Settings">
            <Settings size={16} />
          </button>
        </div>
      </header>

      {/* Main content */}
      <main className="main-grid">
        <section className="camera-section">
          <CameraFeed onStop={handleStop} estop={estop} />
        </section>

        <aside className="chat-section">
          <ChatPanel sseState={sseState} />
        </aside>
      </main>

      {/* Status bar */}
      <StatusBar sseState={sseState} uptimeSeconds={uptimeSeconds} />
    </div>
  )
}

export default App
