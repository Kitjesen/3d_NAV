import { useState, useEffect, useCallback } from 'react'
import { Navigation, Settings } from 'lucide-react'
import { useSSE } from './hooks/useSSE'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { StatusBar } from './components/StatusBar'
import { MapView } from './components/MapView'
import './App.css'

type Tab = 'console' | 'map'

const NAV_STATE_ZH: Record<string, string> = {
  IDLE: '空闲',
  EXECUTING: '执行中',
  PLANNING: '规划中',
  ARRIVED: '已到达',
  FAILED: '失败',
  CANCELLED: '已取消',
}

function App() {
  const sseState = useSSE('/api/v1/events')
  const [uptimeSeconds, setUptimeSeconds] = useState(0)
  const [activeTab, setActiveTab] = useState<Tab>('console')

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
  const navStateZh = NAV_STATE_ZH[navState] ?? navState

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
            {sseState.connected ? '在线' : '离线'}
          </span>
        </div>

        <div className="topbar-center">
          <span className="topbar-stat">
            <span className="stat-label">位置</span>
            <span className="stat-value">{posLabel}</span>
          </span>
          <span className="topbar-divider" />
          <span className="topbar-stat">
            <span className="stat-label">导航</span>
            <span className={`stat-value nav-state--${navState.toLowerCase()}`}>{navStateZh}</span>
          </span>
          {estop && (
            <>
              <span className="topbar-divider" />
              <span className="topbar-estop">急停</span>
            </>
          )}
        </div>

        <div className="topbar-right">
          <button className="btn-icon" aria-label="设置">
            <Settings size={16} />
          </button>
        </div>
      </header>

      {/* Tab bar */}
      <nav className="tab-bar" role="tablist">
        <button
          role="tab"
          aria-selected={activeTab === 'console'}
          className={`tab-btn ${activeTab === 'console' ? 'tab-btn--active' : ''}`}
          onClick={() => setActiveTab('console')}
        >
          控制台
        </button>
        <button
          role="tab"
          aria-selected={activeTab === 'map'}
          className={`tab-btn ${activeTab === 'map' ? 'tab-btn--active' : ''}`}
          onClick={() => setActiveTab('map')}
        >
          地图管理
        </button>
      </nav>

      {/* Main content */}
      <main className="main-content">
        {activeTab === 'console' && (
          <div className="main-grid">
            <section className="camera-section">
              <CameraFeed onStop={handleStop} estop={estop} />
            </section>
            <aside className="chat-section">
              <ChatPanel sseState={sseState} />
            </aside>
          </div>
        )}
        {activeTab === 'map' && <MapView />}
      </main>

      {/* Status bar */}
      <StatusBar sseState={sseState} uptimeSeconds={uptimeSeconds} />
    </div>
  )
}

export default App
