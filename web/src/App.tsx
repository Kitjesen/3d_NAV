import { useState, useEffect, useCallback } from 'react'
import { Navigation, Settings, Radio } from 'lucide-react'
import { useSSE } from './hooks/useSSE'
import { useToast } from './hooks/useToast'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { StatusBar } from './components/StatusBar'
import { MapView } from './components/MapView'
import { SlamPanel } from './components/SlamPanel'
import { ToastContainer } from './components/Toast'
import { LoginPage } from './components/LoginPage'
import './App.css'

type Tab = 'console' | 'map' | 'slam'

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

function Dashboard() {
  const sseState = useSSE('/api/v1/events')
  const { toasts, show: showToast, dismiss } = useToast()
  const [uptimeSeconds, setUptimeSeconds] = useState(0)
  const [activeTab, setActiveTab] = useState<Tab>('console')

  useEffect(() => {
    const t = setInterval(() => setUptimeSeconds(s => s + 1), 1000)
    return () => clearInterval(t)
  }, [])

  const handleStop = useCallback(async () => {
    try {
      await fetch('/api/v1/stop', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: '{}' })
    } catch { /* best-effort */ }
  }, [])

  const estop = sseState.safetyState?.estop ?? false
  const odom = sseState.odometry
  const posLabel = odom ? `(${odom.x.toFixed(1)}, ${odom.y.toFixed(1)})` : '--'
  const navState = sseState.missionStatus?.state ?? 'IDLE'
  const navStateZh = NAV_STATE_ZH[navState] ?? navState
  const slamMode = sseState.slamStatus?.mode ?? null
  const slamModeZh = slamMode ? (SLAM_MODE_ZH[slamMode] ?? slamMode) : null

  return (
    <div className="app">
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
          {slamModeZh && (
            <>
              <span className="topbar-divider" />
              <span className="topbar-stat">
                <span className="stat-label"><Radio size={10} style={{ display: 'inline', verticalAlign: 'middle', marginRight: 2 }} />SLAM</span>
                <span className={`stat-value slam-mode--${slamMode}`}>{slamModeZh}</span>
              </span>
            </>
          )}
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

      <nav className="tab-bar" role="tablist">
        <button
          role="tab" aria-selected={activeTab === 'console'}
          className={`tab-btn ${activeTab === 'console' ? 'tab-btn--active' : ''}`}
          onClick={() => setActiveTab('console')}
        >控制台</button>
        <button
          role="tab" aria-selected={activeTab === 'map'}
          className={`tab-btn ${activeTab === 'map' ? 'tab-btn--active' : ''}`}
          onClick={() => setActiveTab('map')}
        >地图管理</button>
        <button
          role="tab" aria-selected={activeTab === 'slam'}
          className={`tab-btn ${activeTab === 'slam' ? 'tab-btn--active' : ''}`}
          onClick={() => setActiveTab('slam')}
        >SLAM 模式</button>
      </nav>

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
        {activeTab === 'map' && <MapView showToast={showToast} />}
        {activeTab === 'slam' && <SlamPanel sseState={sseState} showToast={showToast} />}
      </main>

      <StatusBar sseState={sseState} uptimeSeconds={uptimeSeconds} />
      <ToastContainer toasts={toasts} dismiss={dismiss} />
    </div>
  )
}

function App() {
  const [authChecked, setAuthChecked] = useState(false)
  const [loggedIn, setLoggedIn] = useState(false)

  useEffect(() => {
    fetch('/api/v1/auth/check')
      .then(r => r.json())
      .then(data => {
        setLoggedIn(!data.auth_required)
        setAuthChecked(true)
      })
      .catch(() => {
        // Backend offline — skip auth
        setLoggedIn(true)
        setAuthChecked(true)
      })
  }, [])

  if (!authChecked) return null
  if (!loggedIn) return <LoginPage onLogin={() => setLoggedIn(true)} />
  return <Dashboard />
}

export default App
