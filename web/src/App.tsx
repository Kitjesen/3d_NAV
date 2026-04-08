import { useState, useEffect, useCallback } from 'react'
import { useSSE } from './hooks/useSSE'
import { useToast } from './hooks/useToast'
import { Topbar } from './components/Topbar'
import { TabBar } from './components/TabBar'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { StatusBar } from './components/StatusBar'
import { MapView } from './components/MapView'
import { SlamPanel } from './components/SlamPanel'
import { PathView } from './components/PathView'
import { ToastContainer } from './components/Toast'
import { LoginPage } from './components/LoginPage'
import * as api from './services/api'
import type { Tab } from './types'
import './App.css'

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
      await api.sendStop()
    } catch { /* best-effort */ }
  }, [])

  const estop = sseState.safetyState?.estop ?? false

  return (
    <div className="app">
      <Topbar sseState={sseState} />
      <TabBar activeTab={activeTab} onTabChange={setActiveTab} />

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
        {activeTab === 'path' && <PathView sseState={sseState} showToast={showToast} />}
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
    api.checkAuth()
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
