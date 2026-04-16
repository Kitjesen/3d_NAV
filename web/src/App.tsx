import { useState, useEffect, useCallback } from 'react'
import { RotateCcw } from 'lucide-react'
import { useSSE } from './hooks/useSSE'
import { useToast } from './hooks/useToast'
import { Topbar } from './components/Topbar'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { GpsCard } from './components/GpsCard'
import { GnssCard } from './components/GnssCard'
import { GnssFusionCard } from './components/GnssFusionCard'
import { StatusBar } from './components/StatusBar'
import { MapView } from './components/MapView'
import { SlamPanel } from './components/SlamPanel'
import { SceneView } from './components/SceneView'
import { MiniMap } from './components/MiniMap'
import { FloatingWidget, resetAllLayouts } from './components/FloatingWidget'
import { ToastContainer } from './components/Toast'
import { LoginPage } from './components/LoginPage'
import { Landing } from './components/Landing'
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
      <Topbar sseState={sseState} activeTab={activeTab} onTabChange={setActiveTab} />

      <main className="main-content" key={activeTab}>
        {activeTab === 'console' && (
          <div className="console-canvas" role="tabpanel" id="panel-console">
            <FloatingWidget
              id="camera"
              defaultPos={{ x: 0, y: 0 }}
              defaultSize={{ w: 1000, h: 930 }}
              minSize={{ w: 400, h: 300 }}
            >
              <CameraFeed onStop={handleStop} estop={estop} sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="gps"
              defaultPos={{ x: 1016, y: 0 }}
              defaultSize={{ w: 440, h: 510 }}
              minSize={{ w: 280, h: 340 }}
            >
              <GpsCard sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="minimap"
              defaultPos={{ x: 1472, y: 0 }}
              defaultSize={{ w: 420, h: 510 }}
              minSize={{ w: 240, h: 260 }}
            >
              <MiniMap sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="chat"
              defaultPos={{ x: 1016, y: 526 }}
              defaultSize={{ w: 456, h: 404 }}
              minSize={{ w: 340, h: 280 }}
            >
              <ChatPanel sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="gnss"
              defaultPos={{ x: 1488, y: 526 }}
              defaultSize={{ w: 420, h: 404 }}
              minSize={{ w: 280, h: 340 }}
            >
              <GnssCard sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="gnss-fusion"
              defaultPos={{ x: 1488, y: 940 }}
              defaultSize={{ w: 420, h: 340 }}
              minSize={{ w: 280, h: 260 }}
            >
              <GnssFusionCard sseState={sseState} />
            </FloatingWidget>

            <button
              className="reset-layout-btn"
              onClick={resetAllLayouts}
              title="恢复默认布局"
            >
              <RotateCcw size={13} />
              重置布局
            </button>
          </div>
        )}
        {activeTab === 'scene' && <SceneView sseState={sseState} showToast={showToast} />}
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

  // Landing page: ?landing bypasses auth and shows the marketing page
  const isLanding = typeof window !== 'undefined' &&
    new URLSearchParams(window.location.search).has('landing')

  // Dev preview: ?login forces the login page to render
  const forceLogin = typeof window !== 'undefined' &&
    new URLSearchParams(window.location.search).has('login')

  useEffect(() => {
    // Landing page needs scroll — remove dashboard overflow:hidden from html/body
    if (isLanding) {
      document.documentElement.style.overflow = 'auto'
      document.documentElement.style.height = 'auto'
      document.body.style.overflow = 'auto'
      document.body.style.height = 'auto'
      const root = document.getElementById('root')
      if (root) { root.style.overflow = 'auto'; root.style.height = 'auto' }
    }
    return () => {
      if (isLanding) {
        document.documentElement.style.overflow = ''
        document.documentElement.style.height = ''
        document.body.style.overflow = ''
        document.body.style.height = ''
        const root = document.getElementById('root')
        if (root) { root.style.overflow = ''; root.style.height = '' }
      }
    }
  }, [isLanding])

  useEffect(() => {
    if (isLanding || forceLogin) {
      setLoggedIn(false)
      setAuthChecked(true)
      return
    }
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
  }, [isLanding, forceLogin])

  if (isLanding) return <Landing />
  if (!authChecked) return null
  if (!loggedIn) return <LoginPage onLogin={() => setLoggedIn(true)} />
  return <Dashboard />
}

export default App
