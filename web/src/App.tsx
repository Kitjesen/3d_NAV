import { useState, useEffect, useCallback } from 'react'
import { Camera, Globe2, Map as MapIcon, MessageSquare, RotateCcw } from 'lucide-react'
import { useSSE } from './hooks/useSSE'
import { useToast } from './hooks/useToast'
import { Topbar } from './components/Topbar'
import { TabBar } from './components/TabBar'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { GpsCard } from './components/GpsCard'
import { StatusBar } from './components/StatusBar'
import { MapView } from './components/MapView'
import { SlamPanel } from './components/SlamPanel'
import { SceneView } from './components/SceneView'
import { MiniMap } from './components/MiniMap'
import { FloatingWidget, resetAllLayouts } from './components/FloatingWidget'
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

      <main className="main-content" key={activeTab}>
        {activeTab === 'console' && (
          <div className="console-canvas" role="tabpanel" id="panel-console">
            <FloatingWidget
              id="camera"
              title="相机"
              titleIcon={<Camera size={13} />}
              defaultPos={{ x: 16, y: 16 }}
              defaultSize={{ w: 780, h: 560 }}
              minSize={{ w: 400, h: 300 }}
            >
              <CameraFeed onStop={handleStop} estop={estop} sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="chat"
              title="智能体"
              titleIcon={<MessageSquare size={13} />}
              defaultPos={{ x: 812, y: 420 }}
              defaultSize={{ w: 520, h: 440 }}
              minSize={{ w: 340, h: 280 }}
            >
              <ChatPanel sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="gps"
              title="定位"
              titleIcon={<Globe2 size={13} />}
              defaultPos={{ x: 812, y: 16 }}
              defaultSize={{ w: 340, h: 390 }}
              minSize={{ w: 280, h: 340 }}
            >
              <GpsCard sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="minimap"
              title="Mini Map"
              titleIcon={<MapIcon size={13} />}
              defaultPos={{ x: 1168, y: 16 }}
              defaultSize={{ w: 300, h: 390 }}
              minSize={{ w: 240, h: 260 }}
            >
              <MiniMap sseState={sseState} />
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

  // Dev preview: ?login forces the login page to render
  const forceLogin = typeof window !== 'undefined' &&
    new URLSearchParams(window.location.search).has('login')

  useEffect(() => {
    if (forceLogin) {
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
  }, [forceLogin])

  if (!authChecked) return null
  if (!loggedIn) return <LoginPage onLogin={() => setLoggedIn(true)} />
  return <Dashboard />
}

export default App
