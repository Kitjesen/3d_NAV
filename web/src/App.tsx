import { useState, useEffect, useCallback } from 'react'
import { useSSE } from './hooks/useSSE'
import { useToast } from './hooks/useToast'
import { Topbar } from './components/Topbar'
import { CameraFeed } from './components/CameraFeed'
import { ChatPanel } from './components/ChatPanel'
import { LocalizationCard } from './components/LocalizationCard'
import { StatusBar } from './components/StatusBar'
import { MapView } from './components/MapView'
import { SlamPanel } from './components/SlamPanel'
import { SceneView } from './components/SceneView'
import { MiniMap } from './components/MiniMap'
import { FloatingWidget } from './components/FloatingWidget'
import { ToastContainer } from './components/Toast'
import { LoginPage } from './components/LoginPage'
import { Landing } from './components/Landing'
import * as api from './services/api'
import type { Tab } from './types'
import './App.css'

interface WidgetBounds {
  width: number
  height: number
}

interface WidgetRect {
  x: number
  y: number
  w: number
  h: number
}

const CONSOLE_GAP = 16

function getConsoleMetrics(bounds: WidgetBounds): {
  camera: WidgetRect
  controls: WidgetRect
  chat: WidgetRect
} {
  const width = Math.max(1, bounds.width)
  const height = Math.max(1, bounds.height)
  const canUseColumns = width >= 1040

  if (!canUseColumns) {
    const controlsH = Math.min(380, Math.max(280, Math.round(height * 0.3)))
    const chatH = Math.min(Math.max(260, Math.round(height * 0.28)), Math.max(1, height - controlsH - CONSOLE_GAP * 2))
    const cameraH = Math.max(300, height - controlsH - chatH - CONSOLE_GAP * 2)
    const chatY = cameraH + controlsH + CONSOLE_GAP * 2

    return {
      camera: { x: 0, y: 0, w: width, h: cameraH },
      controls: { x: 0, y: cameraH + CONSOLE_GAP, w: width, h: controlsH },
      chat: { x: 0, y: chatY, w: width, h: chatH },
    }
  }

  const rightW = Math.min(876, Math.max(620, Math.round(width * 0.43)))
  const leftW = Math.max(400, width - rightW - CONSOLE_GAP)
  const rightX = leftW + CONSOLE_GAP
  const controlsH = Math.min(380, Math.max(280, Math.round(height * 0.42)))
  const chatY = controlsH + CONSOLE_GAP

  return {
    camera: { x: 0, y: 0, w: leftW, h: height },
    controls: { x: rightX, y: 0, w: rightW, h: controlsH },
    chat: { x: rightX, y: chatY, w: rightW, h: Math.max(1, height - chatY) },
  }
}

function cameraLayout(bounds: WidgetBounds) {
  return getConsoleMetrics(bounds).camera
}

function controlPanelLayout(bounds: WidgetBounds) {
  return getConsoleMetrics(bounds).controls
}

function chatLayout(bounds: WidgetBounds) {
  return getConsoleMetrics(bounds).chat
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
              responsiveLayout={cameraLayout}
            >
              <CameraFeed onStop={handleStop} estop={estop} sseState={sseState} />
            </FloatingWidget>

            <FloatingWidget
              id="control-panels"
              defaultPos={{ x: 1016, y: 0 }}
              defaultSize={{ w: 876, h: 380 }}
              minSize={{ w: 620, h: 280 }}
              responsiveLayout={controlPanelLayout}
              dragHandleLeft="59%"
            >
              <div className="control-panel-group">
                <LocalizationCard sseState={sseState} />
                <MiniMap sseState={sseState} />
              </div>
            </FloatingWidget>

            <FloatingWidget
              id="chat"
              defaultPos={{ x: 1016, y: 396 }}
              defaultSize={{ w: 876, h: 534 }}
              minSize={{ w: 340, h: 260 }}
              responsiveLayout={chatLayout}
            >
              <ChatPanel sseState={sseState} />
            </FloatingWidget>

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
  // Landing page: ?landing bypasses auth and shows the marketing page
  const isLanding = typeof window !== 'undefined' &&
    new URLSearchParams(window.location.search).has('landing')

  // Dev preview: ?login forces the login page to render
  const forceLogin = typeof window !== 'undefined' &&
    new URLSearchParams(window.location.search).has('login')
  const bypassAuthCheck = isLanding || forceLogin

  const [authChecked, setAuthChecked] = useState(bypassAuthCheck)
  const [loggedIn, setLoggedIn] = useState(false)

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
    if (bypassAuthCheck) return
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
  }, [bypassAuthCheck])

  if (isLanding) return <Landing />
  if (!authChecked) return null
  if (!loggedIn) return <LoginPage onLogin={() => setLoggedIn(true)} />
  return <Dashboard />
}

export default App
