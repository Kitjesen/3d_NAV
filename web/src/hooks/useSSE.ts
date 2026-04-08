import { useEffect, useRef, useState, useCallback } from 'react'
import type { SSEState, SSEEvent } from '../types'

// Re-export types for backward compatibility
export type {
  OdometryEvent,
  MissionStatusEvent,
  SafetyStateEvent,
  SceneGraphEvent,
  HeartbeatEvent,
  SlamStatusEvent,
  RobotStatusEvent,
  GlobalPathEvent,
  PathPoint,
  SSEEvent,
  SSEState,
} from '../types'

const INITIAL_STATE: SSEState = {
  odometry: null,
  missionStatus: null,
  safetyState: null,
  sceneGraph: null,
  slamStatus: null,
  robotStatus: null,
  globalPath: null,
  lastHeartbeat: null,
  connected: false,
  events: [],
}

export function useSSE(url: string = '/api/v1/events') {
  const [state, setState] = useState<SSEState>(INITIAL_STATE)
  const esRef = useRef<EventSource | null>(null)
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const mountedRef = useRef(true)

  const connect = useCallback(() => {
    if (!mountedRef.current) return
    if (esRef.current) {
      esRef.current.close()
      esRef.current = null
    }

    const es = new EventSource(url)
    esRef.current = es

    es.onopen = () => {
      if (!mountedRef.current) return
      setState(prev => ({ ...prev, connected: true }))
    }

    es.onmessage = (e: MessageEvent) => {
      if (!mountedRef.current) return
      // Handle NDJSON: each line is a JSON object
      const lines = (e.data as string).split('\n').filter(l => l.trim())
      for (const line of lines) {
        try {
          const event = JSON.parse(line) as SSEEvent
          setState(prev => {
            const next = { ...prev, events: [...prev.events.slice(-99), event] }
            switch (event.type) {
              case 'odometry':
                next.odometry = event
                break
              case 'mission_status':
                next.missionStatus = event
                break
              case 'safety_state':
                next.safetyState = event
                break
              case 'scene_graph':
                next.sceneGraph = event
                break
              case 'slam_status':
                next.slamStatus = event
                break
              case 'robot_status':
                next.robotStatus = event
                break
              case 'global_path':
                next.globalPath = event
                break
              case 'heartbeat':
                next.lastHeartbeat = Date.now()
                break
            }
            return next
          })
        } catch {
          // malformed line — ignore
        }
      }
    }

    es.onerror = () => {
      if (!mountedRef.current) return
      es.close()
      esRef.current = null
      setState(prev => ({ ...prev, connected: false }))
      reconnectTimer.current = setTimeout(() => {
        if (mountedRef.current) connect()
      }, 3000)
    }
  }, [url])

  useEffect(() => {
    mountedRef.current = true
    connect()
    return () => {
      mountedRef.current = false
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
      if (esRef.current) {
        esRef.current.close()
        esRef.current = null
      }
    }
  }, [connect])

  return state
}
