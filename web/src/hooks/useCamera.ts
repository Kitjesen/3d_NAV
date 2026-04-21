import { useEffect, useRef, useState, useCallback } from 'react'

export interface CameraState {
  imgSrc: string | null
  connected: boolean
  reconnect: () => void
}

export function useCamera(url: string = '/ws/teleop'): CameraState {
  const [imgSrc, setImgSrc] = useState<string | null>(null)
  const [connected, setConnected] = useState(false)
  const wsRef = useRef<WebSocket | null>(null)
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const mountedRef = useRef(true)
  const prevBlobRef = useRef<string | null>(null)
  const connectRef = useRef<() => void>(() => {})

  const connect = useCallback(() => {
    if (!mountedRef.current) return
    if (wsRef.current) {
      wsRef.current.close()
      wsRef.current = null
    }

    // Empty url signals "stay idle" — used by CameraFeed when the WebRTC
    // path is in charge.  Avoids opening a phantom /ws/teleop connection.
    if (!url) return

    // Build absolute ws URL from current location + path
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const wsUrl = `${protocol}//${window.location.host}${url}`

    const ws = new WebSocket(wsUrl)
    ws.binaryType = 'blob'
    wsRef.current = ws

    ws.onopen = () => {
      if (!mountedRef.current) return
      setConnected(true)
    }

    ws.onmessage = (e: MessageEvent) => {
      if (!mountedRef.current) return
      if (e.data instanceof Blob) {
        // Revoke previous blob URL to avoid memory leak
        if (prevBlobRef.current) {
          URL.revokeObjectURL(prevBlobRef.current)
        }
        const objectUrl = URL.createObjectURL(e.data)
        prevBlobRef.current = objectUrl
        setImgSrc(objectUrl)
      }
      // JSON frames (e.g. ping/control echo) — ignore
    }

    ws.onclose = () => {
      if (!mountedRef.current) return
      setConnected(false)
      reconnectTimer.current = setTimeout(() => {
        if (mountedRef.current) connectRef.current()
      }, 3000)
    }

    ws.onerror = () => {
      ws.close()
    }
  }, [url])

  const reconnect = useCallback(() => {
    if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
    connect()
  }, [connect])

  useEffect(() => {
    connectRef.current = connect
  }, [connect])

  useEffect(() => {
    mountedRef.current = true
    connect()
    return () => {
      mountedRef.current = false
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
      if (wsRef.current) {
        wsRef.current.close()
        wsRef.current = null
      }
      if (prevBlobRef.current) {
        URL.revokeObjectURL(prevBlobRef.current)
      }
    }
  }, [connect])

  return { imgSrc, connected, reconnect }
}
