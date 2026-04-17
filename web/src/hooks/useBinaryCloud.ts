/**
 * useBinaryCloud — opens /ws/cloud, decodes frames in a WebWorker, and
 * exposes the latest Float32Array buffers for Scene3D to consume.
 *
 * Three properties to keep the main thread quiet:
 *   1. WebSocket binaryType='arraybuffer' so the worker receives the raw
 *      bytes without going through Blob → ArrayBuffer round trips.
 *   2. The worker is a single long-lived instance (via vite worker import)
 *      so we don't pay for spin-up per frame.
 *   3. Worker outputs are Transferable, so the main thread receives the
 *      ArrayBuffer at zero copy cost.
 */
import { useEffect, useRef, useState } from 'react'

import CloudDecoderWorker from '../workers/cloudDecoder.ts?worker'

export interface BinaryCloud {
  positions: Float32Array
  colors: Float32Array
  count: number
  seq: number
  connected: boolean
}

const EMPTY: BinaryCloud = {
  positions: new Float32Array(0),
  colors: new Float32Array(0),
  count: 0,
  seq: 0,
  connected: false,
}

export function useBinaryCloud(path: string = '/ws/cloud'): BinaryCloud {
  const [cloud, setCloud] = useState<BinaryCloud>(EMPTY)
  const wsRef = useRef<WebSocket | null>(null)
  const workerRef = useRef<Worker | null>(null)
  const reconnectTimer = useRef<ReturnType<typeof setTimeout> | null>(null)
  const mountedRef = useRef(true)

  useEffect(() => {
    mountedRef.current = true
    const worker = new CloudDecoderWorker()
    workerRef.current = worker

    worker.onmessage = (e: MessageEvent) => {
      if (!mountedRef.current) return
      const m = e.data as {
        type: string
        positions: Float32Array
        colors: Float32Array
        count: number
        seq: number
      }
      if (m.type !== 'cloud') return
      setCloud(prev => ({
        positions: m.positions,
        colors: m.colors,
        count: m.count,
        seq: m.seq,
        connected: prev.connected,
      }))
    }

    const connect = () => {
      if (!mountedRef.current) return
      const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
      const ws = new WebSocket(`${proto}//${window.location.host}${path}`)
      ws.binaryType = 'arraybuffer'
      wsRef.current = ws

      ws.onopen = () => {
        if (!mountedRef.current) return
        setCloud(prev => ({ ...prev, connected: true }))
      }

      ws.onmessage = (e: MessageEvent) => {
        if (!mountedRef.current) return
        if (!(e.data instanceof ArrayBuffer)) return
        // Transfer the raw bytes to the worker — main thread keeps no copy.
        worker.postMessage(e.data, [e.data])
      }

      const reopen = () => {
        if (!mountedRef.current) return
        setCloud(prev => ({ ...prev, connected: false }))
        reconnectTimer.current = setTimeout(connect, 3000)
      }
      ws.onclose = reopen
      ws.onerror = () => ws.close()
    }
    connect()

    return () => {
      mountedRef.current = false
      if (reconnectTimer.current) clearTimeout(reconnectTimer.current)
      if (wsRef.current) {
        wsRef.current.close()
        wsRef.current = null
      }
      worker.terminate()
      workerRef.current = null
    }
  }, [path])

  return cloud
}
