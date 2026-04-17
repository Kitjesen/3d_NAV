/**
 * useWebRTC — pull the robot's camera over WebRTC.
 *
 * Signalling is a single POST to /api/v1/webrtc/offer; the browser sends
 * its SDP offer and receives the server's answer in one round-trip.  On
 * LAN with host-only ICE candidates this completes in <200 ms and yields
 * a H.264 stream that the browser decodes in the media pipeline (not the
 * main JS thread), usually below 150 ms glass-to-glass.
 *
 * Returns ``{ stream, connected, error, reconnect }``.  ``error === 'unavailable'``
 * means the gateway returned 503 — aiortc isn't installed on the robot and
 * the caller should fall back to the JPEG-over-WS path.
 */
import { useCallback, useEffect, useRef, useState } from 'react'

export type WebRTCError = null | 'unavailable' | 'signalling' | 'ice'

export interface WebRTCStats {
  enabled:      boolean
  active_peers: number
  bitrate_bps:  number
  fps:          number
  max_bitrate:  number
  /** Avg encode time per frame across all peers, ms. */
  encode_avg_ms?: number
}

export interface WebRTCState {
  stream:    MediaStream | null
  connected: boolean
  error:     WebRTCError
  reconnect: () => void
  stats:     WebRTCStats | null
}

export function useWebRTC(path: string = '/api/v1/webrtc/offer'): WebRTCState {
  const [stream, setStream]       = useState<MediaStream | null>(null)
  const [connected, setConnected] = useState(false)
  const [error, setError]         = useState<WebRTCError>(null)
  const [nonce, setNonce]         = useState(0)
  const [stats, setStats]         = useState<WebRTCStats | null>(null)
  const pcRef     = useRef<RTCPeerConnection | null>(null)
  const abortRef  = useRef<AbortController | null>(null)
  const mountedRef = useRef(true)

  const teardown = useCallback(() => {
    abortRef.current?.abort()
    abortRef.current = null
    const pc = pcRef.current
    pcRef.current = null
    if (pc) {
      pc.getReceivers().forEach(r => { try { r.track.stop() } catch { /* noop */ } })
      try { pc.close() } catch { /* noop */ }
    }
  }, [])

  const reconnect = useCallback(() => {
    teardown()
    setConnected(false)
    setError(null)
    setStream(null)
    setNonce(n => n + 1)
  }, [teardown])

  useEffect(() => {
    mountedRef.current = true
    const abort = new AbortController()
    abortRef.current = abort
    // Host-only ICE for LAN deployment — no STUN/TURN needed, also
    // prevents a hang when the robot can't reach external servers.
    const pc = new RTCPeerConnection({ iceServers: [] })
    pcRef.current = pc

    pc.addTransceiver('video', { direction: 'recvonly' })

    pc.addEventListener('track', ev => {
      if (!mountedRef.current) return
      setStream(ev.streams[0] ?? new MediaStream([ev.track]))
    })

    pc.addEventListener('connectionstatechange', () => {
      if (!mountedRef.current) return
      const s = pc.connectionState
      if (s === 'connected') setConnected(true)
      if (s === 'failed' || s === 'disconnected' || s === 'closed') {
        setConnected(false)
        if (s === 'failed') setError('ice')
      }
    })

    ;(async () => {
      try {
        const offer = await pc.createOffer()
        await pc.setLocalDescription(offer)
        // Wait for ICE gathering to settle so the POST carries every
        // candidate — avoids trickle-ICE, which would need an open
        // long-lived signalling channel.
        if (pc.iceGatheringState !== 'complete') {
          await new Promise<void>(resolve => {
            const onState = () => {
              if (pc.iceGatheringState === 'complete') {
                pc.removeEventListener('icegatheringstatechange', onState)
                resolve()
              }
            }
            pc.addEventListener('icegatheringstatechange', onState)
            // Hard cap: don't wait forever on a flaky host-only network.
            setTimeout(() => {
              pc.removeEventListener('icegatheringstatechange', onState)
              resolve()
            }, 1500)
          })
        }

        const local = pc.localDescription
        if (!local) throw new Error('no local SDP')
        const resp = await fetch(path, {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ sdp: local.sdp, type: local.type }),
          signal: abort.signal,
        })
        if (resp.status === 503) {
          setError('unavailable')
          return
        }
        if (!resp.ok) {
          setError('signalling')
          return
        }
        const answer = await resp.json() as RTCSessionDescriptionInit
        if (!mountedRef.current) return
        await pc.setRemoteDescription(answer)
      } catch (e) {
        if ((e as { name?: string }).name === 'AbortError') return
        setError('signalling')
      }
    })()

    return () => {
      mountedRef.current = false
      teardown()
    }
  }, [path, nonce, teardown])

  // Stats polling: 1 Hz while the PC is live.  Cheap — the server does
  // aggregation.  Stops when the peer is closed or the hook unmounts.
  useEffect(() => {
    if (!connected) {
      setStats(null)
      return
    }
    let cancelled = false
    const tick = async () => {
      try {
        const resp = await fetch('/api/v1/webrtc/stats', { cache: 'no-store' })
        if (!resp.ok) return
        const json = await resp.json() as WebRTCStats
        if (!cancelled) setStats(json)
      } catch { /* network blip — next tick retries */ }
    }
    tick()
    const id = setInterval(tick, 1000)
    return () => { cancelled = true; clearInterval(id) }
  }, [connected])

  return { stream, connected, error, reconnect, stats }
}
