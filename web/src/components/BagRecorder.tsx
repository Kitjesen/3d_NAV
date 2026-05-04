import { useEffect, useRef, useState } from 'react'
import { Circle, Square } from 'lucide-react'
import * as api from '../services/api'
import type { BagStatusResponse } from '../types'
import styles from './BagRecorder.module.css'

export function BagRecorder() {
  const [status, setStatus] = useState<BagStatusResponse | null>(null)
  const [busy, setBusy] = useState(false)
  const [flash, setFlash] = useState<string | null>(null)
  const pollRef = useRef<ReturnType<typeof setInterval> | null>(null)

  const poll = async () => {
    try {
      setStatus(await api.fetchBagStatus())
    } catch {
      // Bag status is diagnostic; the dashboard can keep running offline.
    }
  }

  useEffect(() => {
    poll()
    pollRef.current = setInterval(poll, 1000)
    return () => {
      if (pollRef.current) clearInterval(pollRef.current)
    }
  }, [])

  const start = async () => {
    setBusy(true)
    try {
      await api.startBagRecording(600, 'web')
      setFlash('录制开始')
    } catch (e) {
      setFlash(`启动失败: ${apiErrorMessage(e)}`)
    } finally {
      setBusy(false)
      poll()
      setTimeout(() => setFlash(null), 2000)
    }
  }

  const stop = async () => {
    setBusy(true)
    try {
      await api.stopBagRecording()
      const sz = status ? fmtMB(status.size_bytes) : '?'
      setFlash(`已保存 ${sz}`)
    } catch (e) {
      setFlash(`停止失败: ${apiErrorMessage(e)}`)
    } finally {
      setBusy(false)
      setTimeout(() => {
        poll()
        setFlash(null)
      }, 2500)
    }
  }

  const recording = !!status?.recording

  return (
    <div className={styles.wrap}>
      {flash && <span className={styles.flash}>{flash}</span>}
      {recording ? (
        <button
          className={styles.btnRec}
          onClick={stop}
          disabled={busy}
          title={`正在录制: ${status?.path}\n点击停止`}
        >
          <Square size={10} fill="currentColor" />
          <span className={styles.recDot} />
          <span className={styles.meta}>
            {fmtDuration(status?.duration_s ?? 0)}  /  {fmtMB(status?.size_bytes ?? 0)}
          </span>
        </button>
      ) : (
        <button
          className={styles.btnIdle}
          onClick={start}
          disabled={busy}
          title={
            '录制 rosbag，默认 10 分钟上限，可手动停止\n' +
            '录制话题:\n' +
            '  /nav/lidar_scan  MID-360 Livox\n' +
            '  /nav/imu  /nav/odometry  /nav/map_cloud\n' +
            '  /nav/registered_cloud  /nav/localization_quality\n' +
            '  /nav/localization_health\n' +
            '  /nav/goal_pose  /nav/cmd_vel\n' +
            '  /exploration/way_point  /path  /runtime  /finish\n' +
            '  /camera/camera_info  (不含相机 raw)'
          }
        >
          <Circle size={11} fill="currentColor" />
          <span>录制</span>
        </button>
      )}
    </div>
  )
}

function apiErrorMessage(error: unknown): string {
  if (api.isGatewayApiError(error)) {
    return error.body?.error || error.body?.message || error.message
  }
  return error instanceof Error ? error.message : String(error)
}

function fmtDuration(s: number): string {
  const m = Math.floor(s / 60)
  const r = Math.floor(s % 60)
  return `${m.toString().padStart(2, '0')}:${r.toString().padStart(2, '0')}`
}

function fmtMB(b: number): string {
  if (b < 1024 * 1024) return `${(b / 1024).toFixed(0)} KB`
  if (b < 1024 * 1024 * 1024) return `${(b / 1024 / 1024).toFixed(1)} MB`
  return `${(b / 1024 / 1024 / 1024).toFixed(2)} GB`
}
