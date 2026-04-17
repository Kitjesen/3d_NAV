/**
 * BagRecorder — Topbar 上的 rosbag 快捷录制按钮.
 *
 * 点击一次 → 立即开始录制(默认 10 分钟上限,中途可手动停)
 * 录制中 → 按钮变红 + 显示 ● REC + 秒数 + MB,再点一次停止
 * 停止后 → 短暂显示"已保存 path size"
 *
 * 后端 3 个 route 见 gateway_module.py:
 *   POST /api/v1/bag/start  {duration, prefix}
 *   POST /api/v1/bag/stop
 *   GET  /api/v1/bag/status
 *
 * 录制内容:见 scripts/record_bag.sh TOPICS[]
 *   /nav/{lidar_scan,imu,odometry,map_cloud,registered_cloud,cmd_vel,goal_pose}
 *   /localization_quality
 *   /exploration/{way_point,path,runtime,finish}
 *   /camera/camera_info
 *   (不含相机 raw, 避免数据爆炸)
 */
import { useEffect, useRef, useState } from 'react'
import { Circle, Square } from 'lucide-react'
import styles from './BagRecorder.module.css'

interface BagStatus {
  recording: boolean
  path: string
  duration_s: number
  size_bytes: number
  pid: number | null
  exit_code: number | null
  disk_free: number
  disk_total: number
}

export function BagRecorder() {
  const [status, setStatus] = useState<BagStatus | null>(null)
  const [busy, setBusy] = useState(false)
  const [flash, setFlash] = useState<string | null>(null)
  const pollRef = useRef<ReturnType<typeof setInterval> | null>(null)

  const poll = async () => {
    try {
      const r = await fetch('/api/v1/bag/status')
      if (r.ok) setStatus(await r.json())
    } catch { /* gateway offline — ignore */ }
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
      const r = await fetch('/api/v1/bag/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ duration: 600, prefix: 'web' }),
      })
      if (!r.ok) {
        const err = await r.json().catch(() => ({ error: `HTTP ${r.status}` }))
        setFlash(`启动失败: ${err.error || err.detail || r.status}`)
      } else {
        setFlash('录制开始')
      }
    } catch (e) {
      setFlash(`启动失败: ${e instanceof Error ? e.message : e}`)
    } finally {
      setBusy(false)
      poll()
      setTimeout(() => setFlash(null), 2000)
    }
  }

  const stop = async () => {
    setBusy(true)
    try {
      const r = await fetch('/api/v1/bag/stop', { method: 'POST' })
      if (!r.ok) {
        setFlash('停止失败')
      } else {
        const sz = status ? fmtMB(status.size_bytes) : '?'
        setFlash(`已保存 ${sz}`)
      }
    } catch (e) {
      setFlash(`停止失败: ${e instanceof Error ? e.message : e}`)
    } finally {
      setBusy(false)
      setTimeout(() => { poll(); setFlash(null) }, 2500)
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
          title={status?.path}
        >
          <Square size={10} fill="currentColor" />
          <span className={styles.recDot} />
          <span className={styles.meta}>
            {fmtDuration(status?.duration_s ?? 0)}  ·  {fmtMB(status?.size_bytes ?? 0)}
          </span>
        </button>
      ) : (
        <button
          className={styles.btnIdle}
          onClick={start}
          disabled={busy}
          title="录制 rosbag — 默认 10 分钟上限,可手动停"
        >
          <Circle size={11} fill="currentColor" />
          <span>录制</span>
        </button>
      )}
    </div>
  )
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
