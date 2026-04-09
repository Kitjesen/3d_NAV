import { useEffect, useRef, useState, useCallback } from 'react'
import { ScanLine } from 'lucide-react'
import styles from './PointCloudViewer.module.css'

interface Props {
  mapName: string | null
}

type Status = 'idle' | 'loading' | 'done' | 'error'

// Parse PCD file (both ASCII and binary) — returns interleaved [x0,y0, x1,y1, ...]
function parsePcd(buffer: ArrayBuffer): { pts: Float32Array; nPoints: number } | null {
  const bytes = new Uint8Array(buffer)

  // Locate "DATA " line by scanning raw bytes (safe for binary PCD body)
  let headerEndByte = -1
  let dataFormat = ''
  for (let i = 0; i < Math.min(bytes.length - 8, 8192); i++) {
    if (bytes[i] === 68 && bytes[i+1] === 65 && bytes[i+2] === 84 &&
        bytes[i+3] === 65 && bytes[i+4] === 32) {
      let j = i + 5
      while (j < bytes.length && bytes[j] !== 10 && bytes[j] !== 13)
        dataFormat += String.fromCharCode(bytes[j++])
      if (bytes[j] === 13) j++ // \r
      if (bytes[j] === 10) j++ // \n
      headerEndByte = j
      break
    }
  }
  if (headerEndByte === -1) return null

  const header = new TextDecoder().decode(bytes.slice(0, headerEndByte))
  const fieldsM = header.match(/^FIELDS\s+(.+)$/m)
  const pointsM = header.match(/^POINTS\s+(\d+)$/m)
  const sizeM   = header.match(/^SIZE\s+(.+)$/m)

  if (!fieldsM) return null
  const fields = fieldsM[1].trim().split(/\s+/)
  const xIdx = fields.indexOf('x')
  const yIdx = fields.indexOf('y')
  if (xIdx === -1 || yIdx === -1) return null

  const fmt = dataFormat.trim()

  if (fmt === 'ascii') {
    const text = new TextDecoder().decode(bytes.slice(headerEndByte))
    const lines = text.trim().split('\n')
    const max = Math.min(lines.length, 300_000)
    const pts = new Float32Array(max * 2)
    let n = 0
    for (let i = 0; i < max; i++) {
      const p = lines[i].trim().split(/\s+/)
      const x = parseFloat(p[xIdx]), y = parseFloat(p[yIdx])
      if (isFinite(x) && isFinite(y)) { pts[n++] = x; pts[n++] = y }
    }
    return { pts: pts.slice(0, n), nPoints: n >> 1 }
  }

  if (fmt === 'binary') {
    const sizes = sizeM
      ? sizeM[1].trim().split(/\s+/).map(Number)
      : fields.map(() => 4)
    const stride = sizes.reduce((a, b) => a + b, 0)
    const offsets: number[] = [0]
    for (let i = 0; i < sizes.length - 1; i++) offsets.push(offsets[i] + sizes[i])
    const xOff = offsets[xIdx]
    const yOff = offsets[yIdx]

    const nPoints = pointsM
      ? parseInt(pointsM[1])
      : Math.floor((bytes.length - headerEndByte) / stride)
    const total = Math.min(nPoints, 500_000)

    const dv = new DataView(buffer, headerEndByte)
    const pts = new Float32Array(total * 2)
    let n = 0
    for (let i = 0; i < total; i++) {
      const base = i * stride
      if (base + stride > dv.byteLength) break
      const x = dv.getFloat32(base + xOff, true)
      const y = dv.getFloat32(base + yOff, true)
      if (isFinite(x) && isFinite(y)) { pts[n++] = x; pts[n++] = y }
    }
    return { pts: pts.slice(0, n), nPoints: n >> 1 }
  }

  return null // binary_compressed not supported
}

const CANVAS_W = 800
const CANVAS_H = 520

function render(canvas: HTMLCanvasElement, pts: Float32Array) {
  const ctx = canvas.getContext('2d')
  if (!ctx) return

  ctx.fillStyle = '#080c14'
  ctx.fillRect(0, 0, CANVAS_W, CANVAS_H)

  if (pts.length === 0) return

  // Compute bounds
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity
  for (let i = 0; i < pts.length; i += 2) {
    if (pts[i]   < minX) minX = pts[i]
    if (pts[i]   > maxX) maxX = pts[i]
    if (pts[i+1] < minY) minY = pts[i+1]
    if (pts[i+1] > maxY) maxY = pts[i+1]
  }

  const pad = 28
  const rangeX = maxX - minX || 1
  const rangeY = maxY - minY || 1
  const scale = Math.min((CANVAS_W - pad*2) / rangeX, (CANVAS_H - pad*2) / rangeY)
  const ox = pad + ((CANVAS_W - pad*2) - rangeX * scale) / 2
  const oy = pad + ((CANVAS_H - pad*2) - rangeY * scale) / 2

  // Downsample large clouds
  const step = pts.length > 400_000 ? 6 : pts.length > 100_000 ? 3 : 2

  ctx.fillStyle = '#00ff88'
  for (let i = 0; i < pts.length; i += step) {
    const px = (ox + (pts[i]   - minX) * scale) | 0
    const py = (CANVAS_H - (oy + (pts[i+1] - minY) * scale)) | 0
    ctx.fillRect(px, py, 1, 1)
  }

  // Draw origin cross
  const ox0 = (ox + (0 - minX) * scale) | 0
  const oy0 = (CANVAS_H - (oy + (0 - minY) * scale)) | 0
  ctx.strokeStyle = 'rgba(255,255,255,0.25)'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(ox0 - 8, oy0); ctx.lineTo(ox0 + 8, oy0)
  ctx.moveTo(ox0, oy0 - 8); ctx.lineTo(ox0, oy0 + 8)
  ctx.stroke()
}

export function PointCloudViewer({ mapName }: Props) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const [status, setStatus] = useState<Status>('idle')
  const [info, setInfo] = useState('')

  const load = useCallback(async (name: string) => {
    setStatus('loading')
    setInfo('')
    try {
      const res = await fetch(`/api/v1/maps/${encodeURIComponent(name)}/pcd`)
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      const buf = await res.arrayBuffer()
      const result = parsePcd(buf)
      if (!result) throw new Error('parse failed')
      const canvas = canvasRef.current
      if (canvas) render(canvas, result.pts)
      setInfo(`${(result.nPoints / 1000).toFixed(0)}k pts`)
      setStatus('done')
    } catch {
      setStatus('error')
    }
  }, [])

  useEffect(() => {
    if (mapName) load(mapName)
    else setStatus('idle')
  }, [mapName, load])

  return (
    <div className={styles.viewer}>
      <div className={styles.header}>
        <span className={styles.title}><ScanLine size={13} /> 点云预览</span>
        {mapName && <span className={styles.mapLabel}>{mapName}</span>}
        {info && <span className={styles.info}>{info}</span>}
      </div>
      <div className={styles.body}>
        {status === 'idle' && (
          <div className={styles.placeholder}>← 点击列表中的「预览」加载点云</div>
        )}
        {status === 'loading' && <div className={styles.placeholder}>加载中…</div>}
        {status === 'error' && (
          <div className={styles.placeholder}>加载失败<br /><small>无 PCD 文件或格式不支持</small></div>
        )}
        <canvas
          ref={canvasRef}
          width={CANVAS_W}
          height={CANVAS_H}
          className={styles.canvas}
          style={{ display: status === 'done' ? 'block' : 'none' }}
        />
      </div>
    </div>
  )
}
