import { Satellite, SatelliteDish } from 'lucide-react'
import type { SSEState } from '../types'
import styles from './GnssCard.module.css'

/** Fix-type string from backend GnssStatus (matches GnssFixType enum names). */
type FixType =
  | 'NO_FIX' | 'SINGLE' | 'DGPS' | 'PPS'
  | 'RTK_FIXED' | 'RTK_FLOAT' | 'ESTIMATED' | 'MANUAL' | 'SIMULATION'

interface GnssStatusData {
  fix_type?: FixType
  num_sat?: number
  num_sat_used?: number
  hdop?: number
  age_s?: number
  rtcm_age_s?: number
  receiver?: string
  link_ok?: boolean
  horizontal_std_m?: number
  vertical_std_m?: number
  is_healthy?: boolean
  // ENU pose (from gnss_odom port, flattened by backend)
  east?: number
  north?: number
}

interface GnssCardProps {
  sseState: SSEState
}

/** Map fix type to label + CSS class for the badge. */
function fixBadgeInfo(fix: FixType | undefined): { label: string; cls: string } {
  switch (fix) {
    case 'RTK_FIXED':
      return { label: 'RTK FIX', cls: styles.fixRtkFixed }
    case 'RTK_FLOAT':
      return { label: 'RTK FLOAT', cls: styles.fixRtkFloat }
    case 'DGPS':
      return { label: 'DGPS', cls: styles.fixSingle }
    case 'SINGLE':
      return { label: 'SINGLE', cls: styles.fixSingle }
    case 'ESTIMATED':
      return { label: 'DR', cls: styles.fixSingle }
    case 'SIMULATION':
      return { label: 'SIM', cls: styles.fixRtkFloat }
    default:
      return { label: '无定位', cls: styles.fixNone }
  }
}

/** Deterministic satellite position on sky chart from index.
 *  Uses fibonacci spiral so satellites look natural. */
function satPos(idx: number, total: number): { x: number; y: number } {
  // Golden-angle spiral within a disc of radius 50
  const golden = Math.PI * (3 - Math.sqrt(5))
  const r = 50 * Math.sqrt((idx + 0.5) / total)
  const theta = idx * golden
  return { x: 60 + r * Math.cos(theta), y: 60 + r * Math.sin(theta) }
}

export function GnssCard({ sseState }: GnssCardProps) {
  // The GnssStatus is surfaced on the SSE stream as a generic event.
  // Backend TODO: expose it as `sseState.gnssStatus` — for now we
  // scan the events array for the most recent gnss_status event.
  // Cast through unknown since SSEEvent union doesn't include gnss yet.
  const gnssEvent = (
    sseState.events as unknown as Array<{ type?: string } & GnssStatusData>
  )
    .slice()
    .reverse()
    .find((e) => e?.type === 'gnss_status')

  const hasData = gnssEvent != null && gnssEvent.link_ok

  // Offline state — driver not connected
  if (!hasData) {
    return (
      <div className={styles.card}>
        <div className={styles.header}>
          <span className={styles.title}>
            <span className={styles.titleDot} />
            GNSS
          </span>
          <span className={styles.fixNone}>离线</span>
        </div>
        <div className={styles.offline}>
          <SatelliteDish size={32} strokeWidth={1.2} />
          <div>GNSS 驱动未连接</div>
          <div className={styles.offlineHint}>
            检查 <code>sudo systemctl status gnss</code><br/>
            或启用 <code>gnss.enabled</code> in robot_config.yaml
          </div>
        </div>
      </div>
    )
  }

  const fix = gnssEvent.fix_type ?? 'NO_FIX'
  const badge = fixBadgeInfo(fix)
  const numSat = gnssEvent.num_sat ?? 0
  const numSatUsed = gnssEvent.num_sat_used ?? 0
  const hdop = gnssEvent.hdop ?? 99.9
  const hStd = gnssEvent.horizontal_std_m
  const east = gnssEvent.east
  const north = gnssEvent.north

  // Sky chart: first numSatUsed are "used" (bright), rest are "seen" (dim)
  const skyDots = Array.from({ length: numSat }, (_, i) => {
    const pos = satPos(i, Math.max(numSat, 8))
    const used = i < numSatUsed
    return { ...pos, used }
  })

  const isLive = fix === 'RTK_FIXED' || fix === 'RTK_FLOAT'

  // Formatters
  const fmt = (n: number | undefined, digits = 2, unit = '') =>
    typeof n === 'number' && Number.isFinite(n)
      ? `${n.toFixed(digits)}${unit}`
      : '—'

  const hdopCls =
    hdop < 1.5 ? styles.statValueHl
    : hdop < 3.0 ? styles.statValue
    : styles.statValueWarn

  const hStdCls =
    typeof hStd === 'number' && hStd < 0.05 ? styles.statValueHl
    : typeof hStd === 'number' && hStd < 0.5 ? styles.statValue
    : styles.statValueWarn

  return (
    <div className={styles.card}>
      <div className={styles.header}>
        <span className={styles.title}>
          <span className={`${styles.titleDot} ${isLive ? styles.live : ''}`} />
          GNSS
        </span>
        <span className={badge.cls}>{badge.label}</span>
      </div>

      <div className={styles.skyView} aria-label="卫星天空视图">
        <div className={styles.skyCircle}>
          <div className={styles.skyRing} />
          <span className={`${styles.skyLabel} ${styles.skyN}`}>N</span>
          <span className={`${styles.skyLabel} ${styles.skyE}`}>E</span>
          <span className={`${styles.skyLabel} ${styles.skyS}`}>S</span>
          <span className={`${styles.skyLabel} ${styles.skyW}`}>W</span>
          <div className={styles.skyRobot} />
          {skyDots.map((d, i) => (
            <div
              key={i}
              className={d.used ? styles.skySatUsed : styles.skySatSeen}
              style={{ left: `${d.x}px`, top: `${d.y}px`, position: 'absolute' }}
            />
          ))}
          {numSat === 0 && (
            <Satellite
              size={20}
              strokeWidth={1.2}
              style={{
                position: 'absolute',
                top: '50%',
                left: '50%',
                transform: 'translate(-50%, -50%)',
                color: 'rgba(255,255,255,0.2)',
              }}
            />
          )}
        </div>
      </div>

      <div className={styles.stats}>
        <div className={styles.stat}>
          <span className={styles.statLabel}>卫星</span>
          <span className={styles.statValue}>
            {numSatUsed}<span style={{ opacity: 0.4 }}>/{numSat}</span>
          </span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>HDOP</span>
          <span className={hdopCls}>{fmt(hdop, 2)}</span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>水平精度</span>
          <span className={hStdCls}>{fmt(hStd, 3, ' m')}</span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>延迟</span>
          <span className={styles.statValueDim}>{fmt(gnssEvent.age_s, 1, ' s')}</span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>E</span>
          <span className={styles.statValue}>{fmt(east, 2, ' m')}</span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>N</span>
          <span className={styles.statValue}>{fmt(north, 2, ' m')}</span>
        </div>
      </div>
    </div>
  )
}
