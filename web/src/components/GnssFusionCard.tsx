import { Anchor, Satellite } from 'lucide-react'
import type { SSEState } from '../types'
import styles from './GnssCard.module.css'

/** GNSS-SLAM fusion diagnostic — complements GnssCard (which shows raw fix).
 *  This card surfaces the alignment / residual / relock state that
 *  SlamBridgeModule.gnss_fusion_health publishes. */

interface GnssFusionData {
  enabled?: boolean
  alignment_locked?: boolean
  map_offset?: [number, number] | null
  antenna_offset_body?: [number, number, number]
  last_residual_m?: number
  fused_count?: number
  relock_count?: number
  last_relock_ts?: number
  last_fix_type?: string
  last_gnss_age_s?: number
}

interface GnssFusionCardProps {
  sseState: SSEState
}

function formatAge(ageS: number | undefined): string {
  if (ageS == null || !isFinite(ageS)) return '∞'
  if (ageS < 60) return `${ageS.toFixed(1)}s`
  return `${(ageS / 60).toFixed(1)}m`
}

function residualBadge(residual: number | undefined, warn = 1.0, crit = 5.0) {
  if (residual == null) return styles.fixNone
  if (residual >= crit) return styles.fixNone
  if (residual >= warn) return styles.fixRtkFloat
  return styles.fixRtkFixed
}

export function GnssFusionCard({ sseState }: GnssFusionCardProps) {
  // Scan SSE events for latest gnss_fusion — pattern mirrors GnssCard.
  const evt = (
    sseState.events as unknown as Array<{ type?: string; data?: GnssFusionData }>
  )
    .slice()
    .reverse()
    .find((e) => e?.type === 'gnss_fusion')

  const data = evt?.data

  // Offline state — no event received (GNSS module disabled or never wired)
  if (data == null) {
    return (
      <div className={styles.card}>
        <div className={styles.header}>
          <span className={styles.title}>
            <span className={styles.titleDot} />
            GNSS 融合
          </span>
          <span className={styles.fixNone}>未接入</span>
        </div>
        <div className={styles.offline}>
          <Anchor size={32} strokeWidth={1.2} />
          <div>GNSS 融合未激活</div>
          <div className={styles.offlineHint}>
            确认 <code>gnss.fusion.enabled: true</code> 且
            <code>GnssModule</code> 已加入蓝图
          </div>
        </div>
      </div>
    )
  }

  const locked = data.alignment_locked === true
  const enabled = data.enabled === true
  const offset = data.map_offset
  const residual = data.last_residual_m ?? 0
  const resCls = residualBadge(residual)

  return (
    <div className={styles.card}>
      <div className={styles.header}>
        <span className={styles.title}>
          <span className={styles.titleDot} />
          GNSS 融合
        </span>
        {enabled ? (
          locked ? (
            <span className={styles.fixRtkFixed}>已锚定</span>
          ) : (
            <span className={styles.fixRtkFloat}>待锚定</span>
          )
        ) : (
          <span className={styles.fixNone}>停用</span>
        )}
      </div>

      <div style={{ display: 'grid', gap: '8px', fontSize: '13px' }}>
        {/* Row: fix type + age */}
        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
          <span style={{ opacity: 0.75 }}>
            <Satellite size={12} style={{ marginRight: 4 }} />
            最近定位
          </span>
          <span>
            {data.last_fix_type ?? 'NONE'}
            <span style={{ opacity: 0.6, marginLeft: 6 }}>
              {formatAge(data.last_gnss_age_s)}
            </span>
          </span>
        </div>

        {/* Row: residual */}
        <div style={{ display: 'flex', justifyContent: 'space-between' }}>
          <span style={{ opacity: 0.75 }}>对齐残差</span>
          <span className={resCls} style={{ padding: '0 6px', borderRadius: 4 }}>
            {residual.toFixed(2)} m
          </span>
        </div>

        {/* Row: map offset (if locked) */}
        {locked && offset && (
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span style={{ opacity: 0.75 }}>Map↔ENU 偏移</span>
            <span style={{ fontFamily: 'ui-monospace, monospace' }}>
              ({offset[0].toFixed(2)}, {offset[1].toFixed(2)})
            </span>
          </div>
        )}

        {/* Row: antenna offset */}
        {data.antenna_offset_body && (
          <div style={{ display: 'flex', justifyContent: 'space-between' }}>
            <span style={{ opacity: 0.75 }}>天线位置 (body)</span>
            <span style={{ fontFamily: 'ui-monospace, monospace', opacity: 0.85 }}>
              ({data.antenna_offset_body[0].toFixed(2)},
              {' '}{data.antenna_offset_body[1].toFixed(2)},
              {' '}{data.antenna_offset_body[2].toFixed(2)})
            </span>
          </div>
        )}

        {/* Row: counters */}
        <div style={{ display: 'flex', justifyContent: 'space-between', opacity: 0.75 }}>
          <span>融合帧数</span>
          <span>{data.fused_count ?? 0}</span>
        </div>
        <div style={{ display: 'flex', justifyContent: 'space-between', opacity: 0.75 }}>
          <span>重锁次数</span>
          <span className={data.relock_count && data.relock_count > 0 ? styles.fixRtkFloat : undefined}>
            {data.relock_count ?? 0}
          </span>
        </div>
      </div>
    </div>
  )
}
