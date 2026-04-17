/**
 * LocalizationCard — 合并 SLAM / GNSS / 融合 的极简信息卡.
 *
 * 原则:
 *   - 沿用 design tokens(var(--glass), var(--border), var(--accent))
 *   - 不做装饰性动画(没有旋转球、卫星天空图)— 信息密度优先
 *   - 一个 accent 只用在:活动指示点 + 链接高亮,其余全部 neutral
 */
import { useState } from 'react'
import type { SSEState } from '../types'
import styles from './LocalizationCard.module.css'

type TabKey = 'slam' | 'gnss' | 'fusion'

interface LocalizationCardProps {
  sseState: SSEState
}

export function LocalizationCard({ sseState }: LocalizationCardProps) {
  const [tab, setTab] = useState<TabKey>('slam')

  return (
    <div className={styles.card}>
      <div className={styles.tabs} role="tablist">
        {(['slam', 'gnss', 'fusion'] as TabKey[]).map((t) => (
          <button
            key={t}
            role="tab"
            aria-selected={tab === t}
            className={tab === t ? styles.tabActive : styles.tab}
            onClick={() => setTab(t)}
          >
            {labelFor(t)}
          </button>
        ))}
      </div>

      <div className={styles.body}>
        {tab === 'slam' && <SlamPanel sseState={sseState} />}
        {tab === 'gnss' && <GnssPanel sseState={sseState} />}
        {tab === 'fusion' && <FusionPanel sseState={sseState} />}
      </div>
    </div>
  )
}

function labelFor(t: TabKey): string {
  switch (t) {
    case 'slam':   return 'SLAM'
    case 'gnss':   return 'GNSS'
    case 'fusion': return '融合'
  }
}

/* ─── SLAM panel ───────────────────────────────────────────────── */

function SlamPanel({ sseState }: { sseState: SSEState }) {
  const odom = sseState.odometry
  const slam = sseState.slamStatus
  const connected = sseState.connected

  const posX = fmt(odom?.x, 2)
  const posY = fmt(odom?.y, 2)
  const yawDeg =
    typeof odom?.yaw === 'number' ? ((odom.yaw * 180) / Math.PI).toFixed(1) : '—'
  const vx = fmt(odom?.vx, 2)
  const mode = slam?.mode ?? '离线'
  const hasFix = connected && odom != null

  return (
    <>
      <Header title="SLAM 里程计" live={hasFix} status={hasFix ? '已锁定' : '未锁定'} />

      <div className={styles.grid}>
        <Field label="X · Y" value={`${posX}, ${posY}`} unit="m" />
        <Field label="航向" value={yawDeg} unit="°" />
        <Field label="线速度" value={vx} unit="m/s" />
        <Field label="模式" value={mode} dim />
      </div>
    </>
  )
}

/* ─── GNSS panel ───────────────────────────────────────────────── */

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
  horizontal_std_m?: number
  link_ok?: boolean
  east?: number
  north?: number
}

function fixLabel(fix: FixType | undefined): string {
  switch (fix) {
    case 'RTK_FIXED':  return 'RTK FIX'
    case 'RTK_FLOAT':  return 'RTK FLOAT'
    case 'DGPS':       return 'DGPS'
    case 'SINGLE':     return 'SINGLE'
    case 'ESTIMATED':  return 'DR'
    case 'SIMULATION': return 'SIM'
    default:           return '无定位'
  }
}

function GnssPanel({ sseState }: { sseState: SSEState }) {
  const events = sseState.events as unknown as Array<{ type?: string } & GnssStatusData>
  const evt = events.slice().reverse().find((e) => e?.type === 'gnss_status')
  const hasData = evt != null && !!evt.link_ok

  if (!hasData) {
    return (
      <>
        <Header title="GNSS 卫星" live={false} status="离线" />
        <Empty
          text="接收机未连接"
          hint={<>启用 <code>gnss.enabled</code> in robot_config.yaml</>}
        />
      </>
    )
  }

  const fix = evt.fix_type ?? 'NO_FIX'
  const isLive = fix === 'RTK_FIXED' || fix === 'RTK_FLOAT'

  return (
    <>
      <Header title="GNSS 卫星" live={isLive} status={fixLabel(fix)} />

      <div className={styles.grid}>
        <Field label="卫星" value={`${evt.num_sat_used ?? 0}/${evt.num_sat ?? 0}`} />
        <Field label="HDOP" value={fmt(evt.hdop, 2)} />
        <Field label="水平精度" value={fmt(evt.horizontal_std_m, 3)} unit="m" />
        <Field label="差分龄" value={fmt(evt.rtcm_age_s, 1)} unit="s" />
        <Field label="E" value={fmt(evt.east, 2)} unit="m" />
        <Field label="N" value={fmt(evt.north, 2)} unit="m" />
      </div>
    </>
  )
}

/* ─── Fusion panel ─────────────────────────────────────────────── */

interface FusionStatusData {
  aligned?: boolean
  fused_count?: number
  residual_m?: number
  alpha?: number
  rtk_available?: boolean
  last_fuse_age_s?: number
}

function FusionPanel({ sseState }: { sseState: SSEState }) {
  const events = sseState.events as unknown as Array<{ type?: string; data?: FusionStatusData }>
  const evt = events.slice().reverse().find((e) => e?.type === 'gnss_fusion')
  const d = evt?.data
  const hasData = d != null
  const aligned = d?.aligned ?? false

  if (!hasData) {
    return (
      <>
        <Header title="SLAM ↔ GNSS" live={false} status="关闭" />
        <Empty
          text="融合未启用"
          hint={<>设置 <code>slam.gnss_fusion: true</code></>}
        />
      </>
    )
  }

  return (
    <>
      <Header
        title="SLAM ↔ GNSS"
        live={aligned}
        status={aligned ? '已对齐' : '等待对齐'}
      />

      <div className={styles.grid}>
        <Field label="融合次数" value={String(d!.fused_count ?? 0)} />
        <Field label="α 权重" value={fmt(d!.alpha, 2)} />
        <Field label="残差" value={fmt(d!.residual_m, 3)} unit="m" />
        <Field label="上次更新" value={fmt(d!.last_fuse_age_s, 1)} unit="s" dim />
        <Field label="RTK" value={d!.rtk_available ? '可用' : '不可用'} dim />
      </div>
    </>
  )
}

/* ─── Shared atoms ─────────────────────────────────────────────── */

function Header({ title, live, status }: {
  title: string
  live: boolean
  status: string
}) {
  return (
    <div className={styles.header}>
      <span className={styles.title}>
        <span className={live ? styles.dotLive : styles.dot} />
        {title}
      </span>
      <span className={styles.status}>{status}</span>
    </div>
  )
}

function Field({
  label, value, unit, dim,
}: {
  label: string
  value: string
  unit?: string
  dim?: boolean
}) {
  return (
    <div className={styles.field}>
      <span className={styles.fieldLabel}>{label}</span>
      <span className={dim ? styles.fieldValueDim : styles.fieldValue}>
        {value}
        {unit && <span className={styles.fieldUnit}> {unit}</span>}
      </span>
    </div>
  )
}

function Empty({ text, hint }: { text: string; hint?: React.ReactNode }) {
  return (
    <div className={styles.empty}>
      <div className={styles.emptyText}>{text}</div>
      {hint && <div className={styles.emptyHint}>{hint}</div>}
    </div>
  )
}

function fmt(n: number | undefined, digits = 2): string {
  return typeof n === 'number' && Number.isFinite(n) ? n.toFixed(digits) : '—'
}
