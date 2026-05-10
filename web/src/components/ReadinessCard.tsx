import { useEffect, useState } from 'react'
import type { ReadinessResponse, SSEState } from '../types'
import * as api from '../services/api'
import styles from './ReadinessCard.module.css'

interface ReadinessCardProps {
  sseState: SSEState
}

function fmtNum(v: unknown, digits = 2): string {
  return typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : '--'
}

function labelList(values: string[], fallback: string): string[] {
  const clean = values.map(v => String(v).trim()).filter(Boolean)
  return clean.length > 0 ? clean.slice(0, 5) : [fallback]
}

function stringList(value: unknown): string[] {
  return Array.isArray(value)
    ? value.map(item => String(item).trim()).filter(Boolean)
    : []
}

function calibrationWarnings(readiness: ReadinessResponse | null): string[] {
  const calibration = readiness?.runtime?.calibration
  if (calibration == null || typeof calibration !== 'object') return []
  return stringList((calibration as Record<string, unknown>).warnings)
}

export function ReadinessCard({ sseState }: ReadinessCardProps) {
  const [clientReadiness, setClientReadiness] = useState<ReadinessResponse | null>(null)
  const nav = sseState.navigationStatus
  const readiness = nav?.readiness
  const localization = nav?.localization
  const control = nav?.control
  const motion = nav?.motion
  const progress = nav?.progress
  const target = nav?.target

  const goalReady = readiness?.can_accept_goal ?? nav?.can_accept_goal ?? false
  const canExecute = readiness?.can_execute_autonomy ?? false
  const clientReasons = clientReadiness?.reasons ?? []
  const clientAdvisories = [
    ...(clientReadiness?.advisories ?? []),
    ...calibrationWarnings(clientReadiness),
  ]
  const blockers = labelList([...(readiness?.blockers ?? []), ...clientReasons], 'No readiness blockers')
  const advisories = labelList([...(readiness?.advisories ?? []), ...clientAdvisories], 'No advisories')
  const owner = control?.active_source?.label ?? control?.active_cmd_source ?? 'none'
  const speedMode = motion?.speed_policy?.mode ?? 'unknown'
  const distanceToGoal = fmtNum(target?.distance_to_goal_m)
  const progressText = progress
    ? `${progress.wp_index}/${progress.wp_total} (${Math.round(progress.fraction * 100)}%)`
    : '--'

  useEffect(() => {
    let cancelled = false
    const load = async () => {
      try {
        const next = await api.fetchReadiness()
        if (!cancelled) setClientReadiness(next)
      } catch {
        if (!cancelled) setClientReadiness(null)
      }
    }
    void load()
    const timer = window.setInterval(load, 5000)
    return () => {
      cancelled = true
      window.clearInterval(timer)
    }
  }, [])

  return (
    <div className={styles.card}>
      <div className={styles.header}>
        <div>
          <div className={styles.eyebrow}>Navigation Readiness</div>
          <div className={styles.title}>{nav ? nav.state : 'NO DATA'}</div>
        </div>
        <span className={goalReady ? styles.readyPill : styles.blockedPill}>
          {nav ? (goalReady ? 'GOAL READY' : 'GOAL BLOCKED') : 'UNKNOWN'}
        </span>
      </div>

      <div className={styles.grid}>
        <Metric label="Autonomy" value={canExecute ? 'READY' : 'HELD'} tone={canExecute ? 'ok' : 'warn'} />
        <Metric label="Owner" value={owner} tone={owner === 'none' ? 'dim' : 'warn'} />
        <Metric label="Localization" value={localization?.state ?? 'unknown'} tone={localization?.ready ? 'ok' : 'warn'} />
        <Metric label="Pose" value={localization?.pose_freshness ?? 'unknown'} tone={localization?.pose_fresh ? 'ok' : 'warn'} />
        <Metric label="Progress" value={progressText} />
        <Metric label="Goal Dist." value={`${distanceToGoal} m`} />
        <Metric label="Speed Mode" value={speedMode} tone={speedMode === 'normal' ? 'dim' : 'warn'} />
        <Metric label="Speed Scale" value={fmtNum(motion?.speed_scale ?? nav?.speed_scale, 2)} />
      </div>

      <section className={styles.section}>
        <div className={styles.sectionTitle}>Blockers</div>
        <ul className={styles.list}>
          {blockers.map(item => (
            <li key={item} className={goalReady ? styles.mutedItem : styles.alertItem}>{item}</li>
          ))}
        </ul>
      </section>

      <section className={styles.section}>
        <div className={styles.sectionTitle}>Advisories</div>
        <ul className={styles.list}>
          {advisories.map(item => (
            <li key={item} className={styles.mutedItem}>{item}</li>
          ))}
        </ul>
      </section>
    </div>
  )
}

function Metric({
  label,
  value,
  tone = 'dim',
}: {
  label: string
  value: string
  tone?: 'ok' | 'warn' | 'dim'
}) {
  const valueClass =
    tone === 'ok' ? styles.metricOk :
    tone === 'warn' ? styles.metricWarn :
    styles.metricValue
  return (
    <div className={styles.metric}>
      <span className={styles.metricLabel}>{label}</span>
      <span className={valueClass}>{value}</span>
    </div>
  )
}
