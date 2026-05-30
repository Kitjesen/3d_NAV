import { useCallback, useEffect, useMemo, useState } from 'react'
import { AlertTriangle, MapPin, RotateCcw, ShieldCheck } from 'lucide-react'
import type {
  InspectionAcceptanceMode,
  InspectionAcceptanceResponse,
  LocationsResponse,
  SSEState,
} from '../types'
import * as api from '../services/api'
import styles from './InspectionAcceptanceView.module.css'

interface InspectionAcceptanceViewProps {
  sseState: SSEState
}

type Tone = 'pass' | 'fail' | 'blocked' | 'unknown'

const MODES: Array<{ key: InspectionAcceptanceMode; label: string }> = [
  { key: 'simulation', label: 'Simulation' },
  { key: 'field', label: 'Field' },
  { key: 'non_motion', label: 'No Motion' },
]

function asRecord(value: unknown): Record<string, unknown> {
  return typeof value === 'object' && value !== null ? value as Record<string, unknown> : {}
}

function text(value: unknown, fallback = '--'): string {
  if (typeof value === 'boolean') return value ? 'true' : 'false'
  if (typeof value === 'number' && Number.isFinite(value)) return String(value)
  return typeof value === 'string' && value.length > 0 ? value : fallback
}

function toneFor(status?: string | null): Tone {
  if (status === 'PASS') return 'pass'
  if (status === 'FAIL') return 'fail'
  if (status === 'BLOCKED') return 'blocked'
  return 'unknown'
}

function targetSummary(report: InspectionAcceptanceResponse | null): string {
  if (!report) return '--'
  return `${report.pass_count}/${report.target_count} pass`
}

function modeLabel(mode: InspectionAcceptanceMode): string {
  return MODES.find(item => item.key === mode)?.label ?? mode
}

export function InspectionAcceptanceView({ sseState }: InspectionAcceptanceViewProps) {
  const [mode, setMode] = useState<InspectionAcceptanceMode>('simulation')
  const [selectedLocationNames, setSelectedLocationNames] = useState<string[]>([])
  const [selectedTag, setSelectedTag] = useState('')
  const [report, setReport] = useState<InspectionAcceptanceResponse | null>(null)
  const [locations, setLocations] = useState<LocationsResponse | null>(null)
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [locationError, setLocationError] = useState<string | null>(null)

  const loadLocations = useCallback(async () => {
    try {
      setLocations(await api.fetchLocations())
      setLocationError(null)
    } catch (err) {
      setLocationError(err instanceof Error ? err.message : String(err))
    }
  }, [])

  const runCheck = useCallback(async () => {
    setLoading(true)
    try {
      const [acceptanceResult, locationsResult] = await Promise.allSettled([
        api.runInspectionAcceptance({
          mode,
          points: selectedLocationNames.length > 0 ? selectedLocationNames : undefined,
          tag: selectedLocationNames.length === 0 ? selectedTag || undefined : undefined,
        }),
        api.fetchLocations(),
      ])
      if (acceptanceResult.status === 'rejected') throw acceptanceResult.reason
      setReport(acceptanceResult.value)
      setError(null)
      if (locationsResult.status === 'fulfilled') {
        setLocations(locationsResult.value)
        setLocationError(null)
      } else {
        setLocationError(
          locationsResult.reason instanceof Error
            ? locationsResult.reason.message
            : String(locationsResult.reason),
        )
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setLoading(false)
    }
  }, [mode, selectedLocationNames, selectedTag])

  const toggleLocation = useCallback((name: string) => {
    setSelectedLocationNames(current =>
      current.includes(name)
        ? current.filter(item => item !== name)
        : [...current, name],
    )
  }, [])

  useEffect(() => {
    void loadLocations()
  }, [loadLocations])

  const availableTags = useMemo(() => {
    const names = new Set<string>()
    for (const location of locations?.locations ?? []) {
      for (const tag of location.tags) {
        if (tag.length > 0) names.add(tag)
      }
    }
    return Array.from(names).sort((left, right) => left.localeCompare(right))
  }, [locations])

  const visibleLocations = useMemo(() => {
    const allLocations = locations?.locations ?? []
    if (!selectedTag) return allLocations
    return allLocations.filter(location => location.tags.includes(selectedTag))
  }, [locations, selectedTag])

  useEffect(() => {
    if (selectedTag && !availableTags.includes(selectedTag)) {
      setSelectedTag('')
    }
  }, [availableTags, selectedTag])

  const fieldCheck = useMemo(() => asRecord(report?.evidence?.field_check), [report])
  const runtime = useMemo(() => asRecord(fieldCheck.runtime), [fieldCheck])
  const navigation = useMemo(() => asRecord(fieldCheck.navigation), [fieldCheck])
  const motionSafety = useMemo(() => asRecord(report?.motion_safety), [report])
  const frontierPreview = useMemo(() => asRecord(report?.frontier_preview), [report])
  const runtimeSwitch = useMemo(() => asRecord(report?.runtime_switch), [report])
  const algorithm = useMemo(
    () => asRecord(asRecord(fieldCheck.algorithm).strict_benchmark),
    [fieldCheck],
  )
  const blockers = report?.blockers ?? []
  const advisories = report?.advisories ?? []
  const advisoryItems = locationError
    ? [...advisories, `locations: ${locationError}`]
    : advisories
  const summaryTone = toneFor(report?.summary)
  const connected = sseState.connected ? 'CONNECTED' : 'DISCONNECTED'

  return (
    <div className={styles.page} role="tabpanel" id="panel-inspection">
      <header className={styles.header}>
        <div>
          <div className={styles.eyebrow}>Inspection Acceptance</div>
          <h1 className={styles.title}>Gateway + ModulePorts</h1>
        </div>
        <div className={styles.actions}>
          <div className={styles.segmented} aria-label="acceptance mode">
            {MODES.map(item => (
              <button
                key={item.key}
                type="button"
                className={mode === item.key ? styles.segmentActive : styles.segment}
                onClick={() => setMode(item.key)}
              >
                {item.label}
              </button>
            ))}
          </div>
          <button
            type="button"
            className={styles.refreshButton}
            onClick={() => void runCheck()}
            disabled={loading}
            title="Run inspection acceptance"
          >
            <RotateCcw size={15} />
            {loading ? 'Checking' : 'Check'}
          </button>
        </div>
      </header>

      <section className={styles.statusBand}>
        <div>
          <div className={styles.metricLabel}>Summary</div>
          <div className={styles[`summary_${summaryTone}`]}>
            {report?.summary ?? 'UNKNOWN'}
          </div>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Mode</span>
          <strong>{report?.mode ? modeLabel(report.mode as InspectionAcceptanceMode) : modeLabel(mode)}</strong>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Targets</span>
          <strong>{targetSummary(report)}</strong>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>ROS2 Required</span>
          <strong>{text(runtime.ros2_topic_required, 'false')}</strong>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>Motion Safety</span>
          <strong>{text(motionSafety.status)}</strong>
        </div>
        <div className={styles.metric}>
          <span className={styles.metricLabel}>SSE</span>
          <strong>{connected}</strong>
        </div>
      </section>

      {error && (
        <div className={styles.alert}>
          <AlertTriangle size={16} />
          <span>{error}</span>
        </div>
      )}

      <section className={styles.content}>
        <div className={styles.mainPanel}>
          <div className={styles.panelTitleRow}>
            <div>
              <div className={styles.sectionTitle}>Targets</div>
              <div className={styles.caption}>
                {report?.locations_count ?? locations?.count ?? 0} saved locations
                {selectedLocationNames.length > 0
                  ? ` | ${selectedLocationNames.length} selected`
                  : selectedTag
                    ? ` | tag ${selectedTag}`
                    : ' | all selected'}
              </div>
            </div>
            <ShieldCheck size={18} />
          </div>

          {availableTags.length > 0 && (
            <div className={styles.tagPicker} aria-label="saved inspection location tags">
              <button
                type="button"
                className={!selectedTag ? styles.tagSelected : styles.tagOption}
                onClick={() => {
                  setSelectedTag('')
                  setSelectedLocationNames([])
                }}
                title="Show all saved locations"
              >
                All tags
              </button>
              {availableTags.map(tag => (
                <button
                  key={tag}
                  type="button"
                  className={selectedTag === tag ? styles.tagSelected : styles.tagOption}
                  onClick={() => {
                    setSelectedTag(current => (current === tag ? '' : tag))
                    setSelectedLocationNames([])
                  }}
                  title={`Select ${tag} inspection group`}
                >
                  {tag}
                </button>
              ))}
            </div>
          )}

          <div className={styles.locationPicker} aria-label="saved inspection locations">
            {visibleLocations.map(location => {
              const selected = selectedLocationNames.includes(location.name)
              return (
                <button
                  key={location.name}
                  type="button"
                  className={selected ? styles.locationSelected : styles.locationOption}
                  onClick={() => toggleLocation(location.name)}
                  title={`Toggle ${location.name}`}
                >
                  <MapPin size={14} />
                  <span>{location.name}</span>
                </button>
              )
            })}
            {selectedLocationNames.length > 0 && (
              <button
                type="button"
                className={styles.clearSelection}
                onClick={() => setSelectedLocationNames([])}
                title="Clear selected locations"
              >
                All
              </button>
            )}
          </div>

          <div className={styles.targetList}>
            {(report?.targets ?? []).map(target => (
              <div key={`${target.name}-${target.source ?? ''}`} className={styles.targetRow}>
                <div className={styles.targetName}>
                  <MapPin size={16} />
                  <div>
                    <strong>{target.name}</strong>
                    <span>{target.source ?? target.target_type ?? '--'}</span>
                  </div>
                </div>
                <span className={styles[`target_${toneFor(target.status)}`]}>
                  {target.status}
                </span>
                <span>{target.preview_feasible ? 'preview feasible' : 'preview blocked'}</span>
                <span>{target.preview_count ?? 0} pts</span>
                <span>{target.planner ?? '--'}</span>
                <span>{target.command_published ? 'published' : 'no command'}</span>
              </div>
            ))}
            {report && report.targets.length === 0 && (
              <div className={styles.empty}>No inspection targets resolved</div>
            )}
          </div>
        </div>

        <aside className={styles.sidePanel}>
          <div>
            <div className={styles.sectionTitle}>Backend Verdict</div>
            <div className={styles.checkGrid}>
              <div><span>Field</span><strong>{report?.field_summary ?? '--'}</strong></div>
              <div><span>Readiness</span><strong>{text(runtime.readiness)}</strong></div>
              <div><span>Dataflow</span><strong>{text(runtime.dataflow)}</strong></div>
              <div><span>Command</span><strong>{text(runtime.command_boundary)}</strong></div>
              <div><span>Route</span><strong>{text(navigation.route_preview)}</strong></div>
              <div><span>Algorithm</span><strong>{text(algorithm.status)}</strong></div>
              <div><span>Frontier</span><strong>{text(frontierPreview.status)}</strong></div>
              <div><span>Frontier Source</span><strong>{text(frontierPreview.candidate_source)}</strong></div>
              <div><span>Runtime Switch</span><strong>{text(runtimeSwitch.status)}</strong></div>
              <div>
                <span>Switch Motion</span>
                <strong>
                  {runtimeSwitch.motion === true
                    ? 'motion=true'
                    : 'motion=false'}
                </strong>
              </div>
              <div>
                <span>Frontier Command</span>
                <strong>
                  {frontierPreview.command_published === true
                    ? 'command_published=true'
                    : 'command_published=false'}
                </strong>
              </div>
              <div><span>Local Path</span><strong>{text(navigation.local_path)}</strong></div>
            </div>
          </div>

          <div>
            <div className={styles.sectionTitle}>Blockers</div>
            <ul className={styles.issueList}>
              {blockers.slice(0, 8).map(item => <li key={item}>{item}</li>)}
              {blockers.length === 0 && <li>none</li>}
            </ul>
          </div>

          <div>
            <div className={styles.sectionTitle}>Advisories</div>
            <ul className={styles.issueList}>
              {advisoryItems.slice(0, 8).map(item => (
                <li key={item}>{item}</li>
              ))}
              {advisoryItems.length === 0 && <li>none</li>}
            </ul>
          </div>
        </aside>
      </section>
    </div>
  )
}
