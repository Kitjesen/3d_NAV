import { useState } from 'react'
import { Settings, Radio } from 'lucide-react'
import type { SSEState, Tab } from '../types'
import { SettingsMenu } from './SettingsMenu'
import styles from './Topbar.module.css'

const SLAM_MODE_ZH: Record<string, string> = {
  fastlio2:  '建图',
  localizer: '导航',
  stop:      '停止',
}

const NAV_STATE_ZH: Record<string, string> = {
  IDLE: '空闲',
  EXECUTING: '执行中',
  PLANNING: '规划中',
  ARRIVED: '已到达',
  FAILED: '失败',
  CANCELLED: '已取消',
}

const NAV_STATE_CLASS: Record<string, string> = {
  executing: styles.navExecuting,
  failed:    styles.navFailed,
  planning:  styles.navPlanning,
}

const SLAM_MODE_CLASS: Record<string, string> = {
  fastlio2:  styles.slamFastlio2,
  localizer: styles.slamLocalizer,
  stop:      styles.slamStop,
}

const TABS: { key: Tab; label: string }[] = [
  { key: 'console', label: '控制台' },
  { key: 'scene',   label: '场景' },
  { key: 'map',     label: '地图' },
  { key: 'slam',    label: 'SLAM' },
]

interface TopbarProps {
  sseState:    SSEState
  activeTab:   Tab
  onTabChange: (tab: Tab) => void
}

export function Topbar({ sseState, activeTab, onTabChange }: TopbarProps) {
  const [settingsOpen, setSettingsOpen] = useState(false)
  const estop = sseState.safetyState?.estop ?? false
  const odom = sseState.odometry
  const posLabel =
    typeof odom?.x === 'number' && typeof odom?.y === 'number'
      ? `(${odom.x.toFixed(1)}, ${odom.y.toFixed(1)})`
      : '--'
  const navState = sseState.missionStatus?.state ?? 'IDLE'
  const navStateZh = NAV_STATE_ZH[navState] ?? navState
  const slamMode = sseState.slamStatus?.mode ?? null
  const slamModeZh = slamMode ? (SLAM_MODE_ZH[slamMode] ?? slamMode) : null

  const handleKeyDown = (e: React.KeyboardEvent, idx: number) => {
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return
    e.preventDefault()
    const delta = e.key === 'ArrowRight' ? 1 : -1
    const next = (idx + delta + TABS.length) % TABS.length
    onTabChange(TABS[next].key)
  }

  return (
    <header className={styles.topbar}>
      <div className={styles.left}>
        {/* Logo */}
        <span className={styles.logo}>
          <span className={styles.logoIcon} aria-hidden="true">
            <svg width="15" height="15" viewBox="0 0 32 32" fill="none" xmlns="http://www.w3.org/2000/svg">
              <circle cx="16" cy="16" r="11.5" stroke="url(#tbRing)" strokeWidth="1.5" fill="none" strokeDasharray="7 3.5" strokeLinecap="round"/>
              <path d="M16 5.5 L23.5 21 L16 17.8 L8.5 21 Z" fill="url(#tbArrow)"/>
              <path d="M16 10.5 L20.5 19.5 L16 17.8 L11.5 19.5 Z" fill="rgba(14,16,32,0.85)"/>
              <circle cx="16" cy="25.5" r="1.8" fill="#22d3ee" opacity="0.9"/>
              <circle cx="16" cy="25.5" r="1" fill="rgba(14,16,32,0.85)"/>
              <defs>
                <linearGradient id="tbArrow" x1="8" y1="5" x2="24" y2="24" gradientUnits="userSpaceOnUse">
                  <stop offset="0" stopColor="#818cf8"/>
                  <stop offset="1" stopColor="#22d3ee"/>
                </linearGradient>
                <linearGradient id="tbRing" x1="5" y1="5" x2="27" y2="27" gradientUnits="userSpaceOnUse">
                  <stop offset="0" stopColor="#6366f1" stopOpacity="0.8"/>
                  <stop offset="1" stopColor="#06b6d4" stopOpacity="0.8"/>
                </linearGradient>
              </defs>
            </svg>
          </span>
          LingTu
        </span>
        <span className={sseState.connected ? styles.badgeOnline : styles.badgeOffline}>
          {sseState.connected ? '在线' : '离线'}
        </span>

        <span className={styles.sep} aria-hidden="true" />

        {/* Tabs inline */}
        <nav className={styles.tabs} role="tablist" aria-label="主视图切换">
          {TABS.map((tab, i) => (
            <button
              key={tab.key}
              role="tab"
              tabIndex={activeTab === tab.key ? 0 : -1}
              aria-selected={activeTab === tab.key}
              aria-controls={`panel-${tab.key}`}
              className={activeTab === tab.key ? styles.tabActive : styles.tab}
              onClick={() => onTabChange(tab.key)}
              onKeyDown={(e) => handleKeyDown(e, i)}
            >
              {tab.label}
            </button>
          ))}
        </nav>
      </div>

      <div className={styles.center}>
        <span className={styles.stat}>
          <span className={styles.statLabel}>位置</span>
          <span className={styles.statValue}>{posLabel}</span>
        </span>
        <span className={styles.divider} />
        <span className={styles.stat}>
          <span className={styles.statLabel}>导航</span>
          <span className={`${styles.statValue} ${NAV_STATE_CLASS[navState.toLowerCase()] ?? ''}`}>{navStateZh}</span>
        </span>
        {slamModeZh && (
          <>
            <span className={styles.divider} />
            <span className={styles.stat}>
              <span className={styles.statLabel}><Radio size={10} style={{ display: 'inline', verticalAlign: 'middle', marginRight: 2 }} />SLAM</span>
              <span className={`${styles.statValue} ${SLAM_MODE_CLASS[slamMode!] ?? ''}`}>{slamModeZh}</span>
            </span>
          </>
        )}
        {estop && (
          <>
            <span className={styles.divider} />
            <span className={styles.estop}>急停</span>
          </>
        )}
      </div>

      <div className={styles.right}>
        <button
          className={styles.btnIcon}
          aria-label="设置"
          aria-expanded={settingsOpen}
          onClick={() => setSettingsOpen(v => !v)}
        >
          <Settings size={16} />
        </button>
      </div>

      <SettingsMenu open={settingsOpen} onClose={() => setSettingsOpen(false)} />
    </header>
  )
}
