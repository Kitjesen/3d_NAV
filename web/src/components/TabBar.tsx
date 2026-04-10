import type { Tab } from '../types'
import styles from './TabBar.module.css'

interface TabBarProps {
  activeTab: Tab
  onTabChange: (tab: Tab) => void
}

const TABS: { key: Tab; label: string }[] = [
  { key: 'console', label: '控制台' },
  { key: 'scene',   label: '场景' },
  { key: 'map',     label: '地图' },
  { key: 'slam',    label: 'SLAM' },
]

export function TabBar({ activeTab, onTabChange }: TabBarProps) {
  return (
    <nav className={styles.tabBar} role="tablist">
      {TABS.map(tab => (
        <button
          key={tab.key}
          role="tab"
          aria-selected={activeTab === tab.key}
          className={activeTab === tab.key ? styles.tabBtnActive : styles.tabBtn}
          onClick={() => onTabChange(tab.key)}
        >
          {tab.label}
        </button>
      ))}
    </nav>
  )
}
