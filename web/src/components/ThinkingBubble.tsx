import { useEffect, useState, useRef } from 'react'
import styles from './ThinkingBubble.module.css'

// Braille dots spinner — classic Claude Code / ora library style
const SPINNER = ['⠋', '⠙', '⠹', '⠸', '⠼', '⠴', '⠦', '⠧', '⠇', '⠏']

// Rotating verbs that change every ~2.5s for a "mind at work" feel.
// Mix of Chinese + occasional English, biased toward navigation/planning verbs.
const VERBS = [
  '思考',
  '推理',
  '规划',
  '搜索目标',
  '解析指令',
  '查询记忆',
  '定位坐标',
  '评估路径',
  '分析场景',
  '构建计划',
]

interface Props {
  hint?: string          // optional override text (e.g. current phase)
  startedAt?: number     // ms timestamp to show elapsed seconds
}

export function ThinkingBubble({ hint, startedAt }: Props) {
  const [frame, setFrame] = useState(0)
  const [verbIdx, setVerbIdx] = useState(() => Math.floor(Math.random() * VERBS.length))
  const [elapsed, setElapsed] = useState(0)
  const startRef = useRef(startedAt ?? Date.now())

  // Fast spinner (80ms) — matches Claude Code cadence
  useEffect(() => {
    const t = setInterval(() => setFrame(f => (f + 1) % SPINNER.length), 80)
    return () => clearInterval(t)
  }, [])

  // Slow verb rotation (2.5s)
  useEffect(() => {
    if (hint) return  // caller-provided hint overrides
    const t = setInterval(() => setVerbIdx(i => (i + 1) % VERBS.length), 2500)
    return () => clearInterval(t)
  }, [hint])

  // Elapsed counter (1s tick)
  useEffect(() => {
    const t = setInterval(() => setElapsed(Math.floor((Date.now() - startRef.current) / 1000)), 1000)
    return () => clearInterval(t)
  }, [])

  const label = hint || VERBS[verbIdx]

  return (
    <div className={styles.thinking} aria-live="polite">
      <span className={styles.spinner}>{SPINNER[frame]}</span>
      <span className={styles.label}>{label}…</span>
      {elapsed > 0 && <span className={styles.elapsed}>{elapsed}s</span>}
    </div>
  )
}
