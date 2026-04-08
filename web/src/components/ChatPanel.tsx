import { useState, useRef, useEffect } from 'react'
import { Send } from 'lucide-react'
import type { SSEState } from '../types'
import * as api from '../services/api'
import { isSlashCommand, executeSlashCommand } from '../utils/slashCommands'
import styles from './ChatPanel.module.css'

interface Message {
  id: number
  role: 'user' | 'system'
  text: string
  ts: number
}

let _id = 0
function nextId() { return ++_id }

function formatTime(ts: number) {
  return new Date(ts).toLocaleTimeString('zh-CN', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' })
}

const NAV_STATE_ZH: Record<string, string> = {
  PLANNING:  '规划路线中',
  EXECUTING: '导航中',
  ARRIVED:   '已到达目的地。',
  IDLE:      '导航空闲。',
  FAILED:    '导航失败。',
  CANCELLED: '导航已取消。',
}

interface ChatPanelProps {
  sseState: SSEState
}

export function ChatPanel({ sseState }: ChatPanelProps) {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: nextId(),
      role: 'system',
      text: '灵途导航系统已上线。发送自然语言指令以开始导航，或使用 /help 查看指令列表。',
      ts: Date.now(),
    },
  ])
  const [input, setInput] = useState('')
  const [sending, setSending] = useState(false)
  const listRef = useRef<HTMLDivElement>(null)
  const prevMissionRef = useRef<string | null>(null)

  // Scroll to bottom on new messages
  useEffect(() => {
    if (listRef.current) {
      listRef.current.scrollTop = listRef.current.scrollHeight
    }
  }, [messages])

  // React to mission_status changes from SSE
  useEffect(() => {
    const ms = sseState.missionStatus
    if (!ms) return
    const key = `${ms.state}:${ms.goal ?? ''}:${ms.progress}`
    if (key === prevMissionRef.current) return
    prevMissionRef.current = key

    const label = NAV_STATE_ZH[ms.state] ?? ms.state
    const text = ms.goal
      ? `[${ms.state}] ${label} 目标：${ms.goal} (${Math.round(ms.progress * 100)}%)`
      : `[${ms.state}] ${label}`

    setMessages(prev => [...prev, { id: nextId(), role: 'system', text, ts: Date.now() }])
  }, [sseState.missionStatus])

  function addSystem(text: string) {
    setMessages(prev => [...prev, { id: nextId(), role: 'system', text, ts: Date.now() }])
  }

  async function sendInstruction() {
    const text = input.trim()
    if (!text || sending) return
    setInput('')
    setMessages(prev => [...prev, { id: nextId(), role: 'user', text, ts: Date.now() }])

    // Slash command path — delegated to utils/slashCommands
    if (isSlashCommand(text)) {
      const response = await executeSlashCommand(text, sseState)
      addSystem(response)
      return
    }

    // Natural language instruction path
    setSending(true)
    try {
      const res = await api.sendInstruction(text)
      if (!res.ok) {
        const err = await res.text()
        addSystem(`错误：${res.status} ${err}`)
      }
      // Successful acknowledgment — mission_status SSE events will follow
    } catch (e) {
      addSystem(`网络错误：${String(e)}`)
    } finally {
      setSending(false)
    }
  }

  function handleKeyDown(e: React.KeyboardEvent<HTMLInputElement>) {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault()
      sendInstruction()
    }
  }

  return (
    <div className={styles.chatPanel}>
      <div className={styles.header}>
        <span className={styles.title}>智能体</span>
        <span
          className={sseState.connected ? styles.sseDotOn : styles.sseDotOff}
          title={sseState.connected ? 'SSE 已连接' : 'SSE 已断开'}
        />
      </div>

      <div className={styles.messages} ref={listRef}>
        {messages.map(msg => (
          <div key={msg.id} className={msg.role === 'user' ? styles.bubbleUser : styles.bubbleSystem}>
            <div className={styles.bubbleText}>{msg.text}</div>
            <div className={styles.bubbleTime}>{formatTime(msg.ts)}</div>
          </div>
        ))}
      </div>

      <div className={styles.inputRow}>
        <input
          className={styles.input}
          type="text"
          placeholder="输入指令…"
          value={input}
          onChange={e => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          disabled={sending}
          aria-label="导航指令"
        />
        <button
          className={styles.btnSend}
          onClick={sendInstruction}
          disabled={sending || !input.trim()}
          aria-label="发送"
        >
          <Send size={16} />
        </button>
      </div>
    </div>
  )
}
