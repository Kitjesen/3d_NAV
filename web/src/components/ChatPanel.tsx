import { useState, useRef, useEffect } from 'react'
import { Send } from 'lucide-react'
import type { SSEState } from '../hooks/useSSE'

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

const HELP_TEXT = `可用指令：
/go <x> <y>   — 导航到坐标
/stop         — 紧急停止
/status       — 显示当前状态
/map list     — 地图管理
/help         — 显示帮助`

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

  // --- Slash command handler ---
  async function handleSlashCommand(raw: string) {
    const parts = raw.trim().split(/\s+/)
    const cmd = parts[0].toLowerCase()

    if (cmd === '/help') {
      addSystem(HELP_TEXT)
      return
    }

    if (cmd === '/status') {
      const ms = sseState.missionStatus
      const odom = sseState.odometry
      const safety = sseState.safetyState
      const stateZh = ms ? (NAV_STATE_ZH[ms.state] ?? ms.state) : '未知'
      const pos = odom ? `(${odom.x.toFixed(2)}, ${odom.y.toFixed(2)})` : '--'
      const estop = safety?.estop ? '急停激活' : '正常'
      addSystem(`状态：${stateZh}\n位置：${pos}\n安全：${estop}`)
      return
    }

    if (cmd === '/map') {
      addSystem('地图管理请切换到地图标签。')
      return
    }

    if (cmd === '/stop') {
      addSystem('正在发送急停指令…')
      try {
        const res = await fetch('/api/v1/stop', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: '{}',
        })
        addSystem(res.ok ? '急停指令已发送。' : `急停失败：HTTP ${res.status}`)
      } catch (e) {
        addSystem(`急停网络错误：${String(e)}`)
      }
      return
    }

    if (cmd === '/go') {
      const x = parseFloat(parts[1])
      const y = parseFloat(parts[2])
      if (isNaN(x) || isNaN(y)) {
        addSystem('用法：/go <x> <y>  （示例：/go 3.5 -1.2）')
        return
      }
      addSystem(`正在导航到坐标 (${x}, ${y})…`)
      try {
        const res = await fetch('/api/v1/goal', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ x, y }),
        })
        addSystem(res.ok ? `目标已提交：(${x}, ${y})` : `导航失败：HTTP ${res.status}`)
      } catch (e) {
        addSystem(`导航网络错误：${String(e)}`)
      }
      return
    }

    addSystem(`未知指令：${cmd}。输入 /help 查看可用指令。`)
  }

  async function sendInstruction() {
    const text = input.trim()
    if (!text || sending) return
    setInput('')
    setMessages(prev => [...prev, { id: nextId(), role: 'user', text, ts: Date.now() }])

    // Slash command path — no setSending needed (fast local dispatch)
    if (text.startsWith('/')) {
      await handleSlashCommand(text)
      return
    }

    // Natural language instruction path
    setSending(true)
    try {
      const res = await fetch('/api/v1/instruction', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text }),
      })
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
    <div className="chat-panel">
      <div className="chat-header">
        <span className="chat-title">智能体</span>
        <span
          className={`sse-dot ${sseState.connected ? 'sse-dot--on' : 'sse-dot--off'}`}
          title={sseState.connected ? 'SSE 已连接' : 'SSE 已断开'}
        />
      </div>

      <div className="chat-messages" ref={listRef}>
        {messages.map(msg => (
          <div key={msg.id} className={`chat-bubble chat-bubble--${msg.role}`}>
            <div className="bubble-text">{msg.text}</div>
            <div className="bubble-time">{formatTime(msg.ts)}</div>
          </div>
        ))}
      </div>

      <div className="chat-input-row">
        <input
          className="chat-input"
          type="text"
          placeholder="输入指令…"
          value={input}
          onChange={e => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          disabled={sending}
          aria-label="导航指令"
        />
        <button
          className="btn-send"
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
