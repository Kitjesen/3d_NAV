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
  return new Date(ts).toLocaleTimeString('en-US', { hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit' })
}

interface ChatPanelProps {
  sseState: SSEState
}

export function ChatPanel({ sseState }: ChatPanelProps) {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: nextId(),
      role: 'system',
      text: 'LingTu navigation system online. Send a natural language instruction to begin.',
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

    const stateLabel: Record<string, string> = {
      PLANNING: 'Planning route…',
      EXECUTING: 'Navigating…',
      ARRIVED: 'Arrived at destination.',
      IDLE: 'Navigation idle.',
      FAILED: 'Navigation failed.',
      CANCELLED: 'Navigation cancelled.',
    }
    const text = ms.goal
      ? `[${ms.state}] ${stateLabel[ms.state] ?? ms.state} Goal: ${ms.goal} (${Math.round(ms.progress * 100)}%)`
      : `[${ms.state}] ${stateLabel[ms.state] ?? ms.state}`

    setMessages(prev => [...prev, { id: nextId(), role: 'system', text, ts: Date.now() }])
  }, [sseState.missionStatus])

  async function sendInstruction() {
    const text = input.trim()
    if (!text || sending) return
    setInput('')
    setMessages(prev => [...prev, { id: nextId(), role: 'user', text, ts: Date.now() }])
    setSending(true)
    try {
      const res = await fetch('/api/v1/instruction', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ text }),
      })
      if (!res.ok) {
        const err = await res.text()
        setMessages(prev => [
          ...prev,
          { id: nextId(), role: 'system', text: `Error: ${res.status} ${err}`, ts: Date.now() },
        ])
      }
      // Successful acknowledgment — mission_status SSE events will follow
    } catch (e) {
      setMessages(prev => [
        ...prev,
        { id: nextId(), role: 'system', text: `Network error: ${String(e)}`, ts: Date.now() },
      ])
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
        <span className="chat-title">Agent</span>
        <span className={`sse-dot ${sseState.connected ? 'sse-dot--on' : 'sse-dot--off'}`} title={sseState.connected ? 'SSE connected' : 'SSE disconnected'} />
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
          placeholder="Send instruction…"
          value={input}
          onChange={e => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          disabled={sending}
          aria-label="Navigation instruction"
        />
        <button
          className="btn-send"
          onClick={sendInstruction}
          disabled={sending || !input.trim()}
          aria-label="Send"
        >
          <Send size={16} />
        </button>
      </div>
    </div>
  )
}
