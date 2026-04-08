import { useState, useCallback } from 'react'
import { Navigation, Key, AlertCircle } from 'lucide-react'

interface LoginPageProps {
  onLogin: () => void
}

export function LoginPage({ onLogin }: LoginPageProps) {
  const [key, setKey] = useState('')
  const [error, setError] = useState('')
  const [loading, setLoading] = useState(false)

  const handleSubmit = useCallback(async (e: React.FormEvent) => {
    e.preventDefault()
    if (!key.trim()) return

    setLoading(true)
    setError('')

    try {
      const res = await fetch('/api/v1/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ key: key.trim() }),
      })
      const data = await res.json()
      if (data.ok) {
        onLogin()
      } else {
        setError(data.message || '认证失败')
      }
    } catch {
      setError('无法连接到机器人')
    } finally {
      setLoading(false)
    }
  }, [key, onLogin])

  return (
    <div className="login-page">
      <div className="login-card">
        <div className="login-logo">
          <Navigation size={32} />
          <h1>LingTu</h1>
          <p>机器人控制台</p>
        </div>

        <form className="login-form" onSubmit={handleSubmit}>
          <div className="login-input-wrap">
            <Key size={16} className="login-input-icon" />
            <input
              type="password"
              className="login-input"
              placeholder="输入 API Key..."
              value={key}
              onChange={(e) => setKey(e.target.value)}
              autoFocus
              disabled={loading}
            />
          </div>

          {error && (
            <div className="login-error">
              <AlertCircle size={14} /> {error}
            </div>
          )}

          <button
            type="submit"
            className="login-btn"
            disabled={loading || !key.trim()}
          >
            {loading ? '验证中...' : '登录'}
          </button>
        </form>

        <p className="login-hint">
          API Key 通过环境变量 LINGTU_API_KEY 配置
        </p>
      </div>
    </div>
  )
}
