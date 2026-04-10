import { useState, useCallback } from 'react'
import { Navigation, Key, AlertCircle } from 'lucide-react'
import * as api from '../services/api'
import styles from './LoginPage.module.css'

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
      const data = await api.login(key.trim())
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
    <div className={styles.loginPage}>
      <div className={styles.card}>
        <div className={styles.logo}>
          <div className={styles.logoIcon}>
            <Navigation size={28} strokeWidth={2} />
          </div>
          <h1>LingTu</h1>
          <p>自主导航控制台</p>
        </div>

        <form className={styles.form} onSubmit={handleSubmit}>
          <div className={styles.inputWrap}>
            <Key size={16} className={styles.inputIcon} />
            <input
              type="password"
              className={styles.input}
              placeholder="输入 API Key..."
              value={key}
              onChange={(e) => setKey(e.target.value)}
              autoFocus
              disabled={loading}
            />
          </div>

          {error && (
            <div className={styles.error}>
              <AlertCircle size={14} /> {error}
            </div>
          )}

          <button
            type="submit"
            className={styles.btn}
            disabled={loading || !key.trim()}
          >
            {loading ? '验证中...' : '登录'}
          </button>
        </form>

        <p className={styles.hint}>
          API Key 通过环境变量 <code>LINGTU_API_KEY</code> 配置
        </p>
      </div>
    </div>
  )
}
