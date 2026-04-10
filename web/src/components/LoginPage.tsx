import { useState, useCallback, useEffect } from 'react'
import { AlertCircle, ArrowRight } from 'lucide-react'
import * as api from '../services/api'
import { LoginParticles } from './LoginParticles'
import styles from './LoginPage.module.css'

interface LoginPageProps {
  onLogin: () => void
}

/**
 * Quadruped robot silhouette — custom SVG for the login page hero.
 * Gradient stroke traces the body outline; 4 legs with dots; eye glow.
 */
function RobotGlyph() {
  return (
    <svg
      viewBox="0 0 64 64"
      width="34"
      height="34"
      fill="none"
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
    >
      <defs>
        <linearGradient id="rgrad" x1="0" y1="0" x2="64" y2="64" gradientUnits="userSpaceOnUse">
          <stop offset="0" stopColor="#FFFFFF" stopOpacity="0.95" />
          <stop offset="1" stopColor="#FFFFFF" stopOpacity="0.7" />
        </linearGradient>
      </defs>
      {/* Body */}
      <rect x="14" y="24" width="36" height="16" rx="5" stroke="url(#rgrad)" strokeWidth="2.2" />
      {/* Head */}
      <path d="M48 28 L58 24 L58 36 L48 36 Z" stroke="url(#rgrad)" strokeWidth="2.2" />
      {/* Eye */}
      <circle cx="54" cy="29" r="1.4" fill="#FBBF24" />
      {/* Legs */}
      <line x1="19" y1="40" x2="19" y2="52" stroke="url(#rgrad)" strokeWidth="2.2" />
      <line x1="28" y1="40" x2="28" y2="52" stroke="url(#rgrad)" strokeWidth="2.2" />
      <line x1="37" y1="40" x2="37" y2="52" stroke="url(#rgrad)" strokeWidth="2.2" />
      <line x1="46" y1="40" x2="46" y2="52" stroke="url(#rgrad)" strokeWidth="2.2" />
      {/* Feet dots */}
      <circle cx="19" cy="54" r="1.6" fill="#FFFFFF" />
      <circle cx="28" cy="54" r="1.6" fill="#FFFFFF" />
      <circle cx="37" cy="54" r="1.6" fill="#FFFFFF" />
      <circle cx="46" cy="54" r="1.6" fill="#FFFFFF" />
      {/* Tail */}
      <path d="M14 28 L8 24" stroke="url(#rgrad)" strokeWidth="2.2" />
    </svg>
  )
}

export function LoginPage({ onLogin }: LoginPageProps) {
  const [key,     setKey]     = useState('')
  const [error,   setError]   = useState('')
  const [loading, setLoading] = useState(false)
  const [success, setSuccess] = useState(false)

  // Ambient system status — purely cosmetic "已上线" indicator
  const [serverAlive, setServerAlive] = useState<boolean | null>(null)

  useEffect(() => {
    let cancelled = false
    api.checkAuth()
      .then(() => { if (!cancelled) setServerAlive(true) })
      .catch(() => { if (!cancelled) setServerAlive(false) })
    return () => { cancelled = true }
  }, [])

  const handleSubmit = useCallback(async (e: React.FormEvent) => {
    e.preventDefault()
    if (!key.trim() || loading) return
    setLoading(true)
    setError('')
    try {
      const data = await api.login(key.trim())
      if (data.ok) {
        setSuccess(true)
        // Delay to let the success animation play
        setTimeout(onLogin, 500)
      } else {
        setError(data.message || '认证失败')
      }
    } catch {
      setError('无法连接到机器人')
    } finally {
      setLoading(false)
    }
  }, [key, loading, onLogin])

  return (
    <div className={[styles.loginPage, success ? styles.pageSuccess : ''].filter(Boolean).join(' ')}>
      <LoginParticles />

      {/* Grid mesh overlay */}
      <div className={styles.grid} aria-hidden="true" />

      {/* Drifting color orbs */}
      <div className={styles.orb1} aria-hidden="true" />
      <div className={styles.orb2} aria-hidden="true" />
      <div className={styles.orb3} aria-hidden="true" />

      {/* Top system status */}
      <div className={styles.systemStatus}>
        <span className={serverAlive === null ? styles.statusDot : serverAlive ? styles.statusDotOn : styles.statusDotOff} />
        {serverAlive === null ? '正在连接…' : serverAlive ? '导航系统已上线' : '机器人离线'}
      </div>

      {/* Bottom version stamp */}
      <div className={styles.version}>
        LINGTU · v1.8.0
      </div>

      <div className={styles.card}>
        <div className={styles.cardGlow} aria-hidden="true" />

        <div className={styles.brand}>
          <div className={styles.brandIcon}>
            <RobotGlyph />
          </div>
          <div className={styles.brandText}>
            <h1>LingTu</h1>
            <p>自主导航控制台</p>
          </div>
        </div>

        <form className={styles.form} onSubmit={handleSubmit}>
          <label className={styles.label} htmlFor="api-key">
            访问密钥
          </label>
          <div className={styles.inputWrap}>
            <input
              id="api-key"
              type="password"
              className={styles.input}
              placeholder="输入 API Key"
              value={key}
              onChange={(e) => { setKey(e.target.value); setError('') }}
              autoFocus
              autoComplete="off"
              spellCheck={false}
              disabled={loading || success}
            />
            <div className={styles.inputGlow} aria-hidden="true" />
          </div>

          {error && (
            <div className={styles.error} role="alert">
              <AlertCircle size={13} />
              <span>{error}</span>
            </div>
          )}

          <button
            type="submit"
            className={styles.btn}
            disabled={loading || success || !key.trim()}
          >
            {loading ? (
              <span className={styles.btnLoading}>
                <span className={styles.spinner} />
                <span>正在验证</span>
              </span>
            ) : success ? (
              <span className={styles.btnSuccess}>
                <span className={styles.checkmark}>✓</span>
                <span>已登录</span>
              </span>
            ) : (
              <span className={styles.btnIdle}>
                <span>进入控制台</span>
                <ArrowRight size={15} strokeWidth={2.5} />
              </span>
            )}
          </button>
        </form>

        <div className={styles.hint}>
          通过环境变量 <code>LINGTU_API_KEY</code> 配置
        </div>
      </div>
    </div>
  )
}
