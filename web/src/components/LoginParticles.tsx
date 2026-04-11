import { useEffect, useRef } from 'react'

/**
 * Canvas particle field for the login page background.
 * Floating dots connected by thin lines — gives the page
 * motion without being distracting.
 */
export function LoginParticles() {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    let rafId = 0
    let particles: Particle[] = []

    interface Particle {
      x: number
      y: number
      vx: number
      vy: number
      r: number
      hue: number
    }

    // Device-pixel-ratio aware sizing — cap DPR at 1.5 for perf
    const resize = () => {
      const dpr = Math.min(window.devicePixelRatio || 1, 1.5)
      const { innerWidth: w, innerHeight: h } = window
      canvas.width = w * dpr
      canvas.height = h * dpr
      canvas.style.width = `${w}px`
      canvas.style.height = `${h}px`
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)

      // Much lower density — 40 particles max is plenty visually
      const target = Math.min(40, Math.floor((w * h) / 24000))
      if (particles.length !== target) {
        particles = Array.from({ length: target }, () => ({
          x: Math.random() * w,
          y: Math.random() * h,
          vx: (Math.random() - 0.5) * 0.2,
          vy: (Math.random() - 0.5) * 0.2,
          r: 0.8 + Math.random() * 1.4,
          hue: Math.random() < 0.5 ? 250 : 320, // indigo or pink
        }))
      }
    }

    resize()
    window.addEventListener('resize', resize)

    // Throttle to ~30fps — enough for background ambience, half the work
    let lastFrame = 0
    const frameInterval = 1000 / 30

    const render = (now: number) => {
      if (now - lastFrame < frameInterval) {
        rafId = requestAnimationFrame(render)
        return
      }
      lastFrame = now

      const w = canvas.clientWidth
      const h = canvas.clientHeight
      ctx.clearRect(0, 0, w, h)

      // Update positions
      for (const p of particles) {
        p.x += p.vx
        p.y += p.vy
        if (p.x < 0) p.x = w
        if (p.x > w) p.x = 0
        if (p.y < 0) p.y = h
        if (p.y > h) p.y = 0
      }

      // Draw connections (batched into one path, single stroke call)
      const maxDist = 120
      const maxDistSq = maxDist * maxDist
      ctx.strokeStyle = 'rgba(140, 150, 220, 0.08)'
      ctx.lineWidth = 0.5
      ctx.beginPath()
      for (let i = 0; i < particles.length; i++) {
        const a = particles[i]
        for (let j = i + 1; j < particles.length; j++) {
          const b = particles[j]
          const dx = a.x - b.x
          const dy = a.y - b.y
          const distSq = dx * dx + dy * dy
          if (distSq < maxDistSq) {
            ctx.moveTo(a.x, a.y)
            ctx.lineTo(b.x, b.y)
          }
        }
      }
      ctx.stroke()

      // Draw particles (no per-particle shadowBlur — too expensive)
      for (const p of particles) {
        ctx.beginPath()
        ctx.arc(p.x, p.y, p.r, 0, Math.PI * 2)
        ctx.fillStyle = `hsla(${p.hue}, 85%, 72%, 0.6)`
        ctx.fill()
      }

      rafId = requestAnimationFrame(render)
    }

    rafId = requestAnimationFrame(render)

    return () => {
      cancelAnimationFrame(rafId)
      window.removeEventListener('resize', resize)
    }
  }, [])

  return (
    <canvas
      ref={canvasRef}
      aria-hidden="true"
      style={{
        position: 'absolute',
        inset: 0,
        pointerEvents: 'none',
        zIndex: 0,
      }}
    />
  )
}
