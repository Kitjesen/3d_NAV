import { useEffect, useRef, useState } from 'react'

interface CountUpProps {
  end: number
  duration?: number
  suffix?: string
  prefix?: string
  decimals?: number
}

export function CountUp({ end, duration = 1800, suffix = '', prefix = '', decimals = 0 }: CountUpProps) {
  const [value, setValue] = useState(0)
  const rafRef = useRef<number>(0)
  const startTimeRef = useRef<number>(0)
  const hasStarted = useRef(false)
  const containerRef = useRef<HTMLSpanElement>(null)

  useEffect(() => {
    const observer = new IntersectionObserver(
      (entries) => {
        if (entries[0].isIntersecting && !hasStarted.current) {
          hasStarted.current = true
          startTimeRef.current = performance.now()
          const tick = (now: number) => {
            const elapsed = now - startTimeRef.current
            const progress = Math.min(elapsed / duration, 1)
            // ease out cubic
            const eased = 1 - Math.pow(1 - progress, 3)
            setValue(parseFloat((eased * end).toFixed(decimals)))
            if (progress < 1) {
              rafRef.current = requestAnimationFrame(tick)
            } else {
              setValue(end)
            }
          }
          rafRef.current = requestAnimationFrame(tick)
        }
      },
      { threshold: 0.3 }
    )
    const el = containerRef.current
    if (el) observer.observe(el)
    return () => {
      observer.disconnect()
      cancelAnimationFrame(rafRef.current)
    }
  }, [end, duration, decimals])

  const display = decimals > 0 ? value.toFixed(decimals) : Math.floor(value).toString()
  return <span ref={containerRef}>{prefix}{display}{suffix}</span>
}
