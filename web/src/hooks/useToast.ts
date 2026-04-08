import { useState, useCallback, useRef } from 'react'
import type { Toast, ToastKind } from '../types'

// Re-export types for backward compatibility
export type { ToastKind, Toast } from '../types'

let _id = 0

export function useToast() {
  const [toasts, setToasts] = useState<Toast[]>([])
  const timers = useRef<Map<number, ReturnType<typeof setTimeout>>>(new Map())

  const dismiss = useCallback((id: number) => {
    setToasts(prev => prev.filter(t => t.id !== id))
    const t = timers.current.get(id)
    if (t) { clearTimeout(t); timers.current.delete(id) }
  }, [])

  const show = useCallback((message: string, kind: ToastKind = 'info') => {
    const id = ++_id
    setToasts(prev => [...prev, { id, message, kind }])
    const t = setTimeout(() => dismiss(id), 2500)
    timers.current.set(id, t)
  }, [dismiss])

  return { toasts, show, dismiss }
}
