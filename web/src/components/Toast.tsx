import { X } from 'lucide-react'
import type { Toast, ToastKind } from '../hooks/useToast'

interface ToastContainerProps {
  toasts: Toast[]
  dismiss: (id: number) => void
}

function kindClass(kind: ToastKind) {
  if (kind === 'success') return 'toast--success'
  if (kind === 'error')   return 'toast--error'
  return 'toast--info'
}

export function ToastContainer({ toasts, dismiss }: ToastContainerProps) {
  if (toasts.length === 0) return null
  return (
    <div className="toast-container" aria-live="polite" aria-atomic="false">
      {toasts.map(t => (
        <div key={t.id} className={`toast ${kindClass(t.kind)}`} role="status">
          <span className="toast-msg">{t.message}</span>
          <button
            className="toast-close"
            onClick={() => dismiss(t.id)}
            aria-label="关闭通知"
          >
            <X size={12} />
          </button>
        </div>
      ))}
    </div>
  )
}
