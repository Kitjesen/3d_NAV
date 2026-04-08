// Slash command parser and executor for ChatPanel
// Extracts command handling logic from ChatPanel.tsx

import type { SSEState } from '../types'
import * as api from '../services/api'

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

export interface SlashCommandResult {
  command: string
  response: string
}

export function isSlashCommand(input: string): boolean {
  return input.trimStart().startsWith('/')
}

export async function executeSlashCommand(raw: string, sseState: SSEState): Promise<string> {
  const parts = raw.trim().split(/\s+/)
  const cmd = parts[0].toLowerCase()

  if (cmd === '/help') {
    return HELP_TEXT
  }

  if (cmd === '/status') {
    const ms = sseState.missionStatus
    const odom = sseState.odometry
    const safety = sseState.safetyState
    const stateZh = ms ? (NAV_STATE_ZH[ms.state] ?? ms.state) : '未知'
    const pos =
      typeof odom?.x === 'number' && typeof odom?.y === 'number'
        ? `(${odom.x.toFixed(2)}, ${odom.y.toFixed(2)})`
        : '--'
    const estop = safety?.estop ? '急停激活' : '正常'
    return `状态：${stateZh}\n位置：${pos}\n安全：${estop}`
  }

  if (cmd === '/map') {
    return '地图管理请切换到地图标签。'
  }

  if (cmd === '/stop') {
    try {
      const res = await api.sendStop()
      return res.ok ? '急停指令已发送。' : `急停失败：HTTP ${res.status}`
    } catch (e) {
      return `急停网络错误：${String(e)}`
    }
  }

  if (cmd === '/go') {
    const x = parseFloat(parts[1])
    const y = parseFloat(parts[2])
    if (isNaN(x) || isNaN(y)) {
      return '用法：/go <x> <y>  （示例：/go 3.5 -1.2）'
    }
    try {
      const res = await api.sendGoal(x, y)
      return res.ok ? `目标已提交：(${x}, ${y})` : `导航失败：HTTP ${res.status}`
    } catch (e) {
      return `导航网络错误：${String(e)}`
    }
  }

  return `未知指令：${cmd}。输入 /help 查看可用指令。`
}
