# LingTu Dashboard

LingTu (灵途) 机器人导航系统的 Web 控制台。Arc / Raycast 风格设计，玻璃拟态 + 可拖拽 Widget 布局。

更详细的本地开发、页面说明、接口调试和润色工作流见 [FRONTEND_USAGE_ZH.md](./FRONTEND_USAGE_ZH.md)。

## Quick Start

```bash
npm install
npm run dev     # http://localhost:3000
npm run build   # 产物 → dist/
```

部署到机器人后由 GatewayModule 在 `:5050` 提供服务 (FastAPI StaticFiles mount)。

## Architecture

```
src/
├── App.tsx              # 主入口 — Auth + Tab 路由 + FloatingWidget 布局
├── App.css              # Design tokens (colors, glass, shadows, motion, spacing)
├── components/
│   ├── FloatingWidget   # 可拖拽/可 resize 的 widget wrapper (localStorage 持久化)
│   ├── Modal            # PromptModal + ConfirmModal (替代原生 prompt/confirm)
│   ├── LoginPage        # 登录页 (粒子背景 + 玻璃卡 + 三态按钮)
│   ├── LoginParticles   # Canvas 粒子系统 (indigo/pink 连线)
│   ├── CameraFeed       # 相机 MJPEG 直播 + E-STOP + HUD 浮层
│   ├── CameraHud        # 遥测 HUD (SLAM Hz / 速度 / 电量 / 温度 / 延迟)
│   ├── ChatPanel        # 智能体对话 + /命令补全 + 快捷指令栏
│   ├── GpsCard          # 旋转地球卡 (定位 + X/Y/航向/速度/SLAM模式)
│   ├── MiniMap          # 2D 俯视 Canvas (机器人 + 轨迹 + 路径 + 目标)
│   ├── SceneView        # RViz 级场景视图 (图层控制 + 点击发目标 + 地图管理)
│   ├── MapView          # 地图管理 (3D 点云预览 + 分类列表 + CRUD)
│   ├── PointCloudViewer # WebGL 点云渲染 (PCD 解析 + 4 配色方案)
│   ├── SlamPanel        # SLAM 模式切换 (fastlio2/localizer/stop)
│   ├── PathView         # 轨迹视图 (legacy, 被 SceneView 替代)
│   ├── Topbar           # 顶栏 (品牌 + 在线状态 + 位置/导航/SLAM stats)
│   ├── TabBar           # Pill 分段标签栏 (控制台/场景/地图/SLAM)
│   ├── StatusBar        # 底部状态栏 (位置/航向/速度/导航/运行时长/版本)
│   └── Toast            # 通知 Toast (success/error/info)
├── hooks/
│   ├── useSSE.ts        # SSE 实时数据流 (/api/v1/events)
│   ├── useCamera.ts     # WebSocket MJPEG 帧 (/ws/teleop)
│   └── useToast.ts      # Toast 状态管理
├── services/
│   └── api.ts           # 集中式 API 层 (fetch 封装)
├── utils/
│   └── slashCommands.ts # /命令注册表 + 自动补全 + 执行器
└── types/
    └── index.ts         # SSEState, MapInfo, Tab, Toast 等类型
```

## Tabs

| Tab | 组件 | 功能 |
|-----|------|------|
| 控制台 | FloatingWidget x4 | 相机直播 + 定位地球 + MiniMap + 智能体对话，可自由拖拽/resize |
| 场景 | SceneView | RViz 级地图：5 图层开关 + 点击发送目标 + 左侧地图抽屉 + 右侧状态栏 |
| 地图 | MapView | 地图 CRUD：3D 点云预览 + 分类(语义/点云/空) + 保存/激活/重命名/删除 |
| SLAM | SlamPanel | 切换 SLAM 模式 (建图 fastlio2 / 定位 localizer / 停止) |

## Design System

**Arc / Raycast 风格**：

- **背景**：深蓝紫 `#0A0B14` + 漂移彩色 blur orb (indigo + pink)
- **表面**：玻璃拟态 `backdrop-filter: blur(20px) saturate(140%)` + `inset 0 1px rgba(255,255,255,0.08)` inner highlight
- **品牌色**：indigo → purple → pink → amber 彩虹渐变 (`--gradient-brand`)
- **状态色**：青薄荷 `#5EEAD4` (正常) / 琥珀 `#FBBF24` (警告) / 珊瑚 `#F87171` (错误)
- **字体**：Geist + Geist Mono + Inter + Noto Sans SC
- **圆角**：6 级 (r-xs 4px … r-2xl 20px)
- **间距**：4px 节奏 (--s-1 4px … --s-6 24px)
- **动效**：cubic-bezier(0.16, 1, 0.3, 1) spring + 300ms 基线

## FloatingWidget

控制台 tab 的 4 个 widget 都可以自由拖拽和 resize：

- **拖拽**：鼠标移到 widget 顶部，出现小 grabber pill，按住拖动
- **Resize**：鼠标移到边缘或四角，光标变化，拖拽调整大小
- **置顶**：点击任意位置自动 z-index 置顶
- **持久化**：布局自动存 localStorage，刷新后恢复
- **重置**：右下角"重置布局"按钮清除记忆恢复默认

## /命令补全

ChatPanel 输入 `/` 自动弹出命令下拉 (Claude Code 风)：

| 命令 | 说明 |
|------|------|
| `/go <x> <y>` | 发送导航目标到指定坐标 |
| `/stop` | 紧急停止 |
| `/status` | 查看当前状态 |
| `/map` | 跳转地图管理 |
| `/help` | 显示帮助 |

键盘操作：↑↓ 选择、Tab/Enter 补全、Esc 关闭。

## Backend API

Dashboard 连接 GatewayModule 的 REST + SSE + WebSocket：

| 协议 | 端点 | 用途 |
|------|------|------|
| SSE | `/api/v1/events` | 实时推送 odometry/mission/safety/slam/scene_graph |
| WS | `/ws/teleop` | MJPEG 相机帧 + 摇杆控制 |
| REST | `/api/v1/health` | 系统健康 (32 modules + brainstem gRPC) |
| REST | `/api/v1/goal` | 发送导航目标 |
| REST | `/api/v1/stop` | 紧急停止 |
| REST | `/api/v1/maps` | 地图 CRUD |
| REST | `/api/v1/slam/switch` | SLAM 模式切换 |

## Build & Deploy

```bash
# 本地开发
npm run dev

# 生产构建
npm run build        # → dist/ (~60KB CSS, ~270KB JS)

# 部署到机器人
scp -r dist/ sunrise@192.168.66.190:~/data/inovxio/lingtu/web/dist/
ssh sunrise@192.168.66.190 "sudo systemctl restart lingtu"

# 或通过 git
ssh sunrise@192.168.66.190 "cd ~/data/inovxio/lingtu && git pull && cd web && npm run build"
```

## Tech Stack

- React 19 + TypeScript
- Vite 8
- CSS Modules (no Tailwind, no styled-components)
- WebGL (PointCloudViewer)
- Canvas 2D (MiniMap, SceneView, LoginParticles)
- Lucide React (icons)
- Zero runtime CSS/animation libraries
