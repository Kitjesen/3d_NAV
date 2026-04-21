# LingTu Dashboard 前端使用说明

> 这份文档面向准备修改、润色和调试前端的人。前端源码位于 `web/`，技术栈是 React 19 + TypeScript + Vite 8 + CSS Modules，不使用 Tailwind 或运行时 CSS 框架。

## 1. 前端能做什么

LingTu Dashboard 是机器人导航系统的 Web 控制台。它通过后端 `GatewayModule` 暴露的 REST、SSE、WebSocket 接口获取机器人状态并发送控制命令。

主要页面：

| 页面 | 入口 Tab | 主要组件 | 功能 |
| --- | --- | --- | --- |
| 控制台 | `console` | `CameraFeed`、`LocalizationCard`、`MiniMap`、`ChatPanel` | 查看相机、状态、轨迹、发送语义指令；Widget 可拖拽/缩放 |
| 场景 | `scene` | `SceneView`、`Scene3D` | 地图/轨迹/目标图层展示，点击地图发送目标 |
| 地图 | `map` | `MapView`、`PointCloudViewer` | 地图保存、激活、重命名、删除、点云预览 |
| SLAM | `slam` | `SlamPanel` | 切换建图、定位、停止等模式 |
| 登录 | 自动判断 | `LoginPage` | 后端启用 API key 时显示登录页 |
| Landing | `?landing` | `Landing` | 展示页面，和机器人控制台分开 |

## 2. 本地启动

推荐使用 conda 环境：

```bash
# 首次创建环境
conda env create -f environment.yml

# 之后每次进入前端开发
conda activate lingtu-web
cd /Users/hanzhuo/Desktop/lingtu/MapPilot/web
npm ci
npm run dev
```

打开：

```text
http://localhost:3000
```

常用脚本：

```bash
npm run dev      # Vite 开发服务器，默认 http://localhost:3000
npm run build    # TypeScript 检查 + 生产构建，输出 web/dist/
npm run lint     # ESLint 检查
npm run preview  # 本地预览 dist 构建产物
```

## 3. 连接后端

开发服务器在 `web/vite.config.ts` 里配置了代理：

```ts
const ROBOT_HOST = process.env.ROBOT_HOST || 'localhost:5050'
```

默认情况下，前端会把这些路径代理到 `http://localhost:5050`：

- `/api/*`
- `/map/*`
- `/mcp/*`
- `/ws/*`

### 3.1 连接本机后端

如果你在本机运行了 `python lingtu.py stub` 或其他 Gateway profile：

```bash
cd web
npm run dev
```

前端会连接：

```text
http://localhost:5050
```

### 3.2 连接机器人后端

如果机器人 IP 是 `192.168.66.190`：

```bash
cd web
ROBOT_HOST=192.168.66.190:5050 npm run dev
```

如果通过 SSH 隧道：

```bash
ssh -L 5050:localhost:5050 sunrise@192.168.66.190
cd web
npm run dev
```

## 4. 目录结构

```text
web/
├── package.json              # npm 脚本和依赖
├── vite.config.ts            # Vite + API/WS 代理
├── index.html                # HTML 入口
├── public/                   # favicon、icons、静态图片
└── src/
    ├── main.tsx              # React root
    ├── App.tsx               # 登录/landing/Dashboard、Tab 路由
    ├── App.css               # 全局设计 token 和主布局
    ├── index.css             # 基础样式
    ├── components/           # 页面组件
    ├── hooks/                # SSE、WebSocket、Toast、点云流 hooks
    ├── services/api.ts       # REST API 封装
    ├── types/index.ts        # 前端类型定义
    ├── utils/slashCommands.ts# ChatPanel 的 / 命令
    └── workers/cloudDecoder.ts # 点云解码 Worker
```

## 5. 主要文件怎么读

建议按这个顺序看：

1. `src/App.tsx`: 看页面如何切换、登录如何判断、Dashboard 如何组织。
2. `src/services/api.ts`: 看按钮最后调用哪些后端接口。
3. `src/hooks/useSSE.ts`: 看实时状态流如何进入前端状态。
4. `src/types/index.ts`: 看状态、事件、地图、Tab 等类型。
5. `src/components/Topbar.tsx`、`StatusBar.tsx`: 看全局状态展示。
6. `src/components/SceneView.tsx`、`MapView.tsx`、`SlamPanel.tsx`: 看核心业务页面。
7. `src/App.css` 和各组件 `*.module.css`: 看视觉风格。

## 6. 后端接口速查

前端主要调用这些接口：

| 类型 | 路径 | 前端位置 | 用途 |
| --- | --- | --- | --- |
| SSE | `/api/v1/events` | `hooks/useSSE.ts` | 接收 odometry、mission、safety、scene_graph、map_cloud 等实时事件 |
| WS | `/ws/teleop` | `hooks/useCamera.ts`、`CameraFeed` | 相机帧和遥控数据 |
| WS | `/ws/cloud` | `hooks/useBinaryCloud.ts` | 二进制点云流 |
| REST | `/api/v1/goal` | `services/api.ts` | 发送坐标目标 |
| REST | `/api/v1/instruction` | `services/api.ts`、`ChatPanel` | 发送自然语言导航指令 |
| REST | `/api/v1/stop` | `services/api.ts` | 急停 |
| REST | `/api/v1/slam/switch` | `services/api.ts`、`SlamPanel` | 切换 SLAM 模式 |
| REST | `/api/v1/slam/maps` | `services/api.ts`、`MapView` | 获取地图列表 |
| REST | `/api/v1/map/save` | `services/api.ts`、`MapView` | 保存地图 |
| REST | `/api/v1/map/activate` | `services/api.ts`、`MapView` | 激活地图 |
| REST | `/api/v1/session` | `services/api.ts` | 查询当前建图/导航/探索 session |
| REST | `/api/v1/session/start` | `services/api.ts` | 开始建图、导航或探索 |
| REST | `/api/v1/session/end` | `services/api.ts` | 结束当前 session |

后端实现集中在：

```text
src/gateway/gateway_module.py
```

## 7. 页面和组件说明

### 7.1 `App.tsx`

职责：

- 判断是否显示 landing 页面。
- 检查后端是否要求登录。
- 保存当前 Tab。
- 订阅 SSE 状态。
- 组织控制台的 4 个可拖拽 Widget。

调试入口：

- `?landing`: 强制显示展示页。
- `?login`: 强制显示登录页。

### 7.2 `FloatingWidget`

用于控制台页的可拖拽/缩放窗口。

注意点：

- 布局保存在 `localStorage`。
- 调 UI 时如果布局异常，可以清浏览器 localStorage 或用页面里的重置布局按钮。
- 固定格式区域需要注意 `minSize`，避免移动端或窄屏挤压内容。

### 7.3 `ChatPanel`

功能：

- 发送自然语言指令。
- 显示 agent/assistant 消息。
- 支持 `/` 命令补全。
- 快捷指令 chip。

相关文件：

- `src/components/ChatPanel.tsx`
- `src/utils/slashCommands.ts`
- `src/services/api.ts`

### 7.4 `SceneView`

功能：

- 地图/网格/轨迹/路径/目标/本机图层。
- 点击地图发送导航目标。
- 左右侧状态面板。

修改重点：

- Canvas/WebGL 绘制是否清晰。
- 图层开关状态是否明显。
- 坐标点击和后端目标发送是否一致。
- 空地图、无定位、断开连接状态。

### 7.5 `MapView`

功能：

- 地图列表。
- 保存、激活、重命名、删除地图。
- 查看点云预览。

修改重点：

- 操作前后的 loading/error/success 提示。
- 删除/重命名确认弹窗。
- 地图为空、地图缺少 `pcd/tomogram/occupancy` 的状态表达。

### 7.6 `SlamPanel`

功能：

- 切换建图、定位、停止。
- 展示 SLAM 状态。

修改重点：

- 防止重复点击。
- pending/error 状态。
- 当前 session 与按钮可用状态对齐。

## 8. 修改前端的工作流

### 8.1 改样式

优先改组件自己的 `*.module.css`。

全局色彩、空间、玻璃效果等在：

```text
src/App.css
src/index.css
```

建议：

- 先确认组件边界，再改视觉。
- 不要让按钮文字在窄屏溢出。
- 控制台 Widget 尺寸要有稳定 `minSize`。
- 地图、相机、点云这类固定比例区域要保持明确高度。

### 8.2 改接口调用

优先改：

```text
src/services/api.ts
src/types/index.ts
```

然后再改具体组件。这样能避免同一个接口在多个组件里重复写 `fetch`。

### 8.3 改实时状态

优先看：

```text
src/hooks/useSSE.ts
src/types/index.ts
```

新增 SSE 事件时，需要同时确认：

- `useSSE.ts` 是否处理新事件类型。
- `types/index.ts` 是否有对应类型。
- 后端 `GatewayModule.push_event()` 发送的字段是否匹配。

### 8.4 改点云或相机

相关文件：

```text
src/components/PointCloudViewer.tsx
src/components/CameraFeed.tsx
src/hooks/useCamera.ts
src/hooks/useBinaryCloud.ts
src/workers/cloudDecoder.ts
```

注意：

- 点云和相机是性能敏感区域。
- 尽量避免每帧创建大量对象。
- 改完要在 Chrome DevTools 里看 FPS、内存、控制台错误。

## 9. 验证清单

每次改完前端，至少做：

```bash
npm run lint
npm run build
```

手工检查：

- 控制台页能打开，不报红色运行时错误。
- Tab 切换正常。
- 断开后端时页面不崩溃。
- `ROBOT_HOST=<host>:5050 npm run dev` 能代理到后端。
- 地图页和 SLAM 页的按钮在 pending/error 状态下不会重复误触。
- 移动或窄屏下文字不溢出。

## 10. 常见问题

### 10.1 页面显示空白

先看浏览器 DevTools Console。

常见原因：

- TypeScript/运行时错误。
- 后端未启动但组件没有处理空状态。
- localStorage 保存了异常 Widget 布局。

处理：

```js
localStorage.clear()
location.reload()
```

### 10.2 前端连不上机器人

确认：

```bash
ROBOT_HOST=192.168.66.190:5050 npm run dev
```

或者先做 SSH 隧道：

```bash
ssh -L 5050:localhost:5050 sunrise@192.168.66.190
```

浏览器访问：

```text
http://localhost:3000
```

### 10.3 `npm run build` 失败

按顺序检查：

1. 当前是否在 `web/` 目录。
2. 是否激活 `lingtu-web` conda 环境。
3. `node --version` 是否满足 Vite 8 要求，推荐 Node 22 LTS 或更高。
4. 是否安装依赖：`npm install`。
5. TypeScript 类型是否和接口字段不一致。

### 10.4 修改 CSS 后没有变化

可能原因：

- 改错了组件对应的 `*.module.css`。
- 组件使用了全局 `App.css` token。
- 浏览器缓存或 Vite HMR 状态异常。

处理：

```bash
# 停掉 dev server 后重启
npm run dev
```

## 11. 推荐第一批润色点

适合先做、风险较小：

1. 给 `MapView` 增强空状态、加载状态、错误状态。
2. 给 `SlamPanel` 的 session/pending 状态做更清楚的按钮禁用和提示。
3. 优化 `ChatPanel` 消息气泡、命令补全和快捷指令排列。
4. 调整 `Topbar` 与 `StatusBar` 的信息密度，突出连接状态和当前任务。
5. 给 `SceneView` 增强图层开关、目标点、路径颜色和点击反馈。
6. 给断开后端状态做统一视觉：所有页面都能优雅降级。

## 12. 生产部署

构建产物在：

```text
web/dist/
```

机器人端由 `GatewayModule` 静态挂载 `web/dist/`。

生产构建：

```bash
cd web
npm run build
```

部署到机器人通常有两种方式：

```bash
# 方式 1: 复制 dist
scp -r dist/ sunrise@192.168.66.190:~/data/inovxio/lingtu/web/dist/
ssh sunrise@192.168.66.190 "sudo systemctl restart lingtu"

# 方式 2: 机器人上拉代码并构建
ssh sunrise@192.168.66.190
cd ~/data/inovxio/lingtu/web
npm install
npm run build
sudo systemctl restart lingtu
```

如果只是在 Mac 上润色，不需要部署；本地 `npm run dev` 即可预览。
