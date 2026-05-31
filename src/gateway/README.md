# Gateway — L6 外部接口模块

## 概述

Gateway 模块对外暴露 REST / SSE / WebSocket / MCP 接口，提供机器人状态查询、控制指令下发、实时数据流推送和 AI Agent 远程控制能力。

| 模块 | 职责 |
|------|------|
| `gateway_module.py` | FastAPI HTTP/WS/SSE 服务（端口 5050），drift watchdog + save hooks |
| `mcp_server.py` | MCP JSON-RPC 服务（端口 8090），暴露 @skill 工具给 AI Agent |
| `rerun_bridge_module.py` | Rerun 可视化桥接 |
| `auth.py` | 认证模块 |
| `schemas.py` | Pydantic 数据模型 |
| `native_factories.py` | C++ 本地工厂注册 |

## routes/

FastAPI 路由处理器：

| 路由 | 用途 |
|------|------|
| `app.py` | 应用主路由 |
| `maps.py` | 地图管理 REST API |
| `commands.py` | 指令下发 |
| `status.py` | 状态查询 |
| `realtime.py` | SSE 实时事件流 |
| `camera.py` | 摄像头流 |
| `session.py` | Session 生命周期 |
| `operations.py` | 运维操作 |
| `auth.py` | 认证端点 |
| `assets.py` | 静态资源 |
| `diagnostics.py` | 诊断接口 |

## services/

业务逻辑层，包含 `runtime_dataflow.py`、`runtime_status.py`、`commands.py`、`goal_builder.py`、`map_safety.py`、`map_paths.py`、`state_snapshot.py`、`traffic.py` 等，封装 Gateway 核心数据流与状态管理。

## templates/

HTML 模板（Teleop 控制面板、地图仪表盘等前端资源）。
