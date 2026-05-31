# CLI — 命令行入口与交互界面

## 概述

CLI 包提供 LingTu 的终端入口、交互式 REPL、配置文件管理和运行状态控制，是用户与系统交互的主要界面。

| 文件 | 职责 |
|------|------|
| `main.py` | CLI 主入口 — 命令行参数解析、profile 选择模块组合与启动 |
| `profiles_data.py` | Profile 定义：14 个命名 profile + 7 个机器人预设，共 21 种组合 |
| `repl.py` | 交互式 REPL — navigate / map / semantic / agent / teleop / monitor 命令 |
| `run_state.py` | 运行状态管理 — session 生命周期、组件启停协调 |
| `bootstrap.py` | 引导初始化 — 环境检查、依赖加载、配置合并 |
| `paths.py` | 路径常量 — 关键目录与文件路径管理 |
| `term.py` | 终端输出格式化 — 颜色、表格、进度条 |
| `ui.py` | 终端 UI 组件 — 选择器、提示框等交互工具 |
| `logging_util.py` | 日志工具 — 统一日志格式与级别配置 |
| `runtime_audit.py` | 运行时审计 — 模块健康检查与性能采样 |
| `runtime_display.py` | 运行时显示 — session 状态面板 |
| `runtime_extra.py` | 运行时扩展命令 |

## 用法

```bash
python lingtu.py              # 交互式 profile 选择
python lingtu.py stub          # 框架测试模式
python lingtu.py s100p         # 真机部署
python lingtu.py --list        # 列出所有 profile
```
