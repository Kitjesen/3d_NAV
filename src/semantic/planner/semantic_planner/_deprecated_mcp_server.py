# Copyright 2025-2026 穹沛科技 (Qiongpei Technology)
# Inspired by DimOS mcp_server.py (dimensionalOS/dimos), Apache 2.0 License
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
LingTu MCP Server — 让外部 AI 工具直接控制机器人。
借鉴 DimOS mcp_server.py, Apache 2.0 License.

协议: MCP (Model Context Protocol) over HTTP POST
端点: POST /mcp (JSON-RPC 2.0)

支持的方法:
  initialize     — 握手
  tools/list     — 列出所有 skill
  tools/call     — 调用 skill

用法:
  server = MCPServer(skill_registry, port=8090)
  server.start()  # 后台线程
  # Claude Desktop 配置: {"url": "http://192.168.66.190:8090/mcp"}
"""

from __future__ import annotations

import json
import logging
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from semantic.planner.semantic_planner.skill_registry import SkillRegistry

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# JSON-RPC helpers
# ---------------------------------------------------------------------------

def _jsonrpc_result(req_id: Any, result: Any) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "result": result}


def _jsonrpc_result_text(req_id: Any, text: str) -> dict:
    return _jsonrpc_result(req_id, {"content": [{"type": "text", "text": text}]})


def _jsonrpc_error(req_id: Any, code: int, message: str) -> dict:
    return {"jsonrpc": "2.0", "id": req_id, "error": {"code": code, "message": message}}


# ---------------------------------------------------------------------------
# JSON-RPC handlers
# ---------------------------------------------------------------------------

def _handle_initialize(req_id: Any) -> dict:
    return _jsonrpc_result(
        req_id,
        {
            "protocolVersion": "2025-11-25",
            "capabilities": {"tools": {}},
            "serverInfo": {"name": "lingtu", "version": "1.0.0"},
        },
    )


def _handle_tools_list(req_id: Any, skill_registry: "SkillRegistry") -> dict:
    """将 SkillRegistry 中所有 skill 转换为 MCP tool schema。"""
    tools = []
    for info in skill_registry.get_skills():
        # 构造 JSON Schema inputSchema
        properties: dict[str, Any] = {}
        required_list: list[str] = []
        for param_name, param_schema in info.parameters.items():
            prop: dict[str, Any] = {"type": param_schema.get("type", "string")}
            if not param_schema.get("required", True):
                prop["default"] = param_schema.get("default", "")
            properties[param_name] = prop
            if param_schema.get("required", True):
                required_list.append(param_name)

        input_schema: dict[str, Any] = {
            "type": "object",
            "properties": properties,
        }
        if required_list:
            input_schema["required"] = required_list

        tool: dict[str, Any] = {
            "name": info.name,
            "description": info.description,
            "inputSchema": input_schema,
        }
        tools.append(tool)

    return _jsonrpc_result(req_id, {"tools": tools})


def _handle_tools_call(req_id: Any, params: dict, skill_registry: "SkillRegistry") -> dict:
    """调用指定 skill，返回结果。"""
    name = params.get("name", "")
    args: dict[str, Any] = params.get("arguments") or {}

    info = skill_registry.get_skill(name)
    if info is None:
        logger.warning(f"MCP tool not found: {name}")
        return _jsonrpc_result_text(req_id, f"工具未找到: {name}")

    logger.info(f"MCP tool call: {name} args={args}")
    t0 = time.monotonic()
    try:
        result = skill_registry.call_skill(name, **args)
    except Exception as e:
        duration = f"{time.monotonic() - t0:.3f}s"
        logger.exception(f"MCP tool error: {name} ({duration})")
        return _jsonrpc_result_text(req_id, f"执行工具 '{name}' 出错: {e}")

    duration = f"{time.monotonic() - t0:.3f}s"
    logger.info(f"MCP tool done: {name} ({duration})")
    return _jsonrpc_result_text(req_id, str(result) if result is not None else "")


def _handle_request(body: dict, skill_registry: "SkillRegistry") -> dict | None:
    """处理单条 JSON-RPC 请求。

    JSON-RPC notifications (无 id) 不需要响应，返回 None。
    """
    method = body.get("method", "")
    params = body.get("params") or {}
    req_id = body.get("id")

    # JSON-RPC notifications 无 id，服务器不应回复
    if "id" not in body:
        return None

    if method == "initialize":
        return _handle_initialize(req_id)
    if method == "tools/list":
        return _handle_tools_list(req_id, skill_registry)
    if method == "tools/call":
        return _handle_tools_call(req_id, params, skill_registry)

    return _jsonrpc_error(req_id, -32601, f"未知方法: {method}")


# ---------------------------------------------------------------------------
# HTTP Request Handler
# ---------------------------------------------------------------------------

class _MCPHandler(BaseHTTPRequestHandler):
    """HTTP 请求处理器，处理 POST /mcp。"""

    # skill_registry 在创建 HTTPServer 后由 MCPServer 注入
    skill_registry: "SkillRegistry"

    def log_message(self, format: str, *args: Any) -> None:
        # 重定向到 Python logging，避免打到 stderr
        logger.debug(f"MCP HTTP: {format % args}")

    def _send_json(self, data: dict, status: int = 200) -> None:
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self) -> None:
        """处理 CORS preflight。"""
        self.send_response(204)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self) -> None:
        if self.path != "/mcp":
            self.send_response(404)
            self.end_headers()
            return

        # 读取请求体
        content_length = int(self.headers.get("Content-Length", 0))
        raw = self.rfile.read(content_length)

        # 解析 JSON
        try:
            body = json.loads(raw)
        except Exception:
            error_resp = {
                "jsonrpc": "2.0",
                "id": None,
                "error": {"code": -32700, "message": "JSON 解析失败"},
            }
            self._send_json(error_resp, status=400)
            return

        # 处理请求
        result = _handle_request(body, self.server.skill_registry)  # type: ignore[attr-defined]
        if result is None:
            # JSON-RPC notification — 无响应体
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            return

        self._send_json(result)


# ---------------------------------------------------------------------------
# MCPServer
# ---------------------------------------------------------------------------

class MCPServer:
    """LingTu MCP Server — 在后台线程中运行，不阻塞 ROS2 spin。

    用法::

        server = MCPServer(skill_registry, port=8090)
        server.start()

        # 在 destroy_node 中:
        server.stop()

    Claude Desktop 配置::

        {
          "mcpServers": {
            "lingtu": {
              "url": "http://192.168.66.190:8090/mcp"
            }
          }
        }
    """

    def __init__(self, skill_registry: "SkillRegistry", port: int = 8090) -> None:
        self._skill_registry = skill_registry
        self._port = port
        self._server: HTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        """在后台守护线程中启动 HTTP 服务。"""
        if self._thread is not None and self._thread.is_alive():
            logger.warning("MCPServer already running")
            return

        # 构造 HTTPServer，将 skill_registry 注入到 server 实例
        server = HTTPServer(("0.0.0.0", self._port), _MCPHandler)
        server.skill_registry = self._skill_registry  # type: ignore[attr-defined]
        self._server = server

        self._thread = threading.Thread(
            target=server.serve_forever,
            daemon=True,
            name="mcp-server",
        )
        self._thread.start()
        logger.info(f"MCPServer started on port {self._port} (POST /mcp)")

    def stop(self) -> None:
        """停止 HTTP 服务。"""
        if self._server is not None:
            self._server.shutdown()
            self._server = None
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        logger.info("MCPServer stopped")
