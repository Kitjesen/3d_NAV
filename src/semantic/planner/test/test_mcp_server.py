"""MCP Server 单元测试 — JSON-RPC 协议，不启动 HTTP server。
不依赖 ROS2，纯 Python 可运行。

运行:
    cd src/semantic_planner && python -m pytest test/test_mcp_server.py -v
"""

import pytest

from semantic_planner.mcp_server import (
    _handle_initialize,
    _handle_request,
    _handle_tools_call,
    _handle_tools_list,
    _jsonrpc_error,
    _jsonrpc_result,
    _jsonrpc_result_text,
)
from semantic_planner.skill_registry import LingTuNavigationSkills, SkillRegistry


# ---------------------------------------------------------------------------
# 辅助: 构建测试用 SkillRegistry
# ---------------------------------------------------------------------------

def _make_registry() -> SkillRegistry:
    registry = SkillRegistry()
    nav = LingTuNavigationSkills()
    registry.register_instance(nav)
    return registry


# ---------------------------------------------------------------------------
# JSON-RPC helpers 测试
# ---------------------------------------------------------------------------

class TestJsonRpcHelpers:
    def test_jsonrpc_result_structure(self):
        resp = _jsonrpc_result(1, {"key": "val"})
        assert resp["jsonrpc"] == "2.0"
        assert resp["id"] == 1
        assert resp["result"] == {"key": "val"}

    def test_jsonrpc_result_text_structure(self):
        resp = _jsonrpc_result_text(2, "hello")
        assert resp["id"] == 2
        content = resp["result"]["content"]
        assert len(content) == 1
        assert content[0]["type"] == "text"
        assert content[0]["text"] == "hello"

    def test_jsonrpc_error_structure(self):
        resp = _jsonrpc_error(3, -32601, "method not found")
        assert resp["jsonrpc"] == "2.0"
        assert resp["id"] == 3
        assert resp["error"]["code"] == -32601
        assert "not found" in resp["error"]["message"]


# ---------------------------------------------------------------------------
# initialize
# ---------------------------------------------------------------------------

class TestInitialize:
    def test_initialize_returns_protocol_version(self):
        resp = _handle_initialize(1)
        assert "protocolVersion" in resp["result"]
        assert resp["result"]["protocolVersion"] == "2025-11-25"

    def test_initialize_returns_capabilities(self):
        resp = _handle_initialize(1)
        assert "capabilities" in resp["result"]
        assert "tools" in resp["result"]["capabilities"]

    def test_initialize_returns_server_info(self):
        resp = _handle_initialize(1)
        assert "serverInfo" in resp["result"]
        assert resp["result"]["serverInfo"]["name"] == "lingtu"

    def test_initialize_preserves_request_id(self):
        resp = _handle_initialize("abc-123")
        assert resp["id"] == "abc-123"


# ---------------------------------------------------------------------------
# tools/list
# ---------------------------------------------------------------------------

class TestToolsList:
    def setup_method(self):
        self.registry = _make_registry()

    def test_tools_list_returns_all_skills(self):
        resp = _handle_tools_list(1, self.registry)
        tools = resp["result"]["tools"]
        assert len(tools) == 11

    def test_tools_list_schema_has_name_and_description(self):
        resp = _handle_tools_list(1, self.registry)
        for tool in resp["result"]["tools"]:
            assert "name" in tool
            assert "description" in tool

    def test_tools_list_schema_has_input_schema(self):
        resp = _handle_tools_list(1, self.registry)
        for tool in resp["result"]["tools"]:
            assert "inputSchema" in tool
            assert tool["inputSchema"]["type"] == "object"

    def test_tools_list_navigate_to_has_required_param(self):
        resp = _handle_tools_list(1, self.registry)
        nav_tool = next(t for t in resp["result"]["tools"] if t["name"] == "navigate_to")
        assert "required" in nav_tool["inputSchema"]
        assert "target" in nav_tool["inputSchema"]["required"]

    def test_tools_list_preserves_request_id(self):
        resp = _handle_tools_list(42, self.registry)
        assert resp["id"] == 42


# ---------------------------------------------------------------------------
# tools/call
# ---------------------------------------------------------------------------

class TestToolsCall:
    def setup_method(self):
        self.registry = _make_registry()

    def test_tools_call_returns_text_content(self):
        params = {"name": "stop", "arguments": {}}
        resp = _handle_tools_call(1, params, self.registry)
        content = resp["result"]["content"]
        assert content[0]["type"] == "text"

    def test_tools_call_navigate_to(self):
        """调用 navigate_to skill，应返回字符串结果（未就绪消息）。"""
        params = {"name": "navigate_to", "arguments": {"target": "体育馆"}}
        resp = _handle_tools_call(1, params, self.registry)
        text = resp["result"]["content"][0]["text"]
        assert isinstance(text, str)
        assert len(text) > 0

    def test_tools_call_with_callback(self):
        """注入回调后调用 skill，应返回回调的结果。"""
        nav = LingTuNavigationSkills()
        nav.set_callbacks(navigate=lambda t: f"前往 {t}")
        registry = SkillRegistry()
        registry.register_instance(nav)

        params = {"name": "navigate_to", "arguments": {"target": "门口"}}
        resp = _handle_tools_call(1, params, registry)
        text = resp["result"]["content"][0]["text"]
        assert "门口" in text

    def test_tool_not_found_returns_error_text(self):
        """调用不存在的 tool → 返回错误文本（不是 JSON-RPC error 对象）。"""
        params = {"name": "nonexistent_tool", "arguments": {}}
        resp = _handle_tools_call(1, params, self.registry)
        text = resp["result"]["content"][0]["text"]
        assert "未找到" in text or "not found" in text.lower()

    def test_tools_call_preserves_request_id(self):
        params = {"name": "stop", "arguments": {}}
        resp = _handle_tools_call(99, params, self.registry)
        assert resp["id"] == 99


# ---------------------------------------------------------------------------
# _handle_request (统一入口)
# ---------------------------------------------------------------------------

class TestHandleRequest:
    def setup_method(self):
        self.registry = _make_registry()

    def test_initialize_via_handle_request(self):
        body = {"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {}}
        resp = _handle_request(body, self.registry)
        assert resp is not None
        assert "protocolVersion" in resp["result"]

    def test_tools_list_via_handle_request(self):
        body = {"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}}
        resp = _handle_request(body, self.registry)
        assert resp is not None
        assert "tools" in resp["result"]

    def test_tools_call_via_handle_request(self):
        body = {
            "jsonrpc": "2.0",
            "id": 3,
            "method": "tools/call",
            "params": {"name": "stop", "arguments": {}},
        }
        resp = _handle_request(body, self.registry)
        assert resp is not None
        assert "content" in resp["result"]

    def test_notification_no_id_returns_none(self):
        """无 id 的请求（JSON-RPC notification）不应返回响应。"""
        body = {"jsonrpc": "2.0", "method": "notifications/initialized", "params": {}}
        resp = _handle_request(body, self.registry)
        assert resp is None

    def test_unknown_method_returns_error_32601(self):
        """未知方法应返回 JSON-RPC 错误码 -32601。"""
        body = {"jsonrpc": "2.0", "id": 5, "method": "unknown/method", "params": {}}
        resp = _handle_request(body, self.registry)
        assert resp is not None
        assert "error" in resp
        assert resp["error"]["code"] == -32601

    def test_unknown_method_preserves_id(self):
        body = {"jsonrpc": "2.0", "id": "xyz", "method": "bad/method"}
        resp = _handle_request(body, self.registry)
        assert resp["id"] == "xyz"
