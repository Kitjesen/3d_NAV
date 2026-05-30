import pytest

from gateway.auth import APIKeyMiddleware


async def _ok_app(scope, receive, send):
    await send({"type": "http.response.start", "status": 200, "headers": []})
    await send({"type": "http.response.body", "body": b"ok"})


async def _request(app, *, headers=None, path="/mcp"):
    sent = []

    async def receive():
        return {"type": "http.request", "body": b"", "more_body": False}

    async def send(message):
        sent.append(message)

    await app(
        {
            "type": "http",
            "path": path,
            "headers": headers or [],
            "query_string": b"",
        },
        receive,
        send,
    )
    return sent


@pytest.mark.asyncio
async def test_api_key_middleware_rejects_missing_key_when_required(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None, require_key=True)

    sent = await _request(app)

    assert sent[0]["status"] == 401


@pytest.mark.asyncio
async def test_api_key_middleware_accepts_valid_key_when_required(monkeypatch):
    monkeypatch.setenv("LINGTU_API_KEY", "secret")
    app = APIKeyMiddleware(_ok_app, require_key=True)

    sent = await _request(app, headers=[(b"x-api-key", b"secret")])

    assert sent[0]["status"] == 200


@pytest.mark.asyncio
async def test_api_key_middleware_rejects_wrong_key_when_required(monkeypatch):
    monkeypatch.setenv("LINGTU_API_KEY", "secret")
    app = APIKeyMiddleware(_ok_app, require_key=True)

    sent = await _request(app, headers=[(b"x-api-key", b"wrong")])

    assert sent[0]["status"] == 403


@pytest.mark.asyncio
async def test_api_key_middleware_keeps_no_key_pass_through_by_default(monkeypatch):
    monkeypatch.delenv("LINGTU_API_KEY", raising=False)
    monkeypatch.setattr("gateway.auth._get_configured_key", lambda: None)
    app = APIKeyMiddleware(_ok_app, api_key=None)

    sent = await _request(app)

    assert sent[0]["status"] == 200


def _mcp_auth_kwargs(app):
    from gateway.auth import APIKeyMiddleware

    for middleware in app.user_middleware:
        if middleware.cls is APIKeyMiddleware:
            return middleware.kwargs
    raise AssertionError("APIKeyMiddleware was not installed")


def test_mcp_server_requires_key_by_default_on_non_localhost(monkeypatch):
    from gateway.mcp_server import MCPServerModule

    captured = {}

    def capture_run(app, **kwargs):
        captured["app"] = app
        captured["kwargs"] = kwargs

    monkeypatch.setattr("uvicorn.run", capture_run)
    MCPServerModule(host="0.0.0.0")._run_server()

    assert _mcp_auth_kwargs(captured["app"])["require_key"] is True


def test_mcp_server_keeps_localhost_dev_pass_through_by_default(monkeypatch):
    from gateway.mcp_server import MCPServerModule

    captured = {}

    def capture_run(app, **kwargs):
        captured["app"] = app
        captured["kwargs"] = kwargs

    monkeypatch.setattr("uvicorn.run", capture_run)
    MCPServerModule(host="127.0.0.1")._run_server()

    assert _mcp_auth_kwargs(captured["app"])["require_key"] is False
