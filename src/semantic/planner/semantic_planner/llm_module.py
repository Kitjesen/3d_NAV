"""LLMModule — pluggable LLM backend as an independent Module.

Wraps any LLMClientBase (OpenAI/Kimi/Claude/Qwen/Mock) into a core Module.
Other modules send prompts via the ``request`` port, receive responses via
the ``response`` port. Swapping LLM = changing one Blueprint argument.

Usage::

    # Kimi (default, China-direct)
    bp.add(LLMModule, backend="kimi", model="kimi-k2.5")

    # Claude
    bp.add(LLMModule, backend="claude", model="claude-3-5-sonnet-20241022")

    # Mock (CI/testing, no API key needed)
    bp.add(LLMModule, backend="mock")

    # Downstream modules wire to it:
    bp.wire("GoalResolverModule", "llm_request", "LLMModule", "request")
    bp.wire("LLMModule", "response", "GoalResolverModule", "llm_response")
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

from core.module import Module
from core.stream import In, Out
from core.registry import register

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Request / Response types
# ---------------------------------------------------------------------------

@dataclass
class LLMRequest:
    """LLM request — prompt + metadata."""
    messages: list          # OpenAI format: [{"role": "system", "content": "..."}]
    request_id: str = ""    # caller-assigned ID for routing responses
    temperature: float = 0.2
    caller: str = ""        # which module sent this

    @staticmethod
    def simple(prompt: str, system: str = "", request_id: str = "", caller: str = ""):
        """Convenience: single user prompt."""
        msgs = []
        if system:
            msgs.append({"role": "system", "content": system})
        msgs.append({"role": "user", "content": prompt})
        return LLMRequest(messages=msgs, request_id=request_id, caller=caller)


@dataclass
class LLMResponse:
    """LLM response — text + metadata."""
    text: str
    request_id: str = ""
    model: str = ""
    latency_ms: float = 0.0
    error: str = ""         # non-empty if failed

    @property
    def ok(self) -> bool:
        return not self.error


# ---------------------------------------------------------------------------
# LLMModule
# ---------------------------------------------------------------------------

@register("llm", "pluggable", description="Pluggable LLM backend module")
class LLMModule(Module, layer=4):
    """Pluggable LLM Module — async chat via In/Out ports.

    In:  request  (LLMRequest)   — prompt from any module
    Out: response (LLMResponse)  — LLM reply routed back

    The async event loop runs in a background thread so LLM calls
    don't block the Module system's synchronous callback chain.
    """

    request: In[LLMRequest]
    response: Out[LLMResponse]

    def __init__(
        self,
        backend: str = "kimi",
        model: str = "",
        api_key_env: str = "",
        timeout_sec: float = 10.0,
        temperature: float = 0.2,
        base_url: str = "",
        **kw,
    ):
        super().__init__(**kw)
        self._backend_name = backend
        self._model = model
        self._api_key_env = api_key_env
        self._timeout_sec = timeout_sec
        self._temperature = temperature
        self._base_url = base_url
        self._client = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._loop_thread: Optional[threading.Thread] = None
        self._call_count = 0
        self._total_latency_ms = 0.0
        self._error_count = 0

    def setup(self):
        """Create LLM client and start async event loop."""
        self._client = self._create_client()
        self.request.subscribe(self._on_request)

        # Start async loop in background thread for non-blocking LLM calls
        self._loop = asyncio.new_event_loop()
        self._loop_thread = threading.Thread(
            target=self._loop.run_forever, daemon=True,
            name=f"llm-{self._backend_name}")
        self._loop_thread.start()
        logger.info("LLMModule: backend='%s' model='%s'",
                     self._backend_name, self._model)

    def _create_client(self):
        """Factory: instantiate the selected LLM backend."""
        from .llm_client import LLMConfig, create_llm_client

        # Resolve defaults per backend
        defaults = {
            "kimi":    {"model": "kimi-k2.5", "api_key_env": "MOONSHOT_API_KEY",
                        "base_url": "https://api.moonshot.cn/v1"},
            "openai":  {"model": "gpt-4o-mini", "api_key_env": "OPENAI_API_KEY"},
            "claude":  {"model": "claude-3-5-sonnet-20241022", "api_key_env": "ANTHROPIC_API_KEY"},
            "qwen":    {"model": "qwen-turbo", "api_key_env": "DASHSCOPE_API_KEY"},
            "mock":    {"model": "mock", "api_key_env": ""},
        }
        d = defaults.get(self._backend_name, {})

        config = LLMConfig(
            backend=self._backend_name,
            model=self._model or d.get("model", ""),
            api_key_env=self._api_key_env or d.get("api_key_env", ""),
            timeout_sec=self._timeout_sec,
            temperature=self._temperature,
            base_url=self._base_url or d.get("base_url", ""),
        )
        return create_llm_client(config)

    def _on_request(self, req: LLMRequest):
        """Dispatch LLM call to async loop, publish response when done."""
        if self._client is None or self._loop is None:
            self.response.publish(LLMResponse(
                text="", request_id=req.request_id,
                error="LLM client not initialized"))
            return

        future = asyncio.run_coroutine_threadsafe(
            self._async_chat(req), self._loop)
        # Non-blocking: response published from async callback
        future.add_done_callback(lambda f: self._handle_result(f, req))

    async def _async_chat(self, req: LLMRequest) -> str:
        """Run the actual LLM call."""
        return await self._client.chat(
            req.messages,
            temperature=req.temperature,
        )

    def _handle_result(self, future, req: LLMRequest):
        """Callback when async LLM call completes."""
        t0 = time.time()
        try:
            text = future.result(timeout=self._timeout_sec + 5)
            latency_ms = (time.time() - t0) * 1000
            self._call_count += 1
            self._total_latency_ms += latency_ms
            self.response.publish(LLMResponse(
                text=text,
                request_id=req.request_id,
                model=self._model,
                latency_ms=latency_ms,
            ))
        except Exception as e:
            self._error_count += 1
            logger.error("LLMModule: call failed: %s", e)
            self.response.publish(LLMResponse(
                text="",
                request_id=req.request_id,
                model=self._model,
                error=str(e),
            ))

    def stop(self):
        """Shutdown async loop and client."""
        loop = self._loop
        if loop is not None:
            loop.call_soon_threadsafe(loop.stop)
        if self._loop_thread:
            self._loop_thread.join(timeout=3.0)
        if loop is not None:
            try:
                loop.close()
            except Exception:
                pass
        self._loop = None
        self._loop_thread = None
        super().stop()

    def health(self) -> Dict[str, Any]:
        info = super().port_summary()
        avg_ms = (self._total_latency_ms / self._call_count
                  if self._call_count > 0 else 0.0)
        info["llm"] = {
            "backend": self._backend_name,
            "model": self._model,
            "calls": self._call_count,
            "errors": self._error_count,
            "avg_ms": round(avg_ms, 1),
        }
        return info
