"""Protocol interface for LLM backends.

All LLM clients (OpenAI, Claude, Qwen, Kimi, MockLLMClient) share the
same async ``chat`` interface defined here.
"""

from __future__ import annotations

from typing import Any, List, Protocol, runtime_checkable


@runtime_checkable
class LLMBackend(Protocol):
    """LLM backend interface.

    Implementations: OpenAIClient, ClaudeClient, QwenClient, MockLLMClient.
    The single required method mirrors the OpenAI chat-completions style
    used throughout the semantic planner.
    """

    async def chat(self, messages: list, **kwargs: Any) -> str:
        """Send a chat request and return the assistant's reply text.

        Args:
            messages: OpenAI-format message list
                      ``[{"role": "system", "content": "..."}, ...]``
            **kwargs: Backend-specific overrides (temperature, etc.)

        Returns:
            The assistant reply as a plain string.
        """
        ...
