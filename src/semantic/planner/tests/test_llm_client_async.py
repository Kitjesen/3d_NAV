"""
test_llm_client_async.py — LLM 客户端异步调用回归测试 (Round 14)

验证 asyncio 事件循环使用的正确性，防止 DeprecationWarning 和死锁。
无需 ROS2 环境，无需真实 API Key。

覆盖:
  - asyncio 事件循环中调用 chat，无 DeprecationWarning
  - ThreadPoolExecutor 线程中调用，get_running_loop() 正确工作
  - LLM 超时返回 None/raise（不崩溃）
  - backend='mock' 模式直接返回固定响应
"""

import asyncio
import concurrent.futures
import json
import sys
import unittest
import warnings
from unittest.mock import MagicMock, patch

# semantic_planner 包路径
sys.path.insert(0, "D:/inovxio/brain/lingtu/src/semantic_planner")

from semantic.planner.semantic_planner.llm_client import (
    LLMConfig,
    LLMError,
    MockLLMClient,
    create_llm_client,
)


class TestMockBackendDirect(unittest.TestCase):
    """backend='mock' 模式基础测试。"""

    def test_mock_client_creation(self):
        """create_llm_client(backend='mock') 正确返回 MockLLMClient。"""
        config = LLMConfig(backend="mock", model="mock-model")
        client = create_llm_client(config)
        self.assertIsInstance(client, MockLLMClient)

    def test_mock_client_is_available(self):
        """MockLLMClient.is_available() 总是返回 True。"""
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)
        self.assertTrue(client.is_available())

    def test_mock_client_returns_valid_json(self):
        """MockLLMClient.chat() 返回合法 JSON 字符串。"""
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)

        messages = [
            {"role": "system", "content": "You are a navigation assistant."},
            {"role": "user", "content": "找到椅子"},
        ]

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(client.chat(messages))
        finally:
            loop.close()

        self.assertIsInstance(result, str)
        parsed = json.loads(result)
        self.assertIn("action", parsed)
        self.assertIn("confidence", parsed)

    def test_mock_client_aliases(self):
        """'offline' 和 'test' 别名也映射到 MockLLMClient。"""
        for alias in ("mock", "offline", "test"):
            config = LLMConfig(backend=alias)
            client = create_llm_client(config)
            self.assertIsInstance(client, MockLLMClient, f"alias '{alias}' failed")


class TestAsyncEventLoop(unittest.TestCase):
    """asyncio 事件循环正确性测试。"""

    def test_chat_in_new_event_loop_no_deprecation(self):
        """在新建 event loop 中调用 chat，不应产生 DeprecationWarning。
        修复前使用 asyncio.get_event_loop() 可能触发 DeprecationWarning (Python 3.10+)。
        """
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)
        messages = [{"role": "user", "content": "go to kitchen"}]

        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            loop = asyncio.new_event_loop()
            try:
                result = loop.run_until_complete(client.chat(messages))
            finally:
                loop.close()

            deprecation_warnings = [
                x for x in w
                if issubclass(x.category, DeprecationWarning)
                and "event loop" in str(x.message).lower()
            ]
            self.assertEqual(
                len(deprecation_warnings), 0,
                f"Got unexpected DeprecationWarning: {deprecation_warnings}"
            )
        self.assertIsNotNone(result)

    def test_chat_in_threadpool_executor(self):
        """在 ThreadPoolExecutor 线程中调用 chat（模拟 ROS2 回调线程场景）。
        验证在没有预设 event loop 的工作线程中也能正常运行。
        """
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)
        messages = [{"role": "user", "content": "find the table"}]

        def _thread_call():
            loop = asyncio.new_event_loop()
            try:
                return loop.run_until_complete(client.chat(messages))
            finally:
                loop.close()

        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(_thread_call)
            result = future.result(timeout=5.0)

        self.assertIsNotNone(result)
        parsed = json.loads(result)
        self.assertIn("action", parsed)

    def test_multiple_concurrent_calls(self):
        """多个并发 chat 调用不会相互干扰。"""
        config = LLMConfig(backend="mock")
        client = MockLLMClient(config)

        async def _multi_call():
            tasks = [
                client.chat([{"role": "user", "content": f"go to room {i}"}])
                for i in range(5)
            ]
            return await asyncio.gather(*tasks)

        loop = asyncio.new_event_loop()
        try:
            results = loop.run_until_complete(_multi_call())
        finally:
            loop.close()

        self.assertEqual(len(results), 5)
        for r in results:
            parsed = json.loads(r)
            self.assertIn("action", parsed)


class TestLLMTimeout(unittest.TestCase):
    """LLM 超时处理测试。"""

    def test_timeout_raises_llm_error(self):
        """模拟 LLM 超时场景，验证不会崩溃而是抛出 LLMError。"""
        config = LLMConfig(backend="openai", timeout_sec=0.001, max_retries=0)

        # Mock OpenAI 客户端使其超时
        with patch.dict("sys.modules", {"openai": MagicMock()}):
            from semantic.planner.semantic_planner.llm_client import OpenAIClient
            client = OpenAIClient(config)

            # 模拟 _client.chat.completions.create 抛超时
            mock_openai = MagicMock()

            async def _raise_timeout(**kw):
                raise TimeoutError("Connection timed out")

            mock_openai.chat.completions.create = _raise_timeout
            client._client = mock_openai

            loop = asyncio.new_event_loop()
            try:
                with self.assertRaises(LLMError) as ctx:
                    loop.run_until_complete(
                        client.chat([{"role": "user", "content": "hello"}])
                    )
                self.assertIn("failed", str(ctx.exception).lower())
            finally:
                loop.close()

    def test_mock_client_never_times_out(self):
        """MockLLMClient 不会超时（即使设置极短超时也能返回）。"""
        config = LLMConfig(backend="mock", timeout_sec=0.001)
        client = MockLLMClient(config)

        loop = asyncio.new_event_loop()
        try:
            result = loop.run_until_complete(
                client.chat([{"role": "user", "content": "find bed"}])
            )
        finally:
            loop.close()

        self.assertIsNotNone(result)
        self.assertIn("action", json.loads(result))


if __name__ == "__main__":
    unittest.main()
