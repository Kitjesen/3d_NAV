"""
conftest.py — semantic_planner 测试共享夹具

提供:
  - 标准场景图 JSON 字符串工厂
  - LLM mock 工厂
  - GoalResolver 工厂 (带 mock LLM)
  - 基础 CLIP 特征工厂
"""

import asyncio
import json
import unittest.mock as mock

import numpy as np
import pytest


# ─────────────────────────────────────────────────────────────────────────────
#  CLIP 特征工厂
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def make_clip_feature():
    """返回归一化 512-dim CLIP 特征工厂函数。"""
    def _factory(seed=None):
        rng = np.random.RandomState(seed)
        f = rng.rand(512).astype(np.float32)
        return f / np.linalg.norm(f)
    return _factory


# ─────────────────────────────────────────────────────────────────────────────
#  场景图 JSON 工厂
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def make_scene_graph_json(make_clip_feature):
    """返回场景图 JSON 字符串工厂。"""
    def _factory(objects=None, relations=None, regions=None):
        if objects is None:
            objects = [
                {
                    "id": "obj_001",
                    "label": "chair",
                    "position": [1.0, 2.0, 0.5],
                    "confidence": 0.95,
                    "clip_feature": make_clip_feature(seed=1).tolist(),
                },
                {
                    "id": "obj_002",
                    "label": "desk",
                    "position": [3.0, 2.0, 0.8],
                    "confidence": 0.88,
                    "clip_feature": make_clip_feature(seed=2).tolist(),
                },
                {
                    "id": "obj_003",
                    "label": "fire extinguisher",
                    "position": [0.5, 5.0, 1.2],
                    "confidence": 0.92,
                    "clip_feature": make_clip_feature(seed=3).tolist(),
                },
            ]
        return json.dumps({
            "objects": objects,
            "relations": relations or [],
            "regions": regions or [
                {"name": "office", "object_ids": ["obj_001", "obj_002"]},
                {"name": "corridor", "object_ids": ["obj_003"]},
            ],
        })

    return _factory


# ─────────────────────────────────────────────────────────────────────────────
#  Mock LLM 工厂
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def mock_llm():
    """返回 AsyncMock LLM 客户端。"""
    llm = mock.MagicMock()
    llm.chat = mock.AsyncMock(return_value=json.dumps({
        "object_id": "obj_001",
        "label": "chair",
        "confidence": 0.85,
        "reasoning": "目标是椅子，直接匹配",
    }))
    return llm


@pytest.fixture
def mock_llm_no_result():
    """返回 LLM 客户端，其 chat 结果为找不到目标。"""
    llm = mock.MagicMock()
    llm.chat = mock.AsyncMock(return_value=json.dumps({
        "object_id": None,
        "label": None,
        "confidence": 0.0,
        "reasoning": "未找到目标物体",
    }))
    return llm


@pytest.fixture
def mock_llm_error():
    """返回 LLM 客户端，其 chat 总是抛出异常。"""
    from semantic_planner.llm_client import LLMError
    llm = mock.MagicMock()
    llm.chat = mock.AsyncMock(side_effect=LLMError("Mock API error"))
    return llm


# ─────────────────────────────────────────────────────────────────────────────
#  Event loop (for async tests)
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def event_loop():
    """Provide a fresh event loop for async tests."""
    loop = asyncio.new_event_loop()
    yield loop
    loop.close()


# ─────────────────────────────────────────────────────────────────────────────
#  GoalResolver 工厂
# ─────────────────────────────────────────────────────────────────────────────

@pytest.fixture
def make_goal_resolver(mock_llm):
    """返回预配置好 mock LLM 的 GoalResolver 工厂。"""
    def _factory(**kwargs):
        try:
            from semantic_planner.goal_resolver import GoalResolver
            resolver = GoalResolver(**kwargs)
            resolver._primary = mock_llm
            return resolver
        except Exception:
            return None

    return _factory
