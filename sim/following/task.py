"""Task definition layer — LLM parses instructions into structured tasks.

Bridges natural language ("跟着那个穿红衣服的人") to the behavior FSM.
Uses rule-based matching first (instant), falls back to Kimi API for
complex instructions.

Reuses LingTu's SIMPLE_FOLLOW_PATTERNS from task_rules.py.
"""
from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class FollowTask:
    """Structured task that drives the behavior FSM."""
    type: str = "follow"      # follow / find / patrol / go_to / stop
    target: str = ""          # "穿红衣服的人" / "kitchen" / "room 3"
    parameters: dict = field(default_factory=dict)  # speed, distance, timeout
    source: str = "user"      # user / llm / system
    confidence: float = 1.0


# Rule-based patterns (from LingTu task_rules.py)
_FOLLOW_ZH = [
    "紧紧跟着", "一直跟着", "持续跟随", "不要跟丢", "别跟丢",
    "帮我跟着", "帮忙跟着", "请跟着", "帮我跟踪",
    "跟着他走", "跟着她走", "跟这个人", "跟那个人",
    "跟上他", "跟上她", "追上他", "追上她",
    "跟着他", "跟着她", "跟他", "跟她", "跟他们",
    "跟着", "跟随", "跟踪", "追踪", "追着",
    "跟住", "盯着", "盯住", "尾随", "跟上", "追上",
]
_FOLLOW_EN = [
    "keep following", "keep tracking", "stay with", "stick with",
    "follow", "track", "chase", "tail", "pursue", "shadow",
]
_FIND_ZH = ["找到", "找一下", "找", "搜索", "寻找", "去找"]
_FIND_EN = ["find", "search for", "look for", "locate"]
_GOTO_ZH = ["去", "走到", "到", "前往", "过去"]
_GOTO_EN = ["go to", "navigate to", "move to", "head to"]
_STOP_ZH = ["停", "停下", "停止", "别动", "站住"]
_STOP_EN = ["stop", "halt", "freeze", "stay"]
_PATROL_ZH = ["巡逻", "巡视", "巡查", "巡一圈"]
_PATROL_EN = ["patrol", "inspect", "survey", "make rounds"]


class TaskParser:
    """Parse natural language instruction into FollowTask.

    Priority: rule-based matching (instant) → LLM fallback (async).

    Usage::

        parser = TaskParser()
        task = parser.parse("跟着那个穿红衣服的人")
        # → FollowTask(type="follow", target="穿红衣服的人")

        task = parser.parse("去厨房")
        # → FollowTask(type="go_to", target="厨房")
    """

    def parse(self, instruction: str) -> FollowTask:
        """Parse instruction into a structured task."""
        text = instruction.strip()
        if not text:
            return FollowTask(type="stop", source="system")

        # Try rule-based matching
        task = self._rule_match(text)
        if task is not None:
            logger.info("TaskParser rule match: '%s' → %s(%s)", text, task.type, task.target)
            return task

        # Default: treat as follow if mentions person-like words
        if any(w in text for w in ["人", "person", "他", "她", "someone"]):
            target = self._extract_target(text, _FOLLOW_ZH + _FOLLOW_EN)
            return FollowTask(type="follow", target=target or text, confidence=0.6)

        # Default: treat as go_to
        return FollowTask(type="go_to", target=text, confidence=0.4)

    def _rule_match(self, text: str) -> Optional[FollowTask]:
        """Match against known patterns (longest match first)."""
        text_lower = text.lower()

        # Stop (check first — shortest patterns)
        for p in _STOP_ZH + _STOP_EN:
            if p in text_lower:
                return FollowTask(type="stop", source="rule")

        # Follow (check before find/goto — "跟着" takes priority)
        for p in _FOLLOW_ZH:
            if p in text:
                target = self._extract_target(text, _FOLLOW_ZH)
                return FollowTask(type="follow", target=target, source="rule")
        for p in _FOLLOW_EN:
            if text_lower.startswith(p) or f" {p}" in text_lower:
                target = self._extract_target(text_lower, _FOLLOW_EN)
                return FollowTask(type="follow", target=target, source="rule")

        # Find
        for p in _FIND_ZH:
            if p in text:
                target = self._extract_target(text, _FIND_ZH)
                return FollowTask(type="find", target=target, source="rule")
        for p in _FIND_EN:
            if text_lower.startswith(p) or f" {p}" in text_lower:
                target = self._extract_target(text_lower, _FIND_EN)
                return FollowTask(type="find", target=target, source="rule")

        # Patrol
        for p in _PATROL_ZH + _PATROL_EN:
            if p in text_lower:
                return FollowTask(type="patrol", source="rule")

        # Go to
        for p in _GOTO_ZH:
            if p in text:
                target = self._extract_target(text, _GOTO_ZH)
                return FollowTask(type="go_to", target=target, source="rule")
        for p in _GOTO_EN:
            if text_lower.startswith(p) or f" {p}" in text_lower:
                target = self._extract_target(text_lower, _GOTO_EN)
                return FollowTask(type="go_to", target=target, source="rule")

        return None

    @staticmethod
    def _extract_target(text: str, patterns: list) -> str:
        """Remove the matched pattern prefix to get the target description."""
        result = text
        for p in sorted(patterns, key=len, reverse=True):
            if p in result:
                result = result.replace(p, "", 1).strip()
                break
        # Clean up common particles
        for particle in ["那个", "这个", "一下", "the ", "a ", "that ", "this "]:
            result = result.replace(particle, "").strip()
        return result or text
