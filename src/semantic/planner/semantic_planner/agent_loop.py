"""AgentLoop — multi-turn observe→think→act cycle with LLM tool calling.

Upgrades SemanticPlannerModule from single-shot resolve to iterative agent:
  1. Observe: gather scene_graph + odometry + memory context + camera image
  2. Think:   LLM decides next action via tool calling
  3. Act:     execute tool (navigate_to, detect_object, query_memory, tag_location,
              describe_scene, assess_situation)
  4. Repeat:  until task complete, max_steps reached, or abort

Tools exposed to LLM:
  navigate_to(x, y)             — publish goal_pose
  navigate_to_object(label)     — Fast Path resolve
  detect_object(label)          — check scene_graph for object
  query_memory(text)            — vector search past locations
  tag_location(name)            — save current position
  say(text)                     — speak to user
  done(summary)                 — mark task complete
  describe_scene()              — VLM: describe current camera view
  assess_situation(goal)        — VLM: does current view help reach goal?

Usage (called by SemanticPlannerModule._on_instruction):
  loop = AgentLoop(llm_client, tools, publish_fns)
  await loop.run("找到红色椅子然后标记位置")
"""

from __future__ import annotations

import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)

# Tool definitions for LLM (OpenAI function-calling format)
AGENT_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "navigate_to",
            "description": "Navigate to a specific coordinate (x, y in meters).",
            "parameters": {
                "type": "object",
                "properties": {
                    "x": {"type": "number", "description": "X coordinate in meters"},
                    "y": {"type": "number", "description": "Y coordinate in meters"},
                },
                "required": ["x", "y"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "navigate_to_object",
            "description": "Navigate to a visible object by its label (e.g. 'red chair', 'door').",
            "parameters": {
                "type": "object",
                "properties": {
                    "label": {"type": "string", "description": "Object label to find and navigate to"},
                },
                "required": ["label"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "detect_object",
            "description": "Check if an object is currently visible in the scene.",
            "parameters": {
                "type": "object",
                "properties": {
                    "label": {"type": "string", "description": "Object label to look for"},
                },
                "required": ["label"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "query_memory",
            "description": "Search past experience for a location matching a description.",
            "parameters": {
                "type": "object",
                "properties": {
                    "text": {"type": "string", "description": "Natural language location description"},
                },
                "required": ["text"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "tag_location",
            "description": "Save the current robot position with a name for later navigation.",
            "parameters": {
                "type": "object",
                "properties": {
                    "name": {"type": "string", "description": "Name for this location"},
                },
                "required": ["name"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "say",
            "description": "Speak a message to the user.",
            "parameters": {
                "type": "object",
                "properties": {
                    "text": {"type": "string", "description": "Message to say"},
                },
                "required": ["text"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "done",
            "description": "Mark the task as complete with a summary.",
            "parameters": {
                "type": "object",
                "properties": {
                    "summary": {"type": "string", "description": "Task completion summary"},
                },
                "required": ["summary"],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "describe_scene",
            "description": (
                "Ask the VLM to describe what the robot camera currently sees. "
                "Use this when you need to understand the visual environment — "
                "e.g. 'what room am I in?', 'is there a path ahead?'."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "context": {
                        "type": "string",
                        "description": "Optional hint about what the robot is trying to do",
                    },
                },
                "required": [],
            },
        },
    },
    {
        "type": "function",
        "function": {
            "name": "assess_situation",
            "description": (
                "Ask the VLM whether what the robot currently sees is useful for "
                "reaching a specific goal. Returns whether the view is relevant, "
                "a suggestion for the next action, and visible obstacles."
            ),
            "parameters": {
                "type": "object",
                "properties": {
                    "goal": {
                        "type": "string",
                        "description": "The navigation goal to assess against the current view",
                    },
                },
                "required": ["goal"],
            },
        },
    },
]

AGENT_SYSTEM_PROMPT = """You are a navigation robot assistant. You execute tasks by calling tools in sequence.

Available information:
- Current position: ({robot_x:.1f}, {robot_y:.1f})
- Visible objects: {visible_objects}
- Navigation status: {nav_status}
- Memory context: {memory_context}
- Camera available: {camera_available}

Rules:
- Break complex tasks into steps and execute them one at a time.
- Use detect_object to check if something is visible before navigating to it.
- Use query_memory for fuzzy location requests ("去上次放背包的地方").
- Use navigate_to_object for objects currently in view.
- Use navigate_to for known coordinates.
- Use describe_scene() when you need to understand what the robot currently sees.
- Use assess_situation(goal) when you are unsure whether the current view is helpful.
- Call done() when the task is complete.
- If you cannot complete the task after several attempts, call done() with an explanation.
"""


@dataclass
class AgentState:
    """Tracks agent loop execution state."""
    instruction: str = ""
    step: int = 0
    max_steps: int = 10
    messages: List[dict] = field(default_factory=list)
    completed: bool = False
    summary: str = ""
    start_time: float = 0.0


class AgentLoop:
    """Multi-turn LLM tool-calling agent for complex navigation tasks.

    Each step: build context → LLM call with tools → parse tool call → execute → feedback.
    Loops until done() called, max_steps, or timeout.
    """

    def __init__(
        self,
        llm_client,
        tool_handlers: Dict[str, Callable],
        context_fn: Callable[[], dict],
        max_steps: int = 10,
        timeout: float = 120.0,
    ):
        """
        Args:
            llm_client: LLM client with chat() method (supports tool calling)
            tool_handlers: {tool_name: handler_fn} — each returns a result string
            context_fn: returns {robot_x, robot_y, visible_objects, nav_status,
                        memory_context, camera_image (np.ndarray or None),
                        scene_graph (dict or None), camera_available (bool)}
            max_steps: max iterations before forced stop
            timeout: max wall-clock seconds
        """
        self._llm = llm_client
        self._handlers = tool_handlers
        self._context_fn = context_fn
        self._max_steps = max_steps
        self._timeout = timeout
        self._vlm_agent = None  # lazy-initialized on first VLM tool call

    async def run(self, instruction: str) -> AgentState:
        """Execute the agent loop for a given instruction."""
        state = AgentState(
            instruction=instruction,
            max_steps=self._max_steps,
            start_time=time.time(),
        )

        # Initial system + user message
        ctx = self._context_fn()
        # Ensure new keys have defaults for backward-compat with older context_fn
        ctx.setdefault("camera_available", ctx.get("camera_image") is not None)
        system_msg = AGENT_SYSTEM_PROMPT.format(**{
            k: v for k, v in ctx.items() if k in (
                "robot_x", "robot_y", "visible_objects",
                "nav_status", "memory_context", "camera_available",
            )
        })
        state.messages = [
            {"role": "system", "content": system_msg},
            {"role": "user", "content": instruction},
        ]

        while state.step < state.max_steps and not state.completed:
            if time.time() - state.start_time > self._timeout:
                state.summary = f"Timeout after {self._timeout}s"
                state.completed = True
                break

            state.step += 1

            # LLM call with tools
            try:
                response = await self._llm_call(state.messages)
            except Exception as e:
                logger.error("AgentLoop: LLM call failed at step %d: %s", state.step, e)
                state.summary = f"LLM error: {e}"
                state.completed = True
                break

            # Parse response
            if response.get("tool_calls"):
                for tc in response["tool_calls"]:
                    result = await self._execute_tool(tc, state)
                    if state.completed:
                        break
            elif response.get("content"):
                # LLM responded with text instead of tool call — treat as thinking
                state.messages.append({"role": "assistant", "content": response["content"]})
                logger.debug("AgentLoop step %d: LLM text: %s", state.step, response["content"][:100])
            else:
                state.summary = "LLM returned empty response"
                state.completed = True

        if not state.completed:
            state.summary = f"Max steps ({self._max_steps}) reached"
            state.completed = True

        logger.info("AgentLoop: '%s' → %d steps, summary: %s",
                     instruction, state.step, state.summary)
        return state

    async def _llm_call(self, messages: List[dict]) -> dict:
        """Call LLM with tool definitions. Returns parsed response."""
        # Try tool-calling format (OpenAI-compatible)
        if hasattr(self._llm, "chat_with_tools"):
            return await self._llm.chat_with_tools(messages, tools=AGENT_TOOLS)

        # Fallback: regular chat with tool descriptions in prompt
        tools_desc = "\n".join(
            f"- {t['function']['name']}: {t['function']['description']}"
            for t in AGENT_TOOLS
        )
        augmented = messages.copy()
        augmented[0] = {
            "role": "system",
            "content": messages[0]["content"] + f"\n\nAvailable tools:\n{tools_desc}\n\n"
                       "Respond with JSON: {\"tool\": \"name\", \"args\": {...}} or "
                       "{\"tool\": \"done\", \"args\": {\"summary\": \"...\"}}"
        }

        text = await self._llm.chat(
            [m["content"] for m in augmented if m["role"] != "system"][-1],
            system_prompt=augmented[0]["content"],
        )

        # Parse JSON tool call from text
        return self._parse_text_tool_call(text)

    @staticmethod
    def _parse_text_tool_call(text: str) -> dict:
        """Extract tool call from LLM text response."""
        # Try to find JSON with "tool" key — scan for balanced braces
        start = text.find('{"tool"')
        if start < 0:
            start = text.find('"tool"')
            if start > 0:
                start = text.rfind('{', 0, start)
        if start >= 0:
            depth = 0
            for i in range(start, len(text)):
                if text[i] == '{':
                    depth += 1
                elif text[i] == '}':
                    depth -= 1
                    if depth == 0:
                        try:
                            data = json.loads(text[start:i + 1])
                            tool_name = data.get("tool", "")
                            args = data.get("args", {})
                            if tool_name:
                                return {
                                    "tool_calls": [{
                                        "function": {"name": tool_name,
                                                     "arguments": json.dumps(args)},
                                        "id": f"call_{time.time_ns()}",
                                    }],
                                }
                        except json.JSONDecodeError:
                            pass
                        break
        return {"content": text}

    async def _execute_tool(self, tool_call: dict, state: AgentState) -> str:
        """Execute a single tool call and append results to message history."""
        fn = tool_call.get("function", {})
        name = fn.get("name", "")
        try:
            args = json.loads(fn.get("arguments", "{}"))
        except json.JSONDecodeError:
            args = {}

        # Append assistant tool call
        state.messages.append({
            "role": "assistant",
            "content": None,
            "tool_calls": [tool_call],
        })

        # Handle done() specially
        if name == "done":
            state.completed = True
            state.summary = args.get("summary", "Task complete")
            result = state.summary
        elif name in ("describe_scene", "assess_situation"):
            # VLM tools — handled internally using the VLMSceneAgent
            result = await self._execute_vlm_tool(name, args)
        elif name in self._handlers:
            try:
                handler = self._handlers[name]
                if asyncio.iscoroutinefunction(handler):
                    result = await handler(**args)
                else:
                    result = handler(**args)
                result = str(result) if result is not None else "OK"
            except Exception as e:
                result = f"Error: {e}"
                logger.warning("AgentLoop: tool '%s' failed: %s", name, e)
        else:
            result = f"Unknown tool: {name}"

        # Append tool result
        state.messages.append({
            "role": "tool",
            "tool_call_id": tool_call.get("id", ""),
            "content": result,
        })

        # Update context for next step
        ctx = self._context_fn()
        state.messages.append({
            "role": "system",
            "content": f"Updated state: position=({ctx['robot_x']:.1f},{ctx['robot_y']:.1f}), "
                       f"nav_status={ctx['nav_status']}, visible={ctx['visible_objects'][:100]}",
        })

        logger.info("AgentLoop step %d: %s(%s) → %s",
                     state.step, name, args, result[:100])
        return result

    def _get_vlm_agent(self):
        """Return the VLMSceneAgent, creating it lazily on first use."""
        if self._vlm_agent is None:
            try:
                from .vlm_scene_agent import VLMSceneAgent
                self._vlm_agent = VLMSceneAgent(self._llm)
                logger.info("AgentLoop: VLMSceneAgent initialized (backend=%s)",
                            type(self._llm).__name__)
            except Exception as e:
                logger.warning("AgentLoop: VLMSceneAgent init failed: %s", e)
        return self._vlm_agent

    async def _execute_vlm_tool(self, name: str, args: dict) -> str:
        """Execute a VLM scene-understanding tool.

        Retrieves the latest camera frame from context_fn and dispatches to
        VLMSceneAgent. Returns a plain string suitable for appending to the
        agent's message history.
        """
        vlm = self._get_vlm_agent()
        if vlm is None:
            return "VLM scene understanding not available"

        # Pull the latest context to get the current camera frame and scene graph
        ctx = self._context_fn()
        camera_image = ctx.get("camera_image")  # np.ndarray or None
        scene_graph = ctx.get("scene_graph")    # dict or None

        try:
            if name == "describe_scene":
                context_hint = args.get("context", "")
                return await vlm.describe_scene(camera_image, context=context_hint)

            elif name == "assess_situation":
                goal = args.get("goal", "")
                result = await vlm.assess_situation(camera_image, goal, scene_graph)
                # Format the structured result as a readable string for the agent
                lines = [f"Relevant to goal: {result.get('relevant', False)}"]
                suggestion = result.get("suggestion", "")
                if suggestion:
                    lines.append(f"Suggestion: {suggestion}")
                obstacles = result.get("obstacles", [])
                if obstacles:
                    lines.append(f"Obstacles: {', '.join(str(o) for o in obstacles)}")
                return "\n".join(lines)

        except Exception as e:
            logger.warning("AgentLoop: VLM tool '%s' failed: %s", name, e)
            return f"VLM tool error: {e}"

        return f"Unknown VLM tool: {name}"


# Need asyncio import at module level for iscoroutinefunction check
import asyncio
