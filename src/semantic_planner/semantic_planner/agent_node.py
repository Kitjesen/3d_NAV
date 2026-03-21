"""
LingTu Agent Node — LLM 作为机器人的决策大脑

借鉴 DimOS Agent 架构，在 ROS2 上实现 LangChain Agent。
Agent 通过 ROS2 话题调用机器人的各项 Skill。

架构:
  用户语音/文本 → /nav/agent/input → Agent Node
    → LLM (GPT-4o / Kimi) 决策
    → 调用 Skill (ROS2 话题)
    → 结果反馈 → /nav/agent/output

订阅:
  /nav/agent/input          (String)     用户输入 (语音转文字或直接文本)
  /nav/odometry             (Odometry)   机器人位姿
  /nav/semantic/scene_graph  (String)     场景图 (环境感知)
  /nav/dialogue_state        (String)     对话状态
  /nav/planner_status        (String)     规划器状态 (GOAL_REACHED / STUCK 等)

发布:
  /nav/agent/output         (String JSON) Agent 回复 + 执行的动作
  /nav/semantic/instruction  (String)     发给 planner 的导航指令
  /nav/semantic/tag_location (String JSON) 标记地点
  /nav/semantic/bbox_navigate (String JSON) BBox 导航
  /nav/voice/response        (String JSON) 语音回复
  /nav/cmd_vel               (TwistStamped) 速度指令 (停止时使用)

借鉴:
  - DimOS agent.py: 消息队列 + 线程循环 + ReAct Agent
  - DimOS navigation.py: 三层降级 (tagged → visual → semantic map)
  - DimOS system_prompt.py: 安全优先 + 技能说明 + 行为准则
"""

import json
import logging
import os
import threading
import time
from typing import Any, Callable, Dict, List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from .skill_registry import LingTuNavigationSkills, SkillRegistry

logger = logging.getLogger(__name__)

# ============================================================
# LangChain 可选导入 (S100P 可能没装)
# ============================================================

try:
    from langchain_core.messages import AIMessage, HumanMessage, SystemMessage
    from langchain_core.tools import StructuredTool
    from langchain_openai import ChatOpenAI
    HAS_LANGCHAIN = True
except ImportError:
    HAS_LANGCHAIN = False
    logger.warning("LangChain not installed, Agent will use fallback mode")


# ============================================================
# System Prompt
# ============================================================

SYSTEM_PROMPT_ZH = """你是灵途 (LingTu)，一个自主导航四足机器人的 AI 大脑。
你由穹沛科技 (Qiongpei Technology) 开发。

# 安全第一
优先保证人类安全。尊重个人边界。绝不执行可能伤害人类、损坏财产或损坏机器人的操作。

# 你的能力
你可以通过以下技能控制机器人:
{skill_descriptions}

# 沟通风格
- 简短、有用、友好
- 用中文回答
- 主动告知你正在做什么

# 环境感知
当前位置: {robot_position}
周围环境: {scene_description}
已知地点: {known_places}

# 行为准则
- 收到导航请求时，先检查已知地点，再尝试场景图匹配，最后视觉搜索
- 如果目标不明确，主动询问
- 导航过程中简短报告进度
- 遇到障碍主动绕行，不需要用户确认
- 收到停止指令时立刻停止，优先于一切其他操作

# 多步任务
当用户请求包含多个步骤时，按顺序逐步执行。每完成一步，简短报告进度。
例如: "先去体育馆，再找红椅子" → 先调 navigate_to("体育馆")，到达后调 find_object("红色椅子")
"""

SYSTEM_PROMPT_EN = """You are LingTu, the AI brain of an autonomous quadruped navigation robot.
Built by Qiongpei Technology.

# Safety First
Prioritize human safety above all. Respect personal boundaries.
Never take actions that could harm humans, damage property, or damage the robot.

# Your Skills
{skill_descriptions}

# Communication
- Be concise, helpful, friendly
- Report what you're doing proactively

# Environment
Position: {robot_position}
Scene: {scene_description}
Known places: {known_places}

# Navigation Flow
- For navigation: check tagged locations first, then scene graph, then visual search
- Ask for clarification when the target is ambiguous
- Stop immediately when asked — this overrides everything else
"""


# ============================================================
# Agent Node
# ============================================================

class AgentNode(Node):
    """LLM Agent ROS2 节点 — 机器人的决策大脑。

    借鉴 DimOS Agent 的消息队列 + 线程循环模式:
      - 输入消息入队，独立线程处理，不阻塞 ROS2 spin
      - LangChain AgentExecutor (ReAct 模式) 做工具调用决策
      - 无 LangChain 时降级为 Mock 模式 (直接转发为导航指令)
    """

    def __init__(self):
        super().__init__("agent_node")

        # ── 参数声明 (与 planner_node.py 风格一致) ──
        self.declare_parameter("llm.backend", "kimi")       # kimi / openai / mock
        self.declare_parameter("llm.model", "")             # 空=使用默认
        self.declare_parameter("llm.temperature", 0.3)
        self.declare_parameter("language", "zh")
        self.declare_parameter("agent.enabled", True)
        self.declare_parameter("agent.max_iterations", 5)
        self.declare_parameter("agent.history_len", 10)     # 保留最近N轮对话

        backend = self.get_parameter("llm.backend").value
        model = self.get_parameter("llm.model").value
        temperature = float(self.get_parameter("llm.temperature").value)
        self._language: str = self.get_parameter("language").value
        self._enabled: bool = self.get_parameter("agent.enabled").value
        self._max_iterations: int = self.get_parameter("agent.max_iterations").value
        self._history_len: int = self.get_parameter("agent.history_len").value

        # ── 发布器 (在 skill callbacks 之前初始化) ──
        self._pub_output = self.create_publisher(String, "/nav/agent/output", 10)
        self._pub_instruction = self.create_publisher(
            String, "/nav/semantic/instruction", 10
        )
        self._pub_tag = self.create_publisher(
            String, "/nav/semantic/tag_location", 10
        )
        self._pub_bbox_nav = self.create_publisher(
            String, "/nav/semantic/bbox_navigate", 10
        )
        self._pub_voice = self.create_publisher(
            String, "/nav/voice/response", 10
        )
        self._pub_cmd_vel = self.create_publisher(
            TwistStamped, "/nav/cmd_vel", 10
        )

        # ── ROS2 状态 ──
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._latest_scene_graph: str = ""
        self._conversation_history: List[Any] = []  # LangChain BaseMessage 列表
        self._processing: bool = False
        self._lock = threading.Lock()

        # ── Skill 初始化 (使用 skill_registry.py 的统一实现) ──
        self._nav_skills = LingTuNavigationSkills()
        self._setup_skill_callbacks()
        self._skill_registry = SkillRegistry()
        self._skill_registry.register_instance(self._nav_skills)

        # ── LLM 初始化 ──
        self._agent = None          # LangChain AgentExecutor
        self._chat_model = None     # 无 Agent 时退化为直接 chat
        self._init_llm(backend, model, temperature)

        # ── 订阅 ──
        self.create_subscription(
            String, "/nav/agent/input", self._input_callback, 10
        )
        self.create_subscription(
            Odometry, "/nav/odometry", self._odom_callback, 10
        )
        self.create_subscription(
            String, "/nav/semantic/scene_graph", self._scene_graph_callback, 1
        )
        self.create_subscription(
            String, "/nav/planner_status", self._planner_status_callback, 10
        )

        # ── 对话历史持久化 (SQLite, 可选) ──
        self.declare_parameter('conversation_db_path', '')
        db_path = self.get_parameter('conversation_db_path').value
        self._conversation_store = None
        if db_path:
            try:
                import sys as _sys
                _pkg_root = os.path.join(os.path.dirname(__file__), '..', '..', 'semantic_perception')
                if _pkg_root not in _sys.path:
                    _sys.path.insert(0, _pkg_root)
                from semantic_perception.storage.sqlite_store import SqliteStore
                self._conversation_store = SqliteStore(db_path, table='conversations')
                self._load_conversation_history()
            except Exception as _e:
                self.get_logger().warn(f"Failed to init conversation SQLite store: {_e}")

        # ── MCP Server (可选) ──
        self.declare_parameter('mcp.enable', False)
        self.declare_parameter('mcp.port', 8090)
        self._mcp_server = None
        if self.get_parameter('mcp.enable').value:
            try:
                from .mcp_server import MCPServer
                self._mcp_server = MCPServer(
                    self._skill_registry,
                    port=int(self.get_parameter('mcp.port').value),
                )
                self._mcp_server.start()
                self.get_logger().info(
                    f"MCP Server started on port {self.get_parameter('mcp.port').value}"
                )
            except Exception as _e:
                self.get_logger().warn(f"Failed to start MCP server: {_e}")

        skill_count = len(self._skill_registry.get_skills())
        self.get_logger().info(
            f"Agent node started | backend={backend} | skills={skill_count} | "
            f"langchain={HAS_LANGCHAIN}"
        )

    # ------------------------------------------------------------------
    # 对话历史持久化
    # ------------------------------------------------------------------

    def _load_conversation_history(self) -> None:
        """从 SQLite 加载最近的对话历史。"""
        if not self._conversation_store or not HAS_LANGCHAIN:
            return
        try:
            for _ts, entry in self._conversation_store.iter_items():
                role = entry.get('role', 'human')
                content = entry.get('content', '')
                if role == 'human':
                    self._conversation_history.append(HumanMessage(content=content))
                elif role == 'ai':
                    self._conversation_history.append(AIMessage(content=content))
            # 只保留最近 20 轮
            self._conversation_history = self._conversation_history[-20:]
            self.get_logger().info(
                f"Loaded {len(self._conversation_history)} conversation entries from SQLite"
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to load conversation history: {e}")

    def _save_conversation_entry(self, role: str, content: str) -> None:
        """保存一条对话记录到 SQLite。"""
        if self._conversation_store is None:
            return
        try:
            self._conversation_store.save(time.time(), {'role': role, 'content': content})
        except Exception as e:
            self.get_logger().warn(f"Failed to save conversation entry: {e}")

    # ------------------------------------------------------------------
    # LLM 初始化
    # ------------------------------------------------------------------

    def _init_llm(self, backend: str, model: str, temperature: float) -> None:
        """初始化 LLM。支持 OpenAI / Kimi / Mock 三种后端。"""
        if not HAS_LANGCHAIN or backend == "mock":
            self.get_logger().info("Agent running in mock mode (no LLM)")
            return

        try:
            if backend == "openai":
                api_key = os.environ.get("OPENAI_API_KEY", "")
                self._chat_model = ChatOpenAI(
                    model=model or "gpt-4o",
                    temperature=temperature,
                    api_key=api_key,
                )
            elif backend == "kimi":
                # Kimi 兼容 OpenAI API
                api_key = os.environ.get("MOONSHOT_API_KEY", "")
                self._chat_model = ChatOpenAI(
                    model=model or "moonshot-v1-auto",
                    temperature=temperature,
                    api_key=api_key,
                    base_url="https://api.moonshot.cn/v1",
                )
            else:
                self.get_logger().warning(
                    f"Unknown LLM backend '{backend}', falling back to mock mode"
                )
                return
        except Exception as e:
            self.get_logger().error(f"Failed to initialize LLM model: {e}")
            return

        if self._chat_model is None:
            return

        # 尝试创建 LangChain ReAct Agent
        tools = self._skill_registry.to_langchain_tools()
        if not tools:
            self.get_logger().warning("No tools available, Agent will use plain chat mode")
            return

        try:
            from langchain.agents import AgentExecutor, create_tool_calling_agent
            from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder

            prompt = ChatPromptTemplate.from_messages([
                ("system", "{system_prompt}"),
                MessagesPlaceholder("chat_history"),
                ("human", "{input}"),
                MessagesPlaceholder("agent_scratchpad"),
            ])

            agent = create_tool_calling_agent(self._chat_model, tools, prompt)
            self._agent = AgentExecutor(
                agent=agent,
                tools=tools,
                verbose=True,
                max_iterations=self._max_iterations,
                handle_parsing_errors=True,
            )
            self.get_logger().info(
                f"LangChain ReAct Agent created | tools={len(tools)}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to create LangChain Agent: {e}. Falling back to plain chat."
            )

    # ------------------------------------------------------------------
    # Skill 回调注入 (借鉴 DimOS NavigationSkillContainer 三层降级)
    # ------------------------------------------------------------------

    def _setup_skill_callbacks(self) -> None:
        """将 ROS2 发布动作注入到 LingTuNavigationSkills。
        使用 skill_registry.py 的 set_callbacks(**kwargs) API。
        """

        def navigate_fn(target: str) -> str:
            msg = String()
            msg.data = target
            self._pub_instruction.publish(msg)
            return f"已发送导航指令: {target}"

        def tag_fn(name: str) -> str:
            msg = String()
            msg.data = json.dumps(
                {"name": name, "x": self._robot_x, "y": self._robot_y},
                ensure_ascii=False,
            )
            self._pub_tag.publish(msg)
            return f"已标记当前位置 ({self._robot_x:.1f}, {self._robot_y:.1f}) 为: {name}"

        def bbox_nav_fn(target: str) -> str:
            msg = String()
            msg.data = json.dumps({"target": target}, ensure_ascii=False)
            self._pub_bbox_nav.publish(msg)
            return f"正在视觉搜索: {target}"

        def stop_fn() -> str:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "body"
            self._pub_cmd_vel.publish(twist)
            return "已停止运动"

        def query_fn(query: str = "status") -> str:
            if query == "list":
                return "暂无已标记地点 (TaggedLocationStore 待接入)"
            return f"当前位置: ({self._robot_x:.1f}, {self._robot_y:.1f})"

        def explore_fn() -> str:
            msg = String()
            msg.data = json.dumps({"command": "explore"}, ensure_ascii=False)
            self._pub_instruction.publish(msg)
            return "已启动自主探索"

        def follow_fn(description: str) -> str:
            self._pub_follow = getattr(self, '_pub_follow', None)
            if self._pub_follow is None:
                self._pub_follow = self.create_publisher(
                    String, '/nav/semantic/follow_person_cmd', 10
                )
            msg = String()
            msg.data = json.dumps(
                {"target_label": description, "follow_distance": 1.5},
                ensure_ascii=False,
            )
            self._pub_follow.publish(msg)
            return f"开始跟随: {description}"

        def describe_fn() -> str:
            if self._latest_scene_graph:
                try:
                    sg = json.loads(self._latest_scene_graph)
                    objects = sg.get('objects', [])
                    labels = [o.get('label', '?') for o in objects[:10]]
                    return (
                        f"当前位置: ({self._robot_x:.1f}, {self._robot_y:.1f})。"
                        f"看到: {', '.join(labels)}"
                    )
                except Exception:
                    pass
            return f"当前位置: ({self._robot_x:.1f}, {self._robot_y:.1f})，周围环境信息不可用"

        def photo_fn() -> str:
            return "拍照功能开发中 (需要接入相机保存)"

        def patrol_fn(places: str) -> str:
            place_list = [p.strip() for p in places.split(',') if p.strip()] if places else []
            if not place_list:
                return "请指定巡逻地点，如: patrol('入口,走廊,办公室')"
            msg = String()
            msg.data = place_list[0]
            self._pub_instruction.publish(msg)
            return f"开始巡逻: {' → '.join(place_list)}。正在前往 {place_list[0]}"

        # skill_registry.py 的 LingTuNavigationSkills.set_callbacks() 接受 **kwargs
        self._nav_skills.set_callbacks(
            navigate=navigate_fn,
            tag=tag_fn,
            bbox_nav=bbox_nav_fn,
            stop=stop_fn,
            query=query_fn,
            explore=explore_fn,
            follow=follow_fn,
            describe=describe_fn,
            photo=photo_fn,
            patrol=patrol_fn,
        )

    # ------------------------------------------------------------------
    # System Prompt 构建
    # ------------------------------------------------------------------

    def _build_system_prompt(self) -> str:
        """动态构建 system prompt，注入当前环境信息。"""
        template = SYSTEM_PROMPT_ZH if self._language == "zh" else SYSTEM_PROMPT_EN
        scene_summary = (
            self._latest_scene_graph[:500]
            if self._latest_scene_graph
            else "未知"
        )
        return template.format(
            skill_descriptions=self._skill_registry.to_prompt_description(),
            robot_position=f"({self._robot_x:.1f}, {self._robot_y:.1f})",
            scene_description=scene_summary,
            known_places="暂无 (TaggedLocationStore 待接入)",
        )

    # ------------------------------------------------------------------
    # ROS2 回调
    # ------------------------------------------------------------------

    def _input_callback(self, msg: String) -> None:
        """用户输入回调 — 触发 Agent 推理 (借鉴 DimOS 消息队列模式)。"""
        user_input = msg.data.strip()
        if not user_input:
            return

        self.get_logger().info(f"Agent input: {user_input}")

        with self._lock:
            if self._processing:
                self.get_logger().warn(
                    "Agent is busy processing previous input, ignoring new input"
                )
                return
            self._processing = True

        # 独立线程处理，不阻塞 ROS2 spin
        thread = threading.Thread(
            target=self._process_input,
            args=(user_input,),
            daemon=True,
            name="agent-inference",
        )
        thread.start()

    def _odom_callback(self, msg: Odometry) -> None:
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _scene_graph_callback(self, msg: String) -> None:
        self._latest_scene_graph = msg.data

    def _planner_status_callback(self, msg: String) -> None:
        """规划器状态回调 — 到达目标或卡住时发布反馈。"""
        status = msg.data.strip()
        if status == "GOAL_REACHED":
            # 发布到达反馈
            out_payload = json.dumps(
                {"response": "已到达目标!", "event": "arrived", "timestamp": time.time()},
                ensure_ascii=False,
            )
            out_msg = String()
            out_msg.data = out_payload
            self._pub_output.publish(out_msg)

            voice_payload = json.dumps(
                {"response": "已到达目标", "timestamp": time.time()},
                ensure_ascii=False,
            )
            voice_msg = String()
            voice_msg.data = voice_payload
            self._pub_voice.publish(voice_msg)

            self.get_logger().info("Planner status: GOAL_REACHED, published arrived feedback")

        elif status == "STUCK":
            # 发布卡住反馈
            out_payload = json.dumps(
                {
                    "response": "导航卡住了，正在尝试重新规划...",
                    "event": "stuck",
                    "timestamp": time.time(),
                },
                ensure_ascii=False,
            )
            out_msg = String()
            out_msg.data = out_payload
            self._pub_output.publish(out_msg)

            self.get_logger().warning("Planner status: STUCK, published stuck feedback")

    # ------------------------------------------------------------------
    # Agent 推理 (在独立线程中执行)
    # ------------------------------------------------------------------

    def _process_input(self, user_input: str) -> None:
        """处理用户输入，执行 LLM 推理或 Mock 模式。"""
        try:
            response = self._run_inference(user_input)

            # 更新对话历史
            if HAS_LANGCHAIN:
                self._conversation_history.append(
                    HumanMessage(content=user_input)
                )
                self._conversation_history.append(
                    AIMessage(content=response)
                )
                # 保留最近 N 轮
                max_msgs = self._history_len * 2
                if len(self._conversation_history) > max_msgs:
                    self._conversation_history = self._conversation_history[-max_msgs:]

            # 持久化对话历史到 SQLite
            self._save_conversation_entry('human', user_input)
            self._save_conversation_entry('ai', response)

            # 发布 Agent 输出
            out_payload = json.dumps(
                {
                    "response": response,
                    "input": user_input,
                    "timestamp": time.time(),
                },
                ensure_ascii=False,
            )
            out_msg = String()
            out_msg.data = out_payload
            self._pub_output.publish(out_msg)

            # 语音回复
            voice_payload = json.dumps(
                {"response": response, "timestamp": time.time()},
                ensure_ascii=False,
            )
            voice_msg = String()
            voice_msg.data = voice_payload
            self._pub_voice.publish(voice_msg)

            self.get_logger().info(f"Agent output: {response[:120]}")

        except Exception as e:
            self.get_logger().error(f"Agent inference error: {e}")
        finally:
            with self._lock:
                self._processing = False

    def _run_inference(self, user_input: str) -> str:
        """执行实际推理，返回回复字符串。"""
        if self._agent is not None:
            # LangChain ReAct Agent 模式
            result = self._agent.invoke(
                {
                    "system_prompt": self._build_system_prompt(),
                    "input": user_input,
                    "chat_history": self._conversation_history[-(self._history_len * 2):],
                }
            )
            return result.get("output", "")

        if self._chat_model is not None:
            # 无 Agent，退化为直接 chat (无工具调用)
            messages = [SystemMessage(content=self._build_system_prompt())]
            messages.extend(
                self._conversation_history[-(self._history_len * 2):]
            )
            messages.append(HumanMessage(content=user_input))
            ai_msg = self._chat_model.invoke(messages)
            return ai_msg.content

        # Mock 模式: 直接将输入转发为导航指令
        nav_msg = String()
        nav_msg.data = user_input
        self._pub_instruction.publish(nav_msg)
        return f"[Mock] 收到指令: {user_input}，已转发给导航规划器。"

    def destroy_node(self) -> None:
        """节点销毁时清理资源。"""
        if getattr(self, '_mcp_server', None) is not None:
            self._mcp_server.stop()
        super().destroy_node()


# ============================================================
# Entry Point
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = AgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
