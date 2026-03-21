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
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

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
# Skill Registry (内联，无需独立文件)
# ============================================================

@dataclass
class SkillDef:
    """单个 Skill 的定义。"""
    name: str
    description: str
    args_schema: Dict[str, Any]    # JSON Schema properties
    required_args: List[str]
    callback: Optional[Callable] = None


class SkillRegistry:
    """管理 Agent 可用的 Skill 列表，并转换为 LangChain Tool。"""

    def __init__(self):
        self._skills: Dict[str, SkillDef] = {}

    def register(self, skill: SkillDef) -> None:
        self._skills[skill.name] = skill

    def get_skills(self) -> List[SkillDef]:
        return list(self._skills.values())

    def to_prompt_description(self) -> str:
        """生成可读的技能描述，注入 system prompt。"""
        lines = []
        for s in self._skills.values():
            args_str = ", ".join(
                f"{k}: {v.get('type', 'any')}"
                for k, v in s.args_schema.items()
            )
            lines.append(f"- {s.name}({args_str}): {s.description}")
        return "\n".join(lines) if lines else "(无可用技能)"

    def to_langchain_tools(self) -> List[Any]:
        """转换为 LangChain StructuredTool 列表。"""
        if not HAS_LANGCHAIN:
            return []

        from pydantic import BaseModel, create_model

        tools = []
        for s in self._skills.values():
            if s.callback is None:
                continue

            # 动态构造 Pydantic 模型作为 args_schema
            field_defs: Dict[str, Any] = {}
            for arg_name, arg_schema in s.args_schema.items():
                py_type = str  # 默认
                if arg_schema.get("type") == "number":
                    py_type = float
                elif arg_schema.get("type") == "integer":
                    py_type = int
                elif arg_schema.get("type") == "boolean":
                    py_type = bool
                # 必填 vs 可选
                if arg_name in s.required_args:
                    field_defs[arg_name] = (py_type, ...)
                else:
                    default_val = arg_schema.get("default", "")
                    field_defs[arg_name] = (Optional[py_type], default_val)

            ArgsModel = create_model(f"{s.name}_args", **field_defs)
            callback = s.callback  # 避免闭包捕获循环变量

            def make_func(cb: Callable) -> Callable:
                def tool_func(**kwargs: Any) -> str:
                    try:
                        return cb(**kwargs)
                    except Exception as e:
                        return f"技能执行错误: {e}"
                return tool_func

            tool = StructuredTool(
                name=s.name,
                description=s.description,
                func=make_func(callback),
                args_schema=ArgsModel,
            )
            tools.append(tool)

        return tools


# ============================================================
# LingTu Navigation Skills (借鉴 DimOS NavigationSkillContainer)
# ============================================================

class LingTuNavigationSkills:
    """
    LingTu 机器人导航技能集合。

    借鉴 DimOS navigation.py 三层降级:
      1. 已标记地点 (tagged location) → 最快
      2. 场景图语义匹配 → /nav/semantic/instruction
      3. 视觉 BBox 搜索 → /nav/semantic/bbox_navigate

    回调通过 set_callbacks() 注入 (解耦 ROS2 发布逻辑)。
    """

    def __init__(self):
        self._cb_navigate: Optional[Callable[[str], str]] = None
        self._cb_tag: Optional[Callable[[str], str]] = None
        self._cb_bbox_nav: Optional[Callable[[str], str]] = None
        self._cb_stop: Optional[Callable[[], str]] = None
        self._cb_query: Optional[Callable[[str], str]] = None
        self._cb_explore: Optional[Callable[[], str]] = None

    def set_callbacks(
        self,
        navigate: Callable[[str], str],
        tag: Callable[[str], str],
        bbox_nav: Callable[[str], str],
        stop: Callable[[], str],
        query: Callable[[str], str],
        explore: Callable[[], str],
    ) -> None:
        """注入 ROS2 发布回调。"""
        self._cb_navigate = navigate
        self._cb_tag = tag
        self._cb_bbox_nav = bbox_nav
        self._cb_stop = stop
        self._cb_query = query
        self._cb_explore = explore

    def navigate_to_target(self, target: str) -> str:
        """语义导航到目标位置。三层降级: 已标记地点 → 场景图匹配 → 视觉搜索。

        Args:
            target: 导航目标描述，如"大门"、"机械设备"、"桌子"
        """
        if self._cb_navigate is None:
            return "错误: 导航回调未初始化"
        return self._cb_navigate(target)

    def tag_current_location(self, name: str) -> str:
        """将当前位置标记到空间记忆，以便之后导航回来。

        Args:
            name: 地点名称，如"充电站"、"起点"、"办公室入口"
        """
        if self._cb_tag is None:
            return "错误: 标记回调未初始化"
        return self._cb_tag(name)

    def visual_search_and_navigate(self, target: str) -> str:
        """通过摄像头视觉搜索目标并导航过去 (适合场景图里没有的目标)。

        Args:
            target: 视觉搜索目标，如"穿红衣服的人"、"快递箱"
        """
        if self._cb_bbox_nav is None:
            return "错误: 视觉导航回调未初始化"
        return self._cb_bbox_nav(target)

    def stop_movement(self) -> str:
        """立刻停止机器人所有运动。紧急情况或用户要求停止时调用。"""
        if self._cb_stop is None:
            return "错误: 停止回调未初始化"
        return self._cb_stop()

    def query_robot_state(self, query: str = "status") -> str:
        """查询机器人当前状态或已知地点列表。

        Args:
            query: "status" 查当前位置, "list" 列出已标记地点
        """
        if self._cb_query is None:
            return "错误: 查询回调未初始化"
        return self._cb_query(query)

    def start_exploration(self) -> str:
        """启动自主探索模式，机器人自动探索未知区域并建立地图。"""
        if self._cb_explore is None:
            return "错误: 探索回调未初始化"
        return self._cb_explore()

    def build_skill_registry(self) -> SkillRegistry:
        """构建并返回已注册所有本实例方法的 SkillRegistry。"""
        registry = SkillRegistry()

        registry.register(SkillDef(
            name="navigate_to_target",
            description="语义导航到目标位置。先查已标记地点，再匹配场景图，最后视觉搜索。",
            args_schema={"target": {"type": "string", "description": "导航目标描述"}},
            required_args=["target"],
            callback=self.navigate_to_target,
        ))
        registry.register(SkillDef(
            name="tag_current_location",
            description="将当前位置标记到空间记忆，以便之后导航回来。",
            args_schema={"name": {"type": "string", "description": "地点名称"}},
            required_args=["name"],
            callback=self.tag_current_location,
        ))
        registry.register(SkillDef(
            name="visual_search_and_navigate",
            description="通过摄像头视觉搜索目标并导航过去，适合场景图里没有的目标。",
            args_schema={"target": {"type": "string", "description": "视觉搜索目标描述"}},
            required_args=["target"],
            callback=self.visual_search_and_navigate,
        ))
        registry.register(SkillDef(
            name="stop_movement",
            description="立刻停止机器人所有运动。紧急情况或用户要求停止时调用。",
            args_schema={},
            required_args=[],
            callback=self.stop_movement,
        ))
        registry.register(SkillDef(
            name="query_robot_state",
            description="查询机器人当前状态或已知地点列表。query='status'查位置, 'list'列地点。",
            args_schema={"query": {"type": "string", "default": "status", "description": "查询类型: status 或 list"}},
            required_args=[],
            callback=self.query_robot_state,
        ))
        registry.register(SkillDef(
            name="start_exploration",
            description="启动自主探索模式，机器人自动探索未知区域。",
            args_schema={},
            required_args=[],
            callback=self.start_exploration,
        ))

        return registry


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

        # ── Skill 初始化 ──
        self._nav_skills = LingTuNavigationSkills()
        self._setup_skill_callbacks()
        self._skill_registry = self._nav_skills.build_skill_registry()

        # ── LLM 初始化 ──
        self._agent = None          # LangChain AgentExecutor
        self._chat_model = None     # 无 Agent 时退化为直接 chat
        self._init_llm(backend, model, temperature)

        # ── ROS2 状态 ──
        self._robot_x: float = 0.0
        self._robot_y: float = 0.0
        self._latest_scene_graph: str = ""
        self._conversation_history: List[Any] = []  # LangChain BaseMessage 列表
        self._processing: bool = False
        self._lock = threading.Lock()

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

        # ── 发布 ──
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

        skill_count = len(self._skill_registry.get_skills())
        self.get_logger().info(
            f"Agent node started | backend={backend} | skills={skill_count} | "
            f"langchain={HAS_LANGCHAIN}"
        )

    # ------------------------------------------------------------------
    # LLM 初始化
    # ------------------------------------------------------------------

    def _init_llm(self, backend: str, model: str, temperature: float) -> None:
        """初始化 LLM。支持 OpenAI / Kimi / Mock 三种后端。"""
        if not HAS_LANGCHAIN or backend == "mock":
            self.get_logger().info("Agent running in mock mode (no LLM)")
            return

        import os

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
        """将 ROS2 发布动作注入到 NavigationSkills。"""

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

        self._nav_skills.set_callbacks(
            navigate=navigate_fn,
            tag=tag_fn,
            bbox_nav=bbox_nav_fn,
            stop=stop_fn,
            query=query_fn,
            explore=explore_fn,
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
