# Inspired by DimOS agents/annotation.py, Apache 2.0 License
"""
LingTu Skill 注册系统 — 让 LLM Agent 能自动发现和调用机器人能力。
借鉴 DimOS @skill 装饰器 + LangChain StructuredTool 集成。

架构:
  @skill 装饰器 → SkillRegistry 注册 → LangChain tools 列表 → Agent 自动调用

用法:
  # 定义 skill
  class NavigationSkills:
      @skill(name="导航到目标", description="用自然语言描述目标位置，机器人将自动导航过去")
      def navigate_to(self, target: str) -> str:
          '''target: 目标描述，如 "体育馆", "红色椅子"'''
          ...

  # 注册到 registry
  registry = SkillRegistry()
  registry.register_instance(nav_skills)

  # 生成 LangChain tools
  tools = registry.to_langchain_tools()
"""

import inspect
import logging
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Optional

logger = logging.getLogger(__name__)

try:
    from pydantic import BaseModel, Field
    from pydantic import create_model as pydantic_create_model
    _PYDANTIC_AVAILABLE = True
except ImportError:
    _PYDANTIC_AVAILABLE = False
    logger.warning("pydantic 不可用，to_langchain_tools() 将返回空列表")

try:
    from langchain_core.tools import StructuredTool
    _LANGCHAIN_AVAILABLE = True
except ImportError:
    _LANGCHAIN_AVAILABLE = False
    logger.warning("langchain_core 不可用，to_langchain_tools() 将返回空列表")


# ---------------------------------------------------------------------------
# 类型映射: Python 注解 → JSON Schema type 字符串
# ---------------------------------------------------------------------------
_PY_TYPE_TO_JSON: dict[type, str] = {
    str: "string",
    int: "integer",
    float: "number",
    bool: "boolean",
    list: "array",
    dict: "object",
}

_PY_TYPE_TO_PYDANTIC: dict[type, Any] = {
    str: str,
    int: int,
    float: float,
    bool: bool,
    list: list,
    dict: dict,
}


# ---------------------------------------------------------------------------
# @skill 装饰器
# ---------------------------------------------------------------------------

def skill(
    name: str = "",
    description: str = "",
    category: str = "general",
) -> Callable:
    """标记方法为 LLM 可调用的 skill。

    用法:
        @skill(name="标记地点", description="记住当前位置")
        def tag_location(self, name: str) -> str: ...
    """
    def decorator(fn: Callable) -> Callable:
        fn.__skill__ = True  # type: ignore[attr-defined]
        fn.__skill_name__ = name or fn.__name__  # type: ignore[attr-defined]
        fn.__skill_description__ = description or (fn.__doc__ or "").strip()  # type: ignore[attr-defined]
        fn.__skill_category__ = category  # type: ignore[attr-defined]
        return fn
    return decorator


# ---------------------------------------------------------------------------
# SkillInfo 数据类
# ---------------------------------------------------------------------------

@dataclass
class SkillInfo:
    name: str                  # 技能名称（给 LLM 看的）
    description: str           # 描述（给 LLM 看的）
    category: str              # 分类: navigation / perception / memory / system
    method: Callable           # 绑定方法
    parameters: dict           # 参数 schema（从类型注解自动提取）
    return_type: str           # 返回类型描述


# ---------------------------------------------------------------------------
# 参数 Schema 提取
# ---------------------------------------------------------------------------

def _extract_parameters(method: Callable) -> dict:
    """从方法的类型注解自动提取参数 schema。

    例如:
        def navigate_to(self, target: str, speed: float = 0.5) -> str:
    →   {"target": {"type": "string", "required": True},
         "speed":  {"type": "number",  "required": False, "default": 0.5}}
    """
    sig = inspect.signature(method)
    params: dict[str, dict] = {}
    for param_name, param in sig.parameters.items():
        if param_name == "self":
            continue
        annotation = param.annotation
        json_type = "string"
        if annotation is not inspect.Parameter.empty:
            json_type = _PY_TYPE_TO_JSON.get(annotation, "string")
        has_default = param.default is not inspect.Parameter.empty
        entry: dict[str, Any] = {
            "type": json_type,
            "required": not has_default,
        }
        if has_default:
            entry["default"] = param.default
        params[param_name] = entry
    return params


def _get_return_type(method: Callable) -> str:
    """提取返回类型描述字符串。"""
    hints = {}
    try:
        hints = method.__annotations__
    except AttributeError:
        pass
    ret = hints.get("return", None)
    if ret is None:
        return "any"
    return getattr(ret, "__name__", str(ret))


def _create_args_model(skill_info: SkillInfo):
    """动态创建 Pydantic BaseModel 作为 LangChain tool 的 args_schema。

    如果 pydantic 不可用则返回 None。
    """
    if not _PYDANTIC_AVAILABLE:
        return None

    field_definitions: dict[str, Any] = {}
    for param_name, schema in skill_info.parameters.items():
        py_type = _PY_TYPE_TO_PYDANTIC.get(
            {v: k for k, v in _PY_TYPE_TO_JSON.items()}.get(schema["type"], str),
            str,
        )
        if schema["required"]:
            field_definitions[param_name] = (py_type, Field(..., description=param_name))
        else:
            field_definitions[param_name] = (
                py_type,
                Field(default=schema.get("default"), description=param_name),
            )

    if not field_definitions:
        # 无参数 skill — 返回空 model
        return pydantic_create_model(f"{skill_info.name}_args")

    return pydantic_create_model(f"{skill_info.name}_args", **field_definitions)


# ---------------------------------------------------------------------------
# SkillRegistry
# ---------------------------------------------------------------------------

class SkillRegistry:
    """Skill 注册中心 — 统一管理所有机器人能力。"""

    def __init__(self) -> None:
        self._skills: dict[str, SkillInfo] = {}  # name → SkillInfo

    # ------------------------------------------------------------------
    # 注册接口
    # ------------------------------------------------------------------

    def register_instance(self, instance: object) -> int:
        """扫描实例的所有 @skill 方法，注册到 registry。返回注册数量。"""
        count = 0
        for attr_name in dir(instance):
            if attr_name.startswith("_"):
                continue
            try:
                method = getattr(instance, attr_name)
            except AttributeError:
                continue
            if not callable(method):
                continue
            if not getattr(method, "__skill__", False):
                continue
            self._register_method(method)
            count += 1
        return count

    def register_function(
        self,
        fn: Callable,
        name: str = "",
        description: str = "",
        category: str = "general",
    ) -> None:
        """注册一个独立函数为 skill。"""
        skill_name = name or getattr(fn, "__skill_name__", fn.__name__)
        skill_desc = description or getattr(fn, "__skill_description__", (fn.__doc__ or "").strip())
        skill_cat = getattr(fn, "__skill_category__", category)
        params = _extract_parameters(fn)
        ret = _get_return_type(fn)
        info = SkillInfo(
            name=skill_name,
            description=skill_desc,
            category=skill_cat,
            method=fn,
            parameters=params,
            return_type=ret,
        )
        self._skills[skill_name] = info
        logger.debug(f"已注册 skill (function): {skill_name}")

    def _register_method(self, method: Callable) -> None:
        skill_name = getattr(method, "__skill_name__", method.__name__)
        skill_desc = getattr(method, "__skill_description__", (method.__doc__ or "").strip())
        skill_cat = getattr(method, "__skill_category__", "general")
        params = _extract_parameters(method)
        ret = _get_return_type(method)
        info = SkillInfo(
            name=skill_name,
            description=skill_desc,
            category=skill_cat,
            method=method,
            parameters=params,
            return_type=ret,
        )
        self._skills[skill_name] = info
        logger.debug(f"已注册 skill: {skill_name} [{skill_cat}]")

    # ------------------------------------------------------------------
    # 查询接口
    # ------------------------------------------------------------------

    def get_skills(self, category: Optional[str] = None) -> list[SkillInfo]:
        """获取所有 skill（可按类别过滤）。"""
        if category is None:
            return list(self._skills.values())
        return [s for s in self._skills.values() if s.category == category]

    def get_skill(self, name: str) -> Optional[SkillInfo]:
        """按名称获取 skill。"""
        return self._skills.get(name)

    def list_names(self) -> list[str]:
        """返回所有已注册 skill 的名称列表。"""
        return list(self._skills.keys())

    # ------------------------------------------------------------------
    # 调用接口
    # ------------------------------------------------------------------

    def call_skill(self, name: str, **kwargs: Any) -> str:
        """调用指定 skill，返回字符串结果。"""
        info = self._skills.get(name)
        if info is None:
            return f"错误: 未找到 skill '{name}'"
        try:
            result = info.method(**kwargs)
            return str(result) if result is not None else ""
        except Exception as exc:
            logger.exception(f"调用 skill '{name}' 时出错")
            return f"错误: {exc}"

    # ------------------------------------------------------------------
    # LangChain 集成
    # ------------------------------------------------------------------

    def to_langchain_tools(self) -> list:
        """生成 LangChain StructuredTool 列表，供 Agent 使用。

        每个 skill → 一个 StructuredTool:
          - name: skill 名称
          - description: skill 描述
          - func: 调用方法
          - args_schema: 从参数类型注解自动生成 Pydantic BaseModel
        """
        if not _LANGCHAIN_AVAILABLE or not _PYDANTIC_AVAILABLE:
            logger.warning("LangChain 或 Pydantic 不可用，返回空工具列表")
            return []

        tools = []
        for info in self._skills.values():
            args_model = _create_args_model(info)
            try:
                tool = StructuredTool(
                    name=info.name,
                    description=info.description,
                    func=info.method,
                    args_schema=args_model,
                )
                tools.append(tool)
            except Exception as exc:
                logger.warning(f"无法为 skill '{info.name}' 创建 StructuredTool: {exc}")
        return tools

    # ------------------------------------------------------------------
    # Prompt 描述生成
    # ------------------------------------------------------------------

    def to_prompt_description(self) -> str:
        """生成给 LLM 的 skill 列表描述（中英双语）。"""
        if not self._skills:
            return "当前没有可用的 skill。\nNo skills available."

        lines = [
            "# 可用技能列表 / Available Skills",
            "",
        ]
        categories: dict[str, list[SkillInfo]] = {}
        for info in self._skills.values():
            categories.setdefault(info.category, []).append(info)

        _category_labels = {
            "navigation": "导航 / Navigation",
            "perception": "感知 / Perception",
            "memory": "记忆 / Memory",
            "system": "系统 / System",
            "general": "通用 / General",
        }

        for cat, skills in sorted(categories.items()):
            label = _category_labels.get(cat, cat)
            lines.append(f"## {label}")
            for info in skills:
                lines.append(f"- **{info.name}**: {info.description}")
                if info.parameters:
                    param_strs = []
                    for p_name, p_schema in info.parameters.items():
                        req = "必填" if p_schema["required"] else "选填"
                        param_strs.append(f"{p_name}({p_schema['type']},{req})")
                    lines.append(f"  参数: {', '.join(param_strs)}")
            lines.append("")

        return "\n".join(lines)


# ---------------------------------------------------------------------------
# LingTu 预定义导航 Skills
# ---------------------------------------------------------------------------

class LingTuNavigationSkills:
    """LingTu 导航技能集 — 注册到 Agent 后可通过自然语言调用。

    这些 skill 方法内部通过回调函数调用 ROS2 话题/服务。
    回调函数在 planner_node 初始化时注入。
    """

    def __init__(self) -> None:
        self._navigate_fn: Optional[Callable] = None    # 注入: 发布 /nav/semantic/instruction
        self._tag_fn: Optional[Callable] = None         # 注入: 调用 TaggedLocationStore.tag
        self._query_fn: Optional[Callable] = None       # 注入: 查询 TaggedLocationStore
        self._bbox_nav_fn: Optional[Callable] = None    # 注入: 发布 /nav/semantic/bbox_navigate
        self._stop_fn: Optional[Callable] = None        # 注入: 停止导航
        self._speak_fn: Optional[Callable] = None       # 注入: 发布 /nav/voice/response
        self._explore_fn: Optional[Callable] = None     # 注入: 启动探索
        self._follow_fn: Optional[Callable] = None      # 注入: 跟随人
        self._describe_fn: Optional[Callable] = None    # 注入: 描述场景
        self._photo_fn: Optional[Callable] = None       # 注入: 拍照
        self._patrol_fn: Optional[Callable] = None      # 注入: 巡逻

    def set_callbacks(self, **kwargs: Any) -> None:
        """注入 ROS2 回调函数。

        支持的键: navigate, tag, query, bbox_nav, stop, speak, explore,
                  follow, describe, photo, patrol
        """
        for key, fn in kwargs.items():
            attr = f"_{key}_fn"
            if hasattr(self, attr):
                setattr(self, attr, fn)
            else:
                logger.warning(f"LingTuNavigationSkills: 未知回调键 '{key}'，已忽略")

    @skill(
        name="navigate_to",
        description=(
            "导航到指定目标。输入自然语言描述，如'体育馆'、'红色椅子旁边'。"
            "机器人会自动规划路径前往。"
            " / Navigate to target described in natural language."
        ),
        category="navigation",
    )
    def navigate_to(self, target: str) -> str:
        """target: 目标描述"""
        if self._navigate_fn:
            return self._navigate_fn(target)
        return "导航功能未就绪"

    @skill(
        name="tag_location",
        description=(
            "记住当前位置并命名。下次可以直接导航到这个名字。"
            " / Tag current location with a name for later navigation."
        ),
        category="memory",
    )
    def tag_location(self, name: str) -> str:
        """name: 地点名称，如'体育馆入口'"""
        if self._tag_fn:
            return self._tag_fn(name)
        return "标记功能未就绪"

    @skill(
        name="find_object",
        description=(
            "在当前视野中寻找物体。机器人会用视觉模型在画面中定位目标并导航过去。"
            " / Find and navigate to an object in the current camera view."
        ),
        category="navigation",
    )
    def find_object(self, target: str) -> str:
        """target: 要找的物体描述"""
        if self._bbox_nav_fn:
            return self._bbox_nav_fn(target)
        return "视觉寻物功能未就绪"

    @skill(
        name="stop",
        description=(
            "立即停止机器人运动。"
            " / Immediately stop the robot."
        ),
        category="system",
    )
    def stop(self) -> str:
        if self._stop_fn:
            return self._stop_fn()
        return "已停止"

    @skill(
        name="explore",
        description=(
            "开始自主探索未知环境。机器人会自动选择 frontier 进行覆盖探索。"
            " / Start autonomous exploration of unknown environment."
        ),
        category="navigation",
    )
    def explore(self) -> str:
        if self._explore_fn:
            return self._explore_fn()
        return "探索功能未就绪"

    @skill(
        name="where_am_i",
        description=(
            "报告机器人当前位置和周围环境。"
            " / Report current robot position and surrounding environment."
        ),
        category="perception",
    )
    def where_am_i(self) -> str:
        if self._query_fn:
            return self._query_fn()
        return "位置查询功能未就绪"

    @skill(
        name="list_known_places",
        description=(
            "列出所有已标记的地点名称。"
            " / List all tagged location names."
        ),
        category="memory",
    )
    def list_known_places(self) -> str:
        if self._query_fn:
            return self._query_fn("list")
        return "无已标记地点"

    @skill(
        name="follow_person",
        description=(
            "跟随指定的人。描述你要跟随的人的外观特征。"
            " / Follow a person matching the given appearance description."
        ),
        category="navigation",
    )
    def follow_person(self, description: str = "person") -> str:
        """description: 人物外观描述，如 '穿红色衣服的人'"""
        if self._follow_fn:
            return self._follow_fn(description)
        return "跟随功能未就绪"

    @skill(
        name="describe_scene",
        description=(
            "描述当前机器人看到的场景。返回周围物体、房间类型等信息。"
            " / Describe the current scene: nearby objects, room type, etc."
        ),
        category="perception",
    )
    def describe_scene(self) -> str:
        if self._describe_fn:
            return self._describe_fn()
        return "场景描述功能未就绪"

    @skill(
        name="take_photo",
        description=(
            "拍摄当前视角的照片并保存。"
            " / Capture and save a photo from the current camera view."
        ),
        category="perception",
    )
    def take_photo(self) -> str:
        if self._photo_fn:
            return self._photo_fn()
        return "拍照功能未就绪"

    @skill(
        name="patrol",
        description=(
            "按照已标记的地点进行巡逻。机器人会依次访问指定的地点列表。"
            " / Patrol a list of named locations in order."
        ),
        category="navigation",
    )
    def patrol(self, places: str = "") -> str:
        """places: 巡逻地点列表，逗号分隔，如 '入口,走廊,办公室'。空=巡逻所有已标记地点"""
        if self._patrol_fn:
            return self._patrol_fn(places)
        return "巡逻功能未就绪"
