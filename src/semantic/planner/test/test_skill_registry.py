"""Skill Registry 单元测试 — 不依赖 ROS2，纯 Python 可运行。

运行:
    cd src/semantic_planner && python -m pytest test/test_skill_registry.py -v
"""

import pytest
from semantic_planner.legacy.skill_registry import (
    SkillRegistry,
    LingTuNavigationSkills,
    skill,
    _extract_parameters,
)


# ---------------------------------------------------------------------------
# 辅助: 最小 skill 类
# ---------------------------------------------------------------------------

class _MinimalSkills:
    @skill(name="greet", description="说你好", category="general")
    def greet(self, who: str = "world") -> str:
        return f"Hello, {who}!"

    @skill(name="add", description="两数相加", category="math")
    def add(self, a: int, b: int) -> int:
        return a + b

    def not_a_skill(self):
        """不应被注册"""
        return "nope"


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestSkillDecorator:
    def test_skill_decorator_marks_flag(self):
        """@skill 装饰器应在函数上设置 __skill__ = True。"""
        @skill(name="test_skill", description="just a test")
        def my_fn(x: str) -> str:
            return x
        assert my_fn.__skill__ is True

    def test_skill_decorator_sets_name(self):
        """@skill name 参数应保存到 __skill_name__。"""
        @skill(name="custom_name", description="desc")
        def fn():
            pass
        assert fn.__skill_name__ == "custom_name"

    def test_skill_decorator_defaults_name_to_function_name(self):
        """未指定 name 时，应回落到函数名。"""
        @skill(description="desc")
        def my_func():
            pass
        assert my_func.__skill_name__ == "my_func"

    def test_skill_decorator_sets_description(self):
        @skill(name="s", description="my description")
        def fn():
            pass
        assert fn.__skill_description__ == "my description"

    def test_skill_decorator_sets_category(self):
        @skill(name="s", description="d", category="navigation")
        def fn():
            pass
        assert fn.__skill_category__ == "navigation"


class TestRegisterInstance:
    def setup_method(self):
        self.registry = SkillRegistry()
        self.skills_obj = _MinimalSkills()

    def test_register_instance_returns_count(self):
        count = self.registry.register_instance(self.skills_obj)
        assert count == 2

    def test_register_instance_names_present(self):
        self.registry.register_instance(self.skills_obj)
        names = self.registry.list_names()
        assert "greet" in names
        assert "add" in names

    def test_register_instance_excludes_non_skill_methods(self):
        self.registry.register_instance(self.skills_obj)
        assert "not_a_skill" not in self.registry.list_names()

    def test_register_lingtu_navigation_skills_has_11_skills(self):
        """LingTuNavigationSkills 应注册 11 个 skill。"""
        registry = SkillRegistry()
        nav = LingTuNavigationSkills()
        count = registry.register_instance(nav)
        assert count == 11


class TestRegisterFunction:
    def setup_method(self):
        self.registry = SkillRegistry()

    def test_register_function_basic(self):
        def compute(x: float) -> float:
            return x * 2.0
        self.registry.register_function(compute, name="double", description="two times x")
        assert "double" in self.registry.list_names()

    def test_register_function_custom_name(self):
        def fn():
            pass
        self.registry.register_function(fn, name="my_skill")
        assert self.registry.get_skill("my_skill") is not None

    def test_register_function_category(self):
        def fn():
            pass
        self.registry.register_function(fn, name="s", category="navigation")
        info = self.registry.get_skill("s")
        assert info.category == "navigation"


class TestGetSkill:
    def setup_method(self):
        self.registry = SkillRegistry()
        self.registry.register_instance(_MinimalSkills())

    def test_get_skill_by_name_found(self):
        info = self.registry.get_skill("greet")
        assert info is not None
        assert info.name == "greet"

    def test_get_skill_by_name_not_found(self):
        assert self.registry.get_skill("nonexistent") is None

    def test_get_skill_description(self):
        info = self.registry.get_skill("greet")
        assert "说你好" in info.description


class TestGetSkillsByCategory:
    def setup_method(self):
        self.registry = SkillRegistry()
        nav = LingTuNavigationSkills()
        self.registry.register_instance(nav)

    def test_filter_navigation(self):
        nav_skills = self.registry.get_skills(category="navigation")
        assert len(nav_skills) > 0
        for s in nav_skills:
            assert s.category == "navigation"

    def test_filter_memory(self):
        mem_skills = self.registry.get_skills(category="memory")
        assert len(mem_skills) > 0
        for s in mem_skills:
            assert s.category == "memory"

    def test_filter_perception(self):
        perc_skills = self.registry.get_skills(category="perception")
        assert len(perc_skills) > 0

    def test_filter_system(self):
        sys_skills = self.registry.get_skills(category="system")
        assert len(sys_skills) > 0

    def test_filter_none_returns_all(self):
        all_skills = self.registry.get_skills()
        assert len(all_skills) == 11


class TestCallSkill:
    def setup_method(self):
        self.registry = SkillRegistry()
        self.registry.register_instance(_MinimalSkills())

    def test_call_skill_returns_result(self):
        result = self.registry.call_skill("greet", who="Alice")
        assert result == "Hello, Alice!"

    def test_call_skill_default_arg(self):
        result = self.registry.call_skill("greet")
        assert "world" in result.lower() or result == "Hello, world!"

    def test_call_skill_not_found(self):
        result = self.registry.call_skill("nonexistent")
        assert "错误" in result or "error" in result.lower()

    def test_call_skill_with_int_args(self):
        result = self.registry.call_skill("add", a=3, b=4)
        assert result == "7"


class TestCallSkillWithCallback:
    def test_navigate_with_callback(self):
        registry = SkillRegistry()
        nav = LingTuNavigationSkills()
        registry.register_instance(nav)

        calls = []
        nav.set_callbacks(navigate=lambda target: f"navigating to {target}")

        result = registry.call_skill("navigate_to", target="体育馆")
        assert "体育馆" in result

    def test_stop_with_callback(self):
        registry = SkillRegistry()
        nav = LingTuNavigationSkills()
        registry.register_instance(nav)

        nav.set_callbacks(stop=lambda: "stopped by callback")
        result = registry.call_skill("stop")
        assert "stopped by callback" == result

    def test_tag_with_callback(self):
        nav = LingTuNavigationSkills()

        tagged = []
        nav.set_callbacks(tag=lambda name: tagged.append(name) or f"tagged {name}")
        # call the method directly to avoid kwarg name collision with call_skill(name=...)
        result = nav.tag_location("入口")
        assert "入口" in tagged
        assert "入口" in result


class TestToPromptDescription:
    def setup_method(self):
        self.registry = SkillRegistry()
        nav = LingTuNavigationSkills()
        self.registry.register_instance(nav)

    def test_prompt_contains_all_skill_names(self):
        prompt = self.registry.to_prompt_description()
        for name in self.registry.list_names():
            assert name in prompt

    def test_prompt_contains_descriptions(self):
        prompt = self.registry.to_prompt_description()
        assert "导航" in prompt

    def test_prompt_sections_for_categories(self):
        prompt = self.registry.to_prompt_description()
        assert "navigation" in prompt.lower() or "导航" in prompt

    def test_empty_registry_message(self):
        empty = SkillRegistry()
        prompt = empty.to_prompt_description()
        assert "没有" in prompt or "No skills" in prompt


class TestParameterExtraction:
    def test_required_parameter(self):
        @skill(name="s", description="d")
        def fn(x: str) -> str:
            return x
        params = _extract_parameters(fn)
        assert "x" in params
        assert params["x"]["required"] is True
        assert params["x"]["type"] == "string"

    def test_optional_parameter_with_default(self):
        @skill(name="s", description="d")
        def fn(x: int = 5) -> int:
            return x
        params = _extract_parameters(fn)
        assert params["x"]["required"] is False
        assert params["x"]["default"] == 5

    def test_float_type_annotation(self):
        @skill(name="s", description="d")
        def fn(speed: float = 0.5) -> str:
            return str(speed)
        params = _extract_parameters(fn)
        assert params["speed"]["type"] == "number"

    def test_bool_type_annotation(self):
        @skill(name="s", description="d")
        def fn(flag: bool = False) -> str:
            return str(flag)
        params = _extract_parameters(fn)
        assert params["flag"]["type"] == "boolean"

    def test_no_parameters(self):
        @skill(name="s", description="d")
        def fn() -> str:
            return "ok"
        params = _extract_parameters(fn)
        assert params == {}

    def test_self_excluded(self):
        """self 参数不应出现在提取结果中。"""
        obj = _MinimalSkills()
        params = _extract_parameters(obj.greet)
        assert "self" not in params


class TestToLangchainTools:
    def setup_method(self):
        self.registry = SkillRegistry()
        nav = LingTuNavigationSkills()
        self.registry.register_instance(nav)

    def test_langchain_tools_count_if_available(self):
        """如果 langchain_core 可用，tools 数量应等于已注册 skill 数量。"""
        try:
            import langchain_core  # noqa: F401
            tools = self.registry.to_langchain_tools()
            assert len(tools) == 11
        except ImportError:
            tools = self.registry.to_langchain_tools()
            assert tools == []

    def test_langchain_tools_returns_list(self):
        """无论 langchain 是否可用，返回值都应是 list。"""
        result = self.registry.to_langchain_tools()
        assert isinstance(result, list)
