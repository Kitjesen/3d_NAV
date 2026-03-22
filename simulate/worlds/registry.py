"""
仿真世界注册表

管理可用的仿真场景 XML，支持路径查找和别名注册。
"""
from pathlib import Path
from typing import Dict, Optional

# 项目根目录 (simulate/ 上两级)
_LINGTU_ROOT = Path(__file__).resolve().parent.parent.parent

# sim/worlds/ 目录 (已有 XML 文件)
_SIM_WORLDS = _LINGTU_ROOT / "sim" / "worlds"

# 内联简单场景 (empty world)
_EMPTY_WORLD_XML = """\
<mujoco model="empty">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <visual>
    <global offwidth="1280" offheight="960"/>
    <headlight ambient="0.5 0.5 0.5"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1=".85 .85 .82" rgb2=".7 .7 .68"/>
    <material name="grid" texture="grid" texrepeat="20 20" texuniform="true"/>
  </asset>
  <worldbody>
    <light pos="0 0 10" dir="0 0 -1" diffuse="0.9 0.88 0.82" castshadow="true"/>
    <geom name="floor" type="plane" size="50 50 0.1" material="grid"
          conaffinity="1" condim="3" friction="1 0.5 0.5"/>
  </worldbody>
</mujoco>
"""


class WorldRegistry:
    """世界场景注册表。

    注册项格式: name -> {"path": Path | None, "xml": str | None, "description": str}
    path 和 xml 二选一: path 优先。
    """

    def __init__(self):
        self._worlds: Dict[str, dict] = {}
        self._register_defaults()

    def _register_defaults(self):
        """注册内置场景."""
        # factory — 工厂场景
        self.register(
            name="factory",
            path=_SIM_WORLDS / "factory_scene.xml",
            description="工厂厂房场景 (含机械设备、大门、通道)",
        )
        # open_field — 开阔地
        self.register(
            name="open_field",
            path=_SIM_WORLDS / "open_field.xml",
            description="开阔平地场景 (无障碍物，适合基础导航测试)",
        )
        # spiral_terrain — 螺旋地形
        self.register(
            name="spiral",
            path=_SIM_WORLDS / "spiral_terrain.xml",
            description="螺旋坡地地形 (坡度测试)",
        )
        # building — 楼道场景
        self.register(
            name="building",
            path=_SIM_WORLDS / "building_scene.xml",
            description="楼道走廊场景 (狭窄通道测试)",
        )
        # empty — 纯平地，内联 XML
        self.register(
            name="empty",
            xml=_EMPTY_WORLD_XML,
            description="空平地场景 (最小测试环境)",
        )

    def register(
        self,
        name: str,
        path: Optional[Path] = None,
        xml: Optional[str] = None,
        description: str = "",
    ):
        """注册一个场景。

        Args:
            name:        场景唯一标识
            path:        MJCF XML 文件路径 (可选)
            xml:         内联 XML 字符串 (可选，path 不存在时使用)
            description: 场景描述
        """
        if path is None and xml is None:
            raise ValueError(f"world '{name}': must provide path or xml")
        self._worlds[name] = {"path": path, "xml": xml, "description": description}

    def get(self, name: str) -> dict:
        """获取场景信息。

        Returns:
            {"path": Path|None, "xml": str|None, "description": str}

        Raises:
            KeyError: 场景不存在
        """
        if name not in self._worlds:
            available = ", ".join(sorted(self._worlds.keys()))
            raise KeyError(
                f"World '{name}' not found. Available: {available}"
            )
        return self._worlds[name]

    def get_xml_or_path(self, name: str) -> tuple:
        """返回 (xml_string_or_None, path_or_None)，调用者按需使用。

        优先返回文件路径 (若文件存在)，否则返回内联 XML。

        Returns:
            (xml: str | None, path: Path | None)
        """
        info = self.get(name)
        path = info.get("path")
        xml = info.get("xml")

        if path is not None and Path(path).exists():
            return None, Path(path)
        if xml is not None:
            return xml, None
        # path 不存在 → 尝试返回 None,None 由调用者处理
        return None, path

    def list(self) -> Dict[str, str]:
        """返回 {name: description} 字典."""
        return {k: v["description"] for k, v in self._worlds.items()}


# 模块级单例
_registry = WorldRegistry()


def get_world(name: str) -> dict:
    """获取场景信息 (模块级函数)."""
    return _registry.get(name)


def list_worlds() -> Dict[str, str]:
    """列出所有注册场景 (模块级函数)."""
    return _registry.list()
