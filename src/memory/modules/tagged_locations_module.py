"""TaggedLocationsModule — 标签地点记忆模块 (Module 模式封装)。

将 TaggedLocationStore 封装为 Module In/Out 端口接口，
支持 "save:名称" 保存当前位置、"goto:名称" 召回已保存位置。

层级: L3 (Perception)
输入: odometry, tag_command
输出: saved_location (召回的位置), tag_status (操作结果状态)
"""

from __future__ import annotations

import logging
from typing import Any, Optional

import numpy as np

from core import Module, In, Out
from core.msgs import Odometry, PoseStamped
from core.msgs.geometry import Pose, Quaternion, Vector3

from memory.spatial.tagged_locations import TaggedLocationStore

logger = logging.getLogger(__name__)


class TaggedLocationsModule(Module, layer=3):
    """标签地点记忆模块。

    通过 tag_command 端口接收命令:
    - "save:<name>" — 保存当前位置为 <name>
    - "goto:<name>" — 召回 <name> 对应位置
    - "remove:<name>" — 删除 <name>
    - "list" — 列出所有标签

    Config:
        json_path: 持久化文件路径 (default "" = 仅内存)
    """

    # -- 端口声明 --
    odometry: In[Odometry]
    tag_command: In[str]

    saved_location: Out[PoseStamped]
    tag_status: Out[str]

    def __init__(self, **config: Any) -> None:
        json_path = config.pop("json_path", "")
        super().__init__(**config)
        self._store = TaggedLocationStore(json_path=json_path)
        self._last_odom: Optional[Odometry] = None

    # -- 生命周期 --

    def setup(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.tag_command.subscribe(self._on_command)

    def _on_odom(self, odom: Odometry) -> None:
        if not (np.isfinite(odom.x) and np.isfinite(odom.y)):
            return
        self._last_odom = odom

    def _on_command(self, command: str) -> None:
        """处理标签命令。"""
        command = command.strip()
        if not command:
            return

        if command.startswith("save:"):
            self._handle_save(command[5:].strip())
        elif command.startswith("goto:"):
            self._handle_goto(command[5:].strip())
        elif command.startswith("remove:"):
            self._handle_remove(command[7:].strip())
        elif command == "list":
            self._handle_list()
        else:
            self.tag_status.publish(f"unknown_command:{command}")

    def _handle_save(self, name: str) -> None:
        if not name:
            self.tag_status.publish("error:empty_name")
            return
        if self._last_odom is None:
            self.tag_status.publish("error:no_odometry")
            return

        odom = self._last_odom
        self._store.tag(name, x=odom.x, y=odom.y, z=odom.z)
        self.tag_status.publish(f"saved:{name}")

    def _handle_goto(self, name: str) -> None:
        if not name:
            self.tag_status.publish("error:empty_name")
            return

        # 先精确匹配，再模糊匹配
        entry = self._store.query(name)
        if entry is None:
            entry = self._store.query_fuzzy(name)

        if entry is None:
            self.tag_status.publish(f"not_found:{name}")
            return

        pos = entry["position"]
        goal = PoseStamped(
            pose=Pose(
                position=Vector3(float(pos[0]), float(pos[1]), float(pos[2])),
                orientation=Quaternion(),
            ),
            frame_id="map",
        )
        self.saved_location.publish(goal)
        self.tag_status.publish(f"recalled:{entry['name']}")

    def _handle_remove(self, name: str) -> None:
        if self._store.remove(name):
            self.tag_status.publish(f"removed:{name}")
        else:
            self.tag_status.publish(f"not_found:{name}")

    def _handle_list(self) -> None:
        entries = self._store.list_all()
        names = [e["name"] for e in entries]
        self.tag_status.publish(f"locations:{','.join(names)}" if names else "locations:")

    # -- 外部 API --

    @property
    def store(self) -> TaggedLocationStore:
        """访问内部 TaggedLocationStore (测试用)。"""
        return self._store
