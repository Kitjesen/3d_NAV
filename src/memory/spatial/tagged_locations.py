"""
Tagged Location Store — 用户/Agent 手动标记的地点记忆。
借鉴 DimOS SpatialMemory.tag_location 概念。

用法:
  store = TaggedLocationStore("/data/maps/office_tags.json")
  store.tag("体育馆", x=25.0, y=30.0, z=0.0)
  result = store.query("体育馆")   # → {"name": "体育馆", "position": [25.0, 30.0, 0.0]}
"""

import json
import logging
import os
import threading
from typing import Dict, List, Optional

logger = logging.getLogger(__name__)


class TaggedLocationStore:
    """线程安全的标签地点存储，JSON 文件持久化。"""

    def __init__(self, json_path: str = "") -> None:
        """
        Args:
            json_path: 持久化 JSON 文件路径。空字符串 = 仅内存，不持久化。
        """
        self._path = json_path
        self._lock = threading.Lock()
        # 存储结构: {name: {"name": str, "position": [x, y, z], "yaw": float|None}}
        self._store: Dict[str, dict] = {}

        if self._path:
            self.load()

    # ── 写操作 ────────────────────────────────────────────────────────────

    def tag(
        self,
        name: str,
        x: float,
        y: float,
        z: float = 0.0,
        yaw: Optional[float] = None,
    ) -> None:
        """存储一个标签地点（已存在则覆盖）。

        Args:
            name: 地点名称（支持中文）
            x, y, z: 地图坐标（map 帧）
            yaw: 朝向角（弧度），可选
        """
        entry = {"name": name, "position": [float(x), float(y), float(z)], "yaw": yaw}
        with self._lock:
            self._store[name] = entry
        logger.info("Tagged location '%s' at (%.2f, %.2f, %.2f)", name, x, y, z)

    def remove(self, name: str) -> bool:
        """删除标签。返回是否实际删除了条目。"""
        with self._lock:
            existed = name in self._store
            self._store.pop(name, None)
        if existed:
            logger.info("Removed tagged location '%s'", name)
        return existed

    # ── 查询操作 ──────────────────────────────────────────────────────────

    def query(self, name: str) -> Optional[dict]:
        """精确匹配名称。

        Returns:
            {"name": str, "position": [x, y, z], "yaw": float|None} 或 None
        """
        with self._lock:
            return self._store.get(name)

    def query_fuzzy(self, text: str) -> Optional[dict]:
        """模糊匹配：text 包含标签名，或标签名包含 text（取最长匹配优先）。

        Args:
            text: 用户指令或关键词（支持中文）

        Returns:
            最佳匹配条目，或 None
        """
        with self._lock:
            entries = list(self._store.values())

        if not entries:
            return None

        best: Optional[dict] = None
        best_len = 0

        for entry in entries:
            name = entry["name"]
            # 双向包含匹配：指令里有标签名，或标签名里有指令词
            if name in text or text in name:
                if len(name) > best_len:
                    best = entry
                    best_len = len(name)

        return best

    def list_all(self) -> List[dict]:
        """列出所有标签地点。"""
        with self._lock:
            return list(self._store.values())

    # ── 持久化 ────────────────────────────────────────────────────────────

    def save(self) -> None:
        """将内存状态写入 JSON 文件。路径为空时跳过。"""
        if not self._path:
            return
        try:
            os.makedirs(os.path.dirname(self._path), exist_ok=True) if os.path.dirname(self._path) else None
            with self._lock:
                data = list(self._store.values())
            with open(self._path, "w", encoding="utf-8") as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            logger.info("Saved %d tagged location(s) to %s", len(data), self._path)
        except Exception as e:
            logger.warning("Failed to save tagged locations to %s: %s", self._path, e)

    def load(self) -> None:
        """从 JSON 文件加载。文件不存在时静默跳过。"""
        if not self._path or not os.path.isfile(self._path):
            return
        try:
            with open(self._path, "r", encoding="utf-8") as f:
                data = json.load(f)
            with self._lock:
                self._store = {entry["name"]: entry for entry in data if "name" in entry}
            logger.info("Loaded %d tagged location(s) from %s", len(self._store), self._path)
        except Exception as e:
            logger.warning("Failed to load tagged locations from %s: %s", self._path, e)
