"""
拓扑记忆 — 记录机器人探索过的位置 + 关键帧。

参考论文:
  - VLMnav (2024):     拓扑图 + 关键帧 → VLM 选择下一个节点
  - L3MVN (ICRA 2024): Language-guided 拓扑图 + frontier 探索
  - CLIP-Nav (2023):   CLIP 特征的拓扑图, 文本查询匹配节点

核心功能:
  1. 每到一个新位置, 存储 (position, keyframe_CLIP_feature, scene_graph_snapshot)
  2. 探索时: 查找最近的未探索 frontier, 避免重访
  3. 回溯时: 找到之前看到目标的位置, 快速返回
  4. LLM 查询时: "回到有红色沙发的房间" → CLIP 匹配拓扑节点
"""

import time
import math
import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


@dataclass
class TopoNode:
    """拓扑图节点 = 机器人到过的一个位置。"""
    node_id: int
    position: np.ndarray                # [x, y, z]
    timestamp: float                    # 首次到达时间
    visit_count: int = 1               # 访问次数
    last_visit: float = 0.0            # 最近访问时间

    # 感知快照 (到达时的观测)
    clip_feature: np.ndarray = field(default_factory=lambda: np.array([]))
    scene_snapshot: str = ""            # 到达时的场景图 JSON 子集
    visible_labels: List[str] = field(default_factory=list)

    # 图连接
    neighbors: List[int] = field(default_factory=list)  # 相邻节点 ID
    edge_distances: Dict[int, float] = field(default_factory=dict)

    # 创新4: 房间关联
    room_id: int = -1                   # 所属房间 ID
    room_name: str = ""                 # 所属房间名称

    def to_dict(self) -> Dict:
        d = {
            "node_id": self.node_id,
            "position": {
                "x": round(float(self.position[0]), 2),
                "y": round(float(self.position[1]), 2),
            },
            "visit_count": self.visit_count,
            "visible_labels": self.visible_labels,
            "neighbors": self.neighbors,
        }
        if self.room_id >= 0:
            d["room_id"] = self.room_id
            d["room_name"] = self.room_name
        return d


class TopologicalMemory:
    """
    拓扑记忆图。

    使用策略:
      - 每次机器人移动 > new_node_distance 米, 创建新节点
      - 节点之间按导航顺序连边
      - 每个节点存储 CLIP 全景特征 (来自感知) 和可见物体列表
      - 查询时支持: 文本匹配节点, 找最近未访问区域, 回溯路径
    """

    def __init__(
        self,
        new_node_distance: float = 2.0,    # 新建节点的最小移动距离
        max_nodes: int = 500,
    ):
        self.new_node_distance = new_node_distance
        self.max_nodes = max_nodes

        self._nodes: Dict[int, TopoNode] = {}
        self._next_id = 0
        self._current_node_id: int = -1
        self._path_history: List[int] = []  # 访问过的节点 ID 序列
        self._last_position: Optional[np.ndarray] = None

        # 创新4: 房间级覆盖追踪
        self._visited_rooms: Dict[int, Dict] = {}  # room_id → {name, first_visit, visit_count, node_ids}
        self._room_transition_history: List[Tuple[int, int]] = []  # (from_room, to_room) 序列

    @property
    def nodes(self) -> Dict[int, TopoNode]:
        return self._nodes

    @property
    def current_node(self) -> Optional[TopoNode]:
        return self._nodes.get(self._current_node_id)

    @property
    def visited_positions(self) -> List[np.ndarray]:
        """所有已访问位置。"""
        return [n.position for n in self._nodes.values()]

    def update_position(
        self,
        position: np.ndarray,
        visible_labels: Optional[List[str]] = None,
        scene_snapshot: str = "",
        clip_feature: Optional[np.ndarray] = None,
        room_id: int = -1,
        room_name: str = "",
    ) -> Optional[TopoNode]:
        """
        更新机器人位置, 必要时创建新节点。

        Args:
            position: [x, y, z]
            visible_labels: 当前可见物体标签列表
            scene_snapshot: 当前场景图子集
            clip_feature: 当前视角的 CLIP 全局特征
            room_id: 当前所在房间 ID (创新4)
            room_name: 当前所在房间名称 (创新4)

        Returns:
            新创建的节点 (如果创建了), 否则 None
        """
        pos = np.array(position[:2], dtype=np.float64)  # 2D

        # 检查是否需要创建新节点
        nearest, nearest_dist = self._find_nearest_node(pos)

        if nearest is not None and nearest_dist < self.new_node_distance:
            # 更新现有节点
            nearest.visit_count += 1
            nearest.last_visit = time.time()
            if visible_labels:
                existing = set(nearest.visible_labels)
                for lbl in visible_labels:
                    if lbl not in existing:
                        nearest.visible_labels.append(lbl)
            if scene_snapshot:
                nearest.scene_snapshot = scene_snapshot
            if clip_feature is not None and clip_feature.size > 0:
                nearest.clip_feature = clip_feature
            if room_id >= 0:
                nearest.room_id = room_id
                nearest.room_name = room_name

            if nearest.node_id != self._current_node_id:
                self._add_edge(self._current_node_id, nearest.node_id, nearest_dist)
                # 检测房间转换
                prev_node = self._nodes.get(self._current_node_id)
                if prev_node and prev_node.room_id >= 0 and nearest.room_id >= 0:
                    if prev_node.room_id != nearest.room_id:
                        self._room_transition_history.append(
                            (prev_node.room_id, nearest.room_id)
                        )
                self._current_node_id = nearest.node_id
                self._path_history.append(nearest.node_id)

            # 更新房间覆盖
            if room_id >= 0:
                self._update_room_coverage(room_id, room_name, nearest.node_id)

            self._last_position = pos
            return None
        else:
            new_node = TopoNode(
                node_id=self._next_id,
                position=np.array(position[:3], dtype=np.float64),
                timestamp=time.time(),
                last_visit=time.time(),
                visible_labels=visible_labels or [],
                scene_snapshot=scene_snapshot,
                clip_feature=clip_feature if clip_feature is not None else np.array([]),
                room_id=room_id,
                room_name=room_name,
            )
            self._nodes[self._next_id] = new_node

            if self._current_node_id >= 0:
                dist = float(np.linalg.norm(
                    pos - self._nodes[self._current_node_id].position[:2]
                ))
                self._add_edge(self._current_node_id, self._next_id, dist)
                # 检测房间转换
                prev_node = self._nodes.get(self._current_node_id)
                if prev_node and prev_node.room_id >= 0 and room_id >= 0:
                    if prev_node.room_id != room_id:
                        self._room_transition_history.append(
                            (prev_node.room_id, room_id)
                        )

            self._current_node_id = self._next_id
            self._path_history.append(self._next_id)
            self._next_id += 1
            self._last_position = pos

            if room_id >= 0:
                self._update_room_coverage(room_id, room_name, new_node.node_id)

            self._prune_if_needed()
            return new_node

    def query_by_text(self, query: str, top_k: int = 3) -> List[TopoNode]:
        """
        文本查询匹配拓扑节点 (CLIP-Nav 方案)。

        匹配策略:
          1. 标签匹配: 查询文本与节点可见物体标签对比
          2. (如果有 CLIP) 文本-图像相似度

        Args:
            query: "有红色沙发的房间" / "entrance area"
            top_k: 返回最多 N 个节点

        Returns:
            匹配的节点列表 (按相关度降序)
        """
        query_lower = query.lower()
        scored_nodes: List[Tuple[TopoNode, float]] = []

        for node in self._nodes.values():
            score = 0.0

            # 标签匹配
            for label in node.visible_labels:
                if label.lower() in query_lower or query_lower in label.lower():
                    score += 1.0

            # 场景快照匹配
            if query_lower in node.scene_snapshot.lower():
                score += 0.5

            if score > 0:
                scored_nodes.append((node, score))

        scored_nodes.sort(key=lambda x: x[1], reverse=True)
        return [node for node, _ in scored_nodes[:top_k]]

    def get_least_visited_direction(
        self,
        current_position: np.ndarray,
        min_distance: float = 3.0,
    ) -> Optional[np.ndarray]:
        """
        获取最少探索的方向 (L3MVN 方案)。

        用于探索策略: 优先去访问次数最少的区域。

        Args:
            current_position: [x, y]
            min_distance: 最小距离, 避免返回太近的位置

        Returns:
            方向向量 [dx, dy] 或 None
        """
        if not self._nodes:
            return None

        # 计算每个方向扇区 (8 个方向) 的访问密度
        sector_count = 8
        sector_visits = [0] * sector_count
        sector_distances = [float('inf')] * sector_count

        pos = np.array(current_position[:2])

        for node in self._nodes.values():
            delta = node.position[:2] - pos
            dist = float(np.linalg.norm(delta))
            if dist < 0.1:
                continue

            angle = math.atan2(delta[1], delta[0])  # -pi to pi
            sector = int((angle + math.pi) / (2 * math.pi) * sector_count) % sector_count
            sector_visits[sector] += node.visit_count
            sector_distances[sector] = min(sector_distances[sector], dist)

        # 找访问最少的方向
        min_visits = min(sector_visits) if sector_visits else 0
        candidates = [
            i for i, v in enumerate(sector_visits)
            if v == min_visits
        ]

        if not candidates:
            return None

        # 在候选中选最远的方向 (鼓励探索新区域)
        best = max(candidates, key=lambda i: sector_distances[i])
        angle = (best / sector_count) * 2 * math.pi - math.pi

        return np.array([math.cos(angle), math.sin(angle)])

    def get_backtrack_position(self, steps_back: int = 1) -> Optional[np.ndarray]:
        """
        回溯到之前的位置 (LOVON 的 BACKTRACK 动作)。

        Args:
            steps_back: 回退步数

        Returns:
            之前位置的 [x, y, z] 或 None
        """
        if len(self._path_history) <= steps_back:
            return None

        target_id = self._path_history[-(steps_back + 1)]
        node = self._nodes.get(target_id)
        return node.position.copy() if node else None

    @property
    def visited_room_ids(self) -> set:
        """已访问的房间 ID 集合。"""
        return set(self._visited_rooms.keys())

    def get_room_coverage(self) -> List[Dict]:
        """获取房间级探索覆盖情况。"""
        coverage = []
        for rid, info in self._visited_rooms.items():
            coverage.append({
                "room_id": rid,
                "room_name": info.get("name", f"room_{rid}"),
                "visit_count": info.get("visit_count", 0),
                "node_count": len(info.get("node_ids", [])),
                "first_visit": info.get("first_visit", 0.0),
            })
        coverage.sort(key=lambda x: x["visit_count"], reverse=True)
        return coverage

    def get_room_transitions(self) -> List[Tuple[int, int]]:
        """获取房间转换历史 (用于验证拓扑边)。"""
        return list(self._room_transition_history)

    def _update_room_coverage(
        self, room_id: int, room_name: str, node_id: int,
    ) -> None:
        """更新房间级覆盖信息。"""
        if room_id not in self._visited_rooms:
            self._visited_rooms[room_id] = {
                "name": room_name,
                "first_visit": time.time(),
                "visit_count": 0,
                "node_ids": set(),
            }
        info = self._visited_rooms[room_id]
        info["visit_count"] += 1
        info["node_ids"].add(node_id)
        if room_name and room_name != info.get("name", ""):
            info["name"] = room_name

    def get_exploration_summary(self) -> Dict:
        """
        生成探索摘要 (给 LLM 消费)。

        Returns:
            {
              "total_nodes": int,
              "total_visits": int,
              "coverage_estimate": float,
              "least_explored_direction": str,
              "recently_visited": [...]
            }
        """
        if not self._nodes:
            return {"total_nodes": 0, "total_visits": 0, "coverage_estimate": 0.0}

        total_visits = sum(n.visit_count for n in self._nodes.values())

        # 最近访问的节点
        recent_ids = self._path_history[-5:]
        recent = [
            self._nodes[nid].to_dict()
            for nid in recent_ids
            if nid in self._nodes
        ]

        # 覆盖范围估计 (凸包面积的近似)
        positions = np.array([n.position[:2] for n in self._nodes.values()])
        if len(positions) >= 3:
            from scipy.spatial import ConvexHull
            try:
                hull = ConvexHull(positions)
                coverage = hull.volume  # 2D 时 volume = area
            except Exception:
                coverage = 0.0
        else:
            coverage = 0.0

        room_cov = self.get_room_coverage()

        return {
            "total_nodes": len(self._nodes),
            "total_visits": total_visits,
            "coverage_area_m2": round(coverage, 1),
            "recently_visited": recent,
            "rooms_visited": len(self._visited_rooms),
            "room_coverage": room_cov[:10],
            "room_transitions": len(self._room_transition_history),
        }

    def get_graph_json(self) -> str:
        """导出拓扑图为 JSON。"""
        import json
        nodes = [n.to_dict() for n in self._nodes.values()]
        edges = []
        seen = set()
        for node in self._nodes.values():
            for neighbor_id in node.neighbors:
                key = (min(node.node_id, neighbor_id), max(node.node_id, neighbor_id))
                if key not in seen:
                    seen.add(key)
                    edges.append({
                        "from": key[0],
                        "to": key[1],
                        "distance": round(node.edge_distances.get(neighbor_id, 0), 2),
                    })

        return json.dumps({
            "nodes": nodes,
            "edges": edges,
            "path_history": self._path_history[-20:],
        }, ensure_ascii=False)

    # ── 内部方法 ──

    def _find_nearest_node(self, pos_2d: np.ndarray) -> Tuple[Optional[TopoNode], float]:
        """找最近节点。"""
        best_node = None
        best_dist = float('inf')

        for node in self._nodes.values():
            dist = float(np.linalg.norm(pos_2d - node.position[:2]))
            if dist < best_dist:
                best_dist = dist
                best_node = node

        return best_node, best_dist

    def _add_edge(self, from_id: int, to_id: int, distance: float):
        """在两个节点间添加边。"""
        if from_id < 0 or to_id < 0:
            return
        if from_id not in self._nodes or to_id not in self._nodes:
            return

        a = self._nodes[from_id]
        b = self._nodes[to_id]

        if to_id not in a.neighbors:
            a.neighbors.append(to_id)
            a.edge_distances[to_id] = distance

        if from_id not in b.neighbors:
            b.neighbors.append(from_id)
            b.edge_distances[from_id] = distance

    def _prune_if_needed(self):
        """如果节点超过上限, 移除最旧最少访问的。"""
        while len(self._nodes) > self.max_nodes:
            # 找访问次数最少的 (排除当前节点)
            worst_id = min(
                (nid for nid in self._nodes if nid != self._current_node_id),
                key=lambda nid: (self._nodes[nid].visit_count, self._nodes[nid].last_visit),
            )
            # 移除边引用
            node = self._nodes[worst_id]
            for neighbor_id in node.neighbors:
                if neighbor_id in self._nodes:
                    nb = self._nodes[neighbor_id]
                    if worst_id in nb.neighbors:
                        nb.neighbors.remove(worst_id)
                    nb.edge_distances.pop(worst_id, None)
            del self._nodes[worst_id]
