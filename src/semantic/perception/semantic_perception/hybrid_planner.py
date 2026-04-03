"""EXPERIMENTAL — not used in production navigation stack.
混合路径规划器 (Hybrid Planner) — 拓扑图辅助的分层路径规划。

核心思想:
  将全局路径规划分解为两个层次:
  1. 拓扑层: 在拓扑图上做 Dijkstra，获得房间序列
  2. 几何层: 对每对相邻房间，在 Tomogram traversability grid 上做局部 A*

优势:
  - 减少 A* 搜索空间 (只在房间对之间搜索，比全局 A* 快 3-10 倍)
  - 利用拓扑图的高层连通性

失败语义 (重要):
  - A* 返回 None → 段规划失败 → 整条路径 success=False。
  - 绝不用直线兜底: 直线路径可能穿墙/过障碍物，让机器人以为有路但实际没有。
  - 调用方收到 success=False 后应触发 LERa 恢复或重新规划，而不是盲目执行。

参考:
  - TopoNav (2025): 拓扑图作为空间记忆
  - Hierarchical Planning: 分层规划减少搜索空间
  - src/global_planning/PCT_planner/planner/scripts/pct_planner_astar.py
"""

import heapq
import logging
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ── A* 在 traversability grid 上的纯 Python 实现 ─────────────────────────────
#
# 移植自 src/global_planning/PCT_planner/planner/scripts/pct_planner_astar.py，
# 去除 ROS2 依赖，直接在 2D traversability grid 上做 8-连通 A*。

def _w2g(wx: float, wy: float, cx: float, cy: float,
         res: float, ox: int, oy: int, nx: int, ny: int) -> Tuple[int, int]:
    """世界坐标 (m) → 格坐标 (clipped)。"""
    ix = int(round((wx - cx) / res)) + ox
    iy = int(round((wy - cy) / res)) + oy
    return int(np.clip(ix, 0, nx - 1)), int(np.clip(iy, 0, ny - 1))


def _g2w(ix: int, iy: int, cx: float, cy: float,
         res: float, ox: int, oy: int) -> Tuple[float, float]:
    """格坐标 → 世界坐标 (m)。"""
    return (ix - ox) * res + cx, (iy - oy) * res + cy


def _find_nearest_free(trav: np.ndarray, ci: int, cj: int,
                       obs_thr: float, radius: int = 5) -> Tuple[int, int]:
    """在 radius 格范围内找最近可行格，找不到就返回原位置（让 A* 自己报 None）。"""
    if trav[ci, cj] < obs_thr:
        return ci, cj
    best, best_dist = (ci, cj), float('inf')
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            ni, nj = ci + di, cj + dj
            if 0 <= ni < trav.shape[0] and 0 <= nj < trav.shape[1]:
                if trav[ni, nj] < obs_thr:
                    d = di * di + dj * dj
                    if d < best_dist:
                        best, best_dist = (ni, nj), d
    return best


def _astar_on_grid(trav: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int],
                   obs_thr: float, timeout_sec: float = 5.0) -> Optional[List[Tuple[int, int]]]:
    """8-连通 A* 在 traversability grid 上规划路径。

    Args:
        trav:        2D float array，trav[i,j] < obs_thr 表示可行
        start/goal:  (ix, iy) 格坐标
        obs_thr:     障碍阈值（与 PCT 一致，默认 49.9）
        timeout_sec: 超时秒数，超时返回 None（而不是 raise，调用方统一处理）

    Returns:
        List[(ix, iy)] 路径，或 None（不可达 / 超时）
    """
    deadline = time.monotonic() + timeout_sec
    h = lambda a, b: abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan 启发
    open_q = [(h(start, goal), 0.0, start, [])]
    visited: dict = {}

    while open_q:
        if time.monotonic() > deadline:
            logger.warning("A* timeout after %.1fs (visited %d nodes)", timeout_sec, len(visited))
            return None
        f, g, cur, path = heapq.heappop(open_q)
        if cur in visited:
            continue
        visited[cur] = True
        path = path + [cur]
        if cur == goal:
            return path
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                       (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            ni, nj = cur[0] + dx, cur[1] + dy
            if 0 <= ni < trav.shape[0] and 0 <= nj < trav.shape[1]:
                if trav[ni, nj] < obs_thr and (ni, nj) not in visited:
                    cost = g + (1.414 if dx and dy else 1.0)
                    heapq.heappush(open_q, (cost + h((ni, nj), goal), cost, (ni, nj), path))
    return None  # 不可达


def _downsample_cells(cells: list, min_dist: int = 3) -> list:
    """等距降采样：相邻保留点距离 >= min_dist 格。"""
    if len(cells) <= 2:
        return cells
    kept = [cells[0]]
    for c in cells[1:]:
        dx, dy = c[0] - kept[-1][0], c[1] - kept[-1][1]
        if dx * dx + dy * dy >= min_dist * min_dist:
            kept.append(c)
    if kept[-1] != cells[-1]:
        kept.append(cells[-1])
    return kept


# ── 数据类 ────────────────────────────────────────────────────────────────────

@dataclass
class PathSegment:
    """路径段 (连接两个房间的局部路径)。"""
    from_room_id: int
    to_room_id: int
    waypoints: List[np.ndarray]  # 路径点序列 [[x, y, z], ...]
    cost: float                   # 路径代价 (欧氏距离之和)
    planning_time: float          # 规划时间 (秒)


@dataclass
class HybridPath:
    """混合路径规划结果。

    success=False 意味着至少一个局部 A* 段无法规划，
    调用方不应执行此路径，应触发恢复策略。
    """
    success: bool
    waypoints: List[np.ndarray]
    room_sequence: List[int]
    segments: List[PathSegment]
    total_cost: float
    total_planning_time: float
    topology_planning_time: float
    geometry_planning_time: float
    num_waypoints: int
    num_rooms: int
    # 失败时携带诊断信息，帮助 LERa 决策
    failure_reason: str = ""


# ── 主规划器 ──────────────────────────────────────────────────────────────────

class HybridPlanner:
    """
    混合路径规划器 — 拓扑图辅助的分层路径规划。

    算法流程:
    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    Input: 起点 start, 终点 goal, 拓扑图 TSG, traversability grid
    Output: HybridPath (success=True 才可执行)

    1. 定位起点和终点所在的房间
    2. 拓扑图 Dijkstra → 房间序列 [R1, R2, ..., Rn]
    3. for each 相邻房间对 (Ri, Ri+1):
    4.     在 traversability grid 上做 8-连通 A*（限制在 search_bbox 内）
    5.     任意段失败 → 整条路径 success=False，立即返回
    6. 返回完整路径

    直线兜底 (fallback) 已被彻底删除。
    A* 失败的唯一正确处理是向上报告，由 SemanticPlannerModule 的 LERa 决策。
    """

    def __init__(
        self,
        topology_graph,
        trav: Optional[np.ndarray] = None,
        trav_res: float = 0.2,
        trav_cx: float = 0.0,
        trav_cy: float = 0.0,
        obstacle_thr: float = 49.9,
        astar_timeout_sec: float = 5.0,
        min_downsample_dist: int = 3,
    ):
        """
        Args:
            topology_graph:    TopologySemGraph 实例，提供 Dijkstra 最短路
            trav:              可选，2D float32 traversability grid (shape: [nx, ny])
                               trav[i,j] < obstacle_thr 表示可行。
                               来自 Tomogram.layers_t[slice_idx] 或 _load_tomogram()[0]。
                               若为 None，局部段规划直接返回 None（路径总是失败）。
            trav_res:          栅格分辨率 (m/格)，默认 0.2m
            trav_cx/trav_cy:   栅格中心世界坐标 (m)
            obstacle_thr:      障碍阈值（与 PCT 一致，49.9）
            astar_timeout_sec: 每个局部 A* 的超时时间
            min_downsample_dist: 路径点降采样最小间距（格数）

        注意：不再接受 planner_fn 回调参数。调用方直接传入 traversability grid，
        HybridPlanner 内部完成 A*。这样：
          1. 失败语义清晰：A* None → plan_path success=False → 调用方 LERa
          2. 无隐式直线兜底
          3. 无 semantic→nav 跨层导入
        """
        self.tsg = topology_graph
        self._trav = trav                          # None → 无地图，所有段失败
        self._trav_res = trav_res
        self._trav_cx = trav_cx
        self._trav_cy = trav_cy
        self._obs_thr = obstacle_thr
        self._astar_timeout = astar_timeout_sec
        self._min_ds = min_downsample_dist

        if trav is not None:
            self._nx, self._ny = trav.shape
            self._ox, self._oy = self._nx // 2, self._ny // 2
        else:
            self._nx = self._ny = self._ox = self._oy = 0

    def update_traversability(
        self,
        trav: np.ndarray,
        res: float,
        cx: float,
        cy: float,
    ) -> None:
        """热更新 traversability grid（地图更新时调用，线程不安全，调用方加锁）。"""
        self._trav = trav
        self._trav_res = res
        self._trav_cx = cx
        self._trav_cy = cy
        self._nx, self._ny = trav.shape
        self._ox, self._oy = self._nx // 2, self._ny // 2

    def plan_path(
        self,
        start: np.ndarray,
        goal: np.ndarray,
        search_radius_factor: float = 1.5,
        max_planning_time: float = 5.0,
    ) -> HybridPath:
        """规划从起点到终点的混合路径。

        Returns:
            HybridPath。success=False 时 waypoints 为空，failure_reason 说明原因。
            调用方必须检查 success 字段再决定是否执行。
        """
        t0 = time.time()

        # 1. 定位房间
        start_room = self._locate_room(start[:2])
        goal_room = self._locate_room(goal[:2])

        if start_room is None or goal_room is None:
            reason = (
                f"room lookup failed: start_room={start_room}, goal_room={goal_room} "
                f"(start={start[:2]}, goal={goal[:2]})"
            )
            logger.warning(reason)
            return self._failed(reason)

        # 2. 拓扑层 Dijkstra
        t_topo = time.time()
        cost, room_seq = self.tsg.shortest_path(start_room, goal_room)
        topo_time = time.time() - t_topo

        if cost == float("inf") or len(room_seq) == 0:
            reason = f"no topological path: room {start_room} → {goal_room}"
            logger.warning(reason)
            return self._failed(reason, topo_time=topo_time)

        logger.info("Topo path: %s cost=%.2f (%.1fms)", room_seq, cost, topo_time * 1000)

        # 3. 几何层 A*
        t_geom = time.time()
        segments: List[PathSegment] = []
        waypoints: List[np.ndarray] = [start.copy()]

        for i in range(len(room_seq) - 1):
            if time.time() - t0 > max_planning_time:
                reason = f"planning timeout at segment {i}/{len(room_seq)-1}"
                logger.warning(reason)
                return self._failed(reason, segments, waypoints,
                                    topo_time, time.time() - t_geom)

            seg = self._plan_local_segment(
                from_room_id=room_seq[i],
                to_room_id=room_seq[i + 1],
                start_pos=waypoints[-1],
                goal_pos=goal if i == len(room_seq) - 2 else None,
                search_radius_factor=search_radius_factor,
            )

            if seg is None:
                reason = (
                    f"A* failed for segment room {room_seq[i]} → {room_seq[i+1]} "
                    f"(start={waypoints[-1][:2]}) — "
                    f"path is BLOCKED, no straight-line fallback"
                )
                logger.error(reason)
                return self._failed(reason, segments, waypoints,
                                    topo_time, time.time() - t_geom)

            segments.append(seg)
            waypoints.extend(seg.waypoints[1:])

        geom_time = time.time() - t_geom
        total_time = time.time() - t0

        return HybridPath(
            success=True,
            waypoints=waypoints,
            room_sequence=room_seq,
            segments=segments,
            total_cost=sum(s.cost for s in segments),
            total_planning_time=total_time,
            topology_planning_time=topo_time,
            geometry_planning_time=geom_time,
            num_waypoints=len(waypoints),
            num_rooms=len(room_seq),
        )

    # ── 内部方法 ──────────────────────────────────────────────────────────────

    def _plan_local_segment(
        self,
        from_room_id: int,
        to_room_id: int,
        start_pos: np.ndarray,
        goal_pos: Optional[np.ndarray],
        search_radius_factor: float,
    ) -> Optional[PathSegment]:
        """规划两个房间之间的局部路径段。

        Returns:
            PathSegment，或 None（A* 不可达/超时/无地图）。
            绝不返回直线路径——None 就是 None。
        """
        t0 = time.time()

        from_room = self.tsg.get_node(from_room_id)
        to_room = self.tsg.get_node(to_room_id)
        if from_room is None or to_room is None:
            logger.warning("Room node missing: %s or %s", from_room_id, to_room_id)
            return None

        if self._trav is None:
            logger.warning(
                "No traversability grid — cannot plan segment %d→%d. "
                "Call update_traversability() before plan_path().",
                from_room_id, to_room_id,
            )
            return None

        # 目标位置（最后一段用真实 goal，中间段用目标房间中心）
        if goal_pos is None:
            goal_pos = np.array([to_room.center[0], to_room.center[1], start_pos[2]])

        # 搜索区域 bbox
        search_bbox = self._compute_search_bbox(from_room, to_room, search_radius_factor)

        # 世界坐标 → 格坐标
        si, sj = _w2g(start_pos[0], start_pos[1],
                       self._trav_cx, self._trav_cy, self._trav_res,
                       self._ox, self._oy, self._nx, self._ny)
        gi, gj = _w2g(goal_pos[0], goal_pos[1],
                       self._trav_cx, self._trav_cy, self._trav_res,
                       self._ox, self._oy, self._nx, self._ny)

        # 若起/终点在障碍格内，尝试在附近找最近可行格
        si, sj = _find_nearest_free(self._trav, si, sj, self._obs_thr)
        gi, gj = _find_nearest_free(self._trav, gi, gj, self._obs_thr)

        # bbox 裁剪：只在 search_bbox 对应的格范围内搜索（创建子视图）
        bi_min, bj_min = _w2g(search_bbox["x_min"], search_bbox["y_min"],
                               self._trav_cx, self._trav_cy, self._trav_res,
                               self._ox, self._oy, self._nx, self._ny)
        bi_max, bj_max = _w2g(search_bbox["x_max"], search_bbox["y_max"],
                               self._trav_cx, self._trav_cy, self._trav_res,
                               self._ox, self._oy, self._nx, self._ny)
        bi_min, bi_max = min(bi_min, bi_max), max(bi_min, bi_max)
        bj_min, bj_max = min(bj_min, bj_max), max(bj_min, bj_max)

        # 把全局格坐标映射到子视图坐标
        si_loc = int(np.clip(si - bi_min, 0, bi_max - bi_min))
        sj_loc = int(np.clip(sj - bj_min, 0, bj_max - bj_min))
        gi_loc = int(np.clip(gi - bi_min, 0, bi_max - bi_min))
        gj_loc = int(np.clip(gj - bj_min, 0, bj_max - bj_min))

        trav_sub = self._trav[bi_min:bi_max + 1, bj_min:bj_max + 1]
        if trav_sub.size == 0:
            logger.warning("Empty trav subgrid for segment %d→%d", from_room_id, to_room_id)
            return None

        # A* in subgrid
        cells = _astar_on_grid(
            trav_sub, (si_loc, sj_loc), (gi_loc, gj_loc),
            self._obs_thr, self._astar_timeout,
        )

        if cells is None:
            logger.warning(
                "A* found no path: room %d → %d "
                "(start_grid=(%d,%d) goal_grid=(%d,%d) subgrid=%s)",
                from_room_id, to_room_id,
                si_loc, sj_loc, gi_loc, gj_loc, trav_sub.shape,
            )
            return None

        # 降采样 + 格坐标 → 世界坐标
        cells = _downsample_cells(cells, self._min_ds)
        z = float(start_pos[2])
        pts: List[np.ndarray] = []
        for (ci, cj) in cells:
            wx, wy = _g2w(ci + bi_min, cj + bj_min,
                          self._trav_cx, self._trav_cy,
                          self._trav_res, self._ox, self._oy)
            pts.append(np.array([wx, wy, z]))

        # 强制首末点精确
        pts[0] = start_pos.copy()
        pts[-1] = goal_pos.copy()

        cost = sum(
            float(np.linalg.norm(pts[k + 1] - pts[k]))
            for k in range(len(pts) - 1)
        )

        return PathSegment(
            from_room_id=from_room_id,
            to_room_id=to_room_id,
            waypoints=pts,
            cost=cost,
            planning_time=time.time() - t0,
        )

    def _locate_room(self, position: np.ndarray) -> Optional[int]:
        """定位位置所在的房间（三级查询：get_room_by_position → 凸包 → 最近邻）。"""
        room_id = self.tsg.get_room_by_position(position[0], position[1], radius=5.0)
        if room_id is not None:
            return room_id

        for room in self.tsg.rooms:
            if room.convex_hull is not None and len(room.convex_hull) >= 3:
                if self._point_in_polygon(np.array(position), room.convex_hull):
                    return room.node_id

        rooms = self.tsg.rooms
        if not rooms:
            return None
        nearest = min(rooms, key=lambda r: float(np.linalg.norm(r.center - position)))
        return nearest.node_id

    def _compute_search_bbox(self, from_room, to_room, radius_factor: float) -> dict:
        """计算两个房间合并后的 bbox，再按 radius_factor 扩展。"""
        if from_room.bounding_box and to_room.bounding_box:
            x_min = min(from_room.bounding_box["x_min"], to_room.bounding_box["x_min"])
            x_max = max(from_room.bounding_box["x_max"], to_room.bounding_box["x_max"])
            y_min = min(from_room.bounding_box["y_min"], to_room.bounding_box["y_min"])
            y_max = max(from_room.bounding_box["y_max"], to_room.bounding_box["y_max"])
        else:
            centers = np.array([from_room.center, to_room.center])
            x_min = centers[:, 0].min() - 5.0
            x_max = centers[:, 0].max() + 5.0
            y_min = centers[:, 1].min() - 5.0
            y_max = centers[:, 1].max() + 5.0

        margin = ((x_max - x_min) + (y_max - y_min)) / 2 * (radius_factor - 1.0)
        return {
            "x_min": x_min - margin,
            "x_max": x_max + margin,
            "y_min": y_min - margin,
            "y_max": y_max + margin,
        }

    @staticmethod
    def _failed(
        reason: str,
        segments: Optional[List[PathSegment]] = None,
        waypoints: Optional[List[np.ndarray]] = None,
        topo_time: float = 0.0,
        geom_time: float = 0.0,
    ) -> HybridPath:
        """构造 success=False 的路径，携带失败原因。"""
        return HybridPath(
            success=False,
            waypoints=[],
            room_sequence=[],
            segments=segments or [],
            total_cost=float("inf"),
            total_planning_time=topo_time + geom_time,
            topology_planning_time=topo_time,
            geometry_planning_time=geom_time,
            num_waypoints=0,
            num_rooms=0,
            failure_reason=reason,
        )

    @staticmethod
    def _point_in_polygon(point: np.ndarray, polygon: np.ndarray) -> bool:
        """Ray casting 算法：检查点是否在多边形内。"""
        x, y = point[0], point[1]
        n = len(polygon)
        inside = False
        p1x, p1y = polygon[0]
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    @staticmethod
    def _point_in_bbox(point: np.ndarray, bbox: dict) -> bool:
        return (bbox["x_min"] <= point[0] <= bbox["x_max"] and
                bbox["y_min"] <= point[1] <= bbox["y_max"])


# ── 性能对比工具 ──────────────────────────────────────────────────────────────

@dataclass
class PlannerComparison:
    """规划器性能对比结果。"""
    hybrid_time: float
    baseline_time: float
    speedup: float
    hybrid_waypoints: int
    baseline_waypoints: int
    hybrid_cost: float
    baseline_cost: float
    hybrid_rooms: int
    hybrid_success: bool
    baseline_success: bool


def compare_planners(
    hybrid_planner: HybridPlanner,
    baseline_planner,
    test_cases: List[Tuple[np.ndarray, np.ndarray]],
) -> List[PlannerComparison]:
    """对比混合规划器和基线规划器的性能。

    Args:
        hybrid_planner:   HybridPlanner 实例
        baseline_planner: 基线规划器 (需要有 plan_path 方法)
        test_cases:       [(start, goal), ...]

    Returns:
        对比结果列表（含 success 字段，失败的 case 也会记录）
    """
    results = []
    for i, (start, goal) in enumerate(test_cases):
        logger.info("Test case %d/%d: %s → %s", i + 1, len(test_cases), start[:2], goal[:2])

        t0 = time.time()
        hybrid_result = hybrid_planner.plan_path(start, goal)
        hybrid_ms = (time.time() - t0) * 1000

        t1 = time.time()
        baseline_result = baseline_planner.plan_path(start, goal)
        baseline_ms = (time.time() - t1) * 1000

        results.append(PlannerComparison(
            hybrid_time=hybrid_ms,
            baseline_time=baseline_ms,
            speedup=baseline_ms / hybrid_ms if hybrid_ms > 0 else 0.0,
            hybrid_waypoints=hybrid_result.num_waypoints,
            baseline_waypoints=getattr(baseline_result, 'num_waypoints', 0),
            hybrid_cost=hybrid_result.total_cost,
            baseline_cost=getattr(baseline_result, 'total_cost', float("inf")),
            hybrid_rooms=hybrid_result.num_rooms,
            hybrid_success=hybrid_result.success,
            baseline_success=getattr(baseline_result, 'success', False),
        ))

    return results
