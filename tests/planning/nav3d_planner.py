#!/usr/bin/env python3
"""
nav3d_planner.py — 3D A* 体素路径规划器

使用建筑点云（PCL PCD）构建三维占用栅格，在自由体素空间中运行
A* 搜索，返回平滑后的 3D 路点列表。

用法:
    planner = Nav3DPlanner(voxel_size=0.25)
    planner.build(pts_xyz)           # pts_xyz: (N,3+) numpy array
    path = planner.plan(start, goal) # start/goal: (x,y,z) tuples/lists
    # path: [(x,y,z), ...] world coordinates, smoothed
"""

import math
import heapq
import numpy as np


class Nav3DPlanner:
    """3D A* 规划器，基于体素占用集合（Python set，O(1)查找）。"""

    def __init__(self, voxel_size: float = 0.25):
        self.voxel_size = voxel_size
        self.occupied: set = set()
        self.origin   = np.zeros(3, dtype=float)
        self.bounds   = np.zeros(3, dtype=int)
        self._built   = False

    # ── 构建占用栅格 ──────────────────────────────────────────────────────────

    def build(self, pts: np.ndarray) -> None:
        """从点云构建3D占用栅格。

        Args:
            pts: (N, 3+) float array，取前3列 x, y, z
        """
        xyz = np.asarray(pts, dtype=float)[:, :3]
        if len(xyz) == 0:
            return

        self.origin = xyz.min(axis=0) - self.voxel_size
        indices = np.floor((xyz - self.origin) / self.voxel_size).astype(int)
        self.bounds = indices.max(axis=0) + 2

        # 使用 set 存储占用体素坐标，支持 O(1) 查找
        self.occupied = set(map(tuple, indices.tolist()))
        self._built = True

    def clear(self) -> None:
        self.occupied.clear()
        self._built = False

    # ── 坐标转换 ──────────────────────────────────────────────────────────────

    def _w2v(self, xyz) -> tuple:
        """世界坐标 → 体素整数坐标"""
        arr = np.array(xyz, dtype=float)
        return tuple(np.floor((arr - self.origin) / self.voxel_size).astype(int).tolist())

    def _v2w(self, vxyz) -> np.ndarray:
        """体素坐标 → 体素中心世界坐标"""
        return self.origin + (np.array(vxyz, dtype=float) + 0.5) * self.voxel_size

    # ── 自由体素检查 ──────────────────────────────────────────────────────────

    def is_free(self, vxyz: tuple) -> bool:
        """检查体素是否在边界内且未占用。"""
        x, y, z = vxyz
        bx, by, bz = self.bounds
        if not (0 <= x < bx and 0 <= y < by and 0 <= z < bz):
            return False
        return vxyz not in self.occupied

    def _nearest_free(self, vxyz: tuple, radius: int = 10):
        """BFS搜索最近的自由体素（用于起/终点落在障碍体素时）。"""
        x, y, z = vxyz
        visited = set()
        queue = [(0, vxyz)]
        while queue:
            d, cur = heapq.heappop(queue)
            if cur in visited:
                continue
            visited.add(cur)
            if self.is_free(cur):
                return cur
            if d >= radius:
                continue
            cx, cy, cz = cur
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == dy == dz == 0:
                            continue
                        nb = (cx+dx, cy+dy, cz+dz)
                        if nb not in visited:
                            heapq.heappush(queue, (d+1, nb))
        return None

    # ── A* 搜索 ───────────────────────────────────────────────────────────────

    def plan(self, start, goal) -> list:
        """3D A* 路径规划。

        Args:
            start: (x,y,z) 起点世界坐标
            goal:  (x,y,z) 终点世界坐标

        Returns:
            [(x,y,z), ...] 平滑路点列表（世界坐标）
            若找不到路径，返回 [start, goal] 直线 fallback
        """
        if not self._built:
            return [list(start), list(goal)]

        sv = self._w2v(start)
        gv = self._w2v(goal)

        # 若起/终点在障碍体素内，找最近自由体素
        if not self.is_free(sv):
            sv = self._nearest_free(sv)
        if not self.is_free(gv):
            gv = self._nearest_free(gv)

        if sv is None or gv is None:
            return [list(start), list(goal)]

        if sv == gv:
            return [self._v2w(sv).tolist(), self._v2w(gv).tolist()]

        # A* 搜索
        def heuristic(v):
            return math.sqrt(sum((a-b)**2 for a, b in zip(v, gv)))

        open_set  = [(heuristic(sv), sv)]
        g_score   = {sv: 0.0}
        came_from = {}

        while open_set:
            _, cur = heapq.heappop(open_set)

            if cur == gv:
                return self._reconstruct_and_smooth(came_from, cur, sv)

            cx, cy, cz = cur
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        if dx == dy == dz == 0:
                            continue
                        nb = (cx+dx, cy+dy, cz+dz)
                        if not self.is_free(nb):
                            continue
                        step = math.sqrt(dx*dx + dy*dy + dz*dz)
                        ng   = g_score[cur] + step
                        if nb not in g_score or ng < g_score[nb]:
                            g_score[nb] = ng
                            f = ng + heuristic(nb)
                            heapq.heappush(open_set, (f, nb))
                            came_from[nb] = cur

        # 无路径 — 直线 fallback
        return [list(start), list(goal)]

    def _reconstruct_and_smooth(self, came_from, cur, start, min_dist: float = 0.5) -> list:
        """重建路径并平滑（移除过密路点）。"""
        raw = []
        while cur in came_from:
            raw.append(self._v2w(cur).tolist())
            cur = came_from[cur]
        raw.append(self._v2w(start).tolist())
        raw.reverse()

        if len(raw) <= 2:
            return raw

        # 保留间距 >= min_dist 的路点
        result = [raw[0]]
        for p in raw[1:-1]:
            d = math.sqrt(sum((a-b)**2 for a, b in zip(p, result[-1])))
            if d >= min_dist:
                result.append(p)
        result.append(raw[-1])
        return result
