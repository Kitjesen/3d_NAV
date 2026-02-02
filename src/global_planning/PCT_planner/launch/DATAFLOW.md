# 3D导航系统 - 数据流简版

## 总览
```
激光雷达 → FASTLIO2 → /cloud_map → 地形分析 → /terrain_map → 局部规划 → /path → 路径跟踪 → /cmd_vel
                         ↑
                   PCT全局规划 → /pct_path → PCT适配器 → /way_point
```

## 关键话题
- `/cloud_map`：全局点云地图（最重要）
- `/terrain_map`：2.5D地形高度图
- `/pct_path`：全局路径
- `/way_point`：局部目标点
- `/path`：局部轨迹
- `/cmd_vel`：速度指令

## 简化流程（按时间）
```
1) 传感器：/livox/lidar
2) SLAM：/cloud_map + /Odometry
3) 地形分析：/terrain_map
4) 全局规划：/pct_path
5) 适配器：/way_point
6) 局部规划：/path
7) 路径跟踪：/cmd_vel
```

## 说明
- 统一使用 `/cloud_map`（全局地图），而不是 `/cloud_registered`（当前帧）。
- 2.5D导航只规划 X/Y，Z 由地形高度决定，适合地面机器人。