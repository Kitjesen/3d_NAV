LT 按下-1  松开 1 axes[2]..要一直按着 LT（axes[2]）才会保持自主模式；一松手就变成手动。
扳机释放（axes[2] > -0.1）→ autonomyMode_ = false → 手动模式
扳机按下（axes[2] < -0.1）→ autonomyMode_ = true → 自主模式


axes[5] (RT): 控制是否避障
// 松开 RT = 启用避障（默认安全）；按下 RT = 关闭避障（强制通过）

buttons[5] 按下时，把 clearingCloud_ 设为 true，后续在体素更新时会清除近距离点云（相当于清图/重建）

手柄速度不是直接发到底盘，而是作为 local_planner 的参考输入，
最终由 pathFollower 融合路径跟踪结果后发布 /cmd_vel。


/terrain_map_ext 目前还没有被任何节点订阅

##n
oRotAtStop_ = false
当 /stop 信号触发紧急停止时：

false：机器人可以原地旋转调整朝向（避障需要）
true：完全锁死，不转
场景：自动送货机器人

1. 正常运行: autonomyMode_=true, noRotAtStop_=false
   - 自动跟随路径，遇到障碍可以旋转避让
2. 紧急停止: /stop=2 (完全停止)
   - 如果 noRotAtStop_=false: 可以原地旋转寻找出路
   - 如果 noRotAtStop_=true:  完全锁死，等待人工
3. 到达目标: noRotAtGoal_=true
   - 精确到达后停止，不原地打转