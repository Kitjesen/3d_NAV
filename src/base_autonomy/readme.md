LT 按下-1  松开 1 axes[2]..要一直按着 LT（axes[2]）才会保持自主模式；一松手就变成手动。
扳机释放（axes[2] > -0.1）→ autonomyMode_ = false → 手动模式
扳机按下（axes[2] < -0.1）→ autonomyMode_ = true → 自主模式


axes[5] (RT): 控制是否避障
// 松开 RT = 启用避障（默认安全）；按下 RT = 关闭避障（强制通过）
