把 BSpline CT odometry 从“frontend solve + backend solve”的显式两阶段结构，
改成“单一分层 factor-graph odometry/SLAM 子系统”。

要求：
- 只有一套优化问题/图状态体系
- frontend / backend 不再是两个独立优化器
- 不同层只决定：
  1. 当前活跃状态集合
  2. 添加哪些因子
  3. 哪一层参与本轮优化
- 但都属于同一个分层图优化系统