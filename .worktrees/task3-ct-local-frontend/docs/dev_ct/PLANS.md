# IAP 连续时间 B-spline 改造计划（Phase 1：Odom + Planner 接口）

**摘要**
- 将 `iap` 的 odometry 主线从离散帧状态迁移为连续时间 B-spline 轨迹，保留 GTSAM / gtsam_points 主栈，不把 `C-LIUO` 的 Ceres 后端直接作为 IAP 主线。
- 首期范围只做连续时间 `LiDAR + IMU + GNSS` estimator 与规划接口预留；`sub_mapping` / `global_mapping` / viewer 继续消费采样后的兼容输出。
- `C-LIUO` 作为连续时间 spline 结构、knot 管理、时间查询、边缘化策略与外部观测建模的参考模板；UWB 不进入首期目标传感器栈。
- spline 同时支持均匀与非均匀两种三次 B-spline，配置可切换，默认均匀。

**实现改动**
1. 轨迹内核
- 新增连续时间 odometry 模块，使用独立配置与独立动态库，和现有 `odometry_estimation_cpu/gpu/ct` 并存；旧 `config_odometry_ct.json` 保留为 legacy CT-GICP 基线，不复用其命名。
- 新增连续时间轨迹子系统，统一维护姿态控制点、位置控制点、knot 时间轴、bias、clock 与按时间查询的 pose / vel / acc / yaw / sigma 能力。
- spline policy 抽象成统一接口，首期同时实现 uniform 与 non-uniform 两套 knot 策略，默认 uniform，后续通过同一配置开关做 A/B 对比。
- 不改动 `EstimationFrame` 的 ABI 敏感核心布局；连续时间窗口、spline 句柄与额外元数据通过 `custom_data` 或 IAP 私有对象挂载，避免破坏现有 GLIM 兼容链路。

2. 连续时间 estimator
- 在 GTSAM 中把优化变量改为控制点窗口 + IMU bias + clock 状态，而不是继续以离散 `X(i)/V(i)/B(i)` 为主状态。
- LiDAR 约束改为按点时间查询 spline 位姿的连续时间残差；IMU 约束改为按 IMU 时间戳约束 spline 的角速度与线加速度；GNSS 伪距 / 多普勒改为按观测时间戳直接约束 spline 状态与 clock。
- GNSS 因子生成下沉到 odometry 内核；现有 `gnss_handler` 仅保留测量缓存、星历/可见性/预处理职责，不再负责连续时间主链的图注入。
- 新增 knot 插入、窗口滑动、旧控制点 marginalization 与 uniform/non-uniform 切换策略；实现目标是“连续时间主链替换离散 odometry 主链”，不是在现有离散 estimator 外再包一层采样器。
- odometry 仍需对外产出兼容 `EstimationFrame`：scan 参考时刻 pose、采样版 `imu_rate_trajectory`、deskewed frame、clock、sigma、ICP quality，保证 local/global mapping 与 viewer 首期不用重写。

3. 规划接口预留
- 新增只读接口 `ContinuousTrajectoryView`，供 planner 按时间查询当前估计 spline 的 pose / vel / acc / yaw / uncertainty。
- 新增低层接口 `SplineControlAccess`，暴露只读 knot / control-point window / spline meta / window clone 入口，为后续真正的 B-spline planner 预留底层访问能力。
- `PlannerInterface` 保持 `plan(...)` / `execution_target(...)` 主签名不变，但增加默认 no-op 的 `set_trajectory_view(...)` 与 `set_control_access(...)`，避免首期就破坏现有规划器调用方式。
- `CandidateTrajectory` 首期继续保留“采样点 + 预测 PL/AL”的兼容表示；真正的 spline candidate planning 放到下一阶段，通过上面的双层接口接入，不在首期重写完整 planner 内核。
- 首期 `IntegrityPlanner` 只需要能够读取 estimator 发布的连续时间轨迹与不确定度；下一阶段再把候选轨迹生成从 motion primitives 升级为 B-spline candidate。

4. 分阶段落地
- Phase 1A：连续时间轨迹内核、query API、uniform/non-uniform 开关、兼容 `EstimationFrame` 的采样层。
- Phase 1B：连续时间 LiDAR + IMU odometry 主链替换并跑通当前 viewer / local mapping。
- Phase 1C：GNSS 因子内核化并入连续时间 estimator，打通 clock / pseudorange / doppler 的时间戳约束。
- Phase 1D：planner 双层接口接线，现有 `IntegrityPlanner` 改为可读取连续时间轨迹。
- Phase 2：基于 `SplineControlAccess` 实现真正的 B-spline 规划器，替换纯离散 motion primitives 内核。

**公共接口 / 类型**
- 新增 `OdometryEstimationBSpline` 作为新的连续时间主模块。
- 新增 `TrajectorySample`，统一承载 `stamp, pose, vel, acc, yaw, sigma` 查询结果。
- 新增 `ContinuousTrajectoryView` 作为 planner / viewer / debug 的只读轨迹接口。
- 新增 `SplineControlAccess` 作为未来 planner 的控制点层接口。
- `PlannerInterface` 增加 `set_trajectory_view(...)` 与 `set_control_access(...)` 默认接口。
- `CandidateTrajectory` 首期不做破坏性重构，继续作为 planner 输出的兼容采样表示。

**测试与验收**
- 单元测试覆盖 spline query 正确性、uniform/non-uniform 一致性、LiDAR/IMU/GNSS Jacobian 数值校验、knot insertion 与 marginalization 连续性。
- 集成测试覆盖 `LiDAR+IMU` 连续时间 odometry 跑通、`LiDAR+IMU+GNSS` 连续时间 odometry 跑通、uniform vs non-uniform A/B 对比。
- 兼容性测试要求 `iap_rosnode` 能通过新配置启动，现有 `sub_mapping` / `global_mapping` / viewer 能直接消费连续时间 odometry 的采样输出。
- planner 验收要求现有 `IntegrityPlanner` 在不改主调用方式的前提下，能够读取 `ContinuousTrajectoryView` 并完成打分流程。
- 最终验收标准是仓库同时保留 legacy discrete、legacy CT-GICP、new B-spline CT 三条 odometry 路径，且 planner 接口对下一阶段 spline planning 不再需要破坏性修改。

**假设与默认**
- 首期范围固定为 `odometry + planner 接口`，不在本轮同时做 continuous-time local/global mapping 与 continuous-time planner optimization。
- 目标传感器栈固定为 `LiDAR + IMU + GNSS`；UWB 仅作为 `C-LIUO` 的结构与外部观测模式参考。
- 优化后端固定为 GTSAM；连续时间因子、变量组织与 marginalization 都按 GTSAM 方式落地。
- spline 规格固定为“三次 B-spline，uniform 与 non-uniform 都实现，默认 uniform”。
- 允许局部移植 `C-LIUO` 的 spline / 工具代码，但真正合入前必须逐文件核对许可证与归属；后端与因子逻辑优先按 IAP/GTSAM 语义重写。

## 2026-03-27 实施状态（本次落地）
- 已完成 Phase 1A 基础骨架：
  - 新增 `ContinuousTrajectoryView` / `SplineControlAccess` / `TrajectorySample` / `SplineWindowSnapshot`
  - 新增 `BSplineTrajectory`，支持 uniform / non-uniform 两种三次 knot policy、时间查询、窗口快照导出
  - 新增 `OdometryEstimationBSpline` 模块与 `config_odometry_bspline.json`
  - 连续时间窗口已通过 `IapSharedState` 与 `EstimationFrame::custom_data` 对 planner / viewer / debug 发布
  - `PlannerInterface` 与 `IntegrityPlanner` 已预留并接通连续时间轨迹接口
- 当前实现仍属于“连续时间骨架层”，不是最终的 spline-native estimator：
  - LiDAR / IMU / GNSS 因子仍未改为基于控制点窗口的连续时间残差
  - GNSS 因子仍未内核化下沉到 `OdometryEstimationBSpline`
  - 旧 local/global mapping 目前继续消费兼容采样输出
- 下一步优先级：
  - Phase 1B：把 LiDAR + IMU 主链从离散状态推进到 spline-native factor graph
  - Phase 1C：把 GNSS 伪距 / 多普勒时间戳约束并入 spline 窗口
  - Phase 1D：让现有 planner 在评分过程中真正利用 trajectory uncertainty / time query，而不只是读最新样本
