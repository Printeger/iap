# IAP 连续时间开发状态

## 更新时间
- 2026-03-27

## 当前结论
- 已完成 Phase 1A 的“连续时间骨架层”落地。
- 已完成 Phase 1B 的最小可用版本：
  - 控制点窗口状态设计
  - 控制点 key 设计
  - CPU 连续时间 LiDAR factor
- 已补上 active fixed-lag spline window 骨架：
  - 多段控制点窗口缓存
  - lag pruning
  - 对 planner / viewer 发布整个 active control window
- 已把 active control window 推进到优化变量层：
  - 整窗控制点进入 LM 初值
  - 全窗 smoothness factors
  - 边界控制点 anchor / prediction priors
- 当前 `src/iap` 已具备：
  - 连续时间轨迹统一接口
  - B-spline 轨迹容器与时间查询能力
  - 新的 `OdometryEstimationBSpline` 模块骨架
  - planner 读取连续时间轨迹的接口预留与基础接线
- 当前实现还不是最终的 spline-native `LiDAR + IMU + GNSS` 连续时间 estimator。

## 本次已完成

### 1. 连续时间公共接口
- 新增 `ContinuousTrajectoryView`
- 新增 `SplineControlAccess`
- 新增 `TrajectorySample`
- 新增 `SplineWindowSnapshot`
- 新增 `ContinuousTrajectoryAttachment`

作用：
- 统一给 planner / viewer / debug 提供连续时间轨迹读取接口。
- 为下一阶段真正的 B-spline planner 和 spline-native odometry 留好 ABI 兼容入口。

## 2. B-spline 轨迹内核
- 新增 `BSplineTrajectory`
- 支持 `uniform` / `non_uniform` 两种 knot policy
- 支持：
  - `sample(stamp)`
  - `latest_sample()`
  - `sample_range(start, end, step)`
  - `knot_vector()`
  - `control_points()`
  - `clone_window()`

当前定位：
- 这是 Phase 1A 的轨迹表达层。
- 目前用于承载连续时间窗口、查询姿态/速度/加速度/不确定度，并服务 planner 接线与兼容输出。

## 3. 新 odometry 模块骨架
- 新增 `OdometryEstimationBSpline`
- 新增 `config/config_odometry_bspline.json`
- 新增 `libodometry_estimation_bspline.so` 创建入口

当前行为：
- 当前同时支持两条路径：
  - `RECONSTRUCT`：保留 Phase 1A 的离散后重建 spline 路径
  - `CT_LIDAR_CPU`：启用 4 控制点窗口 + CPU 连续时间 LiDAR factor 的最小主链
- `CT_LIDAR_CPU` 路径不再只是 `update_frames()` 后的后处理重建。
- 它会直接对活动控制点窗口做局部 LM 优化，再发布连续时间轨迹视图。
- 通过两条路径发布：
  - `IapSharedState`
  - `EstimationFrame::custom_data`
- 同时保留兼容性输出：
  - scan 参考时刻 pose
  - 采样版 `imu_rate_trajectory`
  - 现有 mapping / viewer 可继续消费的输出形式

注意：
- `RECONSTRUCT` 仍然是“离散后重建 continuous trajectory”。
- `CT_LIDAR_CPU` 已把 spline control points 引入为局部前端优化变量，但还不是最终的 fixed-lag smoother 主状态组织。

## 4. planner 接口预留
- `PlannerInterface` 已新增默认 no-op：
  - `set_trajectory_view(...)`
  - `set_control_access(...)`
- `IntegrityPlanner` 已接入连续时间轨迹视图读取逻辑

当前 planner 能做的事：
- 在不改 `plan(...)` 主签名的前提下读取共享的 continuous trajectory。
- 当前只基础使用了最新样本的不确定度，尚未在候选轨迹评分中深度使用完整 time query / spline window。

## 5. Phase 1B 最小可用实现
- 新增 `BSplineControlWindow`
- 新增 `BSplineControlWindowBuffer`
- 新增 4 控制点 key 设计：`symbol('s', idx)`
- 新增 `IntegratedBSplineGICPFactor`
- `OdometryEstimationBSpline` 新增 `frontend_mode = CT_LIDAR_CPU`

当前 Phase 1B 路径的工作方式：
- 不再只是“离散状态优化完成后再重建 spline”。
- 现在可以直接对 4 个活动控制点构成的窗口做连续时间 LiDAR 优化。
- 每个点按照点时间查询当前 segment 的 B-spline pose。
- 当前 active spline window 会跨多帧保留控制点，并随 `smoother_lag` 做裁剪。
- planner / viewer / `ContinuousTrajectoryView` 现在看到的是 active lag-window spline，而不是只有最新一个 segment。
- 当前 active spline window 中的全部控制点已经进入 LM 变量集合，而不再只是发布给 planner / viewer。
- 优化图目前包含：
  - 最新 segment 的 LiDAR factor
  - 全 active window 的 smoothness factors
  - active window 两端的 anchor / prediction priors
- CPU LiDAR factor 当前采用最小可用实现：
  - 4 个 pose control points
  - per-point time query
  - CPU GICP residual
  - 局部 LM 优化

当前 Phase 1B 的边界：
- 这还是 local frontend，不是最终的 fixed-lag smoother 主链。
- 新增的 fixed-lag window 现在已经进入优化变量层，但还没有把多段 LiDAR segment factors 一起放进同一优化图里。
- IMU 目前主要仍用于初始化/后续兼容链路，尚未作为 spline-native continuous-time factor 完整改写进图里。
- GNSS 也尚未进入控制点窗口主链。
- LiDAR factor 目前使用数值 pose Jacobian，是为了先打通最小可用版本；解析 Jacobian 和 GPU 版仍是后续工作。

## 验证状态

### 构建
- 已通过：
```bash
colcon build --packages-select iap --cmake-args -DBUILD_TESTING=ON
```

### 测试
- 已通过：
```bash
colcon test --packages-select iap --event-handlers console_direct+
colcon test-result --all
```

测试结果：
- `test_araim` 通过
- `test_bspline_control_window` 通过
- `test_bspline_trajectory` 通过
- `test_bspline_control_window` 现已覆盖 control buffer 扩展、lag pruning、values/update round-trip
- Phase 1B 代码已完成编译与测试通过

## 当前还没完成的关键部分

### 1. 还不是 spline-native estimator
- 当前 `CT_LIDAR_CPU` 路径已经把 4 控制点窗口作为局部优化变量引入。
- 当前 active spline window 已经具备 fixed-lag 形式的保存与裁剪。
- 当前 active spline window 已经进入优化变量层。
- 但它还不是最终的 fixed-lag GTSAM 主状态组织方式，也还不是多段 LiDAR factors 联合优化。

### 2. LiDAR 连续时间残差还没接入主链
- 当前已经有 CPU 版连续时间 LiDAR factor。
- 但它还是最小可用版本：
  - 当前主要只约束最新 segment
  - 虽然整窗控制点已经进入变量集合，但多段历史 segment 还没有各自的 LiDAR factor
  - 数值化 pose Jacobian
  - 还没有 GPU 版
  - 还没有与最终 fixed-lag 图结构完全统一

### 3. IMU 连续时间约束还没改写
- 还没有将 IMU 约束改为直接约束 spline 的角速度 / 线加速度。
- 当前 IMU 仍然主要走旧的预积分主线。

### 4. GNSS 还没下沉到 bspline odometry
- 还没有把 pseudorange / doppler 的时间戳约束直接并入 spline window。
- `gnss_handler` / `gnss_extension` 仍未完成向连续时间主链的内核化迁移。

### 5. planner 还没真正用 spline 做候选轨迹优化
- 当前只是接口通了。
- 还没有把 motion primitives 升级为 B-spline candidate planning。

## 下一步计划

### Next Step 1：把 `CT_LIDAR_CPU` 从 local frontend 推进到 fixed-lag spline 主链
目标：
- 让 spline control points 成为 odometry 主状态的一部分
- 不再只依赖每帧局部 LM，而是进入真正的“多段窗口联合优化 + 边缘化”

建议子任务：
- 为 active window 内的多个 segment 建立各自的连续时间 LiDAR factors
- 让多段相邻 segment 共享控制点并共同进入图优化
- 设计旧控制点边缘化与先验回灌策略
- 保留现有兼容输出不变

### Next Step 2：补齐 IMU 连续时间约束
目标：
- 用 IMU 时间戳直接约束 spline 的速度、角速度、加速度
- 让 IMU 不再只承担初始化和兼容链路职责

建议子任务：
- 明确 spline 状态表达是否采用纯 pose control points 还是 split pose/position parameterization
- 设计与 bias / clock 的联合状态布局
- 完成 IMU 残差与 Jacobian 数值校验

### Next Step 3：Phase 1C，把 GNSS 纳入连续时间窗口
目标：
- pseudorange / doppler 按观测时间直接约束 spline 状态与 clock

建议子任务：
- 明确 GNSS 因子从 extension 注入迁移到 odometry 内核的边界
- 保留 `gnss_handler` 的缓存/星历/预处理职责
- 将图注入职责逐步迁到 `OdometryEstimationBSpline`

### Next Step 4：补齐 LiDAR continuous-time factor 的工程化能力
目标：
- 让当前 LiDAR factor 从“最小可用”走向“可长期演进”

建议子任务：
- 评估解析 Jacobian 替换数值 Jacobian
- 设计 GPU 版 continuous-time LiDAR factor 的复用接口
- 对比 `RECONSTRUCT` / `CT_LIDAR_CPU` / legacy CT-GICP 的耗时与精度

### Next Step 5：Phase 1D，planner 真正开始消费 continuous-time info
目标：
- 让现有 `IntegrityPlanner` 不只是读最新样本 sigma
- 而是能读取：
  - 未来时刻 pose / vel / yaw
  - uncertainty
  - spline meta

建议子任务：
- 在不破坏当前 `CandidateTrajectory` 的前提下，增强 planner 评分阶段对 trajectory view 的使用
- 为下一阶段 spline candidate planner 做接口验证

## 推荐的下一次实际开发顺序
1. 把当前 `CT_LIDAR_CPU` 从 local LM frontend 推进到 fixed-lag spline window 主链
2. 将 IMU 约束真正切到 spline window
3. 把 GNSS 伪距 / 多普勒并入控制点窗口
4. 完善 LiDAR factor 的 Jacobian / GPU 演进路径
5. 最后增强 planner 对 continuous-time trajectory 的真实利用率

## 风险提醒
- 当前 B-spline 轨迹层已经可用于接口联调，但不能把它误认为“后端已经连续时间化”。
- 真正的工作量集中在：
  - 将当前 LiDAR continuous-time factor 升级为真正主链的一部分
  - IMU continuous-time constraint
  - GTSAM 中控制点窗口的组织与边缘化
  - GNSS 时间戳约束并入

## 相关文件
- `/home/dev/code/ws_iap/src/iap/include/iap/planner/continuous_trajectory_view.hpp`
- `/home/dev/code/ws_iap/src/iap/include/iap/odometry/bspline_trajectory.hpp`
- `/home/dev/code/ws_iap/src/iap/src/iap/odometry/bspline_trajectory.cpp`
- `/home/dev/code/ws_iap/src/iap/include/iap/odometry/odometry_estimation_bspline.hpp`
- `/home/dev/code/ws_iap/src/iap/src/iap/odometry/odometry_estimation_bspline.cpp`
- `/home/dev/code/ws_iap/src/iap/config/config_odometry_bspline.json`
- `/home/dev/code/ws_iap/src/iap/docs/dev_ct/PLANS.md`
