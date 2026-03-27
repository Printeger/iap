# IAP 连续时间开发状态

## 更新时间
- 2026-03-27

## 当前结论
- 已完成 Phase 1A 的“连续时间骨架层”落地。
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
- 暂时复用现有 LiDAR-IMU odometry 主链作为状态来源。
- 在 `update_frames()` 之后，将活动窗口中的离散状态重建为一条连续时间 B-spline。
- 通过两条路径发布：
  - `IapSharedState`
  - `EstimationFrame::custom_data`
- 同时保留兼容性输出：
  - scan 参考时刻 pose
  - 采样版 `imu_rate_trajectory`
  - 现有 mapping / viewer 可继续消费的输出形式

注意：
- 这一层现在是“离散后重建 continuous trajectory”，不是“优化变量本身就是 spline control points”。

## 4. planner 接口预留
- `PlannerInterface` 已新增默认 no-op：
  - `set_trajectory_view(...)`
  - `set_control_access(...)`
- `IntegrityPlanner` 已接入连续时间轨迹视图读取逻辑

当前 planner 能做的事：
- 在不改 `plan(...)` 主签名的前提下读取共享的 continuous trajectory。
- 当前只基础使用了最新样本的不确定度，尚未在候选轨迹评分中深度使用完整 time query / spline window。

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
- `test_bspline_trajectory` 通过
- 总计 `32 tests, 0 errors, 0 failures`

## 当前还没完成的关键部分

### 1. 还不是 spline-native estimator
- 当前优化变量仍然主要来自旧的离散状态链路。
- 还没有把控制点窗口作为主优化变量写入 GTSAM。

### 2. LiDAR 连续时间残差还没接入主链
- 还没有把每点 `T(t)` 查询真正放进 LiDAR factor。
- 还没有实现基于控制点窗口的 continuous-time LiDAR Jacobian / Hessian 累加。

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

### Next Step 1：Phase 1B，先把连续时间 LiDAR + IMU 主链做成真的
目标：
- 不再只是“离散状态后重建 spline”
- 而是让 spline control points 成为 odometry 主状态的一部分

建议子任务：
- 定义控制点窗口的 key 组织方式
- 设计 knot insertion / window slide / marginalization 策略
- 先实现最小可用版本：
  - LiDAR-only 或 LiDAR+IMU 的 spline window optimizer
  - 保留现有兼容输出不变

### Next Step 2：优先改 LiDAR 因子
目标：
- 把 LiDAR 约束从“scan 参考 pose”推进到“按点时间查询 `T(t)`”

建议子任务：
- 在现有 deskew / factor 链路中识别最小改造入口
- 先做 CPU 版本连续时间 LiDAR factor
- 跑通之后再考虑 GPU 版本加速

### Next Step 3：再改 IMU 约束
目标：
- 用 IMU 时间戳直接约束 spline 的速度、角速度、加速度

建议子任务：
- 先明确 spline 状态表达是否采用：
  - pose control points
  - SE(3) split pose/position parameterization
  - 与 bias / clock 的联合状态布局
- 完成 IMU 残差与 Jacobian 数值校验

### Next Step 4：Phase 1C，把 GNSS 纳入连续时间窗口
目标：
- pseudorange / doppler 按观测时间直接约束 spline 状态与 clock

建议子任务：
- 明确 GNSS 因子从 extension 注入迁移到 odometry 内核的边界
- 保留 `gnss_handler` 的缓存/星历/预处理职责
- 将图注入职责逐步迁到 `OdometryEstimationBSpline`

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
1. 先做 Phase 1B 的状态设计与 key 设计
2. 先落一个 CPU 版连续时间 LiDAR factor
3. 再把 IMU 约束切到 spline window
4. 然后接 GNSS
5. 最后再增强 planner 的 spline 利用率

## 风险提醒
- 当前 B-spline 轨迹层已经可用于接口联调，但不能把它误认为“后端已经连续时间化”。
- 真正的工作量集中在：
  - LiDAR continuous-time factor
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
