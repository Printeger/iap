# Commit 0
你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 0: 冻结迁移边界和主线配置”，不要做后续 commit 的事情。

目标：
- 为后续 explicit knot vector / non-uniform spline-native 重构建立护栏。
- 先明确 CPU path 为主线，GPU BUCKET 次之，GPU KERNEL 最后。
- 保留所有旧接口，保证当前工程可以单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 先保留旧接口兼容层，再逐步切主路径。
3. CPU path 优先，GPU BUCKET 第二，GPU KERNEL 最后。
4. LiDAR 保持 GICP/VGICP，不改成 LOAM feature frontend。
5. 最终目标是 explicit knot vector + non-uniform B-spline/NURBS evaluator + IMU/GNSS/LiDAR 统一依赖同一套 spline query。
6. 可以直接拷贝 @src/C-LIUO 代码，参考其架构思想。
7. 本 commit 不允许修改数学行为和残差模型，只允许加注释、加 TODO、加兼容配置和轻量整理。

请修改这些文件：
- config/config_odometry_bspline.json
- include/iap/odometry/odometry_estimation_bspline.hpp
- src/iap/odometry/odometry_estimation_bspline.cpp

具体要求：
1. 在 config/config_odometry_bspline.json 中，把默认 frontend_mode 改为 CT_LIDAR_CPU。
2. 保留 ct_lidar_gpu_backend 配置，但新增明确注释：
   - CPU 是重构主线
   - GPU BUCKET 是第二优先级
   - GPU KERNEL 是实验路径，最后适配
3. 在 odometry_estimation_bspline 的头文件和实现文件顶部新增迁移注释，说明：
   - 当前代码正从 “fixed 4-control-point local spline window” 迁移到 “explicit knot vector + unified spline evaluator”
   - 当前 commit 仅建立迁移边界，不改变现有算法
4. 不允许删除或重命名任何现有 public class / public method。
5. 不允许修改 ROS topic、plugin 名称、shared-state 接口、日志关键字。
6. 不允许引入新依赖。

验收标准：
- 工程能编译通过。
- 默认配置走 CT_LIDAR_CPU。
- 没有现有接口被删掉。
- 代码里出现清晰的迁移注释和 TODO 边界说明。

输出要求：
- 给出修改摘要
- 给出受影响文件清单
- 给出为什么这一步不改行为只建护栏
- 最后确认可以单独编译通过

# Commit 1
你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 1: 引入统一 spline core 类型，但先不接入优化”，不要做后续 commit 的事情。

目标：
- 引入 explicit knot vector / unified spline evaluator / sensor extrinsic + time offset 的基础数据结构。
- 这一步只新增 core 类型，不切主流程，不改现有残差，不删旧类。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 先保留旧接口兼容层，再逐步切主路径。
3. CPU path 优先，GPU BUCKET 第二，GPU KERNEL 最后。
4. LiDAR 保持 GICP/VGICP，不改成 LOAM feature frontend。
5. 最终目标是 explicit knot vector + non-uniform B-spline/NURBS evaluator + IMU/GNSS/LiDAR 统一依赖同一套 spline query。
6. 可以直接拷贝 @src/C-LIUO 代码，参考其 Trajectory / TrajectoryEstimator / NURBS 接口分层思想。
7. 本 commit 不允许改现有 odometry 主流程逻辑。

请新增文件：
- include/iap/odometry/spline_state_layout.hpp
- include/iap/odometry/spline_evaluator.hpp
- include/iap/odometry/spline_sensor_model.hpp
- src/iap/odometry/spline_evaluator.cpp

请轻量修改文件：
- include/iap/odometry/bspline_trajectory.hpp
- include/iap/planner/continuous_trajectory_view.hpp

请实现以下新类型和接口，名称可以微调，但语义必须一致：

1. 传感器模型
```cpp
enum class SplineSensorId { Imu, Lidar, Gnss };

struct SplineSensorModel {
  SplineSensorId id;
  Eigen::Isometry3d T_sensor_imu = Eigen::Isometry3d::Identity();
  double time_offset = 0.0;
};
```
局部支撑信息
```
struct SplineLocalSupport {
  int span_idx = -1;
  double query_time = 0.0;
  double u = 0.0;
  double dt = 0.0;
  std::array<std::size_t, 4> ctrl_indices{};
  std::array<gtsam::Key, 4> pose_keys{};
};
```
状态布局
```
class SplineStateLayout {
public:
  void set_knots(std::vector<double> knots);
  void set_controls(std::vector<BSplineControlPointState> controls);
  void set_sensor_model(SplineSensorId id, const SplineSensorModel& model);

  const std::vector<double>& knots() const;
  const std::vector<BSplineControlPointState>& controls() const;

  std::optional<SplineLocalSupport> support_at(double stamp, SplineSensorId sensor) const;
  std::vector<SplineLocalSupport> supports_in_range(double t0, double t1, SplineSensorId sensor) const;
};
```
统一 evaluator
```
class SplineEvaluator {
public:
  explicit SplineEvaluator(std::shared_ptr<const SplineStateLayout> layout);

  std::array<double, 4> basis(const SplineLocalSupport& support) const;
  std::array<double, 4> basis_d1(const SplineLocalSupport& support) const;
  std::array<double, 4> basis_d2(const SplineLocalSupport& support) const;

  gtsam::Pose3 eval_pose(const gtsam::Values& values,
                         const SplineLocalSupport& support,
                         SplineSensorId sensor) const;

  Eigen::Vector3d eval_world_velocity(const gtsam::Values& values,
                                      const SplineLocalSupport& support,
                                      SplineSensorId sensor) const;

  Eigen::Vector3d eval_world_acceleration(const gtsam::Values& values,
                                          const SplineLocalSupport& support,
                                          SplineSensorId sensor) const;
};
```
实现要求：

先支持 uniform cubic basis。
同时保留 explicit knots 数据结构，哪怕第一版 non-uniform evaluator 只是框架就绪。
support_at() 必须基于 knots 查找 span，不允许继续依赖“隐式 nominal_dt 推导 span”。
evaluator 第一版可以先只完整实现 pose / velocity / acceleration。
旧类、旧流程一律保留，不接入主路径。

验收标准：

新增 core 类型编译通过。
旧流程不受影响。
可以通过 layout + evaluator 查询某个时间戳对应的 span 和 basis。
不改变现有 odometry 行为。

输出要求：

给出新增类型摘要
给出所有新增/修改文件
说明哪些接口是为后续 IMU/GNSS/LiDAR 统一残差预留的
最后确认单独编译通过

---

# Commit 2


你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 2: 把 BSplineTrajectory 改成 explicit-knot adapter”，不要做后续 commit 的事情。

目标：
- 让 BSplineTrajectory 优先消费显式 knots + control states 的快照，而不是只接受 set_control_points() 后内部 rebuild_knots()。
- 保留旧接口兼容层。
- 不修改 odometry 主流程。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 保留旧接口兼容层。
3. 不删 set_control_points()，但把它降级为兼容 wrapper。
4. 本 commit 不接管优化器，只改 trajectory adapter / viewer-facing interface。

请修改文件：
- include/iap/odometry/bspline_trajectory.hpp
- src/iap/odometry/bspline_trajectory.cpp
- include/iap/planner/continuous_trajectory_view.hpp

请新增一个快照类型；如果已有相近类型，可复用并扩展：
```cpp
struct SplineWindowSnapshot {
  SplineMeta meta;
  std::vector<double> knots;
  std::vector<SplineControlPoint> control_points;
};
```
请把 BSplineTrajectory 的主接口扩展为：
```
void set_snapshot(const SplineWindowSnapshot& snapshot);
void set_layout(std::shared_ptr<const SplineStateLayout> layout,
                const gtsam::Values* values = nullptr);
```
兼容要求：

旧的
```
void set_control_points(const std::vector<SplineControlPoint>& control_points);
```
必须保留。
2. set_control_points() 内部可以构造一个 uniform snapshot，然后复用新主路径。
3. knot_vector() / sample() / sample_range() / clone_window() / latest_sample() 都必须优先基于显式 knots。
4. 只有当没有 snapshot/layout 时，才允许回退到旧逻辑。
5. rebuild_knots() 可以保留，但必须只服务于旧兼容路径，不再是主路径。

实现要求：

不改变 ContinuousTrajectoryView 对外语义。
meta() 需要能反映当前 snapshot/layout 的真实 knot mode。
sample() 内部优先走 evaluator / explicit knot support。
要保证 planner/viewer/debug 的现有调用能继续编译。

验收标准：

BSplineTrajectory 可以直接消费 snapshot 或 layout。
set_control_points() 仍然可用。
旧流程不崩。
工程单独编译通过。

输出要求：

说明新旧主路径分别是什么
说明哪些函数已经从“隐式 knot”切到“显式 knot”
给出兼容层保留原因
最后确认单独编译通过

---

# Commit 3


你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 3: 把 BSplineControlWindow 升级成显式 knot window manager”，不要做后续 commit 的事情。

目标：
- 保留类名 BSplineControlWindow，但把内部状态从“固定四控制点滑窗语义”升级为“显式 knot + active controls 的 window manager”。
- 同时扩展 fixed-lag registry 使其不再假设一个 segment 只绑定一个 std::array<4>。
- 先不删旧接口，保留 wrapper。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 不重命名 BSplineControlWindow。
3. 不删除现有 public method；必要时保留 wrapper 或 deprecated path。
4. 不改优化器的核心逻辑，只改 window / registry 的数据表示。

请修改文件：
- include/iap/odometry/bspline_control_window.hpp
- src/iap/odometry/bspline_control_window.cpp
- include/iap/odometry/bspline_fixed_lag_registry.hpp
- src/iap/odometry/bspline_fixed_lag_registry.cpp （若存在）
- include/iap/odometry/bspline_marginalization.hpp

请把 BSplineControlWindow 扩展为支持：
```cpp
void seed_uniform(double t0, double t1, const gtsam::Pose3& initial_pose);
void seed_with_knots(const std::vector<double>& knots,
                     const std::vector<gtsam::Pose3>& poses);

void extend_to(double new_end_time,
               const gtsam::Pose3& predicted_pose);

std::optional<SplineLocalSupport> support_at(double stamp) const;
std::vector<SplineLocalSupport> supports_in_range(double t0, double t1) const;

const std::vector<double>& knots() const;
const std::vector<BSplineControlPointState>& states() const;
```
兼容要求：

保留旧接口：
initialize()
advance()
evaluate(double u)
keys()
poses()
旧接口内部可以转调新数据结构，但不要删。
states() 允许从原来的固定数组迁移为 vector；必要时新增 legacy_states() 或 wrapper，但对外编译不能断。

registry 改造要求：
把当前 segment metadata 从“单一 4 控制点索引”扩成至少包含：
```
struct ActiveSegmentRecord {
  double stamp;
  double scan_end;
  int span_begin_idx;
  int span_end_idx;
  std::vector<std::size_t> active_control_indices;
  std::size_t auxiliary_index;
};
```
实现要求：

一个 segment / scan 可以跨多个 span。
fixed-lag registry 里必须能表达“本段依赖哪些活跃控制点”。
暂时仍允许 evaluator / factor 只消费单个 local support，但 registry 不能再只存 array<4> 假设。
不做行为性大改，不改残差装配主流程。

验收标准：

window manager 已经拥有显式 knot 状态。
registry 已具备表达 multi-span / active-control-set 的能力。
旧接口还能编译。
工程单独编译通过。

输出要求：

说明 window 内部状态如何变化
说明 registry 为什么必须摆脱固定四控制点假设
列出保留了哪些兼容接口
最后确认单独编译通过

---

# Commit 4

你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 4: 把 IMU factor 改成 spline-native，并统一依赖 SplineEvaluator”，不要做后续 commit 的事情。

目标：
- 让 IMU 因子不再持有 measurement_u / segment_duration / finite_difference_dt 这种旧 Phase-1 语义。
- 改成通过 SplineLocalSupport + SplineEvaluator 查询轨迹导数。
- 先允许 Jacobian 保持数值求导，但 basis 和导数必须统一来自 evaluator。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 先保留旧接口兼容层，再切主路径。
3. 本 commit 只处理 IMU factor 和其装配入口。
4. 不修改 GNSS / LiDAR factor。
5. 不改变 bias / gravity state 的整体组织方式。

请修改文件：
- include/iap/odometry/integrated_bspline_imu_factor.hpp
- src/iap/odometry/integrated_bspline_imu_factor.cpp
- include/iap/odometry/odometry_estimation_bspline.hpp
- src/iap/odometry/odometry_estimation_bspline.cpp

请新增统一上下文：
```cpp
struct SplineStampContext {
  SplineLocalSupport support;
  SplineSensorId sensor_id = SplineSensorId::Imu;
};
```
请新增新主构造函数：
```
class IntegratedSplineIMUFactor : public gtsam::NonlinearFactor {
public:
  IntegratedSplineIMUFactor(
      const SplineStampContext& ctx,
      gtsam::Key gyro_bias_key,
      gtsam::Key accel_bias_key,
      gtsam::Key gravity_key,
      const Eigen::Vector3d& measured_gyro,
      const Eigen::Vector3d& measured_accel,
      double accelerometer_precision,
      double gyroscope_precision,
      std::shared_ptr<const SplineStateLayout> layout);
};
```
兼容要求：

旧的 IntegratedBSplineIMUFactor 类可以保留。
旧构造函数也可以保留，但它必须尽量转调新主路径，或最少在实现中明确标记为 legacy path。
新主装配逻辑必须在 odometry_estimation_bspline.cpp 中启用。

实现要求：

因子内部不再直接持有 measurement_u、segment_duration、finite_difference_dt 作为主逻辑。
预测 gyro/accel 时必须通过 SplineEvaluator 的:
eval_pose()
eval_world_velocity()
eval_world_acceleration()
以及需要的话新增 eval_body_gyro()/eval_body_accel()
可以先使用 numeric Jacobian，但 numeric perturbation 必须围绕 active control poses，且 basis 来自 evaluator。
bias / gravity 残差维度和现有观测模型保持一致。
odometry_estimation_bspline.cpp 中，构造 IMU factor 时必须走 support_at(sample_time, Imu)。

验收标准：

IMU 主路径已经切到 spline-native query。
不再依赖 measurement_u / segment_duration 作为主接口。
旧接口还在，编译兼容。
工程单独编译通过。

输出要求：

说明旧 IMU factor 与新主路径的差异
说明 numeric Jacobian 现在依赖什么
列出为了兼容保留了哪些 legacy 接口
最后确认单独编译通过

---

# Commit 5

你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 5: 把 GNSS factor 切到统一 spline evaluator”，不要做后续 commit 的事情。

目标：
- 让 pseudorange / doppler 等 GNSS 因子统一通过 SplineEvaluator 查询轨迹状态。
- 轨迹层负责给出 GNSS 天线位姿/速度，GNSS 因子只做观测模型。
- 保留现有 clock / ECEF anchor / rotation / between-factor 路径。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 本 commit 不改变 clock state 的设计。
3. 本 commit 不改 LiDAR factor。
4. GNSS 观测模型尽量保持现有实现，只迁移几何查询入口。

请修改文件：
- include/iap/odometry/integrated_bspline_gnss_factor.hpp
- src/iap/odometry/integrated_bspline_gnss_factor.cpp
- include/iap/odometry/spline_sensor_model.hpp
- src/iap/odometry/odometry_estimation_bspline.cpp

请扩展 SplineSensorModel，使 GNSS 可表达天线外参：
```cpp
struct SplineSensorModel {
  SplineSensorId id;
  Eigen::Isometry3d T_sensor_imu = Eigen::Isometry3d::Identity();
  double time_offset = 0.0;
};
```
要求在 layout 中注册 Gnss 传感器模型。

请新增或替换主构造函数：
```
IntegratedSplinePseudorangeFactor(
    const SplineStampContext& ctx,
    gtsam::Key clock_key,
    gtsam::Key ecef_origin_key,
    gtsam::Key ecef_rot_key,
    const PseudorangeObservation& obs,
    std::shared_ptr<const SplineStateLayout> layout);

IntegratedSplineDopplerFactor(
    const SplineStampContext& ctx,
    gtsam::Key velocity_key,
    gtsam::Key clock_key,
    gtsam::Key ecef_rot_key,
    const DopplerObservation& obs,
    std::shared_ptr<const SplineStateLayout> layout);
```
实现要求：

GNSS 查询时间统一走 support_at(epoch_time, SplineSensorId::Gnss)。
轨迹位姿、天线位置、速度都通过 evaluator + sensor extrinsic 得出。
观测残差、信息矩阵、clock bias/drift 等现有模型尽量不变。
旧的 IntegratedBSplinePseudorangeFactor / DopplerFactor 若存在，可保留兼容层，但新装配主路径必须切到统一 evaluator。
odometry_estimation_bspline.cpp 中，GNSS factor 装配必须使用 SplineStampContext。

验收标准：

GNSS 主路径已使用统一 spline query。
clock / anchor / rotation 原有状态仍保留。
工程单独编译通过。

输出要求：

说明 GNSS 迁移了哪些“几何查询责任”
说明哪些 GNSS 逻辑没有改
列出保留的 legacy 接口
最后确认单独编译通过

---

# Commit 6


你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 6: 把 LiDAR factor 改成按 knot span / time bucket 的 spline-native GICP/VGICP 因子”，不要做后续 commit 的事情。

目标：
- 保持 LiDAR frontend 为 GICP/VGICP。
- 不做 LOAM feature frontend。
- 不做“一个 scan 一个跨多 span 的 mega factor”。
- 改成“按 knot span / time bucket 切分，每个 bucket 因子只依赖一个 SplineLocalSupport”。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. CPU path 优先，本 commit 先把 CPU 主路径改对。
3. 暂时不要适配 GPU backend，下一 commit 再做。
4. 保留现有 target ivox / vgicp / profiling / robust kernel / degeneracy diagnostics 思路。
5. LiDAR 必须继续使用 GICP/VGICP。

请修改文件：
- include/iap/odometry/integrated_bspline_gicp_factor.hpp
- src/iap/odometry/integrated_bspline_gicp_factor.cpp
- include/iap/odometry/odometry_estimation_bspline.hpp
- src/iap/odometry/odometry_estimation_bspline.cpp

请新增 bucket 上下文：
```cpp
struct SplineBucketContext {
  SplineLocalSupport support;
  SplineSensorId sensor_id = SplineSensorId::Lidar;
  std::vector<int> point_indices;
};
```
请把 CPU LiDAR 因子的主接口切成：
```
class IntegratedSplineGICPFactor : public gtsam::NonlinearFactor {
public:
  IntegratedSplineGICPFactor(
      const SplineBucketContext& ctx,
      std::shared_ptr<const gtsam_points::iVox> target,
      std::shared_ptr<const gtsam_points::PointCloud> source,
      std::shared_ptr<const gtsam_points::NearestNeighborSearch> target_tree = nullptr);
};
```
实现要求：

每个 bucket 只依赖一个 local support，对应 4 个 active pose keys。
一个 scan 先按点时间戳或 time bucket 切成多个 bucket。
odometry_estimation_bspline.cpp 中新增 bucket scheduler：
根据 scan 起止时间、点时间表、显式 knots，构造多个 SplineBucketContext
现有因子里如果有：
time_table_
basis_table_
time_indices_
之类缓存，可以迁到 bucket construction 阶段。
因子内部只对本 bucket 的点做残差与 Jacobian。
保留现有鲁棒核、剔除策略、profiling 接口。
旧构造函数可保留，但新主路径必须改成 bucket 化装配。

验收标准：

CPU LiDAR 因子主路径已经是 bucket 化、spline-native。
不再假设整帧 source 挂在一个固定 4 控制点窗口上。
GICP/VGICP 保持不变。
工程单独编译通过。

输出要求：

说明为什么不能做 mega factor
说明 bucket scheduler 在哪里实现
说明哪些旧接口保留了兼容层
最后确认单独编译通过

---

# Commit 7


你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 7: 重写 odometry_estimation_bspline 的调度层，使其围绕 layout/evaluator 装配因子”，不要做后续 commit 的事情。

目标：
- 把 odometry_estimation_bspline.cpp 从“旧 control window 驱动的装配器”改成“layout + explicit knots + evaluator 驱动的装配器”。
- 这一 commit 不改残差数学，只改主流程的装配责任。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 不修改外部 plugin 边界、shared state、frame attachment、viewer 发布接口的外部名称。
3. IMU/GNSS/LiDAR 的新主路径已经存在时，优先统一走 SplineStateLayout + SplineEvaluator。
4. 旧流程能保留兼容 fallback，但不能再是主路径。

请修改文件：
- include/iap/odometry/odometry_estimation_bspline.hpp
- src/iap/odometry/odometry_estimation_bspline.cpp

请按以下职责重构：

一、初始化阶段
- 初始化 SplineStateLayout
- 初始化显式 knots 和初始 control poses
- 注册 Imu / Lidar / Gnss 的 SplineSensorModel
- 初始化统一 evaluator

二、每帧推进阶段
- 把原来对 control window 的推进，改成：
  1. 更新或扩展显式 knot window
  2. 刷新 active control states
  3. 更新 layout

三、因子装配阶段
- IMU: 对每个 sample 调用 support_at(sample_time, Imu)
- GNSS: 对每个 epoch 调用 support_at(epoch_time, Gnss)
- LiDAR: 先做 prepare_scan_supports(raw_frame)，生成 bucket contexts，再装配多个 GICP/VGICP 因子

四、发布轨迹阶段
- 不再从 control_buffer().spline_control_points() 逆向重建轨迹
- 直接构造 SplineWindowSnapshot{meta, knots, control_points}
- 用 BSplineTrajectory::set_snapshot() 或 set_layout() 发布共享轨迹视图

兼容要求：
1. 旧字段和旧 helper 可以保留，但新增注释表明已是 legacy path。
2. shared_state、planner、viewer 的对外对象类型不要变。
3. ROS2 runtime 接口和 topic 不要改。

验收标准：
- odometry_estimation_bspline 已围绕 layout/evaluator 装配因子。
- 轨迹发布走显式 knot snapshot。
- 旧流程至多作为 fallback。
- 工程单独编译通过。

输出要求：
- 给出新的主流程分层
- 列出初始化、推进、因子装配、轨迹发布四块的主要变化
- 说明哪些 legacy 入口暂时还留着
- 最后确认单独编译通过
# Commit 8
你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 8: 重写 marginalization / fixed-lag active-state partition，使其面向 active keys 而非固定四控制点 segment”，不要做后续 commit 的事情。

目标：
- 让 fixed-lag marginalization 从“segment.control_indices + auxiliary_index”的旧 Phase-1 模式，升级成“active span / active control set / active keys”的模式。
- carried prior 机制尽量保留，不推翻重来。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 不重写整个 carried prior 框架。
3. 本 commit 只重写 partition / active-set 逻辑。
4. 不改 IMU/GNSS/LiDAR 残差模型。

请修改文件：
- include/iap/odometry/bspline_marginalization.hpp
- src/iap/odometry/bspline_marginalization.cpp
- include/iap/odometry/bspline_fixed_lag_registry.hpp
- src/iap/odometry/odometry_estimation_bspline.cpp

请新增或重构活跃状态集合：
```cpp
struct SplineActiveStateSet {
  std::vector<std::size_t> active_control_indices;
  std::vector<std::size_t> removable_control_indices;
  std::vector<gtsam::Key> active_pose_keys;
  std::vector<gtsam::Key> active_aux_keys;
  std::vector<gtsam::Key> removable_keys;
};
```
实现要求：

只要某个 control point 仍被任何 active span / bucket / IMU / GNSS 观测引用，就不能删。
segment / bucket 不再是 marginalization 的根本单元，active key set 才是。
registry 必须能查询：
哪些 span 仍活跃
哪些 control points 被哪些观测引用
carried prior（例如 BSplineCarriedPrior 或等价结构）尽量保留，不改其基本数据流。
odometry_estimation_bspline.cpp 中，边缘化前必须先构造 SplineActiveStateSet，再决定 drop set。
可以保留旧 partition helper，但不得作为主逻辑。

验收标准：

active/removable control key 判定正确。
multi-span / bucket 情况下不会误删控制点。
carried prior 还能工作。
工程单独编译通过。

输出要求：

说明为什么旧 segment 粒度已经不够
说明 active-state partition 的新规则
说明 carried prior 保留了什么
最后确认单独编译通过

---

# Commit 9


你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 9: 加入 @src/C-LIUO 风格的 adaptive / non-uniform knot placement 策略”，不要做后续 commit 的事情。

目标：
- 让 non-uniform 真正生效。
- 引入基于 IMU 活跃度的 knot density 调度策略。
- 保留 uniform 模式。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 可以直接拷贝 @src/C-LIUO 代码，参考其“动态 control point placement + short sliding-window”思想。
3. 不改残差模型，只改 knot placement policy。
4. uniform 模式必须继续可运行。

请新增文件：
- include/iap/odometry/spline_knot_policy.hpp
- src/iap/odometry/spline_knot_policy.cpp

请修改文件：
- src/iap/odometry/odometry_estimation_bspline.cpp
- config/config_odometry_bspline.json
- 若需要，轻量修改 include/iap/odometry/bspline_control_window.hpp

请新增配置项：
```json
"spline_knot_policy": "IMU_ACTIVITY",
"spline_min_dt": 0.03,
"spline_max_dt": 0.15,
"spline_extend_horizon": 0.2,
"spline_activity_gyro_gain": 1.0,
"spline_activity_acc_gain": 1.0,
"spline_target_density_coarse": 1,
"spline_target_density_fine": 4
```
请实现策略接口，例如：
```
struct KnotPlacementDecision {
  std::vector<double> new_knots;
  double recommended_dt = 0.1;
};

class SplineKnotPolicy {
public:
  virtual ~SplineKnotPolicy() = default;
  virtual KnotPlacementDecision decide(
      double current_end_time,
      double target_end_time,
      const std::vector<IMUSample>& imu_samples) const = 0;
};
```
实现要求：

基于短 horizon 内的 IMU gyro/acc 活跃度决定控制点密度。
高动态缩小 dt，低动态放宽 dt。
输出的是显式新 knots，而不是只改 nominal_dt 参数。
non_uniform 模式下，window extend 必须真正使用 policy 的输出。
uniform 模式继续走等间距 knot。
给出清晰注释，说明这是面向后续 spline-native fixed-lag 的 knot placement policy。

验收标准：

uniform / non_uniform 两种模式都能编译并运行。
non_uniform 模式下，显式 knots 会根据 IMU 活跃度变化。
工程单独编译通过。

输出要求：

说明新的 knot policy 如何工作
说明 uniform 与 non_uniform 的切换点
说明这一步为什么不改残差却很关键
最后确认单独编译通过

---

# Commit 10


你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 10: 回补 GPU BUCKET 路径，使其对齐新的 SplineBucketContext 和统一 spline query”，不要做后续 commit 的事情。

目标：
- 在 CPU 主路径已经重构之后，恢复 GPU BUCKET 路径。
- BUCKET 必须优先支持。
- GPU KERNEL 暂时只做接口对齐准备，不做完整重写。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. GPU BUCKET 第二优先级，GPU KERNEL 最后。
3. 不改 LiDAR 为 LOAM feature frontend。
4. 保持 GICP/VGICP。
5. BUCKET 主路径必须对齐新的 SplineBucketContext。

请修改文件：
- include/iap/odometry/integrated_bspline_gicp_factor_gpu.hpp
- src/iap/odometry/integrated_bspline_gicp_factor_gpu.cpp
- src/iap/odometry/odometry_estimation_bspline.cpp

如存在，也请轻量修改：
- include/iap/odometry/integrated_bspline_gicp_factor_gpu_common.hpp
- src/iap/odometry/integrated_bspline_gicp_factor_gpu_common.cpp

请把 GPU BUCKET 的主接口对齐到：
```cpp
class IntegratedSplineGICPFactorGPU : public gtsam::NonlinearFactor {
public:
  IntegratedSplineGICPFactorGPU(
      const SplineBucketContext& ctx,
      std::shared_ptr<const gtsam_points::iVox> target,
      std::shared_ptr<const gtsam_points::PointCloud> source,
      ...);
};
```
实现要求：

GPU BUCKET 路径必须使用和 CPU 一样的 bucket 调度结果。
bucket 内的 active 4-control support 来自显式 knot 查询，而不是旧 control window 假设。
保留原有 profiling / baseline csv / linearization check / numeric reference 等工程接口。
odometry_estimation_bspline.cpp 中，当 frontend_mode=CT_LIDAR_GPU 且 backend=BUCKET 时，走新的 bucket 化 GPU 路径。
不要在本 commit 里重做 GPU kernel-level spline evaluator。

验收标准：

GPU BUCKET 编译通过。
GPU BUCKET 与新 bucket scheduler 对齐。
CPU 路径不受影响。
工程单独编译通过。

输出要求：

说明 GPU BUCKET 如何复用 CPU bucket scheduler
说明你保留了哪些 profiling/diagnostic 能力
说明哪些 KERNEL 相关内容还未处理
最后确认单独编译通过

---

# Commit 11


你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 11: 对齐 GPU KERNEL 接口到统一 spline query，但保持 experimental”，不要做后续 commit 的事情。

目标：
- 让 GPU KERNEL 的接口与新的 explicit-knot / SplineBucketContext / unified spline evaluator 语义一致。
- 这一步以“可编译、接口对齐、保留 experimental 标记”为目标。
- 不要求本 commit 达到成熟优化水平。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. GPU KERNEL 是实验路径。
3. 不破坏 CPU 和 GPU BUCKET 主路径。
4. 不允许把 KERNEL 做成默认主线。

请修改文件：
- include/iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.hpp
- src/iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.cpp
- src/iap/odometry/odometry_estimation_bspline.cpp
- 如需要，轻量修改 config/config_odometry_bspline.json 注释

实现要求：
1. GPU KERNEL 的输入语义必须改成基于：
   - SplineBucketContext
   - 显式 knot
   - active control keys from support
2. 若现有 KERNEL 路径仍有大量旧 `measurement_u / segment_duration / fixed window` 假设，请保留兼容 wrapper，但新增主构造语义。
3. 在代码和配置注释中明确：
   - experimental
   - interface-aligned
   - optimization/performance tuning is follow-up
4. odometry_estimation_bspline.cpp 中保留 backend=KERNEL 的分支，但不要设为默认。
5. 不影响 BUCKET 和 CPU 编译。

验收标准：
- GPU KERNEL 至少接口上与 unified spline query 对齐。
- 保持 experimental。
- 工程单独编译通过。

输出要求：
- 说明 KERNEL 现在达到了什么程度
- 说明为什么仍然标 experimental
- 说明它与 BUCKET 的关系
- 最后确认单独编译通过
# Commit 12
你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 12: 清理旧接口并保留最小兼容层”，不要做后续 commit 的事情。

目标：
- 在新主路径已经建立后，清理明显过时的旧接口。
- 但仍保留最小兼容 wrapper，防止外部模块直接断掉。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 只能删除明确已被新主路径替代、且有兼容 wrapper 的旧内部路径。
3. 不允许删掉外部插件边界和 ROS2 runtime 对外接口。
4. 必须保留最小兼容层和清晰的 deprecated 注释。

请修改文件（按实际需要）：
- include/iap/odometry/bspline_control_window.hpp
- src/iap/odometry/bspline_control_window.cpp
- include/iap/odometry/bspline_trajectory.hpp
- src/iap/odometry/bspline_trajectory.cpp
- include/iap/odometry/integrated_bspline_imu_factor.hpp
- src/iap/odometry/integrated_bspline_imu_factor.cpp
- include/iap/odometry/integrated_bspline_gnss_factor.hpp
- src/iap/odometry/integrated_bspline_gnss_factor.cpp
- include/iap/odometry/integrated_bspline_gicp_factor.hpp
- src/iap/odometry/integrated_bspline_gicp_factor.cpp

清理原则：
1. 旧式主接口若依赖：
   - measurement_u
   - segment_duration
   - finite_difference_dt as primary API
   - implicit rebuild_knots as primary path
   - fixed single-segment array<4> assumptions
   可降级为 deprecated wrapper。
2. 对外仍保留最小兼容层：
   - 旧类名可以保留
   - 旧构造函数可以保留，但必须显式标注 legacy/deprecated，并尽量转调新主路径
3. 删除明显死代码、重复 helper、已经不再调用的旧内部实现。
4. 保留详细迁移注释，说明推荐新接口是什么。

验收标准：
- 新主路径清晰。
- 旧接口只剩最小兼容层。
- 工程单独编译通过。

输出要求：
- 列出删掉了哪些旧内部实现
- 列出保留了哪些兼容 wrapper
- 说明推荐的新接口清单
- 最后确认单独编译通过
# Commit 13
你在修改仓库 @src/iap 的 dev/ct-iap 分支代码。请只完成“Commit 13: 补测试、回归检查和文档，让 spline-native 主线可维护”，不要做后续 commit 的事情。

目标：
- 为新的 explicit knot / non-uniform / unified spline query 主线补足最小可维护性。
- 增加单元测试、回归测试钩子、以及开发文档。
- 必须单独编译通过。

硬性约束：
1. 每个 commit 必须单独编译通过。
2. 不改变算法逻辑，只补测试和文档。
3. 优先覆盖：
   - support_at / span lookup
   - uniform vs non_uniform knot behavior
   - IMU/GNSS/LiDAR factor 统一 query 的基本正确性
   - fixed-lag active-state partition 不误删控制点

请新增或修改测试/文档文件（按仓库结构调整）：
- tests/test_spline_state_layout.cpp
- tests/test_spline_evaluator.cpp
- tests/test_bspline_marginalization.cpp
- tests/test_uniform_vs_nonuniform_knots.cpp
- README 或 docs/continuous_time_bspline_design.md

测试要求：
1. 测试 support_at() 能正确找到显式 knot span。
2. 测试 uniform / non_uniform 两种模式下 knots 数量和间距行为不同。
3. 测试 evaluator 的 basis / basis_d1 / basis_d2 至少在简单案例下数值合理。
4. 测试 active-state partition 在 multi-span / bucket 情况下不会误删控制点。
5. 如已有测试框架，沿用；不要引入笨重新依赖。

文档要求：
1. 文档中明确当前主线是：
   - explicit knot vector
   - non-uniform capable
   - IMU/GNSS/LiDAR unified spline query
   - CPU primary, GPU BUCKET supported, GPU KERNEL experimental
2. 写清楚 legacy 兼容层仍然存在，但不是推荐主路径。
3. 给出关键类关系图或文字说明：
   - SplineStateLayout
   - SplineEvaluator
   - BSplineControlWindow
   - odometry_estimation_bspline
   - IntegratedSplineIMU/GNSS/GICP factors

验收标准：
- 测试能编译并通过。
- 文档清晰描述新架构。
- 工程单独编译通过。

输出要求：
- 给出新增测试清单
- 给出文档摘要
- 说明最关键的回归覆盖点
- 最后确认单独编译通过