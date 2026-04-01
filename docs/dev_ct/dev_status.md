# IAP 连续时间开发状态

## 更新时间
- 2026-04-01

## 执行入口
- 后续连续时间 SLAM 开发以 [SLAM_FINISH_PLAN.md](/home/dev/code/ws_iap/src/iap/docs/dev_ct/SLAM_FINISH_PLAN.md) 为唯一执行入口。
- 本文档从现在开始只负责记录“当前状态、刚完成的增量和仍未完成的关键边界”，不再单独维护另一套并行开发计划。

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
- 已把 active window 内多个 segment 接入同一个 fixed-lag LM 图：
  - 每个 active segment 各自一条连续时间 LiDAR factor
  - 相邻 segment 通过共享控制点产生联合优化
- 已补上更严格的 fixed-lag target / prior 策略：
  - 每个 active segment 冻结 local target snapshot
  - lag-window 起始边界使用显式 marginal prior
- 已把最小可用的连续时间 IMU 因子接进同一个 fixed-lag LM 图：
  - per-segment IMU sample factor
  - 直接按 IMU 时间戳约束 gyro / accel residual
  - shared gyro bias / accel bias / gravity graph states
  - 与 LiDAR factors 共享同一组控制点变量
- 已开始 Phase 1C 的最小可用接入：
  - `gnss_extension` 已通过 shared state 发布原始 GNSS epoch mailbox 与 ECEF anchor
  - `OdometryEstimationBSpline` 已在 active segment 上直接挂接 continuous-time pseudorange / doppler factor
  - per-segment clock state、ECEF origin state、ECEF rotation state 已进入同一个 fixed-lag LM 图
  - GNSS clock-between factor 已在 active segment clock states 间接通
  - GNSS epoch 现在按 segment 时间窗而不是按单一 frame stamp 被消费，更接近连续时间窗口约束
  - `OdometryEstimationBSpline` 现在已经直接内聚自己的 `GnssHandler`
  - shared state 现在只保留“原始 GNSS epoch 邮箱 + anchor”职责，不再承担 segment-range drain 语义
  - `OdometryEstimationBSpline` 现在还直接内聚了 `GnssEpochBuilder`
  - `gnss_extension` 已开始退回为“raw GNSS ingress + legacy bridge”：原始观测 batch、星历更新、iono 参数先进入 shared mailbox，再由 BSpline odometry 在内部完成 epoch 组包与预处理
- 已开始按 [SLAM_FINISH_PLAN.md](/home/dev/code/ws_iap/src/iap/docs/dev_ct/SLAM_FINISH_PLAN.md) 执行 `M1 / WP1`：
  - lag-window boundary prior 现在开始覆盖 boundary auxiliary state，而不再只覆盖最前两个 pose 控制点
  - boundary velocity / clock state 已被结构化快照，并参与下一轮 fixed-lag prior 约束
  - 上一轮求解结束后，现在会对 boundary pose / velocity / clock 子集提取 joint marginal information，并缓存对应 linearization point
  - 下一轮 fixed-lag 求解会优先把这组 boundary information 以 `LinearContainerFactor(HessianFactor, linearization_point)` 的形式回灌到图里
  - 当前又进一步推进到了 removable-factor survivor marginalization：上一轮 carried prior 与即将被 lag pruning 移除的旧 segment 因子，会先在当前图中完成优化，再统一边缘化成新的 survivor prior
  - lag pruning 现在发生在 survivor prior 提取之后，而不是建图之前，这样滑出状态和滑出因子的有效信息不会在求解前直接丢掉
  - 已新增稳定的 `BSplineMarginalizationPartition` 分区工具，统一判定 control points、auxiliary states 和 shared IMU/GNSS alignment states 的 survivor/removable 归属
  - 已新增 `build_bspline_carried_prior(...)` 工具，将 removable nonlinear graph 线性化、边缘化到 survivor key 集合，并重新封装成可重放的 carried prior
  - 已补上针对 carried prior 的专门单测，验证 survivor/removable 分区逻辑以及 carried prior 与参考 marginal graph 的误差一致性
  - 当前又进一步收口到了更严格的 state/factor ownership：factor 归属现在统一经过 `BSplineMarginalizationPartition::classify_factor(...)`，并显式区分 `survivor-only / removable / foreign`
  - carried prior 现在以线性图形式缓存，只在 replay 到当前优化图时才转成 `LinearContainerFactor`；下一轮 prior 构造不再把上一轮 carried prior 重新塞回 removable nonlinear graph
  - 已补上 carried prior “线性 prior + 新 removable nonlinear 子图”组合测试，验证 prior 组合后对 survivor states 的误差与参考 marginal graph 一致
  - 已新增统一的 `BSplineFixedLagStateRegistry`，把 active control buffer、segment 生命周期和 auxiliary state 保留/裁剪规则收口到同一个 fixed-lag registry
  - `OdometryEstimationBSpline` 现在通过这套 registry 管理 window append、segment append、lag pruning 和 aux-value filtering，不再分别在 `control_buffer_`、`active_segment_constraints_` 和 `latest_ct_aux_values_` 上散落维护
  - 已补上 `test_bspline_fixed_lag_registry`，覆盖 control-buffer / segment 生命周期同步裁剪、aux-value filtering 和 reset/append 语义
  - unified registry 现已进一步扩展到 shared fixed-lag states：`gyro bias / accel bias / gravity / ECEF origin / ECEF rotation` 的 seed、回填和图内 ownership 现在也由 registry 统一管理
  - `OdometryEstimationBSpline` 不再单独持有这些 shared-state snapshot；shared IMU/GNSS states 现在和 active window 一样由 fixed-lag registry 统一提供给建图、优化后回写和持续发布链路
  - `test_bspline_fixed_lag_registry` 现已补充 shared-state seed/update round-trip，覆盖 persistent IMU/GNSS shared-state lifecycle
  - fixed-lag registry 现已进一步提供显式 diagnostics / lifecycle telemetry / state-machine 输出：`BSplineFixedLagTelemetry` 会统一报告当前 lag 区间、active segment、aux/shared state 数量、GNSS anchor 状态以及 `Empty / WindowSeeded / TrackingLidar / TrackingLidarGnss` 生命周期阶段
  - `OdometryEstimationBSpline` 现已在每轮 continuous-time 发布链路中同步输出这组 fixed-lag telemetry：一方面写入 `IapSharedState`，供 planner / viewer / debug 读取；另一方面以 trace 日志持续暴露当前 fixed-lag 生命周期状态
  - `test_bspline_fixed_lag_registry` 现已补充 lifecycle telemetry 单测，覆盖 `Empty -> WindowSeeded -> TrackingLidar -> TrackingLidarGnss` 的状态机迁移和计数语义
- 已把 velocity 提升为 fixed-lag 图中的显式状态：
  - 每个 active segment 通过 `symbol('u', idx)` 持有 velocity state
  - 新增 velocity consistency factor 将 pose spline 与 velocity state 绑定
  - `EstimationFrame::v_world_imu` 现在来自优化后的 velocity state，而不是临时 pose 差分
- 已把显式 velocity state 接到连续时间发布层：
  - active window 的 control-point snapshot 现在会携带 velocity state
  - `ContinuousTrajectoryView` 在有 control-point kinematics 时优先返回显式 velocity / acceleration
  - planner / debug / control-access 看到的 spline window 不再是 pose-only 快照
- 已开始按 [SLAM_FINISH_PLAN.md](/home/dev/code/ws_iap/src/iap/docs/dev_ct/SLAM_FINISH_PLAN.md) 执行 `M2 / WP2`：
  - `BUCKET` 现已进入“稳定基线 + runtime/diagnostic 双模式”阶段：当 `ct_lidar_profile_factor / ct_lidar_profile_numeric_reference / ct_lidar_export_baseline_csv / ct_lidar_warn_degeneracy` 都关闭时，运行态只会回收当前 segment 的必要 LiDAR 结果用于 `icp_quality`，不再默认扫描整窗 active LiDAR factors 做完整 diagnostics
  - active-window LiDAR factor 生命周期现已开始缓存化：CPU CT LiDAR factor 会按 active segment 复用，GPU `BUCKET` factor 也会复用 source bucketization，并在 global target 变化时只刷新 target-side GPU resources，而不是每窗从零重建整个 factor
  - pipeline profiling 现已进一步把 BUCKET 热点拆成 cache/build/refresh/result 子阶段：`graph_lidar_factor_new_build_ms / graph_lidar_factor_target_refresh_ms / graph_lidar_factor_reused_attach_ms / cache hit-miss counts / post_lidar_factor_error_ms / numeric_audit_ms / degeneracy_ms / result_pack_ms / window_aggregate_ms`
  - carried prior / shared GNSS state 生命周期现已补上缺键容错：当上一轮 carried prior 的 retained key 在当前 values 中不存在时，会自动跳过这部分 previous prior 的 relinearization；GNSS anchor 初始化时，ECEF shared states 也会稳定地 seed 到当前 BSpline values 中，避免 `key "e0" not exist in the Values` 再次打断 continuous-time GNSS 链路
  - continuous-time LiDAR factor 现已具备显式 target 策略配置：`ACTIVE_WINDOW_SNAPSHOT` 和 `GLOBAL_IVOX_REFERENCE` 两种模式可切换，并新增 snapshot frame-window 参数来收口 frozen target 的生命周期
  - `OdometryEstimationBSpline` 现已为每个 active segment 记录 target mode / target frame count / target point count / target build time，并在当前帧 LiDAR factor 上输出这些 target diagnostics
  - `IntegratedBSplineGICPFactor` 现已新增 profiling stats，能够报告 source/target 点数、匹配数、match ratio，以及 pose update / correspondence / accumulation / total 的耗时分解
  - `IntegratedBSplineGICPFactor` 现已新增 linearization check 入口，支持用受控 perturbation 对比 Hessian 预测误差和实际非线性误差，作为后续解析 Jacobian 的基线校验工具
  - continuous-time target tree 现已统一从 `voxel_points()` 导出的 `PointCloud` 构建，避开了 `KdTree2<iVox>` 在 frame traits 上的不稳定路径
  - 已新增 `test_bspline_gicp_factor`，覆盖 LiDAR factor 的误差响应、linearization check 有效性和 profiling stats 行为
  - `IntegratedBSplineGICPFactor` 现已从 `NUMERIC_FULL` 默认数值 pose Jacobian 推进到 `SEMI_ANALYTIC` 路径：控制点平移块改为显式解析 Jacobian，控制点旋转块保留受控数值差分，并保留 `NUMERIC_FULL` 作为 A/B 基线
  - continuous-time LiDAR factor 现已补上显式 outlier / robust handling：支持 whitened residual norm 的 outlier gate，以及 `NONE / HUBER / CAUCHY` 三种 robust-kernel 策略
  - `IntegratedBSplineGICPFactor` 现已输出更细的 correspondence 诊断：`matched / inlier / rejected_distance / rejected_outlier / inlier_ratio / mean_robust_weight`
  - `OdometryEstimationBSpline` 现已把 LiDAR Jacobian mode、robust kernel、outlier threshold 接成配置项，并把当前 factor 的 inlier / rmse 结果回填到 `EstimationFrame::icp_quality`
  - `test_bspline_gicp_factor` 现已进一步覆盖半解析 Jacobian 在扰动状态下的线性化一致性，以及 outlier threshold / robust kernel 对坏匹配的抑制行为
  - `IntegratedBSplineGICPFactor` 现已把 correspondence 从“单一最近邻”推进到“k-NN 候选 + Mahalanobis 评分择优”，从而能在局部几何/协方差差异较大时避免被最近欧氏邻点误导
  - CT LiDAR factor 现已支持 best/second-best Mahalanobis score 的 ambiguity rejection；profiling stats 和 trace 日志中新增 `rejected_ambiguity_count`，可直接观察模糊对应被拒绝的数量
  - `SEMI_ANALYTIC` 路径现已把控制点旋转块的数值差分从“每个 source point 反复计算”收口成“每个 spline time node 预缓存一次”，进一步减少 hot path 上的重复数值差分
  - `OdometryEstimationBSpline` 现已新增 `ct_lidar_correspondence_candidates` / `ct_lidar_correspondence_accept_ratio` 配置项，并把当前 factor 的候选数、ambiguity gate 和拒绝计数写入 CT LiDAR diagnostics
  - `test_bspline_gicp_factor` 现已新增 Mahalanobis candidate selection 和 ambiguity-ratio rejection 两类专门测试，作为后续继续推进解析 Jacobian / 更强 correspondence 策略的基线
  - `SEMI_ANALYTIC` 路径现已进一步把旋转块推进成基于 normalized-quaternion blend 的解析链式 Jacobian，不再依赖“每个 time node 缓存数值旋转差分”的旧做法；`NUMERIC_FULL` 仍保留作为 A/B 和 debug 基线
  - CT LiDAR correspondence 现已补上 absolute score-gap ambiguity gate，且 robust handling 现已支持 `robust_weight_floor`，可以把被 soft kernel 严重降权的对应直接硬拒绝；profiling stats 现已新增 `rejected_robust_count`
  - snapshot target lifecycle 现已加上显式支持门槛：只有满足 `snapshot_min_frames / snapshot_min_points / snapshot_max_age` 的 frozen snapshot 才会作为 `ACTIVE_WINDOW_SNAPSHOT` 被采用，否则会明确回退到 global ivox reference
  - `OdometryEstimationBSpline` 现已把 `ct_lidar_correspondence_min_score_gap`、`ct_lidar_robust_weight_floor`、`ct_lidar_snapshot_min_frames`、`ct_lidar_snapshot_min_points`、`ct_lidar_snapshot_max_age` 接成配置项，并在 CT LiDAR trace 日志中输出 `snapshot_frames / snapshot_points / snapshot_span_s / snapshot_policy`
  - `test_bspline_gicp_factor` 现已继续补充：半解析旋转 Jacobian 相对 `NUMERIC_FULL` 的 predicted-error 对照、absolute score-gap ambiguity rejection，以及 robust-weight-floor 硬拒绝行为
  - `IntegratedBSplineGICPFactor` 现已新增 `check_against_numeric_full(...)`，可在当前状态下直接把 `SEMI_ANALYTIC` 结果和本地重建的 `NUMERIC_FULL` baseline 做 rotation/translation 分块对照，用于继续推进解析 Jacobian 时的工程化 profiling
  - `check_against_numeric_full(...)` 现已进一步补上 axis-wise rotation audit：会分别对 3 个局部旋转轴做 predicted-error 对照，并输出 `worst_rotation_axis / max_rotation_axis_rel_error / mean_rotation_axis_rel_error`，用于继续剖析旋转块解析 Jacobian 的误差分布
  - CT LiDAR profiling stats 现已进一步输出 correspondence diversity / degeneracy 指标：`unique_target_count / unique_target_ratio / max_target_reuse / max_target_reuse_ratio / mean(max)_match_distance / mean_match_score / mean_score_gap / mean_score_ratio`
  - CT LiDAR profiling 现已开始形成更系统的 GPU baseline：新增 `time_bucket_count / max_time_bucket_population / mean_time_bucket_population / candidate_evaluation_count / mean_candidates_per_source`，用于估计未来 GPU kernel batching 和 correspondence 开销
  - CT LiDAR target lookup 现已收口到 frozen `iVox` 自身的 search/index 域，不再把 `voxel_points()` 拷贝 KD-tree 的索引和 `iVox` 内部 point/cov 访问混用；当前 target point / covariance / correspondence index 已处于同一语义域
  - `OdometryEstimationBSpline` 现已把 `ct_lidar_profile_numeric_reference` 和 `ct_lidar_numeric_reference_scale` 接成配置项，并在 CT LiDAR trace 日志中输出 numeric-reference drift 和 target/correspondence 退化诊断
  - `IntegratedBSplineGICPFactor` 现已新增 `diagnose_degeneracy(...)`，将当前 factor 的 correspondence/target 状态转成 `low_match_ratio / low_target_diversity / high_target_reuse / high_ambiguity_rejection / weak_score_separation` 等可重用告警标志；`OdometryEstimationBSpline` 也已把 `ct_lidar_warn_*` 阈值接成配置项，并新增专门的 `bspline ct lidar degeneracy` warning 日志
  - `test_bspline_gicp_factor` 现已补充 numeric-reference check API 以及 correspondence diversity / score diagnostics 的专门测试，为下一步继续减少剩余数值差分提供基线
  - 已新增 `bspline_lidar_factor_result.hpp`，统一 CT LiDAR 的 `profile / numeric audit / degeneracy / factor result / window summary` 结果类型，作为“CPU 当前实现 + GPU 后续实现”共享的 profile/result 接口
  - `IntegratedBSplineGICPFactor` 现已支持 `profiling_report()` 和 `make_result(...)`，可把当前 factor 的 profiling、numeric-reference audit 和 degeneracy diagnostics 收口成统一结果对象，而不再只依赖零散日志字段
  - `OdometryEstimationBSpline` 现已在每轮 fixed-lag 求解后聚合 active window 内全部 CT LiDAR segment 的结果，并新增 `bspline ct lidar cpu-summary` 汇总日志，输出整窗 weighted match/inlier、候选评估量、time bucket baseline、numeric-audit 最大误差和 warning 计数
  - `test_bspline_gicp_factor` 现已新增窗口级 result aggregation 单测，验证 CPU profiling baseline 汇总逻辑，作为后续 GPU CT LiDAR factor 复用同一接口的对照基线
  - 统一 result/profile 接口现已进一步收口成 backend-agnostic return surface：新增 `make_bspline_lidar_factor_result(...)` / `make_bspline_lidar_minimal_result(...)`，并显式区分 `minimal` 与 detailed profiles，允许 GPU factor 在暂时还拿不到完整 correspondence/timing 细节时，先返回一致的最小结果对象
  - `BSplineLidarWindowProfileSummary` 现已区分 `detailed_profile_count` 和 `minimal_profile_count`，只对 detailed profiles 聚合 diversity / bucket / candidate / timing 指标，因此 CPU CT LiDAR 的 rich summary 和 GPU minimal summary 现在可以共用同一汇总器而互不污染
  - `OdometryEstimationGPU` 现已成为这套统一返回面的第一个真实 GPU 调用方：现有 `IntegratedVGICPFactorGPU` 会被包装成统一 `BSplineLidarFactorResult`，并输出 `vgicp gpu-summary` trace，用作未来 CT GPU LiDAR factor 的直接返回面基线
  - 现已新增 `IntegratedBSplineGICPFactorGPU`，把 source scan 按 time bucket 切分成多个 GPU unary `IntegratedVGICPFactorGPU` 子因子，并通过 4 控制点 spline pose Jacobian 把每个 bucket 的 GPU Hessian 回映射到控制点窗口
  - `OdometryEstimationBSpline` 现已支持 `frontend_mode = CT_LIDAR_GPU`，也就是 active-window 图里已经可以直接挂连续时间 GPU LiDAR factor，而不再只是“GPU return surface 已接通”
  - 新的 `CT_LIDAR_GPU` 路径已经沿用统一 `BSplineLidarFactorResult / WindowProfileSummary` 返回面，并新增 `bspline ct lidar gpu-summary` / `gpu-factor` trace，当前 profile 以 minimal GPU profile 为主
  - `CT_LIDAR_GPU` 现已把控制点 Jacobian 从纯数值映射推进到与 CPU 对齐的 `NUMERIC_FULL / SEMI_ANALYTIC` 两档：默认半解析路径通过共享的 `bspline_pose_jacobian.hpp` 计算 normalized-quaternion-blend 旋转块和解析平移块，只保留 `NUMERIC_FULL` 作为 GPU A/B baseline
  - `OdometryEstimationBSpline` 现已把 `ct_lidar_jacobian_mode` 真正下发到 GPU CT LiDAR factor，而不再只作用于 CPU CT LiDAR factor
  - `test_bspline_gicp_factor` 现已补上不依赖 CUDA 设备的 spline-pose Jacobian 数值对照测试，并保留 `IntegratedBSplineGICPFactorGPU` 的 CUDA smoke test，验证 GPU factor 仍能 linearize 并返回统一 GPU result/profile
  - `CT_LIDAR_GPU` 现已补上更接近 CPU 的 runtime audit / diagnostics：GPU factor 现在支持 `check_against_numeric_full(...)`、`diagnose_degeneracy(...)`、统一的 `numeric_audit / degeneracy` result payload，以及与 CPU 对齐的 correspondence / robust / outlier 配置面
  - GPU factor 的 `profiling_report()` 现已在保留 GPU timing baseline 的同时，懒加载同一 frozen target / 当前 spline pose 上的 CPU-side correspondence audit，因此 `candidate_evaluation_count / unique_target_ratio / max_target_reuse_ratio / mean_score_gap / rejection counts` 等 richer diagnostics 现在也能进入 `bspline ct lidar gpu-summary` / `gpu-factor` 汇总
  - `OdometryEstimationBSpline` 现已在 `CT_LIDAR_GPU` 路径下输出 numeric-reference drift 日志和 degeneracy warning，并将 LiDAR correspondence / robust / warning 配置完整下发到 GPU CT factor
  - CUDA 环境现已验证可跑：`test_bspline_gicp_factor` 的 GPU smoke tests 已改成与生产路径一致地分配 stream/temp-buffer，并在真实 GPU 上覆盖 GPU factor linearization、detailed unified profile、runtime numeric-reference audit 和 degeneracy diagnostics
  - 统一 `BSplineLidarFactorResult / WindowProfileSummary` 返回面现已进一步扩展为可导出的 baseline surface：新增 `BSplineLidarBaselineExport` 以及 CSV helpers，可把每轮 active-window 的 CT LiDAR 结果导出成一条 `window_summary` 和多条 `factor_result` 记录
  - `OdometryEstimationBSpline` 现已支持 `ct_lidar_export_baseline_csv / ct_lidar_baseline_csv_path`，并会在 `CT_LIDAR_CPU` / `CT_LIDAR_GPU` 两条路径下统一导出 active-window baseline CSV；后续 GPU kernel-level CT LiDAR 优化将直接以这份文件作为 A/B baseline 输入，而不再只依赖 trace log
  - `test_bspline_gicp_factor` 现已补上 baseline CSV export 单测，验证 unified result surface 的 summary/factor rows 和 current-factor 标记，作为后续继续抽象 GPU kernel result/profile 接口的回归基线
  - `CT_LIDAR_GPU` 现已新增 `ct_lidar_gpu_backend = BUCKET | KERNEL` 运行时切换：当前工程实现已明确冻结为 `BUCKET` backend，而 `KERNEL` backend 现已从预留入口推进成第一版可运行 MVP，不再是单纯的 fail-fast placeholder
  - 这意味着后续 kernel-level spline-native GPU LiDAR 可以在不破坏现有工程版测试和回归基线的前提下并行开发；当前 baseline CSV / unified result surface 也将继续作为两个 backend 的共同 A/B 验证面
  - `IntegratedBSplineGICPFactorGPUKernel` 现已完成第一版独立实现：不再经过 `bucket pose -> unary VGICP -> map back` 这层中间表示，而是直接在 GPU 上按点时间戳查询 4 控制点 spline pose、做 correspondence / gating / robust weighting，并累计控制点窗口的 `24x24` Hessian / `24x1` gradient
  - `OdometryEstimationBSpline` 现已在 `CT_LIDAR_GPU + KERNEL` 下真正挂接这条新 factor，并为 `BUCKET` / `KERNEL` 分别保留独立 cache slot；`KERNEL` 也继续复用 unified result/profile/baseline CSV surface，不新增第二套导出格式
  - `BSplineLidarFactorProfile` / baseline CSV 现已扩展 kernel-stage 字段：`kernel_pose_query_ms / kernel_correspondence_ms / kernel_residual_weight_ms / kernel_reduction_ms / host_sync_ms / host_result_pack_ms`，作为 `cached BUCKET vs KERNEL` 后续 A/B 的统一 profile 面
  - `test_bspline_gicp_factor` 现已新增 `GpuKernelFactorLinearizesAndReturnsUnifiedProfile` 与 `GpuKernelFactorRefreshesTargetWithoutRebuildingSourceStaging` 两类 CUDA 测试，覆盖 KERNEL smoke path、current-factor numeric parity，以及 target refresh 不重建 source staging 的生命周期约束
  - 当前 `KERNEL` 仍是 MVP：kernel-stage timing 还没完全拆成稳定的子阶段基线，也还没完成 `cached BUCKET vs KERNEL` 与 `runtime vs diagnostic` 的同配置长包 A/B，这两项已经成为当前 GPU 收口阶段的首要剩余任务
- planner 已开始真正消费 continuous-time state：
  - `IntegrityPlanner::plan()` 在可用时会优先使用 `SplineControlAccess` 锚定当前时刻，再从 `ContinuousTrajectoryView` 解析种子状态
  - motion primitives 现在从连续时间 `pos / vel / yaw / sigma` 出发，而不只是把 trajectory view 当成一个 `sigma0` 来源
  - 候选轨迹评分已经开始按未来 waypoint 时刻查询 continuous-time sample，用于抬高 `sigma_pred / PL_pred`
  - 新增 `w_ct_align` 短时对齐代价，现有评分会对 published continuous-time velocity / yaw 演化敏感
- 当前 `src/iap` 已具备：
  - 连续时间轨迹统一接口
  - B-spline 轨迹容器与时间查询能力
  - 新的 `OdometryEstimationBSpline` 模块骨架
  - planner 读取连续时间轨迹的接口预留与基础接线
- 当前实现已经具备最小可用的 spline-native `LiDAR + IMU + GNSS` 联合优化骨架，但还不是最终工程化完成版本。
- explicit-knot `SplineStateLayout + SplineEvaluator` 现已成为 spline-native 查询与回归维护主线；`support_at(...) / supports_in_range(...)`、sensor offset、span-local `dt/u` 与 evaluator basis/kinematics 行为已有专门 regression 覆盖。
- fixed-lag ownership 的维护基线现已明确收口到 `SplineActiveStateSet + BSplineMarginalizationPartition + BSplineFixedLagStateRegistry`；multi-span / bucket-style shared-control 引用需要按 active-state 语义保留，而不是退回旧的固定四控制点假设。
- 旧 fixed-window / control-point helper 与旧 factor 构造函数仍作为 compatibility façade 保留，但不再代表主线语义或新的回归基线。
- 当前 `BUCKET` GPU 路径已经从“可运行工程版”推进到“可作为稳定 A/B baseline 的运行态实现”，下一步可以在不破坏这条基线的前提下继续推进真正的 kernel-level CT LiDAR backend。
- `CT_LIDAR_GPU_KERNEL` 已完成第一版 MVP：
  - 已新增独立 `IntegratedBSplineGICPFactorGPUKernel`
  - 已直接在 GPU 上按点时间戳查询 4 控制点 spline pose、做 correspondence / gating / robust weighting，并累计 `24x24` Hessian / `24x1` gradient
  - 已接入 `OdometryEstimationBSpline` 的 active-window 图，不再是单纯的预留入口
  - 已复用统一 `BSplineLidarFactorResult / WindowProfileSummary / baseline CSV` 返回面
  - 已补上 CUDA smoke test、current-factor numeric parity 和 target-refresh 不重建 source staging 的单测
- 当前距离“GPU odometry 封板”还差的关键项已经收敛到：
  - `KERNEL` kernel-stage profiling 仍需从当前单体 timing 继续细拆成 pose-query / correspondence / residual-weight / reduction / host-sync 等稳定 A/B 指标
  - 还未完成 `cached BUCKET vs KERNEL`、`KERNEL runtime vs diagnostic` 的同配置长包 A/B
  - carried-prior / shared GNSS states 需要在 `KERNEL` 路径下完成长时 replay 验证，确保不再出现 `e0` 缺键和 GNSS 因子中途掉零
  - `KERNEL` 目前是“可运行 MVP”，还不是最终高性能 kernel-level CT LiDAR 实现

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
- 当前 `CT_LIDAR_CPU` 路径也已经开始直接消费 shared GNSS epoch queue / anchor，并把 GNSS factor 接入局部 fixed-lag 图。

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
- 当前已经能够读取带显式 velocity 的 continuous trajectory / control window。
- 当前已经会优先按 control window 锚定当前 planning seed state，而不是盲目取最新发布样本。
- 当前已经会按未来 waypoint 时刻直接查询 published continuous-time sample，并把其 `sigma / vel / yaw` 注入 candidate scoring。
- 但仍未把 motion primitives 升级为真正的 B-spline candidate planning，也还没有把 future-time scoring 扩展到超出当前发布短窗的更长前瞻时域。

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
  - active window 内多个 segment 的 LiDAR factors
  - active window 内多个 segment 的 IMU factors
  - active window 内多个 segment 的 velocity factors
  - active window 内带 GNSS 观测 segment 的 pseudorange / doppler factors
  - active segment clock states 之间的 clock-between factors
  - shared ECEF origin / world->ECEF rotation anchor states
  - 全 active window 的 smoothness factors
  - active window 起始边界的 marginal prior
  - active window 尾端的 prediction priors
- CPU LiDAR factor 当前采用最小可用实现：
  - 4 个 pose control points
  - per-point time query
  - CPU GICP residual
  - 局部 LM 优化
  - 当前已支持 k-NN + Mahalanobis correspondence 选择、ratio/score-gap ambiguity rejection、outlier gate、robust kernel、robust-weight floor，以及解析 quaternion-blend 旋转 Jacobian
- CPU IMU factor 当前采用最小可用实现：
  - 4 个 pose control points
  - per-segment 下采样 IMU samples
  - 直接 gyro / accel residual
  - shared bias / gravity states
  - 数值 Jacobian
- CPU velocity factor 当前采用最小可用实现：
  - 4 个 pose control points
  - 1 个显式 velocity state
  - 通过有限差分速度预测把 velocity 和 pose spline 绑定
  - 当前主要用于把 velocity 纳入 fixed-lag 图状态组织
- CPU GNSS factor 当前采用最小可用实现：
  - 4 个 pose control points
  - 1 个显式 segment clock state
  - shared ECEF origin / rotation anchor states
  - pseudorange / doppler 按 epoch 时间戳映射到 segment `u`
  - `OdometryEstimationBSpline` 现在直接持有 `GnssEpochBuilder` 和 `GnssHandler`
  - `gnss_extension` 只负责发布 raw GNSS observation batches / ephemeris updates / iono state / anchor
  - epoch 组包、星历查询使用和 ECEF 预处理现在由 odometry-owned `GnssEpochBuilder` 完成，再由 `GnssHandler` 按 `[scan_start, scan_end]` 时间窗消费

当前 Phase 1B 的边界：
- 这还是 local frontend，不是最终的 fixed-lag smoother 主链。
- 新增的 fixed-lag window 现在已经进入优化变量层，并且多段 LiDAR segment factors 已经进入同一优化图。
- 当前已经具备 segment-specific target snapshot 和显式 marginal prior 的雏形。
- 但这些 prior 仍然是工程上的近似替代，不是严格的 Schur complement 边缘化结果。
- 当前已将 boundary velocity / clock 从“仅作初值缓存”推进到“结构化 boundary prior”，但离真正的边缘化信息回灌还有距离。
- 当前 boundary prior 已从“手工 pose / velocity / clock 约束”推进到“boundary 子集信息矩阵回灌”，更接近真实 Schur 边缘化；但它仍只覆盖边界子集，而不是对所有滑出状态做完整信息消元。
- 当前 carried prior 已进一步升级为“对 removable graph 做 survivor marginalization”的形式，不再只盯住 boundary 子集；但它仍是按当前重建图的 removable factors 做图外线性化/回灌，还不是最终 fixed-lag smoother / Bayes tree 级别的完整边缘化实现。
- 当前已把 survivor/removable state/factor partition 抽成稳定工具层，并通过专门单测约束 carried prior 行为；但 prior 仍来自“当前轮 removable 子图”的图外重线性化，而不是增量 Bayes tree / fixed-lag smoother 的原生边缘化。
- 当前 carried prior 已从“非线性因子回灌缓存”进一步收口为“线性图缓存 + replay 时再包装”的形式，减少了旧 prior 在 removable 子图里的重复线性化；但它仍然不是 fixed-lag smoother 原生 Bayes tree 边缘化。
- 当前 fixed-lag lifecycle 已经进一步扩展到 bias / gravity / ECEF anchor 这类 shared states，并补上了 diagnostics / lifecycle telemetry / explicit state-machine outputs；M1 / WP1 下一步更适合转向“默认主线职责收口”和 carried prior / window 推进的最终封板。
- IMU 现在已经开始按时间戳直接约束 spline 的角速度 / 线加速度，并且 bias / gravity 已进入联合优化。
- velocity 现在已经作为独立状态显式进入图，planner 也已开始消费 latest continuous-time sample。
- planner 现在已经开始消费 future-time continuous-time sample，但仍是基于已发布短窗的短时保守化/对齐评分。
- IMU / velocity continuous-time factors 的 Jacobian 目前仍是数值形式。
- GNSS 已经开始进入控制点窗口主链，但目前还是“shared queue + per-segment factor 接线”的最小实现。
- LiDAR factor 目前已切到“半解析 Jacobian + 受控数值旋转块”的过渡工程版，并补上 target strategy、profiling stats、outlier/robust handling 和 linearization check 基线；完整解析 Jacobian 和 GPU 版仍是后续工作。

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
- `test_bspline_imu_factor` 通过
- `test_bspline_trajectory` 通过
- `test_bspline_control_window` 现已覆盖 control buffer 扩展、lag pruning、values/update round-trip
- `test_bspline_imu_factor` 现已覆盖静止匹配样本零残差、bias-state 补偿和 gravity-state 失配
- `test_bspline_velocity_factor` 现已覆盖 matching velocity 零残差、mismatch 非零残差和 linearize 可用性
- `test_bspline_gnss_factor` 现已覆盖 CT pseudorange / doppler factor 的零残差和 clock-state 吸收能力
- `test_gnss_epoch_builder` 现已覆盖 raw GNSS batch 在 anchor / ephemeris 缺失和完整条件下的 epoch 组包行为
- `test_gnss_handler_queue` 现已覆盖 `GnssHandler` 的 segment-range drain 行为和 future epoch 保留
- `test_shared_state_gnss_queue` 现已覆盖 shared state 对 processed epoch、raw observation batch、ephemeris update 和 iono state 的 mailbox 语义
- `test_bspline_marginalization` 现已覆盖 survivor/removable partition 归属，以及 carried prior 与参考 marginal graph 的误差一致性
- `test_bspline_marginalization` 现已进一步覆盖 foreign-key ownership 检测，以及“上一轮线性 carried prior + 本轮 removable nonlinear 子图”的 prior 组合一致性
- `test_bspline_fixed_lag_registry` 现已覆盖 unified fixed-lag registry 的窗口推进、segment 生命周期和 auxiliary state 过滤行为
- `test_bspline_fixed_lag_registry` 现已进一步覆盖 shared fixed-lag states 的 seed/update 行为
- `test_bspline_fixed_lag_registry` 现已进一步覆盖 fixed-lag lifecycle telemetry/state-machine 的迁移与计数行为
- `test_bspline_gicp_factor` 现已覆盖 continuous-time LiDAR factor 的误差响应、linearization check 有效性和 profiling stats 输出
- `test_bspline_control_window` 现已覆盖 velocity state 到 control-point snapshot 的发布
- `test_bspline_trajectory` 现已覆盖带 control-point velocity 时的 trajectory sampling
- `test_integrity_planner` 现已覆盖 planner 对 continuous-time sample 的种子状态消费、future sigma floor 和 future velocity-aware scoring
- Phase 1B/1C 当前代码已完成编译与测试通过

## 当前还没完成的关键部分

### 1. 还不是 spline-native estimator
- 当前 `CT_LIDAR_CPU` 路径已经把 4 控制点窗口作为局部优化变量引入。
- 当前 active spline window 已经具备 fixed-lag 形式的保存与裁剪。
- 当前 active spline window 已经进入优化变量层。
- 当前已经形成“多段 LiDAR factors + 共享控制点”的联合优化骨架。
- 当前已经形成“多段 LiDAR factors + 多段 IMU factors + 共享控制点”的联合优化骨架。
- 但它还不是最终的 fixed-lag GTSAM 主状态组织方式，也还没有成熟的 Schur 边缘化/先验回灌。

### 2. LiDAR 连续时间残差还没接入主链
- 当前已经有 CPU 版连续时间 LiDAR factor。
- 但它还是最小可用版本：
  - 当前已覆盖 active window 内多个 segment
  - 已支持 segment-specific local target snapshot
  - 但 snapshot 构造仍然是工程近似，并非最终 submap target 策略
  - 数值化 pose Jacobian
  - 还没有 GPU 版
  - 还没有与最终 fixed-lag 图结构完全统一

### 3. IMU 连续时间约束还没做完
- 当前已经有 per-segment IMU sample factor。
- 当前已经开始将 IMU 观测直接约束到 spline 的角速度 / 线加速度。
- 当前 shared gyro bias / accel bias / gravity 已进入图。
- velocity 已进入图，但仍是通过独立 velocity consistency factor 与 pose spline 绑定的过渡形式。
- 当前 Jacobian 仍是数值形式，尚未做解析化和更严格的数值校验。

### 4. GNSS 还没完全下沉到 bspline odometry
- 当前已经把 pseudorange / doppler 的时间戳约束直接并入 spline window。
- 当前 `OdometryEstimationBSpline` 已经直接持有并驱动自己的 `GnssEpochBuilder` 和 `GnssHandler`，并在 active segment 上建立 per-segment clock / anchor / GNSS factors。
- 当前 shared state 只承担 processed epoch（完整性监测用）、raw observation batch、ephemeris update、iono state 和 anchor 发布。
- `gnss_extension` 已不再主导 BSpline 主链的 epoch 组包；这些逻辑已向 odometry 内核收口。
- 但 `gnss_extension` 仍然负责 ROS topic ingress、NavSatFix anchor 初始化和 legacy GNSS smoother 注入，尚未完全迁成“odometry 内核单一所有者”。
- GNSS continuous-time factor 当前仍是最小可用版本，尚未补齐更严格的 Jacobian、边缘化和更长时域的 epoch/window 对齐策略。

### 5. planner 还没真正用 spline 做候选轨迹优化
- 当前已经能用 continuous-time seed state 和 future-time published samples 给 planner 提供种子与短时评分约束。
- 当前 motion primitives 的起点已经和 velocity-aware spline state 对齐，`sigma_pred / PL_pred` 也会被短窗 continuous-time uncertainty 抬高。
- 还没有把 motion primitives 升级为 B-spline candidate planning。

## 下一步计划

- 后续开发顺序统一以 [SLAM_FINISH_PLAN.md](/home/dev/code/ws_iap/src/iap/docs/dev_ct/SLAM_FINISH_PLAN.md) 为准。
- 当前正在执行：
  - `M1 / WP1`：把 `CT_LIDAR_CPU` 从 local frontend 推进到 fixed-lag 主链
  - 当前这一步先收口 boundary prior，把 pose / velocity / clock 的 lag-window 边界约束组织到同一条主线上

### Next Step 1：把 `CT_LIDAR_CPU` 从 local frontend 推进到 fixed-lag spline 主链
目标：
- 让 spline control points 成为 odometry 主状态的一部分
- 不再只依赖每帧局部 LM，而是进入真正的“多段窗口联合优化 + 边缘化”

建议子任务：
- 把当前工程化 marginal prior 升级为更接近真实边缘化的先验回灌
- 评估是否需要 segment-specific local submap target 替代当前 snapshot 方案
- 让旧控制点移出活动窗口时保留更严格的信息矩阵
- 保留现有兼容输出不变

### Next Step 2：补齐 IMU 连续时间约束
目标：
- 用 IMU 时间戳直接约束 spline 的速度、角速度、加速度
- 让 IMU 不再只承担初始化和兼容链路职责

建议子任务：
- 在当前 gyro / accel sample factor 基础上继续收敛 velocity / clock 与 spline control points 的联合状态布局
- 完成 IMU 残差的解析 Jacobian 或更严格的数值校验

### Next Step 3：继续收口 Phase 1C，把 GNSS 从“最小接线”推进到“内核化主链”
目标：
- pseudorange / doppler 按观测时间直接约束 spline 状态与 clock

建议子任务：
- 明确 legacy `gnss_extension` smoother 注入和 continuous-time odometry GNSS front-end 的最终长期边界
- 评估是否继续把 NavSatFix anchor 初始化与更多 raw GNSS ownership 从 `gnss_extension` 向 `OdometryEstimationBSpline` 收口
- 补齐 GNSS factor 的更严格 Jacobian / clock prior / epoch-window 对齐策略

### Next Step 4：补齐 LiDAR continuous-time factor 的工程化能力
目标：
- 让当前 LiDAR factor 从“最小可用”走向“可长期演进”

### Next Step 5：把 planner 从 short-horizon CT scoring 推进到 spline candidate planning
目标：
- 让规划器不只“参考”已发布 spline，而是直接生成和评分 B-spline 候选轨迹

建议子任务：
- 让 candidate 表达从离散 motion primitives 逐步过渡到 spline-native 参数化
- 扩展 planner 可消费的 future-time trajectory horizon，不只依赖当前发布的短窗
- 复用 `ContinuousTrajectoryView` / `SplineControlAccess` 做候选轨迹初始化与约束对齐

建议子任务：
- 评估解析 Jacobian 替换数值 Jacobian
- 设计 GPU 版 continuous-time LiDAR factor 的复用接口
- 对比 `RECONSTRUCT` / `CT_LIDAR_CPU` / legacy CT-GICP 的耗时与精度

### Next Step 5：Phase 1D，planner 真正开始消费 continuous-time info
目标：
- 让现有 `IntegrityPlanner` 不只是读 latest sample 作为 seed
- 而是能读取：
  - 未来时刻 pose / vel / yaw
  - uncertainty
  - spline meta

建议子任务：
- 在不破坏当前 `CandidateTrajectory` 的前提下，增强 planner 评分阶段对 future-time trajectory view 的使用
- 为下一阶段 spline candidate planner 做接口验证

## 推荐的下一次实际开发顺序
1. 把当前 `CT_LIDAR_CPU` 从 local LM frontend 推进到 fixed-lag spline window 主链
2. 将 IMU 约束真正切到 spline window
3. 把 GNSS 伪距 / 多普勒并入控制点窗口
4. 完善 LiDAR factor 的 Jacobian / GPU 演进路径
5. 最后把 planner 从“latest-sample seed”推进到“future-time trajectory-aware scoring”

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
