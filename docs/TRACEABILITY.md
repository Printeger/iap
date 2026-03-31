# Traceability Matrix (IAP)

> 目的：确保“需求（IAP-RQ）↔ 实现 ↔ 测试/实验 ↔ 日志/指标”可追溯，防止做错/做多/漏做。

## 0. 规则
- 每个代码改动必须引用至少一个 IAP-RQ
- 每个 IAP-RQ 必须在本表中至少有：
  - 实现文件路径（Implementation）
  - 验证方式（Test/Experiment）
  - 可观测日志字段（Logs/Metrics）

---

## 1. 需求追溯表

| Req ID | 需求描述 | Talk/Idea 对照点 | Implementation（文件/模块） | Test/Experiment（如何验证） | Logs/Metrics（必须输出） | 状态 |
|---|---|---|---|---|---|---|
| IAP-RQ-000 | Repo guardrails：AGENTS.md、doc-guard（pre-commit hook + tools/doc_guard.py）、docs 三件套 | — | `AGENTS.md`, `.githooks/pre-commit`, `tools/doc_guard.py`, `docs/` | 提交代码时 hook 拦截缺失文档；`git config core.hooksPath` = `.githooks` | hook exit code | **DONE** |
| IAP-RQ-001 | Rename ROS2 package to `iap` | — | `package.xml`, `CMakeLists.txt`, `src/iap/`, `include/iap/`, `cmake/iap-config.cmake.in` | `colcon build --packages-select iap` 成功；`ros2 pkg list \| grep iap` 可见 | build exit code 0 | **DONE** |
| IAP-RQ-002 | Build artifacts + 模块耗时测量 | 开发基础设施; 实时性验证 | `apps/iap_status.cpp`, `launch/iap_demo.launch.py`, `.clangd`; **timing**: `gnss_extension.cpp` `on_smoother_update_finish_`, `integrity_monitor.cpp` `compute()`, `araim.cpp` `run()`, `trunk_detector.cpp` `detect()` — `std::chrono` + fopen/fprintf → `/tmp/iap_timing.csv` | `python3 tools/plot_icp_timing.py ... /tmp/iap_timing.csv` → Fig C2: 各模块 p99 < 50 ms | `stamp,module,elapsed_ms` in `/tmp/iap_timing.csv`; 4 modules | **DONE** |
| IAP-RQ-003 | Standalone ROS2 operation — no runtime dep on `glim_ros` | 独立部署需求 | `include/iap/util/rviz_viewer.hpp`, `src/iap/util/rviz_viewer.cpp` (**fix**: skip imu→lidar TF when frames identical); `include/iap/util/standard_viewer.hpp`, `include/iap/util/standard_viewer_mem.hpp`, `src/iap/util/standard_viewer*.cpp` (4 files); `CMakeLists.txt` (rviz_viewer + standard_viewer targets + tf2_ros/nav_msgs/geometry_msgs deps); `config/config_ros.json` (removed libmemory_monitor.so; **fix**: add dump_path, complete QoS sub-objects) | `ros2 run iap iap_rosnode` 启动后 RViz2 可见 `~/aligned_points` + `~/odom`；3D viewer 窗口弹出；无 TF_SELF_TRANSFORM 报错 | `[rviz] published odom`, `[rviz] published aligned_points` log lines | **DONE** |
| IAP-RQ-010 | 状态扩展：clk_bias δt [m], clk_drift δṫ [m/s] 加入 factor graph (`C(i)=gtsam::Vector2`) | 状态向量包含 clock bias/drift | `include/iap/odometry/estimation_frame.hpp`, `odometry_estimation_imu.hpp/.cpp` (**fix**: per-type `FastMap` relinearize threshold, `clk_bias_relin_thresh=500`/`clk_drift_relin_thresh=5`); `config/config_odometry_gpu.json` (add `clk_bias_noise`, `clk_drift_noise`, clock relin thresholds; bump `isam2_relinearize_skip` 1→5) | `colcon build` 通过；`trace` 日志含 `clk_bias/clk_drift`；无 sync-mode 警告洪水 | `clk_bias(m), clk_drift(m/s)` | **DONE** |
| IAP-RQ-015 | Expose Σ_p 位置协方差块（3×3）到 EstimationFrame；供 PL proxy 使用 | 保护级别需基于不确定性 | `estimation_frame.hpp` (+sigma_p), `odometry_estimation_imu.cpp` (marginalCovariance) | `trace` 日志含 `trace(Σ_p)` 和 `lambda_max(Σ_p)` | `trace_sigma_p, lambda_max_sigma_p, PL_proxy` | **DONE** |
| IAP-RQ-020 | GNSS 紧耦合观测：伪距 + 多普勒建因子（含 clock bias/drift，per-sat） | GNSS tightly-coupled | `include/iap/gnss/`, `src/iap/gnss/` (PseudorangeFactor NoiseModelFactor4<Pose3,Vector2,Vector3,Rot3>, DopplerFactor NoiseModelFactor4<Pose3,Vector3,Vector2,Rot3>, GnssHandler, `GnssEpochBuilder`, **GnssExtensionModule** ROS2 bridge); **Phase-1C CT path**: `include/iap/odometry/integrated_bspline_gnss_factor.hpp`, `src/iap/odometry/integrated_bspline_gnss_factor.cpp`, `src/iap/odometry/odometry_estimation_bspline.cpp`, `include/iap/util/shared_state.hpp` (raw GNSS observation mailbox + ephemeris mailbox + iono state + anchor), `test/test_bspline_gnss_factor.cpp`, `test/test_gnss_epoch_builder.cpp`, `test/test_gnss_handler_queue.cpp`, `test/test_shared_state_gnss_queue.cpp`; **CT ownership update**: `OdometryEstimationBSpline` now owns both `GnssEpochBuilder` and `GnssHandler`, rebuilding processed epochs from raw shared-state GNSS inputs before segment-window factor generation; **ECEF pipeline**: E(0)/R(0) free variables with priors σ_E=5m/σ_R=5°; **corrections**: Klobuchar iono + Hopfield trop + Sagnac + TGD; **svdt sign fix**: `pr_meas=pr+svdt*c` (ADD); **svddt Doppler fix**: `dop_meas=dop+svddt*c`; iono from `/ublox_driver/iono_params`; gps_sec/tgd/svddt stored in SatObs/GnssEpoch; epoch/factor logs at `info` level; **post-opt diagnostic**: `on_smoother_update_finish` logs `clk_bias / clk_drift / PR_rms / Dop_rms`; **timestamp fix**: `gpst2utc(obs[0]->time)`; **clock init fix**: `C(frame_id)` inserted if absent; **clock warm-start**: post-opt clk state propagated to next frame via `bias+drift×dt` | 关闭/开启 GNSS 对比；`PR rms` 收敛至 <10 m；`clk_bias` 稳定跟踪接收机时钟偏差 (~300–400 km)；`test_bspline_gnss_factor` 校验 CT pseudorange/doppler factor 的零残差和 clock-state 吸收能力；`test_gnss_epoch_builder` 校验 raw GNSS batch 到 ECEF epoch 的组包条件；`test_gnss_handler_queue` 校验 handler 的 segment-window drain 与 future-epoch 保留；`test_shared_state_gnss_queue` 校验 raw observation / ephemeris / iono mailbox 语义 | `res_pr, res_dop, clk, clk_dot`; `[gnss_ext] injection #N / diag #N clk_bias/clk_drift/PR_rms/Dop_rms`; `[gnss_ext] E(0)/R(0) inserted`; `bspline ct active_gnss_pr_factors / active_gnss_dop_factors / raw_gnss_batches / gnss_handler queue ownership` | **DONE** |
| IAP-RQ-025 | GNSS 参数外部化到 config_gnss.json；移除环境变量覆盖 | 参数管理/可维护性 | `config/config_gnss.json` (16 params); `gnss_extension.cpp` (Config 读取); `gnss_handler_` → `unique_ptr` | 修改 JSON 后重启 PR_rms 变化；`enable_debug_csv=true` 生成 CSV | `[gnss_ext] Config loaded: ...` 日志行 | **DONE** |
| IAP-RQ-030 | GNSS per-satellite NIS gating：downweight / exclude（RAIM-ish baseline） | 卫星级完整性、FDE 思路 | `src/integrity/gnss_integrity.*` | 注入某卫星 bias，触发剔星 | `sat_nis, exclude_sats, gamma_R, global_nis` | TODO |
| IAP-RQ-040 | LiDAR ICP 因子健康度：退化/错配检测 → noise inflation / drop factor; CSV 输出 | trunk/几何退化会影响可观测性 | `EstimationFrame::IcpQuality`, `odometry_estimation_gpu/cpu.cpp` (Hessian cond, gamma_lidar); `config_odometry_gpu.json` `enable_icp_csv`/`icp_csv_path`; ICP CSV write in `update_frames()` + `create_factors()` | `python3 tools/plot_icp_timing.py /tmp/iap_icp.csv /tmp/iap_timing.csv` → Fig C1/C2 | `icp_rmse, inliers, cond, gamma_lidar, drop`; `/tmp/iap_icp.csv` | **DONE** |
| IAP-RQ-045 | Global mapping multiscan_window：保留最近 N 帧用于 point-to-multiscan 匹配 | 多帧约束改善全局一致性 | `GlobalMappingParams::multiscan_window` (default 3); `global_mapping.cpp` (frame window pruning); `config_global_mapping_cpu/gpu.json` | 调 N=1/3/5 观察全局漂移变化 | `global_mapping frame_window_size` | **DONE** |
| IAP-RQ-100 | Trunk 检测与圆拟合（中心, 半径, confidence） | 树干几何地标 | `include/iap/trunk/trunk_types.hpp`, `trunk_detector.hpp/.cpp` (Kasa fit, grid BFS) | 对进树林场景启动，日志输出 trunk 数量/半径/置信度 | `trunk_count, radii, confidence[]` | **DONE** |
| IAP-RQ-110 | Trunk 健康度因子接口 (Baseline-A: 不入图) | 树干布局影响 LiDAR 可观测性 | `TrunkDetector::health_factor()` ([0,1]) | health~0 时硬件应填充更大噪声 | `trunk_health` | **DONE (Baseline-A)** |
| IAP-RQ-120 | TDOP 指标（角度多样性） | 树干几何与完整性联系 | `TrunkDetectionResult::tdop/tdop2/lambda_min_H` | 树更分散时 TDOP 下降 | `tdop, lambda_min_H` | **DONE** |
| IAP-RQ-200 | Integrity 输出：PL/AL/IM/mode + 关键中间量; ARAIM CSV; K_fa_used 修复 | PL < AL 安全条件 | `integrity_types.hpp` (+K_fa_used); `araim_debug.hpp` (config constructor, write+worst_hyp); `integrity_monitor.hpp/.cpp` (last_araim_result_, K_fa_used forwarding); `integrity_extension.cpp` (CSV write, k_fa_used bug fix, traj CSV); `config_gnss.json` `"integrity"` section; **fix**: `integrity_monitor.cpp` distinguish UNSAFE-due-to-PL>=AL (warn) vs UNSAFE-in-recovery (info) | `python3 tools/plot_araim_timeline.py /tmp/iap_araim.csv` → Fig B1/B2/B3; IM>0 帧占比>80%; HPL<HAL 始终成立 | `/tmp/iap_araim.csv` (`row_type,stamp,HPL/VPL/HAL/VAL/IM,worst_hyp data`); `PL, AL, IM, mode, K_fa_used` | **DONE** |
| IAP-RQ-210 | Alert Limit AL 由障碍距离动态给出 | 近障碍时 AL 缩小 | `IntegrityMonitor::compute_AL()`, `set_obstacle_distance()` | 越靠近障碍 AL 越小；日志可见 | `AL, obstacle_dist` | **DONE** |
| IAP-RQ-220 | GNSS per-satellite NIS gating（RAIM-ish） | 卫星级 FDE | `IntegrityMonitor::run_gnss_gating()` (chi2 test, gamma_R, FDE greedy) | 注入 bias 卫星被降权/剔除 | `sat_nis, gamma_R, excluded_sats` | **DONE** |
| IAP-RQ-300 | 候选轨迹生成（motion primitives） | 运动原语离散化候选轨迹 | `include/iap/planner/trajectory_types.hpp`, `trajectory_generator.hpp/.cpp`; **dev_ct foundation**: `include/iap/planner/continuous_trajectory_view.hpp`, `include/iap/odometry/bspline_trajectory.hpp`, `src/iap/odometry/bspline_trajectory.cpp`, `include/iap/odometry/bspline_control_window.hpp`, `src/iap/odometry/bspline_control_window.cpp`, `include/iap/odometry/integrated_bspline_gicp_factor.hpp`, `src/iap/odometry/integrated_bspline_gicp_factor.cpp`, `include/iap/odometry/integrated_bspline_gnss_factor.hpp`, `src/iap/odometry/integrated_bspline_gnss_factor.cpp`, `include/iap/odometry/integrated_bspline_imu_factor.hpp`, `src/iap/odometry/integrated_bspline_imu_factor.cpp`, `include/iap/odometry/integrated_bspline_velocity_factor.hpp`, `src/iap/odometry/integrated_bspline_velocity_factor.cpp`, `include/iap/odometry/odometry_estimation_bspline.hpp`, `src/iap/odometry/odometry_estimation_bspline.cpp`, `include/iap/planner/integrity_planner.hpp`, `src/iap/planner/integrity_planner.cpp`, `test/test_bspline_trajectory.cpp`, `test/test_bspline_control_window.cpp`, `test/test_bspline_gnss_factor.cpp`, `test/test_bspline_imu_factor.cpp`, `test/test_bspline_velocity_factor.cpp`, `test/test_integrity_planner.cpp` | 生成 M 条候选轨迹并可视化时间戳点序列；`test_bspline_trajectory` 校验 uniform/non-uniform spline query 与 window snapshot；`test_bspline_control_window` 校验 control-point key shift、窗口扩展与 lag pruning；`test_bspline_gnss_factor` 校验 continuous-time GNSS factor 的零残差与 clock-state 一致性；`test_bspline_imu_factor` 校验 continuous-time IMU sample factor 的静止零残差、bias-state 补偿、gravity-state 失配行为；`test_bspline_velocity_factor` 校验 velocity state 的零残差、失配残差和 linearize 可用性；`test_integrity_planner` 现同时校验 planner 使用 continuous-time sample 作为 motion-primitive seed，以及 future-time trajectory sample 对 `sigma_pred` 和 candidate scoring 的影响 | `trajectory_count, speeds, yaw_rates`; `spline knot mode / control_point_count`; `control point key index / segment time span / active window size / active segment factor count / active IMU factor count / active velocity factor count / active_gnss_pr_factors / active_gnss_dop_factors / shared bias state / gravity state`; `seed_from_ct, ct_future_match_count, sigma_pred` | **DONE** |
| IAP-RQ-310 | 预测可见/可观测性集合（占位） | ray-check 遮挡预测 | `include/iap/planner/predicted_integrity.hpp/.cpp` (placeholder) | TODO: 接地图后做 ray-check；当前返回占位值 | `n_vis_placeholder` | **DONE (placeholder)** |
| IAP-RQ-320 | 协方差传播 → Σ_pred → PL_pred | PL 预测供规划使用 | `include/iap/planner/predicted_integrity.hpp/.cpp` (sigma growth, K_pl=3.0) | PL_pred 随时间增长且不同轨迹有差异；σ_grow 可配置 | `PL_pred(s), sigma_pred(s)` | **DONE (baseline)** |
| IAP-RQ-400 | Integrity-aware planning objective | hinge(PL_pred−AL)²代价+goal+effort | `include/iap/planner/integrity_planner.hpp/.cpp`; `evaluate()`; continuous-time short-horizon alignment term `w_ct_align * D_ct(τ)` | IM<0时选绕行轨迹；J_integrity > J_goal场景可复现；`test_integrity_planner` 校验 future-time velocity / yaw mismatch 会改变 candidate ranking | `J_total, J_integrity, J_goal, J_effort, J_ct_align` | **DONE** |
| IAP-RQ-410 | Receding horizon loop | 执行Δt后重规划 | `IntegrityPlanner::execution_target()`, `plan()`; **dev_ct foundation**: `PlannerInterface::set_trajectory_view/set_control_access`, `IntegrityPlanner` continuous-time view hookup, `IapSharedState` trajectory publication, `OdometryEstimationBSpline` sampled output bridge, velocity-aware `BSplineTrajectory` sampling, control-access-anchored seed resolution, future-time trajectory sampling during candidate evaluation | 调用`plan()`+`execution_target()`模拟多步闭环；`test_bspline_trajectory` + odometry module startup smoke test verify planner can consume a published spline view without changing `plan(...)` signature；velocity-aware sampling 由 `test_bspline_trajectory` 覆盖；`test_integrity_planner` 覆盖 planner seed-state consumption、future sigma floor 和 future velocity-aware scoring | `chosen_traj_id, dt_execute`; `continuous trajectory available`; `trajectory sample velocity`; `seed_from_ct, ct_future_match_count` | **DONE** |
| IAP-RQ-500 | 三种 baseline（Passive/CovMin/IntegAware） | 对比 integrity 驱动的优势 | `apps/iap_experiment.cpp` (run_baseline ×3) | 同场景三 baseline 均输出指标 CSV | `baseline, violation_frac, avg_PL, mission_success` | **DONE (stub)** |
| IAP-RQ-510 | 指标：Time(PL>AL)%, AvgPL, MinIM, path/time/effort | 量化对比表格 | `include/iap/experiments/metrics.hpp` (MetricsCollector, write_comparison_table) | `ros2 run iap iap_experiment` 输出 /tmp/*_summary.md | `violation%, avg_PL, min_IM, path_len, time, effort` | **DONE (stub)** |
| IAP-RQ-900 | 自动生成 IEEE Trans methodology.tex（流程图+模块小节+公式） | 论文写作辅助 | `tools/gen_methodology.py` → `docs/methodology/methodology.tex`, `docs/figures/system_flow.tex` | `python3 tools/gen_methodology.py` 生成 .tex；结构无误（12 env 平衡） | gen exit code 0; env mismatch=0 | **DONE** |
| IAP-RQ-311 | 局部占用栅格供光线检测 | Talk §7.2 预测可见卫星/地标 | `include/iap/map/local_occupancy.hpp`, `src/iap/map/local_occupancy.cpp`; VoxelKey + Morton hash; DDA ray traversal | 合成占用体素验证光线命中/未命中正确 | `ray_occluded bool, occupancy_ratio κ` | **DONE** |
| IAP-RQ-312 | 对候选路径点预测卫星可见性集合 V̂(τ) | Talk §7.2 predicted V̂, geometry | `include/iap/gnss/visibility_predictor.hpp`, `src/iap/gnss/visibility_predictor.cpp`; ENU dir from (el,az); ray_occluded per sat | 树冠地图下移动减少 n_vis；开阔增加 | `n_vis, vis_flags[], mean_kappa` | **DONE** |
| IAP-RQ-313 | 估计 LOS 上的冠层密度 κ（预测时刻） | Talk §3.2 σ_eff(κ,θ) | `VisibilityPredictor::predict()` 调用 `occupancy_ratio()` → κ per satellite; `SatObs.kappa` 字段 | κ 在密集冠层下增大；开阔时接近 0 | `kappa per sat, mean_kappa` | **DONE** |
| IAP-RQ-314 | 实现 σ_eff(κ,θ) 和权重矩阵 W | Talk §3.2 σ²_eff = σ²_c · exp(α κ / sin θ) | `include/iap/gnss/canopy_noise_model.hpp` (header-only); `sigma_eff_canopy()`, `info_weight_canopy()` | 相同仰角 κ 更高 → σ_eff 更大，单调性 | `sigma_eff per sat` | **DONE** |
| IAP-RQ-321 | 使 PL_pred 与轨迹相关（替换单一 sigma 增长） | Talk §7.2 predicted covariance from predicted geometry | `predicted_integrity.hpp/.cpp`: `set_occupancy/set_epoch` API; `sigma_grow_at(pos)` = sigma_grow × max(1, f(n_vis, κ)) | 不同候选轨迹产生不同的 PL_pred 序列 | `sigma_grow_eff(s), PL_pred(s)` | **DONE** |
| IAP-RQ-130 | Trunk FGO 扩展模块激活 + ROS2 可视化；EstimationFrame ABI 布局修复 | 树干建图入因子图，RViz 高亮 | `include/iap/trunk/trunk_extension.hpp`, `src/iap/trunk/trunk_extension.cpp` (ExtensionModuleROS2, MarkerArray pub ~/trunks, map_mutex_); `CMakeLists.txt` (独立 trunk_extension .so); `config_ros.json`; **ABI fix**: IAP fields (`clk_bias,clk_drift,sigma_p,icp_quality`) 移至 struct 末尾 → `raw_frame` 偏移恢复与 libglim.so 一致 | 回放时 RViz 看到黄绿色树干圆柱；无 SIGSEGV | `[trunk_ext] visualization publisher created`; trunk cylinders in ~/trunks topic | **DONE** |
| IAP-RQ-131 | 树干数据关联与持久地标 ID | Talk §4.2 landmark map L={c_k} | `include/iap/trunk/trunk_map.hpp`, `src/iap/trunk/trunk_map.cpp`; EMA 平滑; XY距离+半径门限关联; 陈旧剪枝 | 回放中同一树干跨帧保持一致 ID >80% | `landmark_id, seen_count, center_xy` | **DONE** |
| IAP-RQ-132 | 树干观测因子入因子图 (Full-B) | Talk §4.2 TrunkFactor + Σ_trunk | `include/iap/trunk/trunk_factor.hpp`, `src/iap/trunk/trunk_factor.cpp`; `NoiseModelFactor1<Pose3>`; 3D 残差 r = z_k − R^T(c_k−p); 解析 H(3×6); `make_noise(confidence)` | 开启树干因子减小 Σ_p 和 PL_proxy | `r_trunk (3D), H_trunk` | **DONE** |
| IAP-RQ-133 | 置信度加权 TDOP | Talk: TDOP = sqrt(tr((G^T W G)^{-1})) | `trunk_types.hpp`: `tdop_weighted` 字段; `trunk_detector.cpp: compute_tdop()` += W=diag(conf²) 加权分支 | 树干分散且置信度高时 TDOP_weighted 更低 | `tdop_weighted` | **DONE** |
| IAP-RQ-241 | ARAIM 故障假设集枚举 | Talk §6.2 H0 + GNSS单星 + 树干地标故障 | `include/iap/integrity/araim_types.hpp`: FaultHypothesis; `araim.cpp: enumerate_hypotheses()` — 每个非排除卫星一个 GNSS_SAT 假设 + N_trunk 个 TRUNK 假设 | 日志输出 N_hyp = 1 + N_sat + K_trunk | `araim_n_hyp` | **DONE** |
| IAP-RQ-242 | 全解与子集解（解分离） | Talk §6.4 S0 + S_k via zeroed weights | `araim.cpp: compute_core()` — S0=(G^T W G)^{-1}; W_k 第 k 行归零; S_k=(G_k^T W_k G_k)^{-1}; p0−p_k=d_k; `SatObs::pr_residual` 字段 | 确定性输出；去除某测量后子集解与全解有差异 | `d_k (4D)` | **DONE** |
| IAP-RQ-243 | 分离统计量 σ_ss,q,k | Talk §6.4.3/§6.5 δS=S0−S_k | `compute_core()`: σ_ss_E=√(δS[0,0]), σ_ss_N=√(δS[1,1]), σ_ss_horiz=√(σ_E²+σ_N²) | 几何弱化/卫星少时 σ_ss 增大 | `sigma_ss_E, sigma_ss_N, sigma_ss_horiz` | **DONE** |
| IAP-RQ-244 | 检测门限与乘子 (K_fa, K_md) | Talk §6.5/§6.6 P_FA 分配 | `Araim::Params{K_fa=4.5, K_md=5.5, K_ff=5.33}`; `SubsetSolution::threshold = K_fa·σ_ss_horiz` | 更严 P_FA/P_HMI → 门限单调变化 | `threshold per hyp` | **DONE** |
| IAP-RQ-245 | 故障 PL 与总 ARAIM PL | Talk §6.6.2–6.6.3 pl_faulted + pl_ff | `compute_core()`: pl_faulted=d_horiz+K_md·σ_ss_horiz; pl_ff=K_ff·√(S0[0,0]+S0[1,1]); pl_araim=max(pl_ff, max_k pl_faulted_k). `IntegrityReport::pl_araim/pl_ff`; `report.PL ← pl_araim` | 注入偏置卫星后 PL 升高 | `pl_ff, pl_faulted_k, pl_araim` | **DONE** |
| IAP-RQ-246 | FDE 闭环（检测→排除→重算） | Talk: 检测到异常则排除并重解 | `compute_core()`: fault_detected=(d_horiz>threshold); `AraimResult::detected_rows`; `IntegrityMonitor::run_araim()` 报告检测结果并更新 report.PL | 日志显示 "detected→excluded" 流程 | `araim_n_det, araim_detected_rows` | **DONE** |
| IAP-RQ-331 | 沿候选轨迹预测 ARAIM PL（规划用） | Talk §7.2 predicted PL_ARAIM at future waypoints | `include/iap/planner/predicted_araim.hpp/.cpp`: `PredictedAraimComputer`; VisibilityPredictor → SatGeometry 列表 → `Araim::predict_geometry()` (r=0) → pl_araim | 树冠路径候选 PL_pred 更高；开阔路径更低 | `pl_araim per waypoint` | **DONE** |
| IAP-RQ-421 | 沿轨迹的动态 AL(τ) | Talk: AL 来自障碍接近度（未来路径点） | `CandidateTrajectory::AL_pred` 字段; `IntegrityPlanner::set_al_fn()` 回调; `plan()` 为每个路径点计算 AL_pred[k] = al_fn_(wpt_pos) | 靠近障碍时 AL_i 更小 | `AL_pred[k]` | **DONE** |
| IAP-RQ-422 | 规划器使用 (PL_pred_ARAIM_i − AL_i) | Talk §7.3 hinge cost 使用预测 ARAIM PL | `IntegrityPlanner::evaluate()` 使用 traj.AL_pred[k] 替代标量 AL; `plan()` 当 use_araim_pl 时用 araim_predictor_.predict_araim_pl() 替换 PL_pred[k] | 完整性违约时规划器选择更安全（即使更长）的路径 | `J_integrity per waypoint` | **DONE** |
| IAP-RQ-050 | IMU 健康度：饱和/模型失配 → noise inflation / alarm | 传感器健康度影响可信度 | `src/health/imu_health.*` | 人为制造饱和或异常噪声 | `acc_norm, gyro_norm, sat_flag, gamma_imu` | TODO |
| IAP-RQ-060 | 规划目标：J(τ)=Σ hinge(PL_pred-AL)^2 + λ_goal*dist + λ_u*effort | 优化版 integrity-aware cost | `src/planner/cost.*` | 单步生成候选轨迹打分 | `J_total, J_integrity, J_goal, J_effort` | TODO |
| IAP-RQ-070 | 预测层：对候选轨迹预测 PL_pred（baseline：代理；升级：ARAIM） | 预测可见/可观测集合与 PL_pred | `src/predictor/*` | 对比不同轨迹的 PL_pred 差异 | `PL_pred(s), AL(s), IM_pred(s)` | TODO |
| IAP-RQ-080 | Receding horizon 闭环：执行第一段，重估计重规划 | active perception loop | `src/planner/mpc_loop.*` / `apps/*` | 跑闭环仿真/回放 | `chosen_traj_id, replanning_rate` | TODO |
| IAP-RQ-090 | 实验与消融：直飞 vs 协方差最小 vs 完整性驱动 | 证明减少违约 PL>AL | `apps/experiments/*` + `docs/*` | 批量跑场景并出表 | `violation_time, min_IM, path_len, time` | TODO |

---

## 2. 未映射改动（临时区）
> 如果你临时改了代码但还没决定它对应哪个需求，先把改动写在这里（提交前必须移入上表）。

- 2026-03-31: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - `CT_LIDAR_GPU + KERNEL` 现已完成本轮 README_REFACTOR 路线切换：旧 `IntegratedBSplineGICPFactorGPU` (`BUCKET`) 已从 `CMakeLists.txt`、运行时集成和 `test_bspline_gicp_factor.cpp` 中删除，公开配置也只保留 `ct_lidar_gpu_backend = KERNEL`。
  - shared GNSS state / carried-prior 的长包阻断项已通过 headless KERNEL 回放验收：最新验证中未再出现 `key "e0"`、`ValuesKeyDoesNotExist`、`failed to build bspline marginal survivor prior`、`authoritative incremental update failed`，并且后段窗口不再出现 `gnss_pr_factors = 0 / gnss_dop_factors = 0` 的塌零。
  - 为支撑上述验收，`IapSharedState` 与 `GnssHandler` 的 raw/epoch queue 容量已提升，`OdometryEstimationBSpline` 也已支持把迟到 epoch 回填到仍然活跃的 solve-domain segments，并在需要时触发 authoritative incremental factor re-add。

- 2026-03-31: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - `OdometryEstimationBSpline` 现已进一步按 `BSplineSolveDomain` 收紧 control-point anchor / prediction / smoothness priors，只让当前 local solve-domain control span 进入这部分先验，而不再默认把整条历史 active-window control states 全部拖回当前 solve。
  - 这让当前 batch-compatible 求解壳在“图组织方式”上更接近 `README_REFACTOR_CT_SOLVER` 所要求的 GLIM-style local-domain / incremental fixed-lag 路线，即使 authoritative long-lived solver 仍未最终替换完成。

- 2026-03-31: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - `BSplineIncrementalSolverSkeleton` 现已开始按稳定的 solve-domain segment id（`auxiliary_index`）而不是临时 ordinal 跟踪 active/new/retired segments；这让 KERNEL authoritative 路径可以按稳定 owner 维护 local-domain segment-local factor inventory。
  - `OdometryEstimationBSpline` 现已为 authoritative `CT_LIDAR_GPU + KERNEL` 路径维护 `incremental_segment_factor_indices_` 与 `incremental_prior_factor_indices_`：retired/replaced solve-domain segment 会显式 remove 对应 smoother factor indices，而仍然活跃的 solve-domain segment-local LiDAR / velocity / IMU / GNSS factors 则不会在每帧被整批 replace。
  - 同一轮 headless 长包验证现已跑到 400+ frame，期间未再出现 `key "e0"`、`ValuesKeyDoesNotExist`、`failed to build bspline marginal survivor prior` 或 `authoritative incremental update failed`；这说明 shared-state / carried-prior 的阻断性 missing-key failure 已不再打断 KERNEL authoritative 路径，但 GNSS 后段 `factor_count = 0` 的现象仍需继续验收解释。

- 2026-03-31: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - `ActiveSplineSegmentConstraint` 现已把 velocity / IMU / GNSS continuous-time factors 也收口进 per-segment 生命周期缓存；当前 batch-compatible 壳不再为同一 active solve-domain segment 每帧重新构造这些非 LiDAR 因子。
  - `OdometryEstimationBSpline` 现在会复用 cached velocity factor、cached IMU factor 列表以及 cached GNSS factor 列表，并继续与已存在的 LiDAR factor cache 协同工作，从而把更多 factor ownership 从 per-scan batch 建图逻辑前移到 solve-domain 生命周期层。

- 2026-03-31: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - `BSplineFixedLagStateRegistry` 现已新增 `seed_clock_values(...)`，开始把 solve-domain 所需的 segment clock states 显式种入 authoritative `Values`，而不是只在 batch-compatible 建图后半段按需补齐。
  - `OdometryEstimationBSpline` 现在会在 `BSplineIncrementalSolverSkeleton::prepare_update(...)` 之前，先种 shared `j / k / g / e / r` states 和局部求解域真正需要的 `c` keys，使 future incremental fixed-lag solver / carried-prior replay / key retirement 能共享同一份 authoritative key set。
  - B-spline 路径现已通过 `reset_bspline_incremental_smoother()` 重置一套 CT 专用 smoother shell，并显式注册 `s / u / j / k / g / c / e / r` 的 relinearization policy，避免未来长期存活 incremental solver 继续隐式沿用 legacy discrete-time threshold 集。
  - `test_bspline_fixed_lag_registry.cpp` 已补上 solve-domain clock seeding 回归测试，覆盖 auxiliary clock reuse、bias/drift forward propagation 与已有时钟状态保留语义。

- 2026-03-30: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - 新增 `docs/dev_ct/README_REFACTOR_CT_SOLVER.md`，正式将连续时间 odometry 求解器重构收口到“KERNEL-only public route + staged incremental fixed-lag migration”的执行规范。
  - 新增 `include/iap/odometry/ct_solve_domain.hpp`，引入 `ICTSolveDomain / BSplineSolveDomain`，并在 `OdometryEstimationBSpline` 中把公开连续时间 LiDAR solve scope 收口到“当前 segment + 最近重叠段”，不再默认覆盖全部历史 active-window segment。
  - 新增 `include/iap/odometry/ct_incremental_solver_skeleton.hpp` / `src/iap/odometry/ct_incremental_solver_skeleton.cpp`，引入 `BSplineIncrementalSolverSkeleton / CTSolverLifecycleDelta`，开始显式跟踪 local-domain 的 active/new/retired segments、new/retired keys 和 future incremental smoother 需要的 `new_values / new_stamps` 生命周期载荷。
  - 新增 `include/iap/odometry/shared_target_handle.hpp`，引入 `ISharedTargetHandle / SharedTargetHandle`，开始把 active segment 的 target metadata 从 per-segment runtime ownership 迁移到 shared target handle 语义。
  - `OdometryEstimationBSpline` 现已维护按 `target identity + revision` 复用的 shared target handle cache，`IntegratedBSplineGICPFactorGPUKernel` 也已通过 `refresh_target_handle(...)` 直接绑定 shared target GPU resources，从而把 KERNEL 的 target refresh 从“每个 factor 重建 target GPU map”推进到“共享 target-side GPU resource handle”。
  - `OdometryEstimationBSpline` 现已在 pipeline summary 中显式输出 `incremental_domain_prepare_ms`、active/new/retired solve-domain segments，以及 new/retired key counts，从而把未来长期存活 incremental fixed-lag solver 所需的 add/remove surface 先稳定成可测试、可观测的兼容壳。
  - `config_odometry_bspline.json` 与 `OdometryEstimationBSpline` 现已把 `KERNEL` 固定为公开默认 GPU route；公开选择 `BUCKET` 时会直接报 deprecated error，只有设置 `IAP_ALLOW_DEPRECATED_BUCKET=1` 才允许内部 parity / 一次性 A/B 使用。
  - 新增 `test_ct_solve_domain.cpp`、`test_ct_incremental_solver_skeleton.cpp` 与 `test_shared_target_handle.cpp`，分别覆盖 local-domain 选择/retirement 语义、incremental solver lifecycle skeleton 的 key/segment add-remove 语义，以及 shared-target-handle revision/identity/metadata 语义；`test_bspline_gicp_factor.cpp` 现已进一步覆盖 KERNEL 通过 shared target resources 绑定/切换 target-side GPU state 的路径。
- 2026-03-29: IAP-RQ-300 / IAP-RQ-410
  - `OdometryEstimationBSpline` 现已把 `BUCKET` frontend 分成 runtime / diagnostic 两种结果回收模式：默认运行态只回收当前 segment 的必要 LiDAR result，用于 `icp_quality` 和连续时间主链输出；整窗 full `make_result / aggregate / numeric audit / degeneracy` 只在 profile / CSV / warning 开关开启时执行。
  - active-window LiDAR factor 现已进入缓存复用阶段：CPU CT LiDAR factor 会在 active segment 生命周期内直接复用；GPU `BUCKET` factor 也会保留 source bucketization，仅在 target identity/revision 变化时刷新 target-side GPU resources。
  - `bspline ct pipeline-summary` 现已进一步输出 `graph_lidar_factor_new_build_ms / graph_lidar_factor_target_refresh_ms / graph_lidar_factor_reused_attach_ms / cache hit-miss / post_lidar_factor_error_ms / post_lidar_numeric_audit_ms / post_lidar_degeneracy_ms / post_lidar_result_pack_ms / post_lidar_window_aggregate_ms`，作为 cached-BUCKET 与未来 KERNEL backend 的统一 A/B baseline。
  - carried prior builder 现已对 previous prior retained keys 做缺键过滤，ECEF shared states 也会在 GNSS anchor 已初始化时稳定 seed 到当前 values 中；对应 regression 已补到 `test_bspline_marginalization.cpp`，直接覆盖此前 `key "e0" does not exist in the Values` 的 shared-state lifecycle failure。
  - 已继续按 `SLAM_FINISH_PLAN.md` 推进 `M2 / WP2`：`IntegratedBSplineGICPFactor` 现已支持 `NUMERIC_FULL / SEMI_ANALYTIC` 两种 Jacobian mode，默认走“解析 control-translation block + 数值 control-rotation block”的半解析路径，并保留 full-numeric 作为 A/B/debug 基线。
  - continuous-time LiDAR factor 现已补上显式 outlier/robust handling：可按 whitened residual norm 做 outlier gate，并支持 `NONE / HUBER / CAUCHY` 三种 robust kernel 及其宽度配置。
  - `OdometryEstimationBSpline` 现已将 LiDAR Jacobian mode、数值差分步长、outlier threshold、robust kernel 类型和宽度都接成配置项，并将当前 factor 的 inlier / rmse proxy 回填到 `EstimationFrame::icp_quality`。
  - `IntegratedBSplineGICPFactor` profiling stats 现已扩展到 `matched / inlier / rejected_distance / rejected_outlier / inlier_ratio / mean_robust_weight`，用于后续 profiling / GPU 设计和退化分析。
  - `test_bspline_gicp_factor.cpp` 现已补充 perturbed-state 下的半解析 linearization consistency 检查，以及 outlier threshold / robust kernel 对坏匹配抑制行为的专门测试。
  - CT LiDAR correspondence 现已从“单一最近欧氏邻点”推进到“k-NN 候选 + Mahalanobis score 选择”；`IntegratedBSplineGICPFactor` 新增 `correspondence_candidate_count` / `correspondence_accept_ratio` 配置能力，可在局部几何模糊时挑选更合理的 target match。
  - factor 现已支持 best/second-best Mahalanobis score 的 ambiguity rejection，并在 profiling stats 中新增 `rejected_ambiguity_count`，便于区分距离拒绝、模糊拒绝和 outlier gate 拒绝。
  - `SEMI_ANALYTIC` 路径现已把控制点旋转块的数值差分预缓存到“每个 spline time node 一次”，而不是在每个 source point 上反复做旋转差分，进一步降低 hot path 的重复数值线性化成本。
  - `test_bspline_gicp_factor.cpp` 现已新增 Mahalanobis candidate selection 优于单最近邻，以及 ambiguity-ratio 拒绝近似等价对应的专门测试。
  - `SEMI_ANALYTIC` 路径现已进一步把旋转块改成 normalized-quaternion blend 的解析链式 Jacobian，减少对 time-node 级旋转数值差分缓存的依赖，同时保留 `NUMERIC_FULL` 作为 reference baseline。
  - factor 现已新增 `correspondence_min_score_gap` 和 `robust_weight_floor` 两个工程化 gate，可分别收紧近似等价对应和被 soft kernel 严重降权的坏匹配；profiling stats 现已新增 `rejected_robust_count`。
  - `OdometryEstimationBSpline` 现已把 snapshot target policy 收紧为“满足最小 frame/point 支持并可选 age gate 才使用 local snapshot，否则回退 global ivox reference”，并在 trace 日志中输出 snapshot support diagnostics。
  - `test_bspline_gicp_factor.cpp` 现已补充 semi-analytic 对 `NUMERIC_FULL` 的 predicted-error 对照、absolute score-gap ambiguity rejection，以及 robust-weight-floor rejection 的专门测试。
  - `IntegratedBSplineGICPFactor` 现已新增 `check_against_numeric_full(...)`，可在当前状态下直接把 `SEMI_ANALYTIC` 和本地重建的 `NUMERIC_FULL` baseline 做 rotation / translation 分块 predicted-error 对照。
  - 该 numeric-reference 入口现已进一步补上 axis-wise 旋转块 audit，可分别对 3 个局部旋转轴输出 predicted-error 对照，并给出 `worst_rotation_axis / max_rotation_axis_rel_error / mean_rotation_axis_rel_error`。
  - CT LiDAR profiling 现已进一步输出 `unique_target_count / unique_target_ratio / max_target_reuse / max_target_reuse_ratio / mean(max)_match_distance / mean_match_score / mean_score_gap / mean_score_ratio`，用于识别 correspondence reuse 和 target degeneracy。
  - CT LiDAR profiling 现已新增 `time_bucket_count / max_time_bucket_population / mean_time_bucket_population / candidate_evaluation_count / mean_candidates_per_source`，作为后续 GPU 连续时间 LiDAR factor 设计的 baseline 指标。
  - 已新增 `bspline_lidar_factor_result.hpp`，统一 `BSplineLidarFactorProfile / NumericAudit / DegeneracyReport / FactorResult / WindowProfileSummary`，作为 CPU 当前实现和 GPU 后续实现共享的 CT LiDAR profile/result 接口。
  - `IntegratedBSplineGICPFactor` 现已支持 `profiling_report()` / `make_result(...)`，将单 factor 的 profiling、numeric-reference audit 和 degeneracy diagnostics 收口成一份可聚合结果，而不再只通过 ad-hoc trace 字段暴露。
  - `OdometryEstimationBSpline` 现已在每轮求解后聚合 active-window 内所有 CT LiDAR factor 结果，并新增 `bspline ct lidar cpu-summary` 汇总日志，输出整窗 weighted match/inlier、candidate baseline、time-bucket baseline、numeric-audit 峰值和 warning 数量。
  - `test_bspline_gicp_factor.cpp` 现已补充窗口级 result aggregation 单测，为后续 GPU CT LiDAR factor 复用同一 summary/result 接口提供回归基线。
  - 统一 LiDAR return surface 现已扩展到 backend-agnostic builders：`make_bspline_lidar_factor_result(...)` / `make_bspline_lidar_minimal_result(...)` 支持 GPU factor 先以最小 profile（inlier/error/source-target counts）形式返回统一结果。
  - `BSplineLidarWindowProfileSummary` 现已显式区分 `detailed_profile_count` 和 `minimal_profile_count`，确保 GPU minimal profiles 不会污染 CPU detailed profiles 的 diversity / timing / bucket 基线统计。
  - `OdometryEstimationGPU` 现已把 `IntegratedVGICPFactorGPU` 封装成统一 `BSplineLidarFactorResult`，并新增 `vgicp gpu-summary` trace，作为未来 CT GPU LiDAR factor 复用这套 return surface 的真实 GPU caller baseline。
  - `test_bspline_gicp_factor.cpp` 现已补充 minimal GPU result 和 minimal-profile summary 的专门单测，验证 unified return surface 在 GPU minimal 模式下的 weighted summary 语义。
  - 已新增 `IntegratedBSplineGICPFactorGPU`：source scan 会按 per-point time bucket 切分成多个 GPU unary `IntegratedVGICPFactorGPU` 子因子，并通过 4 控制点 spline pose 的数值 Jacobian，把 bucket Hessian 回映射到控制点窗口，形成真正的连续时间 GPU LiDAR factor 本体。
  - `OdometryEstimationBSpline` 现已支持 `frontend_mode = CT_LIDAR_GPU`，并把这类 GPU CT LiDAR factor 直接挂入 active-window 图优化；同一条 fixed-lag LiDAR/IMU/GNSS 生命周期主线现在已可选择 CPU 或 GPU CT LiDAR frontend。
  - 新的 GPU CT factor 现已复用统一的 `BSplineLidarFactorResult / WindowProfileSummary` 返回面，并新增 `bspline ct lidar gpu-summary` 与 `gpu-factor` trace；当前 GPU profiling 已从最初的 minimal profile 进一步扩展到“GPU timing baseline + CPU-side correspondence audit”的 unified detailed profile。
  - `test_bspline_gicp_factor.cpp` 现已补充 `IntegratedBSplineGICPFactorGPU` 的 CUDA smoke test，验证该 factor 能 linearize 并返回有效统一 GPU result/profile；本轮 smoke test 已切换到与生产路径一致的 `StreamTempBufferRoundRobin` 资源管理，并在真实 GPU 上运行。
  - `IntegratedBSplineGICPFactorGPU` 现已支持 `NUMERIC_FULL / SEMI_ANALYTIC` 两种控制点 Jacobian 模式；新的半解析路径复用了共享 `bspline_pose_jacobian.hpp` 里的 normalized-quaternion-blend 链式 Jacobian，只保留 `NUMERIC_FULL` 作为 GPU baseline/debug 模式。
  - `OdometryEstimationBSpline` 现已把 `ct_lidar_jacobian_mode` 下发到 GPU CT LiDAR factor；`test_bspline_gicp_factor.cpp` 也已新增不依赖 CUDA 设备的 spline-pose Jacobian 数值对照测试，用来持续校验这条共享半解析 Jacobian 链路。
  - `IntegratedBSplineGICPFactorGPU` 现已补上 `check_against_numeric_full(...)` 与 `diagnose_degeneracy(...)`，并支持与 CPU 一致的 `max_correspondence_distance / candidate_count / ambiguity ratio / score-gap gate / outlier threshold / robust kernel / robust weight floor` 配置面；统一 `make_result(...)` 现在也能携带 GPU `numeric_audit / degeneracy` payload。
  - `OdometryEstimationBSpline` 现已在 `CT_LIDAR_GPU` 路径下输出 GPU numeric-reference drift 和 GPU degeneracy warnings；`bspline ct lidar gpu-summary` / `gpu-factor` trace 现已包含 `candidate_evaluation_count / unique_target_ratio / max_target_reuse_ratio / mean_score_gap / rejection counts` 等 richer correspondence diagnostics。
  - CT LiDAR target lookup 现已统一回到 frozen `iVox` 的原生 search/index 域，避免把 copied-point KD-tree 返回的索引和 `iVox` 内部 point/cov 访问混用。
  - `IntegratedBSplineGICPFactor` 现已新增 `diagnose_degeneracy(...)`，把当前 target/correspondence 状态转成可重用的 warning flags；`OdometryEstimationBSpline` 进一步把 `ct_lidar_warn_*` 阈值接成配置并输出专门的 degeneracy warning line。
  - `OdometryEstimationBSpline` 现已把 `ct_lidar_profile_numeric_reference` / `ct_lidar_numeric_reference_scale` 接成配置项，并把 numeric-reference drift 与 correspondence degeneracy diagnostics 写入 CT LiDAR trace 日志。
  - `test_bspline_gicp_factor.cpp` 现已补充 numeric-reference axis audit、profiling baseline、degeneracy diagnostics 和 ambiguity-rejection warning 的专门测试。
  - `bspline_lidar_factor_result.hpp` 现已进一步扩展为 `BSplineLidarBaselineExport` + CSV helpers；`OdometryEstimationBSpline` 新增 `ct_lidar_export_baseline_csv / ct_lidar_baseline_csv_path` 配置，可把每轮 active-window CT LiDAR 结果统一导出为 `window_summary + factor_result rows`，作为后续 GPU kernel-level CT LiDAR 优化的稳定 baseline/result 文件接口。
  - `test_bspline_gicp_factor.cpp` 现已补充 baseline CSV export 单测，验证 unified result surface 的 header、summary/factor row 以及 current-factor 标记语义。
  - `OdometryEstimationBSpline` 现已新增 `ct_lidar_gpu_backend = BUCKET | KERNEL` 配置；当前工程 GPU CT LiDAR 路径已冻结为 `BUCKET` backend，而未来 kernel-level spline-native GPU backend 将通过独立 `KERNEL` 入口接入，不会与现有工程版运行时语义混用。
  - `IntegratedBSplineGICPFactorGPUKernel` 现已作为独立 backend 落地：该 factor 直接在 GPU 上按点时间戳查询 4 控制点 spline pose、做 correspondence / gating / robust weighting，并累计 `24x24` Hessian / `24x1` gradient，不再经过 `bucket pose -> unary VGICP -> map back` 的中间层。
  - `OdometryEstimationBSpline` 现已把 `CT_LIDAR_GPU + KERNEL` 直接接入 active-window 主链，并为 `BUCKET` / `KERNEL` 分别保留独立 cache slot；这使得冻结的 BUCKET baseline 和新的 KERNEL backend 可以在同一条 odometry 生命周期中直接 A/B，而不会相互污染缓存语义。
  - `BSplineLidarFactorProfile / WindowProfileSummary / baseline CSV` 现已补充 kernel-stage timing 字段：`kernel_pose_query_ms / kernel_correspondence_ms / kernel_residual_weight_ms / kernel_reduction_ms / host_sync_ms / host_result_pack_ms`，作为后续 `cached BUCKET vs KERNEL` 的统一比较面。
  - `test_bspline_gicp_factor.cpp` 现已新增 `GpuKernelFactorLinearizesAndReturnsUnifiedProfile` 与 `GpuKernelFactorRefreshesTargetWithoutRebuildingSourceStaging` 两类 CUDA 测试，覆盖 KERNEL smoke path、current-factor numeric parity，以及 target refresh 不重建 source-side staging 的生命周期要求。
- 2026-03-29: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - `OdometryEstimationBSpline` 不再在建图前立即裁掉超过 lag 边界的 control points / segment constraints，而是先带着这些“即将滑出”的状态完成当前轮优化。
  - 新增 removable-factor marginalization：上一轮 carried prior 与本轮即将被 lag pruning 移除的 LiDAR/IMU/velocity/GNSS/clock/smoothness 因子会被单独收集、线性化，并对 survivor states 做边缘化。
  - 新的 carried prior 不再只盯住 boundary pose / velocity / clock 子集，而是会覆盖修剪后仍然保留的 control points、surviving auxiliary states，以及 persistent shared IMU/GNSS alignment states。
  - 新增 `BSplineMarginalizationPartition`，统一 control points、auxiliary states 和 shared IMU/GNSS alignment states 的 survivor/removable 归属判定，减少 `OdometryEstimationBSpline` 内部零散的分区逻辑。
  - 新增 `build_bspline_carried_prior(...)`，把 removable nonlinear graph 线性化后边缘化到 survivor key 集合，并重新封装为下一轮可重放的 carried prior。
  - 新增 `test_bspline_marginalization.cpp`，专门校验 partition 归属以及 replayed carried prior 与参考 marginal graph 在 survivor perturbation 下的误差一致性。
  - `BSplineMarginalizationPartition` 现已进一步提供 factor ownership 分类与 carried-prior replay 安全检查，显式区分 `survivor-only / removable / foreign`。
  - carried prior 现已改为以 `GaussianFactorGraph + linearization_point + retained_keys` 的线性形式缓存；进入当前优化图时再转成 `LinearContainerFactor`，而不是继续把旧 prior 混进 removable nonlinear graph 再统一线性化。
  - `test_bspline_marginalization.cpp` 现已补充 foreign-key ownership 检测，以及“上一轮线性 carried prior + 本轮 removable nonlinear factors”组合的一致性校验。
  - 新增 `BSplineFixedLagStateRegistry`，统一 active control buffer、segment 生命周期以及 auxiliary velocity/clock state 的保留与裁剪逻辑。
  - `OdometryEstimationBSpline` 现在通过该 registry 统一执行 window append、segment append、lag pruning、marginalization-state 导出和 auxiliary-value filtering。
  - 新增 `test_bspline_fixed_lag_registry.cpp`，校验 unified fixed-lag registry 的 control-buffer/segment 同步裁剪与 auxiliary state 生命周期行为。
  - `BSplineFixedLagStateRegistry` 现已进一步纳入 shared IMU/GNSS states：gyro bias、accel bias、gravity、ECEF origin、ECEF rotation 的 seed/update 和 graph ownership 也由该 registry 统一管理。
  - `OdometryEstimationBSpline` 不再单独维护这些 shared-state snapshot，而是通过 registry 统一完成 graph seeding、GNSS anchor 激活、求解后回写和持续发布。
  - `test_bspline_fixed_lag_registry.cpp` 现已补充 shared-state seed/update round-trip 校验。
  - `BSplineFixedLagStateRegistry` 现已进一步提供 `BSplineFixedLagTelemetry`，显式输出 lag 区间、active segment、aux/shared-state 数量、GNSS anchor readiness 和 `Empty / WindowSeeded / TrackingLidar / TrackingLidarGnss` 生命周期阶段。
  - `OdometryEstimationBSpline` 现已在 continuous-time 轨迹发布链路上同步向 `IapSharedState` 发布 fixed-lag telemetry，并以 trace 日志暴露生命周期状态机。
  - `test_bspline_fixed_lag_registry.cpp` 现已补充 lifecycle telemetry/state-machine 单测，覆盖 empty、window-seeded、LiDAR-tracking 和 LiDAR+GNSS-tracking 的状态迁移与计数语义。
  - 已开始 `M2 / WP2` 的 LiDAR factor 工程化：`OdometryEstimationBSpline` 现已支持 `ACTIVE_WINDOW_SNAPSHOT / GLOBAL_IVOX_REFERENCE` 两种 CT LiDAR target mode，并新增 snapshot frame-window 参数来约束 frozen target 生命周期。
  - active CT LiDAR segment 现已显式记录 target mode、contributing frames、target point count 和 target build latency，并将当前 segment 的 factor profiling / target diagnostics 输出到日志。
  - `IntegratedBSplineGICPFactor` 现已新增 profiling stats 与 linearization check 入口，用于后续解析 Jacobian 的基线校验。
  - CT LiDAR target tree 现已统一改为从 `voxel_points()` 导出的 `PointCloud` 构建，避免 `KdTree2<iVox>` 的 traits 不稳定路径。
  - 新增 `test_bspline_gicp_factor.cpp`，覆盖 LiDAR factor 的误差响应、linearization check 有效性和 profiling stats 行为。
- 2026-03-28: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - 已开始按 `docs/dev_ct/SLAM_FINISH_PLAN.md` 执行 `M1 / WP1`，先收口 fixed-lag 主状态边界。
  - `OdometryEstimationBSpline::ActiveSplineMarginalPrior` 现在会结构化快照 boundary pose、boundary auxiliary index，以及对应的 velocity / clock state。
  - 每轮求解结束后，`OdometryEstimationBSpline` 会对 boundary pose / velocity / clock 子集提取 `jointMarginalInformation(...)`，并保存与之匹配的 linearization point。
  - 下一轮优化会优先把这组 boundary information 通过 `LinearContainerFactor(HessianFactor, linearization_point)` 回灌到 fixed-lag 图中；若提取失败，则回退到手工 pose / velocity / clock priors。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - `OdometryEstimationBSpline` 目前是连续时间 B-spline 骨架层，不是最终的 spline-native LiDAR/IMU/GNSS factor graph。
  - 现阶段它复用了现有 LiDAR-IMU odometry 后端，并在优化后发布 `ContinuousTrajectoryView` / `SplineControlAccess`，用于 planner / viewer / debug 接线与后续 Phase-1B/1C 演进。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - 已开始 Phase-1B 最小实现：引入 4 控制点窗口和 CPU 连续时间 LiDAR factor。
  - 当前 `CT_LIDAR_CPU` 路径仍是局部 LM frontend，不是最终的 fixed-lag spline-native LiDAR+IMU+GNSS 图优化器。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - 已补上 active spline window buffer、lag pruning 和多段控制点发布骨架。
  - 当前控制窗口已经具备 fixed-lag 形式的状态保存与对外发布，但优化本身仍然主要按“最新 segment 的局部 LM”运行。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - 已将 active spline window 推进到优化变量层：整窗控制点现在会共同进入 LM 初值和 smoothness / boundary prior 图结构。
  - 当前仍是“整窗变量 + 最新 segment LiDAR factor”的过渡实现，尚未达到“多段 LiDAR factors 联合优化”的最终目标。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - 已将 active window 内多个 segment 各自挂上连续时间 LiDAR factor，并在同一 LM 图内共享重叠控制点。
  - 当前仍是过渡实现：多个 segment 共享同一个 active target map，尚未引入 segment-specific target snapshot，也还没有 IMU/GNSS 连续时间 factors。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - 已为每个 active segment 冻结 local target snapshot，并加入显式 marginal prior 以约束 lag-window 起始边界。
  - 已在同一个 fixed-lag LM 图内加入 per-segment continuous-time IMU factor；当前 IMU 因子是最小可用的 relative-pose 版本，bias 仍为固定输入而非联合优化状态。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - 已将 per-segment IMU 因子从 relative-pose 过渡版本推进到 sample-based 版本，开始按 IMU 时间戳直接约束 spline 的角速度与线加速度。
  - 当前仍是工程化最小实现：bias 仍为固定输入，角速度/加速度 Jacobian 仍为数值形式，尚未升级到联合 bias/velocity/gravity 状态。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - 已将 shared gyro bias、accel bias、gravity 提升为 fixed-lag 图中的显式状态，并由 `IntegratedBSplineIMUFactor` 直接联合优化。
  - 当前 velocity 仍由控制点位姿差分导出，尚未作为独立状态进入图；IMU Jacobian 也仍然是数值形式。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - 已将 per-segment velocity 提升为 fixed-lag 图中的显式状态，并新增 `IntegratedBSplineVelocityFactor` 将 velocity state 与 pose spline 绑定。
  - 当前 planner / control-access 已能读取这组显式 velocity states，但评分逻辑尚未系统消费它们；IMU / velocity Jacobian 也仍然是数值形式。
- 2026-03-27: IAP-RQ-300 / IAP-RQ-410
  - `IntegrityPlanner` 已开始使用 `SplineControlAccess` 锚定当前 planning seed 时刻，并对候选轨迹未来 waypoint 直接查询已发布的 continuous-time trajectory sample。
  - 当前已开始用 future-time sample 抬高 `sigma_pred / PL_pred` 并加入短时 `w_ct_align` 对齐代价，但仍只利用当前已发布的短窗 trajectory，而没有升级到真正的 B-spline candidate planning。
- 2026-03-27: IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410
  - 已开始 Phase-1C 最小接入：`OdometryEstimationBSpline` 现在会直接消费 shared GNSS epoch queue / ECEF anchor，并把 per-segment pseudorange / doppler factor 接入同一个 fixed-lag LM 图。
  - 当前仍是过渡实现：GNSS epoch 缓存仍主要由 `gnss_extension` 生产，BSpline 主链只消费 shared queue；因子 Jacobian 仍以数值/半解析混合方式实现，完整的 handler 内核化迁移还没完成。
- 2026-03-22: IAP-RQ-010 / IAP-RQ-200
  - GNSS clock single-owner contract收敛：`clock_owner_mode` 跨模块联动，默认切到 `gnss`。
  - 增加 ready 时序契约：`IapSharedState::{set,clear,is}_clock_ready`；GNSS 生产 ready，odometry 在 GNSS-owner 下仅 `current+ready` 才读 `C(i)`。
  - 观测与一致性：`KeyLifecycleMonitor` 记录 ownership/missing/conflict/violation；A/B 验收日志显示 `c missing/conflicts/violations = 0`，无 hard optimizer error。
