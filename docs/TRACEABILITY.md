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
| IAP-RQ-002 | Build artifacts: compile_commands.json + 最小 demo | 开发基础设施 | `apps/iap_status.cpp`, `launch/iap_demo.launch.py`, `.clangd`, `CMakeLists.txt` (IAP_VERSION, install launch/) | `ros2 run iap iap_status config` 输出 OK; clangd 通过 .clangd 读取 build/iap/ | `iap_status: OK` | **DONE** |
| IAP-RQ-010 | 状态扩展：clk_bias δt [m], clk_drift δṫ [m/s] 加入 factor graph (`C(i)=gtsam::Vector2`) | 状态向量包含 clock bias/drift | `include/iap/odometry/estimation_frame.hpp`, `odometry_estimation_imu.hpp/.cpp` | `colcon build` 通过；`trace` 日志含 `clk_bias/clk_drift` | `clk_bias(m), clk_drift(m/s)` | **DONE** |
| IAP-RQ-015 | Expose Σ_p 位置协方差块（3×3）到 EstimationFrame；供 PL proxy 使用 | 保护级别需基于不确定性 | `estimation_frame.hpp` (+sigma_p), `odometry_estimation_imu.cpp` (marginalCovariance) | `trace` 日志含 `trace(Σ_p)` 和 `lambda_max(Σ_p)` | `trace_sigma_p, lambda_max_sigma_p, PL_proxy` | **DONE** |
| IAP-RQ-020 | GNSS 紧耦合观测：伪距 + 多普勒建因子（含 clock bias/drift，per-sat） | GNSS tightly-coupled | `include/iap/gnss/`, `src/iap/gnss/` (PseudorangeFactor, DopplerFactor, GnssHandler, **GnssExtensionModule** ROS2 bridge); epoch/factor logs at `info` level | 关闭/开启 GNSS 对比 | `res_pr, res_dop, clk, clk_dot` | **DONE** |
| IAP-RQ-030 | GNSS per-satellite NIS gating：downweight / exclude（RAIM-ish baseline） | 卫星级完整性、FDE 思路 | `src/integrity/gnss_integrity.*` | 注入某卫星 bias，触发剔星 | `sat_nis, exclude_sats, gamma_R, global_nis` | TODO |
| IAP-RQ-040 | LiDAR ICP 因子健康度：退化/错配检测 → noise inflation / drop factor | trunk/几何退化会影响可观测性 | `EstimationFrame::IcpQuality`, `odometry_estimation_cpu.cpp` (Hessian cond, gamma_lidar) | 走廊/单面结构场景退化 | `icp_rmse, inliers, cond, gamma_lidar, drop` | **DONE** |
| IAP-RQ-100 | Trunk 检测与圆拟合（中心, 半径, confidence） | 树干几何地标 | `include/iap/trunk/trunk_types.hpp`, `trunk_detector.hpp/.cpp` (Kasa fit, grid BFS) | 对进树林场景启动，日志输出 trunk 数量/半径/置信度 | `trunk_count, radii, confidence[]` | **DONE** |
| IAP-RQ-110 | Trunk 健康度因子接口 (Baseline-A: 不入图) | 树干布局影响 LiDAR 可观测性 | `TrunkDetector::health_factor()` ([0,1]) | health~0 时硬件应填充更大噪声 | `trunk_health` | **DONE (Baseline-A)** |
| IAP-RQ-120 | TDOP 指标（角度多样性） | 树干几何与完整性联系 | `TrunkDetectionResult::tdop/tdop2/lambda_min_H` | 树更分散时 TDOP 下降 | `tdop, lambda_min_H` | **DONE** |
| IAP-RQ-200 | Integrity 输出：PL/AL/IM/mode + 关键中间量 | PL < AL 安全条件 | `include/iap/integrity/integrity_types.hpp`, `integrity_monitor.hpp/.cpp` | PL/AL/IM 曲线可画；mode 切换可复现 | `PL, AL, IM, mode` | **DONE** |
| IAP-RQ-210 | Alert Limit AL 由障碍距离动态给出 | 近障碍时 AL 缩小 | `IntegrityMonitor::compute_AL()`, `set_obstacle_distance()` | 越靠近障碍 AL 越小；日志可见 | `AL, obstacle_dist` | **DONE** |
| IAP-RQ-220 | GNSS per-satellite NIS gating（RAIM-ish） | 卫星级 FDE | `IntegrityMonitor::run_gnss_gating()` (chi2 test, gamma_R, FDE greedy) | 注入 bias 卫星被降权/剔除 | `sat_nis, gamma_R, excluded_sats` | **DONE** |
| IAP-RQ-300 | 候选轨迹生成（motion primitives） | 运动原语离散化候选轨迹 | `include/iap/planner/trajectory_types.hpp`, `trajectory_generator.hpp/.cpp` | 生成 M 条候选轨迹并可视化时间戳点序列 | `trajectory_count, speeds, yaw_rates` | **DONE** |
| IAP-RQ-310 | 预测可见/可观测性集合（占位） | ray-check 遮挡预测 | `include/iap/planner/predicted_integrity.hpp/.cpp` (placeholder) | TODO: 接地图后做 ray-check；当前返回占位值 | `n_vis_placeholder` | **DONE (placeholder)** |
| IAP-RQ-320 | 协方差传播 → Σ_pred → PL_pred | PL 预测供规划使用 | `include/iap/planner/predicted_integrity.hpp/.cpp` (sigma growth, K_pl=3.0) | PL_pred 随时间增长且不同轨迹有差异；σ_grow 可配置 | `PL_pred(s), sigma_pred(s)` | **DONE (baseline)** |
| IAP-RQ-400 | Integrity-aware planning objective | hinge(PL_pred−AL)²代价+goal+effort | `include/iap/planner/integrity_planner.hpp/.cpp`; `evaluate()` | IM<0时选绕行轨迹；J_integrity > J_goal场景可复现 | `J_total, J_integrity, J_goal, J_effort` | **DONE** |
| IAP-RQ-410 | Receding horizon loop | 执行Δt后重规划 | `IntegrityPlanner::execution_target()`, `plan()` | 调用`plan()`+`execution_target()`模拟多步闭环 | `chosen_traj_id, dt_execute` | **DONE** |
| IAP-RQ-500 | 三种 baseline（Passive/CovMin/IntegAware） | 对比 integrity 驱动的优势 | `apps/iap_experiment.cpp` (run_baseline ×3) | 同场景三 baseline 均输出指标 CSV | `baseline, violation_frac, avg_PL, mission_success` | **DONE (stub)** |
| IAP-RQ-510 | 指标：Time(PL>AL)%, AvgPL, MinIM, path/time/effort | 量化对比表格 | `include/iap/experiments/metrics.hpp` (MetricsCollector, write_comparison_table) | `ros2 run iap iap_experiment` 输出 /tmp/*_summary.md | `violation%, avg_PL, min_IM, path_len, time, effort` | **DONE (stub)** |
| IAP-RQ-900 | 自动生成 IEEE Trans methodology.tex（流程图+模块小节+公式） | 论文写作辅助 | `tools/gen_methodology.py` → `docs/methodology/methodology.tex`, `docs/figures/system_flow.tex` | `python3 tools/gen_methodology.py` 生成 .tex；结构无误（12 env 平衡） | gen exit code 0; env mismatch=0 | **DONE** |
| IAP-RQ-311 | 局部占用栅格供光线检测 | Talk §7.2 预测可见卫星/地标 | `include/iap/map/local_occupancy.hpp`, `src/iap/map/local_occupancy.cpp`; VoxelKey + Morton hash; DDA ray traversal | 合成占用体素验证光线命中/未命中正确 | `ray_occluded bool, occupancy_ratio κ` | **DONE** |
| IAP-RQ-312 | 对候选路径点预测卫星可见性集合 V̂(τ) | Talk §7.2 predicted V̂, geometry | `include/iap/gnss/visibility_predictor.hpp`, `src/iap/gnss/visibility_predictor.cpp`; ENU dir from (el,az); ray_occluded per sat | 树冠地图下移动减少 n_vis；开阔增加 | `n_vis, vis_flags[], mean_kappa` | **DONE** |
| IAP-RQ-313 | 估计 LOS 上的冠层密度 κ（预测时刻） | Talk §3.2 σ_eff(κ,θ) | `VisibilityPredictor::predict()` 调用 `occupancy_ratio()` → κ per satellite; `SatObs.kappa` 字段 | κ 在密集冠层下增大；开阔时接近 0 | `kappa per sat, mean_kappa` | **DONE** |
| IAP-RQ-314 | 实现 σ_eff(κ,θ) 和权重矩阵 W | Talk §3.2 σ²_eff = σ²_c · exp(α κ / sin θ) | `include/iap/gnss/canopy_noise_model.hpp` (header-only); `sigma_eff_canopy()`, `info_weight_canopy()` | 相同仰角 κ 更高 → σ_eff 更大，单调性 | `sigma_eff per sat` | **DONE** |
| IAP-RQ-321 | 使 PL_pred 与轨迹相关（替换单一 sigma 增长） | Talk §7.2 predicted covariance from predicted geometry | `predicted_integrity.hpp/.cpp`: `set_occupancy/set_epoch` API; `sigma_grow_at(pos)` = sigma_grow × max(1, f(n_vis, κ)) | 不同候选轨迹产生不同的 PL_pred 序列 | `sigma_grow_eff(s), PL_pred(s)` | **DONE** |
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

- (none)

