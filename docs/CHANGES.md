# Changes Log (IAP)

> 规则：任何代码改动必须在这里记录，并包含 IAP-RQ-XXX。

## Unreleased
- (none)

## 2026-03-05 (Phase-2)
- IAP-RQ-131/132/133: Trunk data association + TrunkFactor + confidence-weighted TDOP.
  - `include/iap/trunk/trunk_map.hpp` + `src/iap/trunk/trunk_map.cpp`: `TrunkMap` with EMA-smoothed landmark table; nearest-XY + radius-gate association; stale pruning; `confirmed_landmarks()`; `TrunkLandmark{id, center_xy, radius, confidence, seen_count, last_stamp}`.
  - `include/iap/trunk/trunk_factor.hpp` + `src/iap/trunk/trunk_factor.cpp`: `TrunkFactor : NoiseModelFactor1<Pose3>`; 3D residual `r = z_k − R^T(c_k−p)`; analytical H (3×6); `make_noise(confidence)` diagonal noise deflated by `1/√conf`.
  - `trunk_types.hpp`: `TrunkDetectionResult` += `tdop_weighted` field (IAP-RQ-133).
  - `trunk_detector.cpp`: `compute_tdop()` extended — W = diag(conf²), TDOP_W = sqrt(trace((G^T W G)^{-1})).
  - CMakeLists.txt: +trunk_map.cpp, +trunk_factor.cpp.
  - `colcon build` passes.

## 2026-03-05 (Phase-1)
- IAP-RQ-311/312/313/314/321: Local occupancy + visibility predictor + canopy noise model + trajectory-dependent PL_pred.
  - `include/iap/map/local_occupancy.hpp` + `src/iap/map/local_occupancy.cpp`: `LocalOccupancyGrid` (voxel hash `unordered_map<VoxelKey, uint8_t>`, Morton hash); Amanatides & Woo DDA ray traversal; `occupancy_ratio(origin, dir, L)` → κ; `insert(cloud, T_world_sensor)`.
  - `include/iap/gnss/canopy_noise_model.hpp` (header-only): `σ_eff(κ, θ) = σ_c · exp(0.5·α·κ / sin θ)` and `info_weight_canopy()` (Talk §3.2).
  - `include/iap/gnss/visibility_predictor.hpp` + `src/iap/gnss/visibility_predictor.cpp`: `VisibilityPredictor`; ENU direction from (el, az); per-sat ray_occluded + occupancy_ratio → κ → σ_eff; `VisibilityResult{n_vis, vis_flags, kappas, sigma_effs, mean_kappa}`.
  - `gnss_types.hpp`: `SatObs` += `kappa` field.
  - `predicted_integrity.hpp/.cpp`: `set_occupancy(grid*)` + `set_epoch(epoch*)` API; `sigma_grow_at(pos)` = sigma_grow × max(1, 1 + β_vis·visibility_deficit + γ_κ·κ); called per waypoint in `predict()`.
  - CMakeLists.txt: +visibility_predictor.cpp, +local_occupancy.cpp.
  - `colcon build` passes.

## 2026-03-05
- IAP-RQ-900: Auto-generate IEEE Trans methodology chapter.
  - `tools/gen_methodology.py`: reads `docs/TRACEABILITY.md` → writes `docs/methodology/methodology.tex` (IEEEtran class, TikZ flowchart, per-module subsections with formulas, traceability table) and `docs/figures/system_flow.tex` (standalone TikZ).
  - Generated .tex has 12 balanced `\begin`/`\end` environments; structure verified.
  - Formula skeletons for RQ-015/020/040/100/200/220/320/400 embedded.
  - Run: `python3 tools/gen_methodology.py`
- IAP-RQ-500/510: Experiment runner + metrics (Passive/CovMin/IntegAware baselines).
  - `include/iap/experiments/metrics.hpp`: MetricSample (stamp,PL,AL,IM,violation,path_increment,control_effort,mode); MetricsCollector (add/reset/log_summary/write_csv); ExperimentResult; write_comparison_table() → Markdown.
    * Metrics: Time(PL>AL)%, AvgPL, MinIM, path length, mission time, control effort, success.
  - `apps/iap_experiment.cpp`: iap_experiment node; runs three baselines (Passive/CovMin/IntegAware) against a synthetic degraded-zone scenario; writes per-baseline CSV + Markdown summary table to /tmp/.
  - CMakeLists.txt: +iap_experiment executable.
  - `colcon build` passes.
- IAP-RQ-400/410: Integrity-aware planner + receding horizon loop.
  - `include/iap/planner/integrity_planner.hpp`: IntegrityPlanner with Params (w_integrity, w_mission, w_smooth, search_weight_multiplier, dt_execute, al_default); `plan(pos0,vel0,yaw0,goal,sigma0,report)` → best CandidateTrajectory; `execution_target(chosen)` → first point ≥ dt_execute.
  - `src/iap/planner/integrity_planner.cpp`:
    * Generates candidates via TrajectoryGenerator (IAP-RQ-300).
    * Predicts PL_pred via PredictedIntegrityComputer (IAP-RQ-320).
    * Evaluates J_total = w_int * Σ hinge(PL_pred−AL)² + w_miss * dist + w_smooth * effort.
    * mode==SEARCH boosts w_int by search_weight_multiplier (default ×5).
    * `execution_target()` returns first waypoint at stamp ≥ dt_execute (receding horizon IAP-RQ-410).
    * trace-log: n_candidates, best_id, J_total/J_int/J_goal/J_eff, AL, sigma0.
  - `colcon build` passes [33.5s].
- IAP-RQ-300/310/320: Planner modules — trajectory generator + predicted integrity.
  - `include/iap/planner/trajectory_types.hpp`: TrajectoryPoint (stamp, pos, vel, yaw), CandidateTrajectory (id, points[], PL_pred[], sigma_pred[], J_total/integrity/goal/effort).
  - `include/iap/planner/trajectory_generator.hpp/.cpp`: motion primitives (speed×yaw_rate×alt_rate grid); default speeds={0.5,1.0,1.5} m/s; yaw_rates={−0.3,0,0.3} rad/s; alt_rates={−0.2,0,0.2} m/s; horizon=3 s, dt=0.2 s; `generate(state)` → vector of CandidateTrajectory.
  - `include/iap/planner/predicted_integrity.hpp/.cpp`: sigma growth model σ(t+dt)=sqrt(σ²+σ_grow²·dt); PL_pred = K_pl·σ_pred (RQ-320 baseline); `predict(traj, sigma0)` and `predict_all(trajs, sigma0)`.
  - RQ-310: visibility/observability placeholder implemented; actual ray-cast deferred to map integration phase.
  - `colcon build` passes [2.89s].
- IAP-RQ-200/210/220: Integrity monitoring module.
  - `include/iap/integrity/integrity_types.hpp`: IntegrityMode (NOMINAL/CAUTION/ALERT/SEARCH), IntegrityReport (PL, AL, IM, mode, lambda_max_sigma_p, sat_nis, excluded_sats, gamma_R, icp_degenerate, gamma_lidar, tdop, safe()).
  - `include/iap/integrity/integrity_monitor.hpp`: IntegrityMonitor with Params; set_obstacle_distance(); compute(frame, epoch, trunk).
  - `src/iap/integrity/integrity_monitor.cpp`:
    * PL = K_pl * sqrt(lambda_max(Σ_p)) via SelfAdjointEigenSolver (RQ-200 baseline)
    * AL = al_scale * obstacle_dist − uav_radius, clamped to al_min (RQ-210)
    * IM = AL − PL; safe() when IM > 0
    * GNSS NIS gating: per-sat NIS_k = r_k² / σ_k²; exclude if > χ²(1,0.01); global NIS greedy FDE; gamma_R = sqrt(max_nis/thresh) (RQ-220)
    * Mode state machine: NOMINAL→CAUTION→ALERT→SEARCH→NOMINAL; recovery_counter
    * trace-log: PL/AL/IM/mode/lambda_max/icp_degenerate/gamma_lidar/tdop; warn on ALERT
  - `colcon build` passes [4.82s].
- IAP-RQ-100/110/120: Trunk detection + TDOP metric + health factor.
  - `include/iap/trunk/trunk_types.hpp`: TrunkObservation (center_xy, radius, confidence, bearing_xy, p_fault), TrunkDetectionResult (trunks, tdop, tdop2, lambda_min_H).
  - `include/iap/trunk/trunk_detector.hpp`: TrunkDetector with Params; height+range filter; 8-connected grid BFS clustering; Kasa circle fit; TDOP = sqrt(trace(H⁻¹)) via SelfAdjointEigen; `health_factor()` scalar [0,1] (Baseline-A).
  - `src/iap/trunk/trunk_detector.cpp`: implementation.
  - Full-B (trunk as FGO factor) deferred to upgrade phase.
  - `colcon build` passes.
- IAP-RQ-040: ICP quality report + noise inflation in LiDAR odometry.
  - `EstimationFrame::IcpQuality`: inlier_count, inlier_fraction, rmse, cond_number, degeneracy_flag, gamma_lidar.
  - `OdometryEstimationCPUParams`: +`icp_cond_threshold` (500), +`gamma_lidar_max` (10.0).
  - `odometry_estimation_cpu.cpp`: re-linearize at optimal; `hessianBlockDiagonal()` + JacobiSVD → cond_number; dynamic_cast GICP/VGICP factors → inlier metrics; `gamma_lidar = sqrt(cond/thresh)` clamped to max; BetweenFactor/PriorFactor precision divided by gamma².
  - Trace-level log per frame: inliers, rmse, cond, degenerate, gamma.
- IAP-RQ-020: GNSS measurement model — pseudorange + Doppler factors.
  - New `include/iap/gnss/`: `gnss_types.hpp` (SatObs, GnssEpoch), `pseudorange_factor.hpp`, `doppler_factor.hpp`, `gnss_handler.hpp`.
  - New `src/iap/gnss/`: `pseudorange_factor.cpp` (PseudorangeFactor: NoiseModelFactor2<Pose3,Vector2>, analytical Jacobians), `doppler_factor.cpp` (DopplerFactor: NoiseModelFactor3<Pose3,Vector3,Vector2>), `gnss_handler.cpp` (epoch queue, elevation-dependent noise, get_factors()).
  - Each satellite is an independent observation channel (per-sat gating ready).
  - Ephemeris (sat_pos/sat_vel) pre-computed outside factors; minimal stub accepted.
  - `colcon build` passes.
- IAP-RQ-015: Expose Σ_p from smoother marginal covariance.
  - `EstimationFrame`: +`sigma_p` (Eigen::Matrix3d, zero-init)
  - `odometry_estimation_imu.cpp`: `smoother->marginalCovariance(X(i))`; extract `pose_cov.block<3,3>(3,3)`; compute `trace`, `lambda_max` via SelfAdjointEigenSolver; `trace`-level log `sigma_p trace/lambda_max/PL_proxy`.
  - Interface placeholder: downstream can read `frame->sigma_p`; replace block with exact `HΣH^T` when needed (RQ-320).
- IAP-RQ-010: Extend state with `clk_bias`/`clk_drift` [δt m, δṫ m/s].
  - `EstimationFrame`: +`clk_bias`, +`clk_drift` (double)
  - `OdometryEstimationIMUParams`: +`clk_bias_noise`(100m), +`clk_drift_noise`(1m/s)
  - `odometry_estimation_imu.cpp`: `C(i)=Vector2[δt,δṫ]` key; loose PriorFactor; clock prediction δt_next=δt+δṫ*Δt; `trace`-level log: `clk_bias / clk_drift`.
  - `colcon build` passes; fixed-lag smoother runs with clock states. → `.githooks`; pre-commit doc-guard verified. AGENTS.md + CHANGES/TRACEABILITY/REQS confirmed present.
- IAP-RQ-002: Add `apps/iap_status.cpp` (smoke-test demo), `launch/iap_demo.launch.py`, `.clangd` (CompilationDatabase path). CMakeLists.txt: `IAP_VERSION` define, install launch/. `colcon build` 产生 compile_commands.json; workspace root symlink.
  - `ros2 launch iap iap_demo.launch.py` 可用，输出 `iap_status: OK`
- IAP-RQ-001: Generate `docs/SPEC_VS_IMPL.md`：对照 spec/ 与代码现状，汇总已实现/待实现条目，建议新目录结构。
- IAP-RQ-001: Renamed ROS2 package from `glim` to `iap`.
  - `src/glim/` → `src/iap/` (C++ source directory)
  - `include/glim/` → `include/iap/` (public header directory)
  - `cmake/glim-config.cmake.in` → `cmake/iap-config.cmake.in`
  - CMakeLists.txt: `add_library(glim)` → `add_library(iap)`, all install targets/exports renamed.
  - All `glim_LIBRARIES` → `iap_LIBRARIES`; install paths `share/glim` → `share/iap`, `bin/glim` → `bin/iap`.
  - `colcon build --packages-select iap` passes; `ros2 pkg list | grep iap` shows `iap`.
- Init: add traceability & agent rules. (IAP-RQ-000)
