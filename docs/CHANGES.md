# Changes Log (IAP)

> 规则：任何代码改动必须在这里记录，并包含 IAP-RQ-XXX。

## Unreleased
- feat(dev-ct-mainline-lidar-analytic): IAP-RQ-300 / IAP-RQ-410 — continue `M2 / WP2` by pushing CT LiDAR rotation Jacobians and target policy beyond the first semi-analytic baseline.
  - `IntegratedBSplineGICPFactor` now evaluates the `SEMI_ANALYTIC` rotation block through an analytic quaternion-blend chain rule instead of caching per-time-node rotational finite differences, further reducing hot-path numeric differentiation while keeping `NUMERIC_FULL` as the fallback baseline.
  - The factor now supports an additional absolute ambiguity gate (`ct_lidar_correspondence_min_score_gap`) plus a robust-weight floor (`ct_lidar_robust_weight_floor`), and profiling stats now expose `rejected_robust_count`.
  - `OdometryEstimationBSpline` now tightens snapshot-target lifecycle with explicit minimum frame/point support and optional age gating before using `ACTIVE_WINDOW_SNAPSHOT`; undersupported snapshots fall back to the global ivox reference.
  - CT LiDAR trace logs now include score-gap / robust-weight-floor diagnostics together with snapshot support statistics (`snapshot_frames`, `snapshot_points`, `snapshot_span_s`, `snapshot_policy`).
  - `test_bspline_gicp_factor.cpp` now covers semi-analytic-vs-numeric predicted-error agreement, score-gap ambiguity rejection, and robust-weight-floor hard rejection.
- feat(dev-ct-mainline-lidar-correspondence): IAP-RQ-300 / IAP-RQ-410 — continue `M2 / WP2` by strengthening CT LiDAR correspondence selection and reducing hot-path numerical differencing.
  - `IntegratedBSplineGICPFactor` now supports k-NN target candidate search with Mahalanobis-score selection instead of hardwiring a single nearest Euclidean correspondence.
  - The factor now exposes an ambiguity-rejection ratio on the best/second-best Mahalanobis scores and reports `rejected_ambiguity_count` in profiling stats.
  - The `SEMI_ANALYTIC` Jacobian path now caches control-point rotation Jacobians per spline time node, so rotational finite differencing is no longer repeated per source point during the hot correspondence/linearization path.
  - `OdometryEstimationBSpline` now wires `ct_lidar_correspondence_candidates` and `ct_lidar_correspondence_accept_ratio` through config/logging, and CT LiDAR trace logs now include ambiguity-rejection diagnostics alongside target/inlier stats.
  - `test_bspline_gicp_factor.cpp` now covers Mahalanobis candidate selection beating nearest-Euclidean matching and ambiguity-ratio rejection of nearly equivalent correspondences.
- feat(dev-ct-mainline-lidar-jacobian): IAP-RQ-300 / IAP-RQ-410 — continue `M2 / WP2` by pushing the CT LiDAR factor from full numeric pose Jacobians to a semi-analytic engineering path.
  - `IntegratedBSplineGICPFactor` now supports explicit Jacobian modes (`NUMERIC_FULL` / `SEMI_ANALYTIC`); the new default keeps analytic control-translation blocks and controlled finite-difference control-rotation blocks, while preserving the old full-numeric mode as an A/B/debug fallback.
  - The factor now exposes explicit outlier/robust handling knobs: whitened-residual outlier gating plus `NONE / HUBER / CAUCHY` robust kernels with configurable widths.
  - LiDAR profiling now reports matched/inlier/rejected counts together with inlier ratio and mean robust weight, and `OdometryEstimationBSpline` logs those diagnostics and publishes the resulting inlier/RMSE proxy back to `EstimationFrame::icp_quality`.
  - `config_odometry_bspline.json` now includes `ct_lidar_jacobian_mode`, `ct_lidar_jacobian_numeric_eps`, `ct_lidar_outlier_mahalanobis_thresh`, `ct_lidar_robust_kernel`, and `ct_lidar_robust_kernel_width`.
  - `test_bspline_gicp_factor.cpp` now covers perturbed-state semi-analytic linearization consistency together with outlier-threshold and robust-kernel behavior on bad correspondences.
- feat(dev-ct-mainline-lidar): IAP-RQ-300 / IAP-RQ-410 — start `M2 / WP2` by engineering the continuous-time LiDAR factor target strategy and validation hooks.
  - `OdometryEstimationBSpline` now exposes explicit CT LiDAR target modes (`ACTIVE_WINDOW_SNAPSHOT` / `GLOBAL_IVOX_REFERENCE`) plus a snapshot frame-window parameter, so the lifecycle of frozen target snapshots is no longer implicit.
  - Active CT LiDAR segments now record target mode, contributing-frame count, target-point count, and target-build latency, and the current segment logs those diagnostics together with factor profiling output.
  - `IntegratedBSplineGICPFactor` now provides profiling stats and a linearization-check helper that compares Hessian-predicted error against nonlinear error under a controlled perturbation.
  - Target KD-trees are now built from `voxel_points()` exported as `PointCloud`, which avoids the unstable `KdTree2<iVox>` traits path.
  - Added `test_bspline_gicp_factor.cpp` to cover LiDAR factor error response, linearization-check validity, and profiling-stat reporting.
- feat(dev-ct-mainline-telemetry): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — add explicit fixed-lag diagnostics, lifecycle telemetry, and state-machine output to the unified registry.
  - `BSplineFixedLagStateRegistry` now publishes `BSplineFixedLagTelemetry`, which reports lag-window bounds, active segment range, auxiliary/shared-state counts, GNSS-anchor readiness, and lifecycle stages (`Empty / WindowSeeded / TrackingLidar / TrackingLidarGnss`).
  - `OdometryEstimationBSpline` now publishes that telemetry through `IapSharedState` alongside the continuous trajectory view and also emits the lifecycle state in trace logs during each continuous-time update.
  - `test_bspline_fixed_lag_registry.cpp` now covers telemetry/state-machine transitions and validates the corresponding count semantics across empty, seeded, LiDAR-tracking, and LiDAR+GNSS-tracking states.
- feat(dev-ct-mainline-shared-registry): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — extend the unified fixed-lag registry to own shared IMU/GNSS state snapshots.
  - `BSplineFixedLagStateRegistry` now owns persistent shared state snapshots for gyro bias, accel bias, gravity, ECEF origin, and ECEF rotation, and exposes seed/update helpers for graph construction and post-solve writeback.
  - `OdometryEstimationBSpline` no longer maintains separate private snapshots for those shared states; the active fixed-lag registry now provides shared-state ownership to graph seeding, GNSS-anchor activation, optimization priors, and continuous-time publication.
  - `test_bspline_fixed_lag_registry.cpp` now covers shared-state seed/update round-trip in addition to control-buffer/segment lifecycle behavior.
- feat(dev-ct-mainline-registry): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — introduce a unified fixed-lag lifecycle/state registry for the B-spline odometry mainline.
  - Added `BSplineFixedLagStateRegistry`, which owns the active control-buffer timeline together with the segment-level lifecycle used to retain/prune auxiliary velocity/clock states across the lag window.
  - `OdometryEstimationBSpline` now drives control-window append, segment append, lag pruning, marginalization-state export, and auxiliary-value filtering through that shared registry instead of maintaining separate `control_buffer_` / `active_segment_constraints_` / ad hoc aux pruning paths.
  - Added `test_bspline_fixed_lag_registry.cpp` to validate synchronized control-buffer pruning, surviving segment ownership, auxiliary-value filtering, and reset/append ordering semantics.
- feat(dev-ct-mainline-carried-linear): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — tighten carried-prior ownership and keep it in linear form between fixed-lag updates.
  - `BSplineMarginalizationPartition` now exposes explicit factor ownership classification (`survivor-only / removable / foreign`) and a replay-safety check for carried-prior keys, so state/factor ownership is no longer inferred ad hoc at each callsite.
  - `BSplineCarriedPrior` now stores the carried prior as a retained-key set plus linearization-point `GaussianFactorGraph`, and only converts it back into `LinearContainerFactor`s when replaying into the active optimization graph.
  - `OdometryEstimationBSpline` now excludes the replayed carried prior from the removable nonlinear subgraph used for the next marginalization step; instead, the previous linear carried prior is re-used explicitly when building the next survivor prior.
  - `test_bspline_marginalization.cpp` now covers foreign-key ownership detection and composition of a previous linear carried prior with new removable nonlinear factors.
- feat(dev-ct-mainline-carried-prior): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — stabilize fixed-lag survivor/removable partitioning and add dedicated carried-prior tests.
  - Added `BSplineMarginalizationPartition`, which centralizes survivor/removable state ownership for control points, segment auxiliary states, and persistent shared IMU/GNSS alignment states during lag pruning.
  - Added `build_bspline_carried_prior(...)`, which linearizes the removable nonlinear subgraph, marginalizes it onto the retained survivor keys, and converts the result back into replayable `LinearContainerFactor`s for the next solve.
  - `OdometryEstimationBSpline` now classifies LiDAR/IMU/velocity/GNSS/clock/smoothness factors against that shared partition, seeds GNSS clock states before partitioning, and only replays carried priors when every referenced key remains inside the current survivor set.
  - Added `test_bspline_marginalization.cpp` to validate survivor/removable partition membership and verify that the replayed carried prior matches the reference marginal error on perturbed survivor states.
- feat(dev-ct-mainline-marginal): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — push `SLAM_FINISH_PLAN` WP1 from boundary-subset replay toward removable-factor survivor marginalization.
  - `OdometryEstimationBSpline` now keeps the soon-to-be-removed lag-window segment factors in the current solve instead of pruning them before graph construction.
  - A dedicated removable graph is now assembled from the previous carried prior plus the LiDAR/IMU/velocity/GNSS/clock/smoothness factors that will disappear after lag pruning.
  - After each solve, that removable graph is linearized and marginalized onto the next-iteration survivor state set (retained control points, retained auxiliary velocity/clock states, and persistent shared IMU/GNSS alignment states), then converted back into replayable `LinearContainerFactor`s.
  - Lag pruning is now applied only after this survivor prior is extracted, which makes the fixed-lag carry-over closer to an actual Schur-style marginalization than the earlier boundary-only information replay.
- feat(dev-ct-mainline-boundary): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — start `SLAM_FINISH_PLAN` WP1 by turning fixed-lag boundary priors into a reusable boundary information prior.
  - `OdometryEstimationBSpline::ActiveSplineMarginalPrior` now snapshots not only the lag-window boundary poses but also the boundary auxiliary state index plus optional velocity / clock values.
  - After each solve, `OdometryEstimationBSpline` now attempts to extract a joint marginal information matrix over the boundary pose/velocity/clock subset and stores it together with the matching linearization point.
  - The next fixed-lag solve now prefers replaying that boundary information through a `LinearContainerFactor(HessianFactor, linearization_point)` and falls back to the earlier handcrafted pose/velocity/clock priors if marginal extraction fails.
  - `docs/dev_ct/dev_status.md` now points to `docs/dev_ct/SLAM_FINISH_PLAN.md` as the sole execution entry for finishing the continuous-time SLAM stack.
- feat(dev-ct-gnss-preproc): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — move GNSS epoch assembly, ephemeris lookup, and raw preprocessing closer to `OdometryEstimationBSpline`.
  - Added `GnssEpochBuilder`, a reusable GNSS front-end that converts raw observation batches plus ephemeris/iono/anchor state into processed ECEF `GnssEpoch` packets.
  - `gnss_extension` now acts primarily as ROS ingress plus legacy bridge: it publishes raw GNSS batches, ephemeris updates, and ionosphere parameters to `IapSharedState`, while using the shared builder locally for legacy diagnostics/injection.
  - `OdometryEstimationBSpline` now owns both `GnssEpochBuilder` and `GnssHandler`; it rebuilds processed epochs from raw shared-state mailboxes before segment-window GNSS factor construction.
  - Added `test_gnss_epoch_builder.cpp`, expanded `test_shared_state_gnss_queue.cpp` for raw GNSS front-end mailboxes, and rewrote `docs/methodology/methodology.tex` in IEEE-style to document the continuous-time SLAM pipeline and measurement models.
- feat(dev-ct-gnss-handler): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — move BSpline GNSS buffering ownership into an odometry-owned `GnssHandler`.
  - `OdometryEstimationBSpline` now constructs and owns its own `GnssHandler`, drains raw epochs from `IapSharedState`, and consumes segment-window epochs through handler-managed buffering instead of directly range-draining shared state.
  - `GnssHandler` now exposes `consume_epochs_in_range(...)`, which unifies legacy discrete frame-near draining and the new continuous-time segment-window draining path.
  - `IapSharedState` GNSS queue is now treated as a raw mailbox (`consume_pending_gnss_epochs()`), and tests were split so `test_gnss_handler_queue.cpp` covers range draining while `test_shared_state_gnss_queue.cpp` covers mailbox draining semantics.
- feat(dev-ct-gnss-window): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — align BSpline GNSS consumption with segment time windows instead of point-near-frame draining.
  - Continuous-time GNSS consumption was widened from single `frame_stamp` matching to `[scan_start, scan_end]` segment windows, which is a better fit for continuous-time factor construction.
  - That segment-window policy is now executed through the odometry-owned `GnssHandler`, while shared state remains only a raw mailbox.
  - Queue-behavior coverage now lives in `test_gnss_handler_queue.cpp` and `test_shared_state_gnss_queue.cpp`.
- feat(dev-ct-gnss): IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410 — start Phase 1C by injecting GNSS pseudorange/doppler directly into the BSpline control-window graph.
  - Added shared GNSS epoch queue + ECEF anchor publication to `IapSharedState`, and `GnssExtensionModule` now publishes the NavSat-derived ECEF anchor for continuous-time odometry consumption.
  - Added `IntegratedBSplinePseudorangeFactor` / `IntegratedBSplineDopplerFactor`, clock/ECEF key helpers, and wired `OdometryEstimationBSpline` to consume GNSS epochs per active segment, optimize per-segment clock states, and add GNSS factors/clock-between factors into the same fixed-lag LM graph as LiDAR/IMU/velocity.
  - Added `test_bspline_gnss_factor.cpp` and updated `config_odometry_bspline.json` comments to reflect the initial Phase-1C GNSS path.
- feat(dev-ct-planner-future): IAP-RQ-300 / IAP-RQ-400 / IAP-RQ-410 — let planner scoring consume future-time continuous-time trajectory samples.
  - `IntegrityPlanner::plan()` now resolves its planning origin from `SplineControlAccess` when available, so the current seed stamp tracks the active control window instead of blindly using the latest published sample.
  - Candidate evaluation now queries future-time `ContinuousTrajectoryView` samples at each waypoint, floors `sigma_pred` / `PL_pred` with the published spline uncertainty, and adds a short-horizon `w_ct_align` term from velocity / yaw mismatch to the published continuous-time trajectory.
  - Expanded `test_integrity_planner.cpp` to verify future sigma flooring and future trajectory velocity alignment influence candidate selection.
- feat(dev-ct-planner-seed): IAP-RQ-300 / IAP-RQ-410 — let the planner seed from the published continuous-time trajectory state.
  - `IntegrityPlanner::plan()` now resolves its seed state from `ContinuousTrajectoryView::latest_sample()` when available, overriding fallback `pos0 / vel0 / yaw0 / sigma0` with the published spline state.
  - Motion-primitive generation, smoothness cost, and turn cost now start from the velocity-aware continuous-time state instead of only using the trajectory view as a sigma source.
  - Added `test_integrity_planner.cpp` and wired it into `CMakeLists.txt` to verify both state seeding and trajectory-view-driven candidate selection.
- feat(dev-ct-trajectory-kinematics): IAP-RQ-300 / IAP-RQ-410 — publish explicit spline velocity states through control-access and trajectory sampling.
  - `BSplineControlWindow` / `BSplineControlWindowBuffer` now export control-point snapshots with optional velocity states when the fixed-lag graph has them.
  - `OdometryEstimationBSpline` now caches active velocity states for publication, seeds the initial spline snapshot with the initialization velocity, and exposes the active-window velocity states through `ContinuousTrajectoryView` / `SplineControlAccess`.
  - `BSplineTrajectory` now blends control-point `vel/acc` when kinematic data is present instead of always finite-differencing pose, and trajectory/control-window tests were expanded accordingly.
- feat(dev-ct-velocity): IAP-RQ-300 / IAP-RQ-410 — promote segment velocity into an explicit fixed-lag graph state.
  - Added `bspline_velocity_key(symbol('u', idx))` and `IntegratedBSplineVelocityFactor`, which ties each active segment's four pose control points to an explicit velocity state.
  - `OdometryEstimationBSpline` now seeds per-segment velocity variables, adds velocity consistency factors into the shared LM graph, and writes the optimized active velocity back into `EstimationFrame::v_world_imu`.
  - Expanded `config_odometry_bspline.json` with `velocity_ct_inf_scale`, added `test_bspline_velocity_factor.cpp`, and wired the new factor/test into `CMakeLists.txt`.
- feat(dev-ct-imu-state): IAP-RQ-300 / IAP-RQ-410 — promote shared IMU bias and gravity into explicit fixed-lag graph states.
  - `IntegratedBSplineIMUFactor` now optimizes four pose control points together with shared gyro bias, accel bias, and gravity variables instead of baking them in as constants.
  - `OdometryEstimationBSpline` now inserts shared bias/gravity values and priors into the active fixed-lag LM graph, then writes the optimized bias estimate back into `EstimationFrame`.
  - Expanded `config_odometry_bspline.json` with `imu_ct_bias_inf_scale` / `imu_ct_gravity_inf_scale`, and updated `test_bspline_imu_factor.cpp` to cover bias-state compensation and gravity-state mismatch.
- feat(dev-ct-imu-samples): IAP-RQ-300 / IAP-RQ-410 — replace the provisional IMU relative-pose hook with sample-based continuous-time gyro/acc factors.
  - `IntegratedBSplineIMUFactor` now directly evaluates per-sample body angular velocity and specific force from the four pose control points instead of only comparing one segment-level relative pose.
  - `OdometryEstimationBSpline` now freezes downsampled IMU samples for each active segment, carries per-segment fixed bias snapshots, and adds multiple IMU sample factors into the same shared fixed-lag LM graph.
  - Expanded `config_odometry_bspline.json` with `imu_ct_sample_stride`, and updated `test_bspline_imu_factor.cpp` to validate stationary matching samples and mismatched sample residuals.
- feat(dev-ct-imu): IAP-RQ-300 / IAP-RQ-410 — tighten fixed-lag target/prior handling and add a continuous-time IMU factor to the shared spline graph.
  - Added `IntegratedBSplineIMUFactor`, a minimal continuous-time IMU relative-pose factor over the four control points of each active segment.
  - `OdometryEstimationBSpline` now freezes a local target snapshot for each active segment, carries an explicit marginal prior for the lag-window boundary, and adds both LiDAR and IMU factors for every active segment into the same LM graph.
  - Added `test_bspline_imu_factor.cpp` and expanded `config_odometry_bspline.json` with marginal-prior / IMU factor weights.
- feat(dev-ct-multiseg): IAP-RQ-300 / IAP-RQ-410 — add per-segment continuous-time LiDAR factors across the active spline window.
  - `OdometryEstimationBSpline` now stores active segment constraints (`raw source cloud + 4 control-point indices`) inside the lag window.
  - Each active segment now contributes its own `IntegratedBSplineGICPFactor` to the same LM graph, so overlapping control points are jointly optimized across multiple segments instead of only the newest one.
  - Active segment constraints are pruned together with the lag window.
- feat(dev-ct-joint-window): IAP-RQ-300 / IAP-RQ-410 — move the active spline window from publication-only state into the optimization variable set.
  - `BSplineControlWindowBuffer` now exposes `keys()`, `values()`, and `update_from_values()` so the active lag-window control points can directly participate in optimization.
  - `OdometryEstimationBSpline` now seeds LM from the whole active control window, adds smoothness factors across all active control points, and anchors/predicts the boundary control points while the current LiDAR factor still constrains the latest segment.
  - Expanded `test_bspline_control_window.cpp` with buffer value round-trip coverage.
- feat(dev-ct-window): IAP-RQ-300 / IAP-RQ-410 — extend the Phase-1B frontend with an active fixed-lag spline window skeleton.
  - Added `BSplineControlWindowBuffer` to retain the active multi-segment control-point sequence instead of publishing only the latest 4-point segment.
  - `OdometryEstimationBSpline` now resets/appends/prunes the active spline control window together with frame history marginalization, so planner/viewer/control-access consume the lag-window trajectory rather than a single segment.
  - Expanded `test_bspline_control_window.cpp` with buffer growth and lag-pruning coverage.
- feat(dev-ct-phase1b): IAP-RQ-300 / IAP-RQ-410 — start the minimal viable continuous-time odometry frontend.
  - Added `BSplineControlWindow` and explicit control-point key design (`symbol('s', idx)`) for the active 4-knot spline window.
  - Added `IntegratedBSplineGICPFactor`, a CPU continuous-time LiDAR factor over four pose control points with per-point time query and GICP residual accumulation.
  - `OdometryEstimationBSpline` now has a new `frontend_mode = CT_LIDAR_CPU` path that uses the control-point window + local LM optimization instead of only reconstructing a spline after discrete odometry.
  - Added `test_bspline_control_window.cpp` and expanded `config_odometry_bspline.json` with Phase-1B factor / prior settings.
- feat(dev-ct-foundation): IAP-RQ-300 / IAP-RQ-410 — add the first continuous-time B-spline foundation for odometry and planner integration.
  - Added planner-facing continuous trajectory interfaces: `ContinuousTrajectoryView`, `SplineControlAccess`, `TrajectorySample`, `SplineWindowSnapshot`.
  - Added `BSplineTrajectory` with uniform/non-uniform cubic knot policies, time query, range sampling, knot/control-point snapshot export.
  - Added `OdometryEstimationBSpline` and `config_odometry_bspline.json`. The new module currently reuses the existing LiDAR-IMU odometry backend, then publishes a continuous-time B-spline window via `IapSharedState` and `EstimationFrame::custom_data` while preserving legacy sampled outputs.
  - `PlannerInterface` now has default no-op `set_trajectory_view()` / `set_control_access()` hooks; `IntegrityPlanner` can consume the published trajectory view without breaking the existing `plan(...)` signature.
  - Added `test_bspline_trajectory.cpp` unit tests and updated `docs/dev_ct/PLANS.md` with current implementation status.
- fix(clock-ownership): IAP-RQ-010 / IAP-RQ-200 — converge single-owner clock contract and suppress GNSS-owner read noise.
  - Added `clock_owner_mode` (`dual|odometry|gnss`) wiring across odometry/GNSS/trunk; defaults switched to `gnss` in odometry configs.
  - Added cross-module clock readiness marker in `IapSharedState` (`set_clock_ready/clear_clock_ready/is_clock_ready`).
  - GNSS extension now sets readiness when `C(frame_id)` is prepared and clears it on clock-chain reset/recovery.
  - Odometry in GNSS-owner mode now reads only `C(current)` and only when ready; non-ready frames are treated as warmup (no `c missing` warning storm).
  - Added lifecycle/ownership observability hardening (`KeyLifecycleMonitor`) and per-symbol relinearization policy registry with startup validation (including `l:Point2` threshold dimension).
  - Acceptance A/B replay (`dual` vs `gnss`) now shows `c missing=0`, `conflicts=0`, `violations=0`, and no hard optimizer errors.
- fix(clk-relin): IAP-RQ-010 — per-type iSAM2 relinearization threshold to eliminate sync-mode GPU linearization flood.
  - Root cause: `clk_bias` jumps ~`drift×dt`=61×0.1=6m/frame at cold-start, always exceeding global threshold 0.1, forcing iSAM2 to re-linearize clock state every frame. Each re-linearization calls `IntegratedVGICPFactorGPU::linearize()` without pre-computed result → sync-mode GPU stall → warning flood.
  - `odometry_estimation_imu.cpp`: replace single `setRelinearizeThreshold(double)` with `FastMap<char,Vector>` per key type: `x/v/b/e/r` keep 0.1; `c` (clock) uses `[clk_bias_relin_thresh, clk_drift_relin_thresh]`.
  - `odometry_estimation_imu.hpp`: add `clk_bias_relin_thresh` / `clk_drift_relin_thresh` fields to Params.
  - `config_odometry_gpu.json`: add `"clk_bias_relin_thresh": 500.0` / `"clk_drift_relin_thresh": 5.0`.
- fix(warnings): IAP-RQ-003 / IAP-RQ-010 / IAP-RQ-200 — silence 6 startup/runtime warning categories.
  - **W1** `config_ros.json`: add missing `dump_path` key.
  - **W2** `config_odometry_gpu.json`: add `clk_bias_noise=100` / `clk_drift_noise=1` (IAP-RQ-010 params missing from config); bump `isam2_relinearize_skip` 1→5 to batch relinearization and eliminate sync-mode flood during GNSS injection.
  - **W3** `config_ros.json`: complete `imu_qos` / `points_qos` / `image_qos` with `history`, `reliability`, `durability` fields.
  - **W4** `rviz_viewer.cpp`: skip `imu→lidar` TF when `imu_frame_id == lidar_frame_id` (Livox shares one frame — was spamming `TF_SELF_TRANSFORM`).
  - **W6** `integrity_monitor.cpp`: distinguish `UNSAFE`-due-to-`PL>=AL` (warn) from `UNSAFE`-in-recovery-phase (info) — message was misleadingly printing `"PL=1.719 >= AL=10.000"` when 1.719 < 10.000.
- feat(standalone): IAP-RQ-003 — self-contained ROS2 operation; no runtime dependency on `glim_ros` package.
  - **Root cause**: `config_ros.json` listed `librviz_viewer.so` and `libstandard_viewer.so` which only existed in GLIM's install. `iap_rosnode` silently skipped them via `continue`, leaving RViz blank.
  - **`librviz_viewer.so`**: ported from `glim_ros2/src/glim_ros/rviz_viewer.cpp` into `src/iap/util/rviz_viewer.{hpp,cpp}`. Publishes `~/aligned_points`, `~/odom`, `~/pose`, `~/map`; broadcasts TF `map→odom→base_frame` and `imu→lidar`.
  - **`libstandard_viewer.so`**: ported from `glim/src/glim/viewer/standard_viewer*.cpp` (4 files) into `src/iap/util/standard_viewer*.{hpp,cpp}`. Iridescence 3D desktop viewer unchanged.
  - **CMakeLists.txt**: added `find_package` for `tf2_ros`, `nav_msgs`, `geometry_msgs`; added `rviz_viewer` and `standard_viewer` shared library targets.
  - **`config_ros.json`**: removed `libmemory_monitor.so` (GLIM-only, no IAP port needed).
  - **CLAUDE.md**: updated primary run command to `ros2 run iap iap_rosnode`; legacy GLIM mode documented as secondary.
- refactor(config): IAP-RQ-025 / IAP-RQ-000 — consolidate config directory from 18 → 15 files.
  - **Merge 1 (logging)**: `config_logging.json` deleted; its `"logging"` section inlined into `config.json`.
    `util/logging.cpp`: use `GlobalConfig::instance()` directly instead of loading a separate file.
  - **Merge 2 (sensor + preprocess)**: `config_preprocess.json` deleted; its `"preprocess"` section
    appended to `config_sensors.json`. `config.json` global manifest: removed `config_preprocess` key.
    `preprocess/cloud_preprocessor.cpp`: now loads only `config_sensors` (one `Config` object) and
    reads both `"sensors"` and `"preprocess"` sections from it.
  - **Merge 3 (GNSS + integrity)**: `config_integrity.json` deleted; its `"integrity"` section appended
    to `config_gnss.json`. `config.json` global manifest: removed `config_integrity` key (the existing
    `config_gnss` key covers both). `integrity/integrity_extension.cpp:42`: changed
    `get_config_path("config_integrity")` → `get_config_path("config_gnss")`.
    `integrity/integrity_extension.hpp`: updated comments to reference `config_gnss.json`.
  - No algorithmic or parameter changes — pure file consolidation.
- feat(observability): IAP-RQ-200 / IAP-RQ-040 / IAP-RQ-002 — validation output & visualization suite.
  - **IAP-RQ-200 (ARAIM CSV)**: `config_integrity.json` new flags `enable_araim_csv`/`araim_csv_path`.
    `araim_debug.hpp`: added config constructor `AraimDebugCSV(bool, path)` and `write(report, AraimResult&)` overload that emits `row_type=epoch` + optional `row_type=worst_hyp` row with full per-hypothesis 3-term data.
    `integrity_types.hpp`: added `K_fa_used` field to `IntegrityReport`.
    `integrity_monitor.cpp`: `run_araim()` stores `last_araim_result_` and forwards `K_fa_used`.
    `integrity_monitor.hpp`: `last_araim_result_` member + getter.
    `integrity_extension.cpp`: reads config flags, instantiates `AraimDebugCSV`, calls `write()` each smoother update.
    Bug fix: `integrity_extension.cpp:220` — `msg.k_fa_used = 0.0` → `= report.K_fa_used`.
  - **IAP-RQ-040 (ICP CSV)**: `config_odometry_gpu.json` new flags `enable_icp_csv`/`icp_csv_path`.
    `odometry_estimation_gpu.hpp/.cpp` and `cpu.hpp/.cpp`: params read config; `update_frames()` appends per-frame row `stamp,frame_id,rmse,inlier_fraction,condition_number,gamma_lidar,drop_flag`.
  - **IAP-RQ-002 (timing)**: `std::chrono` instrumentation in `gnss_extension.cpp` (`on_smoother_update_finish_`), `integrity_monitor.cpp` (`compute()`), `araim.cpp` (`run()`), `trunk_detector.cpp` (`detect()`). Each writes `stamp,module,elapsed_ms` to `/tmp/iap_timing.csv`.
  - **Trajectory CSV**: `config_integrity.json` new flags `enable_traj_csv`/`traj_csv_path`. `integrity_extension.cpp`: appends `stamp,x,y,z` after each smoother update for trajectory comparison.
  - **Config GNSS CSV enabled**: `config_gnss.json` `enable_debug_csv` set to `true`.
  - **Python plotting**: `tools/plot_araim_timeline.py` (Fig B1/B2/B3), `tools/plot_icp_timing.py` (Fig C1/C2), `tools/plot_trajectory_comparison.py` (Fig D1).
- fix(odometry): IAP-RQ-130 — fix EstimationFrame ABI layout mismatch vs libglim.so.
  - IAP additions (`clk_bias`, `clk_drift`, `sigma_p`, `icp_quality`) were inserted before original GLIM fields, shifting `raw_frame` offset by ~136 bytes.
  - Moved all IAP-added fields to **after** `custom_data` (struct tail), preserving original GLIM field offsets.
  - SIGSEGV in `TrunkExtensionModule::on_new_frame_` resolved.
- feat(trunk): IAP-RQ-130 — activate trunk FGO extension module with ROS2 visualization.
  - `trunk_extension.hpp/cpp`: base class → `ExtensionModuleROS2`; added `create_subscriptions()`, `publish_markers_()`.
  - `publish_markers_()`: detections as yellow-green cylinders (1 s TTL, ns=`det`), landmarks as bright-green cylinders + white text IDs (ns=`lm`/`lm_label`), DELETEALL on each update.
  - `map_mutex_` guards all `map_.update()` / `map_.landmarks()` / `map_.confirmed_landmarks()` accesses.
  - `CMakeLists.txt`: `trunk_extension.cpp` moved out of `libiap` → separate `trunk_extension` shared library with `rclcpp` + `visualization_msgs` deps.
  - `config_ros.json`: `libtrunk_extension.so` added to `extension_modules`.
  - Entry point: `extern "C" create_extension_module()` → `TrunkExtensionModule`.
  - `on_new_frame_`: uses `frame->raw_frame->points` (CPU, always valid) instead of `*frame->frame` (may be GPU null).
- feat(gnss): IAP-RQ-025 — externalize GNSS parameters to `config_gnss.json`.
  - New `config/config_gnss.json` with 16 parameters (pr noise, canopy model, elevation cut, lever arm, clock Q, ECEF priors, debug CSV).
  - `gnss_extension.cpp`: load all params via `glim::Config`; removed `IAP_GNSS_DEBUG_CSV` env-var fallback (config is sole source of truth).
  - `gnss_handler_` changed to `std::unique_ptr<GnssHandler>` (fixes mutex move issue).
- feat(mapping): IAP-RQ-045 — add `multiscan_window` parameter to global mapping.
  - `GlobalMappingParams::multiscan_window` (default 3): keep last N frames for point-to-multiscan matching.
  - Config key `global_mapping/multiscan_window` in `config_global_mapping_cpu.json` and `config_global_mapping_gpu.json`.
- feat(gnss): IAP-RQ-020 — full ECEF pipeline with E(0)/R(0) free variables.
  - Replace local-ENU coordinate frame with ECEF throughout GNSS pipeline.
  - `PseudorangeFactor`: `NoiseModelFactor4<Pose3, Vector2, Vector3, Rot3>` — keys X(i), C(i), E(0), R(0).
    Corrections: Klobuchar iono, Hopfield trop, Sagnac, TGD. Analytical Jacobians for all 4 keys.
  - `DopplerFactor`: `NoiseModelFactor4<Pose3, Vector3, Vector2, Rot3>` — keys X(i), V(i), C(i), R(0).
    Sagnac velocity correction included.
  - `GnssExtension`: inserts `E(0)` + `R(0)` with loose priors (σ_E=5 m, σ_R≈5°) on first GNSS
    injection; stamps both on every injection to keep them alive in fixed-lag smoother.
    Subscribes to `/ublox_driver/iono_params` (Klobuchar GPS coefficients).
  - `GnssHandler::get_factors()` now takes `anc_ecef` parameter for Doppler Sagnac.
  - `gnss_types`: `SatObs` gains `tgd`, `svddt`; `GnssEpoch` gains `gps_sec`, `iono_params`.
  - `CMakeLists`: `ament_target_dependencies(iap gnss_comm)` so factor .cpp can include gnss_utility.
  - svdt sign fix: `sat.pr_meas = pr + svdt * c` (ADD per RTKLIB/LIGO; previous commit used subtract).
  - svddt correction applied to Doppler: `sat.dop_meas = dop_raw + svddt * c`.
  - Elevation filter: skip satellites below 5° elevation.
  - `ecef_to_local()` removed (no longer needed; factors work in ECEF directly).
- fix(gnss): IAP-RQ-020 — apply svdt satellite clock correction to pseudorange.
  - Root cause: `sat.pr_meas` was storing raw pseudorange `pr` without subtracting the
    satellite clock error `svdt * CLIGHT`. Each GPS satellite has a unique clock offset
    of ±1 µs ≈ ±300 m. Without correction, the shared receiver clock state `C(i)` received
    conflicting information from each satellite, preventing convergence (PR rms = 237 km).
  - Fix: `sat.pr_meas = pr - svdt * CLIGHT` in `on_range_meas_()`.
    `eph2pos`/`geph2pos` already compute `svdt`; it is now applied.
  - Expected outcome: `PR rms` drops from ~237 km to O(<20 m) after first convergence;
    `clk_bias` converges to true receiver clock offset (~300–400 km).
- fix(gnss): IAP-RQ-020 — clock warm-start to eliminate PR rms ~237 km divergence.
  - Root cause: `C(frame_id)` was always inserted as `[0,0]` (cold-start). iSAM2
    cannot converge receiver clock from 0 → ~350 km in one real-time linearization
    step, leaving large per-satellite pseudorange residuals that never collapsed.
  - Fix: `on_smoother_update_finish_` stores post-opt `clk_bias / clk_drift /
    frame_stamp` into atomics; next `on_smoother_update_` propagates them forward
    with clock-walk model (`bias_next = bias + drift × dt`, `drift_next = drift`).
  - Warm-start `call_once` log now includes initial bias/drift for observability.
  - Expected outcome: `PR rms` drops from ~237 km → O(<10 m) after first convergence;
    `clk_bias` rapidly tracks true receiver clock offset.
- fix(gnss): IAP-RQ-020 — inject `C(frame_id)` into `new_values`/`new_stamps` explicitly.
  - Root cause: glim's `OdometryEstimationIMU::update_smoother()` (no clock variable)
    takes precedence over IAP's override at runtime due to shared-library symbol resolution.
    `C(frame_id)` was never added to the smoother → iSAM2 fallback discarded all GNSS factors.
  - Fix: in `on_smoother_update_`, if `!new_values.exists(C(frame_id))`, insert
    `Vector2(0,0)` initial estimate + `frame_stamp` in `new_stamps`.  Also always writes
    `new_stamps[C(frame_id)] = frame_stamp` even when already present (odometry path).
  - One-time `call_once` log confirms which path was taken ('not in new_values').
- fix(gnss): IAP-RQ-020 — `epoch.stamp` now derived from GPS observation time (UTC)
  instead of `node_->get_clock()->now()` (wall clock).
  - Root cause: wall clock during bag replay differs from bag recording time by months,
    causing `GnssHandler::get_factors()` to drain all epochs as "too old" (delta >> `time_tolerance`).
  - Fix: `gnss_comm::gpst2utc(obs_list[0]->time)` → UTC Unix time → `epoch.stamp`;
    aligns with LiDAR `frame_stamp` within ±0.1 s.
  - Adds a `std::call_once` diagnostic log on the first epoch: prints
    `epoch UTC stamp`, `last_frame_stamp`, and `delta` for alignment verification.
- feat(gnss): post-optimization diagnostic via `on_smoother_update_finish`.
  - Registers `on_smoother_update_finish` callback; fires after iSAM2 optimization.
  - Stores last-injected PR/Doppler factors (by key-count heuristic: 2=PR, 3=Dop).
  - Queries `C(frame_id)` from smoother → `clk_bias [m]`, `clk_drift [m/s]`.
  - Evaluates `NoiseModelFactor::unwhitenedError()` per factor → PR RMS [m], Dop RMS [m/s].
  - Logs at `info` level (first call + every 50): `clk_bias / clk_drift / PR_rms / Dop_rms`.
  - Adds `factor_count_diag_` counter and `last_pr_factors_`, `last_dop_factors_` storage.
- IAP-RQ-020 (bridge): `GnssExtensionModule` — full ROS2 GNSS data pipeline.
  - `include/iap/gnss/gnss_extension.hpp` + `src/iap/gnss/gnss_extension.cpp`:
    `GnssExtensionModule : ExtensionModuleROS2`; subscribes `/ublox_driver/range_meas`,
    `/ublox_driver/ephem`, `/ublox_driver/glo_ephem`, `/ublox_driver/receiver_lla`.
  - NavSatFix → WGS-84 geodetic → ECEF origin + ENU rotation matrix; thread-safe origin.
  - `GnssMeasMsg` → L1 obs index, sat-clock-corrected pseudorange, Doppler m/s
    (`dop = -dopp_hz × c/f`), sat ECEF pos/vel from `eph2pos/geph2pos`/vel,
    transformed to local ENU; per-satellite elevation from ENU unit vector.
  - `OdometryEstimationCallbacks::on_new_frame` → track `last_frame_id/stamp`.
  - `on_smoother_update` → `GnssHandler::get_factors()` → inject into `new_factors`.
  - `create_extension_module()` C entry-point for GLIM dlopen.
  - CMakeLists.txt: `gnss_extension` SHARED lib; `find_package(glog, gnss_comm, sensor_msgs)`.
  - `config/config_ros.json`: `libgnss_extension.so` added to `extension_modules`.
  - `colcon build` passes; `libgnss_extension.so` exports all symbols.

## 2026-03-05 (Phase-4)
- IAP-RQ-331/421/422: Predicted ARAIM PL in planner + per-waypoint AL.
  - `include/iap/planner/predicted_araim.hpp` + `src/iap/planner/predicted_araim.cpp`: `PredictedAraimComputer`; calls `VisibilityPredictor::predict()` at each waypoint → builds `SatGeometry` list → `Araim::predict_geometry()` (r=0) → returns `pl_araim` (geometry-only upper bound).
  - `include/iap/planner/trajectory_types.hpp`: `CandidateTrajectory` += `AL_pred` (per-waypoint Alert Limit vector).
  - `include/iap/planner/integrity_planner.hpp` + `.cpp`: `Params::use_araim_pl`, `Params::araim_pred_params`; `set_occupancy()`, `set_epoch()`, `set_al_fn()` setters; `plan()` fills `AL_pred` via `al_fn_` callback and replaces `PL_pred[k]` with `araim_predictor_.predict_araim_pl(wpt)` when `use_araim_pl`; `evaluate()` uses `traj.AL_pred[k]` per step.
  - CMakeLists.txt: +predicted_araim.cpp.
  - `colcon build` passes.

## 2026-03-05 (Phase-3)
- IAP-RQ-241/242/243/244/245/246: Full ARAIM engine (single-fault, horizontal PL).
  - `include/iap/integrity/araim_types.hpp`: `FaultHypothesis{type, row, sat_id, p_fault}`, `SubsetSolution{d_horiz, sigma_ss_E/N/horiz, threshold, pl_faulted, fault_detected}`, `AraimResult{valid, pl_ff, pl_araim, S0, hypotheses, subsets, n_det, detected_rows}`.
  - `include/iap/integrity/araim.hpp`: `Araim` class; `Params{K_fa=4.5, K_md=5.5, K_ff=5.33}`; `run(epoch, n_trunk)` (real residuals + FDE); `predict_geometry(visible_sats)` (r=0, planning mode).
  - `src/iap/integrity/araim.cpp`: `build_G/W/r(epoch)`; `enumerate_hypotheses()`; `compute_core()` — S0=(G^T W G)^{-1}; subset solutions exclude row k; σ_ss_horiz, threshold, pl_faulted per hypothesis; pl_ff=K_ff·√(S0[0,0]+S0[1,1]); pl_araim=max(pl_ff, max_k pl_faulted_k); FDE: detected_rows.
  - `include/iap/gnss/gnss_types.hpp`: `SatObs` += `pr_residual` field (meas−pred [m]).
  - `include/iap/integrity/integrity_types.hpp`: `IntegrityReport` += `pl_araim`, `pl_ff`, `araim_valid`, `araim_n_hyp`, `araim_n_det`, `araim_detected_rows`.
  - `include/iap/integrity/integrity_monitor.hpp` + `.cpp`: `Params::araim_params`; private `Araim araim_`; `run_araim()` — calls `araim_.run()`, merges result into report (replaces `report.PL` with `pl_araim` when valid); trace log extended with `pl_araim/araim_n_hyp/araim_n_det`.
  - CMakeLists.txt: +araim.cpp.
  - `colcon build` passes.

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
