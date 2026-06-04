# Stage 0: LiDAR ARAIM PL Stability Repair

## Summary

Stage 0 repairs only the current LiDAR ARAIM monitor path. It preserves certified GNSS ARAIM, current certified monitor fusion (`PL_mon_q = max(PL_G_q, PL_L_q)`), planner behavior, PI cost, launch topics, and ROS message topics.

The implementation scope is limited to LiDAR ARAIM risk stabilization, an ARAIM-only target window, component diagnostics, config defaults, and focused tests.

## Relevant Code Path

- `include/iap/integrity/lidar_araim.hpp`
  - `LidarAraimBlock`, `LidarAraimSnapshot`, `LidarHypothesis`, `LidarSubsetSolution`, `LidarAraimResult`, `LidarAraim::Params`
- `src/iap/integrity/lidar_araim.cpp`
  - `LidarAraim::compute_risk_components()`
  - `LidarAraim::enumerate_hypotheses()`
  - `LidarAraim::run()`
- `src/iap/odometry/odometry_estimation_cpu.cpp`
  - CPU VGICP block metadata creation and `"lidar_araim_snapshot"` attachment
- `src/iap/odometry/odometry_estimation_gpu.cpp`
  - GPU VGICP block metadata creation and `"lidar_araim_snapshot"` attachment
- `src/iap/integrity/integrity_extension.cpp`
  - retrieves `"lidar_araim_snapshot"`, loads config, writes Stage 0 CSV
- `src/iap/integrity/integrity_monitor.cpp`
  - runs LiDAR ARAIM and preserves max-based certified monitor fusion

## Implementation Details

- Replace the unbounded LiDAR age term with bounded `gamma_age`.
  - Default: `gamma_age = min(1 - exp(-age_sec / age_tau_s), gamma_age_max)`
  - Optional config mode: `linear_capped`
- Split and store risk components:
  - `gamma_rmse`
  - `gamma_inlier`
  - `gamma_condition`
  - `gamma_age`
  - `gamma_total`
- Enforce an ARAIM-only target-keyframe window:
  - group blocks by `target_frame_id`
  - keep at most `target_window_K = 10` target IDs
  - prioritize finite spatial proximity, then temporal freshness, then newer target ID
  - preserve all voxel-level blocks for selected targets
- Replace silent `cwiseMax(0.0)` solution-separation covariance handling with explicit fallback:
  - use `Sigma_f(q,q) - Sigma0(q,q)` only when finite and above `sigma_ss_min_m^2`
  - otherwise fallback to `Sigma_f(q,q)` with `sigma_ss_min_m` floor
  - log raw variance, used sigma, and fallback flag
- Add Stage 0 CSV logging:
  - `export/iap_lidar_araim_stage0.csv`
  - rows for E/N/U worst-axis subset components

## Config Defaults

Add under `"integrity"`:

```json
"lidar_araim_target_window_K": 10,
"lidar_araim_age_model": "exp_saturating",
"lidar_araim_age_tau_s": 30.0,
"lidar_araim_gamma_age_max": 1.0,
"lidar_araim_gamma_rmse_max": 5.0,
"lidar_araim_condition_ref": 10000.0,
"lidar_araim_gamma_condition_max": 5.0,
"lidar_araim_sigma_ss_min_m": 0.02,
"enable_lidar_araim_stage0_csv": true,
"lidar_araim_stage0_csv_path": "/tmp/iap_lidar_araim_stage0.csv"
```

## Test Plan

Focused gtests in `test/test_araim.cpp`:

- `LidarAraimTest.AgeRiskSaturates`
- `LidarAraimTest.TargetWindowCapsHypotheses`
- `LidarAraimTest.SigmaSsFallbackForNegativeRawVariance`

Commands:

```bash
cd /home/dev/ws_iap
colcon build --base-paths src/iap src/gnss_comm --packages-select iap --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim"
colcon test-result --verbose
```

Launch sanity:

```bash
source install/setup.bash
ros2 launch iap demo9_ego_planner_closed_loop.launch.py \
  start_rviz:=false \
  run_duration_s:=60 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_gnss:=true \
  use_araim:=true
```

## Acceptance Checks

- `selected_target_count <= target_window_size` for every Stage 0 CSV row.
- `gamma_age <= gamma_age_max` for every Stage 0 CSV row.
- No monotonic unbounded age-driven growth in `bias_term_m`.
- LiDAR PL remains composed as `|d_f| + K_fa*sigma_ss + K_md*sigma_f + b_f`.
- Certified monitor fusion remains max-based.
- No planner, PI cost, GNSS ARAIM, or topic-name changes.
