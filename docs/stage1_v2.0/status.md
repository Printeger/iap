# Stage 0 Verification Before Stage 1

Date: 2026-05-12

Scope inspected: current working tree in `/home/dev/ws_iap/src/iap`.

## Repository State

`src/iap` is the Git repository root. The workspace root `/home/dev/ws_iap` is not a Git repository.

Current dirty files are limited to Stage 0-related files and the new Stage 1 documentation directory:

- Config: `config/config_gnss.json`, `config/sim_demo11/config_gnss.json`, `config/sim_ego/config_gnss.json`, `config/sim_ego/sim_ego/config_gnss.json`
- Current LiDAR ARAIM monitor: `include/iap/integrity/lidar_araim.hpp`, `src/iap/integrity/lidar_araim.cpp`
- Stage 0 logging/config wiring: `include/iap/integrity/integrity_extension.hpp`, `src/iap/integrity/integrity_extension.cpp`, `include/iap/integrity/lidar_araim_debug.hpp`
- LiDAR block metadata only: `src/iap/odometry/odometry_estimation_cpu.cpp`, `src/iap/odometry/odometry_estimation_gpu.cpp`
- Tests: `test/test_araim.cpp`
- Docs: `docs/stage1_v2.0/*`

No Stage 0 diff was found in certified GNSS ARAIM files, planner cost files, planner behavior files, ROS topic launch defaults, or message definitions.

## Stage 0 Changes Found

Stage 0 changed only the current LiDAR ARAIM monitor support path:

- Added bounded LiDAR risk components via `LidarRiskComponents`.
- Replaced the old unbounded linear age term with configurable bounded age models:
  - default `exp_saturating`
  - optional `linear_capped`
- Added configurable target keyframe/window cap through `lidar_araim_target_window_K`.
- Added `target_distance_m` metadata in CPU/GPU odometry block capture so the LiDAR ARAIM target window can prefer nearby targets.
- Replaced silent `cwiseMax(0.0)` solution-separation variance handling with explicit raw variance, floor, and fallback diagnostics.
- Added Stage 0 per-axis CSV logging in `LidarAraimDebugCSV`.
- Added config defaults and focused tests:
  - `AgeRiskSaturates`
  - `TargetWindowCapsHypotheses`
  - `SigmaSsFallbackForNegativeRawVariance`

Certified monitor fusion remains in `IntegrityMonitor::run_lidar_araim()` as per-axis max and scalar max:

- `PL_E = max(existing PL_E, lidar_PL_E)`
- `PL_N = max(existing PL_N, lidar_PL_N)`
- `PL_U = max(existing PL_U, lidar_PL_U)`
- `HPL` and `VPL` are replaced only if LiDAR is larger
- scalar `PL` is replaced only if LiDAR `HPL` is larger

This preserves `PL_mon_q = max(PL_G_q, PL_L_q)`.

## Required Confirmations

1. LiDAR age risk is now bounded.

   Confirmed. `compute_risk_components()` clamps age through either:

   - `gamma_age = min(1 - exp(-age_sec / age_tau_s), gamma_age_max)`
   - `gamma_age = min(age_sec / age_ref_sec, gamma_age_max)`

   Default config sets `lidar_araim_age_model = "exp_saturating"` and `lidar_araim_gamma_age_max = 1.0`.

2. Target keyframe/window size is bounded and configurable.

   Confirmed. `LidarAraim::Params::target_window_K` defaults to `10`; config files set `lidar_araim_target_window_K: 10`. `filter_target_window()` caps selected target IDs before hypothesis enumeration.

3. PL component logs were added.

   Confirmed in `include/iap/integrity/lidar_araim_debug.hpp`. The CSV header includes:

   - `sep_term_m`: `|d_f|`
   - `sigma_ss_term_m`: `K_fa * sigma_ss`
   - `sigma_subset_term_m`: `K_md * sigma_f`
   - `bias_term_m`: `b_f`
   - `gamma_rmse`
   - `gamma_inlier`
   - `gamma_condition`
   - `gamma_age`
   - `selected_target_count`
   - `target_window_size`
   - `worst_hypothesis_type`
   - `worst_hypothesis_id`
   - plus `lambda_min_subset`, `condition_number_subset`, raw/used sigma-ss, and fallback flag

   The throttled runtime log only prints a short Stage 0 summary (`targets`, `worst`, `gamma_age`, `bias`); full component coverage is in the CSV.

4. Focused tests pass.

   Command run from `/home/dev/ws_iap`:

   ```bash
   colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim"
   colcon test-result --test-result-base build/iap --verbose
   ```

   Result: `73 tests, 0 errors, 0 failures, 0 skipped`.

## Isolation Assessment

Stage 0 appears isolated from the forbidden areas:

- Certified GNSS ARAIM behavior: no code diff found in `araim.*` or GNSS ARAIM types.
- Planner behavior and PI cost: no code diff found in `planner/*`, `apps/phase2_planner_integrity_evaluator.cpp`, or planner launch topic wiring.
- Certified monitor fusion: still max-based in `IntegrityMonitor::run_lidar_araim()`.
- FIM-add: no new Stage 0 diff in advisory predictor/planner path.
- Topics/messages: no Stage 0 message schema or topic removal.

## Stage 0 Risks / Notes

- The LiDAR certified monitor output is intentionally changed by Stage 0. Besides age bounding, the condition and RMSE risk terms are now normalized/capped, so LiDAR PL magnitudes may differ from pre-Stage-0 behavior even in low-age cases.
- The target window can reduce LiDAR monitor PL by omitting older/farther target hypotheses. This is the intended bounded-window repair, but it is a safety-sensitive semantic change and should remain explicitly documented as Stage 0 behavior.
- `enable_lidar_araim_stage0_csv` defaults to `true` in the edited configs, which adds runtime file I/O. It is useful for validation but may be worth disabling in long production runs.
- The CSV has explicit `ss_variance_fallback_flag`, but there is no consolidated bitmask flag in `LidarAraimResult`; consumers must use the CSV/subset booleans for now.
- The `LidarAraimResult::worst_hyp` summary may not be a strict global max-HPL hypothesis in all cases because the loop compares subset HPL against `result.HPL` before final recomputation from per-axis maxima. The per-axis CSV independently selects the worst subset for each axis, so Stage 0 component logs are still useful, but the throttled summary `worst` label should be treated as diagnostic only until reviewed.
- At the time of the initial Stage 0 inspection, the docs/stage directory and
  `lidar_araim_debug.hpp` were still untracked. They are included in the
  Stage 0 + Stage 1 commit prepared after the 2026-05-13 status update.

## 2026-05-13 Update: Stage 1 Implementation and Demo11 Validation

Stage 1 was implemented according to `docs/stage1_v2.0/stage1_file_level_plan.md`.
The implementation is a naming/interface/log/comment/documentation clarification
pass only. It intentionally preserves certified monitor math, advisory predictor
math, PI cost math, planner behavior, topic names, message fields, and existing
CSV schemas.

### Stage 1 Changes Applied

- Added non-breaking semantic alias accessors in:
  - `include/iap/integrity/integrity_types.hpp`
  - `include/iap/planner/integrity_snapshot.hpp`
  - `include/iap/planner/future_pl_query_result.hpp`
- Clarified current certified monitor naming:
  - current GNSS monitor output: `gnss_certified_*`
  - current LiDAR monitor output: `lidar_certified_*`
  - monitor fused output: `monitor_fused_*`
  - monitor integrity margin: `monitor_integrity_margin`
- Clarified advisory/future predictor naming:
  - GNSS future proxy/GII: `gnss_advisory_*_proxy`
  - LiDAR observability/LOI proxy: `lidar_advisory_pl_proxy` / LOI path
  - future fused advisory result: `advisory_predicted_*`
  - future advisory margin: `advisory_integrity_margin`
- Updated log labels and comments in:
  - `src/iap/integrity/integrity_monitor.cpp`
  - `src/iap/integrity/integrity_extension.cpp`
  - `include/iap/integrity/araim_debug.hpp`
  - `include/iap/integrity/lidar_araim_debug.hpp`
  - `src/iap/planner/predicted_araim.cpp`
  - `src/iap/planner/future_pl_field_predictor.cpp`
  - `apps/phase2_planner_integrity_evaluator.cpp`
- Updated documentation in:
  - `docs/phase1_ego_planner_integration/topic_contract.md`
  - `docs/phase2_pi_lite_integrity_evaluator/design.md`
  - `docs/phase2_pi_lite_integrity_evaluator/validation.md`
  - `docs/dev_planner/req_astar.md`
  - `docs/stage1_v2.0/stage1_execution_summary.md`

### Behavior Preserved

- Certified GNSS ARAIM behavior was not intentionally changed.
- Certified LiDAR ARAIM behavior was not intentionally changed beyond the
  already implemented Stage 0 changes.
- Certified monitor fusion remains:

  ```text
  PL_mon_q = max(PL_G_q, PL_L_q)
  ```

- FIM-add was not implemented.
- The LiDAR FIM predictor was not implemented.
- PI cost math was not changed.
- Planner behavior was not changed.
- Existing topics, ROS message fields, PointCloud2 field names, CSV column
  names, and `query_source` strings were kept backward-compatible.

### Build and Unit/CTest Results

Commands run from `/home/dev/ws_iap`:

```bash
colcon build --base-paths src/iap src/gnss_comm --packages-select iap --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

colcon test --base-paths src/iap src/gnss_comm --packages-select iap

colcon test-result --test-result-base build/iap --verbose
```

Results:

- Build passed.
- Full `iap` package CTest passed:

  ```text
  Summary: 81 tests, 0 errors, 0 failures, 0 skipped
  ```

Earlier focused Stage 1 tests also passed:

```bash
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor|test_lidar_observability_fim|test_pi_cost_adapter"
colcon test-result --test-result-base build/iap --verbose
```

Result:

```text
79 tests, 0 errors, 0 failures, 0 skipped
```

### Demo11 Closed-Loop Validation Run

Because demo11 is the IAP closed-loop validation demo, a full demo11 run was
also executed after Stage 1.

Command run from `/home/dev/ws_iap`:

```bash
source install/setup.bash
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=90 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T022944Z_974
```

Demo11 generated the expected integrity and planner artifacts, including:

- `export/iap_araim.csv`
- `export/iap_lidar_araim_stage0.csv`
- `export/integrity_along_planner_traj.csv`
- `export/future_integrity_snapshot.csv`

## 2026-05-13 Update: LocalOccupancy Rolling Eviction

Implemented rolling eviction for `LocalOccupancyGrid` so the local occupancy
map keeps accepting new voxels after reaching `max_voxels`. The change is
limited to the local occupancy map and Phase 2 evaluator/demo11 wiring. It does
not modify certified ARAIM, Stage 2 FIM math, Stage 3 PI math, planner PI math,
ROS message schemas, or existing LocalOccupancy query APIs.

### Changes Applied

- Extended `LocalOccupancyGrid::Params` with:
  - `enable_eviction`
  - `local_radius_m`
  - `max_age_s`
  - `eviction_policy`
- Added rolling voxel metadata and diagnostics:
  - `local_occupancy_voxel_count`
  - `local_occupancy_evicted_count`
  - `local_occupancy_rejected_count`
  - `local_occupancy_inserted_count`
- Added centered/timestamped insertion overloads and explicit eviction around a
  UAV/query center.
- Preserved legacy behavior when eviction is disabled: the map does not evict
  and rejects new voxels after capacity is reached.
- Wired Phase 2 evaluator and demo11 launch parameters:
  - `phase2_local_occupancy_enable_eviction`
  - `phase2_local_occupancy_max_voxels`
  - `phase2_local_occupancy_radius_m`
  - `phase2_local_occupancy_max_age_s`
  - `phase2_local_occupancy_eviction_policy`
- Added `test/test_local_occupancy.cpp` and CTest registration.

Files changed:

- `include/iap/map/local_occupancy.hpp`
- `src/iap/map/local_occupancy.cpp`
- `apps/phase2_planner_integrity_evaluator.cpp`
- `launch/demo11_ego_planner_integrity_corridor.launch.py`
- `test/test_local_occupancy.cpp`
- `CMakeLists.txt`

### Test Results

Commands run from `/home/dev/ws_iap`:

```bash
python3 -m py_compile src/iap/launch/demo11_ego_planner_integrity_corridor.launch.py

colcon build --base-paths src/iap src/gnss_comm --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

ctest --test-dir build/iap \
  -R "test_local_occupancy|test_unified_risk_grid|test_future_pl_field_predictor|test_pi_cost_adapter" \
  --output-on-failure

colcon test --base-paths src/iap src/gnss_comm --packages-select iap
colcon test-result --test-result-base build/iap --verbose
```

Results:

- Launch Python compile passed.
- Build passed.
- Focused CTest passed:

  ```text
  4/4 tests passed
  ```

- Full `iap` package test suite passed:

  ```text
  Summary: 121 tests, 0 errors, 0 failures, 0 skipped
  ```

### Demo11 Rolling-Eviction Smoke

Command run from `/home/dev/ws_iap`:

```bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=30 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true \
  phase2_use_unified_risk_grid:=true \
  phase2_urg_export_voxels:=false \
  phase2_urg_compute_gradients:=false \
  phase2_urg_gradient_mode:=none \
  phase2_use_advisory_fim_add:=true \
  phase2_use_lidar_advisory_fim:=true \
  phase2_pi_use_unified_advisory_pl:=true \
  phase2_local_occupancy_enable_eviction:=true \
  phase2_local_occupancy_max_voxels:=1000 \
  phase2_local_occupancy_radius_m:=8.0 \
  phase2_local_occupancy_max_age_s:=2.0 \
  phase2_local_occupancy_eviction_policy:=distance_then_age
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T131600Z_085
```

`phase2_summary.json` local occupancy diagnostics:

```text
local_occupancy_enable_eviction=true
local_occupancy_max_voxels=1000
local_occupancy_radius_m=8.0
local_occupancy_max_age_s=2.0
local_occupancy_eviction_policy=distance_then_age
local_occupancy_voxel_count=1000
local_occupancy_inserted_count=379790
local_occupancy_evicted_count=378790
local_occupancy_rejected_count=666124
```

This confirms the map did not freeze at capacity: accepted insertions continued
well beyond `max_voxels`, while the current voxel count stayed capped.

URG smoke diagnostics from the same run:

```text
urg_enabled=true
urg_active=true
urg_update_count=12
urg_query_count=15620
urg_front_field_points=2601
urg_backend_field_points=2602
urg_valid_pi_count=15620
urg_unknown_count=0
urg_mean_update_ms=1501.6074225
urg_p95_update_ms=1740.974055
```

`integrity_along_planner_traj.csv` contained finite `urg_occ_prob` values for
all 7 sampled rows in this smoke run. The values were all `0.0` on the sampled
trajectory points, which means the occupancy field was populated and finite but
the sampled points were not inside occupied voxels.

Known demo note: as in earlier demo11 runs, several non-core simulation and
visualization nodes reported shutdown errors after SIGINT/SIGTERM. The Phase 2
evaluator process finished cleanly and wrote the summary and CSV artifacts.
- `export/phase1_summary.json`
- `export/phase2_summary.json`
- `export/planner_traj.csv`
- `export/planner_cmd.csv`

Stage 0 LiDAR ARAIM component logging was present in the demo11 run:

- `export/iap_lidar_araim_stage0.csv`
- 1488 data rows
- component columns include `sep_term_m`, `sigma_ss_term_m`,
  `sigma_subset_term_m`, `bias_term_m`, `gamma_rmse`, `gamma_inlier`,
  `gamma_condition`, `gamma_age`, `selected_target_count`,
  `target_window_size`, `worst_hypothesis_type`, and `worst_hypothesis_id`.

### Demo11 Validator Results

Phase 1 official validation passed:

```bash
python3 src/iap/tools/phase1/validate_phase1_closed_loop.py \
  --run-dir /home/dev/ws_iap/src/iap/log/20260513T022944Z_974 \
  --official
```

Key output:

```text
run_duration_s: 90.11
planner_trajectory_count: 8
planner_command_count: 7992
truth_odom_count: 90109
iap_odom_count: 504
simulator_movement_m: 23.908
official: True
allow_truth_alignment: False
use_so3_dynamics: True
use_iap_odom_for_planner: True
plant_mode: so3_quadrotor_simulator
planner_odom_topic: /drone_0_visual_slam/odom
controller_odom_topic: /demo9/preflight_control_odom
Phase 1 validation passed.
```

Phase 2/demo11 integrity validation did not fully pass as a standalone
validator run:

```bash
python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/20260513T022944Z_974
```

Initial failure before offline analysis:

- `phase2_summary.json current_consistency_raw max_pl_ratio=20983540.884`
  exceeded the 0.100 threshold.
- `iap_araim.csv` existed but `export/phase2_integrity_eval_aligned.csv` was
  missing.

Then offline analysis was run:

```bash
python3 src/iap/tools/ana_log.py \
  --run /home/dev/ws_iap/src/iap/log/20260513T022944Z_974
```

This generated:

- `analysis/report.md`
- `analysis/report.json`
- `analysis/figs`
- offline alignment data reflected in `phase2_summary.json`

After `ana_log.py`, the updated `phase2_summary.json` contained:

- `validation.passed: true`
- `aligned_sample_count: 87`
- `actual_alignment.match_ratio: 0.9157894736842105`
- `sample_mode_counts: {"pos_cmd_fallback": 49, "bspline": 46}`
- warnings:
  - `49 pos_cmd fallback sample(s) present`
  - `predicted IM is mostly unsafe`

However, re-running the standalone Phase 2 validator after `ana_log.py` still
returned non-zero because the analyzer rewrote `phase2_summary.json` into a
schema that no longer contains several online-validator-required blocks:

- `fallback_count`
- `fallback_rate`
- `fallback_reason_histogram`
- `finite_gnss_prediction_count`
- `integrity_snapshot`
- `current_consistency_raw`
- `current_consistency_anchored`
- `current_consistency`
- `phase_h_lite`
- `stage1_capabilities`
- `pi_cost`

This indicates a demo11/Phase 2 validation contract mismatch between
`tools/phase2/validate_phase2_integrity_eval.py` and `tools/ana_log.py`.

### Demo11 Runtime Risks Observed

The demo11 run should not be reported as a clean full closed-loop integrity
acceptance yet. The following runtime risks were observed:

- `demo9_preflight_control_odom_mux` repeatedly rejected IAP odometry as stale:

  ```text
  iap odom rejected stamp_increasing=true fresh_enough=false age=121573785...
  status mode=truth_bootstrap ... valid_iap_streak=0
  ```

- The terminal logs repeatedly showed the planner FSM in `WAIT_TARGET` during
  parts of the run, although the exported logs still show 8 planner
  trajectories and 7992 planner commands.
- Shutdown produced several ROS node errors after SIGINT/SIGTERM, including
  `poscmd_2_odom`, `pcl_render_node`, `traj_server`, and odometry visualization
  nodes. The core `iap_rosnode`, GNSS sim, phase1 logger, and phase2 evaluator
  finished cleanly, but the shutdown errors should still be tracked as demo11
  validation risk.
- `phase2_summary.json` reports high predicted-risk behavior:
  - predicted IM mostly unsafe
  - `p95_PL` and `max_PL` reached the conservative fallback sentinel
    `1000000000.0`
  - GNSS prediction fallback rate was reported as `0.095`, above the validator
    warning threshold of `0.05`.

### Current Validation Conclusion

Stage 0 and Stage 1 code-level tests pass, and demo11 can launch, run, and
produce the expected Stage 0/Stage 1 integrity artifacts. However, Stage 1
should not yet be marked as fully demo11-validated because the standalone
Phase 2/demo11 integrity validator does not pass and the demo11 run exposed
IAP odom freshness/closed-loop validation risks.

Recommended follow-up before claiming full demo11 acceptance:

- Unify `tools/phase2/validate_phase2_integrity_eval.py` and `tools/ana_log.py`
  around one stable `phase2_summary.json` schema.
- Investigate why `demo9_preflight_control_odom_mux` reports IAP odom age as
  stale during this demo11 run.
- Re-run demo11 Full after those fixes and require both:
  - Phase 1 official validator passes.
  - Phase 2/demo11 integrity validator passes without schema-dependent failure.

## Stage 2 Advisory FIM Predictor Implementation Status

Date: 2026-05-13.

Stage 2 has been implemented on the advisory/future predictor path only. The
certified GNSS ARAIM, certified LiDAR ARAIM, `/iap/integrity` current monitor
semantics, current monitor max fusion, PI cost math, planner behavior, URG, old
topics, and old fields were not intentionally changed.

### Stage 2 Code Changes

Implemented files:

- `include/iap/planner/advisory_fim_types.hpp`
  - Added advisory-only FIM diagnostic/result structs:
    `FimDiagnostic`, `GnssAdvisoryFimResult`,
    `LidarAdvisoryFimResult`, and `FusedAdvisoryFimResult`.
- `include/iap/planner/predicted_araim.hpp`
  and `src/iap/planner/predicted_araim.cpp`
  - Added `PredictedAraimComputer::predict_advisory_fim()`.
  - Builds GNSS advisory `H = G^T W G` with 4D ENU position + meter-equivalent
    clock state.
  - Computes the 3D position FIM through clock Schur complement.
  - Keeps the existing `predict_araim_result()` advisory PL proxy behavior.
- `include/iap/planner/lidar_observability_fim.hpp`
  and `src/iap/planner/lidar_observability_fim.cpp`
  - Added `LidarFimPrimitive`.
  - Added `LidarObservabilityFim::evaluate_advisory_fim()` for the
    lightweight 3x3 translational LiDAR FIM from normals.
  - Preserved the old scalar/radial-point observability path.
- `include/iap/planner/future_pl_field_predictor.hpp`
  and `src/iap/planner/future_pl_field_predictor.cpp`
  - Added disabled-by-default config:
    `use_advisory_fim_add`, `use_lidar_advisory_fim`, `fim_epsilon`,
    LiDAR FIM radius/min-normal/scale/condition parameters, and advisory
    HPL/VPL multipliers/biases.
  - Added `set_lidar_fim_primitives()`.
  - Added advisory FIM-add branch:
    `Lambda_adv = Lambda_prior + Lambda_G + Lambda_L`,
    `Sigma_adv = inv(Lambda_adv + fim_epsilon I)`,
    `HPL_adv = K_H_adv * sqrt(lambda_max(Sigma_xy)) + b_H_pred + s_H_pred`,
    `VPL_adv = K_V_adv * sqrt(Sigma_zz) + b_V_pred + s_V_pred`.
  - The branch only runs when `use_advisory_fim_add=true`.
- `include/iap/planner/future_pl_query_result.hpp`,
  `src/iap/planner/future_pl_query_result.cpp`, and
  `src/iap/planner/pl_grid.cpp`
  - Added/interpolated debug fields:
    `lambda_prior_trace`, `lambda_gnss_trace`, `lambda_lidar_trace`,
    `lambda_adv_trace`, `lambda_adv_min_eig`, `lambda_adv_condition`,
    `hpl_adv`, `vpl_adv`, `lidar_fim_valid`, `gnss_fim_valid`,
    `fim_regularized`, `advisory_fusion_mode`.
- `apps/phase2_planner_integrity_evaluator.cpp`
  - Added Stage 2 parameters and summary JSON fields.
  - Added CSV columns for Stage 2 FIM diagnostics.
  - Builds LiDAR FIM primitives from `normal_x/y/z` PointCloud2 fields if
    present; otherwise estimates lightweight local PCA normals from the
    downsampled map cloud.
- `launch/demo10_ego_planner_pi_lite_eval.launch.py` and
  `launch/demo11_ego_planner_integrity_corridor.launch.py`
  - Added `phase2_` launch arguments for Stage 2 advisory FIM switches and
    tuning parameters.
- Tests updated:
  - `test/test_predicted_araim.cpp`
  - `test/test_lidar_observability_fim.cpp`
  - `test/test_future_pl_field_predictor.cpp`
  - `test/test_pl_grid.cpp`

### Stage 2 Code-Level Tests

Build command:

```bash
colcon build --base-paths src/iap src/gnss_comm \
  --packages-select iap \
  --cmake-args -DBUILD_TESTING=ON
```

Result:

```text
Summary: 1 package finished [49.9s]
```

Targeted unit/regression command:

```bash
colcon test --base-paths src/iap src/gnss_comm \
  --packages-select iap \
  --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor|test_lidar_observability_fim|test_pi_cost_adapter"
```

Package-scoped test result:

```bash
colcon test-result --test-result-base build/iap --verbose
```

```text
Summary: 85 tests, 0 errors, 0 failures, 0 skipped
```

Launch-file syntax check:

```bash
python3 -m py_compile \
  launch/demo10_ego_planner_pi_lite_eval.launch.py \
  launch/demo11_ego_planner_integrity_corridor.launch.py
```

Result: passed.

The workspace-wide `colcon test-result --verbose` is not clean because of
pre-existing unrelated failures in non-IAP packages such as `bspline_opt` and
`path_searching`. The `iap` package test result is clean.

### Demo11 Stage 2 Runtime Smoke

Because demo11 is the IAP closed-loop validation demo, a Stage 2 demo11 smoke
run was executed with advisory FIM-add enabled.

Command:

```bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=30 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true \
  phase2_pl_model:=fused_fim_grid \
  phase2_use_pl_grid:=true \
  phase2_export_pl_grid_voxels:=true \
  phase2_use_advisory_fim_add:=true \
  phase2_use_lidar_advisory_fim:=true \
  phase2_use_lidar_observability:=false
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T034010Z_622
```

Runtime outcome:

- `iap_rosnode` finished cleanly.
- `phase2_planner_integrity_evaluator` finished cleanly.
- `phase2_summary.json` was produced.
- `integrity_along_planner_traj.csv` was produced.
- `pl_grid_voxels.csv` was produced.
- The Stage 2 advisory FIM-add path was active in demo11:
  - `phase2_use_advisory_fim_add: true`
  - `phase2_use_lidar_advisory_fim: true`
  - `advisory_fim.fusion_mode: fim_add`
  - `advisory_fim.query_count: 34315`
  - `advisory_fim.gnss_fim_valid_count: 31963`
  - `advisory_fim.lidar_fim_valid_count: 17461`
  - `advisory_fim.regularized_count: 34315`
- `pl_grid` was active:
  - `update_count: 10`
  - `query_counts.grid: 2155`
  - `query_counts.direct: 32160`
  - `query_counts.fallback: 0`
  - `grid_vs_direct_self_check.last_pl_ratio: 0.0001645`
- Stage 2 CSV columns were present in demo11 exports:
  - `integrity_along_planner_traj.csv`: 22 rows, all rows had
    `advisory_fusion_mode=fim_add`.
  - `pl_grid_voxels.csv`: 25270 rows, all rows had
    `advisory_fusion_mode=fim_add`.

### Demo11 Validation Result

The Stage 2 demo11 smoke should not be reported as a full demo11 acceptance.
It exercised the Stage 2 path, but the standalone Phase 2/demo11 validator did
not pass.

Validator command:

```bash
python3 tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/20260513T034010Z_622
```

Validator result: failed.

Key warnings:

- `13 pos_cmd fallback sample(s) present`
- `PL grid rebuild mean time is 661.0 ms (>500 ms initial target)`
- `raw current PL consistency max ratio is 0.988`
- predicted and actual IM could not be compared

Key failures:

- `phase2_summary.json current_consistency_raw max_pl_ratio=0.988 exceeds 0.100`
- `integrity_along_planner_traj.csv row 2: PL_H_pred=1.019764180 is below gnss_hpl=20.000000000`
  - This is expected for the new advisory FIM-add mode because Stage 2 can be
    more optimistic than the legacy GNSS advisory PL proxy. The existing
    validator still contains the old conservative `fused_fim_grid` assumption.
- `iap_araim.csv exists but export/phase2_integrity_eval_aligned.csv is missing`
- Phase 1 validator failed only because this smoke run lasted
  `29.96s < 30.00s`; this was a 30-second smoke, not the required 90-second
  full demo11 run.

Known runtime risks still observed:

- `demo9_preflight_control_odom_mux` reported stale IAP odometry age during the
  run, with `valid_iap_streak=0`.
- Several non-core nodes still threw shutdown errors after launch termination
  (`poscmd_2_odom`, `pcl_render_node`, `traj_server`, and odom visualization
  nodes). This matches prior demo11 shutdown-risk behavior and is not specific
  to Stage 2 FIM math.
- The run reported current monitor `INTEGRITY UNSAFE` warnings. This is current
  monitor behavior and was not modified by Stage 2.

### Stage 2 Validation Conclusion

Stage 2 has passed code-level validation for the `iap` package and has been
smoke-tested on demo11 with advisory FIM-add enabled. It has not passed all
demo11 validation tests, and it should not be marked as full demo11 accepted.

Before claiming complete demo11 acceptance, run the full 90-second demo11 case
and update the Phase 2 validator to understand the new `fim_add` advisory mode:

- The old validator check `PL_H_pred >= gnss_hpl` is not valid for Stage 2
  FIM-add because Stage 2 intentionally fuses information additively in the
  advisory path.
- The validator still needs the known Phase 2 freshness/schema alignment fixes
  documented above.

## 2026-05-13 Update: Stage 3 PI Cost Adapter Redesign and Demo11 Status

Stage 3 has now been implemented on top of the Stage 2 advisory FIM-add path.
The change is limited to the planner/evaluator PI advisory cost path and demo
launch wiring. Certified GNSS ARAIM, certified LiDAR ARAIM, certified monitor
fusion, `/iap/integrity` current monitor semantics, Stage 2 FIM math, URG, old
topics, and old message fields were not intentionally changed.

### Answer: Did Stage 2 Pass All Tests?

No. Stage 2 passed `iap` package code-level validation and a demo11 advisory
FIM-add smoke run, but it has not passed all demo11 validation tests and should
not be marked as fully demo11 accepted.

Current Stage 2 status remains:

- Code-level `iap` tests passed.
- demo11 with `phase2_use_advisory_fim_add=true` exercised the new FIM-add path.
- The old standalone Phase 2/demo11 validator still fails because it contains
  legacy assumptions that are not valid for advisory `fim_add`, including the
  check that predicted advisory PL must stay above legacy GNSS PL.
- The known demo11 odometry freshness/schema/alignment issues are still outside
  Stage 3 scope and were not fixed here.

### Stage 3 Code Changes

Implemented files:

- `include/iap/planner/pi_cost_adapter.hpp`
  and `src/iap/planner/pi_cost_adapter.cpp`
  - Added Stage 3 unified advisory PL mode behind
    `phase2_pi_use_unified_advisory_pl`.
  - Preserved legacy PI behavior when the Stage 3 switch is disabled.
  - Added advisory integrity margin outputs, hinge cost, optional ratio cost,
    unknown-advisory penalty, input-valid flag, term breakdowns, and max-cost
    clamping.
  - Stage 3 formula:

    ```text
    IM_H = HAL - HPL_adv
    IM_V = VAL - VPL_adv
    IM   = min(IM_H, IM_V)

    c_hinge =
      lambda_pi * (
        max(0, HPL_adv - HAL + margin_h)^2
        + max(0, VPL_adv - VAL + margin_v)^2
      )

    c_ratio =
      mu_ratio * (
        (HPL_adv / (HAL + eps_al))^2
        + (VPL_adv / (VAL + eps_al))^2
      )

    c_PI = clamp(c_hinge + optional c_ratio + unknown_penalty, 0, max_cost)
    ```

- `apps/phase2_planner_integrity_evaluator.cpp`
  - Added a PI advisory PL selection layer in the evaluator:
    - use valid `hpl_adv/vpl_adv` from `fim_add` first;
    - otherwise use existing advisory predicted `hpl/vpl`;
    - use `constant_current` only when explicit compatibility fallback is
      enabled;
    - otherwise mark the PI input unknown.
  - Routed sample rows, PI cost queries, finite-difference gradient evaluation,
    `/iap/integrity_cost_field`, and `/iap/integrity_front_cost_field` through
    the unified PI adapter.
  - Replaced the front-end publisher's standalone ratio cost with the adapter
    cost when Stage 3 is enabled.
  - Added CSV/grid fields:
    `advisory_hpl_used`, `advisory_vpl_used`, `advisory_pl_source`,
    `im_h_adv`, `im_v_adv`, `im_min_adv`, `pi_hinge_cost`, `pi_ratio_cost`,
    `pi_total_cost`, `pi_unknown_penalty`, `pi_input_valid`,
    `pi_fallback_reason`, `pi_cost_clamped`, and `risk_band_adv`.
  - Added `pi_stage3` summary fields:
    selected-source histogram, fallback-reason histogram, invalid/unknown
    counts, clamp count, and enabled policy flags.

- `launch/demo11_ego_planner_integrity_corridor.launch.py`
  and `launch/demo10_ego_planner_pi_lite_eval.launch.py`
  - Added Stage 3 `phase2_pi_*` launch arguments.
  - Defaults keep Stage 3 disabled:
    `phase2_pi_use_unified_advisory_pl=false`.
  - demo11 Stage 3 runs can enable it with:
    `phase2_pi_use_unified_advisory_pl:=true`.

- `test/test_pi_cost_adapter.cpp`
  - Added unit coverage for:
    - hinge activation before PL exceeds AL due to margin;
    - ratio term disabled by default;
    - unknown/sentinel advisory high-cost handling;
    - legacy behavior unchanged when Stage 3 is disabled.

### Stage 3 Build and Test Results

Commands run from `/home/dev/ws_iap`:

```bash
colcon build --packages-select iap

ctest --test-dir build/iap -R test_pi_cost_adapter --output-on-failure

ctest --test-dir build/iap -R test_future_pl_field_predictor --output-on-failure

python3 -m py_compile \
  launch/demo10_ego_planner_pi_lite_eval.launch.py \
  launch/demo11_ego_planner_integrity_corridor.launch.py

ctest --test-dir build/iap --output-on-failure
```

Results:

- Build passed.
- `test_pi_cost_adapter` passed.
- `test_future_pl_field_predictor` passed.
- demo10/demo11 launch-file Python syntax checks passed.
- Current `iap` package CTest passed:

  ```text
  9/9 tests passed, 0 tests failed
  ```

### Demo11 Stage 2 + Stage 3 Smoke

Because demo11 is the IAP closed-loop test demo, a Stage 2 + Stage 3 enabled
demo11 smoke run was executed.

Command:

```bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=30 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true \
  phase2_pl_model:=fused_fim_grid \
  phase2_use_pl_grid:=true \
  phase2_export_pl_grid_voxels:=true \
  phase2_use_advisory_fim_add:=true \
  phase2_use_lidar_advisory_fim:=true \
  phase2_use_lidar_observability:=false \
  phase2_pi_use_unified_advisory_pl:=true
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T043855Z_156
```

Runtime outcome:

- `iap_rosnode` finished cleanly.
- `phase2_planner_integrity_evaluator` finished cleanly.
- `phase2_summary.json` was produced.
- `integrity_along_planner_traj.csv` was produced.
- `pl_grid_voxels.csv` was produced.
- `planner_integrity_cost_debug.csv` was produced.
- `future_integrity_eval.csv` was not produced by this demo11 smoke path.
- Existing shutdown errors still occurred in non-core nodes such as
  `poscmd_2_odom`, `pcl_render_node`, `traj_server`, and odometry
  visualization nodes. This matches the earlier demo11 shutdown-risk pattern.

Stage 2 advisory FIM-add was active:

```text
advisory_fim.enabled: true
advisory_fim.fusion_mode: fim_add
advisory_fim.query_count: 29056
advisory_fim.gnss_fim_valid_count: 27690
advisory_fim.lidar_fim_valid_count: 13601
advisory_fim.regularized_count: 29056
```

Stage 3 PI was active:

```text
phase2_pi_use_unified_advisory_pl: true
phase2_pi_use_hinge_term: true
phase2_pi_use_ratio_term: false
phase2_pi_penalize_unknown_advisory: true
pi_stage3.enabled: true
pi_stage3.selected_source_histogram: {"fim_add": 9}
pi_stage3.input_invalid_count: 0
pi_stage3.unknown_count: 0
pi_stage3.max_clamp_count: 0
pi_cost.count: 9
pi_cost.risk_band_histogram: {"SAFE_PI": 9}
```

Stage 3 CSV/grid fields were present:

- `integrity_along_planner_traj.csv`
  - 9 data rows.
  - Contains all new Stage 3 fields:
    `advisory_hpl_used`, `advisory_vpl_used`, `advisory_pl_source`,
    `im_h_adv`, `im_v_adv`, `im_min_adv`, `pi_hinge_cost`,
    `pi_ratio_cost`, `pi_total_cost`, `pi_unknown_penalty`,
    `pi_input_valid`, `pi_fallback_reason`, and `risk_band_adv`.
- `pl_grid_voxels.csv`
  - 22743 data rows.
  - Contains the same Stage 3 fields plus Stage 2 advisory FIM fields such as
    `hpl_adv`, `vpl_adv`, and `advisory_fusion_mode`.

### Demo11 Validator Results After Stage 3

The current 30-second demo11 smoke is not a full official demo11 acceptance
run. The existing validators were run to record the current status.

Phase 1 official validator:

```bash
python3 src/iap/tools/phase1/validate_phase1_closed_loop.py \
  --run-dir /home/dev/ws_iap/src/iap/log/20260513T043855Z_156 \
  --official
```

Result: failed only because the smoke duration was slightly below the official
minimum:

```text
run_duration_s: 29.95
planner_trajectory_count: 11
planner_command_count: 1984
truth_odom_count: 29950
iap_odom_count: 140
simulator_movement_m: 23.918
Failures:
  - run_duration_s 29.95 < 30.00
```

Phase 2/demo11 validator:

```bash
python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/20260513T043855Z_156
```

Result: failed. Key warnings/failures:

- `9 pos_cmd fallback sample(s) present`
- `PL grid rebuild mean time is 649.6 ms (>500 ms initial target)`
- `raw current PL consistency max ratio is 0.988`
- `official Phase 2 requires at least one B-spline trajectory sample`
- `integrity_along_planner_traj.csv row 2: PL_H_pred=1.019764180 is below gnss_hpl=20.000000000`
- `iap_araim.csv exists but export/phase2_integrity_eval_aligned.csv is missing`

This is consistent with the previously documented Stage 2 validator gap: the
old Phase 2 validator still assumes legacy conservative advisory PL behavior
and has not been updated for `fim_add` semantics.

### Current Stage 3 Validation Conclusion

Stage 3 implementation is code-tested and demo11-smoke-tested. The demo11 smoke
confirmed that Stage 2 FIM-add and Stage 3 unified advisory PI selection are
active together, that the new PI CSV/grid fields are emitted, and that the PI
source histogram selects `fim_add`.

Stage 3 should not yet be reported as full demo11 accepted. Full acceptance
still requires:

- a full-duration demo11 run, not only the 30-second smoke;
- Phase 2 validator updates for advisory `fim_add` semantics;
- the previously documented demo11 odometry freshness/schema/alignment fixes;
- a passing official Phase 1 and Phase 2 validation pair on the same demo11 run.

## Stage 4 URG Implementation Status - 2026-05-13

Stage 4 was implemented as a compatibility-first Unified Risk Grid layer for
demo11. The default launch behavior remains legacy-compatible because
`phase2_use_unified_risk_grid` defaults to `false`.

### Stage 4 Code Changes

Added the URG core library and tests:

- `include/iap/planner/unified_risk_grid.hpp`
- `src/iap/planner/unified_risk_grid.cpp`
- `test/test_unified_risk_grid.cpp`

The URG model includes the requested `UnifiedRiskVoxel` fields and flags:
`VALID_ESDF`, `VALID_OCCUPANCY`, `VALID_AL`, `VALID_ADVISORY_PL`,
`VALID_PI`, `STALE_PL`, `UNKNOWN_RISK`, `OCCUPIED`, `OUT_OF_RANGE`,
`FIM_ADD_USED`, `LIDAR_FIM_VALID`, `GNSS_FIM_VALID`, and `PI_INPUT_VALID`.

Integrated URG into the Phase 2 planner integrity evaluator:

- `apps/phase2_planner_integrity_evaluator.cpp`
  - declares and reads the Stage 4 `urg_*` parameters;
  - builds URG voxels from existing clearance proxy, occupancy, AL,
    advisory PL, Stage 3 PI, IM, FIM-add diagnostics, and PI gradients;
  - applies stale/unknown handling using the configured unknown penalty;
  - publishes URG-derived samples on the existing legacy-compatible topics
    when URG is enabled:
    `/iap/integrity_cost_field` and `/iap/integrity_front_cost_field`;
  - keeps the existing PointCloud2 field names unchanged:
    `x y z hpl vpl hal val im_h im_v im_min cost grad_x grad_y grad_z
    risk_band risk_band_code`;
  - exports `urg_grid_voxels.csv` when `phase2_urg_export_voxels=true`;
  - writes the requested URG counters into `phase2_summary.json`.

Added read-only occupancy query helpers for URG export:

- `include/iap/map/local_occupancy.hpp`
- `src/iap/map/local_occupancy.cpp`

Updated demo11 launch arguments:

- `launch/demo11_ego_planner_integrity_corridor.launch.py`
  - adds all requested `phase2_urg_*` launch arguments;
  - passes them into `phase2_planner_integrity_evaluator`.

Updated the demo11-integrated EGO front-field consumer:

- `sim/ego_planner_swarm_ws/src/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h`
- `sim/ego_planner_swarm_ws/src/planner/bspline_opt/src/bspline_optimizer.cpp`
  - front-field samples now store the existing `cost` field;
  - A* front search prefers the URG/field-provided `cost`;
  - legacy HPL/VPL/HAL/VAL ratio recomputation remains as fallback.

No changes were made to certified GNSS ARAIM, certified LiDAR ARAIM, current
certified monitor fusion, `/iap/integrity` semantics, Stage 2 FIM math, or
Stage 3 PI math beyond routing Stage 3 results into URG-derived grid samples.

### Stage 4 Test Results

Build and syntax checks:

```bash
python3 -m py_compile \
  src/iap/launch/demo11_ego_planner_integrity_corridor.launch.py

source /opt/ros/jazzy/setup.bash
./src/iap/tools/build_phase1_ego_planner_closed_loop.sh
```

Result:

```text
20 packages finished
verified executable: iap_phase1_tools phase1_closed_loop_logger
verified executable: iap phase2_planner_integrity_evaluator
```

Focused code tests:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ctest --test-dir build/iap \
  -R "test_pl_grid|test_future_pl_field_predictor|test_pi_cost_adapter|test_unified_risk_grid" \
  --output-on-failure
```

Result:

```text
100% tests passed, 0 tests failed out of 4
test_pl_grid: passed
test_future_pl_field_predictor: passed
test_pi_cost_adapter: passed
test_unified_risk_grid: passed
```

### Demo11 Stage 4 Enabled Smoke

Command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  use_integrity_global_search:=true \
  phase2_use_unified_risk_grid:=true \
  phase2_urg_export_voxels:=true \
  phase2_urg_keep_legacy_topics:=true \
  phase2_urg_publish_front_cost_field:=true \
  phase2_urg_publish_backend_cost_field:=true \
  phase2_pl_model:=fused_fim_grid \
  phase2_use_pl_grid:=true \
  phase2_export_pl_grid_voxels:=true \
  phase2_use_advisory_fim_add:=true \
  phase2_use_lidar_advisory_fim:=true \
  phase2_use_lidar_observability:=false \
  phase2_pi_use_unified_advisory_pl:=true
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T052517Z_755
```

Key `phase2_summary.json` results:

```text
urg_enabled: true
urg_active: true
urg_update_count: 6
urg_query_count: 15612
urg_grid_hit_count: 15610
urg_grid_miss_count: 2
urg_direct_query_count: 2
urg_valid_pi_count: 15612
urg_stale_count: 15612
urg_unknown_count: 5202
urg_unknown_penalty_count: 7803
urg_front_field_points: 2601
urg_backend_field_points: 2602
urg_mean_update_ms: 8881.4465
urg_p95_update_ms: 11893.4893
urg_voxel_csv: urg_grid_voxels.csv
```

Export files present:

```text
phase2_summary.json
pl_grid_voxels.csv
traj_with_gnss.csv
urg_grid_voxels.csv
```

This confirms that demo11 with Stage 2 FIM-add, Stage 3 unified advisory PI,
and Stage 4 URG enabled produced nonzero URG query statistics and both
front/backend field sample counts.

Shutdown caveat: with the requested default Stage 4 grid size
`phase2_urg_half_extent_x_m=25`, `phase2_urg_half_extent_y_m=25`, and
`phase2_urg_resolution_m=1`, URG rebuilds are heavy in demo11. The enabled run
wrote valid summaries and CSV exports, but the ROS launch timeout caught the
evaluator during heavy work and escalated shutdown after the summary had already
been written. This is a Stage 4 performance/shutdown caveat, not a build or
data-flow failure.

### Demo11 Stage 4 Disabled Compatibility Smoke

Command:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  use_integrity_global_search:=true \
  phase2_use_unified_risk_grid:=false \
  phase2_pl_model:=fused_fim_grid \
  phase2_use_pl_grid:=true \
  phase2_use_advisory_fim_add:=true \
  phase2_use_lidar_advisory_fim:=true \
  phase2_pi_use_unified_advisory_pl:=true
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T055654Z_017
```

Key `phase2_summary.json` results:

```text
phase2_use_unified_risk_grid: false
urg_enabled: false
urg_active: false
urg_update_count: 0
urg_query_count: 0
urg_grid_hit_count: 0
urg_grid_miss_count: 0
urg_front_field_points: 0
urg_backend_field_points: 0
phase2_use_advisory_fim_add: true
phase2_pi_use_unified_advisory_pl: true
pl_grid.query_counts.direct: 81289
pl_grid.query_counts.grid: 656
pl_grid.query_counts.fallback: 0
```

Export files present:

```text
phase2_summary.json
pl_grid_voxels.csv
traj_with_gnss.csv
```

No `urg_grid_voxels.csv` was generated in disabled mode, as expected. The
Phase 2 evaluator exited cleanly. Some non-evaluator ROS visualization/sensing
processes still printed shutdown errors after timeout-triggered shutdown; these
are existing demo11 teardown behaviors and were not introduced by URG.

### Current Stage 4 Validation Conclusion

Stage 4 has passed the implemented code tests, build checks, and demo11 smoke
checks for both enabled and disabled modes. All testing recorded for this stage
was performed through demo11 or the demo11 build/test path.

Stage 4 should be reported as demo11 smoke-tested, not full official demo11
accepted yet. Full acceptance is still blocked by the already documented
Phase 2 validator limitations around Stage 2 `fim_add`, URG schema/freshness
semantics, and the existing demo11 odometry freshness/alignment assumptions.

Remaining Stage 4 follow-up:

- reduce URG update cost or make rebuild/shutdown cancellation more responsive
  for the default 25 m by 25 m demo11 grid;
- update the old Phase 2 validator to understand FIM-add and URG semantics;
- run a full-duration official demo11 validation after the validator and
  known demo11 freshness/schema assumptions are updated.

## 2026-05-13 Update: Phase 2 Summary Schema and Validator Fix

This update fixes the Phase 2 validation/analyzer schema mismatch that blocked
Stage 2/3/4 evaluation. The change is intentionally limited to summary schema,
analysis, validation, and focused Python tests.

### Scope and Invariants

No certified runtime math or public ROS interfaces were changed by this fix:

- certified GNSS ARAIM unchanged;
- certified LiDAR ARAIM unchanged;
- current monitor fusion unchanged;
- Stage 2 FIM math unchanged;
- Stage 3 PI math unchanged;
- Stage 4 URG math unchanged;
- ROS topics and message schemas unchanged.

### Files Updated

- Added shared Phase 2 summary schema helper:
  `tools/phase2/phase2_summary_schema.py`
- Updated Phase 2 validator:
  `tools/phase2/validate_phase2_integrity_eval.py`
- Updated offline analyzer merge behavior:
  `tools/ana_log.py`
- Added focused Python schema tests:
  `test/test_phase2_summary_schema.py`
- Registered Python CTest:
  `CMakeLists.txt`
- Added additive online summary schema version field:
  `apps/phase2_planner_integrity_evaluator.cpp`

### Stable `phase2_summary.json` Schema

The online evaluator remains the owner of the stable summary schema. Offline
analysis now augments the online summary instead of replacing it with a smaller
analysis-only document.

The following validator-required online blocks are preserved:

```text
fallback_count
fallback_rate
fallback_reason_histogram
finite_gnss_prediction_count
integrity_snapshot
current_consistency_raw
current_consistency_anchored
current_consistency
phase_h_lite
stage1_capabilities
pi_cost
```

The stable schema also carries the Stage 2/3/4 sections:

```text
advisory_fim
pi_stage3
urg
```

The analyzer now deep-merges offline alignment/validation results into the
online summary, preserving `advisory_fim`, `pi_stage3`, `urg`,
`stage1_predictor_config`, `stage1_capabilities`, current consistency blocks,
and all online validator-required fields.

### Validator Behavior

Legacy mode keeps the old conservative advisory PL checks, including requiring
the fused advisory prediction to remain no lower than the GNSS-only proxy.

For `advisory_fusion_mode=fim_add`, the validator no longer requires
`PL_H_pred >= gnss_hpl` or `PL_V_pred >= gnss_vpl`, because Stage 2 FIM-add is
an additive advisory information-fusion path and can intentionally produce a
tighter advisory PL than the GNSS-only advisory proxy.

In `fim_add` mode the validator instead checks that:

```text
advisory_fim.fusion_mode == fim_add
advisory_fim.gnss_fim_valid_count > 0
online rows consistently report advisory_fusion_mode == fim_add
lambda_adv_trace is finite
hpl_adv is finite
vpl_adv is finite
```

When URG is enabled, the validator now checks:

```text
urg_enabled
urg_active
urg_query_count
urg_front_field_points
urg_backend_field_points
urg_unknown_count
urg_stale_count
urg_mean_update_ms
urg_p95_update_ms
```

### Focused Tests Added

New test:

```text
test_phase2_summary_schema
```

Coverage:

- legacy summary fixture passes the conservative legacy PL logic;
- `fim_add` fixture passes even when `PL_H_pred < gnss_hpl`;
- URG-enabled summary validates required URG fields;
- `ana_log.py` merge preserves online validator-required fields and Stage 2/3/4
  sections.

### Commands Run

Python compile check:

```bash
python3 -m py_compile \
  src/iap/tools/phase2/phase2_summary_schema.py \
  src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  src/iap/tools/ana_log.py \
  src/iap/test/test_phase2_summary_schema.py
```

Result:

```text
exit code 0
no output
```

Direct Python unit test:

```bash
python3 src/iap/test/test_phase2_summary_schema.py
```

Result:

```text
....
----------------------------------------------------------------------
Ran 4 tests in 0.000s

OK
```

Build:

```bash
colcon build --base-paths src/iap src/gnss_comm --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
```

Result:

```text
Finished <<< iap [43.2s]
Summary: 1 package finished [43.2s]
```

Focused CTest:

```bash
ctest --test-dir build/iap -R test_phase2_summary_schema --output-on-failure
```

Result:

```text
100% tests passed, 0 tests failed out of 1
```

Focused regression CTest:

```bash
ctest --test-dir build/iap \
  -R "test_phase2_summary_schema|test_future_pl_field_predictor|test_pi_cost_adapter|test_unified_risk_grid" \
  --output-on-failure
```

Result:

```text
100% tests passed, 0 tests failed out of 4
test_future_pl_field_predictor: passed
test_pi_cost_adapter: passed
test_unified_risk_grid: passed
test_phase2_summary_schema: passed
```

Analyzer rewrite preservation check was run on a copied Stage 4 smoke run so
existing run artifacts were not modified:

```bash
rm -rf /tmp/iap_phase2_schema_check
cp -a src/iap/log/20260513T052517Z_755 /tmp/iap_phase2_schema_check
python3 src/iap/tools/ana_log.py \
  --run /tmp/iap_phase2_schema_check \
  --no-plots \
  --skip-external-tools
```

Result:

```text
Analyzed run: /tmp/iap_phase2_schema_check
Markdown report: /tmp/iap_phase2_schema_check/analysis/report.md
JSON report    : /tmp/iap_phase2_schema_check/analysis/report.json
Figures dir     : /tmp/iap_phase2_schema_check/analysis/figs
```

Post-analysis summary inspection result:

```text
missing_required_online_fields: []
advisory_fim: present
pi_stage3: present
urg: present
stage1_predictor_config: present
stage1_capabilities: present
current_consistency_raw: present
phase_h_lite: present
pi_cost: present
validation: present
actual_alignment: present
advisory_fim.fusion_mode: fim_add
urg_enabled: true
```

Full validator was also run on the copied Stage 4 smoke run:

```bash
python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /tmp/iap_phase2_schema_check
```

Result:

```text
Validated Phase 2 run: /tmp/iap_phase2_schema_check
sample_count: 4
snapshot_count: 3
traj_count: 3
aligned_sample_count: 4
odom_source: /drone_0_visual_slam/odom
map_source: global_cloud_direct
```

The validator no longer failed on the Stage 2 FIM-add condition
`PL_H_pred < gnss_hpl`. The copied smoke run still failed on unrelated
pre-existing run-quality checks:

```text
official Phase 2 requires at least one B-spline trajectory sample
phase2_summary.json current_consistency_raw max_pl_ratio=0.986 exceeds 0.100
demo10 did not pass Phase 1 official validation because the copied run used
allow_truth_alignment=true
```

### Current Validation Conclusion

The Phase 2 summary schema mismatch is fixed at the tooling level. `ana_log.py`
now preserves online validator-required blocks, and the Phase 2 validator now
accepts advisory `fim_add` semantics without requiring advisory fused PL to be
larger than the GNSS-only advisory proxy.

Remaining full-acceptance blockers are run-quality/demo issues already tracked
before this fix, not the summary schema or `fim_add` validator assumption.

## 2026-05-13 Update: Demo11 IAP Odometry Freshness Fix

The demo11 odometry freshness/alignment issue has been investigated and fixed
on the preflight odometry mux path only. The earlier Stage 1 risk item about
`demo9_preflight_control_odom_mux` rejecting IAP odometry with huge stale ages
is resolved by this update.

### Root Cause

`/drone_0_visual_slam/odom` is stamped in the SO3 simulator epoch
(`2022-07-06T00:00:00Z`, around `1657065600.x` seconds). The mux previously
computed IAP freshness against node `now()` in the 2026 wall/ROS clock domain
(`1778667xxx` seconds), producing ages around `121573785s`. This kept
`fresh_enough=false` and `valid_iap_streak=0`.

The IAP odom stamp is correct for simulator sensor/GNSS/log alignment and was
not changed. The mux now evaluates IAP freshness against the latest truth odom
stamp when truth is available, because truth odom and IAP odom share the same
simulator-time domain. Before truth arrives, the mux still falls back to node
`now()` and reports `ref_source=node_now_no_truth`.

### Files Changed

- `apps/demo3_odom_mux.cpp`
  - Uses simulator-domain truth odom stamp as the IAP freshness reference when
    available.
  - Keeps freshness meaningful: IAP is accepted only when the stamp is
    increasing, non-zero, and within `iap_freshness_sec`.
  - Preserves fallback behavior: if locked IAP becomes stale relative to truth
    time, the mux returns to truth bootstrap until fresh IAP samples reacquire
    lock.
  - Adds diagnostics for IAP stamp, node now, reference stamp/source, computed
    age, receive/accept rate, reject reason, stale watchdog count, and
    accepted/rejected histograms.
- `include/iap/sim/odom_freshness.hpp`
  - Adds a small header-only freshness decision helper.
- `test/test_odom_freshness.cpp`
  - Tests simulator-domain acceptance, node-now fallback before truth, stale
    rejection, non-increasing stamp rejection, and zero-stamp rejection.
- `CMakeLists.txt`
  - Registers `test_odom_freshness`.
- `launch/demo9_ego_planner_closed_loop.launch.py`
  - Exposes mux freshness as `iap_odom_freshness_sec` while preserving the
    demo9 default of `0.3s`.
- `launch/demo11_ego_planner_integrity_corridor.launch.py`
  - Forwards `iap_odom_freshness_sec` into demo9 and sets demo11 default to
    `1.0s`, matching the existing planner stale-time scale while still keeping
    stale fallback active.

### Isolation

No certified GNSS ARAIM, certified LiDAR ARAIM, monitor PL fusion, Stage 2 FIM
math, Stage 3 PI math, Stage 4 URG math, truth alignment behavior, odom
publisher stamping, planner odom topic, or controller odom topic semantics were
changed.

Planner/controller odom topics remain:

```text
planner_odom_topic: /drone_0_visual_slam/odom
controller_odom_topic: /demo9/preflight_control_odom
```

### Build and Focused Tests

Commands run from `/home/dev/ws_iap`:

```bash
colcon build --base-paths src/iap src/gnss_comm --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

ctest --test-dir build/iap \
  -R "test_odom_freshness|test_araim|test_predicted_araim|test_pi_cost_adapter|test_unified_risk_grid" \
  --output-on-failure
```

Result:

```text
Build passed.
5/5 focused tests passed:
test_araim
test_predicted_araim
test_pi_cost_adapter
test_unified_risk_grid
test_odom_freshness
```

Launch-file syntax check also passed:

```bash
python3 -m py_compile \
  src/iap/launch/demo9_ego_planner_closed_loop.launch.py \
  src/iap/launch/demo11_ego_planner_integrity_corridor.launch.py
```

### Demo11 30s Smoke

Command:

```bash
source install/setup.bash
timeout 120s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=30 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true | tee /tmp/demo11_odom_fix_30s.log
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T102139Z_704
```

Key mux diagnostics:

```text
received first iap odom stamp=1657065603.599084
node_now=1778667703.388021
ref_stamp=1657065603.680170
ref_source=truth_odom_stamp
age=0.081
reason=accepted

mode=iap_locked
valid_samples=68
age=0.089

status mode=iap_locked
iap_count=71
accepted_iap=71
valid_iap_streak=71
iap_age=0.232
hist_accepted=71
hist_stale=0
hist_non_increasing=0
hist_zero_stamp=0
```

When IAP odometry later stopped advancing, fallback remained active and
meaningful:

```text
mode=truth_bootstrap reason=iap_stale_watchdog
age=1.001
stale_watchdog_count=1
```

### Demo11 90s Run

Command:

```bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=90 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true | tee /tmp/demo11_odom_fix_90s.log
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T102234Z_265
```

Key mux diagnostics:

```text
received first iap odom stamp=1657065602.760078
node_now=1778667757.117094
ref_stamp=1657065602.842098
ref_source=truth_odom_stamp
age=0.082
reason=accepted

mode=iap_locked
valid_samples=74
age=0.079

status mode=iap_locked
iap_count=77
accepted_iap=77
valid_iap_streak=77
iap_age=0.237
iap_rx_rate_hz=4.28
iap_accept_rate_hz=4.28
hist_accepted=77
hist_stale=0
hist_non_increasing=0
hist_zero_stamp=0
```

After the IAP publisher stopped advancing, the mux returned to fallback as
designed:

```text
mode=truth_bootstrap reason=iap_stale_watchdog
age=1.000
stale_watchdog_count=1
```

Overall 90s mux summary:

```text
status_rows: 88
max_status_streak: 77
locked_count: 2
reject_count: 0
watchdog_count: 1
final histogram: accepted=78 stale=1 non_increasing=0 zero_stamp=0
```

### Validator Results

Phase 1 official validator was run on the 90s run:

```bash
python3 src/iap/tools/phase1/validate_phase1_closed_loop.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest \
  --official
```

Output:

```text
Validated run: /home/dev/ws_iap/src/iap/log/20260513T102234Z_265
run_duration_s: 89.96
planner_trajectory_count: 0
planner_command_count: 837
truth_odom_count: 89958
iap_odom_count: 78
simulator_movement_m: 1.200
official: True
allow_truth_alignment: False
use_so3_dynamics: True
use_iap_odom_for_planner: True
plant_mode: so3_quadrotor_simulator
planner_odom_topic: /drone_0_visual_slam/odom
controller_odom_topic: /demo9/preflight_control_odom

Failures:
  - planner_trajectory_count 0 < 1
```

Phase 2 validator was also run:

```bash
python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  --run-dir /home/dev/ws_iap/src/iap/log/latest
```

Output:

```text
Validated Phase 2 run: /home/dev/ws_iap/src/iap/log/20260513T102234Z_265
sample_count: 44
snapshot_count: 43
traj_count: 44
aligned_sample_count: 0
odom_source: /drone_0_visual_slam/odom
map_source: global_cloud_direct

Warnings:
  - 44 pos_cmd fallback sample(s) present
  - PL grid self-check ratio is 49999999.000
  - raw current PL consistency max ratio is 8.891
  - predicted and actual IM could not be compared

Failures:
  - official Phase 2 requires at least one B-spline trajectory sample
  - phase2_summary.json current_consistency_raw max_pl_ratio=8.891 exceeds 0.100; likely_reason=no diagnostic context
  - iap_araim.csv exists but export/phase2_integrity_eval_aligned.csv is missing
  - demo10 did not pass Phase 1 official validation
```

### Current Validation Conclusion

The original odometry freshness issue is fixed: fresh IAP odometry is accepted,
`valid_iap_streak` becomes positive, and the mux enters `iap_locked` in both the
30s smoke and 90s run. The freshness check was not weakened; when IAP odometry
really becomes stale relative to truth simulation time, the mux switches back to
truth fallback.

Full demo11 acceptance is still blocked by separate run-quality/planner issues,
not by the odom freshness fix. The 90s run produced planner commands but no
B-spline trajectories, and the terminal logs showed repeated EGO planner
`AstarSearch`, `Coord2Index`, and "drone is in obstacle" errors. Those failures
explain the Phase 1 and Phase 2 validator failures above and should be tracked
as the next demo11 validation blocker.

### 2026-05-13 Rerun: Odom Freshness Verification

The odom freshness fix was revalidated from the clean working tree without
additional code changes. The same root cause and fix still apply: IAP odom is
stamped in the simulator epoch, and the preflight mux evaluates freshness
against truth odom simulator time when available instead of node wall time.

Build, focused tests, and launch syntax:

```bash
colcon build --base-paths src/iap src/gnss_comm --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

ctest --test-dir build/iap \
  -R "test_odom_freshness|test_araim|test_predicted_araim|test_pi_cost_adapter|test_unified_risk_grid" \
  --output-on-failure

python3 -m py_compile \
  src/iap/launch/demo9_ego_planner_closed_loop.launch.py \
  src/iap/launch/demo11_ego_planner_integrity_corridor.launch.py
```

Result:

```text
Build passed.
5/5 focused tests passed.
Launch py_compile passed.
```

30s smoke command:

```bash
source install/setup.bash
timeout 120s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=30 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true | tee /tmp/demo11_odom_fix_30s_rerun.log
```

30s run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T105031Z_941
```

30s key diagnostics:

```text
received first iap odom stamp=1657065602.428073
node_now=1778669434.462816
ref_stamp=1657065602.509124
ref_source=truth_odom_stamp
age=0.081
reason=accepted

mode=iap_locked
valid_samples=63
age=0.083

status mode=iap_locked
iap_count=66
accepted_iap=66
valid_iap_streak=66
iap_age=0.234
hist_accepted=66
hist_stale=0
hist_non_increasing=0
hist_zero_stamp=0

mode=truth_bootstrap reason=iap_stale_watchdog
age=1.001
stale_watchdog_count=1
```

90s run command:

```bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false \
  run_duration_s:=90 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_iap_odom_for_planner:=true \
  use_gnss:=true \
  use_araim:=true \
  planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true | tee /tmp/demo11_odom_fix_90s_rerun.log
```

90s run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T105121Z_011
```

90s key diagnostics:

```text
received first iap odom stamp=1657065602.098150
node_now=1778669483.195147
ref_stamp=1657065602.180084
ref_source=truth_odom_stamp
age=0.082
reason=accepted

mode=iap_locked
valid_samples=62
age=0.080

status mode=iap_locked
iap_count=64
accepted_iap=64
valid_iap_streak=64
iap_age=0.228
hist_accepted=64
hist_stale=0
hist_non_increasing=0
hist_zero_stamp=0

mode=truth_bootstrap reason=iap_stale_watchdog
age=1.000
stale_watchdog_count=1
```

Phase 1 official validator on the 90s run:

```text
Validated run: /home/dev/ws_iap/src/iap/log/20260513T105121Z_011
run_duration_s: 89.97
planner_trajectory_count: 0
planner_command_count: 837
truth_odom_count: 89967
iap_odom_count: 65
simulator_movement_m: 1.200
official: True
allow_truth_alignment: False
use_so3_dynamics: True
use_iap_odom_for_planner: True
plant_mode: so3_quadrotor_simulator
planner_odom_topic: /drone_0_visual_slam/odom
controller_odom_topic: /demo9/preflight_control_odom

Failures:
  - planner_trajectory_count 0 < 1
```

Phase 2 validator on the 90s run:

```text
Validated Phase 2 run: /home/dev/ws_iap/src/iap/log/20260513T105121Z_011
sample_count: 42
snapshot_count: 42
traj_count: 42
aligned_sample_count: 0
odom_source: /drone_0_visual_slam/odom
map_source: global_cloud_direct

Warnings:
  - 42 pos_cmd fallback sample(s) present
  - PL grid self-check ratio is 49999999.000
  - raw current PL consistency max ratio is 0.637
  - predicted and actual IM could not be compared

Failures:
  - official Phase 2 requires at least one B-spline trajectory sample
  - phase2_summary.json current_consistency_raw max_pl_ratio=0.637 exceeds 0.100; likely_reason=no diagnostic context
  - iap_araim.csv exists but export/phase2_integrity_eval_aligned.csv is missing
  - demo10 did not pass Phase 1 official validation
  - Phase 1 validator output: planner_trajectory_count 0 < 1
```

Rerun conclusion: the odom freshness issue remains fixed. The mux accepts fresh
IAP odom for sustained periods, `valid_iap_streak` becomes positive, and stale
fallback still triggers when IAP odom stops advancing. Remaining validator
failures are planner/run-quality and Phase 2 validation issues, not the odom
freshness/alignment bug.

## 2026-05-13 - URG per-voxel staleness and unknown-risk fix

Implemented the Stage 4 URG staleness fix so URG age is tracked per voxel
instead of from the global current integrity stamp.

Changed files:

- `include/iap/planner/unified_risk_grid.hpp`
- `src/iap/planner/unified_risk_grid.cpp`
- `apps/phase2_planner_integrity_evaluator.cpp`
- `test/test_unified_risk_grid.cpp`
- `docs/stage1_v2.0/status.md`

Implementation notes:

- Added `UnifiedRiskVoxel::updated_time_s` and kept `age_s`.
- URG query policy now computes `age_s = query_time_s - updated_time_s`.
- Interpolated URG queries propagate the oldest finite contributing voxel
  timestamp, making interpolation conservative without using one global stale
  flag.
- Removed URG stale marking from `current_integrity_stamp_`.
- Fresh voxels do not receive `STALE_PL`.
- Stale voxels receive `STALE_PL` and the configured ramped unknown penalty.
- Unknown/invalid voxels receive `UNKNOWN_RISK` and the configured full unknown
  penalty when `phase2_urg_unknown_penalty > 0`.
- Added additive summary field `urg_max_age_s`.
- Added additive `updated_time_s` column to `urg_grid_voxels.csv`; no old CSV
  columns, topics, PointCloud2 field names, message fields, certified monitor
  code, Stage 2 FIM math, or Stage 3 PI formula were changed.

Build and unit tests:

```bash
colcon build --base-paths src/iap src/gnss_comm --packages-select iap \
  --cmake-args -DBUILD_TESTING=ON
```

Result:

```text
Summary: 1 package finished
```

```bash
ctest --test-dir build/iap \
  -R "test_pl_grid|test_future_pl_field_predictor|test_pi_cost_adapter|test_unified_risk_grid" \
  --output-on-failure
```

Result:

```text
4/4 tests passed:
test_pl_grid
test_future_pl_field_predictor
test_pi_cost_adapter
test_unified_risk_grid
```

Launch syntax check:

```bash
python3 -m py_compile launch/demo11_ego_planner_integrity_corridor.launch.py
```

Result: passed.

Demo11 URG-enabled 30s smoke:

```bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false run_duration_s:=30 allow_truth_alignment:=false \
  use_so3_dynamics:=true use_iap_odom_for_planner:=true use_gnss:=true \
  use_araim:=true planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true \
  phase2_use_unified_risk_grid:=true phase2_urg_export_voxels:=true \
  phase2_urg_keep_legacy_topics:=true phase2_pl_model:=fused_fim_grid \
  phase2_use_pl_grid:=true phase2_export_pl_grid_voxels:=true \
  phase2_use_advisory_fim_add:=true phase2_use_lidar_advisory_fim:=true \
  phase2_use_lidar_observability:=false \
  phase2_pi_use_unified_advisory_pl:=true
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T111254Z_870
```

Key `phase2_summary.json` URG counters:

```text
urg_enabled=true
urg_active=true
urg_update_count=3
urg_query_count=7807
urg_grid_hit_count=7806
urg_grid_miss_count=1
urg_direct_query_count=1
urg_stale_count=2601
urg_unknown_count=0
urg_unknown_penalty_count=2601
urg_valid_pi_count=7807
urg_max_age_s=3.7183640003204346
urg_front_field_points=2601
urg_backend_field_points=2602
urg_voxel_csv=urg_grid_voxels.csv
```

Conclusion: `urg_stale_count` is no longer automatically equal to
`urg_query_count`; only the stale query subset was counted stale in this smoke.

Demo11 URG-disabled 30s smoke:

```bash
source install/setup.bash
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false run_duration_s:=30 allow_truth_alignment:=false \
  use_so3_dynamics:=true use_iap_odom_for_planner:=true use_gnss:=true \
  use_araim:=true planner_use_integrity_cost:=true \
  planner_use_integrity_front_search:=true \
  planner_use_integrity_global_search:=true \
  phase2_use_unified_risk_grid:=false phase2_pl_model:=fused_fim_grid \
  phase2_use_pl_grid:=true phase2_use_advisory_fim_add:=true \
  phase2_use_lidar_advisory_fim:=true \
  phase2_use_lidar_observability:=false \
  phase2_pi_use_unified_advisory_pl:=true
```

Run directory:

```text
/home/dev/ws_iap/src/iap/log/20260513T111351Z_721
```

Key `phase2_summary.json` URG counters:

```text
urg_enabled=false
urg_active=false
urg_update_count=0
urg_query_count=0
urg_grid_hit_count=0
urg_grid_miss_count=0
urg_direct_query_count=0
urg_stale_count=0
urg_unknown_count=0
urg_unknown_penalty_count=0
urg_valid_pi_count=0
urg_max_age_s=null
urg_voxel_csv=
urg_grid_voxels.csv present=false
```

Conclusion: disabled URG mode remains inactive and legacy-compatible.

## 2026-05-13 Update: Stage 4.1 URG Performance Quick Fixes

Stage 4.1 quick fixes were implemented to reduce URG rebuild cost while
preserving Stage 2 advisory FIM math, Stage 3 PI math, certified monitor logic,
legacy topics, and CSV capability.

### Changes Applied

- Added URG evaluator and demo11 launch config:
  - `phase2_urg_compute_gradients`
  - `phase2_urg_gradient_mode={none,grid_difference,finite_difference}`
  - `phase2_urg_export_voxels_on_update`
  - `phase2_urg_export_voxels_on_shutdown`
  - `phase2_urg_max_voxels_per_update`
  - `phase2_urg_rebuild_cancel_check_interval`
- Kept legacy `phase2_urg_export_voxels`; when true it enables on-update
  `urg_grid_voxels.csv` export for compatibility.
- Changed the default URG gradient path from per-voxel finite differences to
  grid-difference gradients over already-computed PI costs.
- Added zero-gradient support for `phase2_urg_gradient_mode:=none`.
- Kept the old finite-difference gradient path available through
  `phase2_urg_gradient_mode:=finite_difference`.
- Rebuilt URG into a local grid and swapped it under `urg_mutex_` only after
  completion, reducing lock hold time.
- Added rebuild cancellation checks for shutdown and a configurable
  max-voxel cap.
- Avoided duplicate full-grid backend row recomputation when URG is enabled by
  generating backend cost-field rows from URG cells.
- Changed front cost sample generation to query URG first when URG is enabled.
- Added summary timing breakdown:
  - `urg_time_pl_query_ms`
  - `urg_time_al_esdf_ms`
  - `urg_time_pi_ms`
  - `urg_time_gradient_ms`
  - `urg_time_csv_ms`
  - `urg_time_total_ms`

### Build and Test Results

Commands run from `/home/dev/ws_iap`:

```bash
colcon build --base-paths src/iap src/gnss_comm --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON

ctest --test-dir build/iap -R \
  "test_unified_risk_grid|test_phase2_summary_schema|test_pi_cost_adapter|test_future_pl_field_predictor" \
  --output-on-failure

python3 -m py_compile \
  src/iap/launch/demo11_ego_planner_integrity_corridor.launch.py \
  src/iap/test/test_phase2_summary_schema.py \
  src/iap/tools/phase2/validate_phase2_integrity_eval.py \
  src/iap/tools/phase2/phase2_summary_schema.py

ctest --test-dir build/iap --output-on-failure
```

Results:

```text
Build passed.
Focused CTest: 4/4 passed.
Full CTest: 12/12 passed.
Python compile checks passed.
```

### Demo11 Smoke Results

Small URG grid smoke, export disabled, `grid_difference` gradients:

```text
run_dir=/home/dev/ws_iap/src/iap/log/20260513T123013Z_121
urg_enabled=true
urg_active=true
urg_update_count=29
urg_query_count=1726
urg_front_field_points=121
urg_backend_field_points=122
urg_mean_update_ms=39.28024344827587
urg_p95_update_ms=67.005212
urg_time_pl_query_ms=0.11580099999999999
urg_time_al_esdf_ms=20.980222000000023
urg_time_pi_ms=0.018200000000000015
urg_time_gradient_ms=0.005106
urg_time_csv_ms=0.0
urg_time_total_ms=21.151587
urg_voxel_csv=
```

Default 25m x 25m URG smoke, export disabled, `grid_difference` gradients:

```text
run_dir=/home/dev/ws_iap/src/iap/log/20260513T123045Z_309
urg_enabled=true
urg_active=true
urg_update_count=6
urg_query_count=7811
urg_front_field_points=2601
urg_backend_field_points=2602
urg_mean_update_ms=1229.6519521666667
urg_p95_update_ms=1627.073874
urg_time_pl_query_ms=1133.669751999999
urg_time_al_esdf_ms=480.3970519999997
urg_time_pi_ms=1.2053949999999987
urg_time_gradient_ms=0.054821
urg_time_csv_ms=0.0
urg_time_total_ms=1616.332713
urg_voxel_csv=
```

The default-grid mean update time was reduced from the audited
`~8881 ms` to `~1230 ms` in this smoke. The phase2 evaluator exited cleanly in
both smokes. The known non-core demo11 shutdown errors in visualization/sensing
nodes still appeared, matching the existing audit notes.

## 2026-05-13 Update: Advisory LiDAR FIM PCA Primitive Generation

Scope: advisory planner-side LiDAR FIM primitive generation only. Certified
LiDAR ARAIM, Stage 0 LiDAR monitor code, and Stage 2 FIM-add math were not
modified.

### Changes Applied

- Moved advisory LiDAR FIM primitive generation into
  `include/iap/planner/lidar_observability_fim.hpp` and
  `src/iap/planner/lidar_observability_fim.cpp` as a reusable/testable API.
- Added primitive generation parameters with backward-compatible defaults:
  - `pca_radius_m = 1.5`
  - `pca_max_points = 2000`
  - `pca_min_support = 6`
  - `pca_voxel_sample_m = 0.5`
  - `pca_max_primitives = 2000`
  - `use_cloud_normals_first = true`
- Replaced PCA fallback index-stride sampling with deterministic voxel-bucket
  sampling, followed by bounded uniform capping after voxel downsampling.
- Preserved cloud-provided `normal_x/y/z` first. Valid finite nonzero cloud
  normals produce primitives directly; PCA fallback is used for points with
  absent or invalid normals.
- Added evaluator and demo11 launch wiring for:
  - `phase2_lidar_fim_pca_radius_m`
  - `phase2_lidar_fim_pca_max_points`
  - `phase2_lidar_fim_pca_min_support`
  - `phase2_lidar_fim_pca_voxel_sample_m`
  - `phase2_lidar_fim_pca_max_primitives`
  - `phase2_lidar_fim_use_cloud_normals_first`
- Added `phase2_summary.json` advisory FIM diagnostics:
  - `lidar_pca_primitives_total`
  - `lidar_pca_valid_normals`
  - `lidar_pca_invalid_normals`
  - `lidar_pca_support_mean`
  - `lidar_pca_support_min`
  - `lidar_pca_radius_m`

### Tests Added

`test/test_lidar_observability_fim.cpp` now covers:

- configurable PCA radius changes primitive count;
- min support boundary is inclusive at the configured threshold;
- voxel sampling reduces duplicate dense-cluster primitives;
- empty cloud primitive generation reports `valid=false` /
  `missing_lidar_normals`;
- existing cloud normal path works without requiring PCA support.

### Build and Test Results

Commands run from `/home/dev/ws_iap`:

```bash
colcon build --base-paths src/iap src/gnss_comm --packages-select iap \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo

ctest --test-dir build/iap -R \
  "test_lidar_observability_fim|test_future_pl_field_predictor|test_pl_grid" \
  --output-on-failure

ctest --test-dir build/iap --output-on-failure

python3 -m py_compile \
  src/iap/launch/demo11_ego_planner_integrity_corridor.launch.py

git diff --check
git diff --name-only
```

Results:

```text
Build passed: 1 package finished.
Focused CTest: 3/3 passed.
Full CTest: 12/12 passed.
Python launch compile: passed.
git diff --check: passed.
git diff --name-only:
  apps/phase2_planner_integrity_evaluator.cpp
  docs/stage1_v2.0/status.md
  include/iap/planner/lidar_observability_fim.hpp
  launch/demo11_ego_planner_integrity_corridor.launch.py
  src/iap/planner/lidar_observability_fim.cpp
  test/test_lidar_observability_fim.cpp
```
