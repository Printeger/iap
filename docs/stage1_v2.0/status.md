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
