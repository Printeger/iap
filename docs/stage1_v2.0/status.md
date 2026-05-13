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
