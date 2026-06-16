# Predictor Module Experiment Test Plan

## Summary

This plan validates the refactored `PredictorModule` as an independent advisory query module. The acceptance target is limited to predictor advisory output correctness, stability, timing, and explainability for GNSS, LiDAR, and FIM fusion.

Out of scope for this test plan: AL/IM/PI cost, URG, PointCloud2 cost fields, A*, B-spline optimization, and backend planner success. `/iap/integrity` is used only as a current-state input for `IntegritySnapshot`; it is not the predictor acceptance output.

The implemented test harness is:

- `src/iap/launch/test_predictor.launch.py`
- `src/iap/apps/test_predictor_query_probe.cpp`
- `src/iap/scripts/test_predictor_validator.py`

The probe writes predictor-owned CSV and JSON summary files, so test conclusions do not depend on planner or monitor fusion outputs.

Debug logging follows the IAP run-log layout. `test_predictor.launch.py` owns the predictor run directory and does not initialize another `RunLogManager` inside the probe, avoiding conflicts with the `iap_rosnode` run in the same launch.

## Test Architecture

`test_predictor.launch.py` wraps the existing simulation chain from `test_araim.launch.py` with `run_validator:=false`, then starts:

- `test_predictor_query_probe`: subscribes odom, map cloud, GNSS range/ephem, and `/iap/integrity`; builds `IntegritySnapshot`; calls `PredictorModule::query()`.
- `test_predictor_validator.py`: validates the probe CSV/summary after the configured duration.

The probe supports:

- `predictor_output_mode:=gnss_only`: final selected output comes from `GnssAdvisoryResult`.
- `predictor_output_mode:=lidar_only`: final selected output is LiDAR-only FIM fusion using an empty GNSS result.
- `predictor_output_mode:=fusion`: final selected output comes from `PredictorModule` FIM fusion.

The probe records per-query timing:

- `gnss_us`
- `lidar_us`
- `fusion_us`
- `total_step_us`
- `module_total_us`

It also records source flags, fallback reasons, matrix diagnostics, valid/fallback state, and `lambda_sum_error`.

## Debug Log Layout

Debug output is controlled by:

```bash
ros2 launch iap test_predictor.launch.py \
  experiment:=predictor_fusion_nominal \
  predictor_enable_debug_log:=true \
  start_rviz:=false \
  record_bag:=false
```

When `predictor_export_dir` is empty, the launch creates:

```text
<log_root>/<timestamp>_test_predictor/
  runtime/
  export/
  profiling/
  metadata/
```

`log_root` comes from `predictor_log_root` if provided. Otherwise it is read from `config/<config_subdir>/config.json` under `logging.log_dir`; if that cannot be read, it falls back to `/tmp/iap_predictor_log`.

If `predictor_export_dir` is explicitly provided, the main output files are kept in that directory for compatibility, while `runtime/`, `profiling/`, and `metadata/` are created beside it.

Always written:

- `export/test_predictor_query_probe.csv`: one row per predictor query; acceptance data for source, valid/fallback, HPL/VPL/PL, flags, fallback reason, FIM diagnostics, timing input.
- `export/test_predictor_query_probe_summary.json`: query count, valid/fallback ratio, source counts, source flag counts, fallback histograms, latency p50/p95/max.
- `export/test_predictor_validation_summary.json`: validator pass/fail, failures, source/valid observations, debug file status.
- `metadata/predictor_launch_config.json`: launch preset, mode, debug switch, run directories, output paths, validator settings.
- `metadata/predictor_probe_config.json`: probe topics, output mode, predictor params, debug switch, output paths.

Written only when `predictor_enable_debug_log:=true`:

- `export/predictor_query_debug.csv`: one row per query; query pose/time, snapshot availability, decoded source flags, selected result, GNSS/LiDAR/fusion status, fallback reason chain.
- `export/predictor_gnss_epoch_debug.csv`: one row per query-satellite; sat id, constellation, elevation, azimuth, pseudorange sigma, excluded flag, epoch satellite count, GNSS visible/used count, GNSS fallback reason.
- `export/predictor_lidar_debug.csv`: one row per query; cloud point count, PCA primitive diagnostics, valid normals, LiDAR FIM trace/eigen/condition, regularization and fallback reason.
- `export/predictor_lidar_primitives_debug.csv`: sampled primitives per cloud update; center, normal, weight, normal confidence, support count. Sampling is limited by `predictor_debug_max_lidar_primitives`, default `500`.
- `export/predictor_fusion_debug.csv`: one row per query; prior/GNSS/LiDAR/predicted lambda matrices, covariance matrix, source-used flags, regularization flags, conservative max flag, `lambda_sum_error`.
- `profiling/predictor_timing.csv`: one row per query; `gnss_us`, `lidar_us`, `fusion_us`, `total_step_us`, `module_total_us`.

Debug files are for failure analysis and figure/table reproduction. They are not required for normal CI unless `validator_require_debug_logs:=true`.

## Isolated Test Coverage

Run existing GTest coverage first:

```bash
colcon test --packages-select iap --ctest-args -R "test_predictor_module|test_lidar_observability_fim|test_predicted_araim"
```

Required isolated behaviors:

- GNSS predictor:
  - Open-sky multi-satellite input returns finite HPL/VPL and valid 3x3 position FIM.
  - Missing epoch, too few satellites, and excluded satellites return explicit fallback reasons.
  - Map occlusion/skymask changes visible satellite count and degrades PDOP/HPL/VPL.
  - Clock Schur complement output is symmetric, finite, and PSD or explicitly rejected.
- LiDAR predictor:
  - Feature-rich 3D primitives return valid FIM with nonzero primitive count.
  - Sparse, corridor, or planar-degenerate geometry returns fallback or regularization diagnostics.
  - Missing normals/primitives returns an explicit fallback reason.
  - `lidar_enable_legacy_observability:=false` prevents legacy observability from masking FIM failure.
- Fusion predictor:
  - GNSS-only, LiDAR-only, and fused outputs are separately explainable.
  - `lambda_pred = lambda_prior + lambda_gnss + lambda_lidar`.
  - Missing/invalid prior does not block valid GNSS or LiDAR advisory sources.
  - No advisory source returns unavailable with a combined fallback reason.

## System Experiments

The latest system-test target is to show that Predictor is connected to odometry, current ARAIM, GNSS, and LiDAR upstream sources and produces stable advisory query outputs. A passed system experiment does **not** prove that the planner avoids low-integrity regions.

Run each system experiment with the same launch shape:

```bash
ros2 launch iap test_predictor.launch.py \
  experiment:=<experiment_name> \
  start_rviz:=false \
  record_bag:=false \
  predictor_export_dir:=<output_dir>
```

Recommended runtime policy:

- Run every experiment for at least `60 s`.
- Repeat each experiment at least twice with fixed map/GNSS seeds.
- For final report evidence, rerun the selected experiment once with `record_bag:=true`.

Recommended per-experiment output layout:

```text
predictor_system_<experiment_name>/
  run_manifest.json
  test_predictor_query_probe.csv
  test_predictor_query_probe_summary.json
  test_predictor_validation_summary.json
  params/
    launch_args.yaml
    predictor_params.yaml
    araim_sim_preset.yaml
    validator_params.yaml
  debug/
    snapshot_debug.csv
    gnss_epoch_debug.csv
    gnss_visibility_by_query.csv
    lidar_primitives_debug.csv
    lidar_fim_debug.csv
    fusion_matrix_debug.csv
    source_selection_debug.csv
    fallback_reason_by_time.csv
    latency_debug.csv
  figures/
    01_selected_pl_timeline.png
    02_source_validity_timeline.png
    03_source_selection_timeline.png
    04_source_selection_histogram.png
    05_fallback_reason_histogram.png
    06_gnss_geometry_timeline.png
    07_lidar_diagnostics_timeline.png
    08_lambda_contribution_timeline.png
    09_current_vs_advisory.png
    10_latency_distribution.png
    11_spatial_query_map.png
  optional_bag/
    replay.bag
```

If an experiment does not enable GNSS, LiDAR, or Fusion, keep the corresponding figure slot and mark it `not enabled in this run` so all reports have the same structure.

### Required System CSV Fields

The probe already records timing, source flags, fallback reasons, matrix diagnostics, valid/fallback state, and `lambda_sum_error`. System experiments should standardize or add the following fields.

Time and snapshot freshness:

- `stamp_s`, `query_id`, `query_time_s`, `horizon_s`
- `odom_stamp_s`, `integrity_stamp_s`, `gnss_epoch_stamp_s`, `snapshot_stamp_s`
- `odom_age_s`, `integrity_age_s`, `gnss_age_s`, `snapshot_age_s`

Query spatial fields:

- `query_x`, `query_y`, `query_z`
- `query_offset_x`, `query_offset_y`, `query_offset_z`
- `query_label`
- `scenario_region_label`: `open_sky`, `canopy`, `corridor`, `outage`, `lidar_sparse`, `fallback_only`, or `unknown`

Current ARAIM input fields, used only as snapshot context:

- `current_hpl`, `current_vpl`, `current_pl_e`, `current_pl_n`, `current_pl_u`
- `current_hal`, `current_val`, `current_im`, `current_state`
- `current_n_sv_used`, `current_pdop`, `current_n_hypotheses`, `current_n_detected`
- `current_excluded_prns`

GNSS advisory fields:

- `gnss_valid`, `gnss_available`, `gnss_fallback`, `gnss_fallback_reason`
- `gnss_n_visible`, `gnss_n_used`, `gnss_n_excluded`
- `gnss_pdop`, `gnss_hdop`, `gnss_vdop`, `gnss_sigma_h`, `gnss_sigma_v`
- `gnss_hpl`, `gnss_vpl`, `gnss_pl_e`, `gnss_pl_n`, `gnss_pl_u`
- `gnss_lambda_trace`, `gnss_lambda_min_eig`, `gnss_lambda_condition`
- `sat_visible_mask`, `sat_used_mask`, `excluded_prn_mask`
- `effective_sigma_mean`, `effective_sigma_max`

LiDAR advisory diagnostic fields:

- `lidar_valid`, `lidar_available`, `lidar_fallback`, `lidar_fallback_reason`
- `lidar_n_primitives`, `lidar_alpha`, `lidar_tdop`, `lidar_condition`
- `lidar_lambda_trace`, `lidar_lambda_min_eig`, `lidar_lambda_max_eig`
- `lidar_degeneracy_score`, `lidar_allowed_for_fusion`, `lidar_fusion_gate_reason`
- `normal_diversity_score`

Fusion and selected-output fields:

- `fusion_mode`, `selected_source`, `selected_valid`, `selected_available`, `selected_fallback`, `selected_fallback_reason`
- `fused_valid`, `fused_available`, `fused_hpl`, `fused_vpl`, `fused_pl_e`, `fused_pl_n`, `fused_pl_u`
- `lambda_prior_trace`, `lambda_gnss_trace`, `lambda_lidar_trace`
- `lambda_fused_trace`, `lambda_fused_min_eig`, `lambda_fused_condition`
- `lambda_sum_error`

Latency fields:

- `gnss_us`, `lidar_us`, `fusion_us`, `total_step_us`, `module_total_us`

### Implemented System Presets

`predictor_gnss_open_sky_only`

- ARAIM sim preset: `gnss_open_sky`
- Output mode: `gnss_only`
- Expected selected source: `GNSS`
- Required data: probe CSV/summary, `gnss_epoch_debug.csv`, `gnss_visibility_by_query.csv`, `source_selection_debug.csv`, `latency_debug.csv`, `run_manifest.json`
- Required figures: selected PL timeline, GNSS geometry timeline, current-vs-advisory, latency distribution, query spatial map
- Pass targets: `selected_source_GNSS_ratio > 95%`, `gnss_valid_ratio > 95%`, `fallback_ratio < 5%`, `p95 module_total_us < 2 ms`, no NaN/Inf in valid rows

`predictor_lidar_feature_rich_only`

- ARAIM sim preset: `lidar_feature_rich`
- Output mode: `lidar_only`
- Expected selected source: `LIDAR`
- Required data: `lidar_primitives_debug.csv`, `lidar_fim_debug.csv`, `source_selection_debug.csv`, `fallback_reason_by_time.csv`, and `local_map_snapshot.pcd` or `downsampled_map.csv`
- `lidar_primitives_debug.csv` must include primitive id, position, normal, confidence, support count, and map age
- Required figures: LiDAR diagnostics timeline, LiDAR eigenvalues timeline, primitives top-down view, normal distribution, selected PL timeline
- Pass targets: `selected_source_LIDAR_ratio > 80%`, `lidar_valid_ratio > 80%`, median primitive count above configured minimum, median condition below threshold, no NaN/Inf
- This experiment proves LiDAR diagnostic and LiDAR-only query availability; it does not prove LiDAR fusion safety.

`predictor_fusion_nominal`

- ARAIM sim preset: `fused_nominal`
- Output mode: `fusion`
- Expected selected source: `FUSION`
- Required data: `fusion_matrix_debug.csv`, `source_selection_debug.csv`, `gnss_visibility_by_query.csv`, `lidar_fim_debug.csv`, `latency_debug.csv`
- `fusion_matrix_debug.csv` should flatten 3x3 `lambda_prior`, `lambda_gnss`, `lambda_lidar`, `lambda_fused`, and include `lambda_error_norm`
- Required figures: source HPL/VPL timeline, lambda contribution timeline, lambda sum error timeline, lambda condition timeline, source selection histogram, latency distribution
- Pass targets: `selected_source_FUSION_ratio > 80%`, `lambda_sum_error p95 <= 1e-6`, `lambda_fused_min_eig > -1e-9`, `fallback_ratio < 10%`, no NaN/Inf

`predictor_gnss_degraded_lidar_good`

- ARAIM sim preset: `gnss_degraded_lidar_good`
- Output mode: `fusion`
- Expected selected source: `FUSION` when active LiDAR fusion is enabled and gated; otherwise GNSS may remain selected while LiDAR is logged as diagnostic-only
- Required data: `gnss_visibility_by_query.csv`, `satellite_used_mask.csv`, `lidar_fim_debug.csv`, `fusion_matrix_debug.csv`, `source_selection_debug.csv`, `scenario_region_labels.csv`
- Required figures: GNSS degradation timeline, LiDAR support timeline, GNSS-vs-fusion HPL/VPL, lambda contribution timeline, spatial map colored by GNSS HPL, spatial map colored by selected HPL
- Pass targets: degraded-region GNSS HPL and PDOP medians exceed open-region medians, `lidar_valid_ratio > 70%`, `selected_source_FUSION_ratio > 70%` when fusion is enabled, fallback reasons are non-empty and explainable

`predictor_lidar_sparse_gnss_good`

- ARAIM sim preset: `lidar_degraded_gnss_good`
- Output mode: `fusion`
- Expected selected source: `GNSS` or `FUSION`, depending on LiDAR fusion gate. Do not require fixed `FUSION` for unsafe degraded LiDAR.
- Required data: `lidar_primitives_debug.csv`, `lidar_fim_debug.csv`, `lidar_gate_debug.csv`, `gnss_visibility_by_query.csv`, `source_selection_debug.csv`, `fusion_matrix_debug.csv`, `fallback_reason_by_time.csv`
- `lidar_gate_debug.csv` must include `lidar_condition`, `lidar_min_eig`, `lidar_degeneracy_score`, `lidar_allowed_for_fusion`, and `lidar_fusion_gate_reason`
- Required figures: LiDAR degradation timeline, GNSS stability timeline, fusion gate timeline, selected-vs-GNSS PL, LiDAR primitives top-down, LiDAR eigenvalues timeline
- Pass targets: GNSS HPL remains stable, LiDAR degradation indicator triggers, selected output must not become more optimistic than GNSS-only because of degraded LiDAR, no NaN/Inf

`predictor_gnss_outage_lidar_recovery`

- ARAIM sim preset: `gnss_outage_lidar_recovery`
- Output mode: `fusion`
- Expected selected source: `FUSION`, `LIDAR`, or `INVALID`, depending on active policy
- Recommended outage window: `0-20 s` normal, `20-40 s` outage, `40-60 s` recovery
- Required data: `outage_event_times.json`, `fallback_reason_by_time.csv`, `source_selection_debug.csv`, `gnss_visibility_by_query.csv`, `lidar_fim_debug.csv`, `recovery_metrics.json`
- `recovery_metrics.json` must include outage start/end, predictor invalid start, predictor valid recovery, recovery delay, outage valid ratio, and fallback reason histogram
- Required figures: outage window timeline, fallback reason timeline, source selection timeline, recovery latency, GNSS visible-count timeline, selected PL timeline
- Current conservative Stage-1 expectation: during GNSS outage, `gnss_valid=false`, `selected_valid=false`, `selected_source=INVALID`, and fallback reason includes `no_gnss_epoch` or `too_few_sats`, unless LiDAR recovery gate has passed corridor-specific validation.

`predictor_no_source_negative`

- ARAIM sim preset: `fallback_only`
- Output mode: `fusion`
- LiDAR primitives disabled in the probe
- Expected result: selected invalid or fallback, explicit reason
- Required data: `negative_case_rows.csv`, `fallback_reason_by_time.csv`, `source_selection_debug.csv`, `snapshot_debug.csv`, `current_vs_advisory_debug.csv`, `invalid_output_audit.json`
- `invalid_output_audit.json` must include valid row count, finite selected PL count, copied current flag count, fallback reason empty count
- Required figures: valid/fallback flags, fallback reason histogram, selected PL timeline, current-vs-selected PL, source selection timeline
- Pass targets: `selected_valid_ratio = 0`, `fallback_reason_empty_count = 0`, `finite_selected_pl_count = 0` for invalid rows, `copied_current_flag_count = 0`

### Recommended New System Experiments

The latest system-test plan recommends adding five experiments beyond the seven implemented presets:

- `predictor_current_advisory_separation`: inject normal, high-current-HPL/VPL, and unsafe current integrity variants; verify selected advisory follows GNSS geometry, not artificial current PL.
- `predictor_gnss_sigma_degradation`: sweep sigma scale `1x -> 2x -> 4x`; verify `effective_sigma_mean/max` rise, `lambda_gnss_trace` falls, and GNSS HPL/VPL rise.
- `predictor_corridor_lidar_degeneracy`: use `lidar_corridor_degenerate`; verify along-corridor information is weak, condition and degeneracy score rise, and LiDAR is either gated out or gives explicit fallback.
- `predictor_stale_snapshot_guard`: freeze/delay odom, GNSS, or `/iap/integrity`; verify stale age beyond threshold produces invalid/fallback output with explicit stale-source reason.
- `predictor_query_latency_stress`: run fused nominal with query batch sizes `1`, `10`, `50`, and `100`; verify p50/p95/p99 latency scales controllably and sources do not time out.

Recommended risk-prioritized execution order:

1. `predictor_gnss_open_sky_only`
2. `predictor_no_source_negative`
3. `predictor_current_advisory_separation`
4. `predictor_gnss_sigma_degradation`
5. `predictor_gnss_degraded_lidar_good`
6. `predictor_lidar_sparse_gnss_good`
7. `predictor_corridor_lidar_degeneracy`
8. `predictor_fusion_nominal`
9. `predictor_lidar_feature_rich_only`
10. `predictor_gnss_outage_lidar_recovery`
11. `predictor_stale_snapshot_guard`
12. `predictor_query_latency_stress`

If time is limited, `predictor_lidar_feature_rich_only` and `predictor_query_latency_stress` can move later. Highest-priority experiments are GNSS open-sky, no-source negative, GNSS degraded/LiDAR good, LiDAR sparse/GNSS good, and corridor LiDAR degeneracy.

Outputs are written under `predictor_export_dir` if provided. If it is omitted, the current launch creates the run directory under the configured IAP log root as described in the Debug Log Layout section.

## Acceptance Criteria

Each run must produce the core probe outputs:

- `test_predictor_query_probe.csv`
- `test_predictor_query_probe_summary.json`
- `test_predictor_validation_summary.json`
- `metadata/predictor_launch_config.json`
- `metadata/predictor_probe_config.json`

System-test validation passes when:

- Query count is at least `validator_min_queries`.
- Required source validity is observed for the selected experiment.
- Valid selected rows have finite HPL/VPL/PL and finite FIM diagnostics.
- NaN/Inf count is zero for all valid rows.
- Fallback rows have non-empty fallback reasons.
- Source flags are internally consistent.
- Selected source matches the experiment policy; degraded-LiDAR experiments may allow `GNSS` or `FUSION` depending on the LiDAR fusion gate.
- `lambda_sum_error <= validator_max_lambda_sum_error` for fused rows; nominal target is `p95 <= 1e-6`.
- Summary includes valid ratio, fallback ratio, source counts, source flag counts, fallback reason histogram, and latency p50/p95/max.
- When `validator_require_debug_logs:=true`, all configured debug CSV/metadata files exist and are non-empty.

Initial latency thresholds:

- GNSS-only: `p95 module_total_us < 2000 us`
- LiDAR-only: `p95 module_total_us < 10000 us`
- Fusion: `p95 module_total_us < 15000 us`

Safety fallback criteria for no-source, no-GNSS, and too-few-satellite cases:

- `selected_valid == false` or `selected_fallback == true`
- `fallback_reason != ""`
- `selected_source` is not silent `FUSION`

Current/advisory separation criterion:

- `copied_current_flag ratio < 5%`
- Define `copied_current_flag` as `abs(selected_hpl - current_hpl) < eps` and `abs(selected_vpl - current_vpl) < eps` when output mode is not an explicit constant-current mode.

For stability checks, rerun an experiment twice with fixed map/GNSS seeds and compare:

- valid ratio
- selected source counts
- fallback reason histogram
- HPL/VPL median values
- latency p95 values

Use a 5% median HPL/VPL tolerance as the initial baseline gate.

## Notes

The system launch still starts the simulation stack used by `test_araim.launch.py`, but predictor validation is based only on the probe CSV and summary. Planner behavior and trajectory success must not be used to pass or fail predictor experiments.

Use `config_subdir:=sim_demo11` by default so the predictor launch and included ARAIM launch resolve the same simulation config and logging root. Override `config_subdir` only when testing a different simulation configuration.
