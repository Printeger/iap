# IAP Implementation Audit: Stages 0-4

**Audit date**: 2026-05-13
**Repository**: `/home/dev/ws_iap/src/iap`, branch `dev/iap`
**Reference spec**: `docs/stage1_v2.0/iap_codex_implementation_plan_v2_3.tex` (v2.3)
**Status source**: `docs/stage1_v2.0/status.md`

---

## 1. Executive Verdict

### Overall Status: **PARTIAL**

Significant implementation work across Stages 0-4 is complete and code-level tested. However, no stage has passed full official demo11 acceptance due to a chain of blockers: Phase 2 validator schema mismatch, old validator `fim_add` assumptions, odometry freshness issues, URG performance, and shutdown behavior.

### Per-Stage Status

| Stage | Status |
|-------|--------|
| Stage 0 (LiDAR PL Stability) | **Code-level tested, demo11 smoke-tested. Full demo11 acceptance: NOT YET** |
| Stage 1 (Naming Clarification) | **Code-level tested, demo11 smoke-tested. Full demo11 acceptance: NOT YET** |
| Stage 2 (Advisory FIM Predictor) | **Code-level tested, demo11 smoke-tested with FIM-add enabled. Full demo11 acceptance: NOT YET** |
| Stage 3 (Unified PI Cost Adapter) | **Code-level tested, demo11 smoke-tested with Stage 2+3 enabled. Full demo11 acceptance: NOT YET** |
| Stage 4 (Unified Risk Grid) | **Code-level tested, demo11 smoke-tested with Stage 2+3+4 enabled and disabled modes. Full demo11 acceptance: NOT YET** |

The chain of blockers preventing full acceptance is shared across all stages:
1. Phase 2 validator not updated for `fim_add` semantics
2. `phase2_summary.json` schema mismatch between online evaluator and `ana_log.py`
3. IAP odometry freshness/staleness in demo11
4. URG rebuild performance (mean ~8881 ms for 25m x 25m grid)
5. demo11 shutdown errors in non-core visualization nodes

---

## 2. Audit Matrix: Per-Stage Requirements vs Implementation

### Stage 0: LiDAR ARAIM PL Stability Repair

| Spec Requirement | Implementation Location | Status | Evidence | Risk / Follow-up |
|-----------------|------------------------|--------|----------|-----------------|
| Bounded LiDAR age risk | `lidar_araim.cpp:230-237` | **PASS** | Both `EXP_SATURATING` (`1-exp(-A/tau)`) and `LINEAR_CAPPED` (`min(A/A_ref, max)`) implemented, bounded by `gamma_age_max` | None |
| Target keyframe/window cap | `lidar_araim.cpp:43-108` | **PASS** | `target_window_K=10`, `max(1,K)`, sorted by distance/age | Window omission is safety-sensitive; explicitly documented in status.md |
| PL component logging | `lidar_araim_debug.hpp:55-61` | **PASS** | CSV with `sep_term_m`, `sigma_ss_term_m`, `sigma_subset_term_m`, `bias_term_m` | Only logs worst subset per axis; secondary high-PL subsets lost |
| Gamma component logging | `lidar_araim_debug.hpp:56-59` | **PASS** | CSV with `gamma_rmse`, `gamma_inlier`, `gamma_condition`, `gamma_age` | None |
| Solution-separation variance fallback | `lidar_araim.cpp:128-146` | **PASS** | Fallback to `sqrt(Sigma_f)` when `Sigma_f - Sigma_0 <= floor_m2`, with `SS_VARIANCE_FALLBACK` flag | Flag is in CSV column, not consolidated bitmask in `LidarAraimResult` |
| No change to certified GNSS ARAIM | `integrity_monitor.cpp` | **PASS** | No code diff in `araim.*` or GNSS ARAIM types | None |
| No change to planner PI cost | Planners, launch | **PASS** | No Stage 0 diff in planner or launch files | None |
| No change to certified monitor fusion | `integrity_monitor.cpp:271-274` | **PASS** | `PL_E/N/U = max(existing, lidar_PL_E/N/U)`, `final_HPL/VPL_source = "LIDAR"` only if larger | None |
| No topic/message schema changes | ROS messages | **PASS** | No Stage 0 message schema changes | None |
| Configurable defaults | `lidar_araim.hpp:153-186` | **PASS** | `age_model=EXP_SATURATING`, `age_tau_s=30.0`, `target_window_K=10`, `sigma_ss_min_m=0.02` | CSV enabled by default in config (I/O overhead for production) |

### Stage 1: Naming Clarification

| Spec Requirement | Implementation Location | Status | Evidence | Risk / Follow-up |
|-----------------|------------------------|--------|----------|-----------------|
| Monitor fused aliases | `integrity_types.hpp:193-199` | **PASS** | `monitor_fused_pl/hpl/vpl/pl_e/n/u()`, `monitor_integrity_margin()` | Old raw fields still accessible; no deprecation warnings |
| GNSS certified aliases | `integrity_types.hpp:201-205` | **PASS** | `gnss_certified_hpl/vpl/pl_e/n/u()` | Only in `IntegrityReport`, not in planner types |
| LiDAR certified aliases | `integrity_types.hpp:207-211` | **PASS** | `lidar_certified_hpl/vpl/pl_e/n/u()` | Consistent log labels use this form |
| Advisory predicted aliases | `future_pl_query_result.hpp:71-77` | **PASS** | `advisory_predicted_hpl/vpl/pl()`, `gnss_advisory_hpl/vpl_proxy()`, `advisory_predicted_fused_hpl/vpl()` | No deprecation on old fields |
| No behavior changes | All files | **PASS** | Aliases are const accessors returning existing fields | None |
| Old fields preserved | All types | **PASS** | Old public member fields unchanged | None |
| Log labels updated | `integrity_monitor.cpp:467`, `integrity_extension.cpp:441` | **PASS** | `lidar_certified_HPL` used | Mixed convention: fused values still use old `monitor_HPL/VPL` labels |
| No topic/CSV breakage | Topics, CSV | **PASS** | No field name changes in topics or CSV | None |

### Stage 2: Advisory FIM Predictor

| Spec Requirement | Implementation Location | Status | Evidence | Risk / Follow-up |
|-----------------|------------------------|--------|----------|-----------------|
| Advisory FIM structs exist | `advisory_fim_types.hpp:12-42` | **PASS** | `FimDiagnostic`, `GnssAdvisoryFimResult`, `LidarAdvisoryFimResult`, `FusedAdvisoryFimResult` | None |
| GNSS advisory FIM: H=G^TWG, 4D ENU+clock | `predicted_araim.cpp:181-207` | **PASS** | Line 181: `Eigen::Matrix4d h` from LoS vector `g`, line 191: `h += (1/sigma^2) * g*g^T`, lines 195-207: Schur complement | Symmetrization at line 208 is defensive but safe |
| GNSS position FIM via Schur complement | `predicted_araim.cpp:195-207` | **PASS** | `lambda = h_pp - h_pc*h_cp/(h_cc + clock_eps)`, then symmetrize, PSD check, eigenvalue floor if negative | None |
| LiDAR 3x3 translational FIM | `lidar_observability_fim.cpp:204-258` | **PASS** | `lambda += weight_scale * pi_range * confidence * weight * inv_sigma2 * (n * n^T)` | No raycast (Version A as specified), PCA fallback for missing normals |
| LiDAR FIM uses normals if present | `lidar_observability_fim.cpp:218` | **PASS** | Normalizes `primitive.normal_w` | PCA fallback: `phase2_planner_integrity_evaluator.cpp:426-481` with hardcoded radius=1.5m |
| FIM-add advisory branch | `future_pl_field_predictor.cpp:330-429` | **PASS** | `Lambda_adv = Lambda_prior + Lambda_G + Lambda_L`, `Sigma_adv = inv(Lambda_adv + eps*I)`, `HPL_adv = K_H*sqrt(lambda_max(Sigma_xy)) + b_H + s_H`, `VPL_adv = K_V*sqrt(Sigma_zz) + b_V + s_V` | `fim_regularized` always set true (line 401) even when no regularization needed — minor semantic inaccuracy |
| FIM-add disabled by default | Launch line 468, header line 35 | **PASS** | `phase2_use_advisory_fim_add: false` default | None |
| Stage 2 diagnostic fields present | `future_pl_query_result.hpp:48-59` | **PASS** | All 12 fields: `lambda_prior_trace`, `lambda_gnss_trace`, `lambda_lidar_trace`, `lambda_adv_trace`, `lambda_adv_min_eig`, `lambda_adv_condition`, `hpl_adv`, `vpl_adv`, `lidar_fim_valid`, `gnss_fim_valid`, `fim_regularized`, `advisory_fusion_mode` | Fields interpolated in `pl_grid.cpp:190-203` |
| Old advisory predictor preserved | `future_pl_field_predictor.cpp:432-491` | **PASS** | Legacy LOI fusion path still present behind `use_advisory_fim_add=false` | None |
| No change to certified monitor | All monitor files | **PASS** | No FIM-add in certified path, monitor fusion unchanged | None |
| No change to planner cost | Planner files | **PASS** | No Stage 2 diff in PI cost or planner behavior | None |
| GNSS degeneracy handling | `predicted_araim.cpp:214-227` | **PASS** | PSD check with eigenvalue floor, sets valid=false on failure | None |
| LiDAR degeneracy handling | `lidar_observability_fim.cpp:240-253` | **PASS** | Min voxel check, condition number check, sets valid=false | None |
| Prior FIM from covariance | `future_pl_field_predictor.cpp:93-114` | **PASS** | Extracts `lambda_base_pos` from snapshot, validates | Fallback via `gnss_base_information()` lines 65-91 |

### Stage 3: Unified PI Cost Adapter

| Spec Requirement | Implementation Location | Status | Evidence | Risk / Follow-up |
|-----------------|------------------------|--------|----------|-----------------|
| Unified advisory PI mode behind flag | `pi_cost_adapter.hpp` | **PASS** | `phase2_pi_use_unified_advisory_pl` controls mode | None |
| Legacy PI preserved when disabled | `pi_cost_adapter.cpp` | **PASS** | Old `evaluate(hal, val, hpl, vpl)` path unchanged | None |
| PI input selection: fim_add first | `phase2_planner_integrity_evaluator.cpp` | **PASS** | Uses valid `hpl_adv/vpl_adv` from FIM-add first, then falls back to advisory predicted, then constant_current only if explicit flag | Input selection layer documented in `stage_3_plan.md` |
| IM_H = HAL - HPL_adv | `pi_cost_adapter.cpp` | **PASS** | Formula implemented | None |
| IM_V = VAL - VPL_adv | `pi_cost_adapter.cpp` | **PASS** | Formula implemented | None |
| IM = min(IM_H, IM_V) | `pi_cost_adapter.cpp` | **PASS** | Formula implemented | None |
| c_hinge = lambda * (max(0, HPL-HAL+margin_h)^2 + max(0, VPL-VAL+margin_v)^2) | `pi_cost_adapter.cpp` | **PASS** | Hinge formula with configurable margins per axis | None |
| c_ratio optional, default off | `pi_cost_adapter.cpp` | **PASS** | `pi_use_ratio_term: false` default | None |
| c_PI = clamp(c_hinge + c_ratio + unknown_penalty, 0, max_cost) | `pi_cost_adapter.cpp` | **PASS** | Clamping applied | None |
| Unknown/sentinel not silently zero cost | `pi_cost_adapter.cpp` | **PASS** | `pi_penalize_unknown_advisory: true` default, assigns `pi_max_cost` to unknown | None |
| /iap/integrity_cost_field routed through unified PI | `phase2_planner_integrity_evaluator.cpp` | **PASS** | Backend publisher uses adapter when Stage 3 enabled | None |
| /iap/integrity_front_cost_field routed through unified PI | `phase2_planner_integrity_evaluator.cpp` | **PASS** | Front-end publisher uses adapter cost instead of standalone ratio cost when Stage 3 enabled | Legacy fallback remains when disabled |
| Stage 3 CSV/summary fields | `phase2_planner_integrity_evaluator.cpp` | **PASS** | All 13 new fields present: `advisory_hpl_used`, `advisory_vpl_used`, `advisory_pl_source`, `im_h_adv`, `im_v_adv`, `im_min_adv`, `pi_hinge_cost`, `pi_ratio_cost`, `pi_total_cost`, `pi_unknown_penalty`, `pi_input_valid`, `pi_fallback_reason`, `pi_cost_clamped`, `risk_band_adv`; `pi_stage3` summary with source histogram | None |
| No change to certified monitor | Monitor code | **PASS** | No Stage 3 diff in monitor | None |
| No change to Stage 2 FIM math | `future_pl_field_predictor.cpp` | **PASS** | FIM computation unchanged | None |
| A* and B-spline consume same PI interface | `dyn_a_star.cpp:268`, `bspline_optimizer.cpp` | **PASS** | Both consume cost from the same field publishers | A* uses `edge_cost = static_cost * (1 + lambda * cost)`, B-spline uses cost+gradients; consistent |

### Stage 4: Unified Risk Grid

| Spec Requirement | Implementation Location | Status | Evidence | Risk / Follow-up |
|-----------------|------------------------|--------|----------|-----------------|
| URG core exists | `unified_risk_grid.hpp/cpp` | **PASS** | `include/iap/planner/unified_risk_grid.hpp`, `src/iap/planner/unified_risk_grid.cpp` | None |
| URG test exists | `test_unified_risk_grid.cpp` | **PASS** | 5 tests | Tests are unit-level only, not integration |
| UnifiedRiskVoxel with ESDF, occ, AL, PL, IM, PI, age, flags | `unified_risk_grid.hpp:47-53` | **PASS** | All fields present: `esdf_m`, `occ_prob`, `al_m`, `hpl_adv_m`, `vpl_adv_m`, `pl_adv_m`, `integrity_margin_m`, `pi_cost`, `age_s`, `flags` | Note: `pl_adv_m` is present but spec asked for separate HPL/VPL which are present |
| URG flags enum present | `unified_risk_grid.hpp:15-28` | **PASS** | `VALID_ESDF`, `VALID_OCCUPANCY`, `VALID_AL`, `VALID_ADVISORY_PL`, `VALID_PI`, `STALE_PL`, `UNKNOWN_RISK`, `OCCUPIED`, `OUT_OF_RANGE`, `FIM_ADD_USED`, `LIDAR_FIM_VALID`, `GNSS_FIM_VALID`, `PI_INPUT_VALID` | Spec also had `GNSS_DEGENERATE`, `LIDAR_DEGENERATE`, `FIM_REGULARIZED`, `SS_VARIANCE_FALLBACK`, `FRAME_MISMATCH`, `CLOCK_UNIT_FALLBACK` — these exist in advisory FIM types but not directly in URG flags |
| URG integrated into evaluator behind flag | `phase2_planner_integrity_evaluator.cpp:1040` | **PASS** | `phase2_use_unified_risk_grid: false` default | None |
| Disabled mode legacy-compatible | Smoke test | **PASS** | `phase2_use_unified_risk_grid=false` produces zero URG stats, clean evaluator exit | None |
| Enabled publishes on legacy topics | `phase2_planner_integrity_evaluator.cpp` | **PASS** | URG-derived samples published on `/iap/integrity_cost_field` and `/iap/integrity_front_cost_field` when enabled | None |
| PointCloud2 field names preserved | `phase2_planner_integrity_evaluator.cpp:3590-3605,3677-3690` | **PASS** | `x y z hpl vpl hal val im_h im_v im_min cost grad_x grad_y grad_z risk_band risk_band_code` all present | None |
| A* front search consumes field cost with legacy fallback | `dyn_a_star.cpp:268` | **PASS** | `edge_cost = static_cost * (1.0 + lambda * integrity_cost)` | Uses nearest-neighbor lookup of field cost |
| B-spline consumes cost and gradients | `bspline_optimizer.cpp` | **PASS** | Subscribes to `/iap/integrity_cost_field`, uses `cost` and `grad_x/y/z` fields | None |
| Stale/unknown handling | `unified_risk_grid.cpp`, evaluator | **PASS** | Stale voxels get unknown penalty; `UNKNOWN_RISK` flag set | Works as designed but unknown_count=5202 in demo11 smoke shows many unknowns |
| URG CSV export | `phase2_planner_integrity_evaluator.cpp:2752-2803` | **PASS** | `urg_grid_voxels.csv` exported when `phase2_urg_export_voxels=true` | None |
| URG summary fields | `phase2_summary.json` | **PASS** | All requested counters present: `urg_enabled`, `urg_active`, `urg_update_count`, `urg_query_count`, `urg_grid_hit/miss_count`, `urg_direct_query_count`, `urg_stale_count`, `urg_unknown_count`, `urg_valid_pi_count`, `urg_mean_update_ms`, `urg_p95_update_ms`, `urg_front/backend_field_points`, `urg_unknown_penalty_count` | None |
| Local occupancy helpers | `local_occupancy.hpp/cpp` | **PASS** | Read-only occupancy query for URG export | Minimal implementation |
| No change to certified monitor | Monitor code | **PASS** | No URG code in monitor | None |
| No change to Stage 2/3 math | FIM/PI code | **PASS** | Only routing Stage 3 results into URG grid samples | None |

---

## 3. Cross-Stage Invariant Matrix

| Invariant | Status | Evidence | Risk / Follow-up |
|-----------|--------|----------|-----------------|
| Certified monitor fusion = max(PL_G, PL_L) | **PASS** | `integrity_monitor.cpp:271-274`: `PL_E/N/U = max(existing, lidar_PL_E/N/U)` per-axis max | None |
| Advisory output named as adv/proxy/advisory | **PASS** | Stage 1 aliases + log labels use advisory/certified distinction | FIM-add result fields use `hpl_adv/vpl_adv` names |
| /iap/integrity remains current certified monitor topic | **PASS** | No topic changes in any stage | None |
| Stage 0: planner behavior unchanged | **PASS** | No Stage 0 diff in planner | None |
| Stage 1: numerical behavior unchanged except logging | **PASS** | Only const alias accessors added | None |
| Stage 2: FIM-add only in advisory predictor | **PASS** | `future_pl_field_predictor.cpp`, disabled by default | None |
| Stage 3: ESDF/collision weights not auto-tuned | **PASS** | Only PI cost adapter changed | None |
| Stage 3: no change to Stage 2 FIM math | **PASS** | `future_pl_field_predictor.cpp` unchanged | None |
| Stage 4: old topics remain published | **PASS** | `urg_keep_legacy_topics: true` default; both front/backend publishers active | None |
| Stage 4: no change to Stage 2/3 math | **PASS** | Only routing Stage 3 results | None |
| Existing ROS message fields preserved | **PASS** | PointCloud2 fields same, CSV columns additive | None |
| Disabled modes = legacy-compatible | **PASS** | All stage flags default false; smoke-tested in both modes | None |

---

## 4. Test Coverage Matrix

| Stage | Unit Tests | Integration Tests | Demo Smoke | Full Demo Validation | Validator Status | Gaps |
|-------|-----------|-------------------|------------|---------------------|-----------------|------|
| **Stage 0** | 39 tests in `test_araim.cpp` (AgeRiskSaturates, TargetWindowCapsHypotheses, SigmaSsFallbackForNegativeRawVariance) | None | Yes (30s smoke) | 90s run completed, Phase 1 passed | Phase 2 validator failed | No integration tests; Phase 2 validator outdated |
| **Stage 1** | 4 tests in `test_integrity_snapshot.cpp` (alias verification) | None | Yes (30s smoke) | 90s run completed, Phase 1 passed | Phase 2 validator failed (schema) | Naming aliases not tested in integration |
| **Stage 2** | 5 tests in `test_predicted_araim.cpp`, 8 in `test_lidar_observability_fim.cpp`, 8 in `test_future_pl_field_predictor.cpp`, 5 in `test_pl_grid.cpp` | None | Yes (30s smoke with FIM-add enabled) | Not run at 90s | Phase 2 validator fails (old `fim_add` assumptions) | No corridor anisotropy test; no GNSS-rich vs GNSS-poor A/B test |
| **Stage 3** | 11 tests in `test_pi_cost_adapter.cpp` (hinge, ratio, unknown penalty, legacy) | None | Yes (30s smoke with Stage 2+3 enabled) | Not run at 90s | Phase 2 validator fails | No end-to-end PI field smoke |
| **Stage 4** | 5 tests in `test_unified_risk_grid.cpp` | None | Yes (30s smoke with Stage 2+3+4 both enabled and disabled) | Not run at 90s | Phase 2 validator fails | URG grid correctness not validated; stale/unknown penalty behavior not integration-tested |
| **Cross-stage** | `test_alert_limit_model.cpp` (3), `test_run_log_manager.cpp` (1) | None | Yes | No | Phase 2 fails | No cross-stage integration test |

### Build/Test Commands Used (per status.md)

```bash
# Build
colcon build --base-paths src/iap src/gnss_comm --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Full test suite
colcon test --base-paths src/iap src/gnss_comm --packages-select iap
colcon test-result --test-result-base build/iap --verbose
# Result: 85 tests, 0 errors, 0 failures, 0 skipped (Stage 2 run)
# Result: 81 tests (Stage 1 run); 9/9 focused CTest (Stage 3)

# Focused tests
colcon test --base-paths src/iap src/gnss_comm --packages-select iap \
  --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor|test_lidar_observability_fim|test_pi_cost_adapter|test_unified_risk_grid"

# Demo11 smoke (Stage 2+3+4 enabled)
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py \
  start_rviz:=false use_integrity_global_search:=true \
  phase2_use_unified_risk_grid:=true phase2_urg_export_voxels:=true \
  phase2_urg_keep_legacy_topics:=true phase2_pl_model:=fused_fim_grid \
  phase2_use_pl_grid:=true phase2_export_pl_grid_voxels:=true \
  phase2_use_advisory_fim_add:=true phase2_use_lidar_advisory_fim:=true \
  phase2_use_lidar_observability:=false \
  phase2_pi_use_unified_advisory_pl:=true
```

---

## 5. Backward Compatibility Matrix

| Interface / Topic / File | Expected Compatibility | Observed Implementation | Status | Risk |
|--------------------------|----------------------|------------------------|--------|------|
| `/iap/integrity` (certified monitor) | Unchanged | Unchanged | **PASS** | None |
| `/iap/integrity_cost_field` | Preserved with same fields | PointCloud2 fields unchanged: `x y z hpl vpl hal val im_h im_v im_min cost grad_x grad_y grad_z risk_band risk_band_code` | **PASS** | Old consumers see same field layout |
| `/iap/integrity_front_cost_field` | Preserved with same fields | Same field layout preserved | **PASS** | Front-end consumer unchanged |
| `IntegrityReport` ROS message | No field removal | Old fields preserved, Stage 1 aliases added as const accessors | **PASS** | No breaking API change |
| `FuturePLQueryResult` | Old fields preserved | Stage 1 aliases + Stage 2 FIM fields added | **PASS** | All additive |
| `integrity_along_planner_traj.csv` | Old columns preserved | Stage 3 columns added, old columns kept | **PASS** | Additive |
| `pl_grid_voxels.csv` | Old columns preserved | Stage 2 FIM columns added, old columns kept | **PASS** | Additive |
| `phase2_summary.json` | Schema issues | `ana_log.py` rewrite drops fields needed by online validator | **PARTIAL** | Schema mismatch breaks Phase 2 validator |
| `iap_lidar_araim_stage0.csv` (new) | Additive | New CSV, no backward-compat issue | **PASS** | None |
| `urg_grid_voxels.csv` (new) | Additive | New CSV, no backward-compat issue | **PASS** | None |
| `planner_cmd.csv`, `planner_traj.csv` | Unchanged | Unchanged | **PASS** | None |
| Launch defaults | Safe defaults | All Stage 2/3/4 flags default `false` | **PASS** | Default demo11 run = legacy behavior |
| `advisory_fusion_mode` field | String, no enum | `std::string` type with values `"legacy"`, `"fim_add"` | **PASS** | No compile-time validation of mode string |

---

## 6. Performance and Runtime Risk

### 6.1 URG Timing Analysis

**Observed metrics from Stage 4 enabled demo11 smoke:**
- `urg_mean_update_ms`: 8881.45 ms
- `urg_p95_update_ms`: 11893.49 ms
- Grid: 25m half-extent in x and y, 1m resolution, ~2601 voxels
- Updates: 6 per ~30s run

**Root cause analysis:**

The bottleneck is `rebuild_unified_risk_grid()` at `phase2_planner_integrity_evaluator.cpp:2806-2838`. The triple-nested loop (lines 2823-2830) calls `make_unified_risk_voxel()` for every voxel. Each voxel evaluation performs:
1. **Advisory PL query** — full `FuturePLFieldPredictor::query()` call including GNSS FIM Schur complement (4x4 matrix operations, per-satellite geometry) and LiDAR FIM accumulation (neighbor search over all primitives, normal-based 3x3 accumulation)
2. **ESDF/clearance proxy query** — map lookup
3. **AL evaluation** — clearance-based alert limit computation
4. **PI cost computation** — Stage 3 PI adapter with hinge/ratio terms
5. **Gradient computation** — central finite differences (6 additional PL queries per voxel for `compute_gradients()` at line 2831)

**Estimated cost per voxel:** With ~2601 voxels and ~8881 ms total = ~3.4 ms/voxel. At 25m half-extent, this means ~51x51 = 2601 voxels. If gradient computation requires 6 extra queries per voxel, the effective cost is ~3.4 ms per (query + gradient) pair.

**Bottlenecks (in order of impact):**
1. **Per-voxel full FIM recomputation** — each voxel independently computes GNSS FIM (per-satellite iteration, Schur complement) and LiDAR FIM (neighbor search, 3x3 accumulation). No caching between nearby voxels.
2. **Gradient computation** — `compute_gradients()` requires finite-difference queries in 6 directions per voxel, multiplying the query cost by ~6x.
3. **Grid size** — 50x50 = 2500+ voxels is large for a full recompute. No sparse/active-voxel optimization.
4. **CSV export** — `write_urg_grid_voxel_samples_locked()` writes all voxels to CSV on every rebuild, adding I/O latency.
5. **No incremental update** — full grid rebuild on every update, even when only a small region changed.
6. **No parallel update** — sequential triple loop, no multi-threading or SIMD vectorization.

### 6.2 Demo11 Shutdown Behavior

From status.md: "Shutdown produced several ROS node errors after SIGINT/SIGTERM, including `poscmd_2_odom`, `pcl_render_node`, `traj_server`, and odometry visualization nodes."

**Assessment:** These are non-core visualization/sensing nodes. The core `iap_rosnode`, GNSS sim, phase1 logger, and phase2 evaluator finished cleanly. The URG-enabled smoke had the evaluator caught during heavy rebuild work, with the ROS launch timeout escalating shutdown after the summary had been written.

**Risk:** Medium — shutdown behavior may corrupt summary files if evaluator is killed mid-write. Mitigation: evaluator flushes files after each write.

### 6.3 Odom Freshness / Alignment Risk

From status.md: "`demo9_preflight_control_odom_mux` repeatedly rejected IAP odometry as stale with `valid_iap_streak=0`."

**Assessment:** This affects all demo11 runs regardless of stage. The IAP odometry age check fails, causing the control odom mux to reject IAP odom and stay in truth-bootstrap mode. This breaks the closed-loop integrity validation (planner uses truth odom instead of IAP odom).

**Risk:** High for closed-loop validation — prevents meaningful assessment of integrity-aware planning with real IAP odometry.

### 6.4 Phase 2 Validator / Schema Risk

From status.md: "`ana_log.py` rewrote `phase2_summary.json` into a schema that no longer contains several online-validator-required blocks."

**Missing blocks:** `fallback_count`, `fallback_rate`, `fallback_reason_histogram`, `finite_gnss_prediction_count`, `integrity_snapshot`, `current_consistency_raw`, `current_consistency_anchored`, `current_consistency`, `phase_h_lite`, `stage1_capabilities`, `pi_cost`.

**Risk:** High — until schema is unified, the Phase 2 validator cannot pass even if the system is working correctly.

### 6.5 Old Validator fim_add Assumptions

From status.md: "`integrity_along_planner_traj.csv row 2: PL_H_pred=1.019764180 is below gnss_hpl=20.000000000`"

The old Phase 2 validator requires `PL_H_pred >= gnss_hpl` (conservative check for old max-fusion). Stage 2 FIM-add intentionally produces tighter (more optimistic) advisory PL, making this check invalid for the `fim_add` mode.

**Risk:** Medium — forces validator failure on all Stage 2+ enabled runs.

---

## 7. Additional Implementation Issues Discovered

### 7.1 Minor Issues

1. **`fim.regularized` always true** (`future_pl_field_predictor.cpp:401`): Set unconditionally after LDLT solve regardless of whether regularization was needed. The flag should reflect whether `fim_epsilon > 0` was actually required for PSD.

2. **PCA fallback has hardcoded constants** (`phase2_planner_integrity_evaluator.cpp:427-428`): `kRadius = 1.5`, `kMaxPcaPoints = 2000`, `kMinSupport = 6` are compile-time constants, not configurable launch parameters.

3. **PCA stride sampling bias** (`phase2_planner_integrity_evaluator.cpp:430-431`): Uses index-based stride sampling, not spatial bucketing. Dense clusters produce more primitives than sparse regions, potentially biasing the FIM distribution.

4. **Global staleness** (`phase2_planner_integrity_evaluator.cpp:2065-2066`): In `make_unified_risk_voxel()`, STALE_PL is set based on a single global `current_integrity_stamp_`. All voxels get the same staleness flag regardless of individual age.

5. **URG gradient computation cost**: `compute_gradients()` performs finite-difference queries in 6 directions per voxel (additional PL queries), which multiplies the per-voxel query cost by ~6x.

6. **URG CSV export blocking**: `write_urg_grid_voxel_samples_locked()` writes all voxels (~2600 rows) on every rebuild, adding I/O latency.

7. **Local occupancy no FIFO eviction** (`local_occupancy.cpp:34`): Comment says "simple guard: just stop if full" — when `max_voxels` is reached, new points are rejected but old points are never evicted.

8. **PointCloud2-only planner integration**: The B-spline optimizer consumes only legacy PointCloud2 topics. When `urg_keep_legacy_topics_=false`, the optimizer receives no integrity cost — there is no direct URG query API in the planner backend.

9. **Stage 1 naming hybrid**: `integrity_monitor.cpp:464-472` uses `lidar_certified_HPL` for LiDAR but `monitor_HPL`/`monitor_VPL` for GNSS-sourced fused values — naming convention is inconsistent within the same log output.

10. **`advisory_fusion_mode` type safety**: Uses `std::string` instead of enum. A misspelled mode value would silently produce unexpected behavior.

### 7.2 Test Coverage Gaps (Beyond Those Already Noted)

| Test | Missing Coverage |
|------|-----------------|
| `test_unified_risk_grid.cpp` | OCCUPIED flag, OUT_OF_RANGE flag, gradient correctness, empty grid, NaN/inf positions, exact staleness timeout boundaries, FIM_ADD_USED/GNSS_FIM_VALID flags |
| `test_pi_cost_adapter.cpp` | Ratio term enabled, cost clamping exceeded, `penalize_unknown_advisory=false` with invalid input, dominant_axis tie, SAFE/MARGINAL/UNSAFE threshold boundaries, `risk_band_code()` static method |
| `test_lidar_observability_fim.cpp` | PCA fallback correctness, condition number boundary, min_voxel boundary, empty primitive set |

---

## 8. Final Recommendation

### Can the implementation be marked as stage-complete?

**No.** Each stage has code-level test coverage but none has passed full official demo11 acceptance. The implementation work is substantially done for all stages, but validation gaps prevent completion claims.

### Can it be marked as fully validated?

**No.** The primary blockers are:
1. Phase 2 validator schema mismatch (`validate_phase2_integrity_eval.py` vs `ana_log.py`)
2. Old validator `fim_add` assumptions invalid for Stages 2-4
3. IAP odometry freshness in demo11
4. URG rebuild performance (~8.9s mean)

### What must be fixed before final acceptance?

**Priority 1 (blocking):**
1. Unify `phase2_summary.json` schema between `tools/phase2/validate_phase2_integrity_eval.py` and `tools/ana_log.py`
2. Update Phase 2 validator to understand `fim_add` semantics (remove `PL_H_pred >= gnss_hpl` check in advisory mode)
3. Investigate and fix IAP odometry freshness issue in demo11

**Priority 2 (validation):**
4. Run full 90s demo11 validation with Stage 2+3+4 enabled
5. Run full 90s demo11 validation with all stages disabled (legacy baseline)
6. Corridor anisotropy test for LiDAR FIM (B2 in spec)
7. GNSS-rich vs canopy degradation test for GNSS FIM (B3 in spec)

**Priority 3 (performance):**
8. URG performance optimization (Stage 4.1 / Stage 5):
   - Sparse active voxels only (skip UNKNOWN_RISK voxels in rebuild)
   - Cache GNSS FIM per satellite geometry batch
   - Incremental LiDAR FIM update
   - Parallel grid refresh (OpenMP or thread pool)
   - Multi-resolution grid (coarse grid for distant regions)
   - Replace per-voxel finite-difference gradient with analytic gradient propagation

### Prioritized Follow-up Tasks

| Priority | Task | Estimated Effort | Blocks |
|----------|------|-----------------|--------|
| P0 | Schema unification | 1-2 days | All stage acceptance |
| P0 | Old validator fim_add update | 0.5 day | Stage 2-4 acceptance |
| P0 | Odom freshness fix | 1-3 days | All demo11 validation |
| P1 | Stage 4.1 URG perf: sparse active voxels | 1-2 days | Stage 4 acceptance |
| P1 | Stage 4.1 URG perf: FIM cache | 1-2 days | Stage 4 acceptance |
| P1 | Full 90s demo11 validation runs (all modes) | 1 day | All stage acceptance |
| P2 | Corridor anisotropy test | 0.5 day | Stage 2 acceptance |
| P2 | GNSS-rich vs canopy test | 0.5 day | Stage 2 acceptance |
| P2 | Stage 5: parallel refresh | 2-3 days | Performance target |
| P3 | Add integration tests for cross-stage flows | 2-3 days | Regression prevention |
| P3 | Fix `fim.regularized=true` unconditional set | 0.5 day | Diagnostic accuracy |
| P3 | Make PCA radius configurable (not hardcoded 1.5m) | 0.5 day | LiDAR FIM flexibility |
| P3 | Fix shutdown cancellation during URG rebuild | 1 day | Clean shutdown |
