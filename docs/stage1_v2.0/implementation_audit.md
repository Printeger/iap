# IAP Stage 0-4 Implementation Audit

Date: 2026-05-13

Scope: read-only audit of `src/iap` against `docs/stage1_v2.0/iap_codex_implementation_plan_v2_3.tex`, `docs/stage1_v2.0/status.md`, and the current repository implementation. Repository code is treated as authoritative where docs and implementation differ. Historical command results are reported from `status.md`; this audit did not re-run build, test, launch, or validator commands.

## 1. Executive Verdict

Overall status: **PARTIAL**.

The Stage 0-4 implementation is substantially present in code, and the recorded evidence shows code-level tests plus demo11 smoke runs for the later stages. It cannot be marked fully accepted because full official demo11 validation remains pending, the Phase 2 validator still contains old `fim_add` assumptions, there is a known `phase2_summary.json` schema mismatch between analyzer and validator outputs, demo11 odometry freshness/alignment remains unresolved, and Stage 4 URG update cost is too high for comfortable default-runtime use.

Per-stage status:

| Stage | Status |
|---|---|
| Stage 0: LiDAR ARAIM PL Stability Repair | Implementation complete, code-level tested, Stage 0 logging seen in demo11. Safety-sensitive bounded-window semantics remain a validation risk. |
| Stage 1: Certified Monitor vs Advisory Predictor Naming Clarification | Implementation complete, code-level tested, demo11 exercised. Behavior-change audit passes at code level; full official demo11 integrity acceptance pending. |
| Stage 2: Advisory Future FIM Predictor | Implementation complete, code-level tested, demo11 smoke-tested with FIM-add enabled. Full official demo11 acceptance pending due to validator and freshness/alignment gaps. |
| Stage 3: Unified Advisory PI Cost Adapter | Implementation complete, code-level tested, demo11 smoke-tested with Stage 2 + Stage 3 enabled. Full official demo11 acceptance pending. |
| Stage 4: Unified Risk Grid / URG | Implementation complete, code-level tested, demo11 smoke-tested in enabled and disabled modes. Full official demo11 acceptance pending and performance/shutdown risk remains high. |

## 2. Audit Matrix

| Stage | Spec requirement | Implementation location | Status | Evidence | Risk / follow-up |
|---|---|---|---|---|---|
| Stage 0 | Bounded LiDAR age risk is implemented as `1 - exp(-Age/tau)` or capped linear age. | `src/iap/src/iap/integrity/lidar_araim.cpp`, `src/iap/include/iap/integrity/lidar_araim.hpp` | PASS | `compute_risk_components()` supports `EXP_SATURATING` and `LINEAR_CAPPED`; configs set `lidar_araim_age_model` and `lidar_araim_gamma_age_max`. | Verify tuned `tau/ref/max` values against field data. |
| Stage 0 | Target keyframe/window cap exists and is configurable. | `lidar_araim.cpp`, `integrity_extension.cpp`, `config/*config_gnss.json` | PASS | `filter_target_window()` caps selected target IDs; `lidar_araim_target_window_K` is loaded and defaulted to 10. | Target-window omission can reduce PL by excluding older/farther hypotheses; keep explicitly documented as safety-sensitive Stage 0 behavior. |
| Stage 0 | LiDAR PL component logging includes `|d_f|`, sigma terms, bias, risk gammas, selected target count, and worst hypothesis id. | `include/iap/integrity/lidar_araim_debug.hpp` | PASS | CSV header includes `sep_term_m`, `sigma_ss_term_m`, `sigma_subset_term_m`, `bias_term_m`, `gamma_rmse`, `gamma_inlier`, `gamma_condition`, `gamma_age`, `selected_target_count`, `target_window_size`, `worst_hypothesis_type`, `worst_hypothesis_id`. | Runtime log is abbreviated; full component evidence is in CSV. |
| Stage 0 | Negative/raw solution-separation variance handling is explicit. | `src/iap/src/iap/integrity/lidar_araim.cpp`, `include/iap/integrity/lidar_araim_debug.hpp` | PASS | Status records raw variance, floor, fallback diagnostics and `ss_variance_fallback_flag`; tests include `SigmaSsFallbackForNegativeRawVariance`. | No consolidated bitmask in `LidarAraimResult`; consumers must inspect CSV/subset fields. |
| Stage 0 | Certified GNSS ARAIM is unchanged. | `src/iap/src/iap/integrity/araim.cpp`, `include/iap/integrity/araim_types.hpp` | PASS | Stage 0 status records no diff in GNSS ARAIM files; later Stage 2 FIM code is in planner-side advisory classes. | This is a source-level audit, not numerical regression against pre-Stage-0 binaries. |
| Stage 0 | Planner PI cost, topics, message schemas unchanged by Stage 0. | `msg/IntegrityReport.msg`, planner files, launch/config | PASS | Stage 0 status records no Stage 0 diff in planner cost files, planner behavior, ROS topic launch defaults, or message definitions. | Later stages add planner behavior behind flags. |
| Stage 0 | Certified monitor fusion remains `PL_mon_q = max(PL_G_q, PL_L_q)`. | `src/iap/src/iap/integrity/integrity_monitor.cpp` | PASS | `run_lidar_araim()` applies max updates for `PL_E`, `PL_N`, `PL_U`, `HPL`, `VPL`, and scalar `PL`. | Continue to regression-test this invariant when changing monitor code. |
| Stage 1 | Naming clarification implemented without behavior changes. | `include/iap/integrity/integrity_types.hpp`, `include/iap/planner/integrity_snapshot.hpp`, `include/iap/planner/future_pl_query_result.hpp` | PASS | Stage 1 added semantic alias accessors and comments; storage fields remain unchanged. | Behavior-preservation claim relies on source review plus recorded tests. |
| Stage 1 | Current monitor fields documented/aliased as `gnss_certified_*`, `lidar_certified_*`, `monitor_fused_*`, `monitor_integrity_margin`. | `IntegrityReport`, `CurrentIntegrityState`, `IntegrityReport.msg` | PASS | Accessors include `gnss_certified_hpl()`, `lidar_certified_hpl()`, `monitor_fused_hpl()`, `monitor_integrity_margin()`; message comments call fields monitor-fused certified outputs. | ROS message does not add new fields, by design. |
| Stage 1 | Advisory/future predictor fields documented/aliased as advisory/proxy fields. | `future_pl_query_result.hpp`, `predicted_araim.hpp`, planner docs | PASS | `gnss_advisory_hpl_proxy()`, `advisory_predicted_hpl()`, `advisory_predicted_fused_hpl()` exist; comments state advisory outputs are not certified monitor PL. | Downstream consumers still see legacy field names in CSV/topics for compatibility. |
| Stage 1 | No breaking topic/message/CSV changes; old fields remain. | `msg/IntegrityReport.msg`, `araim_debug.hpp`, evaluator CSV headers | PASS | Existing message fields and legacy CSV names are retained; Stage 1 status records additive naming/log/comment changes only. | Future cleanup must avoid removing legacy names without migration. |
| Stage 2 | Advisory-only FIM structs exist. | `include/iap/planner/advisory_fim_types.hpp` | PASS | Defines `FimDiagnostic`, `GnssAdvisoryFimResult`, `LidarAdvisoryFimResult`, `FusedAdvisoryFimResult`. | None found. |
| Stage 2 | GNSS advisory FIM uses 4D ENU position plus meter-equivalent clock state. | `src/iap/src/iap/planner/predicted_araim.cpp` | PASS | `predict_advisory_fim()` builds a 4x4 `h_full` from satellite line-of-sight rows `[E,N,U,clock]` and weights by `1/sigma^2`. | Frame convention should remain documented in any future refactor. |
| Stage 2 | GNSS position FIM uses clock Schur complement. | `predicted_araim.cpp`, `test/test_predicted_araim.cpp` | PASS | Code computes `h_pp - (h_pc * h_cp)/(h_cc + clock_eps)`; test `AdvisoryFimUsesClockSchurComplement` covers this. | Keep PSD/regularization thresholds visible in config. |
| Stage 2 | LiDAR advisory FIM uses 3x3 translational normal information. | `src/iap/src/iap/planner/lidar_observability_fim.cpp` | PASS | `evaluate_advisory_fim()` sums weighted `n * n.transpose()` over nearby `LidarFimPrimitive` normals. | First version is translational only, as specified. |
| Stage 2 | LiDAR FIM uses normals if present or local PCA fallback if implemented. | `apps/phase2_planner_integrity_evaluator.cpp`, `lidar_observability_fim.cpp` | PASS | Evaluator reads `normal_x/y/z` PointCloud2 fields when present; status records local PCA fallback from downsampled map cloud. | PCA normal estimation can be a performance bottleneck if repeated too often. |
| Stage 2 | FIM-add branch formula implemented. | `src/iap/src/iap/planner/future_pl_field_predictor.cpp` | PASS | Adds prior, GNSS, and LiDAR lambdas; solves `(Lambda_adv + fim_epsilon I)^-1`; computes HPL from `lambda_max(Sigma_xy)` and VPL from `Sigma_zz` with K/bias/slack terms. | Always regularized; logs should distinguish numerical regularization from fallback. |
| Stage 2 | FIM-add branch disabled by default and explicitly enabled only. | `future_pl_field_predictor.hpp`, `phase2_planner_integrity_evaluator.cpp`, demo10/demo11 launches | PASS | `use_advisory_fim_add` defaults false in params and launch args. | Demo smoke enables it intentionally. |
| Stage 2 | Stage 2 diagnostics are present. | `future_pl_query_result.hpp`, `pl_grid.cpp`, evaluator CSV headers | PASS | Fields include `lambda_prior_trace`, `lambda_gnss_trace`, `lambda_lidar_trace`, `lambda_adv_trace`, `lambda_adv_min_eig`, `lambda_adv_condition`, `hpl_adv`, `vpl_adv`, `lidar_fim_valid`, `gnss_fim_valid`, `fim_regularized`, `advisory_fusion_mode`. | Validator must be updated to interpret `fim_add` mode correctly. |
| Stage 2 | Old advisory predictor behavior remains available. | `future_pl_field_predictor.cpp`, `predicted_araim.cpp` | PASS | Existing `predict_araim_result()` path still runs; FIM-add is conditional. | Legacy and `fim_add` outputs can have different conservatism, which old validator does not understand. |
| Stage 3 | Unified advisory PI mode exists behind `phase2_pi_use_unified_advisory_pl`. | `pi_cost_adapter.hpp/cpp`, evaluator, demo10/demo11 launches | PASS | `PICostAdapter::Params::use_unified_advisory_pl` and evaluator parameter `pi_use_unified_advisory_pl` are wired; launch defaults are false. | None found. |
| Stage 3 | Legacy PI behavior is preserved when disabled. | `pi_cost_adapter.cpp`, `test/test_pi_cost_adapter.cpp` | PASS | Non-Stage-3 branch keeps legacy weighted hinge; test `LegacyDefaultBehaviorRemainsUnchanged` covers this. | Keep legacy test if later changing adapter internals. |
| Stage 3 | PI input selection prefers valid FIM-add HPL/VPL, falls back to advisory predicted PL, and uses current only with explicit compatibility flag. | `apps/phase2_planner_integrity_evaluator.cpp` | PASS | `select_pi_advisory_pl()` chooses `fim_add`, then advisory predicted/fallback fields, then `constant_current_compat` only if `pi_allow_constant_current_fallback` is true; otherwise `unknown`. | Fallback histograms should be monitored in demos. |
| Stage 3 | Stage 3 formula matches spec. | `src/iap/src/iap/planner/pi_cost_adapter.cpp` | PASS | Computes `IM_H`, `IM_V`, min margin, hinge cost using `max(0, PL - AL + margin)^2`, optional ratio term, unknown penalty, and clamp. | `penalize_unknown_advisory` defaults true in evaluator, false in raw adapter params; evaluator default is the operational path. |
| Stage 3 | Unknown/sentinel advisory does not silently become zero cost by default. | `pi_cost_adapter.cpp`, evaluator params, `test/test_pi_cost_adapter.cpp` | PASS | Stage 3 invalid/sentinel inputs can receive max-cost unknown penalty; evaluator declares `pi_penalize_unknown_advisory` default true. | Direct adapter default remains false for legacy/library compatibility. |
| Stage 3 | `/iap/integrity_cost_field` and `/iap/integrity_front_cost_field` route through unified PI adapter when enabled. | `apps/phase2_planner_integrity_evaluator.cpp` | PASS | Back-end publisher writes `pi_cost_total`; front sample builder calls `pi_cost_adapter_.evaluate()`. | Field values depend on Stage 3 flag and URG overlay when enabled. |
| Stage 3 | Front-end cost no longer recomputes separate ratio cost when Stage 3 data is available; legacy fallback remains. | evaluator and `bspline_optimizer.cpp` | PASS | Evaluator publishes adapter cost; B-spline front search consumes finite supplied `sample.cost`; `normalizedFrontIntegrityCost()` remains only fallback when cost is nonfinite. | Front consumer still requires finite `hpl/vpl/hal/val` to keep a sample. |
| Stage 3 | Stage 3 CSV/summary fields exist. | `apps/phase2_planner_integrity_evaluator.cpp` | PASS | CSV fields include `advisory_hpl_used`, `advisory_vpl_used`, `advisory_pl_source`, `im_h_adv`, `im_v_adv`, `im_min_adv`, `pi_hinge_cost`, `pi_ratio_cost`, `pi_total_cost`, `pi_unknown_penalty`, `pi_input_valid`, `pi_fallback_reason`, `pi_cost_clamped`, `risk_band_adv`; summary includes `pi_stage3`. | CSV has both `pi_total_cost` and compatibility `pi_cost_total`; keep both. |
| Stage 4 | URG core exists with tests. | `include/iap/planner/unified_risk_grid.hpp`, `src/iap/planner/unified_risk_grid.cpp`, `test/test_unified_risk_grid.cpp` | PASS | Core files and tests are present and wired in `CMakeLists.txt`. | Stage 4 files are currently untracked/dirty in the local worktree. |
| Stage 4 | `UnifiedRiskVoxel` includes ESDF, occupancy, AL/HAL/VAL, advisory PL, advisory IM, PI cost/gradient, age, flags. | `unified_risk_grid.hpp` | PASS | Struct includes `esdf_m`, `occ_prob`, `al_h_m`, `al_v_m`, `hal_m`, `val_m`, `hpl_adv_m`, `vpl_adv_m`, `pl_adv_m`, IM fields, PI cost/grad fields, `age_s`, `flags`. | None found. |
| Stage 4 | Required URG flags exist. | `unified_risk_grid.hpp` | PASS | Defines `VALID_ESDF`, `VALID_OCCUPANCY`, `VALID_AL`, `VALID_ADVISORY_PL`, `VALID_PI`, `STALE_PL`, `UNKNOWN_RISK`, `OCCUPIED`, `OUT_OF_RANGE`, `FIM_ADD_USED`, `LIDAR_FIM_VALID`, `GNSS_FIM_VALID`, `PI_INPUT_VALID`. | None found. |
| Stage 4 | URG integrated into phase2 evaluator behind `phase2_use_unified_risk_grid`. | evaluator and demo11 launch | PASS | Evaluator declares `use_unified_risk_grid` default false; demo11 launch exposes `phase2_use_unified_risk_grid` default false. | demo10 launch has Stage 2/3 args, but Stage 4 URG args appear only in demo11. |
| Stage 4 | Disabled mode is legacy-compatible. | evaluator, status demo run `20260513T055654Z_017` | PASS | Disabled smoke recorded `urg_enabled=false`, zero URG query/update counts, no `urg_grid_voxels.csv`, and Stage 2/3 path continued. | Full validator still pending. |
| Stage 4 | Enabled mode publishes URG-derived samples on legacy-compatible topics. | evaluator publishers, status demo run `20260513T052517Z_755` | PASS | Enabled smoke recorded front/backend field counts and legacy topics `/iap/integrity_cost_field`, `/iap/integrity_front_cost_field`; publishers keep same topic names. | Heavy rebuilds can delay publishing and shutdown. |
| Stage 4 | Existing PointCloud2 fields are preserved. | `phase2_planner_integrity_evaluator.cpp` | PASS | Both front and back field publishers set 16 fields: `x,y,z,hpl,vpl,hal,val,im_h,im_v,im_min,cost,grad_x,grad_y,grad_z,risk_band,risk_band_code`. | Field semantics are advisory planner cost, not certified PL; comments document this. |
| Stage 4 | A* front search consumes field-provided cost when available, with old ratio fallback. | `sim/.../bspline_optimizer.cpp` | PASS | `queryFrontIntegrityCost()` uses finite `sample.cost`; otherwise falls back to `normalizedFrontIntegrityCost(hpl,vpl,hal,val)`. | It clamps to `integrity_front_cost_max`; ensure configured max matches PI scale. |
| Stage 4 | B-spline backend consumes cost and gradients as before. | `bspline_optimizer.cpp` | PASS | `onIntegrityCostField()` reads `cost`, `grad_x/y/z`, `risk_band`; backend adds sample cost and gradient during optimization. | Samples with `risk_band == 0` are skipped by backend. |
| Stage 4 | Stale/unknown risk does not decay to zero risk by default. | `unified_risk_grid.cpp`, tests | PASS | `apply_unified_risk_stale_policy()` adds penalty for stale/unknown; tests cover stale penalty and unknown high cost after timeout. | Enabled demo showed all URG queries stale, indicating freshness tuning issue. |
| Stage 4 | URG CSV and summary fields exist. | evaluator summary/CSV code, status | PASS | Summary includes `urg_enabled`, `urg_active`, update/query/hit/miss/direct/stale/unknown/valid PI counts, update timing, front/backend points, unknown penalty count, flags histogram, and `urg_voxel_csv`; `urg_grid_voxels.csv` exported when enabled/requested. | Summary schema must be reconciled with Phase 2 validator/analyzer. |

## 3. Cross-Stage Invariant Matrix

| Invariant | Status | Evidence | Risk / follow-up |
|---|---|---|---|
| Certified GNSS ARAIM behavior remains unchanged except approved changes. | PASS | GNSS certified monitor code remains in `integrity/araim.*`; advisory GNSS FIM is added under planner `PredictedAraimComputer::predict_advisory_fim()`. Stage 0/1 status records no GNSS ARAIM diff. | Needs numerical regression if certification claims are made. |
| Certified LiDAR ARAIM behavior changed only through Stage 0 stability repair. | PASS | LiDAR monitor changes are localized to `lidar_araim.*`, Stage 0 debug/config, and odometry block metadata. | Window cap and normalized/capped risk terms are semantic changes requiring validation. |
| Current certified monitor fusion remains `PL_mon_q = max(PL_G_q, PL_L_q)`. | PASS | `IntegrityMonitor::run_lidar_araim()` max-fuses LiDAR PL axes and scalar/H/V values into report. | Add a small regression test for mixed GNSS/LiDAR axis dominance. |
| `/iap/integrity` remains current certified monitor topic. | PASS | Configs and launch files keep `publish_topic`/`integrity_topic` as `/iap/integrity`; `IntegrityReport.msg` comments call it current certified monitor result. | None found. |
| Advisory FIM-add is limited to future predictor/planner path. | PASS | FIM-add lives in planner predictor/evaluator files and is guarded by `use_advisory_fim_add`. | Avoid importing planner FIM results into `IntegrityMonitor`. |
| Stage 2 does not alter certified monitor math. | PASS | Stage 2 code is in `planner/*` plus evaluator/launch/test wiring; certified monitor files are not part of Stage 2 implementation. | Source-level audit only. |
| Stage 3 does not alter certified monitor math or Stage 2 FIM math. | PASS | Stage 3 changes are `PICostAdapter`, evaluator PI selection/routing, launch args, and tests. | None found. |
| Stage 4 does not alter certified monitor, Stage 2 FIM, or Stage 3 PI math except routing Stage 3 results into URG samples. | PASS | URG builds voxels from `select_pi_advisory_pl()` and `PICostAdapter` outputs; certified monitor files untouched by Stage 4. | URG query policy can add stale/unknown penalties to PI cost, which is intended routing behavior. |
| Existing topics remain backward-compatible. | PASS | `/iap/integrity`, `/iap/integrity_cost_field`, and `/iap/integrity_front_cost_field` remain default topic names. | The cost topics carry advisory PI semantics; consumers must not treat them as certified monitor topics. |
| Existing ROS message fields and old CSV columns are not removed. | PASS | `IntegrityReport.msg` retains existing fields; evaluator CSV fields are additive; Stage 1 aliases do not rename storage. | Future cleanup must preserve old columns until consumers migrate. |
| Existing PointCloud2 fields required by A* and B-spline are preserved. | PASS | Both publishers include all required fields: `x,y,z,hpl,vpl,hal,val,im_h,im_v,im_min,cost,grad_x,grad_y,grad_z,risk_band,risk_band_code`. | Front field gradients are currently zeros; backend field gradients are PI gradients. |
| Disabled/default modes remain legacy-compatible unless explicitly enabled. | PASS | Stage 2 FIM-add, Stage 3 unified PI, and Stage 4 URG default false in launch/evaluator params; status includes disabled URG smoke. | Demo11 defaults still publish integrity cost fields, but URG/FIM/Stage3 remain gated. |
| Stage 2 FIM-add output feeds Stage 3 PI selection. | PASS | `select_pi_advisory_pl()` selects `pl.hpl_adv/vpl_adv` when `advisory_fusion_mode == "fim_add"` and valid. | Validator must stop enforcing old `PL_H_pred >= gnss_hpl` assumption for this mode. |
| Stage 3 PI output feeds Stage 4 URG. | PASS | `make_unified_risk_voxel()` evaluates PI cost via `pi_cost_adapter_` and stores PI cost/grad/flags in `UnifiedRiskVoxel`. | Per-voxel direct PI evaluation is a major performance cost. |
| Stage 4 URG-derived cost feeds front-end and back-end compatibility topics. | PASS | `annotate_row_with_urg()` overwrites PI fields from URG queries; publishers emit unchanged PointCloud2 fields to legacy topics. | Heavy URG rebuilds can delay or interrupt publication. |
| Summary JSON additions are additive. | PARTIAL | Current evaluator writes old expected blocks plus new `advisory_fim`, `pi_stage3`, and `urg`; status also records analyzer/validator schema mismatch after `ana_log.py`. | Phase 2 validator and analyzer must converge on one stable schema. |

## 4. Test Coverage Matrix

| Stage | Unit tests | Integration tests | Demo smoke | Full demo validation | Validator status | Gaps |
|---|---|---|---|---|---|---|
| Stage 0 | `test_araim.cpp`: `AgeRiskSaturates`, `TargetWindowCapsHypotheses`, `SigmaSsFallbackForNegativeRawVariance`; recorded `test_araim` 73 tests passed. | Stage 0 logging/config exercised by iap package tests. | Demo11 90s run produced `iap_lidar_araim_stage0.csv` with 1488 rows. | One 90s demo11 run recorded for Stage 0/1 context, but Stage 0 not separately accepted as final. | Phase 1 official passed in run `20260513T022944Z_974`; Phase 2 standalone failed. | Safety validation of target-window hypothesis omission and tuned risk caps. |
| Stage 1 | Full `iap` CTest recorded as 81 tests passed; focused 79-test subset passed. | Alias/log/doc changes compile through package tests. | Demo11 90s run recorded with expected artifacts. | Not full accepted due to Phase 2 validator failure. | Phase 1 official passed; Phase 2 failed due to consistency/schema/alignment issues. | Stable Phase 2 validator/analyzer schema and odometry freshness. |
| Stage 2 | `test_predicted_araim`, `test_lidar_observability_fim`, `test_future_pl_field_predictor`, `test_pl_grid`; recorded 85 tests passed. | Launch py_compile for demo10/demo11; PL grid/FIM diagnostics interpolation tests. | 30s demo11 smoke with FIM-add active, `query_count=34315`, FIM valid counts nonzero. | Not run as full 90s official acceptance. | Existing Phase 2 validator failed due to old `fim_add` assumptions and missing aligned CSV. | Validator must accept advisory FIM-add being more optimistic than legacy GNSS proxy. |
| Stage 3 | `test_pi_cost_adapter`; recorded `ctest --test-dir build/iap` 9/9 passed after Stage 3. | Evaluator PI selection/routing covered by demo smoke and code inspection. | 30s demo11 Stage 2+3 smoke, `pi_stage3.enabled=true`, selected source `fim_add`, zero unknowns in that run. | Not full accepted. | Phase 1 failed only due to 29.95s smoke duration; Phase 2 failed old `fim_add` assumptions and missing aligned CSV. | Full-duration run after validator updates; monitor fallback histograms for unknown/fallback sources. |
| Stage 4 | `test_unified_risk_grid`; focused tests `test_pl_grid|test_future_pl_field_predictor|test_pi_cost_adapter|test_unified_risk_grid` all passed per status. | `build_phase1_ego_planner_closed_loop.sh` recorded 20 packages finished and verified executables. | Demo11 enabled and disabled URG smokes recorded. Enabled run had nonzero URG updates/queries and front/backend field points. | Not full accepted. | Full official Phase 2 still blocked by `fim_add`, URG schema/freshness semantics, and odometry assumptions. | URG performance, shutdown cancellation, validator schema, full 90s acceptance. |

Recorded commands from `status.md`:

```bash
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim"
colcon test-result --test-result-base build/iap --verbose

colcon build --base-paths src/iap src/gnss_comm --packages-select iap --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --base-paths src/iap src/gnss_comm --packages-select iap
colcon test-result --test-result-base build/iap --verbose

colcon build --base-paths src/iap src/gnss_comm --packages-select iap --cmake-args -DBUILD_TESTING=ON
colcon test --base-paths src/iap src/gnss_comm --packages-select iap --ctest-args -R "test_araim|test_predicted_araim|test_integrity_snapshot|test_pl_grid|test_future_pl_field_predictor|test_lidar_observability_fim|test_pi_cost_adapter"
python3 -m py_compile launch/demo10_ego_planner_pi_lite_eval.launch.py launch/demo11_ego_planner_integrity_corridor.launch.py

colcon build --packages-select iap
ctest --test-dir build/iap -R test_pi_cost_adapter --output-on-failure
ctest --test-dir build/iap -R test_future_pl_field_predictor --output-on-failure
ctest --test-dir build/iap --output-on-failure

python3 -m py_compile src/iap/launch/demo11_ego_planner_integrity_corridor.launch.py
./src/iap/tools/build_phase1_ego_planner_closed_loop.sh
ctest --test-dir build/iap -R "test_pl_grid|test_future_pl_field_predictor|test_pi_cost_adapter|test_unified_risk_grid" --output-on-failure
```

Recorded demo/validator commands from `status.md` include:

```bash
ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py start_rviz:=false run_duration_s:=90 allow_truth_alignment:=false use_so3_dynamics:=true use_iap_odom_for_planner:=true use_gnss:=true use_araim:=true planner_use_integrity_cost:=true planner_use_integrity_front_search:=true planner_use_integrity_global_search:=true

python3 src/iap/tools/phase1/validate_phase1_closed_loop.py --run-dir /home/dev/ws_iap/src/iap/log/20260513T022944Z_974 --official
python3 src/iap/tools/phase2/validate_phase2_integrity_eval.py --run-dir /home/dev/ws_iap/src/iap/log/20260513T022944Z_974
python3 src/iap/tools/ana_log.py --run /home/dev/ws_iap/src/iap/log/20260513T022944Z_974

timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py ... phase2_use_advisory_fim_add:=true phase2_use_lidar_advisory_fim:=true
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py ... phase2_pi_use_unified_advisory_pl:=true
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py ... phase2_use_unified_risk_grid:=true phase2_urg_export_voxels:=true
timeout 180s ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py ... phase2_use_unified_risk_grid:=false
```

## 5. Backward Compatibility Matrix

| Interface/topic/file/CSV/message | Expected compatibility | Observed implementation | Status | Risk |
|---|---|---|---|---|
| `/iap/integrity` | Remains certified current monitor topic. | Configs, launch files, and evaluator subscription keep `/iap/integrity`; message comments say current certified monitor. | PASS | None found. |
| `iap::msg::IntegrityReport` | Existing fields are not removed. | Message still exposes `hpl`, `vpl`, `pl_e`, `pl_n`, `pl_u`, `hal`, `val`, `im`, diagnostics, and quality fields. | PASS | Message lacks new semantic alias fields intentionally; aliases are in C++ comments/accessors. |
| `IntegrityReport` C++ struct | Old public fields remain available. | Legacy storage fields remain; alias accessors are additive. | PASS | Avoid replacing storage names in future. |
| Current monitor CSV `iap_araim.csv` | Old columns retained. | Stage 1 comments/log labels are clarified; Stage 0/1 status records old schemas preserved. | PASS | New docs should define semantic names to reduce confusion. |
| Stage 0 `iap_lidar_araim_stage0.csv` | Additive diagnostic output. | New Stage 0 file adds component columns without replacing old certified monitor artifacts. | PASS | Extra file I/O may affect long runs if enabled by default. |
| `integrity_along_planner_traj.csv` | Old columns retained; new fields additive. | Evaluator CSV headers include original current/predicted fields plus Stage 2/3/4 additions. | PASS | Validators must tolerate additive fields and duplicate compatibility fields. |
| `pl_grid_voxels.csv` | Existing grid CSV remains; FIM/PI fields additive. | Stage 2/3 fields present when exported. | PASS | Large exports can affect runtime. |
| `urg_grid_voxels.csv` | New only when URG enabled/export requested. | Status confirms present in enabled run and absent in disabled run. | PASS | Export can worsen Stage 4 runtime and shutdown. |
| `phase2_summary.json` | Additions should not break consumers. | Evaluator writes old blocks plus `advisory_fim`, `pi_stage3`, `urg`; `ana_log.py` can rewrite into schema missing validator-required fields. | PARTIAL | Resolve analyzer/validator schema mismatch before acceptance. |
| `/iap/integrity_cost_field` | Topic name and PointCloud2 fields preserved. | Publisher emits required 16 fields; cost is Stage 3/URG PI cost when enabled. | PASS | Field semantics are advisory, not certified. |
| `/iap/integrity_front_cost_field` | Topic name and PointCloud2 fields preserved. | Publisher emits same 16 fields; front A* consumes finite `cost`. | PASS | Front publisher uses zero gradients. |
| B-spline backend cost field | Continues reading `cost` and gradients. | `onIntegrityCostField()` reads `cost`, `grad_x/y/z`, `risk_band`; optimizer uses cost/gradient. | PASS | Risk band 0 samples skipped. |
| A* front search field | Consumes provided cost with old ratio fallback. | `queryFrontIntegrityCost()` uses finite `sample.cost`, otherwise ratio fallback. | PASS | If supplied cost is nonfinite, legacy semantics return. |
| Launch defaults | New modes disabled unless explicitly enabled. | demo10/demo11 Stage 2/3 args default false; demo11 URG args default false. | PASS | demo10 lacks Stage 4 URG launch wiring, which appears intentional because Stage 4 is demo11-integrated. |

## 6. Performance and Runtime Risk

### URG Timing Analysis

The Stage 4 enabled demo11 smoke recorded:

| Metric | Recorded value |
|---|---:|
| `urg_update_count` | 6 |
| `urg_query_count` | 15612 |
| `urg_grid_hit_count` | 15610 |
| `urg_grid_miss_count` | 2 |
| `urg_direct_query_count` | 2 |
| `urg_front_field_points` | 2601 |
| `urg_backend_field_points` | 2602 |
| `urg_mean_update_ms` | about 8881 ms |
| `urg_p95_update_ms` | about 11893 ms |

Likely bottlenecks found in code:

- Grid size: default demo11 URG extent is 25 m by 25 m at 1 m resolution, which produces about 2601 2D samples per rebuild before trajectory/backend additions.
- Per-voxel direct query: `rebuild_unified_risk_grid()` calls `make_unified_risk_voxel()` for every voxel.
- FIM recomputation: each voxel can call `pl_values()`, which may evaluate Stage 2 advisory FIM-add and LiDAR FIM.
- Local normal/PCA path: status records fallback PCA normal generation when `normal_x/y/z` are unavailable; this can be expensive if refreshed frequently.
- AL/ESDF/occupancy recomputation: each voxel queries alert limits, clearance proxy, and occupancy probability.
- PI gradient recomputation: `make_unified_risk_voxel()` calls `pi_cost_gradient_at()`, which performs finite differences and therefore additional PI/PL queries per voxel.
- CSV export: `urg_grid_voxels.csv` is written when export is requested; this adds synchronous I/O.
- Lack of incremental update: URG rebuild resets and recomputes the full grid each time.
- Shutdown cancellation: rebuild holds the URG mutex and does not check cancellation inside the nested voxel loops, so timeout shutdown can catch the evaluator in heavy work.

Stage 4.1 performance tasks, not implemented here:

1. Reduce URG grid size or make it adaptive around the active planning corridor and current trajectory envelope.
2. Cache or batch per-voxel advisory PL, FIM, AL, ESDF, occupancy, and PI computations.
3. Avoid local PCA/FIM recomputation when the map cloud and query neighborhood are unchanged.
4. Add incremental URG updates keyed by generation, center shift, and changed map regions.
5. Add cancellation checks and shorter mutex-held sections inside URG rebuild for responsive shutdown.
6. Gate `urg_grid_voxels.csv` export away from default performance runs.
7. Add timing breakdown fields for PL query, LiDAR FIM, AL/ESDF, PI gradient, CSV export, and total update.

### Demo11 Shutdown Behavior

Status records repeated shutdown errors in non-core nodes such as `poscmd_2_odom`, `pcl_render_node`, `traj_server`, and odometry visualization nodes. Stage 4 enabled smoke wrote valid summaries and CSVs, but launch timeout caught the evaluator during heavy URG work and escalated shutdown. This is a runtime robustness risk, not evidence that the data path failed.

### Odom Freshness / Alignment Risk

The demo11 run logs repeatedly reported `demo9_preflight_control_odom_mux` rejecting IAP odometry as stale with `valid_iap_streak=0`. Phase 1 official validation passed in the Stage 0/1 90s run, but the freshness issue remains a cross-stage demo11 acceptance risk because planner/controller alignment affects whether the exported integrity samples represent the intended closed-loop path.

### Validator / Schema Risk

The Phase 2 standalone validator still fails for old assumptions:

- It expects conservative legacy behavior such as `PL_H_pred >= gnss_hpl`, which is not valid for advisory FIM-add because Stage 2 intentionally fuses information additively on the advisory path.
- It requires `phase2_integrity_eval_aligned.csv` in cases where the online run did not produce it.
- After `ana_log.py`, `phase2_summary.json` can be rewritten into a schema missing online-validator-required blocks such as `fallback_count`, `fallback_rate`, `integrity_snapshot`, `current_consistency_raw`, `phase_h_lite`, and `pi_cost`.

The validator and analyzer need one stable schema before full Stage 2-4 acceptance can be claimed.

## 7. Final Recommendation

The implementation can be marked **stage-implemented** for Stages 0-4 at code level, with the following nuance:

- Stage 0: implementation complete and code/demo artifact verified, but target-window hypothesis omission remains safety-sensitive and needs explicit validation.
- Stage 1: implementation complete and behavior-compatible at source level.
- Stage 2: implementation complete and smoke-tested, but full validation blocked by old Phase 2 validator assumptions.
- Stage 3: implementation complete and smoke-tested with Stage 2, but full validation depends on Stage 2 validator updates and full demo11 runs.
- Stage 4: implementation complete and smoke-tested in enabled/disabled modes, but performance and shutdown behavior require Stage 4.1 work before final acceptance.

The implementation should **not** be marked fully validated yet.

Must fix before final acceptance:

1. Update Phase 2 validator logic for advisory `fim_add` semantics and URG fields.
2. Stabilize the `phase2_summary.json` schema across online evaluator, `ana_log.py`, and validators.
3. Investigate and fix demo11 IAP odometry freshness/alignment.
4. Reduce URG update time or make rebuild cancellable/responsive.
5. Run a full-duration official demo11 validation with Stage 2+3+4 intended modes and require both Phase 1 and Phase 2 validators to pass.

Prioritized follow-up tasks:

1. Validator/schema unification, because it gates objective acceptance.
2. URG Stage 4.1 performance and shutdown cancellation, because current rebuild times are multi-second.
3. Odom freshness/alignment fix, because it affects closed-loop validity.
4. Safety validation for Stage 0 target-window cap and LiDAR PL component tuning.
5. Add small invariant regression tests for certified monitor max fusion and disabled-mode behavior for Stage 2/3/4.
