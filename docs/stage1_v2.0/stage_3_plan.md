**Stage 3 PI Cost Adapter Redesign**

**Summary**
- Redesign planner/evaluator PI cost to consume one unified advisory PL signal: selected `HPL_adv/VPL_adv`, `HAL/VAL`, and advisory integrity margin.
- Keep certified monitor paths unchanged: GNSS ARAIM, LiDAR ARAIM, monitor fusion `PL_mon_q=max(PL_G_q,PL_L_q)`, and `/iap/integrity` semantics remain untouched.
- Preserve legacy behavior unless `phase2_pi_use_unified_advisory_pl:=true`; all demo11 Stage 3 checks should use demo11 because it is the IAP closed-loop test.

**Current PI Paths**
- PI adapter: [pi_cost_adapter.hpp](/home/dev/ws_iap/src/iap/include/iap/planner/pi_cost_adapter.hpp:8), [pi_cost_adapter.cpp](/home/dev/ws_iap/src/iap/src/iap/planner/pi_cost_adapter.cpp:25).
- Evaluator: [phase2_planner_integrity_evaluator.cpp](/home/dev/ws_iap/src/iap/apps/phase2_planner_integrity_evaluator.cpp:716).
- Future PL query shape: [future_pl_query_result.hpp](/home/dev/ws_iap/src/iap/include/iap/planner/future_pl_query_result.hpp:13), FIM-add query population in [future_pl_field_predictor.cpp](/home/dev/ws_iap/src/iap/src/iap/planner/future_pl_field_predictor.cpp:330).
- Back-end cost field publisher: `/iap/integrity_cost_field`, built from `pi_cost_total/grad/risk_band` in [phase2_planner_integrity_evaluator.cpp](/home/dev/ws_iap/src/iap/apps/phase2_planner_integrity_evaluator.cpp:2981).
- Front-end cost field publisher: `/iap/integrity_front_cost_field`, currently separate ratio cost in [phase2_planner_integrity_evaluator.cpp](/home/dev/ws_iap/src/iap/apps/phase2_planner_integrity_evaluator.cpp:2827).
- A* consumer: [dyn_a_star.cpp](/home/dev/ws_iap/src/iap/sim/ego_planner_swarm_ws/src/planner/path_searching/src/dyn_a_star.cpp:147), using nearest front cost as `edge_cost = static_cost * (1 + lambda * cost)`.
- B-spline consumer: [bspline_optimizer.cpp](/home/dev/ws_iap/src/iap/sim/ego_planner_swarm_ws/src/planner/bspline_opt/src/bspline_optimizer.cpp:1427), using nearest sampled `cost` and `grad_x/y/z`; added into rebound/refine objectives at [bspline_optimizer.cpp](/home/dev/ws_iap/src/iap/sim/ego_planner_swarm_ws/src/planner/bspline_opt/src/bspline_optimizer.cpp:2335).
- Demo11 launch wiring: [demo11_ego_planner_integrity_corridor.launch.py](/home/dev/ws_iap/src/iap/launch/demo11_ego_planner_integrity_corridor.launch.py:195).

**Current Behavior**
- Inputs to `PICostAdapter`: evaluator calls `evaluate(hal, val, pl.hpl, pl.vpl)` and finite-difference gradient via `evaluate_with_gradient(...)`.
- HAL/VAL: evaluator calls `evaluate_alert_limit`; `fixed_alert_limit` uses configured `hal_m/val_m`, `cloud_clearance` derives HAL from nearest cloud obstacle and VAL from vertical bounds.
- Current cost formula:
  `cost_h = weight_h * max(0, HPL + marginal_margin - HAL)^2`;  
  `cost_v = weight_v * max(0, VPL + marginal_margin - VAL)^2`;  
  `cost_total = cost_h + cost_v`.
- `constant_current`: `pl_values()` copies current monitor `current_hpl_/current_vpl_`, labels `query_source="current"`, and fills `hpl_adv/vpl_adv` with the same values as compatibility diagnostics.
- `fused_fim_grid`: `FuturePLFieldPredictor::query()` returns advisory grid/direct PL; with Stage 2 FIM-add enabled it fills `hpl_adv/vpl_adv`, `lambda_*`, `*_fim_valid`, `fim_regularized`, and `advisory_fusion_mode`.
- Front-end currently does not consume `PICostAdapter`; it recomputes a ratio band cost `max(HPL/HAL,VPL/VAL)` and returns zero for invalid inputs.
- Back-end consumes sampled cost plus gradient; front-end consumes sampled cost only. `PICostAdapter` itself has no analytic PL gradient; evaluator uses finite differences.

**Stage 3 Changes**
- Add a small PI input selection layer in the evaluator, not in certified monitor code:
  `select_pi_advisory_pl(pl_fields) -> {hpl_used, vpl_used, source, valid, fallback_reason}`.
- Selection order when `phase2_pi_use_unified_advisory_pl=true`:
  1. Use `hpl_adv/vpl_adv` when finite, below sentinel, and FIM validity passes config.
  2. Else use existing advisory predicted `pl.hpl/pl.vpl` when finite and not sentinel.
  3. Else use `constant_current` only if `phase2_pi_allow_constant_current_fallback=true`, and label source explicitly.
  4. Else mark unknown; never silently use current monitor PL as future advisory PL.
- Implement Stage 3 formula in `PICostAdapter`:
  `IM_H = HAL - HPL_adv`, `IM_V = VAL - VPL_adv`, `IM = min(IM_H, IM_V)`.
  `c_hinge = lambda_pi * (max(0,HPL_adv-HAL+margin_h)^2 + max(0,VPL_adv-VAL+margin_v)^2)`.
  Optional ratio term, default off:
  `c_ratio = mu_pi * ((HPL_adv/(HAL+eps_al))^2 + (VPL_adv/(VAL+eps_al))^2)`.
  `c_PI = clamp(c_hinge + c_ratio + unknown_penalty, 0, phase2_pi_max_cost)`.
- Preserve old adapter behavior behind `phase2_pi_use_unified_advisory_pl=false`; existing tests and legacy launch outputs should remain unchanged.
- Make both publishers use the same `PICostAdapter` result:
  `/iap/integrity_cost_field` keeps existing fields and adds new fields only if needed.
  `/iap/integrity_front_cost_field` should publish the adapter’s `cost` instead of recomputing the old ratio cost; keep legacy field names.
- Do not alter `FuturePLFieldPredictor` Stage 2 FIM math except for reading/query-source labeling if required.

**Config Defaults**
- Add evaluator params:
  `phase2_pi_use_unified_advisory_pl=false`,
  `phase2_pi_use_hinge_term=true`,
  `phase2_pi_use_ratio_term=false`,
  `phase2_pi_margin_h_m=1.0`,
  `phase2_pi_margin_v_m=1.0`,
  `phase2_pi_lambda=1.0`,
  `phase2_pi_mu_ratio=0.0`,
  `phase2_pi_eps_al_m=1e-3`,
  `phase2_pi_max_cost=3000.0`,
  `phase2_pi_allow_constant_current_fallback=false`,
  `phase2_pi_require_fim_valid=false`,
  `phase2_pi_penalize_unknown_advisory=true`.
- For demo11 Stage 3 enabled run, override `phase2_pi_use_unified_advisory_pl:=true`; keep planner weights unchanged unless separately approved.

**Unknown / Invalid Handling**
- NaN/Inf HPL/VPL, sentinel `>=1e9`, invalid FIM when required, or AL `<= eps_al`: mark `pi_input_valid=false`, `risk_band_adv=UNKNOWN_PI`.
- If `phase2_pi_penalize_unknown_advisory=true`, assign `pi_unknown_penalty=phase2_pi_max_cost`; otherwise assign zero only with explicit config and CSV reason.
- If FIM-add is invalid but legacy advisory PL is valid, use advisory fallback and label source/reason; do not call it FIM-add.
- Stale grid/query miss remains unknown unless direct advisory query succeeds.

**Logging / CSV / Summary**
- Add columns to `future_integrity_eval.csv` and grid voxel export where practical:
  `advisory_hpl_used`, `advisory_vpl_used`, `advisory_pl_source`, `im_h_adv`, `im_v_adv`, `im_min_adv`,
  `pi_hinge_cost`, `pi_ratio_cost`, `pi_total_cost`, `pi_unknown_penalty`,
  `pi_input_valid`, `pi_fallback_reason`, `risk_band_adv`.
- Add summary section `pi_stage3` with enabled flags, selected-source histogram, invalid/unknown counts, fallback reasons, cost quantiles, and max clamp count.
- Keep existing columns (`PL_H_pred`, `hpl_adv`, `pi_cost_total`, etc.) for compatibility.

**Implementation Order**
1. Extend `PICostAdapter::Params/PICostResult` with Stage 3 flags, margins, ratio, max cost, unknown penalty, and term breakdown while keeping legacy defaults.
2. Add unit tests for new adapter behavior and keep old tests passing under legacy/default config.
3. Add evaluator PI input selection helper and route `make_sample_row()`, `pi_cost_at()`, gradient calculation, and both cost-field publishers through it.
4. Replace front-end publisher’s standalone ratio cost with adapter cost, preserving old topic/field names.
5. Add launch arguments and parameter forwarding in demo11; optionally mirror in demo10 for regression checks.
6. Update summary/CSV fields and demo11 smoke scripts or documentation commands only after behavior is stable.

**Tests**
- Unit:
  `colcon test --packages-select iap --ctest-args -R test_pi_cost_adapter --output-on-failure`
- Add/adjust tests:
  hinge activates before PL exceeds AL due to margin;
  ratio term disabled by default;
  unknown advisory produces high cost when configured;
  FIM-add `hpl_adv/vpl_adv` selected when valid;
  `constant_current` works only as explicit compatibility fallback;
  legacy behavior unchanged when Stage 3 switch is disabled.
- Predictor sanity:
  `colcon test --packages-select iap --ctest-args -R test_future_pl_field_predictor --output-on-failure`
- Build:
  `colcon build --packages-select iap`

**Demo11 Checks**
- Legacy/off:
  `ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py start_rviz:=false run_duration_s:=30 phase2_pi_use_unified_advisory_pl:=false`
  Expect legacy-compatible PI fields and no Stage 3 source changes.
- Stage 3 on:
  `ros2 launch iap demo11_ego_planner_integrity_corridor.launch.py start_rviz:=false run_duration_s:=30 phase2_pi_use_unified_advisory_pl:=true phase2_use_advisory_fim_add:=true phase2_use_lidar_advisory_fim:=true`
  Expect new PI CSV fields, `/iap/integrity_cost_field`, `/iap/integrity_front_cost_field`, and nonempty source histograms.
- Demo11 smoke with Stage 2 + Stage 3 enabled should publish cost fields and CSV rows; do not require old Phase 2 validator until it is updated for FIM-add semantics.

**Risks / Open Questions**
- Unknown-as-high-cost can make A* and B-spline avoid large regions if advisory queries are stale; keep it configurable and summarize unknown counts.
- Front-end cost scale will change when it moves from ratio cost to hinge cost; clamp and existing planner weights should be left unchanged in first implementation.
- Finite-difference gradient is expensive because each point performs extra PL/AL queries; acceptable for Stage 3, but analytic/advisory grid gradients can be considered later.
- `phase2_pi_require_fim_valid` needs exact policy: recommend default `false`, and if true require at least `gnss_fim_valid` plus finite `hpl_adv/vpl_adv`; LiDAR FIM should not be required unless `use_lidar_advisory_fim=true`.
