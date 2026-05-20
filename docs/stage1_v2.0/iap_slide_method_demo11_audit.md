# Audit: `iap_slide_v2.tex` Method Section vs Demo11 Implementation

Date: 2026-05-14

Scope:

- Slide deck: `src/iap/docs/stage1_v2.0/iap_slide_v2.tex`
- Method section audited: Ch3 Methodology, plus the Ch2.5 transition claims that describe the method blocks.
- Code/demo flow audited: `demo11_ego_planner_integrity_corridor.launch.py` -> demo9/EGO Planner -> `phase2_planner_integrity_evaluator` -> EGO A* / B-spline cost-field consumers.
- Constraint: read-only audit of implementation. This report does not require or imply any IAP code changes.

Important interpretation:

- For demo11, the runtime planner is the EGO Planner stack under `sim/ego_planner_swarm_ws`, not only the standalone IAP `IntegrityPlanner`.
- Therefore, previous audits that checked only `iap::IntegrityPlanner` under `src/iap/src/iap/planner/` understate the demo11 planning integration. Demo11 does have integrity-aware A* and B-spline hooks in EGO Planner.

## Executive Summary

The slide's high-level method is directionally correct: it separates current certified monitoring from future advisory prediction, and demo11 does connect advisory integrity costs into an A* front-end and B-spline back-end.

However, several Ch3 claims are stronger than the current code. The biggest mismatches are:

1. GNSS state/factor description is missing the ECEF anchor variables and the ECEF measurement model.
2. Estimator "source-wise FIM export" is not implemented as a current-estimator interface.
3. ESDF wording overclaims a full distance-transform layer.
4. URG/PL map is rebuilt as a full centered grid, not an incremental rolling active-voxel cache.
5. Full URG + unified advisory PI + LiDAR advisory FIM are not demo11 defaults; they are optional/v2 enabled-mode settings.
6. A* and B-spline do consume integrity fields, but with simpler discrete/nearest-sample behavior than the slide's continuous integral/interpolation formulation.

## Demo11 Actual Data Flow

The current demo11 flow is:

1. `launch/demo11_ego_planner_integrity_corridor.launch.py`
   - Starts the corridor map publisher.
   - Includes `demo9_ego_planner_closed_loop.launch.py`.
   - Starts `phase2_planner_integrity_evaluator`.
   - Forwards planner integrity parameters into EGO Planner.

2. EGO Planner launch/config path:
   - `sim/ego_planner_swarm_ws/src/planner/plan_manage/launch/advanced_param.launch.py`
   - Enables optional planner integrity cost and front/global search parameters.
   - Passes `/iap/integrity_cost_field` to the B-spline optimizer.
   - Passes `/iap/integrity_front_cost_field` to A* front/global search.

3. `phase2_planner_integrity_evaluator`
   - Builds advisory PL, AL, IM, PI cost rows.
   - Optionally rebuilds `UnifiedRiskGrid`.
   - Publishes backend cost field on `/iap/integrity_cost_field`.
   - Publishes front-end cost field on `/iap/integrity_front_cost_field`.

4. EGO Planner consumers:
   - `path_searching/src/dyn_a_star.cpp` calls an integrity cost callback per neighbor and applies:
     `edge_cost = static_cost * (1 + lambda_integrity_cost * integrity_cost)`.
   - `bspline_opt/src/bspline_optimizer.cpp` subscribes to the backend cost field, reads `cost` and `grad_x/y/z`, and adds `lambda_integrity * f_integrity` into rebound/refine optimization.

## Finding Table

| # | Slide claim | Code reality | Classification |
|---|---|---|---|
| 1 | State is `[T, v, b_a, b_g, clock bias, clock drift]`; pseudorange is world-frame range | GNSS graph also estimates `E(0)` and `R(0)` and computes pseudorange in ECEF with Sagnac, lever arm, iono/trop/tgd | Modify slide |
| 2 | Estimator exports source-wise FIM `Lambda_prior + Lambda_IMU + Lambda_G + Lambda_L` | Current FGO export has total covariance/information and factor counts; source-wise FIM is advisory predictor-side | Modify slide |
| 3 | Shared map includes ESDF from occupancy distance transform | EGO has occupancy/inflated occupancy and rebound obstacle distance; URG `esdf_m` is obstacle-clearance/AL proxy, not a full ESDF transform | Modify slide |
| 4 | Rolling PL/IM map with active voxel update and staleness | URG is full-grid reset/rebuild around a center; local occupancy has eviction, but URG has no incremental rolling cell shift | Modify slide or implement later |
| 5 | Complete URG/FIM-add/LiDAR advisory FIM implied as demo11 method | demo11 defaults keep URG, unified advisory PI, and LiDAR advisory FIM disabled; v2 acceptance can enable them | Modify slide wording |
| 6 | Current monitor separated from advisory path; monitor uses conservative fusion | Code uses GNSS certified ARAIM plus LiDAR ARAIM and `max(GNSS, LiDAR)` monitor fusion | Matches |
| 7 | A* + B-spline consume integrity costs | Demo11 EGO Planner has integrity-aware A* and B-spline cost-field hooks | Matches, with implementation caveats |
| 8 | A* edge cost integrates `J_PI` along edge samples | Current A* samples the neighbor node via callback, not a multi-sample edge integral | Slide better; improve code if claiming formula |
| 9 | B-spline cost integrates continuous trajectory samples with interpolation-gradient chain rule | Current optimizer uses nearest field sample at control points and directly adds provided gradient | Slide better; improve code if claiming formula |
| 10 | Shared front/back field is a single URG by default | Default demo11 publishes legacy cost fields; URG can publish both when explicitly enabled | Clarify slide |

## Detailed Findings

### 1. GNSS State and Pseudorange Model Are Under-Specified

Slide sections:

- 3.1.1 State Definition
- 3.1.2 GNSS Pseudorange Factor

Slide claim:

- The state vector is only `T_k, v_k, b^a_k, b^g_k, delta t_clk, dot delta t_clk`.
- The pseudorange residual is written as a world-frame satellite-to-receiver range.

Code reality:

- `src/iap/src/iap/gnss/gnss_extension.cpp` declares and owns shared graph variables:
  - `E(0)`: ECEF coordinates of the world-frame origin.
  - `R(0)`: world-to-ECEF rotation.
- `src/iap/include/iap/gnss/pseudorange_factor.hpp` documents a four-key factor: pose `X(i)`, clock `C(i)`, ECEF origin `E(0)`, and world-to-ECEF rotation `R(0)`.
- `src/iap/src/iap/gnss/pseudorange_factor.cpp` computes:
  - antenna position with lever arm,
  - world-to-ECEF transform,
  - geometric range in ECEF,
  - Sagnac correction,
  - ionosphere/troposphere delay,
  - satellite group delay,
  - receiver clock bias in meters.

Correction for slide:

- Add `E(0)` and `R(0)` as shared graph variables or global calibration variables next to the per-keyframe state.
- Replace the pseudorange model with an ECEF-frame model:
  `P_ecef = R(0) * (p_world + R_wb * lever_arm) + E(0)`.
- Mention Sagnac, lever arm, iono/trop, and TGD as implemented corrections.

Classification: **modify slide**. The code implementation is more complete and should be reflected in the method.

### 2. Source-Wise FIM Export Is Not an Estimator Output

Slide section:

- 3.1.4b Estimator Information Interface: Source-Wise FIM Export

Slide claim:

- The estimator exports:
  `Lambda_0 = Lambda_prior + Lambda_IMU + Lambda_G + Lambda_L`.
- It also exports `Lambda_G` and `Lambda_L` from factor Jacobians.

Code reality:

- `src/iap/include/iap/integrity/fgo_information_matrix.hpp` defines `FGOPositionInfo` with:
  - `sigma_p`,
  - `lambda_p`,
  - `pose_cov_6x6`,
  - `p_world`,
  - factor counts and metadata lists.
- `src/iap/src/iap/integrity/fgo_information_manager.cpp` computes `marginalCovariance(X(frame_id))`, extracts the pose/position covariance block, and inverts `sigma_p` into `lambda_p`.
- It counts GNSS/trunk/IMU/clock factors, but it does not assemble source-wise information matrices.
- Source-wise advisory FIM is computed later in `src/iap/src/iap/planner/future_pl_field_predictor.cpp`:
  - `prior_information(snapshot)`,
  - `predict_advisory_fim()`,
  - optional `LidarObservabilityFim`,
  - `fim.lambda = prior + gnss + lidar`.

Correction for slide:

- Move source-wise FIM decomposition from the estimator section to the advisory predictor section.
- In the estimator section, state the actual interface: total marginal covariance/information plus factor counts/metadata.
- If source-wise estimator FIM is desired, implement it in `FGOInformationManager` by grouping active linearized factors by source and accumulating source-specific Hessian blocks.

Classification: **modify slide**, unless future work is explicitly to implement source-wise current-estimator FIM.

### 3. ESDF Claims Are Too Strong

Slide sections:

- 3.0.4 Data Contract
- 3.1.5 Localization Module Output Summary
- 3.4.5 Shared Multi-Layer Map
- 3.5.1/3.5.4/3.5.5b planning sections

Slide claim:

- The method includes occupancy plus ESDF.
- ESDF is described as an occupancy distance transform used by A*, B-spline obstacle gradient, and AL.

Code reality:

- EGO `plan_env` maintains occupancy and inflated occupancy buffers.
- EGO B-spline obstacle cost uses rebound control-point base points/directions in `calcDistanceCostRebound`, not a general ESDF layer.
- `UnifiedRiskVoxel` has `esdf_m`, but `phase2_planner_integrity_evaluator` fills it from `al.dist_to_obstacle_m` / clearance proxy.
- `LocalOccupancyGrid` is a voxel hash for ray occlusion and occupancy ratio; it does not compute an ESDF.

Correction for slide:

- Replace "ESDF" with "occupancy/inflated occupancy and obstacle-clearance proxy" for the implemented demo11 method.
- If keeping ESDF in the slide, label it as a future extension or planned improvement.
- Do not claim a full distance-transform ESDF exists unless an ESDF module is added and wired into EGO/URG.

Classification: **modify slide**. A full ESDF would be a reasonable code upgrade, but it is not present.

### 4. URG Is Full Rebuild, Not Incremental Rolling Active-Voxel Cache

Slide sections:

- 3.4.0 to 3.4.8
- 3.5.8 Runtime and Unknown-Risk Fallback Logic

Slide claim:

- The predicted PL map is a rolling local grid.
- Active voxels are sampled.
- Staleness and cache age drive fallback.
- The map builder avoids recomputing everything.

Code reality:

- `src/iap/src/iap/planner/unified_risk_grid.cpp` implements `UnifiedRiskGrid::reset(center, half_extent_x, half_extent_y, z_slices, resolution)`, which clears and reallocates cells.
- `phase2_planner_integrity_evaluator::rebuild_unified_risk_grid()` creates a local grid, calls `reset()`, and evaluates every voxel unless capped by `urg_max_voxels_per_update`.
- URG has per-voxel timestamps, staleness policy, direct query fallback, unknown penalties, and gradient modes, but no incremental grid shift or active-only update.
- `LocalOccupancyGrid` does have rolling eviction by distance/age, but that behavior belongs to occupancy storage, not the URG/PL map itself.

Correction for slide:

- Say "centered full-grid rebuild with staleness policy" for the implemented method.
- Reserve "rolling/incremental active-voxel update" for future work.
- If the slide must present rolling URG as the main contribution, update code to shift/reuse cells, recompute only newly exposed or stale cells, and validate update latency.

Classification: **modify slide now; slide design is better as future code direction**.

### 5. Demo11 Defaults Do Not Enable the Full v2 Method

Slide sections:

- 3.3 Advisory predictor
- 3.4 Predicted PL map / URG
- 3.5 Integrity-aware planning

Slide claim:

- The method reads like complete FIM-add + LiDAR advisory FIM + URG + unified PI is active in the demo flow.

Code reality:

`launch/demo11_ego_planner_integrity_corridor.launch.py` default arguments include:

- `phase2_use_advisory_fim_add=true`
- `phase2_use_lidar_advisory_fim=false`
- `phase2_use_unified_risk_grid=false`
- `phase2_pi_use_unified_advisory_pl=false`
- `planner_use_integrity_cost=true`
- `planner_use_integrity_front_search=true`
- `planner_use_integrity_global_search=true`

The v2 acceptance automation can run enabled mode with:

- `phase2_use_advisory_fim_add=true`
- `phase2_use_lidar_advisory_fim=true`
- `phase2_pi_use_unified_advisory_pl=true`
- `phase2_use_unified_risk_grid=true`

Correction for slide:

- Distinguish "demo11 default" from "v2 enabled/acceptance mode".
- Say default demo11 uses published integrity cost fields with FIM-add available, but full URG + unified advisory PI + LiDAR advisory FIM are optional enabled-mode features.

Classification: **modify slide**. If the paper/report wants the full method as the demo11 default, code/launch defaults should be updated and revalidated.

### 6. Current Monitor vs Advisory Predictor Separation Matches the Code

Slide sections:

- 3.0.5 Certified Monitoring vs Advisory Prediction
- 3.2 Current ARAIM Monitor
- 3.3 Advisory PL Predictor

Slide claim:

- Current ARAIM uses real measurements/residuals and produces certified current monitor PL/alarm.
- Future prediction is advisory and must not be called certified ARAIM.
- Current monitor fusion is conservative, while FIM-add belongs to advisory planning.

Code reality:

- `src/iap/src/iap/integrity/integrity_monitor.cpp` runs GNSS ARAIM and LiDAR ARAIM on current data.
- `run_lidar_araim()` fuses current certified monitor values by max:
  - `PL_E = max(PL_E, lidar_PL_E)`
  - `HPL`/`VPL` source labels switch to LiDAR only if LiDAR is worse.
- `FuturePLFieldPredictor` performs advisory FIM-add separately when enabled.
- `PICostAdapter` comments explicitly warn not to pass current certified monitor PL except compatibility/diagnostic modes.

Correction:

- Keep this slide structure.
- Tighten terminology to "GNSS certified monitor PL", "LiDAR certified monitor PL", "monitor-fused PL", and "advisory predicted PL".

Classification: **matches implementation**.

### 7. Demo11 Has A* and B-Spline Integrity Hooks

Slide sections:

- 2.5 Transition to Methodology
- 3.4.6 Planner Query Interface
- 3.5.1 to 3.5.6 Planning Module

Slide claim:

- A* front-end and B-spline back-end use integrity cost.
- Front-end and back-end are connected through planner-compatible PI fields.

Code reality:

- `path_searching/src/dyn_a_star.cpp` has `setIntegrityCostCallback()` and `setIntegrityCostParams()`.
- A* calls `integrity_cost_query_(neighbor_pos, &integrity_cost)` and includes the integrity term in edge cost.
- `bspline_opt/src/bspline_optimizer.cpp` subscribes to `/iap/integrity_cost_field`, parses `cost` and `grad_x/y/z`, and adds integrity cost/gradient in both rebound and refine optimization.
- `phase2_planner_integrity_evaluator.cpp` publishes backend and front-end cost fields as `sensor_msgs::msg::PointCloud2`.

Correction:

- Keep the A* + B-spline integration claim, but describe the implemented cost-field interface accurately:
  - front-end field is nearest-neighbor sampled,
  - backend field is nearest-sample control-point cost/gradient,
  - URG is optional rather than always the source.

Classification: **matches demo11 flow with caveats**.

### 8. A* Edge-Cost Formula Is More Advanced Than Current Code

Slide section:

- 3.5.2 Integrity-Aware A* Front-End

Slide claim:

- `c_int(e) = sum_{p_i in Sample(e)} J_PI(p_i) Delta s`.

Code reality:

- `dyn_a_star.cpp` queries the integrity field at `neighbor_pos` only.
- It multiplies static move cost by `1 + lambda_integrity_cost * integrity_cost`.
- It does not sample multiple points along each edge or integrate cost over edge length.

Recommended code improvement:

- Add edge sampling between current and neighbor node for longer step sizes.
- Aggregate mean or integral PI cost along the edge.
- Preserve existing single-sample behavior for small step sizes or as a fast mode.

Slide correction if code is not changed:

- Replace the edge integral formula with the implemented neighbor-node sampled multiplicative cost.

Classification: **slide method is better; update code if claiming this formula**.

### 9. B-Spline PI Cost Sampling/Gradient Is Simpler Than Slide Formula

Slide sections:

- 3.5.4 B-spline Back-End Objective
- 3.5.5 B-spline Trajectory and PI Cost Sampling
- 3.5.6 Gradient for Back-End Optimization

Slide claim:

- The optimizer samples `J_PI(p(t_r), t_r)` along the continuous B-spline trajectory.
- It obtains `J_PI(p)` by interpolation from the map.
- It applies the chain rule through B-spline basis functions.

Code reality:

- `BsplineOptimizer::calcIntegrityCost()` iterates over control point columns.
- For each control point, it finds the nearest published field sample within a radius.
- It adds `sample.cost` to the raw integrity cost and adds `sample.gradient` directly to the corresponding control point gradient.
- It does not evaluate De Boor trajectory samples for PI cost, does not trilinearly interpolate the field, and does not explicitly distribute gradients through B-spline basis weights.

Recommended code improvement:

- Sample the actual B-spline curve at fixed time/arc-length intervals.
- Query the PI field using bilinear/trilinear interpolation or URG `queryRisk()`.
- Accumulate `J_PI` over samples and distribute gradients to affected control points using B-spline basis values.

Slide correction if code is not changed:

- Describe the current backend as a control-point nearest-field penalty with supplied PI gradients.

Classification: **slide method is better; update code if claiming this formula**.

### 10. Shared URG Field Is Optional, Not the Default Shared Runtime Interface

Slide sections:

- 3.4.5 Shared Multi-Layer Map
- 3.5.5b Planner Query over Unified Risk Grid
- 3.5.8 Runtime fallback

Slide claim:

- A* and B-spline query the same URG.

Code reality:

- With `phase2_use_unified_risk_grid=false` default, `phase2_planner_integrity_evaluator` publishes legacy generated cost fields.
- With URG enabled and `urg_keep_legacy_topics=true`, URG can publish both front and backend cost fields.
- The planner itself still consumes PointCloud2 cost fields, not a typed URG API.

Correction for slide:

- Say: "The evaluator can materialize the advisory PI field as either legacy PointCloud2 cost fields or an optional URG-backed field; EGO Planner consumes the published fields."
- If the slide keeps "planner queries URG", qualify that it is true only indirectly through URG-generated fields, not a direct planner API.

Classification: **modify slide**.

## What the Slide Gets Right

- The monitor/predictor separation is implemented and important.
- Current monitor max-fusion is implemented and conservative.
- Future prediction is advisory, not certified ARAIM.
- FIM-add belongs to advisory prediction, not current monitor fusion.
- Demo11 includes integrity-aware A* and B-spline hooks in the EGO Planner path.
- The evaluator exports PL/AL/IM/PI diagnostics and cost-field topics for planning.

## Slide-Better Items Recommended for Code Updates

These methods are more rigorous than the current code and should be considered implementation upgrades if the slide is intended to describe final system behavior:

1. **A* edge integral**
   - Implement multi-point edge sampling for PI cost.
   - Keep current neighbor-only query as a fast fallback.

2. **B-spline continuous PI objective**
   - Query the PI field along the actual B-spline trajectory rather than only at control points.
   - Use interpolation instead of nearest-neighbor sample lookup.
   - Propagate gradients with B-spline basis functions.

3. **True rolling URG**
   - Shift/reuse cells when the center moves.
   - Recompute only new, stale, or invalid cells.
   - Keep the current full rebuild as fallback.

4. **Full ESDF layer**
   - Add an actual distance-transform layer if obstacle-gradient claims should be ESDF-based.
   - Feed both AL and optimizer obstacle gradients from that layer, or explicitly document the current EGO rebound-distance model.

5. **Source-wise current-estimator FIM**
   - If needed for current monitor diagnostics, extend `FGOInformationManager` to accumulate per-source Hessian blocks.
   - Otherwise keep source-wise FIM in advisory prediction only.

## Slide Text Changes Recommended Now

Recommended edits to `iap_slide_v2.tex`:

1. In 3.1.1, add ECEF anchor variables:
   - `E(0)` for world-origin ECEF position.
   - `R(0)` for world-to-ECEF rotation.

2. In 3.1.2, replace the world-frame pseudorange equation with the implemented ECEF equation and correction terms.

3. In 3.1.4b, change title from "Estimator Information Interface: Source-Wise FIM Export" to "Estimator Marginal Information and Advisory FIM Decomposition".

4. In 3.4 and 3.5, replace unqualified "ESDF" with "occupancy/inflated occupancy plus obstacle-clearance proxy", unless marking ESDF as future work.

5. In 3.4, change "rolling active-voxel PL map" to "centered full-grid rebuild with timestamps/staleness policy" for current implementation.

6. In 3.5, keep "A* + B-spline" for demo11, but replace exact integral/interpolation language with current field-consumer behavior or label the integral/interpolation equations as the target upgrade.

7. Add a note distinguishing:
   - demo11 default mode,
   - v2 enabled/acceptance mode.

## Final Verdict

The slide method is **partially aligned** with demo11.

- The architectural message is correct: current certified integrity monitoring is separated from future advisory planning, and demo11 does feed advisory integrity costs into EGO Planner.
- The implementation details in Ch3 should be revised where they imply stronger math/engineering than exists today: source-wise estimator FIM, ESDF, incremental rolling URG, default full URG mode, A* edge integration, and continuous B-spline PI interpolation.
- The best next action for documentation is to revise the slide to present the current demo11 implementation accurately, while marking the more rigorous A*/B-spline/URG formulations as planned upgrades unless those code changes are implemented and validated.
