# Audit: Slide Methodology (Ch3) vs IAP Code Implementation

**Date**: 2026-05-14
**Slide deck**: `src/iap/docs/stage1_v2.0/iap_slide_v2.tex`
**Code base**: `src/iap/`

---

## Summary

10 discrepancies found: 5 major, 3 moderate, 2 minor.

| # | Section | Claim | Severity |
|---|---------|-------|----------|
| 1 | 3.5.1–3.5.6 | A\* front-end + B-spline back-end planner | **Major** |
| 2 | 3.5.5b | Unified Risk Grid queried by planner | **Major** |
| 3 | 3.4.5, 3.5.4 | ESDF computation pipeline | **Major** |
| 4 | 3.4.2, 3.4.7 | Rolling PL/IM map with eviction | **Major** |
| 5 | 3.1.4b | Source-wise FIM decomposition by estimator | **Major** |
| 6 | 3.1.1 | State vector — missing ECEF anchor variables | Moderate |
| 7 | 3.4.0–3.4.8 | PL map construction in mapping module | Moderate |
| 8 | 3.3.9, 3.5.2 | Planner PI cost formula | Moderate |
| 9 | 3.2.1 | LiDAR PL 3-term vs 4-term formula | Minor |
| 10 | 3.4.4 | Active-voxel sampling optimisation | Minor |

---

## Discrepancy 1: A\* + B-spline Back-End → Motion-Primitive Enumeration

**Severity**: Major

### Slide claim (sections 3.5.1–3.5.6)

> "A\* front-end searches a collision-free initial path" (3.5.1)
> "B-spline back-end optimizes smoothness, dynamics, obstacle clearance, and PI cost" (3.5.1)
> `p(t) = Σ B_i^k(t) Q_i` — B-spline position trajectory (3.5.5)
> Chain-rule gradient: `∂J/∂Q_i = Σ ∇_p J_PI(p(t_r)) B_i^k(t_r) Δt_r` (3.5.6)
> "front-end and back-end query the same unified risk grid" (3.5.5b)

### Code reality

`IntegrityPlanner` (`src/iap/include/iap/planner/integrity_planner.hpp`) is pure
motion-primitive enumeration. `TrajectoryGenerator::generate()` iterates over a
grid of (forward-speed × yaw-rate × altitude-rate), integrates forward with
`dt = 0.2 s`, and returns `CandidateTrajectory`.

`src/iap/include/iap/planner/trajectory_generator.hpp:16` explicitly states:
> "Extensible to spline / MINCO (IAP-RQ-300 full upgrade) by replacing generate()."

A grep for `spline`, `bspline`, `B.spline`, or `MINCO` across the entire codebase
returns **zero results**. No B-spline, no control-point optimisation, no
chain-rule gradient w.r.t. control points exist.

### Suggested correction

Replace all B-spline claims in the slides with an accurate description of the
motion-primitive planner, and label B-spline back-end as **future work**
(IAP-RQ-300). Remove sections 3.5.4, 3.5.5, 3.5.6, or clearly mark them as
"planned / not yet implemented."

---

## Discrepancy 2: Unified Risk Grid Not Integrated With Planner

**Severity**: Major

### Slide claim (3.5.5b)

> "A\* front-end: for each expanded node p_j, check occupancy / ESDF, query c_PI(p_j) from URG"
> "B-spline back-end: query PI cost by trilinear interpolation from URG"
> "front-end and back-end query the same unified risk grid"

### Code reality

`IntegrityPlanner` (`src/iap/src/iap/planner/integrity_planner.cpp`) does not
reference `UnifiedRiskGrid`, `FuturePLFieldPredictor`, or `PLGrid` at all.
The planner uses its own inlined cost function in `evaluate()`:

- `PredictedIntegrityComputer` — simple isotropic covariance growth
  (`sigma(t+dt) = sqrt(sigma(t)^2 + sigma_grow_eff^2 * dt)`)
- `PredictedAraimComputer` — geometry-only GNSS advisory PL at waypoints

`UnifiedRiskGrid`, `FuturePLFieldPredictor`, and `PLGrid` are implemented
(`src/iap/src/iap/planner/unified_risk_grid.cpp`, `future_pl_field_predictor.cpp`,
`pl_grid.cpp`) but are **orphaned** — only referenced in the standalone
`phase2_planner_integrity_evaluator` app, not in the runtime planner.

### Suggested correction

Either: (a) wire `UnifiedRiskGrid` into `IntegrityPlanner`, or (b) remove URG
claims from the slides and describe the actual direct point-predictor approach.
Option (b) is recommended for the current slide deck, with a roadmap note that
URG integration is planned.

---

## Discrepancy 3: ESDF Computation Pipeline Does Not Exist

**Severity**: Major

### Slide claim (multiple sections)

> `M_map(p) = {Occ(p), ESDF(p), AL(p), Λ^G(p), Λ^L(p), PL^{pred}(p), IM(p), J_PI(p)}` (3.4.5)
> "ESDF — LiDAR occupancy distance transform — used by A\*, B-spline obstacle gradient, AL" (3.4.5 table)
> "J_obs — Obstacle avoidance cost from ESDF" (3.5.4)

ESDF is presented as a computed map layer throughout Ch3.

### Code reality

`UnifiedRiskVoxel` has an `esdf_m` field (`src/iap/include/iap/planner/unified_risk_grid.hpp:27`),
but this field is populated externally from trunk-clearance distances, not from a
true Euclidean Signed Distance Field computation.

A grep for `ESDF`, `signed.distance`, or `distance.transform` across all IAP source
files returns only `unified_risk_grid.cpp`. There is **no ESDF computation module**
anywhere in the codebase. No distance transform, no brushfire algorithm, no
marching-parabolas, no FMM-based SDF.

The only "distance" available is `al.dist_to_obstacle_m` from the `AlertLimitModel`
(trunk clearance geometry).

### Suggested correction

Remove ESDF from the slide's map layer definitions. Replace with "obstacle
clearance distance" (from trunk geometry or occupancy grid) and note that a
full ESDF pipeline is future work. Or implement an ESDF module and integrate it.

---

## Discrepancy 4: Rolling PL/IM Map → Static Grid Reset

**Severity**: Major

### Slide claim (3.4.2, 3.4.7)

> "Rolling local grid centered around the UAV" with half-extents `L_x, L_y, L_z` (3.4.2)
> "stamp_i — Last update time of the voxel; used for staleness checking" (3.4.2)
> "valid(p_i) = (t_now − stamp_i ≤ T_max_age) ∧ (p_i ∈ G_t)" (3.4.7)
> Staleness-based fallback with `c_unknown(p) = λ_u (1 − exp(−Δt(p)/τ_u))` (3.5.8)

### Code reality

`PLGrid::reset(center, sx, sy, sz, res)` (`src/iap/src/iap/planner/pl_grid.cpp`)
completely rebuilds the grid from scratch at a new center. There is no sliding
window, no incremental update as the UAV moves, no per-voxel timestamps, and no
staleness eviction.

`FuturePLFieldPredictor::rebuild_grid()` calls `evaluate_point()` for every cell
in the new grid. No rolling/incremental update.

`UnifiedRiskGrid` has `updated_time_s` and staleness logic (`apply_unified_risk_stale_policy`),
but it is not used by the planner.

The only module with true rolling behaviour is `LocalOccupancyGrid`
(`src/iap/include/iap/map/local_occupancy.hpp`), which supports distance/age-based
eviction. But this is occupancy-only, not PL/IM.

### Suggested correction

Either implement incremental rolling updates in `PLGrid` (shift cells, recompute
only newly-entered voxels), or remove "rolling" claims from the slides and
describe the current behaviour as "periodic full-grid rebuild."

---

## Discrepancy 5: Source-Wise FIM Decomposition Not Exported by Estimator

**Severity**: Major

### Slide claim (3.1.4b)

> `Λ_0 = Λ^{prior} + Λ^{IMU} + Λ^G + Λ^L`
> `Λ^G = Σ_s J_s^⊤ R_s^{−1} J_s`,  `Λ^L = Σ_{j,ℓ} J_{j,ℓ}^⊤ R_{j,ℓ}^{−1} J_{j,ℓ}`
> "Source-wise information supports ARAIM downdate, advisory prediction, calibration, and debugging."

The slide claims the estimator exports decomposed information matrices per source.

### Code reality

`FGOPositionInfo` (`src/iap/include/iap/integrity/fgo_information_matrix.hpp`)
contains:
- `lambda_p` — total 3×3 position information matrix (`Σ_p^{−1}`)
- `sigma_p` — 3×3 position covariance from iSAM2 marginal
- `pose_cov_6x6` — full 6×6 pose covariance block
- Per-source **factor counts** (n_gnss_factors, n_trunk_factors, n_imu_factors,
  n_clock_factors) — but these are counts, not information matrices.

`FGOInformationManager::extract()` (`src/iap/src/iap/integrity/fgo_information_manager.cpp`)
extracts the marginal covariance via `smoother.marginalCovariance(X(frame_id))`
and inverts it to get `lambda_p`. It does **not** decompose by source.

Source-wise FIM decomposition exists only in `FuturePLFieldPredictor::evaluate_point()`
(`src/iap/src/iap/planner/future_pl_field_predictor.cpp:347-456`), where `Lambda_prior`,
`Lambda_gnss`, and `Lambda_lidar` are computed separately for **advisory** prediction
— not extracted from the current FGO estimator.

### Suggested correction

Three options:
1. Implement source-wise FIM export in `FGOInformationManager` and update slides.
2. Remove the source-wise decomposition claims from the estimator section (3.1.4b)
   and move them to the advisory predictor section where they belong.
3. Clarify that source-wise decomposition is advisory-only, not from the current
   certified monitor.

---

## Discrepancy 6: State Vector Missing ECEF Anchor Variables

**Severity**: Moderate

### Slide claim (3.1.1)

> `x_k = [T_k, v_k, b^a_k, b^g_k, δt^{clk}_k, δ̇t^{clk}_k]`

### Code reality

The GNSS extension (`src/iap/src/iap/gnss/gnss_extension.cpp:3-7`) introduces two
additional shared factor-graph variables that self-calibrate the world↔ECEF transform:

- `E(0)` — ECEF origin coordinates (Vector3), estimated online
- `R(0)` — world→ECEF rotation (Rot3), estimated online with a loose prior

These are critical for the GNSS factor graph (pseudorange and Doppler factors
operate in ECEF) and are a significant architectural feature — they eliminate the
need for a hard-coded ENU reference point and allow the optimiser to correct IMU
initial heading error automatically.

The slide's state vector omits both variables entirely. Section 3.1.2 on GNSS
pseudorange factors also does not mention that `p_k` must go through the ECEF
transform before the range is computed.

### Suggested correction

Add `E(0)` and `R(0)` to the state definition in slide 3.1.1, or at minimum add
a footnote noting the self-calibrating ECEF anchor as shared graph variables.

---

## Discrepancy 7: PL Map Construction Located in Mapping Module

**Severity**: Moderate

### Slide claim (3.4.0–3.4.8)

The entire section 3.4 is titled "Predicted PL Map Construction and Planner
Interface" and immediately follows the predictor section (3.3). The slide
presents it as a mapping-layer component:

> "Build a rolling, cached, multi-layer Unified Risk Grid (URG). The map builder
> samples active voxels, stores PL_pred, AL, IM, c_PI, age, and flags, then
> serves the same field to A\* and B-spline." (3.4.0)

### Code reality

`PLGrid`, `UnifiedRiskGrid`, and `FuturePLFieldPredictor` all live in
`src/iap/src/iap/planner/` and `src/iap/include/iap/planner/`. They are planner-layer
constructs, not mapping-layer.

The mapping module (`src/iap/src/iap/mapping/`, `src/iap/include/iap/mapping/`)
contains:
- `sub_mapping.cpp` / `global_mapping.cpp` — geometric submaps and pose-graph
- `local_occupancy.hpp` — voxel-hash occupancy grid for ray-casting

No PL, IM, AL, or PI cost computation exists in the mapping module.

### Suggested correction

Relabel the PL map construction as a planner-layer component in the slides.
Alternatively, if the intent is truly a mapping-layer service, implement it
there. Either way, the slide's architecture diagram should show the correct
module boundaries.

---

## Discrepancy 8: Planner PI Cost Formula Mismatch

**Severity**: Moderate

### Slide claim (3.3.9, 3.5.2)

> `J_PI(p,t) = λ_r [PL^{pred}(p,t) − AL(p) + m]²_+ + λ_ratio (PL^{pred}(p,t) / (AL(p) + ε))²` (3.3.9)
> A\* edge cost: `c_int(e) = Σ_{p_i∈Sample(e)} J_PI(p_i) Δs` (3.5.2)

### Code reality

`IntegrityPlanner::evaluate()` (`src/iap/src/iap/planner/integrity_planner.cpp`)
uses a different formula:

```cpp
// Soft zone: ratio ∈ [0.8, 1.0) → (ratio − 0.8)²
// Hard zone: ratio ≥ 1.0    → (ratio − 1.0)² + infeasibility flag
J = w_integrity * Σ_k hinge(HPL_pred_k / AL_k − 1)²
```

Key differences:
1. The code uses **ratio** `HPL/AL`, not absolute difference `PL − AL`.
2. The code has a **soft zone** starting at 0.8 (not present in slides).
3. The slide's `λ_ratio·(PL/AL)²` soft-preference term is absent from the planner.
4. `PICostAdapter` (`src/iap/include/iap/planner/pi_cost_adapter.hpp`) does implement
   the slide's formula, but it is **not called by** `IntegrityPlanner`.

### Suggested correction

Update the slide formula to match the actual planner code (HPL/AL ratio hinge
with soft/hard zones). Or update the planner to use `PICostAdapter` and match
the slide. The code and docs should agree.

---

## Discrepancy 9: LiDAR PL Formula — 3 Terms in Slide, 4 Terms in Code

**Severity**: Minor

### Slide claim (3.2.1)

> `PL_{q,f} = |d_{q,f}| + K_{fa,f}·σ_{ss,q,f} + K_{md,f}·σ_{q,f} + b_{q,f}`

The slide's ARAIM recap shows 3 main terms (separation + FA margin + MD margin)
with an optional bias `b_{q,f}` mentioned but not emphasised.

### Code reality

The LiDAR ARAIM (`src/iap/src/iap/integrity/lidar_araim.cpp:377-458`) computes a
**4-term** PL where the 4th term `bias_H` / `bias_V` is derived from the block
risk score `γ_total`:

```cpp
bias_H = alpha_H * gamma_mode;  // γ_total from weighted sum of RMSE, inlier,
bias_V = alpha_V * gamma_mode;  // condition, and age risk components
```

This bias term is significant — it directly translates block-level quality
metrics into meter-level PL inflation. The slide's GNSS ARAIM formula (3.2.2)
has only 3 terms without the block-quality bias, and the LiDAR bias overbound
discussion is separated into its own slide (3.2.5).

### Suggested correction

The slide is not strictly wrong since `b_{q,f}` is listed in the formula, but it
under-emphasises the LiDAR-specific bias term. Add a note in 3.2.1 that LiDAR
ARAIM uses a 4th bias term derived from block quality metrics, distinguishing it
from GNSS ARAIM's 3-term formula.

---

## Discrepancy 10: Active-Voxel Sampling Not Implemented

**Severity**: Minor

### Slide claim (3.4.4)

> "Active-voxel mode — Used for online planning. Free voxels only, current local
> corridor around the nominal path, A\* search frontier or visited nodes, B-spline
> trajectory samples and neighbors, high-risk boundary region from previous map."

### Code reality

`FuturePLFieldPredictor::rebuild_grid()` (`src/iap/src/iap/planner/future_pl_field_predictor.cpp:208`)
calls `evaluate_point()` for **every cell** in the grid:

```cpp
for (int ix = 0; ix < nx(); ++ix)
  for (int iy = 0; iy < ny(); ++iy)
    for (int iz = 0; iz < nz(); ++iz)
      evaluate_point(...);
```

There is no active-voxel filtering, no corridor-restricted computation, no
frontier-only sampling. The slide's "full-grid mode" vs "active-voxel mode"
distinction does not exist in code.

### Suggested correction

Either implement active-voxel sampling (significant performance win for online
use), or remove the active-voxel claims and document the current full-grid
behaviour. Given that a full grid at the stated resolution
(30×30×8 m at 1 m → 7200 voxels) is manageable at 1–2 Hz, this is a minor
optimisation gap rather than a correctness issue.

---

## Items That Match (for completeness)

The following slide claims are **consistent** with the code:

| Section | Claim | Code confirmation |
|---------|-------|-------------------|
| 3.0.3 | Sliding-window MAP estimation with IMU + LiDAR + GNSS + prior factors | `OdometryEstimationCPU` with iSAM2 |
| 3.0.4 | Snapshot data contract `S_t` | `IntegritySnapshot` + `IntegritySnapshotBuilder` |
| 3.0.5 | Certified monitoring vs advisory prediction separation | Clear architectural split: `IntegrityMonitor` vs `PredictedAraimComputer` |
| 3.1.2 | GNSS pseudorange residual formula | `pseudorange_factor.cpp` — ECEF range + clock + iono + tropo |
| 3.1.3 | LiDAR VGICP factor-block abstraction | `LidarAraimBlock` with `Lambda_B`, `eta_B` per VGICP factor |
| 3.1.4 | Conservative covariance inflation | `η_Σ ≥ 1` factor applied in `fgo_information_manager.cpp` |
| 3.2.1 | ARAIM solution separation formula | `araim.cpp:387-390` — exact 3-term match for GNSS |
| 3.2.2 | GNSS hypothesis library (H₀, H_s, H_c) | `araim.cpp:164-214` — SAT + CONSTELLATION + TRUNK hypotheses |
| 3.2.3 | LiDAR hypothesis library (H_source, H_target, H_level) | `lidar_araim.cpp:252-299` — exact match |
| 3.2.4 | Snapshot downdate for subset solutions | `lidar_araim.cpp:382-389` — `Λ_f = Λ_0 − Σ ΔΛ`, `η_f = η_0 − Σ Δη` |
| 3.2.5 | LiDAR block risk score `γ_{j,ℓ}` | `lidar_araim.cpp:215-249` — RMSE + inlier + condition + age |
| 3.2.5 | Bounded age model `γ_age = 1 − exp(−Age/τ_age)` | `lidar_araim.cpp:237-240` — configurable via `AgeModel::EXPONENTIAL` |
| 3.2.6 | Current monitor max fusion `PL_mon = max(PL_G, PL_L)` | `integrity_monitor.cpp:271-286` — per-axis max |
| 3.2.6 | Advisory FIM-add `Λ_pred = Λ_prior + Λ_G + Λ_L` | `future_pl_field_predictor.cpp:347-456` — advisory path only |
| 3.3.3 | GNSS geometry matrix and effective variance | `visibility_predictor.cpp` + `canopy_noise_model.hpp` |
| 3.3.8 | Unified predicted information fusion | `FuturePLFieldPredictor::evaluate_point()` FIM-add path |
| 3.3.9 | Alert limit from obstacle clearance | `integrity_monitor.cpp:54-123` — HAL from trunk, VAL from altitude |

---

## Recommended Slide Corrections (Priority Order)

1. **Remove B-spline claims** (sections 3.5.4–3.5.6). Replace with accurate
   motion-primitive description. Mark B-spline as IAP-RQ-300 future work.

2. **Fix planner-module claims** (3.5.1–3.5.3). Replace "A\* front-end" with
   "motion-primitive enumeration", and remove all URG-query pseudo-code (3.5.5b).
   Describe the actual direct point-predictor approach.

3. **Replace "ESDF" with "obstacle clearance"** throughout. ESDF is not computed.
   The actual clearance comes from trunk geometry and occupancy checks.

4. **Remove "rolling" from PL map description** (3.4.2, 3.4.7). Describe as
   "periodic full-grid rebuild" with staleness-based fallback.

5. **Move source-wise FIM claims** from estimator section (3.1.4b) to advisory
   predictor section (3.3), or implement source-wise FIM export in the estimator.

6. **Add E(0)/R(0) to state definition** (3.1.1) to reflect the self-calibrating
   ECEF anchor.

7. **Relabel PL map construction** as a planner-layer component (3.4), not a
   mapping-layer one.

8. **Align PI cost formula** (3.3.9, 3.5.2) with the actual planner implementation
   (HPL/AL ratio hinge with soft/hard zones).

9. **Note LiDAR-specific 4th bias term** in the PL formula explanation (3.2.1).

10. **Remove active-voxel claims** (3.4.4) or implement the optimisation.
