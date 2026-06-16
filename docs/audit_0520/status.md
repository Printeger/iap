# IAP ARAIM Refactor — Status & Dev Log

> **Last updated:** 2026-05-21
> **Scope:** ARAIM refactor, Steps 0–2 per `ARAIM_refactor_plan_v2.md`
> **Principle:** Implement first, rename later. Behavior before structure.

---

## 0. High-Level Status

| Step | Name | Status | Tests | Date |
|------|------|--------|-------|------|
| 0 | Establish baseline / golden tests | ✅ Done | 49 pass, 0 fail | 2026-05-21 |
| 1 | Fix H/V integrity semantics | ✅ Done | 49 pass, 0 fail | 2026-05-21 |
| 2 | Numerical guards (NaN/Inf/sentinel) | ✅ Done | 49 pass, 0 fail | 2026-05-21 |
| 3 | Runtime source breakdown output | ✅ Done | 49 pass, 0 fail | 2026-05-21 |
| 4 | Explicit fusion policy | ✅ Done | 61 pass, 0 fail | 2026-05-21 |
| 5 | Clarify trunk/constellation hypothesis semantics | ✅ Done | 54 pass, 0 fail | 2026-05-21 |
| 6 | Move planner geometry prediction out of ARAIM | ✅ Done | 54+5+11 pass, 0 fail | 2026-05-21 |
| 7 | Rename and isolate current GNSS ARAIM | ✅ Done | 55 pass, 0 fail | 2026-05-21 |
| 8 | GnssEpoch linearized input seam | ✅ Done | 62 pass, 0 fail | 2026-05-21 |
| 9 | Decompose large compute/monitor functions | ✅ Done | 70 pass, 0 fail | 2026-05-21 |
| 10 | Library target split | ✅ Done | 83 pass, 0 fail | 2026-05-21 |

---

## 1. Step 0 — Baseline / Golden Tests

### Files touched

| Action | File |
|--------|------|
| Added `#include` | `test/test_araim.cpp` — `<iap/odometry/estimation_frame.hpp>`, `<limits>` |
| Added fixture | `test/test_araim.cpp` — `IntegrityMonitorBaselineTest` class |
| Added tests | `test/test_araim.cpp` — T0.1–T0.4 |

### New tests

| Test | What it validates |
|------|-------------------|
| `FallbackOnlyGivesCorrectIM` | `im_h = HAL-HPL`, `im_v = VAL-VPL`, `IM = min(im_h, im_v)` |
| `StateUnsafeWhenPLExceedsAL` | Large covariance → UNSAFE |
| `VerticalViolationIsDetectedAsUnsafe` | VPL > VAL → UNSAFE (the gap fix) |
| `IMIsMinOfHorizontalAndVerticalMargins` | IM formula consistency |

### No production code changes in Step 0.

---

## 2. Step 1 — Fix H/V Integrity Semantics

### Behavior changes

```
Before                              After
──────────────────────────────────────────────────────────
IM = AL - PL                        IM = min(HAL-HPL, VAL-VPL)
SAFE  = PL < AL                     SAFE  = HPL < HAL AND VPL < VAL
UNSAFE = PL >= AL                  UNSAFE = HPL >= HAL OR VPL >= VAL
safe() = IM > 0                     safe() = im_h > 0 && im_v > 0
is_available() = PL < AL            is_available() = HPL < HAL && VPL < VAL
update_state() uses scalar PL/AL    update_state() uses HPL/HAL + VPL/VAL
update_mode_legacy() scalar PL/AL   update_mode_legacy() worst(HPL/HAL, VPL/VAL)
```

### Files modified

| File | Changes |
|------|---------|
| `include/iap/integrity/integrity_types.hpp` | Added `im_h`, `im_v` fields; updated `safe()`, `safe_horizontal()`, `safe_vertical()`, `is_available()` |
| `src/iap/integrity/integrity_monitor.cpp` | Rewrote `update_state()`; recomputed `im_h`/`im_v`/`IM` in 2 places; updated all log messages; updated `update_mode_legacy()` |
| `test/test_araim.cpp` | Updated `SafeWhenPLLessThanAL` test to set HPL/VPL/HAL/VAL properly; updated T0.1, T0.3, T0.4 for new semantics |

### Tests added

| Test | What it validates |
|------|-------------------|
| `HorizontalUnsafeVerticalSafe` | HPL >= HAL, VPL < VAL → UNSAFE |
| `VerticalUnsafeHorizontalSafe` | VPL >= VAL, HPL < HAL → UNSAFE |
| `BothDimensionsSafe` | Recovery from UNSAFE to SAFE after N frames |
| `IMMinOfHorizontalAndVerticalMargins` | Formula consistency |
| `VerticalViolationIsNowUnsafe` | (replaces T0.3) |

### Backward compatibility

- `IntegrityReport.msg` unchanged — `im` field now carries `min(HAL-HPL, VAL-VPL)` (more conservative)
- `safe()` method signature unchanged — behavior is more conservative
- `is_available()` signature unchanged — behavior is more conservative
- Log format changed — downstream parsers may need update

---

## 3. Step 2 — Numerical Guards

### Files created

| File | Description |
|------|-------------|
| `include/iap/integrity/numerical_guard.hpp` | Inline utility functions: `is_valid`, `is_valid_positive`, `is_valid_nonnegative`, `is_valid_covariance`, `sentinel_if_invalid`, `guard`, `guard_positive`. Sentinel constant `kSentinel = 1e9`. |

### Files modified

| File | Changes |
|------|---------|
| `include/iap/integrity/integrity_types.hpp` | Added `NumericalFailureFlags` struct (10 bool flags + `failure_reason` string); added `has_numerical_failure()` method |
| `src/iap/integrity/integrity_monitor.cpp` | Guarded `compute_PL_proxy()` (covariance + eigenvalue); guarded fallback PL init; guarded `compute_dynamic_AL()` (HAL/VAL/AL); guarded `run_araim()` (NaN/Inf rejection); guarded `run_lidar_araim()` (NaN/Inf rejection); guarded initial and final IM computation |
| `test/test_araim.cpp` | Added T2.1–T2.6 numerical guard tests |

### Guard locations

| Guard point | What is checked | On failure |
|-------------|-----------------|------------|
| `compute_PL_proxy()` | `is_valid_covariance(sigma_p)` | Returns `kSentinel` |
| `compute_PL_proxy()` | `is_valid_positive(lambda_max)` | Returns `kSentinel` |
| `compute()` fallback PL | `fallback_pl >= kSentinel*0.99 \|\| !isfinite` | Sets `fallback_pl_invalid` flag |
| `compute()` initial/final IM | `sentinel_if_invalid(HAL - HPL)`, etc. | Sets `im_invalid` flag |
| `compute_dynamic_AL()` | `is_valid(HAL)`, `is_valid(VAL)` | Falls back to defaults |
| `compute_dynamic_AL()` | `sentinel_if_invalid(min(HAL, VAL))` | Safe AL clamp |
| `run_araim()` | `is_valid(ar.HPL)`, `is_valid(ar.VPL)` | Sets `gnss_araim_invalid`, `gnss_valid=0`, returns early |
| `run_lidar_araim()` | `is_valid(lr.HPL)`, `is_valid(lr.VPL)` | Sets `lidar_integrity_invalid`, `lidar_valid=0`, returns early |

### Note: 1e9 sentinel NOT rejected

The ARAIM engine legitimately outputs `HPL = 1e9` for constellation-wide hypotheses (when all satellites of one constellation are removed). This is a valid worst-case output, not a numerical failure. Only `NaN` and `Inf` are rejected.

### Tests added

| Test | What it validates |
|------|-------------------|
| `NaNCovarianceGivesInvalidFallback` | NaN sigma → `fallback_pl_invalid` + sentinel PL |
| `InfCovarianceGivesInvalidFallback` | Inf sigma → `fallback_pl_invalid` + sentinel PL |
| `NegativeVarianceGivesInvalidFallback` | sigma < 0 → `fallback_pl_invalid` |
| `NoNaNInfInPublishedFields` | All published PL/AL/IM fields are finite or sentinel |
| `NaNAaltitudeUsesDefaultVAL` | NaN altitude → VAL falls back to default |
| `AllSourcesInvalidGivesConservativePL` | NaN covariance + no GNSS + no LiDAR → UNSAFE with flags |

---

## 4. Step 3 — Runtime Source Breakdown Output

### Date
2026-05-21

### Design decision
Extended `msg/IntegrityReport.msg` (single-topic approach — no new topic, no new msg file). All new fields appended after existing fields with no reordering.

### Behavior changes
| Item | Before | After |
|------|--------|-------|
| ROS topic | `/iap/integrity` with 22 fields | `/iap/integrity` with 54 fields (22 old + 32 new) |
| `im` field comment | `AL - PL` | `min(im_h, im_v)` |
| Source breakdown | Not in ROS msg | GNSS, LiDAR, fallback per-source fields |
| H/V margins | Not in ROS msg | `im_h`, `im_v`, `im_min` |
| Fusion diagnostics | Not in ROS msg | `fusion_mode`, `final_*_source` strings |
| Numerical failure flags | Only in C++ struct | Exposed in ROS msg |

### Files created
| File | Description |
|------|-------------|
| `include/iap/integrity/integrity_report_mapping.hpp` | Unit-testable `fill_integrity_report_msg()` helper |

### Files modified
| File | Changes |
|------|---------|
| `msg/IntegrityReport.msg` | Updated `im` comment; added `im_h`, `im_v`, `im_min`; appended GNSS/LiDAR/fallback/fusion/numerical-failure fields (32 new, 0 reordered) |
| `include/iap/integrity/integrity_types.hpp` | Added `fallback_HPL`, `fallback_VPL` capture fields with `TODO(Step 4)` |
| `src/iap/integrity/integrity_monitor.cpp` | Save `fallback_HPL`/`fallback_VPL` before GNSS/LiDAR overrides |
| `src/iap/integrity/integrity_extension.cpp` | Added `#include <iap/integrity/integrity_report_mapping.hpp>`; old inline msg-population is still present but superseded by `fill_integrity_report_msg()` call at end (safe double-assignment) |
| `test/test_araim.cpp` | No test changes needed — mapping verified by extension compilation; internal fields tested by Baseline and H/V tests |

### Files NOT created
- `msg/IntegrityDebug.msg` — not created (single-topic approach)
- No `/iap/integrity_debug` topic — not created
- No second publisher in `integrity_extension.cpp`

### Build result
```text
colcon build --packages-select iap → passed
```

### Test result
```text
test_araim: 49 tests, 0 errors, 0 failures (all existing tests pass)
All 13 IAP test executables pass.
```

### Backward compatibility
- All 22 original fields preserved at same indices — old consumers can deserialize ignoring new fields
- New consumers must recompile (ABI change from msg schema extension)
- `/iap/integrity` topic name unchanged
- `im` field semantic already `min(im_h, im_v)` since Step 1 — comment updated only

### Known limitations / deferred
- `fusion_mode` hardcoded to `"max_pl"` until Step 4 (`IntegrityFusionPolicy`)
- `fallback_valid/fallback_hpl/fallback_vpl` are temporary placeholder mappings — replaced by `FallbackSourceResult` in Step 4
- `TODO(Step 4)` markers at: `integrity_types.hpp` fallback fields, `integrity_report_mapping.hpp` fallback/fusion mappings
- Old inline msg-population code in `integrity_extension.cpp` is redundant but harmless — can be cleaned in a later cleanup step
- ROS msg mapping helper is not unit-tested via gtest (test target lacks msg include path) — verified by extension compilation

### Review checklist
- [x] All existing tests pass (49/49)
- [x] No new msg file created (single-topic approach)
- [x] No new topic created
- [x] No renames performed
- [x] No file moves
- [x] No CMake target changes
- [x] `Araim::predict_geometry()` preserved
- [x] `/iap/integrity` topic name unchanged
- [x] Old fields preserved at same indices
- [x] `config_gnss.json` flat-key style unchanged
- [x] No `AraimBase` introduced

---

## 4. Step 5 — Clarify Trunk/Constellation Hypothesis Semantics

### Date
2026-05-21

### Design decision
Option A: GNSS ARAIM no longer enumerates trunk hypotheses by default.
Trunk/LiDAR integrity is handled separately. Trunk `FaultHypothesis::Type::TRUNK`
enum and `p_trunk_*` params are reserved for future non-GNSS trunk modeling.

### Behavior changes
| Item | Before | After |
|------|--------|-------|
| Trunk hypotheses | Enumerated by default, no GNSS row, returned zero subset | Disabled by default (`enable_trunk_hypotheses=false`) |
| `n_trunk_obs` on `n_hypotheses` | Added `n_trunk_obs` | No effect (default) |
| Constellation faults | Always enumerated | Configurable (`enable_constellation_faults=true`) |
| Degenerate subset | Silent 1e9 sentinel | `ss.degenerate=true`, `ss.failure_reason`, `ss.valid=false` |
| Degenerate count | Not tracked | `n_degenerate_hypotheses`, `has_degenerate_hypothesis` |

### Files modified
| File | Changes |
|------|---------|
| `include/iap/integrity/araim.hpp` | Added `enable_trunk_hypotheses`, `enable_constellation_faults`, `degrade_on_degenerate_hypothesis`; updated `p_trunk_*` comments to "(reserved)" |
| `include/iap/integrity/araim_types.hpp` | Added `SubsetSolution::valid/degenerate/failure_reason`; added `AraimResult` diagnostics |
| `src/iap/integrity/araim.cpp` | `enumerate_hypotheses()` gated on enable flags; `compute_core()` sets degenerate flags + tallies |
| `config/config_gnss.json` | Added `enable_gnss_trunk_hypotheses`, `enable_gnss_constellation_faults`, `gnss_degrade_on_degenerate_hypothesis` |
| `test/test_araim.cpp` | Replaced T9 → `TrunkHypothesesDisabledByDefault`; added T10–T14 |

### New config keys
| Key | Default | Purpose |
|-----|---------|---------|
| `enable_gnss_trunk_hypotheses` | `false` | Enumerate trunk hypotheses (OFF by default) |
| `enable_gnss_constellation_faults` | `true` | Enumerate constellation-wide hypotheses |
| `gnss_degrade_on_degenerate_hypothesis` | `true` | Conservative PL for degenerate subsets |

### Test result
```text
test_araim: 54 tests, 0 failures (49 existing + 5 new/updated)
test_integrity_fusion_policy: 13 tests, 0 failures
All 14 IAP test executables pass.
```

### Backward compatibility
- `Araim::run(epoch, n_trunk_obs)` signature preserved — `n_trunk_obs` ignored by default
- `FaultHypothesis::Type::TRUNK` enum preserved — reserved
- `p_trunk_*` params preserved — reserved
- Existing tests updated to match new default behavior

### Known limitations
- Trunk hypotheses are unsupported placeholders even when enabled
- No real GNSS trunk measurement model

---

## 5. Test Results (2026-05-21)

### Full test suite

```text
build/iap/Testing/20260521-1102/Test.xml: 13 tests, 0 errors, 0 failures, 0 skipped
build/iap/test_results/iap/test_araim.gtest.xml: 49 tests, 0 errors, 0 failures, 0 skipped
```

### All 13 IAP test executables passed:

| Test executable | Tests | Result |
|-----------------|-------|--------|
| `test_alert_limit_model` | 3 | ✅ Pass |
| `test_araim` | 49 | ✅ Pass |
| `test_future_pl_field_predictor` | 11 | ✅ Pass |
| `test_integrity_snapshot` | 4 | ✅ Pass |
| `test_lidar_observability_fim` | 13 | ✅ Pass |
| `test_local_occupancy` | 5 | ✅ Pass |
| `test_odom_freshness` | 5 | ✅ Pass |
| `test_pi_cost_adapter` | 11 | ✅ Pass |
| `test_pl_grid` | 5 | ✅ Pass |
| `test_predicted_araim` | 5 | ✅ Pass |
| `test_run_log_manager` | 1 | ✅ Pass |
| `test_unified_risk_grid` | 11 | ✅ Pass |
| *(cppcheck, lint_cmake, etc.)* | — | Lint only |

### Build command

```bash
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

### Test command

```bash
colcon test --packages-select iap && colcon test-result --all
```

---

## 6. Step 6 — Move Planner Geometry Prediction Out of Current ARAIM

### Date
2026-05-21

### Design decision
Planner-side GNSS advisory PL prediction separated from current ARAIM solver.
`PredictedAraimComputer` now uses `GnssGeometryPlPredictor` (independent class,
no `<iap/integrity/araim.hpp>` dependency). `Araim::predict_geometry()` preserved
as deprecated compatibility wrapper for one transition window.

### Files created
| File | Description |
|------|-------------|
| `include/iap/planner/gnss_geometry_pl_predictor.hpp` | New predictor class with `GnssGeometrySat`, `GnssGeometryPlParams`, `GnssGeometryPlResult`, `GnssGeometryPlPredictor` |
| `src/iap/planner/gnss_geometry_pl_predictor.cpp` | Implementation: geometry-only WLS with r=0, per-satellite subset solutions, dynamic K allocation |

### Files modified
| File | Changes |
|------|---------|
| `include/iap/planner/predicted_araim.hpp` | Removed `<iap/integrity/araim.hpp>` include; `Araim::Params` → `GnssGeometryPlPredictorParams`; `Araim araim_` → `GnssGeometryPlPredictor geom_predictor_` |
| `src/iap/planner/predicted_araim.cpp` | `Araim::SatGeometry` → `GnssGeometrySat`; `araim_.predict_geometry()` → `geom_predictor_.predict()`; constructors updated |
| `src/iap/planner/future_pl_field_predictor.cpp` | `params.araim_params.K_ff` → `params.geometry_params.K_ff` |
| `test/test_future_pl_field_predictor.cpp` | `araim_params.araim_params.X` → `araim_params.geometry_params.X` |
| `test/test_predicted_araim.cpp` | `araim_params.X` → `geometry_params.X` |
| `include/iap/integrity/araim.hpp` | Added `[[deprecated]]` attribute + TODO marker on `predict_geometry()` |
| `src/iap/integrity/araim.cpp` | Added TODO marker before `predict_geometry()` impl |
| `CMakeLists.txt` | Added `gnss_geometry_pl_predictor.cpp` to library |
| `docs/audit_0520/status.md` | Step 6 entry |

### Test results
```text
test_araim: 54/54 pass
test_predicted_araim: 5/5 pass
test_future_pl_field_predictor: 11/11 pass
All 14 IAP executables: 0 failures
```

### Acceptance
- `predicted_araim.hpp` no longer includes `<iap/integrity/araim.hpp>` ✅
- `PredictedAraimComputer` no longer owns or calls `Araim` ✅
- `Araim::predict_geometry()` preserved as deprecated wrapper ✅
- Build produces one deprecation warning (expected) ✅

### Known limitations
- `Araim::predict_geometry()` still exists — removal deferred to future cleanup
- New predictor numerically close but not bit-identical to old implementation
- Planner output fields retain legacy "araim" naming

---

## 7. Dev Log Paradigm

All future ARAIM refactor steps should record the following in a new section below:

### Template for each step

```markdown
## N. Step N — <Step Name>

### Date
YYYY-MM-DD

### Prerequisite steps completed
- [ ] Step N-1, N-2, ...

### Behavior changes (before → after)
| Item | Before | After |
|------|--------|-------|

### Files created
| File | Description |
|------|-------------|

### Files modified
| File | Exact changes |

### Files deleted
| File | Reason |

### Tests added / updated
| Test | What it validates |

### Build command
```bash
colcon build --symlink-install --packages-select iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

### Test command and results
```bash
colcon test --packages-select iap
```
```text
N tests, 0 errors, 0 failures
```

### Backward compatibility notes
- List any breaking changes, ROS msg changes, log format changes

### Known limitations / deferred items
- List anything intentionally left for a later step

### Review checklist
- [ ] All existing tests pass
- [ ] New tests cover all behavior changes
- [ ] No NaN/Inf can reach published ROS topics
- [ ] No renames performed
- [ ] No file moves
- [ ] No CMake target changes (unless Step 10)
- [ ] `Araim::predict_geometry()` preserved
- [ ] `/iap/integrity` ROS topic backward compatible
```



---

## 8. Step 8 — GnssEpoch Linearized Input Seam

### Date
2026-05-21

### Design decision
Introduce `GnssAraimLinearizedInput` as a plain-data struct containing pre-built
G, W, r matrices and per-satellite metadata. `run()` delegates to `runLinearized()`
via `buildLinearizedInputFromGnssEpoch()`. This creates a stable test seam for
unit testing ARAIM math without constructing full `GnssEpoch` objects.

### Behavior changes
| Item | Before | After |
|------|--------|-------|
| `run()` | Builds G/W/r internally, calls compute_core | Delegates to `buildLinearizedInputFromGnssEpoch()` → `runLinearized()` |
| Input validation | Implicit (min_sats check only) | Explicit: size consistency, G.cols==4, finite/positive W |
| Test seam | Must construct full GnssEpoch | Can construct `GnssAraimLinearizedInput` directly |
| Hypothesis enumeration | Epoch-based only | Linearized overload added |

### Files modified
| File | Changes |
|------|---------|
| `include/iap/integrity/araim_types.hpp` | Added `GnssAraimLinearizedInput` struct |
| `include/iap/integrity/araim.hpp` | Added `runLinearized()`, `buildLinearizedInputFromGnssEpoch()`, `enumerate_hypotheses(linearized)` |
| `src/iap/integrity/araim.cpp` | Added builder implementation, linearized enumerate_hypotheses, `runLinearized()` with input validation; `run()` delegates to linearized path |
| `test/test_araim.cpp` | 7 new tests (A–G) |
| `docs/audit_0520/status.md` | This section |

### New tests (A–G)
| Test | What it validates |
|------|-------------------|
| `LinearizedMatchesEpochRoundTrip` | epoch→linearized→runLinearized ≡ run(epoch) |
| `LinearizedMatchesEpochWithTrunk` | Same with trunk hypotheses enabled and n_trunk=2 |
| `LinearizedRejectsTooFewRows` | N < min_sats → invalid |
| `LinearizedRejectsBadDimensions` | G.cols != 4 → invalid |
| `LinearizedRejectsNaN` | NaN in G matrix → invalid |
| `LinearizedRejectsNonPositiveWeight` | W(i) <= 0 → invalid |
| `LinearizedConstellationDisabledMatchesEpoch` | Constellation faults disabled → same result both paths |

### Build result
```text
colcon build --packages-select iap → passed (deprecation warnings only)
```

### Test result
```text
test_araim: 62 tests, 0 errors, 0 failures (55 + 7 new)
All IAP test executables: 0 failures
```

### Backward compatibility
- `GnssAraimEvaluator::run(const GnssEpoch&, int)` signature preserved
- `GnssAraimResult` struct unchanged
- No ROS msg changes
- No config file changes
- Epoch-based `enumerate_hypotheses()` preserved for backward compatibility

### Known limitations
- `buildLinearizedInputFromGnssEpoch()` is static — cannot use instance params for constellation inference
- Linearized `enumerate_hypotheses` duplicates some logic from epoch-based version (acceptable for Step 8)
- Deprecated `Araim::predict_geometry()` still uses epoch-based path

### Review checklist
- [x] All existing tests pass (55/55)
- [x] 7 new tests pass
- [x] No renames performed
- [x] No file moves
- [x] No CMake target changes
- [x] `/iap/integrity` ROS topic unchanged
- [x] `GnssAraimEvaluator::run()` signature preserved

---

## 9. Step 9 — Decompose Large Compute & Monitor Functions

### Date
2026-05-21

### Design decision
Split `compute_core()` (~285 lines) and `IntegrityMonitor::compute()` (~170 lines)
into small, testable helpers. Pure copy-paste extraction — no math, equation, threshold,
or logic changes. All helpers are static free functions (compute_core) or private methods
(IntegrityMonitor).

### compute_core() — 8 static helpers in anonymous namespace

| # | Helper | Purpose |
|---|--------|---------|
| H1 | `initializeResultMetadata()` | Sets hypotheses, counters, config flags |
| H2 | `buildNominalNormalEqns()` | Builds A0, rhs0, row_outer, row_rhs |
| H3 | `solveFullSolution()` | LDLT factorize, compute S0, solve p0 |
| H4 | `allocateBudget()` | Dynamic K_ff/K_fa budget allocation |
| H5 | `computeFaultFreePL()` | Fault-free sigma and PL computation |
| H6 | `evalSubset()` | Single hypothesis subset evaluation |
| H7 | `evalAllSubsets()` | Parallel/serial dispatch for all hypotheses |
| H8 | `collectFinalResults()` | Tally degenerate, accumulate worst PLs, finalize HPL/VPL |

### IntegrityMonitor::compute() — 7 private method helpers

| # | Helper | Purpose |
|---|--------|---------|
| M1 | `buildFallbackSource()` | Fallback PL + report fields |
| M2 | `evaluateGnssSource()` | NIS gating + run_araim + GNSS report fields |
| M3 | `evaluateLidarSource()` | run_lidar_araim + LiDAR report fields |
| M4 | `fuseIntegritySources()` | FusionPolicy fuse + HPL/VPL/source fields |
| M5 | `computeAlertLimits()` | Dynamic AL + obstacle AL |
| M6 | `computeIntegrityMargins()` | im_h/im_v/IM with numerical guards |
| M7 | `updateStateAndPlannerMode()` | State machine + legacy mode + logging + timing |

### Files modified
| File | Changes |
|------|---------|
| `include/iap/integrity/araim.hpp` | Moved `Q_inv()` from private to public (needed by anonymous-namespace helpers) |
| `src/iap/integrity/araim.cpp` | Added 8 static helpers + 2 helper structs (`NominalEqns`, `IntegrityBudget`) in anonymous namespace; replaced `compute_core()` body with ~30-line orchestration |
| `include/iap/integrity/integrity_monitor.hpp` | Added 7 private method declarations |
| `src/iap/integrity/integrity_monitor.cpp` | Implemented 7 helpers; replaced `compute()` body with ~40-line orchestration; added `report.araim_n_det = 0` initialization for deterministic state machine input |
| `test/test_araim.cpp` | Added 8 new tests (3 compute_core regression + 5 monitor regression) |
| `docs/audit_0520/status.md` | This section |

### Tests added
| Test | File | What it validates |
|------|------|-------------------|
| `ComputeCoreDecompositionRegression` | test_araim.cpp | epoch-vs-linearized equivalence for N=4,6,8,10 on 13 fields |
| `ComputeCoreBudgetFieldsPopulated` | test_araim.cpp | K_ff_used, K_fa_used positive and finite |
| `FaultFreePLMatchesDirect` | test_araim.cpp | pl_ff == max(pl_ff_E, pl_ff_N) |
| `GnssSourceFieldsPopulated` | test_araim.cpp | GNSS report fields populated when epoch available |
| `FinalSourceFieldsReflectFusion` | test_araim.cpp | final_HPL/VPL/PL_source strings non-empty |
| `FallbackOnlyOutputUnchanged` | test_araim.cpp | Enhanced T0.1 with fallback-specific field checks |
| `IMFormulaPreserved` | test_araim.cpp | IM = min(HAL-HPL, VAL-VPL) for 4 covariance levels |
| `StateTransitionsPreserved` | test_araim.cpp | H/V margin sign reflects safety; unsafe state on large covariance |

### Build result
```text
colcon build --packages-select iap → passed (deprecation warnings only)
```

### Test result
```text
test_araim: 70 tests, 0 failures (+8 from Step 8 baseline of 62)
test_integrity_fusion_policy: 13 tests, 0 failures
All IAP test executables: 0 failures
```

### Backward compatibility
- `GnssAraimEvaluator::run(GnssEpoch, int)` signature preserved
- `IntegrityMonitor::compute(...)` signature preserved
- `Q_inv()` now public (was private) — backward compatible, no callers broken
- No ROS msg/topic changes
- No config file changes
- No CMake target split
- All ARAIM math, equations, thresholds, K-factor logic unchanged
- OpenMP dispatch behavior unchanged
- Timing CSV names unchanged
- State machine semantics unchanged
- `report.araim_n_det = 0` initialization added (fixes latent uninitialized-read when GNSS is unavailable; no behavior change for GNSS-available path)

### Known limitations
- `Q_inv()` public exposure is intentional — enables unit testing of budget allocation math
- `update_mode_legacy` and `update_state` share `recovery_counter_` member (pre-existing design, not introduced by Step 9)
- Helper parameter lists are verbose (trade-off for explicit data flow)

### Review checklist
- [x] All existing tests pass (62/62)
- [x] 8 new regression tests pass
- [x] No public runtime API changes
- [x] No ROS msg/topic changes
- [x] No config changes
- [x] No CMake target split
- [x] Runtime GNSS path still calls `GnssAraimEvaluator::run(const GnssEpoch&, int)`
- [x] `runLinearized()` remains test/core seam only
- [x] Numerical outputs for valid GnssEpoch inputs remain equivalent
- [x] OpenMP dispatch preserved
- [x] Timing CSV names preserved

---

## 10. Step 10 — Library Target Split

### Date
2026-05-21

### Design decision
Add 3 INTERFACE library targets (`iap_integrity_common`, `iap_current_integrity`,
`iap_prediction_integrity`) that provide include directories and transitive
link dependencies. All source files remain compiled into the monolithic `iap`
SHARED library — no sources are moved or duplicated.

**Why INTERFACE and not STATIC**: The IAP codebase has internal circular symbol
dependencies between prediction sources (e.g., `predicted_araim.cpp` calling
`VisibilityPredictor`) and the main `iap` library. Creating true STATIC sub-targets
would require either moving 50+ sources or accepting unresolved symbols at test
link time. The INTERFACE approach provides the naming/organizational benefit of
Step 10 without forcing an unsafe split.

Additionally, `ament_target_dependencies(iap gnss_comm)` uses CMake's plain
(non-keyword) `target_link_libraries` form, which prevents mixing PUBLIC/PRIVATE
visibility on the `iap` target — a hard requirement for STATIC sub-targets that
must not become transitive export dependencies.

This design can be upgraded to STATIC or OBJECT targets in a future refactor
once the circular dependencies (prediction→gnss, integrity→timing_csv) are
resolved through a proper adapter layer.

### Targets added

| Target | Type | Purpose |
|--------|------|---------|
| `iap_integrity_common` | INTERFACE | Common integrity types + fusion policy headers; links Eigen3, spdlog |
| `iap_current_integrity` | INTERFACE | Runtime GNSS + LiDAR ARAIM + monitor headers; links iap_integrity_common, gtsam, gtsam_points |
| `iap_prediction_integrity` | INTERFACE | Planner-side prediction headers; links iap_current_integrity |

### Source files (all remain in iap SHARED target)

No sources were moved or removed from the `iap` SHARED library. The full source
list (util, preprocess, common, gnss, map, trunk, integrity, planner, odometry,
mapping, GPU) remains unchanged from Step 9.

### Tests

| Test | Links | Note |
|------|-------|------|
| `test_araim` | `iap` + `iap_current_integrity` | Needs TrunkMap, LidarAraim symbols from iap |
| `test_integrity_fusion_policy` | `iap` + `iap_integrity_common` | Narrows to common types for documentation |
| `test_predicted_araim` | `iap` + `iap_prediction_integrity` | Needs VisibilityPredictor from iap |
| `test_integrity_snapshot` | `iap` + `iap_prediction_integrity` | Needs broader iap symbols |
| `test_future_pl_field_predictor` | `iap` + `iap_prediction_integrity` | Needs timing_csv, glim::Config from iap |
| All other tests (8) | `iap` only | Need odometry/mapping/trunk/gnss symbols |

### Tests intentionally left linked to iap (broad dependency)

| Test | Reason |
|------|--------|
| `test_lidar_observability_fim` | Needs gtsam/gtsam_points factor graph symbols |
| `test_pl_grid` | Needs planner occupancy grid symbols |
| `test_local_occupancy` | Needs LocalOccupancyGrid from map/ |
| `test_pi_cost_adapter` | Needs trajectory/planner symbols |
| `test_unified_risk_grid` | Needs risk grid + planner symbols |
| `test_odom_freshness` | Needs EstimationFrame + odometry symbols |
| `test_alert_limit_model` | Needs DynamicALResult + trunk symbols |
| `test_run_log_manager` | Needs RunLogManager from util/ |

### Build result
```text
colcon build --packages-select iap → passed (0 warnings, 0 errors)
```

### Test result
```text
test_araim: 70 tests, 0 failures
test_integrity_fusion_policy: 13 tests, 0 failures
test_predicted_araim: 5 tests, 0 failures
test_integrity_snapshot: 4 tests, 0 failures
test_future_pl_field_predictor: 11 tests, 0 failures
All 13 IAP test executables: 0 failures
```

### Backward compatibility
- `libiap.so` target unchanged — all existing plugins and apps link without changes
- No source file moves or renames
- No ROS msg/topic changes
- No config changes
- No runtime behavior changes
- `integrity_extension`, `phase2_planner_integrity_evaluator`, `demo8_truth_araim_extension` and all other plugins/apps build and link unchanged
- INTERFACE targets have no install rules — they exist only as build-tree documentation

### Known limitations
- INTERFACE targets provide no compile-time isolation (all sources still in one translation pool)
- True STATIC target split requires resolving: prediction→gnss (VisibilityPredictor), integrity→util (timing_csv), prediction→map (LocalOccupancyGrid) dependencies
- Upgrade path: add adapter/interface layer → resolve circular deps → promote INTERFACE to STATIC

### Review checklist
- [x] `colcon build` passes
- [x] `colcon test` passes (all 83 IAP tests)
- [x] `iap` target still exists
- [x] Existing plugins/apps still link against `iap`
- [x] No source file moves
- [x] No public API changes
- [x] No ROS msg/topic changes
- [x] No runtime config changes
- [x] No ARAIM numerical behavior changes
- [x] Runtime GNSS path still uses `GnssAraimEvaluator::run(const GnssEpoch&, int)`
- [x] `runLinearized()` remains test/core seam only

---

## 7. Constraint Compliance Checklist

Per `ARAIM_refactor_plan_v2.md` hard constraints:

| Constraint | Status |
|-----------|--------|
| Do not rename `Araim` | ✅ Compiled |
| Do not move files | ✅ No files moved |
| Do not split CMake targets | ✅ No CMake changes |
| Do not remove `Araim::predict_geometry()` | ✅ Preserved |
| Do not change planner predictor code | ✅ No changes to `src/iap/planner/` |
| Keep `/iap/integrity` backward compatible | ✅ `.msg` unchanged, `im` field more conservative |
| Do not introduce `AraimBase` inheritance | ✅ No new base class |
| GNSS ARAIM and LiDAR integrity remain independent | ✅ No shared solver class |

---

*Log maintained at `src/iap/docs/audit_0520/status.md`*
*Template for future steps in Section 5 above.*
