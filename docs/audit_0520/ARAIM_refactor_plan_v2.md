# IAP ARAIM Module Refactor Plan v2.0

> Scope: This document redesigns the ARAIM refactor plan based on the latest ARAIM audit and the updated architectural decision: **Current ARAIM and Future Predictor should remain separate modules for better debugging and clearer semantics.**

---

## 0. Executive Decision

### 0.1 Final architecture decision

The ARAIM subsystem should be redesigned around the following principle:

```text
Current ARAIM answers:
  Am I safe now?

Future Predictor answers:
  Which future region is likely to be safer for localization?
```

Therefore:

```text
Do NOT make GNSS ARAIM and LiDAR ARAIM two strong subclasses under one generic AraimBase.
Do NOT let Future Predictor call Current ARAIM as its planner-side entry point.
Do NOT force Current ARAIM and Future Predictor to share one solver at this stage.
```

Instead:

```text
CurrentIntegrityMonitor
  ├── GnssAraimEvaluator / CurrentAraimSolver
  ├── LidarIntegrityEvaluator
  ├── AlertLimitModel
  ├── IntegrityFusionPolicy
  ├── IntegrityStateMachine
  └── IntegrityReportBuilder
```

Current ARAIM and Future Predictor may share small common utilities and message/data schemas, but they should not share the main solver implementation during this refactor stage.

### 0.2 Audit-driven implementation priority

This refactor must be implementation-driven, not rename-driven.  The latest
code audit and source inspection show several behavior risks that must be fixed
before large file renames or CMake target splits:

```text
P0 behavior correctness:
  - Current monitor state must use both HPL/HAL and VPL/VAL.
  - IM must be min(HAL - HPL, VAL - VPL), not AL - scalar HPL.
  - NaN/Inf/degenerate sentinel PL values must never be published without flags.
  - Runtime debug output must expose GNSS, LiDAR, fallback, and fused sources.

P1 debuggability and switches:
  - GNSS, LiDAR, fallback, and fusion mode must be independently controllable.
  - Existing config_gnss.json flat-key loading must remain supported.
  - ROS output must carry enough source-level diagnostics, or a dedicated debug
    topic/message must be added.

P2 semantic cleanup:
  - Move planner-side geometry prediction out of current ARAIM before renaming.
  - Rename Araim only after behavior is protected by tests.
  - Do not claim trunk hypotheses are fully handled by GNSS ARAIM unless there is
    an actual trunk measurement model in that solver path.

P3 build modularity:
  - Split library targets only after public APIs and runtime behavior stabilize.
```

Therefore the implementation order in this document supersedes the original
"numerical guards -> switches -> rename -> predictor -> adapter -> CMake" order.
Behavioral correctness and observable runtime semantics come first.

---

## 1. Current ARAIM Module Status

According to the latest audit, the current code structure is approximately:

```text
include/iap/integrity/araim.hpp
include/iap/integrity/araim_types.hpp
src/iap/integrity/araim.cpp
src/iap/integrity/lidar_araim.cpp
src/iap/integrity/integrity_monitor.cpp
src/iap/integrity/integrity_extension.cpp
src/iap/integrity/fgo_information_manager.cpp
src/iap/planner/predicted_araim.cpp
```

Current observations:

| Aspect | Current Status | Refactor Judgment |
|---|---|---|
| `araim.cpp` | Pure C++ computation, no ROS dependency | Good foundation |
| `araim.hpp` | Public API includes `run()` and `predict_geometry()` | Needs semantic split |
| `Araim::Params` | Nested inside `Araim` | Move to standalone current-ARAIM params type |
| `GnssEpoch` | Directly referenced by public ARAIM API | Do not decouple in one step; introduce adapter gradually |
| `lidar_araim.cpp` | LiDAR/VGICP block-level integrity logic | Keep separate from GNSS ARAIM |
| `integrity_monitor.cpp` | Orchestrates PL/AL/IM/state | Should remain orchestrator, but be decomposed |
| `integrity_extension.cpp` | ROS/plugin/publishing layer | Keep as ROS adapter only |
| `predicted_araim.cpp` | Planner-side advisory geometry proxy | Must be separated from Current ARAIM |
| `libiap.so` | ARAIM compiled into monolithic library | Split current ARAIM into a dedicated target |

---

## 2. Main Design Clarifications

### 2.1 GNSS ARAIM vs LiDAR ARAIM

GNSS ARAIM and LiDAR ARAIM should not be modeled as:

```cpp
class AraimBase {
public:
  virtual AraimResult run(...) = 0;
};

class GnssAraim : public AraimBase {};
class LidarAraim : public AraimBase {};
```

This inheritance-based abstraction is not recommended because the two inputs, fault models, residuals, and debug requirements are not truly isomorphic.

Instead, use independent evaluators:

```cpp
class GnssAraimEvaluator {
public:
  GnssIntegrityResult evaluate(
      const GnssEpoch& epoch,
      const CurrentStateSnapshot& state,
      const GnssAraimParams& params) const;
};

class LidarIntegrityEvaluator {
public:
  LidarIntegrityResult evaluate(
      const LidarIntegritySnapshot& lidar,
      const CurrentStateSnapshot& state,
      const LidarIntegrityParams& params) const;
};

class CurrentIntegrityMonitor {
public:
  IntegrityReport compute(const CurrentIntegrityInput& input);

private:
  GnssAraimEvaluator gnss_araim_;
  LidarIntegrityEvaluator lidar_integrity_;
  IntegrityFusionPolicy fusion_policy_;
  AlertLimitModel alert_limit_model_;
  IntegrityStateMachine state_machine_;
  IntegrityReportBuilder report_builder_;
};
```

### 2.2 Why composition is preferred over inheritance

GNSS ARAIM requires:

```text
GNSS epoch
satellite PRN
constellation ID
pseudorange residual
satellite geometry matrix G
measurement weights W
fault hypotheses: satellite / constellation faults
```

LiDAR integrity requires:

```text
VGICP blocks
scan matching residuals
trunk/feature associations
condition number / observability
LiDAR FIM or registration covariance
fault hypotheses: block / trunk / feature-group faults, if implemented
```

A single base class would either become too generic or force one evaluator to fake fields that only exist for the other sensor. That makes debugging harder and can hide theoretical differences between GNSS ARAIM and LiDAR integrity.

### 2.3 Recommended naming

Use names that reflect certainty and sensor semantics:

```text
Current/GNSS side:
  Araim                → GnssAraimEvaluator or CurrentAraimSolver
  Araim::Params        → GnssAraimParams or CurrentAraimParams
  AraimResult          → GnssAraimResult or CurrentAraimResult

LiDAR side:
  lidar_araim.cpp      → lidar_integrity_evaluator.cpp
  LidarAraim           → LidarIntegrityEvaluator

Planner prediction side:
  predicted_araim.cpp  → gnss_geometry_pl_predictor.cpp
  PredictedAraimComputer → GnssGeometryPlPredictor or AdvisoryGnssPlPredictor
```

If LiDAR truly implements fault hypotheses, subset downdates, solution separation, and risk allocation, it can later be renamed to `LidarAraimEvaluator`. Until then, `LidarIntegrityEvaluator` is safer and more honest.

---

## 3. Runtime Switch and Output Design

The refactored module must support independent debugging and fused debugging,
but the implementation must match the current repository:

```text
Current runtime config source:
  config/config_gnss.json
  section: "integrity"
  style: flat JSON keys loaded by IntegrityExtensionModule

Current public ROS output:
  msg/IntegrityReport.msg
  topic: /iap/integrity
  currently contains fused PL/AL/IM and limited ARAIM diagnostics only
```

Do not introduce nested YAML-only parameters unless the config loader is changed
in the same step and tested.  For the first implementation pass, add flat JSON
keys and optionally mirror them later in `araim_params.yaml` documentation.

### 3.1 Required flat-key switches

Add or formalize these keys under `"integrity"` in `config_gnss.json` and all
scenario override configs that need non-default behavior:

```json
{
  "enable_current_integrity_monitor": true,

  "enable_gnss_integrity": true,
  "enable_gnss_araim": true,
  "enable_gnss_satellite_faults": true,
  "enable_gnss_constellation_faults": true,
  "gnss_integrity_debug_verbose": false,

  "enable_lidar_integrity": true,
  "enable_lidar_block_faults": true,
  "lidar_integrity_debug_verbose": false,

  "integrity_fusion_mode": "max_pl",
  "integrity_publish_source_breakdown": true,
  "integrity_require_valid_gnss": false,
  "integrity_require_valid_lidar": false,

  "integrity_conservative_hpl_m": 999.0,
  "integrity_conservative_vpl_m": 999.0,
  "integrity_reject_nan_inf": true,
  "integrity_reject_negative_variance": true
}
```

Compatibility mapping:

| Existing key | Keep? | New meaning |
|---|---:|---|
| `enable` | Yes | Master module enable |
| `enable_araim` | Yes, deprecated | Alias for `enable_gnss_araim` during transition |
| `enable_fgo_info` | Yes | Enables fallback covariance extraction |
| `enable_dynamic_al` | Yes | Enables trunk/altitude AL inputs |

### 3.2 Fusion modes

The monitor must support these modes:

| Mode | Behavior | Purpose |
|---|---|---|
| `gnss_only` | Final HPL/VPL come only from valid GNSS ARAIM; no LiDAR max fusion | Debug GNSS current monitor |
| `lidar_only` | Final HPL/VPL come only from valid LiDAR integrity; no GNSS max fusion | Debug LiDAR current monitor |
| `fallback_only` | Final HPL/VPL come from covariance proxy only | Debug estimator covariance fallback |
| `max_pl` | `fused_hpl=max(valid sources)`, `fused_vpl=max(valid sources)` | Conservative default |
| `weighted_debug_only` | Experimental weighted fusion, never default | Debug/analysis only |

Default should remain conservative and close to current behavior:

```json
{
  "integrity_fusion_mode": "max_pl"
}
```

Important: `fallback_only` and invalid-source behavior must be explicit.  The
current implementation initializes the report from fallback PL and then lets GNSS
or LiDAR override/max it.  After this refactor, fallback must be treated as a
named source with its own validity flag, not an invisible default.

### 3.3 Correct H/V safety semantics

The current monitor must not reduce safety to one scalar `AL - PL` unless that
scalar is derived from both axes.  Required semantics:

```text
im_h = HAL - fused_HPL
im_v = VAL - fused_VPL
IM   = min(im_h, im_v)

SAFE condition:
  fused_HPL < HAL AND fused_VPL < VAL

UNSAFE condition:
  fused_HPL >= HAL OR fused_VPL >= VAL
```

The published scalar `im` in `IntegrityReport.msg` remains for compatibility,
but it must mean `min(HAL-HPL, VAL-VPL)`.

### 3.4 Debug outputs

Each cycle should expose enough data to answer source and failure questions.
Internal C++ structs are not sufficient because the planner and offline tools
consume ROS topics and CSV logs.

Required fields in ROS output or a dedicated debug topic:

```text
gnss_hpl, gnss_vpl, gnss_validity, gnss_dominant_hypothesis, gnss_fault_flags
lidar_hpl, lidar_vpl, lidar_validity, lidar_dominant_block, lidar_fault_flags
fallback_hpl, fallback_vpl, fallback_validity
fused_hpl, fused_vpl, fusion_mode, final_hpl_source, final_vpl_source
hal, val, im_h, im_v, im_min
state_transition_reason
numerical_failure_flags
```

Implementation options:

| Option | Pros | Cons | Recommendation |
|---|---|---|---|
| Extend `msg/IntegrityReport.msg` | One topic, easy consumers | Breaks generated message ABI and downstream build | Good if all consumers are rebuilt together |
| Add `msg/IntegrityDebug.msg` on `/iap/integrity_debug` | Non-breaking primary report | One more topic to record and validate | Preferred first step |
| CSV-only | Low implementation cost | Not visible to online consumers | Insufficient by itself |

This makes it possible to answer:

```text
Did GNSS cause the unsafe state?
Did LiDAR cause the unsafe state?
Did fallback covariance dominate?
Did fusion mode cause the final PL to increase?
Was the result invalid due to numerical failure?
Was the unsafe state horizontal or vertical?
```

---

## 4. GnssEpoch Decoupling Strategy

### 4.1 Do not decouple `GnssEpoch` in one step

Yes, “do not decouple `GnssEpoch` in one step” means exactly this:

```text
Do not replace every ARAIM API input with a fully abstract measurement interface immediately.
Do not rewrite the GNSS data path while also renaming ARAIM and splitting modules.
Do not break IntegrityMonitor or tests by doing a large interface rewrite.
```

The current ARAIM solver is still current-GNSS-facing. It is acceptable for the current evaluator to consume `GnssEpoch` during the first refactor pass.

### 4.2 Two-stage decoupling

#### Stage A: Introduce an internal linearized input while preserving public API

Keep the current public entry:

```cpp
CurrentAraimResult run(const GnssEpoch& epoch, ...);
```

Internally convert it to:

```cpp
struct GnssAraimLinearizedInput {
  Eigen::MatrixXd G;
  Eigen::VectorXd w;
  Eigen::VectorXd r;
  std::vector<int> prns;
  std::vector<int> constellation_ids;
  std::vector<double> elevations_rad;
  std::vector<double> sigmas_m;
};
```

Then compute from:

```cpp
CurrentAraimResult runLinearized(const GnssAraimLinearizedInput& input, ...);
```

This gives us a clean seam without breaking existing call sites.

#### Stage B: Move `GnssEpoch` conversion to an adapter

Later, move conversion out of the solver:

```cpp
class GnssAraimInputBuilder {
public:
  GnssAraimLinearizedInput build(const GnssEpoch& epoch) const;
};
```

Final dependency:

```text
IntegrityMonitor
  → GnssAraimInputBuilder
  → GnssAraimEvaluator::runLinearized()
```

At that point, the core solver no longer depends directly on the full GNSS epoch type.

---

## 5. Removing `Araim::predict_geometry()` Without Breaking Predictor

### 5.1 Problem

The current `Araim` class has two different responsibilities:

```text
run()
  Real current ARAIM monitoring from actual GNSS epoch/residuals.

predict_geometry()
  Planner-side advisory PL proxy with r=0 geometry assumption.
```

These are semantically different. The planner-side method should not remain as the official ARAIM entry point.

### 5.2 Goal

Separate planner-side prediction from Current ARAIM while keeping the code easy to find during the future Predictor refactor.

### 5.3 Low-risk transition plan

#### Step 1: Create a new planner-side class and move the logic verbatim

Create:

```text
include/iap/planner/gnss_geometry_pl_predictor.hpp
src/iap/planner/gnss_geometry_pl_predictor.cpp
```

or, if a prediction package exists:

```text
include/iap/prediction/gnss_geometry_pl_predictor.hpp
src/iap/prediction/gnss_geometry_pl_predictor.cpp
```

Class:

```cpp
class GnssGeometryPlPredictor {
public:
  explicit GnssGeometryPlPredictor(const GnssGeometryPlPredictorParams& params);

  AdvisoryGnssPlResult predict(
      const Eigen::Vector3d& query_position,
      const GnssEpoch& reference_epoch,
      const VisibilityPrediction& visibility) const;
};
```

This code may initially copy the exact internal behavior of `Araim::predict_geometry()` to preserve runtime behavior.

#### Step 2: Keep a temporary compatibility wrapper

Inside `CurrentAraimSolver` / old `Araim`, keep:

```cpp
[[deprecated("Use GnssGeometryPlPredictor for planner-side advisory PL prediction.")]]
AraimResult predict_geometry(...) const;
```

Implementation should forward to the new class or preserve old behavior temporarily.

Add a comment block:

```cpp
// TODO(IAP-ARAIM-REFAC-PREDICTOR): This method exists only for backward compatibility.
// Planner-side prediction has moved to GnssGeometryPlPredictor.
// Remove after PredictedAraimComputer is migrated.
```

This allows the next Predictor refactor to find the old entry point easily.

#### Step 3: Rename `PredictedAraimComputer`

Rename:

```text
src/iap/planner/predicted_araim.cpp
include/iap/planner/predicted_araim.hpp
```

To:

```text
src/iap/planner/gnss_geometry_pl_predictor.cpp
include/iap/planner/gnss_geometry_pl_predictor.hpp
```

Or, if you want an intermediate compatibility layer:

```text
PredictedAraimComputer → AdvisoryGnssPlComputer
```

Keep a temporary alias:

```cpp
using PredictedAraimComputer [[deprecated("Use GnssGeometryPlPredictor")]] = GnssGeometryPlPredictor;
```

#### Step 4: Update call sites gradually

Update planner code from:

```cpp
predicted_araim_.predict(...)
// or
araim_.predict_geometry(...)
```

To:

```cpp
gnss_geometry_pl_predictor_.predict(...)
```

Do this in a separate PR from Current ARAIM renaming to reduce risk.

#### Step 5: Remove wrapper after predictor refactor

Only after the predictor module is fully refactored and tests pass:

```text
Remove Araim::predict_geometry()
Remove deprecated PredictedAraimComputer alias
Remove planner dependency on current_araim_solver.hpp
```

### 5.4 Important naming rule

Planner-side prediction must not output fields named as if they are certified current ARAIM results.

Use:

```text
advisory_hpl
advisory_vpl
advisory_pdop
advisory_fim
predicted_validity
```

Avoid:

```text
araim_hpl
araim_vpl
certified_pl
```

---

## 6. Proposed File Layout

### 6.1 Current target layout

```text
include/iap/integrity/
  integrity_common.hpp
  current_integrity_monitor.hpp
  gnss_araim_evaluator.hpp
  gnss_araim_types.hpp
  lidar_integrity_evaluator.hpp
  lidar_integrity_types.hpp
  alert_limit_model.hpp
  integrity_fusion_policy.hpp
  integrity_state_machine.hpp
  integrity_report_builder.hpp
  fgo_information_adapter.hpp

src/iap/integrity/
  current_integrity_monitor.cpp
  gnss_araim_evaluator.cpp
  lidar_integrity_evaluator.cpp
  alert_limit_model.cpp
  integrity_fusion_policy.cpp
  integrity_state_machine.cpp
  integrity_report_builder.cpp
  fgo_information_adapter.cpp
  integrity_extension.cpp
```

### 6.2 Planner-side prediction layout

```text
include/iap/planner/
  gnss_geometry_pl_predictor.hpp
  future_pl_field_predictor.hpp
  lidar_observability_fim.hpp
  advisory_integrity_types.hpp

src/iap/planner/
  gnss_geometry_pl_predictor.cpp
  future_pl_field_predictor.cpp
  lidar_observability_fim.cpp
```

Alternative if prediction becomes its own package:

```text
include/iap/prediction/
src/iap/prediction/
```

### 6.3 Common utility layout

```text
include/iap/integrity/integrity_common.hpp
include/iap/integrity/numerical_guard.hpp
```

Only put small shared types here:

```text
ProtectionLevel
AlertLimit
IntegrityMargin
Validity flags
Source enum
Numerical failure flags
```

Do not put GNSS ARAIM solver or Future Predictor solver here.

---

## 7. CMake Target Plan

### 7.1 Recommended targets

```cmake
add_library(iap_integrity_common
  src/iap/integrity/numerical_guard.cpp
)

target_link_libraries(iap_integrity_common
  PUBLIC Eigen3::Eigen
)

add_library(iap_current_integrity
  src/iap/integrity/gnss_araim_evaluator.cpp
  src/iap/integrity/lidar_integrity_evaluator.cpp
  src/iap/integrity/current_integrity_monitor.cpp
  src/iap/integrity/alert_limit_model.cpp
  src/iap/integrity/integrity_fusion_policy.cpp
  src/iap/integrity/integrity_state_machine.cpp
  src/iap/integrity/integrity_report_builder.cpp
  src/iap/integrity/fgo_information_adapter.cpp
)

target_link_libraries(iap_current_integrity
  PUBLIC iap_integrity_common Eigen3::Eigen
  PRIVATE ${GTSAM_LIBRARIES}
)

add_library(iap_prediction_integrity
  src/iap/planner/gnss_geometry_pl_predictor.cpp
  src/iap/planner/future_pl_field_predictor.cpp
  src/iap/planner/lidar_observability_fim.cpp
)

target_link_libraries(iap_prediction_integrity
  PUBLIC iap_integrity_common Eigen3::Eigen
)
```

The existing `libiap.so` can continue to link these libraries until the project is fully modularized.

### 7.2 Test targets

```cmake
add_executable(test_gnss_araim test/test_araim.cpp)
target_link_libraries(test_gnss_araim PRIVATE iap_current_integrity gtest_main)

add_executable(test_lidar_integrity test/test_lidar_integrity.cpp)
target_link_libraries(test_lidar_integrity PRIVATE iap_current_integrity gtest_main)

add_executable(test_gnss_geometry_pl_predictor test/test_predicted_araim.cpp)
target_link_libraries(test_gnss_geometry_pl_predictor PRIVATE iap_prediction_integrity gtest_main)
```

---

## 8. Implementation Plan

This section is ordered by implementation risk and runtime value.  Each step must
land with tests.  Do not combine behavior changes with broad renames.

### Step 0 — Establish Baseline and Golden Outputs

Goal: capture current behavior before modifying monitor semantics.

Scope:

```text
Read-only or test-only changes.
No public API rename.
No formula change.
```

Tasks:

```text
1. Add/refresh golden unit tests around current GNSS ARAIM outputs.
2. Add monitor tests that document current fused HPL/VPL, HAL/VAL, and state.
3. Add a small fixture for constructing EstimationFrame + optional GNSS/LiDAR inputs.
4. Record current behavior for:
   - GNSS-only
   - LiDAR-only snapshot
   - fallback-only covariance
   - GNSS + LiDAR max fusion
```

Required tests:

| Test | File | Required check |
|---|---|---|
| Current GNSS golden | `test/test_araim.cpp` | Existing PL formula and hypothesis tests still pass |
| Current monitor fixture | `test/test_integrity_monitor.cpp` or `test/test_araim.cpp` | Can construct report with fallback, GNSS, LiDAR inputs |
| Current fused baseline | same | Documents current HPL/VPL source behavior before Step 1 |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_araim|test_integrity_monitor" --output-on-failure
```

Acceptance:

```text
Baseline tests pass before behavior changes.
Golden values are explicit enough to catch accidental formula drift.
```

---

### Step 1 — Fix H/V Integrity Semantics

Goal: make current monitor state and IM mathematically consistent with HPL/HAL
and VPL/VAL.

Problem in current code:

```text
IntegrityMonitor computes report.IM = report.AL - report.PL.
State machine uses scalar report.PL >= report.AL.
report.PL is effectively fused HPL.

This can miss VPL >= VAL cases.
```

Tasks:

```text
1. Add explicit report fields or helper accessors:
   - im_h = HAL - HPL
   - im_v = VAL - VPL
   - IM = min(im_h, im_v)
2. Update IntegrityMonitor::update_state() to use:
   - unsafe if HPL >= HAL OR VPL >= VAL
   - safe only if HPL < HAL AND VPL < VAL
3. Keep existing ROS field `im`, but redefine it as min(HAL-HPL, VAL-VPL).
4. Audit logs and warnings so they report horizontal vs vertical cause.
5. Preserve legacy scalar `PL` as monitor_fused_hpl unless a broader message
   migration is done later.
```

Required tests:

| Test | File | Required check |
|---|---|---|
| Horizontal unsafe | `test/test_integrity_monitor.cpp` | `HPL >= HAL`, `VPL < VAL` gives UNSAFE and negative/zero `im_h` |
| Vertical unsafe | same | `HPL < HAL`, `VPL >= VAL` gives UNSAFE and negative/zero `im_v` |
| Both safe | same | `HPL < HAL`, `VPL < VAL` can recover to SAFE after recovery count |
| IM scalar semantics | same | Published/internal `IM == min(HAL-HPL, VAL-VPL)` |
| Regression | `test/test_araim.cpp` | GNSS ARAIM formulas unchanged |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_integrity_monitor|test_araim" --output-on-failure
```

Acceptance:

```text
No test can construct VPL >= VAL and receive SAFE.
The scalar IM no longer hides vertical violations.
Existing ARAIM unit tests remain unchanged.
```

---

### Step 2 — Numerical Guard and Failure Semantics

Goal: prevent invalid current integrity output from entering `/iap/integrity`
without explicit flags.

Tasks:

```text
1. Add a small numerical guard utility for:
   - finite scalar/vector/matrix checks
   - non-negative variance checks
   - positive-definite or acceptable LDLT checks
2. Apply guards in GNSS ARAIM:
   - G, W, r dimensions
   - finite elevations/azimuths/sigmas/residuals
   - finite S0, Sk, p0, pk
   - finite HPL/VPL/PL_E/PL_N/PL_U
3. Apply guards in fallback covariance PL:
   - frame.sigma_p all finite
   - lambda_max finite and non-negative
4. Apply guards in LiDAR integrity result before fusion.
5. Define invalid-source behavior:
   - invalid source is not fused in `max_pl`
   - if all sources invalid, publish conservative HPL/VPL and set failure flags
   - do not silently publish NaN/Inf
6. Add failure reason enum/string for debug output.
```

Important policy:

```text
Sentinel values such as 1e9 must be treated as conservative degraded outputs,
not as ordinary valid PL, unless a validity/failure flag explains why.
```

Required tests:

| Test | File | Required check |
|---|---|---|
| NaN residual | `test/test_araim.cpp` | Invalid or conservative flagged result; no NaN HPL/VPL |
| Inf sigma | same | Invalid or conservative flagged result; no Inf HPL/VPL |
| Singular geometry | same | Deterministic invalid/degraded result with failure reason |
| Bad fallback covariance | `test/test_integrity_monitor.cpp` | Conservative report with numerical failure flag |
| Bad LiDAR result | same or `test/test_lidar_integrity.cpp` | Invalid LiDAR source is not fused as valid |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_araim|test_integrity_monitor|test_lidar" --output-on-failure
```

Acceptance:

```text
No NaN/Inf HPL/VPL/IM can be produced by tested paths.
Every conservative fallback caused by numerical failure is observable.
```

---

### Step 3 — Runtime Source Breakdown Output

Goal: make fused integrity decisions debuggable online, not only in internal
C++ structs.

Tasks:

```text
1. Choose output strategy:
   Preferred: add msg/IntegrityDebug.msg and publish /iap/integrity_debug.
   Alternative: extend msg/IntegrityReport.msg if all consumers will be rebuilt.
2. Include:
   - gnss_hpl, gnss_vpl, gnss_valid
   - lidar_hpl, lidar_vpl, lidar_valid
   - fallback_hpl, fallback_vpl, fallback_valid
   - fused_hpl, fused_vpl
   - hal, val, im_h, im_v, im_min
   - fusion_mode
   - final_hpl_source, final_vpl_source
   - numerical_failure_flags or failure_reason
3. Update IntegrityExtensionModule to publish the debug output when enabled.
4. Update CSV/debug logs to use the same field names.
5. Keep `/iap/integrity` backward compatible unless intentionally changing the
   primary message contract.
```

Required tests:

| Test | File | Required check |
|---|---|---|
| Message generation | build test | New msg builds under ROS2 |
| Debug conversion | `test/test_integrity_report_builder.cpp` or extension unit helper | Internal report maps to debug msg fields correctly |
| CSV schema | `test/test_run_log_manager.cpp` or new schema test | Required debug columns exist |
| Topic smoke | launch/integration test | `/iap/integrity` still publishes; debug topic publishes when enabled |

Suggested commands:

```bash
colcon build --packages-select iap
colcon test --packages-select iap --ctest-args -R "test_integrity|test_run_log|test_araim" --output-on-failure
```

Acceptance:

```text
An operator can tell whether GNSS, LiDAR, fallback, fusion, or H/V limits caused
the final state.
Primary `/iap/integrity` consumers do not silently lose compatibility.
```

---

### Step 4 — Explicit GNSS/LiDAR/Fallback Fusion Policy

Goal: replace ad hoc in-place report mutation with a tested fusion policy.

Current behavior to preserve as default:

```text
Initialize from fallback covariance PL.
If GNSS valid, replace report with GNSS.
If LiDAR valid, max-fuse LiDAR with current report.
```

New behavior:

```text
Build independent source results:
  fallback_result
  gnss_result
  lidar_result

Fuse them using IntegrityFusionPolicy.
```

Tasks:

```text
1. Add `IntegritySourceResult` or equivalent internal struct.
2. Add `IntegrityFusionPolicy` with modes:
   - gnss_only
   - lidar_only
   - fallback_only
   - max_pl
   - weighted_debug_only
3. Add flat config keys from Section 3.
4. Keep `enable_araim` as deprecated alias for `enable_gnss_araim`.
5. Ensure disabled source is marked disabled, not invalid numerical failure.
6. Ensure `require_valid_gnss` and `require_valid_lidar` force conservative
   output when required sources are missing.
```

Required tests:

| Test | File | Required check |
|---|---|---|
| GNSS-only mode | `test/test_integrity_fusion_policy.cpp` | Final HPL/VPL equal GNSS result |
| LiDAR-only mode | same | Final HPL/VPL equal LiDAR result |
| Fallback-only mode | same | Final HPL/VPL equal fallback result |
| Max-PL mode | same | Per-axis max across valid enabled sources |
| Missing required source | same | Conservative output and failure reason |
| Config alias | `test/test_integrity_config.cpp` or extension test | `enable_araim` still controls GNSS during transition |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_integrity_fusion|test_integrity_monitor|test_araim" --output-on-failure
```

Acceptance:

```text
Fusion behavior is unit-tested without ROS.
Default max_pl mode matches existing behavior except for intentional H/V and
failure-flag fixes.
```

---

### Step 5 — Clarify Trunk and Constellation Hypothesis Semantics

Goal: prevent the GNSS ARAIM evaluator from overclaiming unsupported fault
handling.

Observed current behavior:

```text
GNSS ARAIM enumerates trunk hypotheses, but trunk hypotheses have no GNSS row
and return zero/default subset results in compute_core().
Constellation hypotheses can create degenerate subsets and 1e9 PL.
```

Tasks:

```text
1. Decide and document trunk policy for current GNSS ARAIM:
   Option A: remove trunk hypotheses from GNSS evaluator and leave trunk/LiDAR
             integrity to LiDARIntegrityEvaluator.
   Option B: keep trunk hypotheses only as budget placeholders, but rename/debug
             them as unsupported_placeholder and never count them as detected.
2. Add a config switch for constellation hypotheses:
   enable_gnss_constellation_faults.
3. Define degenerate constellation subset behavior:
   - conservative degraded source with flag, or
   - skip invalid hypothesis with explicit diagnostic.
4. Update tests that currently tolerate `HPL < 1e10` so they assert the intended
   behavior directly.
```

Required tests:

| Test | File | Required check |
|---|---|---|
| Trunk policy | `test/test_araim.cpp` | Trunk hypotheses either absent or explicitly placeholder-flagged |
| Constellation disabled | same | Hypothesis count excludes constellation hypotheses |
| Degenerate constellation | same | Result has deterministic degraded/invalid flag, not accidental pass |
| No false trunk detection | same | Unsupported trunk placeholder cannot appear as real excluded trunk |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_araim" --output-on-failure
```

Acceptance:

```text
The implementation no longer implies GNSS ARAIM truly solves trunk fault
hypotheses unless that math exists.
Degenerate constellation behavior is intentional and tested.
```

---

### Step 6 — Move Planner Geometry Prediction Out of Current ARAIM

Goal: separate Future Predictor from Current ARAIM before renaming current
classes.

Tasks:

```text
1. Create planner-side `GnssGeometryPlPredictor` or `AdvisoryGnssPlComputer`.
2. Move the behavior of `Araim::predict_geometry()` into that class.
3. Keep `Araim::predict_geometry()` as a deprecated compatibility wrapper for
   one transition window.
4. Update `PredictedAraimComputer` so it does not own a current `Araim` object.
5. Keep planner output names advisory:
   - advisory_hpl
   - advisory_vpl
   - advisory_pdop
   - predicted_validity
6. Add TODO marker:
   `TODO(IAP-ARAIM-REFAC-PREDICTOR): remove Araim::predict_geometry() after planner predictor migration.`
```

Required tests:

| Test | File | Required check |
|---|---|---|
| Predictor compatibility | `test/test_predicted_araim.cpp` | New predictor matches old `predict_geometry()` output for fixed geometry |
| Missing epoch fallback | same | Existing fallback reasons preserved |
| Too few sats fallback | same | Existing fallback behavior preserved |
| Current ARAIM independence | include/build test | Planner predictor header no longer includes current solver header, except temporary wrapper if needed |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_predicted_araim|test_future_pl_field_predictor|test_araim" --output-on-failure
```

Acceptance:

```text
Planner behavior remains unchanged.
Current ARAIM no longer owns planner prediction semantics.
```

---

### Step 7 — Rename and Isolate Current GNSS ARAIM

Goal: make current GNSS monitor semantics explicit after behavior is protected.

Tasks:

```text
1. Rename `Araim` to `GnssAraimEvaluator` or `CurrentGnssAraimSolver`.
2. Move `Araim::Params` to `GnssAraimParams`.
3. Rename `AraimResult` to `GnssAraimResult` only if the compatibility cost is
   acceptable; otherwise add aliases first.
4. Keep deprecated aliases:
   - Araim
   - AraimResult
5. Update IntegrityMonitor and tests.
6. Do not change formulas in this step.
```

Suggested compatibility aliases:

```cpp
using Araim [[deprecated("Use GnssAraimEvaluator")]] = GnssAraimEvaluator;
using AraimResult [[deprecated("Use GnssAraimResult")]] = GnssAraimResult;
```

Required tests:

| Test | File | Required check |
|---|---|---|
| GNSS ARAIM regression | `test/test_araim.cpp` | Golden values unchanged from Step 0/2 |
| Monitor integration | `test/test_integrity_monitor.cpp` | Monitor still consumes GNSS evaluator |
| Deprecated API compile | existing tests or small compile test | Temporary aliases still compile |
| Predictor separation | `test/test_predicted_araim.cpp` | Predictor does not regress |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_araim|test_integrity_monitor|test_predicted_araim" --output-on-failure
```

Acceptance:

```text
Rename does not change numerical output.
Runtime integrity extension still publishes `/iap/integrity`.
Planner-side code no longer directly depends on current GNSS ARAIM except
temporary deprecated wrappers.
```

---

### Step 8 — Introduce GnssEpoch Linearized Input Seam

Goal: decouple gradually without rewriting GNSS data flow.

Tasks:

```text
1. Introduce `GnssAraimLinearizedInput`:
   - G
   - W
   - r
   - prns
   - constellation_ids
   - elevations_rad
   - sigmas_m
2. Implement `buildLinearizedInputFromGnssEpoch()`.
3. Make public `run(GnssEpoch)` convert internally.
4. Add `runLinearized()` for unit tests and later adapter split.
5. Keep `GnssEpoch` public API for now.
```

Required tests:

| Test | File | Required check |
|---|---|---|
| Epoch vs linearized equivalence | `test/test_araim.cpp` | Same HPL/VPL/hypothesis outputs for fixed epoch |
| Linearized bad dimensions | same | Invalid flagged result |
| Linearized finite guards | same | NaN/Inf rejected |
| Header dependency smoke | build test | New linearized type compiles without constructing full epoch |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_araim" --output-on-failure
```

Acceptance:

```text
Existing GNSS ARAIM behavior unchanged.
New `runLinearized()` can be tested without a full `GnssEpoch`.
```

---

### Step 9 — Decompose Large Compute and Monitor Functions

Goal: improve maintainability after semantics and tests are stable.

Tasks:

```text
1. Split GNSS evaluator internals:
   - buildFullSolution()
   - allocateIntegrityRisk()
   - computeFaultFreeProtectionLevel()
   - enumerateFaultHypotheses()
   - solveSubset()
   - computeSolutionSeparation()
   - accumulateWorstProtectionLevel()
   - runFaultDetection()
2. Split monitor orchestration:
   - buildFallbackSource()
   - evaluateGnssSource()
   - evaluateLidarSource()
   - fuseSources()
   - computeMarginsAndState()
   - buildReport/debug output
3. Add debug structures for dominant hypothesis/source.
4. Preserve golden outputs.
```

Required tests:

| Test | File | Required check |
|---|---|---|
| GNSS golden regression | `test/test_araim.cpp` | Outputs unchanged |
| Fusion regression | `test/test_integrity_fusion_policy.cpp` | Outputs unchanged |
| Monitor regression | `test/test_integrity_monitor.cpp` | H/V state unchanged |
| Debug dominant source | same | Worst source/hypothesis fields populated |

Suggested command:

```bash
colcon test --packages-select iap --ctest-args -R "test_araim|test_integrity" --output-on-failure
```

Acceptance:

```text
Refactor changes structure only.
Golden tests pass before and after the split.
```

---

### Step 10 — Library Target Split

Goal: enable independent compilation and testing only after APIs settle.

Tasks:

```text
1. Add `iap_integrity_common`.
2. Add `iap_current_integrity`.
3. Add `iap_prediction_integrity`.
4. Keep `libiap.so` linking these targets for compatibility.
5. Move tests to the narrowest target they need.
6. Do not change runtime behavior.
```

Required tests:

| Test | File/target | Required check |
|---|---|---|
| Current integrity target | `test_araim`, `test_integrity_monitor` | Link against current integrity target |
| Prediction target | `test_predicted_araim`, `test_future_pl_field_predictor` | Link against prediction target |
| Full package build | CMake/colcon | Existing apps and plugins still link |
| Runtime smoke | launch/integration | `libintegrity_extension.so` still loads |

Suggested commands:

```bash
colcon build --packages-select iap
colcon test --packages-select iap --output-on-failure
```

Acceptance:

```text
Target split does not break existing applications, plugins, or tests.
Current and prediction tests no longer require unrelated odometry/mapping symbols
where practical.
```

---

## 9. Risk Register

| Risk | Cause | Mitigation |
|---|---|---|
| Vertical alert violation missed | State machine uses scalar HPL/AL only | Step 1 fixes H/V IM and state tests |
| NaN/Inf or sentinel PL published silently | Numerical guards and failure flags incomplete | Step 2 requires finite guards and observable failure reasons |
| Source of unsafe state unclear | ROS output lacks GNSS/LiDAR/fallback breakdown | Step 3 adds debug message/topic or extends report |
| Fusion modes change behavior accidentally | Existing fallback/GNSS/LiDAR mutation order is implicit | Step 4 introduces tested `IntegrityFusionPolicy` |
| GNSS ARAIM overclaims trunk fault support | Trunk hypotheses are enumerated but not solved in GNSS WLS path | Step 5 either removes or marks placeholders explicitly |
| Planner breaks after removing `predict_geometry()` | Predictor still calls old ARAIM API | Step 6 keeps deprecated wrapper for one transition phase |
| GNSS ARAIM behavior changes during rename | Rename mixed with formula edits | Step 7 is rename-only and guarded by golden tests |
| GnssEpoch decoupling causes broad breakage | Too much API rewrite at once | Step 8 adds internal linearized seam while preserving public API |
| CMake split causes dependency chaos | Existing `libiap.so` expects all symbols | Step 10 keeps `libiap.so` linking new targets until full modularization |

---

## 10. Validation Matrix

| Step | Test target | Required result |
|---|---|---|
| Step 0 | Baseline golden tests | Current behavior captured before changes |
| Step 1 | H/V monitor tests | `VPL >= VAL` cannot be SAFE; `IM=min(H/V margins)` |
| Step 2 | Numerical guard tests | No NaN/Inf HPL/VPL/IM; failures flagged |
| Step 3 | Debug output tests | Source breakdown visible in ROS msg/topic or schema |
| Step 4 | Fusion policy tests | `gnss_only`, `lidar_only`, `fallback_only`, `max_pl` deterministic |
| Step 5 | Hypothesis semantics tests | Trunk and constellation behavior explicit |
| Step 6 | Predictor compatibility tests | New planner predictor matches old geometry behavior |
| Step 7 | Rename regression tests | GNSS ARAIM numerical outputs unchanged |
| Step 8 | Linearized input tests | `run(GnssEpoch)` equals `runLinearized()` |
| Step 9 | Decomposition regression tests | Structure changed, outputs unchanged |
| Step 10 | Build/link tests | Split targets and existing plugins/apps build |

Minimum command set before merging any implementation step:

```bash
colcon build --packages-select iap
colcon test --packages-select iap --ctest-args --output-on-failure
```

For ROS message or launch-affecting steps, also run a smoke check that verifies:

```text
/iap/integrity is published.
/iap/integrity_debug is published when enabled, if the debug-topic option is used.
No ERROR logs are emitted during startup.
```

---

## 11. Recommended Codex Prompt

```text
We need to refactor the IAP ARAIM module according to the updated
ARAIM_refactor_plan_v2.md.

Important ordering constraints:
1. Do not start with renames. First fix behavior and observability.
2. Step 1: fix H/V safety semantics:
   - IM = min(HAL-HPL, VAL-VPL)
   - unsafe if HPL >= HAL OR VPL >= VAL
   - safe only if both dimensions are below their alert limits
3. Step 2: add numerical guards and explicit failure flags.
4. Step 3: expose source breakdown through ROS debug output or an extended report.
5. Step 4: implement tested fusion modes for fallback/GNSS/LiDAR.
6. Step 5: clarify unsupported trunk hypotheses and degenerate constellation behavior.
7. Step 6: move planner geometry prediction out of current ARAIM with a compatibility wrapper.
8. Only then rename/isolate Current GNSS ARAIM, add the GnssEpoch linearized seam,
   decompose large functions, and split CMake targets.

Architectural decisions:
- Do not model GNSS ARAIM and LiDAR integrity as subclasses of a generic AraimBase.
- Use independent evaluators composed by the current integrity monitor.
- Keep Current ARAIM and Future Predictor separate modules.
- Preserve current public behavior unless a step explicitly changes it and tests prove it.
- Keep existing config_gnss.json flat-key loading compatible during migration.

For every implementation step, provide:
- files modified
- public API or ROS message changes
- runtime behavior changes
- tests added/updated
- exact build/test commands and results
```

---

## 12. Final Target State

The final target state after this refactor is:

```text
Current integrity subsystem:
  - current-time integrity monitor
  - GNSS ARAIM evaluator
  - LiDAR integrity evaluator
  - fallback covariance source
  - explicit fusion policy
  - correct H/V margins and state machine
  - numerical failure flags
  - online source breakdown output
  - no planner prediction responsibility

Future prediction subsystem:
  - planner-side advisory GNSS geometry PL predictor
  - future PL field predictor
  - LiDAR observability/FIM predictor
  - no dependency on Current ARAIM solver

Common layer:
  - PL/AL/IM structs
  - validity/failure flags
  - numerical guards
  - source/fusion debug schema
```

The most important invariant is:

```text
Current ARAIM and Future Predictor are related by concept, not by solver ownership.
Current ARAIM is a monitoring module.
Future Predictor is an advisory planning module.
The current monitor is safe only when both horizontal and vertical protection
levels are below their corresponding alert limits.
```
