# ARAIM Implementation Changes

## Overview
Full implementation of the 12-phase ARAIM integrity monitoring system per `docs/ARIAM_TODO_Check_list.md`, with strict compliance to `docs/spec/talk_spec.pdf`.

## Files Modified

### Core Types
- **`include/iap/integrity/araim_types.hpp`** — Per-axis `SubsetSolution` with 3-term PL formula fields (§1.9-§1.11); enriched `AraimResult` with per-axis `PL_E/PL_N/PL_U`, `HPL=max(PL_E,PL_N)`, `VPL=PL_U`, K multipliers, FDE summary
- **`include/iap/integrity/integrity_types.hpp`** — 3-state `IntegrityState` (SAFE/SAFE_EXCLUDED/UNSAFE per §1.13); `PlannerState`; `DynamicALResult` (HAL+VAL per §1.12); expanded `IntegrityReport`; deprecated `IntegrityMode` retained for ABI compat
- **`include/iap/integrity/araim.hpp`** — `Params` struct with correct defaults per spec: `P_HMI_req=1e-7`, `P_FA_req=1e-5`, per-constellation fault priors, trunk fault priors

### Core Math
- **`src/iap/integrity/araim.cpp`** — Complete rewrite:
  - `Q_inv()`: Abramowitz & Stegun 26.2.23 + Newton-Raphson on Q-function
  - Dynamic budget allocation (§1.8): `K_ff = Q_inv(P_HMI_req/4)`, `K_fa = Q_inv(P_FA_req/(2·N_f))`
  - 3-term per-axis PL: `PL_{q,k} = |d_{q,k}| + K_{fa,k}·σ_{ss,q,k} + K_{md,k}·σ_{q,k}` (§1.11)
  - `HPL = max(PL_E, PL_N)` (not sqrt(σ²_E + σ²_N))
  - Per-axis fault detection (§1.10): `|d_{q,k}| > T_{q,k}`
  - FDE summary with `excluded_prns[]` and `excluded_trunk_ids[]`

### Integrity Monitor
- **`include/iap/integrity/integrity_monitor.hpp`** — 3-state machine, VAL params, new method signatures
- **`src/iap/integrity/integrity_monitor.cpp`** — Complete rewrite:
  - `compute_dynamic_AL()`: HAL from trunk geometry + VAL from altitude bounds (§1.12)
  - `update_state()`: 3-state logic per §1.13 with recovery counter
  - `update_mode_legacy()`: Old 4-state kept for backward compat

### Planner
- **`include/iap/planner/integrity_planner.hpp`** — Inherits `PlannerInterface`; cost function with `w_turn`, `w_infeasible`
- **`src/iap/planner/integrity_planner.cpp`** — HPL/AL hinge cost + D_turn + infeasibility penalty

### Trunk Map
- **`include/iap/trunk/trunk_map.hpp`** — `TrunkLandmark` EKF fields (P matrix, confirmed, active); EKF params
- **`src/iap/trunk/trunk_map.cpp`** — EKF predict/update cycle with 2×2 covariance; EMA fallback

### Build System
- **`CMakeLists.txt`** — Added `fgo_information_manager.cpp` source; `rosidl_generate_interfaces` with 4 msg files; C language support for rosidl; gtest infrastructure
- **`package.xml`** — Added rosidl dependencies

## Files Created

### New Implementation
- **`include/iap/integrity/fgo_information_matrix.hpp`** — `FGOPositionInfo` + `FGOInformationManager` (thread-safe iSAM2 Hessian extraction)
- **`src/iap/integrity/fgo_information_manager.cpp`** — `marginalCovariance(X(i))` extraction, LLT inversion, eigenvalue validation
- **`include/iap/planner/planner_interface.hpp`** — Pure virtual `PlannerInterface` base class for RL extensibility
- **`include/iap/planner/mdp_state.hpp`** — `MDPStateVector` (16 features) + `MDPStateAssembler` for RL interface
- **`include/iap/integrity/araim_debug.hpp`** — `AraimDebugCSV` (env-var gated) + `AraimDebugLogger` (4 verbosity levels)

### ROS2 Messages
- **`msg/IntegrityReport.msg`** — Per-axis ARAIM + integrity state + GNSS quality
- **`msg/DynamicALResult.msg`** — HAL/VAL/AL + trunk distance + altitude
- **`msg/TrunkLandmark.msg`** — Trunk landmark state for visualization
- **`msg/MDPState.msg`** — MDP state vector for RL agent

### Configuration
- **`config/araim_params.yaml`** — ~100 parameters: ARAIM budget, K multipliers, fault priors, integrity monitor, planner, FGO, debug

### Tests
- **`test/test_araim.cpp`** — 27 tests:
  - Q_inv accuracy, valid geometry, HPL = max(PL_E, PL_N)
  - 3-term PL formula verification, total PL max-over-hypotheses
  - Degenerate geometry, fault-free PL components, predict_geometry
  - Trunk hypotheses, S0 positive semi-definite
  - IntegrityState/PlannerState enums, DynamicALResult defaults
  - IntegrityReport safe/unsafe logic
  - TrunkMap: creation, association, EKF uncertainty reduction, EMA fallback, stale pruning, multiple landmarks
  - FaultHypothesis types and defaults

## Bug Fixes During Implementation
- **Q_inv()**: Fixed incorrect `erfc_inv` + `sqrt(2)` scaling (was computing erfc_inv then multiplying by sqrt(2) instead of using the direct normal quantile approximation)
- **Worst-hypothesis tracking**: Fixed operator-precedence bug in ternary expression that caused undefined behavior
- **TrunkObservation.id**: Fixed reference to non-existent field; use index instead
- **Eigen includes**: Added `<Eigen/LU>` for `Matrix2d::inverse()`, `<Eigen/Eigenvalues>` for `SelfAdjointEigenSolver`
- **rosidl**: Fixed uppercase field names to snake_case; fixed target name conflict; added C language support
