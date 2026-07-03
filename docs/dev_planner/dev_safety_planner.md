# IAP × EGO-Planner Integrity-Aware Refactor Handbook

## 0. Purpose

This handbook defines how to inject IAP integrity information into EGO-Planner without destroying the original EGO design.

The target system is:

```text
GNSS + LiDAR + IMU Estimator
        ↓
Current ARAIM Monitor
        ↓
PredictorModule
        ↓
PlannerIntegrityField
        ↓
EGO-Planner
```

The design goal is not to replace EGO-Planner with a new risk-grid planner. The goal is to make EGO-Planner integrity-aware while preserving its original structure:

```text
thin front-end + strong B-spline backend + no mandatory ESDF
```

---

## 1. Core Design Principles

### 1.1 Separate Preference from Safety

The backend optimizer should only express a preference for lower predicted localization risk.

The runtime supervisor should make hard safety decisions.

```text
Backend optimizer:
  Prefer low PL / low risk regions.

Runtime supervisor:
  Decide whether PLpred < AL is satisfied.
```

Do not claim that minimizing backend PL cost alone guarantees safety.

The hard safety condition is:

```math
PL_{pred}(p(t), t) < AL(p(t))
```

The planner may use a soft cost derived from predicted PL, but the final trajectory must still be checked before publication.

---

### 1.2 Do Not Merge Risk Into EGO GridMap

EGO's `GridMap` stores occupancy and inflated occupancy. It is high-rate, local, and geometry-focused.

IAP risk is a localization-side prediction. It has different semantics, resolution, update rate, and validity rules.

Therefore:

```text
Do:
  Keep PlannerIntegrityField independent from EGO GridMap.
  Align frame, rolling window center, and query convention.

Do not:
  Insert PL/risk layers directly into EGO's occupancy buffers.
```

---

### 1.3 One New Core Entity Only

Avoid unnecessary entities.

The only new core planning-side entity should be:

```cpp
PlannerIntegrityField
```

It owns:

```text
risk cache
time-horizon layers
query API
cost and gradient API
trajectory evaluation API
unknown / stale / fallback policy
```

Avoid creating separate classes such as:

```text
QuasiStaticSafetyGrid
MultiHorizonSafetyGrid
TrajectoryBatchValidator
RiskMapForAstar
RiskGridForOptimizer
PlannerSafetyAdapter
```

Those are different functions of the same field, not separate subsystems.

---

### 1.4 EGO Should Only See a Query Interface

EGO should not understand GNSS, LiDAR, ARAIM hypotheses, FIM, or predictor internals.

EGO should only query:

```cpp
costAndGrad(p, t)
evaluateTrajectory(traj, t0)
```

The predictor remains a generic advisory library. The planner only consumes cached risk/cost information through `PlannerIntegrityField`.

---

## 2. Terminology

### 2.1 Protection Level

`PL` is a localization-side bound on position error.

For planner use:

```text
HPL: horizontal protection level
VPL: vertical protection level
PL scalar: max(HPL, VPL) or configured scalar reduction
```

### 2.2 Alert Limit

`AL` is the environment-side tolerable error.

A simple form is:

```math
AL(p) = min(AL_H(p), AL_V(p))
```

where:

```math
AL_H(p) = γ_H · clearance_proxy(p)
AL_V(p) = γ_V · vertical_clearance(p)
```

### 2.3 Integrity Margin

```math
IM(p,t) = AL(p) - PL_{pred}(p,t)
```

Interpretation:

```text
IM > 0:
  predicted localization error bound is within environment tolerance.

IM <= 0:
  trajectory is not integrity-safe under the selected policy.
```

### 2.4 Homotopy Class

A homotopy class is a path topology category.

Two paths are in the same homotopy class if one can be continuously deformed into the other without crossing obstacles.

Example:

```text
start ---- obstacle ---- goal
```

A path going above the obstacle and a path going below the obstacle are usually different homotopy classes.

EGO's backend optimizer is local. It can improve a trajectory inside the current corridor, but it usually cannot jump to a different corridor. This is why risk-aware A* is only needed when the safe route belongs to another homotopy class.

No `HomotopyClass` class should be implemented. This is only a planning concept.

---

## 3. Final Architecture

```text
Existing IAP:
  PredictorModule
    - query predicted GNSS PL
    - query predicted LiDAR PL
    - query fused advisory PL
    - returns validity, fallback, source flags

New planning-side component:
  PlannerIntegrityField
    - rebuilds cached PL/risk layers from PredictorModule
    - supports time-aware query using horizon layers
    - returns planner cost and spatial gradient
    - evaluates final trajectory safety

Existing EGO:
  EGOReplanFSM
    - current integrity gate
    - final trajectory gate

  EGOPlannerManager
    - candidate initial trajectory scoring
    - optional local risk-aware A* initial guide

  BsplineOptimizer
    - trajectory-sampled PL preference cost
```

---

## 4. PlannerIntegrityField

### 4.1 Responsibility

`PlannerIntegrityField` is the only new core class.

It should provide:

```text
1. cached PL/risk field construction
2. time-aware risk query
3. planner cost and gradient query
4. trajectory-level integrity evaluation
5. unknown / stale / fallback handling
6. double-buffered safe access
```

It should not:

```text
1. estimate GNSS or LiDAR PL directly
2. run ARAIM internally
3. own EGO occupancy map
4. replace EGO collision checking
5. publish trajectories
6. decide FSM states directly
```

---

### 4.2 Suggested Public API

```cpp
class PlannerIntegrityField {
public:
  struct Params;
  struct QueryResult;
  struct CostGradResult;
  struct TrajectoryReport;

  explicit PlannerIntegrityField(const Params& params);

  void setPredictor(std::shared_ptr<PredictorModule> predictor);

  void setEgoGridMap(std::shared_ptr<GridMap> ego_grid_map);

  void rebuild(const IntegritySnapshot& snapshot,
               double field_stamp_s,
               const std::vector<double>& horizons_s);

  QueryResult query(const Eigen::Vector3d& p_map,
                    double query_time_s) const;

  CostGradResult costAndGrad(const Eigen::Vector3d& p_map,
                             double query_time_s) const;

  TrajectoryReport evaluateTrajectory(
      const UniformBspline& traj,
      double traj_start_time_s,
      EvalMode mode) const;

  bool isFresh(double now_s) const;

  uint64_t generationId() const;
};
```

---

### 4.3 Query Modes

Use one class and one implementation. Different behavior is controlled by parameters.

```cpp
enum class EvalMode {
  CachedFieldFast,
  DirectPredictorExact
};
```

Use cases:

```text
Optimizer inner loop:
  CachedFieldFast

Candidate scoring:
  CachedFieldFast

Final trajectory publication check:
  DirectPredictorExact if latency allows
  otherwise CachedFieldFast with conservative margin

Runtime monitoring:
  CachedFieldFast at high frequency
  DirectPredictorExact for selected trajectories when possible
```

---

### 4.4 Horizon Vector

Do not implement separate static and multi-horizon grids.

Use one field with a horizon vector:

```cpp
std::vector<double> horizons_s;
```

Examples:

```yaml
horizons_s: [0.0]
```

This means quasi-static use.

```yaml
horizons_s: [0.0, 0.5, 1.0, 1.5, 2.0]
```

This means time-aware multi-horizon use.

Both cases use the same class and the same query API.

---

### 4.5 Query Time Semantics

Let:

```text
field_stamp_s = snapshot.stamp
query_time_s = trajectory_start_time_s + sample_relative_time_s
τ = query_time_s - field_stamp_s
```

Then `PlannerIntegrityField::query(p, query_time_s)` should:

```text
1. compute τ
2. find the two horizon layers surrounding τ
3. do trilinear interpolation in each layer
4. do linear interpolation across horizon
5. return PL / flags / cost / gradient / age
```

If `τ` is outside the supported horizon range:

```text
before first layer:
  clamp to first layer or return stale depending on policy

after last layer:
  clamp to last layer with stale flag or return unavailable depending on policy
```

Recommended default:

```text
τ < 0:
  use horizon 0 but mark query_time_before_field_stamp

τ > max_horizon:
  return unavailable unless allow_horizon_clamp = true
```

---

## 5. Grid Data Model

### 5.1 Raw Layers

Each voxel per horizon should store raw predictor outputs:

```cpp
struct IntegrityVoxelRaw {
  float hpl_adv = NaN;
  float vpl_adv = NaN;
  float pl_scalar = NaN;

  uint32_t source_flags = 0;

  bool valid = false;
  bool available = false;
  bool fallback = true;

  uint16_t fallback_reason_code = 0;

  float query_time_s = NaN;
  float age_s = NaN;
};
```

Raw layers are predictor-facing.

They should not depend on planner weights.

---

### 5.2 Derived Layers

Derived values may be cached for speed but should be treated as planner-facing.

```cpp
struct IntegrityVoxelDerived {
  float al = NaN;
  float im = NaN;
  float c_pi = NaN;

  bool unknown = true;
  bool stale = true;
  bool near_obstacle = false;
};
```

Derived layers depend on:

```text
AL policy
unknown policy
stale policy
cost weights
soft margin
normalization
saturation
```

Therefore, raw PL should always be stored even if derived `c_pi` is cached.

---

### 5.3 Query Result

```cpp
struct PlannerIntegrityField::QueryResult {
  bool valid = false;
  bool available = false;
  bool unknown = true;
  bool stale = true;
  bool fallback = true;

  Eigen::Vector3d p_map = Eigen::Vector3d::Zero();
  double query_time_s = 0.0;
  double horizon_s = 0.0;

  double hpl_adv = std::numeric_limits<double>::quiet_NaN();
  double vpl_adv = std::numeric_limits<double>::quiet_NaN();
  double pl_scalar = std::numeric_limits<double>::quiet_NaN();

  double al = std::numeric_limits<double>::quiet_NaN();
  double im = std::numeric_limits<double>::quiet_NaN();

  double age_s = std::numeric_limits<double>::infinity();
  uint64_t generation_id = 0;
  uint32_t source_flags = 0;

  std::string fallback_reason;
};
```

---

### 5.4 Cost and Gradient Result

```cpp
struct PlannerIntegrityField::CostGradResult {
  bool valid = false;
  bool use_gradient = false;

  double cost = 0.0;
  Eigen::Vector3d grad_p = Eigen::Vector3d::Zero();

  QueryResult query;
};
```

If the query is unknown, stale, or unavailable, return a conservative cost.

Default gradient behavior:

```text
known valid voxel:
  use interpolated gradient

unknown / unavailable:
  cost = high constant
  gradient = zero

stale:
  cost = high constant
  gradient = zero
```

Reason: do not generate fake gradients from missing data.

---

## 6. Double Buffering

`PlannerIntegrityField` must be safe for simultaneous grid rebuild and optimizer query.

Use double buffering:

```text
front buffer:
  read-only, used by optimizer and FSM

back buffer:
  rebuilt asynchronously

swap:
  atomic or mutex-protected swap after rebuild finishes
```

Suggested implementation:

```cpp
struct GridBuffer {
  uint64_t generation_id = 0;
  double stamp_s = 0.0;
  std::vector<double> horizons_s;
  std::vector<IntegrityVoxelRaw> raw;
  std::vector<IntegrityVoxelDerived> derived;
};
```

Access rule:

```text
The optimizer only reads front buffer.
The builder only writes back buffer.
No query should read a partially updated buffer.
```

Each query should return:

```text
generation_id
age_s
stale flag
```

The final trajectory report should also record which grid generation was used.

---

## 7. Alert Limit Policy

### 7.1 Purpose

The backend does not need AL to prefer low PL.

The runtime supervisor needs AL to decide whether:

```math
PL_{pred} < AL
```

Therefore AL computation belongs to `PlannerIntegrityField` or its internal policy function, not to `BsplineOptimizer`.

---

### 7.2 Minimal AL Policy

Use one enum, not multiple classes.

```cpp
enum class AlertLimitMode {
  Constant,
  OccupancyShellSearch,
  CoarseClearanceLayer
};
```

---

### 7.3 Constant Mode

Useful for early testing and environments with fixed safety tolerance.

```math
AL_H = constant_al_h
AL_V = constant_al_v
AL = min(AL_H, AL_V)
```

YAML:

```yaml
alert_limit_mode: constant
constant_al_h_m: 2.0
constant_al_v_m: 1.0
```

---

### 7.4 Occupancy Shell Search Mode

Use EGO's inflated occupancy map to estimate a local clearance proxy.

This is only used for trajectory evaluation and hard gate.

It should not introduce a full ESDF.

Pseudo-code:

```cpp
double clearanceProxy(const Eigen::Vector3d& p) {
  for radius from min_radius to max_radius step resolution:
    for each voxel on shell(radius):
      if ego_grid_map->getInflateOccupancy(voxel_center):
        return radius;
  return max_radius;
}
```

Then:

```math
AL_H(p) = γ_H · clearance_proxy(p)
```

This is approximate but compatible with EGO's no-ESDF design.

---

### 7.5 Coarse Clearance Layer Mode

If a coarse clearance layer already exists in the integrity field, it may be used.

Do not create this layer unless needed.

```text
Allowed:
  coarse clearance as a derived layer in PlannerIntegrityField

Not allowed:
  separate ESDF subsystem just for AL
```

---

## 8. Planner Cost Definition

### 8.1 Backend Cost Is a Preference

The backend integrity cost should primarily prefer lower predicted PL.

Default backend cost:

```math
J_I =
λ_{pref} · ψ(PL_{pred})
+
λ_{unk} · J_{unknown}
+
λ_{stale} · J_{stale}
+
λ_{fallback} · J_{fallback}
```

Optional soft margin term:

```math
J_{margin}
=
λ_{margin}
· smoothHinge(PL_{pred} - AL + m)^2
```

The margin term is optional because EGO already handles obstacle avoidance through its rebound collision cost.

Recommended default:

```text
Backend:
  use PL preference + unknown/stale penalty

P5 supervisor:
  use IM hard check
```

---

### 8.2 Smooth Hinge

Use a smooth hinge to avoid non-smooth gradients.

For example:

```math
smoothHinge(x) = 0.5 · (x + sqrt(x^2 + ε^2))
```

Pseudo-code:

```cpp
double smoothHinge(double x, double eps) {
  return 0.5 * (x + std::sqrt(x * x + eps * eps));
}
```

---

### 8.3 PL Normalization

Do not optimize raw PL directly without normalization.

Recommended:

```math
z = clamp((PL - PL_ref) / PL_scale, 0, z_max)
```

Then:

```math
J_{pref} = z^2
```

Pseudo-code:

```cpp
double normalizedPL(double pl) {
  double z = (pl - params.pl_ref_m) / params.pl_scale_m;
  return std::clamp(z, 0.0, params.pl_norm_max);
}
```

---

### 8.4 Gradient Clipping

Integrity gradient must not dominate collision gradient.

```cpp
if (grad.norm() > params.max_integrity_grad_norm) {
  grad = grad.normalized() * params.max_integrity_grad_norm;
}
```

Recommended initial value:

```yaml
max_integrity_grad_norm: 10.0
```

---

## 9. Backend B-Spline Integrity Cost

### 9.1 Do Not Use Control-Point-Only Cost as Formal Implementation

Control-point-only cost:

```math
J_I = Σ_i c(Q_i)
```

is only acceptable for debug or ablation.

Formal implementation should sample the B-spline trajectory.

Reason:

```text
Risk field is not convex.
Low-risk control points do not guarantee low-risk curve interior.
```

---

### 9.2 Trajectory-Sampled Cost

Let:

```math
p(t_r) = Σ_i B_i(t_r) Q_i
```

Define:

```math
J_I(Q) = Σ_r c_I(p(t_r), t_r) Δt
```

Gradient:

```math
∂J_I / ∂Q_i =
Σ_r B_i(t_r) · ∇_p c_I(p(t_r), t_r) Δt
```

---

### 9.3 Pseudo-Code

```cpp
void BsplineOptimizer::calcIntegrityCost(
    const Eigen::MatrixXd& ctrl_pts,
    double& cost,
    Eigen::MatrixXd& grad) {

  cost = 0.0;
  grad.setZero();

  if (!planner_integrity_field_) {
    return;
  }

  const double dt_sample = params.integrity_sample_dt;
  const double t_start = current_traj_start_time_s_;
  const double duration = estimateBsplineDuration(ctrl_pts);

  for (double t_rel = 0.0; t_rel <= duration; t_rel += dt_sample) {
    BsplineBasisInfo basis = evaluateBasis(t_rel);

    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    for (const auto& item : basis.active_items) {
      int i = item.ctrl_index;
      double b = item.basis_value;
      p += b * ctrl_pts.col(i);
    }

    double query_time_s = t_start + t_rel;

    auto cg = planner_integrity_field_->costAndGrad(p, query_time_s);

    double weight = dt_sample;
    cost += cg.cost * weight;

    if (!cg.use_gradient) {
      continue;
    }

    Eigen::Vector3d grad_p = cg.grad_p;

    for (const auto& item : basis.active_items) {
      int i = item.ctrl_index;
      double b = item.basis_value;

      if (!isOptimizedControlPoint(i)) {
        continue;
      }

      grad.col(i) += b * grad_p * weight;
    }
  }
}
```

---

### 9.4 Integration Into Existing EGO Cost

Existing cost:

```text
smoothness
distance / rebound collision
feasibility
swarm
terminal
```

Add:

```text
integrity preference
```

Combined form:

```math
J =
λ_s J_s
+ λ_o J_o
+ λ_f J_f
+ λ_{swarm} J_{swarm}
+ λ_t J_t
+ λ_I J_I
```

Pseudo-code:

```cpp
f_combine =
    lambda_smooth_ * f_smooth
  + lambda_dist_   * f_dist
  + lambda_feasi_  * f_feasi
  + lambda_integrity_ * f_integrity;

grad =
    lambda_smooth_ * g_smooth
  + lambda_dist_   * g_dist
  + lambda_feasi_  * g_feasi
  + lambda_integrity_ * g_integrity;
```

Collision and feasibility should remain higher priority than integrity preference.

---

## 10. Runtime Integrity Supervisor

### 10.1 Responsibility

The runtime supervisor is not a new class. It is logic added to FSM / planner manager using `PlannerIntegrityField`.

It has two hooks:

```text
1. Current integrity gate
2. Trajectory integrity gate
```

---

### 10.2 Current Integrity Gate

Runs before or during planning.

Inputs:

```text
Current ARAIM PLmon
Current pose
Current AL
Current predictor/grid availability
```

Policy:

```text
if current PLmon > current AL:
  trigger emergency stop / hover using existing EGO state

if current integrity is marginal:
  reduce max velocity
  shorten planning horizon
  increase replanning frequency
```

Do not add new FSM states unless existing states are insufficient.

Prefer existing transitions:

```text
REPLAN_TRAJ
EMERGENCY_STOP
```

---

### 10.3 Trajectory Integrity Gate

Runs after a candidate trajectory is generated and before publication.

It evaluates:

```text
min_IM
min_AL
max_PL
unknown_ratio
fallback_ratio
stale_ratio
worst_sample_time
worst_sample_position
```

Hard rejection rules:

```text
if min_IM < min_required_im_m:
  reject trajectory

if unknown_ratio > max_unknown_ratio:
  reject trajectory or force replan

if stale_ratio > max_stale_ratio:
  reject trajectory or force replan

if any sample unavailable near obstacle:
  reject trajectory
```

---

### 10.4 Trajectory Report

```cpp
struct PlannerIntegrityField::TrajectoryReport {
  bool accepted = false;

  double min_im = std::numeric_limits<double>::infinity();
  double min_al = std::numeric_limits<double>::infinity();
  double max_pl = 0.0;

  double unknown_ratio = 1.0;
  double stale_ratio = 1.0;
  double fallback_ratio = 1.0;

  double worst_time_s = 0.0;
  Eigen::Vector3d worst_position = Eigen::Vector3d::Zero();

  uint64_t generation_id = 0;
  std::string reject_reason;
};
```

---

### 10.5 Recommended Runtime Policy

```text
Case 1:
  current PLmon > AL
Action:
  EMERGENCY_STOP / hover

Case 2:
  final trajectory min_IM < 0
Action:
  reject trajectory, trigger REPLAN_TRAJ

Case 3:
  final trajectory min_IM is small but positive
Action:
  accept only if no better candidate exists;
  reduce speed or shorten horizon

Case 4:
  risk field stale
Action:
  reject if stale_age > max_grid_age_s

Case 5:
  predictor unavailable
Action:
  unknown cost in optimizer;
  reject final trajectory if unknown ratio is too high
```

---

## 11. Candidate Initial Trajectory Ranking

### 11.1 Purpose

Candidate ranking chooses the best existing initial guess.

It does not guarantee discovering a new homotopy class.

Candidate sources may include:

```text
previous trajectory extension
global reference sampling
straight-line polynomial
random polynomial fallback
risk-aware A* guide if enabled
```

---

### 11.2 Candidate Score

```math
Score =
w_{len} · length
+ w_{smooth} · smoothness
+ w_I · meanRisk
+ w_{worst} · maxRisk
+ w_{unk} · unknownRatio
```

Pseudo-code:

```cpp
double scoreInitialCandidate(const CandidateTraj& cand) {
  auto report = integrity_field_->evaluateTrajectory(
      cand.bspline,
      cand.start_time_s,
      EvalMode::CachedFieldFast);

  double score = 0.0;
  score += w_len * cand.length;
  score += w_smooth * cand.smoothness;
  score += w_integrity_mean * cand.mean_integrity_cost;
  score += w_integrity_worst * report.max_pl;
  score += w_unknown * report.unknown_ratio;

  if (report.min_im < params.min_required_im_m) {
    score += params.hard_reject_score;
  }

  return score;
}
```

---

### 11.3 Integration Point

In `EGOPlannerManager::reboundReplan()`:

```text
1. generate candidate initial trajectories
2. score each candidate using PlannerIntegrityField
3. choose lowest score
4. send selected initial control points to BsplineOptimizer
```

Do not force A* as a mandatory front-end.

---

## 12. Reference Bias

### 12.1 Local Reference Bias by Default

If only rolling `PlannerIntegrityField` exists, reference bias is local.

Do not claim mission-level global route optimization unless a global risk source already exists.

Default behavior:

```text
bias local reference samples away from high-risk cells
only inside the current local field
fallback to original reference when field is stale or unavailable
```

---

### 12.2 Global Reference Bias Only If Global Risk Exists

If there is already a global risk source, the global reference path may be selected using risk-aware search.

Do not introduce a new global risk map solely for this refactor.

---

## 13. Optional Risk-Aware A*

### 13.1 Purpose

Risk-aware A* is only for topology rescue.

Use it when:

```text
backend PL preference cannot escape a high-risk corridor
existing candidates do not cover a safer homotopy class
two-corridor or urban-canyon experiments fail without it
```

Do not make it the default planning path.

---

### 13.2 A* Cost

```math
cost(cell) =
step_cost
+ λ_occ · occupancy_penalty
+ λ_risk · risk_cost
+ λ_unknown · unknown_penalty
```

Rules:

```text
inflated occupied cell:
  forbidden

unknown risk:
  high but finite cost, unless policy says forbidden

stale risk:
  high cost or forbidden depending on max_grid_age_s
```

---

### 13.3 Output

A* should output only an initial guide path.

Then:

```text
A* guide path
  ↓
resample to point_set
  ↓
parameterizeToBspline
  ↓
normal EGO backend optimization
```

A* should not replace the B-spline optimizer.

---

### 13.4 Hysteresis

To avoid path flipping:

```text
keep previous selected corridor unless new corridor improves score by threshold
penalize frequent switching
fallback to original EGO initial guess if A* fails
```

Recommended parameter:

```yaml
a_star_switch_gain_threshold: 0.25
```

---

## 14. Unknown, Stale, and Fallback Policy

### 14.1 Optimizer Policy

For optimizer cost:

```text
valid:
  use interpolated cost and gradient

unknown:
  cost = unknown_cost
  gradient = zero

stale:
  cost = stale_cost
  gradient = zero

fallback:
  cost = fallback_cost
  gradient = zero or degraded interpolated gradient
```

Recommended default:

```yaml
unknown_cost: 10.0
stale_cost: 10.0
fallback_cost: 10.0
```

---

### 14.2 Runtime Supervisor Policy

For final trajectory check:

```text
unknown_ratio > max_unknown_ratio:
  reject

stale_ratio > max_stale_ratio:
  reject

fallback_ratio > max_fallback_ratio:
  reject or slow down

any unavailable sample near obstacle:
  reject
```

Recommended defaults:

```yaml
max_unknown_ratio: 0.2
max_stale_ratio: 0.2
max_fallback_ratio: 0.5
max_grid_age_s: 0.5
```

---

## 15. Suggested Parameters

```yaml
planner_integrity_field:
  enabled: true

  frame_id: map

  resolution_m: 0.5
  window_size_x_m: 20.0
  window_size_y_m: 20.0
  window_size_z_m: 5.0

  horizons_s: [0.0, 0.5, 1.0, 1.5, 2.0]
  max_grid_age_s: 0.5

  rebuild_rate_hz: 2.0
  active_voxels_only: true

  alert_limit_mode: occupancy_shell_search
  constant_al_h_m: 2.0
  constant_al_v_m: 1.0
  gamma_h: 0.8
  gamma_v: 0.8
  clearance_search_max_m: 5.0
  clearance_search_step_m: 0.25

  pl_ref_m: 0.5
  pl_scale_m: 2.0
  pl_norm_max: 5.0

  integrity_sample_dt_s: 0.1
  lambda_integrity: 1.0
  lambda_pl_preference: 1.0
  lambda_margin: 0.0
  lambda_unknown: 1.0
  lambda_stale: 1.0
  lambda_fallback: 1.0

  soft_margin_im_m: 0.5
  min_required_im_m: 0.0

  smooth_hinge_eps: 0.05
  max_integrity_grad_norm: 10.0

  unknown_cost: 10.0
  stale_cost: 10.0
  fallback_cost: 10.0

  max_unknown_ratio: 0.2
  max_stale_ratio: 0.2
  max_fallback_ratio: 0.5

  enable_direct_final_validation: true
  allow_horizon_clamp: false

  enable_candidate_ranking: true
  enable_reference_bias: false
  enable_risk_aware_astar: false
  a_star_switch_gain_threshold: 0.25
```

---

## 16. EGO Integration Map

### 16.1 Files Likely to Change

Adjust file names according to the actual fork.

```text
New:
  iap/include/iap/planning_integrity/planner_integrity_field.hpp
  iap/src/planning_integrity/planner_integrity_field.cpp

Modify:
  planner/bspline_opt/include/bspline_optimizer.h
  planner/bspline_opt/src/bspline_optimizer.cpp

  planner/plan_manage/include/ego_planner_manager.h
  planner/plan_manage/src/ego_planner_manager.cpp

  planner/plan_manage/src/ego_replan_fsm.cpp

Optional:
  planner/path_searching/*
```

---

### 16.2 BsplineOptimizer Hook

Add:

```cpp
void setPlannerIntegrityField(
    std::shared_ptr<PlannerIntegrityField> field);
```

Use it inside:

```text
combineCostRebound()
combineCostRefine()
```

Add:

```text
calcIntegrityCost()
```

---

### 16.3 PlannerManager Hook

Responsibilities:

```text
own or receive PlannerIntegrityField pointer
call candidate scoring
call final trajectory evaluation
pass field pointer into BsplineOptimizer
```

---

### 16.4 FSM Hook

Responsibilities:

```text
current integrity gate
trajectory publish gate
replan / emergency policy
logging of rejection reasons
```

Use existing EGO states first.

Do not add new FSM states unless absolutely necessary.

---

## 17. Logging and Diagnostics

Every planning cycle should log:

```text
grid_generation_id
grid_age_s
horizons_s
num_valid_voxels
num_unknown_voxels
num_fallback_voxels

trajectory_min_im
trajectory_max_pl
trajectory_unknown_ratio
trajectory_stale_ratio
trajectory_fallback_ratio
trajectory_reject_reason

backend_integrity_cost
backend_integrity_grad_norm_max
backend_integrity_query_count

candidate_scores
selected_candidate_id

current_PLmon
current_AL
current_IM
```

Recommended CSV columns:

```text
stamp,
generation_id,
grid_age_s,
traj_id,
sample_id,
t_rel,
x,y,z,
hpl,vpl,pl_scalar,
al,im,
c_pi,
valid,available,unknown,stale,fallback,
source_flags,
fallback_reason
```

---

## 18. Required Tests

### 18.1 Unit Tests

#### Test 1: Horizon Query

```text
Given:
  horizons_s = [0.0, 1.0]
  known PL values in both layers

Check:
  query at τ = 0.5 returns time-interpolated PL
```

#### Test 2: Trilinear Gradient

```text
Given:
  analytic linear field c(x,y,z) = ax + by + cz

Check:
  costAndGrad returns gradient [a,b,c]
```

#### Test 3: Unknown Policy

```text
Given:
  query outside valid grid

Check:
  cost = unknown_cost
  grad = zero
  unknown = true
```

#### Test 4: Double Buffer

```text
Given:
  optimizer queries front buffer while rebuild runs

Check:
  no partial read
  generation_id changes only after swap
```

---

### 18.2 Planner Tests

#### Test 5: Backend PL Preference

```text
Environment:
  no obstacle conflict
  left side high PL
  right side low PL

Expected:
  EGO + integrity cost shifts trajectory toward low PL side
```

#### Test 6: Collision Priority

```text
Environment:
  low PL near wall
  high PL farther from wall

Expected:
  trajectory does not collide or reduce clearance below limit
```

#### Test 7: Final Gate Rejection

```text
Given:
  optimized trajectory has PL > AL at one sample

Expected:
  trajectory rejected before publication
```

#### Test 8: Unknown Field Rejection

```text
Given:
  large unknown risk area on planned path

Expected:
  optimizer penalizes unknown
  final supervisor rejects if unknown_ratio exceeds threshold
```

#### Test 9: Two-Corridor Topology

```text
Corridor A:
  shorter, high PL

Corridor B:
  longer, low PL

Expected:
  backend-only may improve within selected corridor
  optional A* or reference bias selects low-risk corridor when enabled
```

---

## 19. Anti-Patterns

Do not implement:

```text
1. A separate SafetyGrid class for each time mode.
2. A separate TrajectoryBatchValidator class.
3. A new global risk map unless a global risk source already exists.
4. A mandatory A* front-end replacing EGO initial guess.
5. ESDF reconstruction just to compute AL.
6. Predictor calls inside every L-BFGS finite-difference gradient query.
7. Treat unknown predictor output as zero risk.
8. Treat backend PL preference as a formal safety guarantee.
9. Add new FSM states before trying existing REPLAN_TRAJ / EMERGENCY_STOP.
10. Store only c_PI without raw PL / flags / age.
```

---

## 20. One-Pass Implementation Checklist

This is not a staged design. All items belong to the final architecture.

### Core Field

```text
[ ] Implement PlannerIntegrityField.
[ ] Implement horizon-vector field layout.
[ ] Implement raw PL layers.
[ ] Implement derived cost layers.
[ ] Implement trilinear spatial interpolation.
[ ] Implement horizon interpolation.
[ ] Implement costAndGrad().
[ ] Implement double buffering.
[ ] Implement generation_id and age reporting.
```

### Predictor Integration

```text
[ ] Batch query PredictorModule during rebuild.
[ ] Store valid / available / fallback / source_flags.
[ ] Never convert unavailable prediction into safe finite PL.
[ ] Encode fallback reasons.
```

### EGO Backend

```text
[ ] Add PlannerIntegrityField pointer to BsplineOptimizer.
[ ] Add trajectory-sampled calcIntegrityCost().
[ ] Add B-spline chain-rule gradient.
[ ] Add lambda_integrity parameter.
[ ] Add gradient clipping.
[ ] Keep collision and feasibility costs higher priority.
```

### Runtime Supervisor

```text
[ ] Add current integrity gate in FSM or PlannerManager.
[ ] Add final trajectory integrity gate before publish.
[ ] Compute min_IM, max_PL, unknown_ratio, stale_ratio, fallback_ratio.
[ ] Reject unsafe trajectory.
[ ] Use existing REPLAN_TRAJ / EMERGENCY_STOP when possible.
```

### Candidate Ranking

```text
[ ] Score existing initial candidates using evaluateTrajectory().
[ ] Select lowest score.
[ ] Do not claim new homotopy class unless candidate generator provides it.
```

### Optional A*

```text
[ ] Keep disabled by default.
[ ] Use only as initial guide.
[ ] Add hysteresis.
[ ] Fall back to original EGO initial guess.
```

### Logging

```text
[ ] Log grid generation and age.
[ ] Log trajectory integrity report.
[ ] Log backend integrity cost and gradient norm.
[ ] Log rejection reasons.
```

---

## 21. Final Expected Behavior

With this design, EGO-Planner should behave as follows:

```text
1. If both sides are collision-free but one side has lower PL,
   the backend preference shifts the trajectory toward lower PL.

2. If the current corridor is safe but marginal,
   the trajectory remains collision-free while improving integrity margin.

3. If the final optimized trajectory violates PL < AL,
   the runtime supervisor rejects it before publication.

4. If predictor or grid data is unavailable,
   the system treats it as unknown risk, not as safe space.

5. If a safer path requires another corridor,
   optional risk-aware A* or reference bias can provide a better initial guess,
   but the B-spline backend remains the final optimizer.
```

---

## 22. Summary

The final architecture is:

```text
PredictorModule
  ↓
PlannerIntegrityField
  - cached time-aware PL/risk field
  - value and gradient query
  - final trajectory evaluation
  ↓
EGO-Planner
  - backend PL preference
  - candidate ranking
  - runtime hard gate
  - optional A* topology rescue
```

The central rule is:

```text
Backend cost guides.
Runtime supervisor decides.
Predictor predicts.
PlannerIntegrityField caches and translates.
EGO still plans.
```
