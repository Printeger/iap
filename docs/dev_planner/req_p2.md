# Phase 2 Requirement: PI-lite / Trajectory Integrity Evaluator for Demo10

You are working in:

```bash
/home/dev/ws_iap
```

Repository:

```text
Printeger/iap
branch: dev/iap
```

Phase 1 has already been implemented and passes validation. Your task is **Phase 2: PI-lite / trajectory integrity evaluator**.

---

## 0. High-Level Goal

Add a **read-only trajectory integrity evaluator** for a new demo:

```text
demo10_ego_planner_pi_lite_eval.launch.py
```

The evaluator must assess ego planner’s **future trajectory** and export predicted:

```text
AL_pred
PL_pred
IM_pred = AL_pred - PL_pred
```

along the trajectory.

It must also provide an offline analyzer that aligns the predicted values against actual ARAIM and tracking logs after the run.

The intended architecture is:

```text
demo9  = Phase 1 ordinary ego-planner closed-loop baseline
demo10 = demo9 closed-loop stack + read-only PI-lite trajectory evaluator
```

Phase 2 is **not** integrity-aware planning yet. It is only an evaluator.

---

## 1. Critical Constraints

You must obey all of the following:

```text
Do NOT add ARAIM cost to ego planner.
Do NOT modify bspline_opt cost function.
Do NOT modify path_searching.
Do NOT modify ARAIM math.
Do NOT modify IAP estimator / FGO.
Do NOT modify SO3 controller.
Do NOT modify simulator dynamics.
Do NOT use truth odom in the online evaluator.
Do NOT require full future solution-separation ARAIM prediction in Phase 2.
Do NOT add new custom ROS message types unless absolutely necessary.
Do NOT change demo9 behavior.
```

Phase 1 validation must still pass after all Phase 2 changes.

---

## 2. Phase 2 Concept

The first PI-lite version is intentionally simple:

```text
PI-lite = current ARAIM PL + future trajectory AL sampling + simple PL propagation model
```

The first version must support:

```text
1. Trajectory sampling from ego planner bspline or planner command.
2. Obstacle clearance query from map/cloud.
3. AL prediction from clearance.
4. PL prediction using a constant-current model.
5. IM prediction.
6. CSV export.
7. Offline alignment with actual ARAIM logs.
8. Validation script.
9. Documentation.
```

The goal of Phase 2 is to answer:

```text
Along the trajectory produced by ego planner:
- How close is the trajectory to obstacles?
- How does AL change along the trajectory?
- If the current PL is conservatively propagated forward, where is IM positive or negative?
- Can predicted AL/PL/IM be aligned with actual ARAIM and tracking logs?
```

Do not attempt to solve full future ARAIM in this phase.

---

## 3. Launch Integration Requirement

### 3.1 Do Not Modify Demo9 Behavior

`demo9_ego_planner_closed_loop.launch.py` is the official Phase 1 baseline.

It must remain a clean ordinary ego-planner closed-loop demo.

Do not add Phase 2 evaluator arguments or evaluator nodes to demo9.

Specifically, do not add:

```text
enable_phase2_integrity_eval
phase2_eval_horizon_s
phase2_eval_dt_s
phase2_pl_model
phase2_al_model
phase2_* parameters
phase2_planner_integrity_evaluator
```

to demo9.

### 3.2 Create Demo10

Create a new launch file:

```text
launch/demo10_ego_planner_pi_lite_eval.launch.py
```

Demo10 must be based on demo9 and must start the same closed-loop stack:

```text
- random forest map generator
- SO3 quadrotor simulator
- SO3 controller
- local_sensing LiDAR renderer
- GNSS simulator
- IAP rosnode with GNSS + ARAIM + sim extension
- ego planner
- traj_server
- Phase 1 logger if demo9 already uses it
```

Then demo10 must additionally start:

```text
phase2_planner_integrity_evaluator
```

The Phase 2 evaluator is read-only. It must not change planner, controller, simulator, IAP, or ARAIM behavior.

### 3.3 No Enable Flag

Do not add `enable_phase2_integrity_eval` to demo9.

Do not add `enable_phase2_integrity_eval` to demo10 either.

Demo10 enables the Phase 2 evaluator by definition.

---

## 4. Demo10 Launch Arguments

Demo10 should keep the important baseline parameters inherited from demo9, such as:

```text
start_rviz
run_duration_s
allow_truth_alignment
use_so3_dynamics
use_gnss
use_araim
planner_odom_topic
map_source
goal_x
goal_y
goal_z
```

For official Phase 2 validation, defaults should prefer:

```text
allow_truth_alignment := false
use_so3_dynamics     := true
use_gnss             := true
use_araim            := true
```

Add the following Phase 2 launch arguments only to demo10:

```text
phase2_eval_horizon_s,        default 5.0
phase2_eval_dt_s,             default 0.2
phase2_max_samples_per_traj,  default 30
phase2_pl_model,              default constant_current
phase2_al_model,              default cloud_clearance
phase2_drone_radius,          default 0.35
phase2_safety_buffer,         default 0.20
phase2_gamma_h,               default 0.8
phase2_gamma_v,               default 0.8
phase2_z_min,                 default 0.5
phase2_z_max,                 default 5.0
phase2_safe_margin,           default 0.0
phase2_publish_markers,       default true
```

Official command:

```bash
ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py \
  start_rviz:=false \
  run_duration_s:=60 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_gnss:=true \
  use_araim:=true \
  phase2_pl_model:=constant_current \
  phase2_al_model:=cloud_clearance
```

---

## 5. Online Evaluator Inputs

Add an online node named:

```text
phase2_planner_integrity_evaluator
```

Prefer placing it in the existing `iap` package unless the repository already has a phase tools package.

### 5.1 Required Inputs

#### IAP Odom

```text
topic: /drone_0_visual_slam/odom
type: nav_msgs/msg/Odometry
```

Usage:

```text
- current estimated pose
- current estimated velocity
- trajectory-relative evaluation
- current pose for distance-inflation models later
```

Online evaluator must use this odom source, not truth odom.

#### Planner B-spline / Trajectory

Preferred:

```text
topic: /drone_0_planning/bspline
type: use the actual ego planner bspline message type in the repository
```

If B-spline parsing is difficult, use fallback:

```text
topic: /drone_0_planning/pos_cmd
type: quadrotor_msgs/msg/PositionCommand
```

Priority:

```text
planner bspline > planner pos_cmd fallback
```

The goal is to evaluate the planner’s future trajectory, so B-spline is preferred.

#### Map / Cloud

Use the same map/cloud source used by demo10 planner.

Preferred:

```text
/map_generator/global_cloud
```

or the actual remapped planner cloud topic.

The evaluator should support a point-cloud based nearest obstacle query.

#### ARAIM State / Current PL

Use existing ARAIM ROS topic if available.

If no ARAIM ROS topic exists, online evaluator may:

```text
- export AL-only online with PL fields as NaN, or
- use a lightweight adapter if one already exists, or
- fill PL/current_PL later in the offline analyzer from export/iap_araim.csv
```

Do not implement a new ARAIM algorithm.

### 5.2 Forbidden Online Input

The online evaluator must not subscribe to:

```text
/sim/drone_0/truth_odom
```

Truth odom is allowed only in offline analyzer for evaluation and alignment.

---

## 6. Online Evaluator Behavior

The node must:

```text
- subscribe to IAP odom
- subscribe to planner bspline if available
- subscribe to planner pos_cmd as fallback
- subscribe to map/cloud
- maintain latest current HPL/VPL if available
- sample each received trajectory over horizon_s at sample_dt_s
- compute AL_H_pred, AL_V_pred, AL_pred
- compute PL_H_pred, PL_V_pred, PL_pred using constant_current model
- compute IM_H_pred, IM_V_pred, IM_pred
- publish optional visualization markers
- append export/integrity_along_planner_traj.csv
- write/update export/phase2_summary.json
```

The evaluator must be read-only and must not publish any planner command.

---

## 7. Trajectory Sampling

For each planner trajectory, sample:

```text
t_i = i * Δt,  i = 0, ..., N
0 <= t_i <= min(horizon_s, trajectory_duration)
```

Default parameters:

```yaml
phase2_eval_horizon_s: 5.0
phase2_eval_dt_s: 0.2
phase2_max_samples_per_traj: 30
```

Each trajectory should receive a monotonically increasing `traj_id`.

Each sample should include:

```text
sample_index
sample_t_from_now
sample_abs_time
position
velocity if available
acceleration if available
yaw if available
```

If velocity/acceleration/yaw are unavailable, use NaN for those fields.

---

## 8. AL Model

Use `phase2_al_model := cloud_clearance` in the first version.

For sample position `p`:

### 8.1 Horizontal / Obstacle AL

```text
nearest_obstacle_distance = min ||p - p_obs||
clearance_H = nearest_obstacle_distance - drone_radius - safety_buffer
AL_H_pred = gamma_H * max(clearance_H, 0)
```

Default parameters:

```yaml
phase2_drone_radius: 0.35
phase2_safety_buffer: 0.20
phase2_gamma_h: 0.8
```

Use a KD-tree or equivalent efficient nearest-neighbor structure if possible.

If no map/cloud is available yet, write NaN for obstacle distance and AL_H_pred, and add a warning to the summary.

### 8.2 Vertical AL

```text
clearance_lower = p.z - z_min
clearance_upper = z_max - p.z
AL_V_pred = gamma_V * max(min(clearance_lower, clearance_upper), 0)
```

Default parameters:

```yaml
phase2_z_min: 0.5
phase2_z_max: 5.0
phase2_gamma_v: 0.8
```

### 8.3 Scalar AL

```text
AL_pred = min(AL_H_pred, AL_V_pred)
```

---

## 9. PL Model

First implementation must support:

```text
phase2_pl_model := constant_current
```

### 9.1 Constant Current Model

If current HPL/VPL are available online:

```text
PL_H_pred = current_HPL
PL_V_pred = current_VPL
PL_pred = max(PL_H_pred, PL_V_pred)
```

If current HPL/VPL are not available online:

```text
PL_H_pred = NaN
PL_V_pred = NaN
PL_pred   = NaN
```

In that case, the offline analyzer must fill current/actual PL from `export/iap_araim.csv` for matched samples.

Do not implement full future solution-separation ARAIM prediction.

---

## 10. IM Model

If PL is finite:

```text
IM_H_pred = AL_H_pred - PL_H_pred
IM_V_pred = AL_V_pred - PL_V_pred
IM_pred_axis_min = min(IM_H_pred, IM_V_pred)

IM_pred_scalar = AL_pred - PL_pred

IM_pred = min(IM_pred_axis_min, IM_pred_scalar)
```

If PL is NaN, IM fields may be NaN online and should be marked as `UNKNOWN_PL`.

Risk state:

```text
SAFE_PRED     if IM_pred > phase2_safe_margin
MARGINAL_PRED if abs(IM_pred) <= phase2_safe_margin
UNSAFE_PRED   if IM_pred < -phase2_safe_margin
UNKNOWN_PL    if PL is NaN
UNKNOWN_AL    if AL is NaN
```

---

## 11. Online CSV Export

Create:

```text
export/integrity_along_planner_traj.csv
```

Columns:

```csv
stamp,
traj_id,
sample_index,
sample_t_from_now,
sample_abs_time,
x,y,z,
vx,vy,vz,
ax,ay,az,
yaw,
dist_to_obstacle,
dist_to_vertical_lower,
dist_to_vertical_upper,
AL_H_pred,
AL_V_pred,
AL_pred,
current_HPL,
current_VPL,
current_PL,
PL_H_pred,
PL_V_pred,
PL_pred,
IM_H_pred,
IM_V_pred,
IM_pred_axis_min,
IM_pred_scalar,
IM_pred,
risk_state_pred,
pl_model,
al_model,
odom_source,
map_source
```

### 11.1 Required Finite Online Columns

These columns must be finite for valid online samples:

```text
stamp
traj_id
sample_index
sample_abs_time
x
y
z
AL_V_pred
```

These should be finite if map/cloud is available:

```text
dist_to_obstacle
AL_H_pred
AL_pred
```

PL/IM columns may be NaN online only if no ARAIM state is available online.

After offline analysis with `iap_araim.csv`, matched PL and IM fields must be finite.

---

## 12. Optional Marker Output

Prefer not to add custom message types in the first implementation.

If visualization is implemented, use:

```text
topic: /iap/planner_integrity_markers
type: visualization_msgs/msg/MarkerArray
```

Marker semantics:

```text
SAFE_PRED     -> normal sample marker
MARGINAL_PRED -> larger or highlighted marker
UNSAFE_PRED   -> largest/high-risk marker
UNKNOWN_*     -> neutral marker
```

Do not rely on marker output for validation.

---

## 13. Offline Analyzer

Add:

```text
tools/phase2/analyze_phase2_integrity_eval.py
```

Usage:

```bash
python3 tools/phase2/analyze_phase2_integrity_eval.py --run-dir <run_dir>
```

It must read:

```text
export/integrity_along_planner_traj.csv
export/desired_vs_truth.csv
export/iap_araim.csv
export/iap_sim_truth_vs_est.csv if available
```

It must output:

```text
export/phase2_integrity_eval_aligned.csv
export/phase2_summary.json
```

### 13.1 Alignment Rule

For each predicted sample:

```text
sample_abs_time
```

find nearest rows within:

```text
match_tolerance_s = 0.10
```

from:

```text
- ARAIM CSV
- desired_vs_truth CSV
- truth/IAP estimation CSV if available
```

For each aligned row, compute:

```text
actual_PL = max(actual_HPL, actual_VPL)
actual_IM if actual AL exists; otherwise NaN
pred_actual_PL_error
pred_actual_IM_error if possible
spatial_tracking_error
estimation_error
time_alignment_error_s
same_safe_unsafe_label if possible
```

---

## 14. Aligned CSV Export

Create:

```text
export/phase2_integrity_eval_aligned.csv
```

Columns:

```csv
traj_id,
sample_index,
sample_abs_time,
pred_x,pred_y,pred_z,
executed_truth_x,executed_truth_y,executed_truth_z,
executed_iap_x,executed_iap_y,executed_iap_z,
pred_AL,
pred_PL,
pred_IM,
actual_HPL,
actual_VPL,
actual_PL,
actual_AL,
actual_IM,
time_alignment_error_s,
spatial_tracking_error,
estimation_error,
pred_actual_PL_error,
pred_actual_IM_error,
same_safe_unsafe_label
```

If actual_AL or actual_IM cannot be computed, keep those fields as NaN and add a warning to `phase2_summary.json`.

---

## 15. Phase 2 Summary JSON

Create or update:

```text
export/phase2_summary.json
```

Required schema:

```json
{
  "available": true,
  "run_dir": "",
  "traj_count": 0,
  "sample_count": 0,
  "aligned_sample_count": 0,
  "online_truth_used": false,
  "odom_source": "/drone_0_visual_slam/odom",
  "map_source": "",
  "pl_model": "",
  "al_model": "",
  "sampling": {
    "horizon_s": 0.0,
    "dt_s": 0.0,
    "max_samples_per_traj": 0
  },
  "predicted_integrity": {
    "safe_count": 0,
    "marginal_count": 0,
    "unsafe_count": 0,
    "unknown_count": 0,
    "min_IM": null,
    "mean_IM": null,
    "p05_IM": null,
    "p50_IM": null,
    "p95_PL": null,
    "max_PL": null
  },
  "actual_alignment": {
    "matched_count": 0,
    "match_ratio": 0.0,
    "mean_time_alignment_error_s": null,
    "mean_spatial_tracking_error": null,
    "mean_estimation_error": null,
    "mean_pred_actual_PL_error": null,
    "mean_pred_actual_IM_error": null,
    "safe_unsafe_label_agreement_ratio": null
  },
  "warnings": [],
  "errors": []
}
```

---

## 16. Validation Script

Add:

```text
tools/phase2/validate_phase2_integrity_eval.py
```

Usage:

```bash
python3 tools/phase2/validate_phase2_integrity_eval.py --run-dir <run_dir>
```

The validator must fail if:

```text
- export/integrity_along_planner_traj.csv is missing
- export/phase2_summary.json is missing
- sample_count <= 0
- online_truth_used is true
- official odom_source is not /drone_0_visual_slam/odom
- AL_pred contains only NaN
- no finite IM exists after offline analysis when iap_araim.csv is available
- NaN/inf appears in required finite columns
- Phase 1 required logs are missing
- demo10 does not pass Phase 1 validation
```

The validator may warn, not fail, if:

```text
- predicted IM is mostly unsafe
- predicted and actual IM do not match well
- current PL is very conservative
- tracking error is high
- actual_AL or actual_IM cannot be computed
```

---

## 17. Build Requirement

Update the existing build script or add a new Phase 2 build script.

Acceptable options:

```text
Option A:
Update tools/build_phase1_ego_planner_closed_loop.sh so it also builds phase2_planner_integrity_evaluator.

Option B:
Add tools/build_phase2_pi_lite_eval.sh.
```

Important:

```text
The build must fail if demo10 references an executable that is not built.
Do not silently skip the package/executable needed by demo10.
```

Suggested build command:

```bash
bash tools/build_phase1_ego_planner_closed_loop.sh
```

or if a new script is added:

```bash
bash tools/build_phase2_pi_lite_eval.sh
```

---

## 18. Documentation

Add:

```text
docs/phase2_pi_lite_integrity_evaluator/design.md
docs/phase2_pi_lite_integrity_evaluator/validation.md
```

Update README with:

```text
- demo10 description
- official demo10 command
- Phase 2 analyzer command
- Phase 2 validation command
- explanation that demo10 is read-only evaluation, not integrity-aware planning
```

---

## 19. Suggested Implementation Order

Follow this order:

```text
1. Inspect current demo9 launch, Phase 1 logger, export directory conventions, and ego planner bspline message type.
2. Produce a concise implementation plan with exact files to modify.
3. Add design docs.
4. Add phase2_planner_integrity_evaluator node.
5. Add demo10 launch based on demo9.
6. Add offline analyzer.
7. Add validation script.
8. Update build script.
9. Update README/docs.
10. Build.
11. Run demo10.
12. Run Phase 1 validation.
13. Run Phase 2 analyzer.
14. Run Phase 2 validation.
15. Summarize changed files, commands, generated outputs, validation results, and limitations.
```

---

## 20. Suggested Commit Boundaries

Use these commit boundaries if possible:

```text
Commit 1: Add Phase 2 design and CSV schema docs.
Commit 2: Add online phase2_planner_integrity_evaluator node.
Commit 3: Add demo10 launch based on demo9.
Commit 4: Add offline analyzer.
Commit 5: Add Phase 2 validation script.
Commit 6: Update build script and README.
```

---

## 21. Official Test Commands

Build:

```bash
bash tools/build_phase1_ego_planner_closed_loop.sh
```

or:

```bash
bash tools/build_phase2_pi_lite_eval.sh
```

Run demo10:

```bash
ros2 launch iap demo10_ego_planner_pi_lite_eval.launch.py \
  start_rviz:=false \
  run_duration_s:=60 \
  allow_truth_alignment:=false \
  use_so3_dynamics:=true \
  use_gnss:=true \
  use_araim:=true \
  phase2_pl_model:=constant_current \
  phase2_al_model:=cloud_clearance
```

Run validation and analysis:

```bash
python3 tools/phase1/validate_phase1_closed_loop.py --run-dir <latest_run_dir>

python3 tools/phase2/analyze_phase2_integrity_eval.py --run-dir <latest_run_dir>

python3 tools/phase2/validate_phase2_integrity_eval.py --run-dir <latest_run_dir>
```

---

## 22. Acceptance Criteria

The work is complete only if all of the following are true:

```text
- demo9 still runs exactly as before.
- demo10 runs the same closed-loop stack as demo9 plus the Phase 2 evaluator.
- demo10 still passes Phase 1 validation.
- demo10 generates export/integrity_along_planner_traj.csv.
- Offline analyzer generates export/phase2_integrity_eval_aligned.csv.
- Offline analyzer generates export/phase2_summary.json.
- Phase 2 validation returns 0.
- Online evaluator does not subscribe to truth odom.
- No planner cost was modified.
- No ARAIM math was modified.
- No IAP estimator / FGO code was modified.
- No SO3 controller or simulator dynamics were modified.
- The final report clearly states the limitations of the constant_current PL model.
```

---

## 23. Final Report Requirement

At the end, report:

```text
1. Files changed.
2. New executables added.
3. New launch file added.
4. Build command used and result.
5. Demo10 command used and result.
6. Generated files.
7. Phase 1 validation result.
8. Phase 2 analyzer result.
9. Phase 2 validation result.
10. Confirmation that demo9 behavior was not changed.
11. Confirmation that online evaluator did not use truth odom.
12. Confirmation that planner cost / ARAIM / IAP estimator / SO3 dynamics were not modified.
13. Known limitations, especially:
    - constant_current PL is conservative and not a real future ARAIM predictor
    - cloud_clearance AL depends on map quality
    - actual_IM may be unavailable if actual_AL is not exported
    - prediction accuracy is not the Phase 2 acceptance target
```

Remember: Phase 2 is an evaluator only. It prepares the data foundation for future Phase 3 candidate selection and Phase 4 integrity-aware planner cost.