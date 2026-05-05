# Phase 2 PI-lite Integrity Evaluator Design

Phase 2 adds `demo10_ego_planner_pi_lite_eval.launch.py`, which runs the existing demo9 closed-loop stack plus a read-only evaluator. It does not change EGO planner cost functions, path search, ARAIM math, IAP estimation, controller behavior, or simulator dynamics.

## Online Evaluator

The evaluator executable is `iap_phase1_tools phase2_planner_integrity_evaluator`.

Inputs:

- `/drone_0_visual_slam/odom` (`nav_msgs/msg/Odometry`) for the online IAP time base.
- `/drone_0_planning/bspline` (`traj_utils/msg/Bspline`) as the official future trajectory source.
- `/drone_0_planning/pos_cmd` (`quadrotor_msgs/msg/PositionCommand`) only as a degraded fallback.
- The same planner cloud source selected by demo10 (`/sim/drone_0/lidar` or `/map_generator/global_cloud`).
- `/iap/integrity` (`iap/msg/IntegrityReport`) for current HPL/VPL when available.

The node never subscribes to `/sim/drone_0/truth_odom` and never publishes planner or controller commands.

## Prediction Model

For each received B-spline, the evaluator samples future points using the same De Boor convention as EGO's `UniformBspline`. The `stamp` column is the planner/wall-clock evaluation time; `sample_abs_time` is anchored to the latest IAP odom stamp so ARAIM and estimation logs can be aligned offline.

AL is computed from point-cloud clearance and vertical bounds:

- `AL_H_pred = gamma_h * max(nearest_obstacle_distance - drone_radius - safety_buffer, 0)`
- `AL_V_pred = gamma_v * max(min(z - z_min, z_max - z), 0)`
- `AL_pred = min(AL_H_pred, AL_V_pred)`

PL uses `constant_current`:

- `PL_H_pred = current_HPL`
- `PL_V_pred = current_VPL`
- `PL_pred = max(PL_H_pred, PL_V_pred)`

IM is the conservative minimum of axis and scalar margins. If AL or PL is unavailable, the row is marked `UNKNOWN_AL` or `UNKNOWN_PL`.

## Outputs

Online outputs are written under the active IAP run directory:

- `export/integrity_along_planner_traj.csv`
- `export/phase2_summary.json`
- optional `/iap/planner_integrity_markers`

Offline analysis writes:

- `export/phase2_integrity_eval_aligned.csv`
- updated `export/phase2_summary.json`
