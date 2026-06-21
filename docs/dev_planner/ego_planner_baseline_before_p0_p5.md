# Baseline EGO Planner Checklist Before P0-P5

This checklist locks the current original EGO planner behavior before any P0-P5
safety planner work is added. It is documentation-only: do not wire P0, P1,
P2, P3, P4, or P5 behavior into runtime as part of this baseline step.

## 1. Repository State

- Work from `/home/dev/ws_iap`.
- The active IAP Git repository is `/home/dev/ws_iap/src/iap`.
- `/home/dev/ws_iap` is not itself a Git repository.
- Before editing or running baseline checks, report:

```bash
cd /home/dev/ws_iap/src/iap
git status --short
```

If the output is non-empty, preserve user changes and do not revert unrelated
files.

## 2. Current Build Command

The current planner and simulator packages are nested under
`src/iap/src/iap/planner` and `src/iap/src/uav_simulator`. Older references to
`src/iap/sim/ego_planner_swarm_ws/src` or
`tools/build_phase1_ego_planner_closed_loop.sh` are stale for this checkout.

Build from `/home/dev/ws_iap`:

```bash
colcon build \
  --base-paths src/iap/src/iap/planner src/iap/src/uav_simulator src/gnss_comm src/iap \
  --packages-select \
    gnss_comm \
    cmake_utils quadrotor_msgs pose_utils uav_utils \
    traj_utils plan_env path_searching bspline_opt ego_planner \
    map_generator local_sensing so3_quadrotor_simulator so3_control \
    poscmd_2_odom odom_visualization gnss_sim \
    iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Expected package set:

```text
gnss_comm, cmake_utils, quadrotor_msgs, pose_utils, uav_utils,
traj_utils, plan_env, path_searching, bspline_opt, ego_planner,
map_generator, local_sensing, so3_quadrotor_simulator, so3_control,
poscmd_2_odom, odom_visualization, gnss_sim, iap
```

## 3. Baseline Regression Command

Use the current integrated ARAIM/predictor flow with the original EGO planner.
Do not use `demo9_ego_planner_closed_loop.launch.py` for this baseline.

```bash
cd /home/dev/ws_iap
source install/setup.bash

ros2 launch iap test_araim.launch.py \
  experiment:=lidar_feature_rich \
  enable_araim_pl_decomp_csv:=false \
  start_rviz:=false \
  record_bag:=false \
  run_validator:=true \
  start_planner:=true \
  run_duration_s:=90 \
  validation_duration_s:=85
```

Baseline pass criteria:

- Launch exits by the `run_duration_s` shutdown without planner or controller
  crashes.
- The ARAIM validator passes for `lidar_feature_rich`.
- EGO publishes `/drone_0_planning/bspline`.
- `traj_server` publishes `/drone_0_planning/pos_cmd`.
- The disabled planner integrity parameters remain disabled at runtime:
  - `optimization/use_integrity_cost=false`
  - `optimization/use_integrity_front_search=false`
  - `optimization/use_integrity_global_search=false`
  - `manager/use_integrity_global_search=false`
  - `risk_overlay/use_for_astar=false`
  - `risk_overlay/use_for_bspline=false`

Observed baseline run on 2026-06-20:

- Command exit code: 0.
- Validator summary: `passed=true`, `failures=[]`, `message_count=368`,
  `lidar_valid_seen=true`, `fallback_valid_seen=true`,
  `required_final_source=LIDAR`.
- EGO planner reached `EXEC_TRAJ` and later returned to `WAIT_TARGET` after
  goal completion/replan handling.
- Shutdown emitted post-SIGINT exceptions in helper processes
  (`traj_server`, `poscmd_2_odom`, `pcl_render_node`, and
  `odom_visualization`). Treat these as current shutdown cleanup noise that
  must remain visible in future comparisons; new planner stages must not add
  pre-shutdown planner/controller crashes or new runtime failures.

## 4. Current EGO Planner Controls

`launch/test_araim.launch.py` directly creates `ego_planner_node` and
`traj_server`; it does not include `advanced_param.launch.py`.

Current baseline planner parameters:

- FSM: `fsm/flight_type=2`, `fsm/thresh_replan_time=1.0`,
  `fsm/thresh_no_replan_meter=1.0`, `fsm/planning_horizon=7.5`,
  `fsm/planning_horizen_time=3.0`, `fsm/emergency_time=1.0`,
  `fsm/fail_safe=true`, plus waypoint parameters.
- Manager: `manager/max_vel=2.0`, `manager/max_acc=3.0`,
  `manager/max_jerk=4.0`, `manager/control_points_distance=0.4`,
  `manager/feasibility_tolerance=0.05`,
  `manager/use_distinctive_trajs=true`.
- Optimizer: `optimization/lambda_smooth=1.0`,
  `optimization/lambda_collision=0.5`,
  `optimization/lambda_feasibility=0.1`,
  `optimization/lambda_fitness=1.0`, `optimization/dist0=0.5`,
  `optimization/swarm_clearance=0.5`.
- Grid map: occupancy resolution/map size/local range, obstacle inflation,
  depth filter, ray limits, log-odds thresholds, frame `map`.
- Trajectory server: `traj_server/time_forward=1.0`.

`advanced_param.launch.py` still exposes older integrity-related launch
arguments. Keep them off in any baseline or disabled-mode run:

```text
use_integrity_cost=false
use_integrity_front_search=false
use_integrity_global_search=false
risk_overlay_use_for_astar=false
risk_overlay_use_for_bspline=false
```

## 5. Confirmed Code Paths

- `ego_planner_node.cpp` creates `EGOReplanFSM` and spins.
- `EGOReplanFSM::init()` creates `EGOPlannerManager`, the 10 ms FSM timer, the
  50 ms safety timer, and odom/goal/swarm/B-spline publishers/subscribers.
- `EGOPlannerManager::initPlanModules()` creates `GridMap`,
  `BsplineOptimizer`, and `AStar`.
- `GridMap::initMap()` subscribes odom/cloud/depth, updates log-odds
  occupancy, and publishes inflated occupancy.
- `EGOPlannerManager::reboundReplan()` builds the initial B-spline, calls
  `BsplineOptimizer::initControlPoints()`, optionally uses distinctive
  candidates, then optimizes/refines.
- `BsplineOptimizer::combineCostRebound()` currently combines only original
  costs: smoothness, collision distance, feasibility, swarm, terminal.
- `AStar::AstarSearch()` currently skips occupied voxels and ranks by geometric
  edge length plus heuristic only.
- `traj_server.cpp` subscribes `planning/bspline`, samples
  position/velocity/acceleration/yaw, and publishes `PositionCommand`.

## 6. P0-P5 Guardrails

Before adding any planner functionality:

- All new modules and all new runtime behavior must be disabled by default.
- With all planner integrity flags disabled, original EGO behavior must remain
  unchanged.
- Implement and unit-test P0 before P5/P1/P2/P3/P4.
- P1/P2/P3/P4 must use `RiskGridSnapshot::queryCost()` only.
- P5 must use `RiskGridSnapshot::queryPredictedPL()` and must never use
  `c_pi`.
- P1 must acquire one fixed `RiskGridSnapshot` per optimize attempt and hold it
  for all L-BFGS cost evaluations.
- P2 must rank on `OptimizerCostBreakdown.original_cost`, not `total_cost`.
- P3-global must check sufficient corridor coverage before running.
- P4 must only run inside collision-segment A* fallback invoked from
  `BsplineOptimizer::initControlPoints()`.
- Every stage must emit metrics proving fallback and disabled-baseline
  equivalence.

## 7. Minimal Non-invasive Checks

Documentation/static checks:

```bash
cd /home/dev/ws_iap/src/iap
python3 -m py_compile launch/test_araim.launch.py
test -f docs/dev_planner/ego_planner_baseline_before_p0_p5.md
```

Build check:

```bash
cd /home/dev/ws_iap
colcon build \
  --base-paths src/iap/src/iap/planner src/iap/src/uav_simulator src/gnss_comm src/iap \
  --packages-select \
    gnss_comm \
    cmake_utils quadrotor_msgs pose_utils uav_utils \
    traj_utils plan_env path_searching bspline_opt ego_planner \
    map_generator local_sensing so3_quadrotor_simulator so3_control \
    poscmd_2_odom odom_visualization gnss_sim \
    iap \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Runtime baseline check:

```bash
cd /home/dev/ws_iap
source install/setup.bash
ros2 launch iap test_araim.launch.py \
  experiment:=lidar_feature_rich \
  enable_araim_pl_decomp_csv:=false \
  start_rviz:=false \
  record_bag:=false \
  run_validator:=true \
  start_planner:=true \
  run_duration_s:=90 \
  validation_duration_s:=85
```
