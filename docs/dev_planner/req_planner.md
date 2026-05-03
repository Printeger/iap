# Phase 1: IAP + ego-planner-swarm Ordinary Closed Loop

You are working in `/home/dev/ws_iap` on the Printeger/iap repository, branch `dev/iap`.

The goal of Phase 1 is to connect an ordinary ego-planner closed-loop baseline into IAP. This phase is only about getting the planning/control/simulation loop running with IAP estimated odometry. It is not integrity-aware planning yet.

## 1. High-Level Goal

Implement this closed loop:

```text
IAP estimated odom
  -> ego-planner-swarm ordinary planner
  -> ego_planner traj_server
  -> quadrotor_msgs/msg/PositionCommand
  -> SO3 controller
  -> SO3 quadrotor simulator
  -> simulated LiDAR / IMU / GNSS
  -> IAP + ARAIM
  -> closed-loop logs and validation metrics
```

## 2. Non-Goals And Constraints

- Do not implement integrity-aware planning in Phase 1.
- Do not add PL / AL / IM to ego planner cost.
- Do not change ARAIM math.
- Do not rewrite IAP odometry.
- Do not replace ego planner's optimizer.
- Do not introduce BLOM or MINCO changes.
- Do not refactor unrelated modules.
- Use launch remaps, config, or small bridge/logger nodes before changing core planner or core IAP logic.
- Keep all existing demos working.

## 3. Integration Architecture Decision

Keep ego-planner-swarm as independent ROS2 packages under:

```text
src/iap/sim/ego_planner_swarm_ws/src
```

Do not merge ego planner packages into the main `iap` CMake target.

The integration should be "organic" at the ROS graph and tooling layer:

- IAP owns localization, GNSS, ARAIM, sim odom export, run logs, and validation tools.
- `ego_planner`, `plan_env`, `bspline_opt`, `traj_utils`, `quadrotor_msgs`, `so3_control`, and `so3_quadrotor_simulator` remain separate packages.
- Phase 1 launch files define the closed-loop topic contract.
- If new ROS nodes need `quadrotor_msgs` or `traj_utils`, prefer a small package in the sim workspace, for example:

```text
src/iap/sim/ego_planner_swarm_ws/src/iap_phase1_tools
```

This avoids making the main `iap` package depend on planner/simulator message packages.

## 4. Existing Components And Current Contract

Known IAP simulation topics/components:

- `/sim/drone_0/truth_odom`: simulator truth odometry.
- `/sim/drone_0/imu`: SO3 simulator raw IMU.
- `/sim/drone_0/imu_iap`: IMU variant intended for IAP, used by newer demos.
- `/sim/drone_0/lidar`: local_sensing simulated point cloud in map/world context.
- `/sim/drone_0/lidar_body`: body/lidar-frame point cloud for IAP.
- `/drone_0_visual_slam/odom`: IAP `libsim_extension.so` estimated odom export, used by planner and controller feedback.
- `/map_generator/global_cloud`: random forest global map.
- `/ublox_driver/*`: GNSS simulator outputs consumed by IAP GNSS extension.
- `/gnss_sim/*`: GNSS simulator diagnostics and visualization.
- `/iap/integrity`: ARAIM/integrity report topic, if integrity extension is enabled.

Known ego-planner topics/components:

- `ego_planner_node` subscribes to `odom_world`, remapped by launch.
- `ego_planner_node` subscribes to `grid_map/odom`, remapped by launch.
- `ego_planner_node` subscribes to `grid_map/cloud`, normally the local_sensing cloud.
- `ego_planner_node` publishes `planning/bspline`, remapped to `/drone_0_planning/bspline`.
- `traj_server` subscribes to `planning/bspline` and publishes `quadrotor_msgs/msg/PositionCommand`.
- Existing `traj_server` already samples B-spline trajectories into `PositionCommand`; do not add a replacement sampler unless this path is unavailable.
- `SO3ControlComponent` subscribes to `position_cmd` and `odom`, and publishes `SO3Command`.
- `so3_quadrotor_simulator` subscribes to `SO3Command`, and publishes truth odom plus IMU.

Important current pitfall:

- Existing EGO `simulator.launch.py` remaps SO3 controller feedback `odom` to the plant truth odom by default.
- Phase 1 must override this. Planner and SO3 controller feedback must use `/drone_0_visual_slam/odom`, not `/sim/drone_0/truth_odom`.

## 5. Phase 1 Required Behavior

- ego planner odom input must be `/drone_0_visual_slam/odom`.
- ego planner `grid_map/odom` must also use `/drone_0_visual_slam/odom`.
- SO3 controller feedback must use `/drone_0_visual_slam/odom`.
- Truth odom must remain only for plant state, sensor simulation, GNSS simulation, visualization, and logging.
- The default planner map/cloud input should be the local_sensing simulated point cloud `/sim/drone_0/lidar`, generated from `/map_generator/global_cloud`.
- Directly feeding `/map_generator/global_cloud` into ego planner is allowed only as an explicit debug mode.
- ego planner output must drive the existing `traj_server -> SO3ControlComponent -> so3_quadrotor_simulator` command chain.
- The simulator must move according to planner-generated commands.
- IAP must continue receiving simulated LiDAR, IMU, and GNSS inputs.
- IAP must continue publishing `/drone_0_visual_slam/odom`.
- ARAIM does not need to be safe; it only needs to continue running and logging.
- The run must export desired/truth/IAP/planner tracking logs.

## 6. Frame And Timestamp Policy

- Use `map` as the default global frame for Phase 1.
- Use `imu` as the default IAP/planner body frame unless an existing node requires another child frame.
- Preserve message timestamps when bridging odometry.
- If frame normalization is required by ego planner, document it in the topic contract.
- The current `libsim_extension.so` can align IAP estimated odom to truth for simulation frame initialization. Phase 1 may allow this with an explicit `allow_truth_alignment:=true` or config equivalent, but the topic contract must state whether it is enabled.

## 7. Required Work Packages

### P1-0: Topic Contract

1. Inspect existing IAP launch/config and ego-planner launch/config.
2. Create `docs/phase1_ego_planner_integration/topic_contract.md`.
3. Document:
   - IAP odom, truth odom, IMU, LiDAR, map, GNSS, and ARAIM topics.
   - ego planner odom input, map/cloud input, goal input, trajectory output, and command output.
   - SO3 controller input/output topics.
   - required remaps and bridge nodes.
   - message types, frame_id, expected frequency, and timestamp semantics.
   - whether `libsim_extension.so` truth alignment is enabled.
4. Add `tools/phase1/check_topic_contract.py` that checks required topics and basic hz.

### P1-1: Build Script

1. Add `tools/build_phase1_ego_planner_closed_loop.sh`.
2. It should build IAP, GNSS packages, simulation packages, and ego planner packages under:
   - `src/iap`
   - `src/gnss_comm`
   - `src/iap/sim/ego_planner_swarm_ws/src`
3. Include at least these packages if present:
   - `gnss_comm`
   - `cmake_utils`
   - `quadrotor_msgs`
   - `pose_utils`
   - `uav_utils`
   - `map_generator`
   - `mockamap`
   - `local_sensing`
   - `so3_quadrotor_simulator`
   - `so3_control`
   - `poscmd_2_odom`
   - `odom_visualization`
   - `gnss_sim`
   - `traj_utils`
   - `bspline_opt`
   - `path_searching`
   - `plan_env`
   - `ego_planner`
   - `iap`
   - `iap_phase1_tools`, if added.
4. If package names differ, discover them with `colcon list` and update the script accordingly.
5. Make the script executable.

### P1-2: Odom Interface

1. Ensure ego planner receives `/drone_0_visual_slam/odom`, not `/sim/drone_0/truth_odom`.
2. Ensure ego planner `grid_map/odom` receives `/drone_0_visual_slam/odom`.
3. Ensure SO3 controller feedback receives `/drone_0_visual_slam/odom`.
4. Prefer launch remapping or configuration changes.
5. If necessary, add a minimal odom bridge that republishes `nav_msgs/msg/Odometry` from IAP's odom topic to the topic expected by ego planner.
6. Preserve timestamp, pose, and twist.
7. Normalize `frame_id` only if required by ego planner, and document the normalization.

### P1-3: Map/Cloud Interface

1. Default map pipeline:

```text
/map_generator/global_cloud
  -> local_sensing
  -> /sim/drone_0/lidar
  -> ego_planner grid_map/cloud
```

2. local_sensing odometry input must be truth odom, because it is part of the sensor simulator.
3. ego planner occupancy fusion odom must be IAP odom, because it is part of the autonomy stack.
4. Add `map_source:=local_sensing_cloud` as the default demo9 mode.
5. Allow `map_source:=global_cloud_direct` as an explicit debug mode only.
6. If ego planner needs a cropped local cloud, add a small cloud bridge that crops `/map_generator/global_cloud` around current IAP odom.
7. Do not implement a new mapping system.

### P1-4: Command Interface

1. Use existing `ego_planner/traj_server` as the default trajectory-to-command sampler.
2. `traj_server` must subscribe to `/drone_0_planning/bspline`.
3. `traj_server` must publish `quadrotor_msgs/msg/PositionCommand` to `/drone_0_planning/pos_cmd`.
4. Remap `/drone_0_planning/pos_cmd` to the SO3 controller `position_cmd` input.
5. Only add a new trajectory-to-`PositionCommand` sampler if `traj_server` cannot be used.
6. If a fallback sampler is added:
   - Subscribe to ego planner B-spline output.
   - Publish `quadrotor_msgs/msg/PositionCommand` at 50-100 Hz.
   - Fill position, velocity, acceleration, yaw, and yaw_dot if available.
7. The simulator truth odom must remain only for simulation sensor generation and logging.

### P1-5: New Demo Launch

1. Add `launch/demo9_ego_planner_closed_loop.launch.py`.
2. Prefer composing demo9 explicitly from the required nodes instead of including EGO `single_run_in_sim.launch.py` wholesale, because the existing EGO simulator launch uses truth odom as SO3 controller feedback.
3. demo9 should start:
   - random forest map generator
   - SO3 quadrotor simulator
   - SO3 controller
   - local_sensing LiDAR renderer
   - LiDAR body-frame bridge
   - GNSS simulator, if enabled
   - IAP rosnode with GNSS + ARAIM + sim extension
   - ego planner node
   - ego planner traj_server
   - any required Phase 1 bridge nodes
   - Phase 1 logger
   - optional RViz
4. Expose args:
   - `start_rviz:=false`
   - `use_gnss:=true`
   - `use_araim:=true`
   - `use_iap_odom_for_planner:=true`
   - `goal_x:=8.0`
   - `goal_y:=0.0`
   - `goal_z:=2.0`
   - `run_duration_s:=120`
   - `use_so3_dynamics:=true`
   - `use_dynamic_obstacles:=false`
   - `map_source:=local_sensing_cloud`
   - `allow_truth_alignment:=true`
   - `log_phase1:=true`
5. If backwards compatibility with old argument names is useful, `planner_use_dynamic` may be accepted as an alias, but the canonical Phase 1 argument should be `use_so3_dynamics`.

### P1-6: Phase 1 Logger

1. Add a node named `phase1_closed_loop_logger`.
2. Prefer placing it in `iap_phase1_tools` if it depends on `quadrotor_msgs` or `traj_utils`.
3. Subscribe to:
   - `/sim/drone_0/truth_odom`
   - `/drone_0_visual_slam/odom`
   - `/drone_0_planning/pos_cmd`
   - `/drone_0_planning/bspline`
   - `/iap/integrity`, if available
4. Write:
   - `export/desired_vs_truth.csv`
   - `export/tracking_error.csv`
   - `export/planner_traj.csv`
   - `export/planner_cmd.csv`
   - `export/topic_contract.json`
   - `export/phase1_summary.json`
5. `desired_vs_truth.csv` must include at least:
   - `stamp`
   - `desired_x,desired_y,desired_z`
   - `desired_vx,desired_vy,desired_vz`
   - `desired_ax,desired_ay,desired_az`
   - `truth_x,truth_y,truth_z`
   - `truth_vx,truth_vy,truth_vz`
   - `iap_x,iap_y,iap_z`
   - `err_truth_des_x,err_truth_des_y,err_truth_des_z`
   - `err_iap_truth_x,err_iap_truth_y,err_iap_truth_z`
   - `horizontal_tracking_error`
   - `vertical_tracking_error`
   - `position_tracking_error`
   - `estimation_position_error`
6. `phase1_summary.json` must include:
   - `run_duration_s`
   - `planner_trajectory_count`
   - `planner_command_count`
   - `truth_odom_count`
   - `iap_odom_count`
   - tracking RMSE / P95 / max
   - estimation RMSE / P95 / max
   - topic hz estimates
   - whether `iap_araim.csv` was found

### P1-7: Validation Script

1. Add `tools/phase1/validate_phase1_closed_loop.py`.
2. It should take `--run-dir`.
3. It should check required files:
   - `export/desired_vs_truth.csv`
   - `export/planner_traj.csv`
   - `export/iap_sim_truth_vs_est.csv`
   - `export/phase1_summary.json`
4. It should check `export/iap_araim.csv` when `use_gnss:=true` and `use_araim:=true`. If ARAIM was disabled by launch/config, missing ARAIM CSV should be a warning, not a failure.
5. It should fail if:
   - run duration < 30 s
   - planner_trajectory_count < 1
   - planner_command_count < 100
   - truth_odom_count < 100
   - iap_odom_count < 50
   - NaN/inf appears in desired/truth/IAP columns
   - simulator did not move
   - final distance to goal is not smaller than initial distance to goal
6. It may warn, not fail, if tracking RMSE is large.

## 8. Execution Order

1. Inspect the repository and produce a concise implementation plan listing exact files to modify.
2. Implement P1-0 and P1-1 first.
3. Build.
4. Implement bridges only if remapping/configuration is insufficient.
5. Add demo9 launch.
6. Add logger.
7. Add validation script.
8. Run a short 30 s smoke test.
9. Run validation script.
10. Summarize:
   - files changed
   - topics connected
   - command used
   - generated logs
   - validation result
   - known remaining issues

## 9. Suggested Commit Boundaries

- commit 1: topic contract and checker
- commit 2: Phase 1 build script
- commit 3: required bridge/logger package scaffolding
- commit 4: demo9 launch
- commit 5: Phase 1 logger
- commit 6: validation script and docs

## 10. Acceptance Commands

```bash
bash tools/build_phase1_ego_planner_closed_loop.sh

ros2 launch iap demo9_ego_planner_closed_loop.launch.py \
  start_rviz:=false \
  run_duration_s:=30

python3 tools/phase1/validate_phase1_closed_loop.py --run-dir <latest_run_dir>
```

## 11. Definition Of Done

- demo9 starts from one launch command.
- ego planner uses `/drone_0_visual_slam/odom`, not truth odom.
- ego planner `grid_map/odom` uses `/drone_0_visual_slam/odom`.
- SO3 controller feedback uses `/drone_0_visual_slam/odom`.
- SO3 simulator moves due to planner command.
- IAP continues receiving simulated LiDAR/IMU/GNSS.
- IAP continues publishing estimated odom.
- ARAIM export exists when GNSS and ARAIM are enabled.
- `desired_vs_truth.csv` is generated.
- `planner_traj.csv` is generated.
- `phase1_summary.json` is generated.
- `validate_phase1_closed_loop.py` returns 0 for the default 30 s smoke test.
