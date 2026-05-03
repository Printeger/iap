# Phase 1 EGO Planner Closed-Loop Topic Contract

Requirement: `IAP-RQ-081`

Phase 1 connects the ordinary `ego_planner` closed loop to IAP estimated odometry. It does not add PL/AL/IM to planner cost, change ARAIM math, or replace EGO's optimizer.

Demo9 GNSS defaults to RINEX-driven multi-constellation simulation with `GPS,BDS,GAL,GLO`. Synthetic GNSS remains available as a debug fallback, but synthetic mode is GPS-only in the current simulator.

## Frames

- Global frame: `map`.
- IAP planner odometry: `header.frame_id=map`, `child_frame_id=imu`.
- Body-frame LiDAR cloud for IAP: `/sim/drone_0/lidar_body`, `header.frame_id=lidar`.
- Message timestamps are preserved by bridge/logging nodes. `libsim_extension.so` may align the exported IAP planner odometry to truth at startup when `allow_truth_alignment:=true`.

## Topic Contract

| Role | Topic | Type | Frame | Expected Hz | Producer | Consumer |
|---|---|---|---|---:|---|---|
| Plant truth odom | `/sim/drone_0/truth_odom` | `nav_msgs/msg/Odometry` | `map -> drone_0` | 100 | `so3_quadrotor_simulator` | local_sensing, GNSS sim, LiDAR body bridge, logger |
| SO3 raw IMU | `/sim/drone_0/imu` | `sensor_msgs/msg/Imu` | `imu` | 100+ | `so3_quadrotor_simulator` | debug only |
| IAP IMU | `/sim/drone_0/imu_iap` | `sensor_msgs/msg/Imu` | `imu` | 100+ | `so3_quadrotor_simulator` | `iap_rosnode`, SO3 controller |
| Global map | `/map_generator/global_cloud` | `sensor_msgs/msg/PointCloud2` | `map` | 0.5-1 | `map_generator/random_forest` | local_sensing, GNSS sim, debug planner mode |
| Sim LiDAR cloud | `/sim/drone_0/lidar` | `sensor_msgs/msg/PointCloud2` | `map` | 15 | `local_sensing/pcl_render_node` | EGO grid map, LiDAR body bridge |
| IAP body cloud | `/sim/drone_0/lidar_body` | `sensor_msgs/msg/PointCloud2` | `lidar` | 15 | `demo4_lidar_body_bridge` | `iap_rosnode` |
| IAP planner odom | `/drone_0_visual_slam/odom` | `nav_msgs/msg/Odometry` | `map -> imu` | 5-20 | `libsim_extension.so` | EGO planner, EGO grid map, SO3 controller, logger |
| Planner trajectory | `/drone_0_planning/bspline` | `traj_utils/msg/Bspline` | message-defined | event | `ego_planner_node` | `traj_server`, logger |
| Position command | `/drone_0_planning/pos_cmd` | `quadrotor_msgs/msg/PositionCommand` | `map` | 100 | `traj_server` | SO3 controller, logger |
| Desired command odom | `/demo9/desired/odom` | `nav_msgs/msg/Odometry` | `map` | 100 | `poscmd_2_odom` | desired trajectory visualization |
| SO3 command | `/demo9/so3_cmd` | `quadrotor_msgs/msg/SO3Command` | `drone_0` | command-rate | SO3 controller | SO3 simulator |
| GNSS range | `/ublox_driver/range_meas` | `gnss_comm/msg/GnssMeasMsg` | n/a | trigger-rate | `gnss_sim_node` | `libgnss_extension.so` |
| GNSS ephemeris | `/ublox_driver/ephem` | `gnss_comm/msg/GnssEphemMsg` | n/a | slow/static | `gnss_sim_node` | `libgnss_extension.so` |
| GLONASS ephemeris | `/ublox_driver/glo_ephem` | `gnss_comm/msg/GnssGloEphemMsg` | n/a | slow/static | `gnss_sim_node` | `libgnss_extension.so` |
| IAP integrity | `/iap/integrity` | `iap/msg/IntegrityReport` | message-defined | backend-rate | `libintegrity_extension.so` | logger/visualization |
| IAP estimate path | `/demo9/drone/path` | `nav_msgs/msg/Path` | `map` | odom-rate | `odom_visualization` | RViz |
| Truth path | `/demo9/truth/path` | `nav_msgs/msg/Path` | `map` | odom-rate | `odom_visualization` | RViz |
| Desired path | `/demo9/desired/path` | `nav_msgs/msg/Path` | `map` | command-rate | `odom_visualization` | RViz |

## Required Remaps

- EGO `odom_world` -> `/drone_0_visual_slam/odom`.
- EGO `grid_map/odom` -> `/drone_0_visual_slam/odom`.
- EGO `grid_map/cloud` -> `/sim/drone_0/lidar` by default.
- EGO `planning/bspline` -> `/drone_0_planning/bspline`.
- `traj_server` `planning/bspline` -> `/drone_0_planning/bspline`.
- `traj_server` `position_cmd` and `/position_cmd` -> `/drone_0_planning/pos_cmd`.
- SO3 controller `odom` -> `/drone_0_visual_slam/odom`.
- SO3 controller `position_cmd` -> `/drone_0_planning/pos_cmd`.
- SO3 simulator `odom` -> `/sim/drone_0/truth_odom`.
- Desired-command visualization `poscmd_2_odom` `command` -> `/drone_0_planning/pos_cmd`, `odometry` -> `/demo9/desired/odom`.
- RViz trajectory visualization publishes `/demo9/drone/path`, `/demo9/truth/path`, and `/demo9/desired/path`.

Truth odometry must not be used as planner or SO3 controller feedback in the default mode. `use_iap_odom_for_planner:=false` is debug-only.

## RViz And Waypoints

- `start_rviz:=true` by default, using `config/sim_demo9/demo9_gnss.rviz`.
- The default single target is set with `goal_x`, `goal_y`, and `goal_z`; these are passed to EGO as `point0_x`, `point0_y`, and `point0_z`.
- Multi-waypoint runs use `point_num` plus `point1_x/y/z` through `point4_x/y/z`. `point0` remains the `goal_*` triple.
- GNSS satellite signal rays use `signal_ray_width_m:=0.025` and `signal_ray_alpha:=0.3`; NLOS path markers use `nlos_path_width_m:=0.04` and `nlos_path_alpha:=0.3`.

## Map Modes

- Default: `map_source:=local_sensing_cloud`, using `/map_generator/global_cloud -> local_sensing -> /sim/drone_0/lidar -> ego_planner`.
- Debug-only: `map_source:=global_cloud_direct`, wiring `/map_generator/global_cloud` directly to EGO `grid_map/cloud`.

## Logs And Validation

`phase1_closed_loop_logger` writes these files under the IAP run directory `export/`:

- `desired_vs_truth.csv`
- `tracking_error.csv`
- `planner_traj.csv`
- `planner_cmd.csv`
- `topic_contract.json`
- `phase1_summary.json`

Use:

```bash
python3 tools/phase1/check_topic_contract.py --duration 5
python3 tools/phase1/validate_phase1_closed_loop.py --run-dir /home/dev/ws_iap/src/iap/log/latest
```
