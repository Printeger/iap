# IAP Architecture Audit — As-Built System Documentation

**Audit Date:** 2026-05-20
**Scope:** `/home/dev/ws_iap/src/iap`
**Methodology:** Code-only — documentation claims verified against source. "UNKNOWN" where evidence insufficient.
**Principle:** Code is the only truth.

---

## Table of Contents

- [A. Repository Inventory](#a-repository-inventory)
- [B. Runtime Node Map](#b-runtime-node-map)
- [C. Topic and Interface Table](#c-topic-and-interface-table)
- [D. End-to-End Data Flow](#d-end-to-end-data-flow)
- [E. Call Graph](#e-call-graph)
- [F. Module Contract Summary](#f-module-contract-summary)
- [G. Parameter Audit](#g-parameter-audit)
- [H. Frame / Unit / Timestamp Audit](#h-frame--unit--timestamp-audit)
- [I. Cost Path Audit](#i-cost-path-audit)
- [J. Failure Mode and Fallback Matrix](#j-failure-mode-and-fallback-matrix)
- [K. Integration Audit Matrix](#k-integration-audit-matrix)
- [L. Test Gap Matrix](#l-test-gap-matrix)
- [M. Simulation Environment Infrastructure](#m-simulation-environment-infrastructure)

---

## A. Repository Inventory

### A.0 Complete File Tree

```
src/iap/
├── apps/                                    # 10 ROS executables
│   ├── iap_rosnode.cpp                      #   Main IAP node (glim_rosnode)
│   ├── iap_experiment.cpp                   #   Experiment runner
│   ├── iap_status.cpp                       #   Config smoke-test
│   ├── demo3_iap_bridge.cpp                 #   Topic relay bridge
│   ├── demo3_odom_mux.cpp                   #   Odometry multiplexer
│   ├── demo4_lidar_body_bridge.cpp          #   LiDAR frame transformer
│   ├── demo_takeoff_cmd_publisher.cpp       #   Smooth takeoff publisher
│   ├── phase2_planner_integrity_evaluator.cpp # Cost field bridge node
│   ├── demo11_compare_paths_visualizer.cpp  #   Path comparison RViz
│   └── demo11_corridor_map_publisher.cpp    #   Forest corridor map gen
│
├── msg/                                     # 4 ROS message definitions
│   ├── IntegrityReport.msg                  #   HPL,VPL,HAL,VAL,IM,state,fault info
│   ├── MDPState.msg                         #   MDP state snapshot
│   ├── DynamicALResult.msg                  #   Alert limit result
│   └── TrunkLandmark.msg                    #   Trunk landmark
│
├── launch/                                  # 8 launch files
│   ├── iap_rosnode.launch.py                #   Main IAP node launch
│   ├── iap_demo.launch.py                   #   Minimal smoke-check
│   ├── iap_ego_sim.launch.py                #   Full sim + optional IAP
│   ├── demo6_truth_control_bag.launch.py    #   Truth-controlled bag replay
│   ├── demo9_ego_planner_closed_loop.launch.py    # Closed-loop + integrity
│   ├── demo10_ego_planner_pi_lite_eval.launch.py  # Phase2 PI lite eval
│   ├── demo11_compare_paths.launch.py       #   Path comparison
│   └── demo11_ego_planner_integrity_corridor.launch.py  # Forest corridor
│
├── config/                                  # 20+ config files
│   ├── config.json                          #   Master config
│   ├── config_ros.json                      #   ROS topics / frames / QoS / extensions
│   ├── config_gnss.json                     #   GNSS + integrity switches
│   ├── config_sensors.json                  #   IMU / LiDAR / camera calibration
│   ├── config_preprocess.json               #   Point cloud downsampling / crop
│   ├── config_logging.json                  #   Log rotation
│   ├── config_viewer.json                   #   Viewer settings
│   ├── araim_params.yaml                    #   ARAIM + monitor + planner params
│   ├── config_odometry_{cpu,gpu,ct}.json    #   Odometry backends
│   ├── config_global_mapping_{cpu,gpu,pose_graph}.json
│   ├── config_sub_mapping_{cpu,gpu,passthrough}.json
│   ├── gnss_sim/                            #   GNSS sim scenario YAMLs
│   ├── sim_ego/                             #   Ego sim override configs
│   └── sim_demo{3,4,6,7,8,9,11}/            #   Per-demo config overrides
│
├── test/                                    # 13 test suites
│   ├── test_araim.cpp
│   ├── test_alert_limit_model.cpp
│   ├── test_future_pl_field_predictor.cpp
│   ├── test_integrity_snapshot.cpp
│   ├── test_lidar_observability_fim.cpp
│   ├── test_local_occupancy.cpp
│   ├── test_odom_freshness.cpp
│   ├── test_pi_cost_adapter.cpp
│   ├── test_pl_grid.cpp
│   ├── test_predicted_araim.cpp
│   ├── test_run_log_manager.cpp
│   ├── test_unified_risk_grid.cpp
│   └── test_phase2_summary_schema.py
│
├── tools/                                   # 17 analysis / validation scripts
│   ├── ana_log.py
│   ├── plot_araim_timeline.py
│   ├── plot_trajectory_comparison.py
│   ├── plot_gnss_factor_debug.py
│   ├── plot_icp_timing.py
│   ├── bag_inspect_convert.py
│   ├── doc_guard.py
│   ├── gen_methodology.py
│   ├── record_demo6_phase_z.py
│   ├── phase1/check_topic_contract.py
│   ├── phase1/validate_phase1_closed_loop.py
│   ├── phase2/analyze_phase2_integrity_eval.py
│   ├── phase2/phase2_summary_schema.py
│   ├── phase2/run_phase2_h_lite_scenarios.py
│   ├── phase2/validate_phase2_integrity_eval.py
│   └── stage_v2/run_v2_acceptance.py
│
├── src/iap/                                 # 88 source files in 12 modules
│   ├── common/                              #   IMU / cloud utilities
│   │   ├── cloud_covariance_estimation.cpp
│   │   ├── cloud_deskewing.cpp
│   │   ├── imu_integration.cpp
│   │   └── imu_validation.cpp
│   │
│   ├── gnss/                                #   GNSS processing & factors
│   │   ├── gnss_extension.cpp               #     ROS2 extension module
│   │   ├── gnss_handler.cpp                 #     Measurement handler
│   │   ├── pseudorange_factor.cpp           #     GTSAM pseudorange factor
│   │   ├── doppler_factor.cpp               #     GTSAM Doppler factor
│   │   ├── clock_between_factor.cpp         #     Clock drift factor
│   │   └── visibility_predictor.cpp         #     SV visibility prediction
│   │
│   ├── integrity/                           #   Integrity monitoring (ARAIM)
│   │   ├── integrity_extension.cpp          #     ROS2 extension, publishes /iap/integrity
│   │   ├── integrity_monitor.cpp            #     PL / AL / IM / state machine
│   │   ├── araim.cpp                        #     Core ARAIM engine
│   │   ├── lidar_araim.cpp                  #     LiDAR VGICP-block ARAIM
│   │   └── fgo_information_manager.cpp      #     Σ_p extraction from iSAM2
│   │
│   ├── odometry/                            #   LiDAR-inertial odometry
│   │   ├── odometry_estimation_base.cpp
│   │   ├── odometry_estimation_{cpu,gpu,ct}.cpp
│   │   ├── odometry_estimation_{cpu,gpu,ct}_create.cpp
│   │   ├── odometry_estimation_imu.cpp
│   │   ├── async_odometry_estimation.cpp
│   │   ├── estimation_frame.cpp
│   │   ├── initial_state_estimation.cpp
│   │   ├── loose_initial_state_estimation.cpp
│   │   └── callbacks.cpp
│   │
│   ├── mapping/                             #   Global & sub-mapping
│   │   ├── global_mapping.cpp / _base.cpp / _create.cpp
│   │   ├── global_mapping_pose_graph.cpp / _create.cpp
│   │   ├── async_global_mapping.cpp
│   │   ├── sub_mapping.cpp / _base.cpp / _create.cpp
│   │   ├── sub_mapping_passthrough.cpp / _create.cpp
│   │   ├── async_sub_mapping.cpp
│   │   ├── sub_map.cpp
│   │   └── callbacks.cpp
│   │
│   ├── planner/                             #   Integrity planner & prediction
│   │   ├── integrity_planner.cpp            #     Receding-horizon planner
│   │   ├── future_pl_field_predictor.cpp    #     Advisory PL grid / direct query
│   │   ├── predicted_araim.cpp              #     GNSS geometry-only advisory
│   │   ├── predicted_integrity.cpp          #     Covariance propagation
│   │   ├── lidar_observability_fim.cpp      #     LiDAR FIM advisory
│   │   ├── unified_risk_grid.cpp            #     3D risk voxel grid
│   │   ├── pl_grid.cpp                      #     Advisory PL grid
│   │   ├── pi_cost_adapter.cpp              #     PI cost function
│   │   ├── trajectory_generator.cpp         #     Motion primitives
│   │   ├── integrity_snapshot.cpp           #     Monitor -> planner data bridge
│   │   └── future_pl_query_result.cpp       #     Unified query output
│   │
│   ├── preprocess/                          #   Point cloud preprocessing
│   │   ├── cloud_preprocessor.cpp
│   │   └── callbacks.cpp
│   │
│   ├── trunk/                               #   Trunk detection & mapping
│   │   ├── trunk_extension.cpp
│   │   ├── trunk_detector.cpp
│   │   ├── trunk_map.cpp
│   │   └── trunk_factor.cpp
│   │
│   ├── map/                                 #   Map representation
│   │   └── local_occupancy.cpp
│   │
│   ├── sim/                                 #   Simulation extensions
│   │   ├── sim_extension.cpp
│   │   └── demo8_truth_araim_extension.cpp
│   │
│   ├── util/                                #   Utilities (config, log, viz, serialization)
│   │   ├── config.cpp
│   │   ├── extension_module.cpp
│   │   ├── load_module.cpp
│   │   ├── logging.cpp
│   │   ├── run_log_manager.cpp
│   │   ├── rviz_viewer.cpp
│   │   ├── serialization.cpp
│   │   ├── data_validator.cpp
│   │   ├── debug.cpp
│   │   ├── time_keeper.cpp
│   │   ├── trajectory_manager.cpp
│   │   ├── key_lifecycle_monitor.cpp
│   │   ├── export_factors.cpp
│   │   └── relinearization_policy.cpp
│   │
│   └── viewer/                              #   Interactive / offline / standard viewers
│       ├── standard_viewer.cpp / _callbacks.cpp / _mem.cpp / _ui.cpp
│       ├── interactive_viewer.cpp
│       ├── offline_viewer.cpp
│       ├── map_editor.cpp
│       ├── memory_monitor.cpp
│       ├── editor/map_cell.cpp
│       ├── editor/points_selector.cpp
│       ├── interactive/bundle_adjustment_modal.cpp
│       └── interactive/manual_loop_close_modal.cpp
│
└── include/iap/                             # 101 header files (mirrors src/iap/)
    ├── common/   (4 headers)
    ├── gnss/     (8 headers: factors, types, canopy model, visibility)
    ├── integrity/ (9 headers: araim, monitor, extension, types, debug)
    ├── odometry/ (10 headers)
    ├── mapping/  (10 headers)
    ├── planner/  (17 headers: interface, snapshot, grid, cost, FIM, trajectories)
    ├── preprocess/ (3 headers)
    ├── trunk/    (5 headers)
    ├── map/      (1 header)
    ├── sim/      (1 header)
    ├── util/     (24 headers: config, extensions, logging, viz, serialization...)
    └── viewer/   (10 headers)

sim/ego_planner_swarm_ws/src/
├── planner/                                 # EGO-Planner (5 packages)
│   ├── bspline_opt/                         #   B-spline trajectory optimizer
│   │   ├── include/bspline_opt/
│   │   │   ├── bspline_optimizer.h          #     (integrity cost + gradient integration)
│   │   │   ├── uniform_bspline.h
│   │   │   ├── gradient_descent_optimizer.h
│   │   │   └── lbfgs.hpp
│   │   └── src/
│   │       ├── bspline_optimizer.cpp        #     calcIntegrityCost(), gradient assembly
│   │       ├── uniform_bspline.cpp
│   │       └── gradient_descent_optimizer.cpp
│   │
│   ├── path_searching/                      #   A* path search
│   │   ├── include/path_searching/dyn_a_star.h  # (risk overlay in edge cost)
│   │   └── src/dyn_a_star.cpp               #     AstarSearchImpl with integrity_query
│   │
│   ├── plan_env/                            #   Environment representation
│   │   ├── include/plan_env/
│   │   │   ├── grid_map.h                   #     (risk overlay buffer, ingestRiskOverlayCloud)
│   │   │   ├── obj_predictor.h
│   │   │   ├── raycast.h
│   │   │   └── linear_obj_model.hpp
│   │   └── src/
│   │       ├── grid_map.cpp                 #     riskOverlayPiCost(), queryRiskInterpolated()
│   │       ├── obj_predictor.cpp
│   │       ├── obj_generator.cpp
│   │       └── raycast.cpp
│   │
│   ├── plan_manage/                         #   Planner FSM & trajectory management
│   │   ├── include/ego_planner/
│   │   │   ├── ego_replan_fsm.h
│   │   │   └── planner_manager.h            #     buildIntegrityAwareGlobalWaypoints()
│   │   ├── src/
│   │   │   ├── planner_manager.cpp          #     Integrity-aware global waypoints
│   │   │   ├── ego_replan_fsm.cpp           #     Main planning FSM
│   │   │   ├── ego_planner_node.cpp         #     ROS node entry
│   │   │   └── traj_server.cpp
│   │   └── launch/
│   │       ├── advanced_param.launch.py     #     All integrity/risk/cost params
│   │       ├── single_run_in_sim.launch.py
│   │       ├── run_in_sim.launch.py
│   │       └── swarm*.launch.py
│   │
│   └── traj_utils/                          #   Trajectory utilities
│       ├── include/traj_utils/
│       │   ├── polynomial_traj.h
│       │   ├── plan_container.hpp
│       │   └── planning_visualization.h
│       ├── src/
│       │   ├── polynomial_traj.cpp
│       │   └── planning_visualization.cpp
│       └── msg/
│           ├── Bspline.msg
│           ├── DataDisp.msg
│           └── MultiBsplines.msg
│
├── uav_simulator/                           # UAV Simulator (6 packages)
│   ├── so3_quadrotor_simulator/             #   Quadrotor dynamics
│   │   ├── include/so3_quadrotor_simulator/Quadrotor.h  # 22-state RK4 model
│   │   ├── src/quadrotor_simulator_so3.cpp  #     ROS node: odom + IMU publisher
│   │   └── launch/simulator_example.launch.py
│   │
│   ├── so3_control/                         #   SO(3) geometric controller
│   │   ├── include/so3_control/SO3Control.hpp
│   │   ├── src/
│   │   │   ├── SO3Control.cpp               #     Cascaded position-attitude control law
│   │   │   ├── so3_control_component.cpp    #     Composable ROS node
│   │   │   └── control_example.cpp
│   │   └── config/gains*.yaml
│   │
│   ├── gnss_sim/                            #   GNSS signal simulator
│   │   ├── src/gnss_sim_node.cpp            #     Pseudorange / Doppler / CN0 models
│   │   └── launch/gnss_sim.launch.py
│   │
│   ├── local_sensing/                       #   LiDAR render from map
│   │   ├── src/
│   │   │   ├── pcl_render_node.cpp          #     KD-tree FOV filtering
│   │   │   ├── pointcloud_render_node.cpp   #     PCL+ODOM sync render
│   │   │   ├── depth_render.cu / .cuh       #     GPU depth render
│   │   │   └── device_image.cuh
│   │   └── config/camera.yaml
│   │
│   ├── mockamap/                            #   Procedural map generation
│   │   ├── include/mockamap/
│   │   │   ├── maps.hpp                     #     5 map types (perlin, maze, random...)
│   │   │   └── perlinnoise.hpp
│   │   ├── src/
│   │   │   ├── maps.cpp
│   │   │   ├── perlinnoise.cpp
│   │   │   └── mockamap.cpp / ces_randommap.cpp
│   │   └── launch/maze2d|3d|perlin3d|post2d.launch.py
│   │
│   ├── map_generator/                       #   Random forest map
│   │   └── src/random_forest_sensing.cpp
│   │
│   └── Utils/                               #   Utility packages
│       ├── quadrotor_msgs/                  #     PositionCommand, SO3Command...
│       ├── odom_visualization/
│       ├── pose_utils/
│       ├── uav_utils/scripts/               #     send_odom.py, tf_assist.py...
│       └── cmake_utils/
│
└── iap_phase1_tools/                        # Python evaluation tools
    ├── iap_phase1_tools/
    │   ├── phase1_closed_loop_logger.py
    │   └── phase2_planner_integrity_evaluator.py
    └── setup.py

Total: ~200 source files across 18 packages
```

### A.1 Packages

| Package | Path | Type | Description |
|---------|------|------|-------------|
| `iap` | `src/iap/` | Core C++ | IAP — Integrity-Assured Planner: odometry, mapping, GNSS, integrity monitoring, planner |
| `iap_phase1_tools` | `sim/ego_planner_swarm_ws/src/iap_phase1_tools/` | Python | Phase 1/2 closed-loop logging and evaluation tools |
| `bspline_opt` | `sim/ego_planner_swarm_ws/src/planner/bspline_opt/` | C++ | B-spline trajectory optimizer with integrity cost integration |
| `path_searching` | `sim/ego_planner_swarm_ws/src/planner/path_searching/` | C++ | A* path search with risk overlay integration |
| `plan_env` | `sim/ego_planner_swarm_ws/src/planner/plan_env/` | C++ | Environment representation: occupancy grid, ESDF, risk overlay |
| `plan_manage` | `sim/ego_planner_swarm_ws/src/planner/plan_manage/` | C++ | Planner FSM and trajectory management |
| `traj_utils` | `sim/ego_planner_swarm_ws/src/planner/traj_utils/` | C++ | Trajectory utility types (Bspline.msg, etc.) |
| `gnss_sim` | `sim/ego_planner_swarm_ws/src/uav_simulator/gnss_sim/` | C++ | GNSS signal simulator |
| `local_sensing` | `sim/ego_planner_swarm_ws/src/uav_simulator/local_sensing/` | C++ | LiDAR/depth point cloud rendering from map |
| `so3_control` | `sim/ego_planner_swarm_ws/src/uav_simulator/so3_control/` | C++ | SO(3) geometric controller |
| `so3_quadrotor_simulator` | `sim/ego_planner_swarm_ws/src/uav_simulator/so3_quadrotor_simulator/` | C++ | Quadrotor dynamics simulator |
| `mockamap` | `sim/ego_planner_swarm_ws/src/uav_simulator/mockamap/` | C++ | Random map generation (maze, perlin, forest) |
| `map_generator` | `sim/ego_planner_swarm_ws/src/uav_simulator/map_generator/` | C++ | Random forest map generator |
| Utilities | `sim/ego_planner_swarm_ws/src/uav_simulator/Utils/` | C++ | `cmake_utils`, `odom_visualization`, `pose_utils`, `quadrotor_msgs`, `uav_utils` |

### A.2 Executables (Main IAP Package)

| Executable | Source File | ROS Node? | Description |
|------------|-------------|-----------|-------------|
| `iap_rosnode` | `apps/iap_rosnode.cpp` | Yes — `glim_rosnode` | Main IAP node: odometry, mapping, GNSS, integrity, visualization |
| `iap_status` | `apps/iap_status.cpp` | No | Config smoke-test, prints library info |
| `iap_experiment` | `apps/iap_experiment.cpp` | No | Experiment metrics runner |
| `demo3_iap_bridge` | `apps/demo3_iap_bridge.cpp` | Yes | Topic relay bridge (odom + aligned_points) |
| `demo3_odom_mux` | `apps/demo3_odom_mux.cpp` | Yes | Odometry multiplexer (truth → IAP switch) |
| `demo4_lidar_body_bridge` | `apps/demo4_lidar_body_bridge.cpp` | Yes | LiDAR map→body frame transformer |
| `demo_takeoff_cmd_publisher` | `apps/demo_takeoff_cmd_publisher.cpp` | Yes | Smooth takeoff trajectory publisher |
| `phase2_planner_integrity_evaluator` | `apps/phase2_planner_integrity_evaluator.cpp` | Yes | Phase 2 integrity evaluation & cost field bridge |
| `demo11_compare_paths_visualizer` | `apps/demo11_compare_paths_visualizer.cpp` | Yes | Path comparison RViz visualizer |
| `demo11_corridor_map_publisher` | `apps/demo11_corridor_map_publisher.cpp` | Yes | Forest corridor map generator |

### A.3 Launch Files (Main IAP)

| Launch File | Purpose | Key Nodes Launched |
|-------------|---------|-------------------|
| `launch/iap_rosnode.launch.py` | Main IAP node (bag or realtime) | `iap_rosnode` (+ optional `ros2 bag play`) |
| `launch/iap_demo.launch.py` | Minimal smoke-check | `iap_status` |
| `launch/iap_ego_sim.launch.py` | Full ego_planner simulation + optional IAP | SO3 sim, local_sensing, controller, ego_planner, iap_rosnode |
| `launch/demo6_truth_control_bag.launch.py` | Truth-controlled rosbag replay | `iap_rosnode` + delayed `ros2 bag play` |
| `launch/demo9_ego_planner_closed_loop.launch.py` | Closed-loop planner + integrity demo | Full sim stack + 100+ integrity parameters |
| `launch/demo10_ego_planner_pi_lite_eval.launch.py` | Phase 2 PI lite evaluation | Includes demo9 + `phase2_planner_integrity_evaluator` |
| `launch/demo11_ego_planner_integrity_corridor.launch.py` | Dense forest corridor | Includes demo9 + evaluator + corridor map publisher |

### A.4 Message Definitions

| File | Type | Fields Summary |
|------|------|----------------|
| `msg/IntegrityReport.msg` | `.msg` | header, integrity_state, hpl, vpl, pl_e/n/u, hal, val, im, pl_ff, k_ff_used, k_fa_used, n_sv_used, n_constellations, pdop, sigma_h, n_hypotheses, n_detected, excluded_prns[], excluded_trunk_ids[], n_trunks_observed, tdop |
| `msg/MDPState.msg` | `.msg` | header, hpl, vpl, hal, val, im, integrity_state, n_sv_used, pdop, sigma_h, n_trunks_visible, nearest_trunk_dist, tdop, altitude_agl, speed, heading, dist_to_goal |
| `msg/DynamicALResult.msg` | `.msg` | header, hal, val, al, nearest_trunk_id, nearest_trunk_dist, current_altitude, canopy_height_est, al_from_trunk |
| `msg/TrunkLandmark.msg` | `.msg` | header, trunk_id, center_x, center_y, radius, confidence, sigma_x, sigma_y, n_observations, active |
| `sim/.../traj_utils/msg/Bspline.msg` | `.msg` | B-spline trajectory representation |
| `sim/.../quadrotor_msgs/msg/PositionCommand.msg` | `.msg` | Position/velocity/acceleration/yaw command for controller |

### A.5 Configuration Files (Key)

| File | Role |
|------|------|
| `config/config.json` | Master config: selects sub-configs, logging paths |
| `config/config_ros.json` | ROS topics, frame IDs, QoS, extension module list |
| `config/config_gnss.json` | GNSS parameters + integrity module switches |
| `config/config_sensors.json` | IMU/LiDAR/camera calibration + preprocessing |
| `config/config_preprocess.json` | Point cloud preprocessing (downsample, crop) |
| `config/config_logging.json` | Log rotation and storage |
| `config/config_viewer.json` | Viewer/visualization settings |
| `config/araim_params.yaml` | ARAIM + integrity monitor + planner parameters |
| `config/config_odometry_{cpu,gpu,ct}.json` | Odometry backend configs |
| `config/config_global_mapping_{cpu,gpu,pose_graph}.json` | Global mapping configs |
| `config/config_sub_mapping_{cpu,gpu,passthrough}.json` | Sub-mapping configs |

### A.6 Test Files

| Test | Module Under Test |
|------|-------------------|
| `test/test_alert_limit_model.cpp` | AlertLimitModel (HAL/VAL) |
| `test/test_araim.cpp` | Araim, IntegrityState, TrunkMap EKF |
| `test/test_future_pl_field_predictor.cpp` | FuturePLFieldPredictor |
| `test/test_integrity_snapshot.cpp` | IntegritySnapshot |
| `test/test_lidar_observability_fim.cpp` | LidarObservabilityFim |
| `test/test_local_occupancy.cpp` | LocalOccupancyGrid |
| `test/test_odom_freshness.cpp` | Odometry freshness |
| `test/test_phase2_summary_schema.py` | Phase 2 summary schema validation |
| `test/test_pi_cost_adapter.cpp` | PICostAdapter |
| `test/test_pl_grid.cpp` | PLGrid |
| `test/test_predicted_araim.cpp` | PredictedAraimComputer |
| `test/test_run_log_manager.cpp` | RunLogManager |
| `test/test_unified_risk_grid.cpp` | UnifiedRiskGrid |

### A.7 Scripts (Key)

| Script | Purpose |
|--------|---------|
| `tools/ana_log.py` | Log analysis |
| `tools/plot_araim_timeline.py` | ARAIM timeline plotting |
| `tools/plot_trajectory_comparison.py` | Trajectory comparison plots |
| `tools/phase1/check_topic_contract.py` | Topic contract validation |
| `tools/phase2/validate_phase2_integrity_eval.py` | Phase 2 integrity evaluation validation |
| `tools/phase2/run_phase2_h_lite_scenarios.py` | Phase 2 scenario runner |

---

## B. Runtime Node Map

### B.1 `glim_rosnode` (Main IAP Node)

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `iap_rosnode` |
| **Source** | `apps/iap_rosnode.cpp` |
| **Launch File** | `launch/iap_rosnode.launch.py`, `launch/iap_ego_sim.launch.py`, `launch/demo6_truth_control_bag.launch.py` |
| **Parameters Loaded** | `config_path` (from launch), `imu_topic`, `points_topic` (from ROS params or `config_ros.json`) |
| **Extension Modules** | `libgnss_extension.so`, `libtrunk_extension.so`, `libintegrity_extension.so`, `libstandard_viewer.so`, `librviz_viewer.so` |
| **Publishers** | (via rviz_viewer extension) `~/odom` (nav_msgs/Odometry), `~/points` (PointCloud2), `~/aligned_points` (PointCloud2), `~/map` (PointCloud2), `~/pose` (PoseStamped) — 13 topics total |
| **Additional Publishers** | (via integrity extension) `/iap/integrity` (IntegrityReport), `/iap/araim_envelopes` (MarkerArray), `/iap/dynamic_al` (DynamicALResult), `/iap/trunk_landmarks` (TrunkLandmark), `/iap/mdp_state` (MDPState) |
| **Subscribers** | IMU (`imu_topic_`, sensor_msgs/Imu), PointCloud2 (`points_topic_`, sensor_msgs/PointCloud2) |
| **Timers** | `input_status_timer_` (2s), `extension_watchdog_timer_` (200ms) |
| **Callback Frequency** | IMU: sensor rate (~100-200 Hz). LiDAR: sensor rate (~10 Hz Livox). Integrity publish: ~10-30 Hz (smoother update rate). |
| **Role** | Core state estimation node. Runs LiDAR-inertial odometry (GICP/iSAM2), GNSS tight coupling, trunk mapping, integrity monitoring, and visualization. |

### B.2 `phase2_planner_integrity_evaluator`

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `phase2_planner_integrity_evaluator` |
| **Source** | `apps/phase2_planner_integrity_evaluator.cpp` |
| **Launch Files** | `launch/demo10_ego_planner_pi_lite_eval.launch.py`, `launch/demo11_ego_planner_integrity_corridor.launch.py` |
| **NOT launched by** | `launch/iap_rosnode.launch.py` |
| **Parameters** | 100+ including `odom_topic`, `bspline_topic`, `pos_cmd_topic`, `integrity_topic`, `pl_model`, `al_model`, `use_pl_grid`, `use_lidar_observability`, URG parameters, cost field publishing parameters |
| **Publishers** | `/iap/integrity_cost_field` (PointCloud2, 16 fields), `/iap/integrity_front_cost_field` (PointCloud2, 21 fields), `/iap/integrity_viz/*` (visualization topics), `/iap/planner_integrity_markers` (MarkerArray) |
| **Subscribers** | Odometry (`/drone_0_visual_slam/odom`), Bspline (`/drone_0_planning/bspline`), PositionCommand (`/drone_0_planning/pos_cmd`), LiDAR map (`/sim/drone_0/lidar`), IntegrityReport (`/iap/integrity`), GNSS measurements, ephemeris, receiver LLA, ionosphere |
| **Timers** | `open_timer_` (500ms), `summary_timer_` (2s), `pl_grid_timer_` (conditional), `integrity_front_cost_field_timer_` (conditional, default 2 Hz) |
| **Callback Frequency** | Odometry: ~50-100 Hz. Bspline: ~10-30 Hz. Integrity: ~10-30 Hz. Cost field publish: 2 Hz default. |
| **Role** | Bridge node between IAP integrity monitoring and ego_planner. Consumes IntegrityReport + odometry + GNSS → computes predicted PL/AL/risk fields → publishes PointCloud2 cost fields consumed by ego_planner's GridMap risk overlay. Also performs offline evaluation: 80+ CSV metrics exported per run. |

### B.3 `demo3_iap_bridge`

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `demo3_iap_bridge` |
| **Role** | Simple topic relay: `/glim_rosnode/odom` → `/iap_rosnode/odom` and aligned points relay. Used to decouple IAP node namespaces in demo3. |

### B.4 `demo3_odom_mux`

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `demo3_odom_mux` |
| **Role** | Switches between truth odom (bootstrap phase) and IAP odom (locked phase). Fresheness-based switching with configurable lock count and timeout. Publishes to `/demo3/control_odom`. |

### B.5 `demo4_lidar_body_bridge`

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `demo4_lidar_body_bridge` |
| **Role** | Transforms LiDAR point clouds from map frame to body frame using interpolated odometry poses. Enables body-frame LiDAR processing in demo4. |

### B.6 `demo_takeoff_cmd_publisher`

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `demo_takeoff_cmd_publisher` |
| **Role** | Publishes smooth takeoff trajectory (smoothstep5 interpolation) as PositionCommand messages. Configurable ground hold, takeoff duration, hover position. |

### B.7 `demo11_compare_paths_visualizer`

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `demo11_compare_paths_visualizer` |
| **Role** | Reads CSV from off/on integrity runs, publishes Path + PointCloud2 for RViz comparison. Publishes at 0.5 Hz using transient_local QoS. |

### B.8 `demo11_corridor_map_publisher`

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `demo11_corridor_map_publisher` |
| **Role** | Generates dense forest corridor map with configurable tree density, canopy parameters, terminal walls. Publishes trunk/canopy/wall PointCloud2 at 2 Hz. |

### B.9 `iap_status` (non-ROS)

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `iap_status` |
| **Role** | Smoke-test: validates config loading, prints library information. Does NOT create a ROS node. |

### B.10 `iap_experiment` (non-ROS)

| Property | Detail |
|----------|--------|
| **Package** | `iap` |
| **Executable** | `iap_experiment` |
| **Role** | Offline experiment runner with scenario-based integrity metrics. CLI: `--scenario=forest_01 --config_dir=config`. |

---

## C. Topic and Interface Table

### C.1 Core IAP Topics (Published by `glim_rosnode` extensions)

| Topic | Message Type | Publisher | Subscriber | frame_id | Units | Timestamp Source | Rate | Required? |
|-------|-------------|-----------|------------|----------|-------|-----------------|------|-----------|
| `/iap/integrity` | `iap::msg::IntegrityReport` | `IntegrityExtensionModule` (`libintegrity_extension.so`) | `phase2_planner_integrity_evaluator`, any external monitor | `header.frame_id` (from odom) | m, unitless | `now()` at publish time | ~10-30 Hz (smoother rate) | Required for integrity-aware planning |
| `/iap/dynamic_al` | `iap::msg::DynamicALResult` | `IntegrityExtensionModule` | Debug/monitoring | `header.frame_id` | m | `now()` | ~10-30 Hz | Optional (debug) |
| `/iap/trunk_landmarks` | `iap::msg::TrunkLandmark` | `TrunkExtensionModule` | Debug/monitoring | `header.frame_id` | m | `now()` | ~10-30 Hz | Optional (debug) |
| `/iap/mdp_state` | `iap::msg::MDPState` | `IntegrityExtensionModule` | Debug/monitoring | `header.frame_id` | m, unitless | `now()` | ~10-30 Hz | Optional (debug) |
| `/iap/araim_envelopes` | `visualization_msgs::MarkerArray` | `IntegrityExtensionModule` | RViz | `map` | m | `now()` | ~10-30 Hz | Optional (RViz) |
| `~/odom` | `nav_msgs::msg::Odometry` | `RvizViewerExtension` (`librviz_viewer.so`) | Planner, controller, external consumers | `odom` → `map` via TF | m, m/s, rad/s | Message timestamp from estimator | ~10-50 Hz | Required for planning |
| `~/aligned_points` | `sensor_msgs::msg::PointCloud2` | `RvizViewerExtension` | Debug, external | `map` | m | Message timestamp | ~10 Hz | Optional (debug) |
| `~/map` | `sensor_msgs::msg::PointCloud2` | `RvizViewerExtension` | Debug, external | `map` | m | Message timestamp | Variable | Optional (debug) |

### C.2 Cost Field Topics (Published by `phase2_planner_integrity_evaluator`)

| Topic | Message Type | Publisher | Subscriber | frame_id | Units | Timestamp Source | Rate | Required? |
|-------|-------------|-----------|------------|----------|-------|-----------------|------|-----------|
| `/iap/integrity_cost_field` | `sensor_msgs::msg::PointCloud2` (16 fields) | `phase2_planner_integrity_evaluator` | `BsplineOptimizer` (in ego_planner) | `map` | m, unitless | `now()` | Conditional (PL grid timer) | Required for B-spline integrity cost |
| `/iap/integrity_front_cost_field` | `sensor_msgs::msg::PointCloud2` (21 fields) | `phase2_planner_integrity_evaluator` | `BsplineOptimizer` → `GridMap` risk overlay, `AStar` | `map` | m, unitless | `now()` | 2 Hz default | Required for A* risk overlay + B-spline risk overlay |

**Front Cost Field Schema (21 fields):** `x, y, z` (FLOAT32), `hpl, vpl` (FLOAT32), `hpl_adv, vpl_adv` (FLOAT32), `stamp_s` (FLOAT64), `source_age_s` (FLOAT64), `flags` (FLOAT32), `hal, val` (FLOAT32), `im_h, im_v, im_min` (FLOAT32), `cost` (FLOAT32), `grad_x, grad_y, grad_z` (FLOAT32), `risk_band, risk_band_code` (FLOAT32).

**⚠️ CRITICAL FINDING:** `grad_x, grad_y, grad_z` are set to `0.0f` in the front cost field — gradients are NOT computed or published for the A* path. B-spline gradient integration uses the legacy `integrity_cost_field` topic (which includes gradients) or the risk overlay path (which computes gradients locally in GridMap).

**Backend Cost Field Schema (16 fields):** `x, y, z` (FLOAT32), `hpl, vpl` (FLOAT32), `hal, val` (FLOAT32), `im_h, im_v, im_min` (FLOAT32), `cost` (FLOAT32), `grad_x, grad_y, grad_z` (FLOAT32), `risk_band, risk_band_code` (FLOAT32).

### C.3 Simulator Topics

| Topic | Message Type | Publisher | Subscriber | frame_id | Units | Rate | Required? |
|-------|-------------|-----------|------------|----------|-------|------|-----------|
| `/sim/drone_0/truth_odom` | `nav_msgs::msg::Odometry` | `so3_quadrotor_simulator` | Controller, odom_mux, bridge nodes | `world` | m, m/s | 100 Hz | Required in sim |
| `/sim/drone_0/imu` | `sensor_msgs::msg::Imu` | `so3_quadrotor_simulator` | IAP, controller | `world` | m/s², rad/s | 100-200 Hz | Required for IAP in sim |
| `/sim/drone_0/lidar` | `sensor_msgs::msg::PointCloud2` | `local_sensing` | IAP, evaluator, bridge nodes | `world` or `map` | m | 10 Hz | Required for IAP in sim |
| `/drone_0_visual_slam/odom` | `nav_msgs::msg::Odometry` | Planner-internal | Controller, evaluator | `world` | m, m/s | ~50 Hz | Required for planning |

### C.4 Planner Internal Topics

| Topic | Message Type | Publisher | Subscriber | Rate |
|-------|-------------|-----------|------------|------|
| `/drone_0_planning/bspline` | `traj_utils::msg::Bspline` | `EGOPlannerManager` | Controller, evaluator | ~10-30 Hz |
| `/drone_0_planning/pos_cmd` | `quadrotor_msgs::msg::PositionCommand` | `EGOPlannerManager` | Controller, evaluator | ~50 Hz |
| `/grid_map/risk_overlay_debug` | `sensor_msgs::msg::PointCloud2` | `GridMap` (risk overlay debug) | RViz | 2 Hz |

### C.5 GNSS Topics (from hardware or gnss_sim)

| Topic | Message Type | Publisher | Subscriber | Rate |
|-------|-------------|-----------|------------|------|
| `/ublox_driver/range_meas` | `gnss_comm::msg::GnssMeasMsg` | GNSS HW / gnss_sim | GNSS extension, evaluator | 1-10 Hz |
| `/ublox_driver/ephem` | `gnss_comm::msg::GnssEphemMsg` | GNSS HW / gnss_sim | GNSS extension, evaluator | Episodic |
| `/ublox_driver/glo_ephem` | `gnss_comm::msg::GnssGloEphemMsg` | GNSS HW / gnss_sim | GNSS extension, evaluator | Episodic |
| `/ublox_driver/receiver_lla` | `sensor_msgs::msg::NavSatFix` | GNSS HW / gnss_sim | GNSS extension, evaluator | 1-10 Hz |
| `/ublox_driver/iono_params` | `gnss_comm::msg::GnssIonosphereParameter` | GNSS HW / gnss_sim | Evaluator | Episodic |

---

## D. End-to-End Data Flow

### D.1 Simulation Path (demo9/10/11)

```
so3_quadrotor_simulator
  ├─→ /sim/drone_0/truth_odom (100 Hz, nav_msgs/Odometry, frame: world)
  ├─→ /sim/drone_0/imu        (100-200 Hz, sensor_msgs/Imu)
  └─→ drone pose → local_sensing
                   └─→ /sim/drone_0/lidar  (10 Hz, PointCloud2)

gnss_sim
  └─→ /ublox_driver/range_meas, ephem, receiver_lla, iono_params

iap_rosnode (glim_rosnode)
  ├─ subscribes: /sim/drone_0/imu, /sim/drone_0/lidar
  ├─ subscribes: GNSS topics (via libgnss_extension.so)
  ├─ runs: LiDAR-inertial odometry (GICP + iSAM2)
  ├─ runs: GNSS tight coupling (pseudorange + Doppler factors)
  ├─ runs: Trunk mapping (via libtrunk_extension.so)
  ├─ runs: Integrity monitoring (via libintegrity_extension.so)
  │    ├─ FGOInformationManager::extract() → Σ_p
  │    ├─ IntegrityMonitor::compute() → PL, AL, IM, state
  │    └─ publish → /iap/integrity (IntegrityReport)
  └─ publish → ~/odom (via librviz_viewer.so)

phase2_planner_integrity_evaluator
  ├─ subscribes: /drone_0_visual_slam/odom
  ├─ subscribes: /drone_0_planning/bspline
  ├─ subscribes: /iap/integrity (IntegrityReport)
  ├─ subscribes: GNSS topics (for advisory prediction)
  ├─ subscribes: /sim/drone_0/lidar (for LiDAR FIM)
  ├─ computes: FuturePLFieldPredictor, PredictedAraimComputer, PICostAdapter
  ├─ computes: UnifiedRiskGrid / PLGrid
  ├─ publish → /iap/integrity_front_cost_field (2 Hz, PointCloud2, 21 fields)
  └─ publish → /iap/integrity_cost_field (conditional, PointCloud2, 16 fields)

EGOPlannerManager (in plan_manage)
  ├─ subscribes: /drone_0_visual_slam/odom
  ├─ BsplineOptimizer subscribes: /iap/integrity_cost_field → onIntegrityCostField()
  ├─ BsplineOptimizer subscribes: /iap/integrity_front_cost_field → onFrontIntegrityCostField()
  ├─ GridMap subscribes: /iap/integrity_front_cost_field → ingestRiskOverlayCloud()
  ├─ Plan: buildIntegrityAwareGlobalWaypoints() → integrity-aware A*
  ├─ Optimize: B-spline with integrity cost + gradient term
  └─ publish → /drone_0_planning/pos_cmd (PositionCommand)

SO3Controller
  ├─ subscribes: /drone_0_planning/pos_cmd
  └─ publish → motor commands → so3_quadrotor_simulator
```

### D.2 Live Hardware Path (iap_rosnode.launch.py)

```
Hardware Sensors
  ├─→ IMU (Livox built-in) → /livox/imu
  ├─→ LiDAR (Livox) → /livox/lidar
  └─→ GNSS receiver → /ublox_driver/*

iap_rosnode (glim_rosnode)
  ├─ subscribes: /livox/imu, /livox/lidar
  ├─ [same processing pipeline as sim path]
  └─ publish → ~/odom, /iap/integrity
```

**⚠️ CRITICAL GAP:** In the live pipeline (`iap_rosnode.launch.py`), there is **NO** node that publishes `/iap/integrity_cost_field` or `/iap/integrity_front_cost_field`. The `phase2_planner_integrity_evaluator` is only launched in demo9/10/11. This means the integrity cost fields that the ego_planner needs for integrity-aware planning are **not available** in the live pipeline.

### D.3 Data Flow Diagram Summary

```
┌─────────────┐    ┌──────────────┐    ┌───────────────────┐
│ GNSS / IMU  │───→│  iap_rosnode │───→│ /iap/integrity    │
│ / LiDAR     │    │  (odometry + │    │ (IntegrityReport) │
└─────────────┘    │   integrity) │    └───────┬───────────┘
                   └──────────────┘            │
                          │                    ▼
                          │    ┌──────────────────────────────────┐
                          │    │ phase2_planner_integrity_evaluator│
                          │    │ (cost field bridge)               │
                          │    └────────────┬─────────────────────┘
                          │                 │
                          │    ┌────────────▼─────────────────────┐
                          │    │ /iap/integrity_front_cost_field   │
                          │    │ /iap/integrity_cost_field         │
                          │    └────────────┬─────────────────────┘
                          │                 │
                   ┌──────▼─────────────────▼──────────┐
                   │       EGOPlannerManager             │
                   │  ┌──────────┐  ┌────────────────┐  │
                   │  │  A*      │  │ B-spline Opt   │  │
                   │  │ +risk    │  │ +integrity cost│  │
                   │  │ overlay  │  │ +gradient      │  │
                   │  └──────────┘  └───────┬────────┘  │
                   └─────────────────────────┼──────────┘
                                             │
                                    ┌────────▼────────┐
                                    │ /pos_cmd        │
                                    │ (PositionCmd)   │
                                    └────────┬────────┘
                                             │
                                    ┌────────▼────────┐
                                    │ SO3Controller   │
                                    └─────────────────┘
```

---

## E. Call Graph

### E.1 Current PL Computation Chain

```
IntegrityExtensionModule::on_smoother_update_finish(frame_id, stamp)
  │
  ├─→ FGOInformationManager::extract(smoother, frame_id, stamp)
  │     └─→ FGOPositionInfo { sigma_E, sigma_N, sigma_U, eigenvalues, factor_metadata }
  │
  ├─→ GNSS handler: get latest GnssEpoch (pseudorange, Doppler, SV positions)
  │
  ├─→ Trunk handler: get latest trunk observations
  │
  └─→ IntegrityMonitor::compute(frame, epoch, trunk, fgo_info, lidar_snapshot)
        │
        ├─→ [if GNSS available] Araim::run(epoch, n_trunk_obs)
        │     ├─→ Build design matrix G (ENU), weight matrix W
        │     ├─→ Fault hypotheses enumeration (single-sat + trunk)
        │     ├─→ Subset solutions via LDL^T decomposition
        │     ├─→ Separation statistics: σ_ss,k per hypothesis
        │     ├─→ PL formula: PL_q,k = |d_q,k| + K_fa·σ_ss,q,k + K_md·σ_q,k
        │     └─→ AraimResult { HPL, VPL, pl_e, pl_n, pl_u, detected_faults, excluded_prns }
        │
        ├─→ [if LiDAR snapshot] LidarAraim::run(blocks)
        │     ├─→ VGICP block hypotheses
        │     ├─→ Subset solutions per block
        │     └─→ LidarAraimResult { HPL, VPL, fault_detected, risk_components }
        │
        ├─→ [fallback] PL = K_pl · sqrt(λ_max(Σ_p))
        │
        ├─→ Dynamic AL computation:
        │     HAL = γ_H · min_k(||p_xy - c_k|| - r_k - r_drone)  [trunk-based]
        │     VAL = γ_V · (h_t - h_canopy - h_min)               [altitude-based]
        │     AL = min(HAL, VAL)
        │
        ├─→ Integrity Margin: IM = min(HAL - HPL, VAL - VPL)
        │
        └─→ State Machine:
              SAFE: PL < AL, no faults
              SAFE_EXCLUDED: faults detected & excluded, PL^excl < AL
              UNSAFE: PL ≥ AL

IntegrityExtensionModule::[after compute]
  └─→ publish IntegrityReport on /iap/integrity
```

### E.2 Future PL Prediction Chain (for Planner)

```
FuturePLFieldPredictor::query(pos, snapshot)
  │
  ├─→ [GNSS advisory] PredictedAraimComputer::predict_araim_pl(pos, epoch)
  │     ├─→ VisibilityPredictor::predict(pos, epoch)
  │     │     └─→ visible_sats, effective_sigmas (sky mask, canopy attenuation)
  │     ├─→ Build SatGeometry list (el, az, pr_sigma)
  │     ├─→ Araim::predict_geometry(geom)  [geometry-only, no measurement residuals]
  │     │     └─→ Advisory HPL, VPL (r=0 proxy)
  │     └─→ [fallback] params.fallback_pl (~5m) if no epoch / too few sats / singular
  │
  ├─→ [LiDAR advisory] LidarObservabilityFim::evaluate(pos)
  │     ├─→ make_lidar_fim_primitives(cloud) → surface primitives (PCA per voxel)
  │     ├─→ evaluate_advisory_fim(pos, primitives) → FIM matrix + diagnostics
  │     │     └─→ lidar_alpha [0,1], tdop_proxy, condition number
  │     └─→ [FIM fusion] Combine GNSS + LiDAR information matrices
  │
  └─→ FuturePLQueryResult { advisory_hpl, advisory_vpl, fused_hpl, fused_vpl, fallback flags }

PredictedIntegrityComputer::predict(trajectory, snapshot)
  │
  ├─→ For each waypoint along trajectory:
  │     σ_pred(t+dt) = sqrt(σ_pred(t)² + σ_grow,eff² · dt)
  │     f = 1 + β_vis·(n_vis,nom - n_vis)/n_vis,nom + γ_κ·κ̄
  │     σ_grow,eff = σ_grow · max(1, f)
  │     PL_pred(t) = K_pl · σ_pred(t)
  │
  └─→ Fill CandidateTrajectory.PL_pred and sigma_pred
```

### E.3 Risk / Cost Field Publication Chain

```
Phase2PlannerIntegrityEvaluator::publish_integrity_front_cost_field_periodic()
  │
  ├─→ build_integrity_front_cost_samples()
  │     ├─→ For each grid cell in front cost field:
  │     │     ├─→ Query FuturePLFieldPredictor → FuturePLQueryResult
  │     │     ├─→ Compute PICostAdapter::evaluate(hpl, vpl, hal, val)
  │     │     │     └─→ margin_h = HAL - HPL, margin_v = VAL - VPL
  │     │     │     └─→ hinge_cost = max(0, -margin)²
  │     │     │     └─→ pi_cost_total, risk_band_code
  │     │     └─→ FrontIntegrityCostSample { position, hpl, vpl, hal, val, im_h, im_v, im_min, cost, risk_band_code }
  │     │
  │     └─→ [if UnifiedRiskGrid enabled] URG query → voxel PI cost + gradient
  │
  └─→ publish_integrity_front_cost_field(samples)
        └─→ PointCloud2 with 21 fields → /iap/integrity_front_cost_field
```

### E.4 A* Risk Overlay Integration

```
EGOPlannerManager::planGlobalTraj(start, goal)
  │
  ├─→ [if use_integrity_global_search] buildIntegrityAwareGlobalWaypoints(anchors, inter_points)
  │     ├─→ Pin risk overlay snapshot from GridMap
  │     ├─→ For each anchor segment:
  │     │     └─→ AStar::AstarSearch(step, start, goal, use_integrity_cost=true)
  │     │           │
  │     │           └─→ AstarSearchImpl(...)
  │     │                 │
  │     │                 ├─→ [risk overlay path] GridMap::integrateRiskOnEdge(current_pos, neighbor_pos)
  │     │                 │     ├─→ Sample points along edge
  │     │                 │     ├─→ For each sample: riskOverlayPiCost(pos, hpl_adv, vpl_adv)
  │     │                 │     │     ├─→ r_h = hpl_adv / (hal + eps), r_v = vpl_adv / (val + eps)
  │     │                 │     │     ├─→ r = max(r_h, r_v)
  │     │                 │     │     ├─→ if r < 0.75: cost = 0
  │     │                 │     │     ├─→ elif r < 1.0: cost = w_soft · (r - 0.75)²
  │     │                 │     │     └─→ else: cost = w_hard·(r-1)² + c_unsafe
  │     │                 │     └─→ integrity_cost = sum(sample_costs) / num_samples
  │     │                 │
  │     │                 ├─→ [legacy path] queryFrontIntegrityCost(pos) → nearest front_integrity_sample
  │     │                 │
  │     │                 └─→ edge_cost = static_cost + λ · metric_length · integrity_cost
  │     │
  │     └─→ Downsample to max_waypoints, log statistics
  │
  └─→ [fallback] Simple linear interpolation between anchors
```

### E.5 B-Spline Optimization with Integrity Cost

```
BsplineOptimizer::optimize(control_points)
  │
  └─→ [iteration loop] calcIntegrityCost(q)
        │
        ├─→ [risk overlay path, if risk_overlay_use_for_bspline_]
        │     ├─→ For each B-spline segment [i,i+1,i+2,i+3]:
        │     │     ├─→ For each sample s along segment at u ∈ [0,1]:
        │     │     │     ├─→ B-spline basis: b0=(1-3u+3u²-u³)/6, b1=(4-6u²+3u³)/6, ...
        │     │     │     ├─→ p = b0·q_i + b1·q_{i+1} + b2·q_{i+2} + b3·q_{i+3}
        │     │     │     ├─→ GridMap::queryRiskInterpolated(p) → RiskQuery { cost, gradient }
        │     │     │     ├─→ cost = min(cost, integrity_cost_max)
        │     │     │     ├─→ gradient clamped to integrity_grad_norm_max
        │     │     │     └─→ Accumulate: cost += sample_cost · weight
        │     │     │                    gradient[:] += basis[:] · sample_gradient · weight
        │     │
        │     └─→ (risk overlay computes AL locally in GridMap from clearance model)
        │
        ├─→ [legacy path, if use_integrity_cost_]
        │     ├─→ For each control point i:
        │     │     ├─→ Find nearest integrity_sample within integrity_nearest_radius_m
        │     │     ├─→ cost += sample.cost
        │     │     └─→ gradient.col(i) += sample.gradient
        │     │
        │     └─→ (uses precomputed cost + gradient from /iap/integrity_cost_field)
        │
        └─→ [fallback] No integrity cost (both paths disabled by default)

Total B-spline gradient:
  g_total = λ_smooth·g_smoothness + λ_dist·g_distance + λ_feas·g_feasibility
          + λ_swarm·g_swarm + λ_terminal·g_terminal + λ_integrity·g_integrity
```

---

## F. Module Contract Summary

### F.1 Araim (`src/iap/integrity/araim.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Core ARAIM engine: single-fault hypothesis testing, horizontal/vertical PL computation |
| **Inputs** | `GnssEpoch` (current measurements), `SatGeometry` list (elevation, azimuth, pseudorange sigma), `n_trunk_obs` |
| **Outputs** | `AraimResult` (HPL, VPL, PL_E/N/U, detected faults, excluded PRNs, K factors, separation stats) |
| **Internal State** | Integrity budget (P_HMI_req, P_FA_req), fault priors per constellation, K thresholds |
| **Parameters** | `araim/P_HMI_req`, `araim/P_FA_req`, `araim/K_fa`, `araim/K_md`, `araim/K_ff`, `araim/p_sat_default`, `araim/p_const_*`, `araim/p_trunk_*`, `araim/min_sats`, `araim/eps_degen` |
| **Failure Modes** | Too few SVs → no result; singular geometry → eigenvalue threshold; no epoch → no result |
| **Downstream Users** | `IntegrityMonitor::compute()` |
| **Status** | ✅ **FULLY IMPLEMENTED** and actively used via integrity extension |

### F.2 IntegrityMonitor (`src/iap/integrity/integrity_monitor.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Compute PL, AL, IM, integrity state; manage state machine transitions |
| **Inputs** | GNSS frame, GNSS epoch, trunk observations, FGO sigma_p, LiDAR snapshot, obstacle distance, altitude, canopy height |
| **Outputs** | `IntegrityReport` (PL, AL, IM, state), `DynamicALResult` (HAL, VAL) |
| **Internal State** | Current integrity state (SAFE/SAFE_EXCLUDED/UNSAFE), last ARAIM result, last LiDAR ARAIM result |
| **Parameters** | `integrity_monitor/K_pl`, `integrity_monitor/gamma_H`, `integrity_monitor/gamma_V`, `integrity_monitor/r_drone`, `integrity_monitor/h_min`, `integrity_monitor/canopy_height_default`, `integrity_monitor/HAL_trunk_default`, `integrity_monitor/VAL_default`, `integrity_monitor/caution_fraction`, `integrity_monitor/recovery_count`, `integrity_monitor/chi2_1dof_thresh` |
| **Failure Modes** | No GNSS → PL falls back to K_pl·sqrt(λ_max(Σ_p)); no trunk → HAL uses default; no canopy → VAL uses default |
| **Downstream Users** | `IntegrityExtensionModule` → `/iap/integrity` topic |
| **Status** | ✅ **FULLY IMPLEMENTED** and actively used |

### F.3 IntegrityExtensionModule (`src/iap/integrity/integrity_extension.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | ROS2 bridge: register callbacks, orchestrate monitor compute, publish IntegrityReport |
| **Inputs** | Extension events: `on_new_frame()`, `on_smoother_update_finish()`; FGO state, GNSS state, trunk state |
| **Outputs** | `/iap/integrity` (IntegrityReport), `/iap/dynamic_al` (DynamicALResult), `/iap/araim_envelopes` (MarkerArray), CSV debug logs |
| **Internal State** | `IntegrityMonitor` instance, publish topic name, debug flags |
| **Parameters** | `integrity/enable`, `integrity/enable_araim`, `integrity/enable_fgo_info`, `integrity/enable_dynamic_al`, `integrity/publish_topic`, `integrity/K_pl`, `integrity/gamma_H`, `integrity/gamma_V` |
| **Failure Modes** | Extension not loaded → no integrity published; FGO not ready → sigma_p unavailable |
| **Downstream Users** | `phase2_planner_integrity_evaluator`, any external integrity consumer |
| **Status** | ✅ **FULLY IMPLEMENTED** — loaded as `libintegrity_extension.so` |

### F.4 LidarAraim (`src/iap/integrity/lidar_araim.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | ARAIM for VGICP block hypotheses: block-level fault detection and PL computation |
| **Inputs** | VGICP blocks (source/target point clouds, covariance), hypothesis priors |
| **Outputs** | `LidarAraimResult` (HPL, VPL per axis, fault_detected, subset solutions, risk components γ_* ) |
| **Internal State** | Block history, age model (LINEAR_CAPPED vs EXP_SATURATING) |
| **Parameters** | `integrity/lidar_araim_target_window_K`, `integrity/lidar_araim_age_model`, `integrity/lidar_araim_age_tau_s`, `integrity/lidar_araim_gamma_age_max`, `integrity/lidar_araim_gamma_rmse_max`, `integrity/lidar_araim_condition_ref` |
| **Failure Modes** | No blocks → no result; aged blocks → inflated PL via risk components |
| **Downstream Users** | `IntegrityMonitor::compute()` (optional, when LiDAR snapshot available) |
| **Status** | ✅ **IMPLEMENTED** and used conditionally |

### F.5 FGOInformationManager (`src/iap/integrity/fgo_information_manager.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Extract marginal position covariance Σ_p from iSAM2 smoother |
| **Inputs** | iSAM2 smoother state, frame_id, timestamp |
| **Outputs** | `FGOPositionInfo` (sigma_E, sigma_N, sigma_U, eigenvalues, factor counts per type) |
| **Internal State** | Latest extracted result, thread-safe access |
| **Parameters** | `fgo_info/count_factors`, `fgo_info/min_eigenvalue` |
| **Failure Modes** | Smoother not ready → no data; singular Σ_p → eigenvalue check fails |
| **Downstream Users** | `IntegrityMonitor::compute()` |
| **Status** | ✅ **IMPLEMENTED** and actively used |

### F.6 IntegrityPlanner (`src/iap/planner/integrity_planner.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Receding-horizon integrity-aware trajectory planner using motion primitives |
| **Inputs** | pos0, vel0, yaw0, goal, sigma0, IntegrityReport, occupancy map, GNSS epoch, AL function |
| **Outputs** | `CandidateTrajectory` with best cost, `TrajectoryPoint` execution target |
| **Internal State** | Last chosen trajectory, state transition handler |
| **Parameters** | `planner/w_integrity`, `planner/w_turn`, `planner/w_mission`, `planner/w_smooth`, `planner/w_infeasible`, `planner/dt_execute`, `planner/al_default`, `planner/use_araim_pl` |
| **Failure Modes** | No feasible candidates → empty trajectory; goal unreachable → infeasibility penalty |
| **Downstream Users** | **⚠️ NOT INTEGRATED** with ego_planner — this is a parallel planner implementation, not connected to `EGOPlannerManager` |
| **Status** | ⚠️ **CODE EXISTS BUT NOT CONNECTED** — pure C++ class, never instantiated by any ROS node |

### F.7 FuturePLFieldPredictor (`src/iap/planner/future_pl_field_predictor.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Grid-based or direct future PL prediction for advisory planning |
| **Inputs** | Occupancy map, LiDAR map points, LiDAR FIM primitives, IntegritySnapshot |
| **Outputs** | `FuturePLQueryResult` (advisory HPL/VPL, fused HPL/VPL, GNSS/LiDAR diagnostics) |
| **Internal State** | 3D advisory PL grid, generation counter, age tracking |
| **Parameters** | Grid resolution, size, freshness |
| **Failure Modes** | Stale grid → flagged; no GNSS epoch → fallback PL; no LiDAR → GNSS-only advisory |
| **Downstream Users** | `Phase2PlannerIntegrityEvaluator` (for cost field generation) |
| **Status** | ✅ **USED** by evaluator node |

### F.8 PredictedAraimComputer (`src/iap/planner/predicted_araim.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Geometry-only GNSS advisory PL proxy (r=0 ARAIM, not certified) |
| **Inputs** | Query position, GNSS epoch (for SV positions), visibility predictor |
| **Outputs** | Advisory HPL, VPL (not certified), PDOP, advisory FIM |
| **Internal State** | Fallback PL value |
| **Parameters** | `fallback_pl` (~5m) |
| **Failure Modes** | No epoch → fallback PL; too few SVs → fallback; singular geometry → fallback |
| **Downstream Users** | `FuturePLFieldPredictor`, `IntegrityPlanner` |
| **Status** | ✅ **IMPLEMENTED** and used |

### F.9 PredictedIntegrityComputer (`src/iap/planner/predicted_integrity.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Propagate position uncertainty along trajectory with visibility-dependent growth |
| **Inputs** | Candidate trajectories (waypoints, timestamps), IntegritySnapshot |
| **Outputs** | `PL_pred`, `sigma_pred` filled along each trajectory |
| **Internal State** | Growth parameters |
| **Parameters** | σ_grow (base growth rate), β_vis (visibility sensitivity), γ_κ (canopy density sensitivity) |
| **Failure Modes** | Unknown |
| **Downstream Users** | `IntegrityPlanner` |
| **Status** | ✅ **IMPLEMENTED** but only used by `IntegrityPlanner` (not ego_planner) |

### F.10 UnifiedRiskGrid (`src/iap/planner/unified_risk_grid.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | 3D voxel grid for combined risk representation (ESDF, occupancy, AL, PL, IM, PI cost) |
| **Inputs** | Point cloud (for ESDF), AL model, advisory PL, IM |
| **Outputs** | Voxel queries with interpolated risk values and gradients |
| **Internal State** | 3D voxel buffer, per-voxel flags (VALID_AL, VALID_ADVISORY_PL, STALE_PL, OCCUPIED, etc.) |
| **Parameters** | Resolution, half-extent, z-slices, fresh/stale timeouts |
| **Failure Modes** | Stale voxels → STALE_PL flag; out of range → OUT_OF_RANGE flag |
| **Downstream Users** | `Phase2PlannerIntegrityEvaluator` (URG path for cost field) |
| **Status** | ✅ **IMPLEMENTED**, conditionally used |

### F.11 PLGrid (`src/iap/planner/pl_grid.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | 3D advisory PL grid for fast spatial PL queries |
| **Inputs** | FuturePLQueryResult per cell |
| **Outputs** | Interpolated PL values and PL gradients |
| **Internal State** | Grid center, size, resolution, generation number |
| **Parameters** | `pl_grid_resolution_m`, `pl_grid_size_x/y/z_m` |
| **Failure Modes** | Out of bounds → invalid query; stale generation → cache invalidation |
| **Downstream Users** | `FuturePLFieldPredictor` |
| **Status** | ✅ **IMPLEMENTED** and used |

### F.12 TrajectoryGenerator (`src/iap/planner/trajectory_generator.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Generate candidate trajectories from discretized motion primitives |
| **Inputs** | pos0, vel0, yaw0 |
| **Outputs** | Vector of `CandidateTrajectory` (each with waypoints, timestamps, velocities) |
| **Internal State** | Time horizon, time step, speed/yaw_rate/alt_rate discretization |
| **Parameters** | Forward speeds [0.5, 1.0, 1.5] m/s, yaw rates [-0.3, 0.0, 0.3] rad/s, altitude rates [-0.2, 0.0, 0.2] m/s, horizon 3.0s, step 0.2s |
| **Failure Modes** | None — always generates candidates |
| **Downstream Users** | `IntegrityPlanner` |
| **Status** | ✅ **IMPLEMENTED** — only used by `IntegrityPlanner` (not ego_planner) |

### F.13 LidarObservabilityFim (`src/iap/planner/lidar_observability_fim.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Compute advisory LiDAR FIM from surface primitives |
| **Inputs** | Query position, LiDAR map point cloud |
| **Outputs** | Advisory FIM matrix, lidar_alpha [0,1], tdop_proxy, condition number |
| **Internal State** | FIM primitives (PCA-derived surface normals per voxel) |
| **Parameters** | `lidar_fim_radius_m` (8.0m), `lidar_sigma_m` (0.5m), voxel radius, min points |
| **Failure Modes** | No points in radius → invalid FIM; degenerate geometry → high condition number |
| **Downstream Users** | `FuturePLFieldPredictor` (FIM fusion) |
| **Status** | ✅ **IMPLEMENTED** and conditionally used |

### F.14 PICostAdapter (`src/iap/planner/pi_cost_adapter.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Compute PI cost from HAL, VAL, HPL, VPL (hinge loss or ratio-based) |
| **Inputs** | HPL, VPL, HAL, VAL (all advisory) |
| **Outputs** | PI cost total, hinge cost, ratio cost, risk band code (SAFE_PI/MARGINAL_PI/UNSAFE_PI) |
| **Internal State** | None (stateless) |
| **Parameters** | Weights, thresholds |
| **Failure Modes** | Invalid/NaN inputs → unknown penalty cost |
| **Downstream Users** | `Phase2PlannerIntegrityEvaluator`, `IntegrityPlanner` |
| **Status** | ✅ **IMPLEMENTED** and used |

### F.15 IntegritySnapshot (`src/iap/planner/integrity_snapshot.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Frozen copy of current monitor state for decoupled advisory prediction |
| **Inputs** | Monitor outputs (HPL, VPL, HAL, VAL, IM, state), GNSS epoch, LiDAR snapshot |
| **Outputs** | `IntegritySnapshot` struct |
| **Internal State** | None (value object) |
| **Parameters** | None |
| **Failure Modes** | Stale snapshot → flagged by consumer |
| **Downstream Users** | `FuturePLFieldPredictor`, `PredictedIntegrityComputer` |
| **Status** | ✅ **IMPLEMENTED** as data structure |

### F.16 FuturePLQueryResult (`src/iap/planner/future_pl_query_result.cpp`)

| Property | Detail |
|----------|--------|
| **Responsibility** | Unified output format for all advisory PL queries |
| **Inputs** | Computed by predictor modules |
| **Outputs** | Advisory HPL/VPL, fused HPL/VPL, GNSS/LiDAR diagnostics, fallback flags, grid metadata |
| **Internal State** | None (value object) |
| **Parameters** | None |
| **Failure Modes** | fallback=true when prediction unavailable |
| **Downstream Users** | All planner prediction consumers |
| **Status** | ✅ **FULLY USED** — return type for all planner PL queries |

---

## G. Parameter Audit

### G.1 Overview

The IAP system has approximately **150+ parameters** spread across YAML, JSON, and launch files. Parameters are loaded at three levels:
1. **JSON configs** (`config/*.json`) — loaded by the IAP core library (`Config` class)
2. **YAML** (`config/araim_params.yaml`) — loaded by ARAIM/integrity modules
3. **Launch file args** — declared as ROS parameters, override defaults

### G.2 Critical Enable/Disable Parameters

These parameters control whether integrity features are active. **All integrity cost paths in the planner are OFF by default.**

| Parameter | File | Default | Effect of Default |
|-----------|------|---------|-------------------|
| `integrity/enable` | `config_gnss.json` | `true` | ✅ Integrity monitoring enabled |
| `integrity/enable_araim` | `config_gnss.json` | `true` | ✅ ARAIM computation enabled |
| `integrity/enable_fgo_info` | `config_gnss.json` | `true` | ✅ FGO sigma extraction enabled |
| `integrity/enable_dynamic_al` | `config_gnss.json` | `true` | ✅ Dynamic AL computation enabled |
| `use_integrity_cost` | `advanced_param.launch.py` | **`false`** | ❌ B-spline integrity cost OFF |
| `use_integrity_front_search` | `advanced_param.launch.py` | **`false`** | ❌ Rebound A* integrity cost OFF |
| `use_integrity_global_search` | `advanced_param.launch.py` | **`false`** | ❌ Global A* integrity cost OFF |
| `risk_overlay_enable` | `advanced_param.launch.py` | `true` | ✅ Risk overlay ingestion enabled |
| `risk_overlay_use_for_astar` | `advanced_param.launch.py` | **`false`** | ❌ A* risk overlay OFF |
| `risk_overlay_use_for_bspline` | `advanced_param.launch.py` | **`false`** | ❌ B-spline risk overlay OFF |
| `publish_integrity_cost_field` | `phase2_planner_integrity_evaluator.cpp` | **`false`** | ❌ Backend cost field not published |
| `publish_integrity_front_cost_field` | `phase2_planner_integrity_evaluator.cpp` | **`false`** | ❌ Front cost field not published |
| `use_pl_grid` | `phase2_planner_integrity_evaluator.cpp` | **`false`** | ❌ PL grid not built |
| `use_lidar_observability` | `phase2_planner_integrity_evaluator.cpp` | **`false`** | ❌ LiDAR FIM not computed |
| `use_unified_risk_grid` | `phase2_planner_integrity_evaluator.cpp` | **`false`** | ❌ URG not built |

### G.3 Integrity Monitor Parameters

| Parameter | File | Default | Where Read | Where Used |
|-----------|------|---------|------------|------------|
| `integrity_monitor/K_pl` | `araim_params.yaml` | `3.0` | `IntegrityMonitor` ctor | PL fallback: PL = K_pl·sqrt(λ_max) |
| `integrity_monitor/gamma_H` | `araim_params.yaml` | `0.5` | `IntegrityMonitor` ctor | HAL = γ_H·min(clearance distances) |
| `integrity_monitor/gamma_V` | `araim_params.yaml` | `0.8` | `IntegrityMonitor` ctor | VAL = γ_V·(h - canopy - h_min) |
| `integrity_monitor/r_drone` | `araim_params.yaml` | `0.35` | `IntegrityMonitor` ctor | HAL: subtract from trunk distance |
| `integrity_monitor/h_min` | `araim_params.yaml` | `2.0` | `IntegrityMonitor` ctor | VAL: minimum clearance below canopy |
| `integrity_monitor/canopy_height_default` | `araim_params.yaml` | `5.0` | `IntegrityMonitor` ctor | VAL: fallback when canopy unknown |
| `integrity_monitor/HAL_trunk_default` | `araim_params.yaml` | `10.0` | `IntegrityMonitor` ctor | HAL: fallback when no trunks |
| `integrity_monitor/VAL_default` | `araim_params.yaml` | `20.0` | `IntegrityMonitor` ctor | VAL: fallback when altitude unknown |
| `integrity_monitor/caution_fraction` | `araim_params.yaml` | `0.8` | `IntegrityMonitor` ctor | State machine: PL/AL > 0.8 → CAUTION |
| `integrity_monitor/nominal_fraction` | `araim_params.yaml` | `0.6` | `IntegrityMonitor` ctor | State machine: PL/AL < 0.6 → NOMINAL |
| `integrity_monitor/recovery_count` | `araim_params.yaml` | `5` | `IntegrityMonitor` ctor | Consecutive SAFE frames needed for recovery |
| `integrity_monitor/chi2_1dof_thresh` | `araim_params.yaml` | `6.63` | `IntegrityMonitor` ctor | NIS gating: per-sat χ²(1) threshold |
| `integrity_monitor/gamma_R_max` | `araim_params.yaml` | `5.0` | `IntegrityMonitor` ctor | NIS gating: maximum residual inflation |

### G.4 ARAIM Parameters

| Parameter | File | Default | Where Used |
|-----------|------|---------|------------|
| `araim/P_HMI_req` | `araim_params.yaml` | `1.0e-7` | Integrity budget: P(HMI) requirement |
| `araim/P_FA_req` | `araim_params.yaml` | `1.0e-5` | Integrity budget: P(FA) requirement |
| `araim/dynamic_budget` | `araim_params.yaml` | `true` | Use dynamic K factors vs. fixed |
| `araim/K_fa` | `araim_params.yaml` | `4.50` | Fault-free PL multiplier |
| `araim/K_md` | `araim_params.yaml` | `5.50` | Missed-detection multiplier |
| `araim/K_ff` | `araim_params.yaml` | `5.42` | Fault-free K factor (full solution) |
| `araim/p_sat_default` | `araim_params.yaml` | `1.0e-5` | Per-satellite fault prior (default) |
| `araim/p_const_GPS` | `araim_params.yaml` | `1.0e-4` | Per-constellation fault prior (GPS) |
| `araim/p_const_GAL` | `araim_params.yaml` | `1.0e-4` | Per-constellation fault prior (Galileo) |
| `araim/p_const_BDS` | `araim_params.yaml` | `1.0e-4` | Per-constellation fault prior (BeiDou) |
| `araim/p_const_GLO` | `araim_params.yaml` | `1.0e-4` | Per-constellation fault prior (GLONASS) |
| `araim/p_trunk_base` | `araim_params.yaml` | `1.0e-3` | Trunk fault prior (base) |
| `araim/p_trunk_scale` | `araim_params.yaml` | `0.1` | Trunk fault prior (scale with distance) |
| `araim/min_sats` | `araim_params.yaml` | `4` | Minimum satellites for ARAIM |
| `araim/eps_degen` | `araim_params.yaml` | `1.0e-10` | Degeneracy eigenvalue threshold |

### G.5 Planner Integrity Parameters (Legacy)

| Parameter | File | Default | Where Used |
|-----------|------|---------|------------|
| `planner/w_integrity` | `araim_params.yaml` | `2.0` | `IntegrityPlanner` cost: integrity weight |
| `planner/w_turn` | `araim_params.yaml` | `0.5` | `IntegrityPlanner` cost: turn weight |
| `planner/w_mission` | `araim_params.yaml` | `1.0` | `IntegrityPlanner` cost: mission (goal distance) weight |
| `planner/w_infeasible` | `araim_params.yaml` | `100.0` | `IntegrityPlanner` cost: infeasibility penalty |
| `planner/dt_execute` | `araim_params.yaml` | `0.2` | `IntegrityPlanner`: execution segment duration [s] |
| `planner/use_araim_pl` | `araim_params.yaml` | `true` | `IntegrityPlanner`: use ARAIM PL vs. simple K·σ |

### G.6 Planner Integrity Parameters (ego_planner — advanced_param.launch.py)

| Parameter | Default | Effect |
|-----------|---------|--------|
| `use_integrity_cost` | **`false`** | Enable B-spline integrity cost term (legacy path) |
| `lambda_integrity` | `0.00001` | Weight of integrity cost in B-spline total gradient |
| `integrity_cost_topic` | `/iap/integrity_cost_field` | Topic for legacy integrity cost PointCloud2 |
| `integrity_field_stale_timeout_s` | `0.5` | Max age of integrity samples before rejection |
| `integrity_nearest_radius_m` | `1.0` | Search radius for nearest integrity sample |
| `integrity_cost_max` | `1000.0` | Clamp max cost per sample |
| `integrity_grad_norm_max` | `0.1` | Clamp max gradient norm per sample |
| `integrity_min_samples` | `3` | Minimum samples needed for cost to be valid |
| `use_integrity_front_search` | **`false`** | Enable integrity cost in rebound A* |
| `lambda_integrity_front` | `2.0` | Weight of integrity cost in A* edge cost |
| `integrity_front_cost_topic` | `/iap/integrity_front_cost_field` | Topic for front integrity cost PointCloud2 |
| `integrity_front_nearest_radius_m` | `1.5` | Search radius for nearest front sample |
| `integrity_front_stale_timeout_s` | `1.0` | Max age of front samples |
| `integrity_front_cost_max` | `10.0` | Clamp max cost per front sample |
| `use_integrity_global_search` | **`false`** | Enable integrity-aware global A* waypoints |
| `integrity_global_astar_step_m` | `0.5` | A* step size for global search |
| `integrity_global_max_waypoints` | `80` | Max waypoints after downsampling |

### G.7 Risk Overlay Parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `risk_overlay_enable` | `true` | Enable risk overlay ingestion in GridMap |
| `risk_overlay_use_for_astar` | **`false`** | Use risk overlay in A* edge cost |
| `risk_overlay_use_for_bspline` | **`false`** | Use risk overlay in B-spline gradient |
| `risk_overlay_lambda_unknown` | `10.0` | Cost when risk data is unknown |
| `risk_overlay_lambda_stale` | `1.0` | Cost multiplier for stale data |
| `risk_overlay_stale_timeout_s` | `1.0` | When data is considered stale |
| `risk_overlay_stale_tau_s` | `1.0` | Decay time constant for stale data |
| `risk_overlay_r_soft` | `0.75` | HPL/HAL ratio below which cost=0 |
| `risk_overlay_w_soft` | `1.0` | Soft cost weight (0.75 ≤ r < 1.0) |
| `risk_overlay_w_hard` | `10.0` | Hard cost weight (r ≥ 1.0) |
| `risk_overlay_c_unsafe` | `10.0` | Constant cost added when unsafe (r ≥ 1.0) |
| `risk_overlay_eps_al_m` | `0.001` | Epsilon to avoid division by zero |
| `risk_overlay_gamma_h` | `0.8` | Horizontal clearance weight for HAL |
| `risk_overlay_gamma_v` | `0.8` | Vertical clearance weight for VAL |
| `risk_overlay_drone_radius_m` | `0.35` | Drone radius for clearance |
| `risk_overlay_safety_buffer_m` | `0.20` | Additional safety margin |
| `risk_overlay_clearance_max_m` | `5.0` | Max clearance considered |
| `risk_overlay_clearance_unknown_m` | `1.0` | Clearance when occupancy unknown |
| `risk_overlay_edge_sample_alpha` | `0.75` | Edge sampling alpha for integration |
| `risk_overlay_bspline_samples_per_segment` | `3` | Samples per B-spline segment |

### G.8 Phase2 Evaluator Parameters

| Parameter | Default | Effect |
|-----------|---------|--------|
| `pl_model` | `"constant_current"` | PL prediction model |
| `al_model` | `"cloud_clearance"` | AL model |
| `use_pl_grid` | **`false`** | Build advisory PL grid |
| `pl_grid_resolution_m` | `1.0` | PL grid resolution |
| `pl_grid_size_x/y/z_m` | `30.0, 30.0, 8.0` | PL grid dimensions |
| `use_lidar_observability` | **`false`** | Compute LiDAR FIM |
| `use_lidar_advisory_fim` | **`false`** | Use LiDAR advisory FIM in fusion |
| `lidar_fim_radius_m` | `8.0` | LiDAR FIM search radius |
| `lidar_sigma_m` | `0.5` | LiDAR measurement sigma |
| `use_unified_risk_grid` | **`false`** | Build URG |
| `urg_resolution_m` | `1.0` | URG resolution |
| `urg_half_extent_x/y_m` | `25.0, 25.0` | URG extents |
| `urg_z_slices` | `1` | URG vertical slices |
| `urg_fresh_timeout_s` | `1.0` | URG freshness threshold |
| `urg_stale_timeout_s` | `5.0` | URG staleness threshold |
| `publish_integrity_cost_field` | **`false`** | Publish backend cost field |
| `publish_integrity_front_cost_field` | **`false`** | Publish front cost field |
| `integrity_front_cost_field_publish_hz` | `2.0` | Front cost field publish rate |
| `integrity_front_cost_field_resolution_m` | `1.0` | Front cost field grid resolution |
| `integrity_front_cost_field_size_x/y_m` | `50.0, 50.0` | Front cost field dimensions |
| `integrity_front_cost_field_frame_id` | `"map"` | Front cost field frame |

---

## H. Frame / Unit / Timestamp Audit

### H.1 Frame Conventions

| Frame | Origin | Used By | Notes |
|-------|--------|---------|-------|
| `map` | IAP global map origin (first frame) | IAP odometry, cost fields, map points | Set via `config_ros.json`: `glim_ros/map_frame_id` |
| `odom` | IAP odometry origin | IAP odometry messages | Set via `config_ros.json`: `glim_ros/odom_frame_id` |
| `lidar` | LiDAR sensor origin | Raw LiDAR points, body-frame clouds | Set via `config_ros.json`: `glim_ros/lidar_frame_id` (auto-detect) |
| `base_link` | IMU/body origin | IMU messages | Set via `config_ros.json`: `glim_ros/base_frame_id` (auto-detect) |
| `world` | Simulator world origin | Simulator truth odom, IMU, LiDAR | **⚠️ Does NOT match IAP's `map` frame — potential mismatch** |
| `map` (sim) | Planner/cost field frame | `integrity_front_cost_field_frame_id_` | Default: `"map"` — must match GridMap's frame_id |

### H.2 Critical Frame Mismatch Risk

| Mismatch | Risk |
|----------|------|
| Simulator publishes in `world`, IAP expects `map` | TF lookup may fail if no `world→map` transform is published |
| `phase2_planner_integrity_evaluator` publishes cost field in `map` frame | GridMap expects `mp_.frame_id_` to match — if GridMap uses `world`, ingestion is **rejected** with warning |
| `demo4_lidar_body_bridge` outputs in `lidar` frame | Consumed by IAP which auto-detects LiDAR frame — should match |
| GNSS topics from hardware | Typically in sensor-specific frames, not `map` — handled by GNSS extension's lever arm transform |

### H.3 Unit Conventions

| Quantity | Unit | Evidence |
|----------|------|----------|
| Position | meters [m] | All pose/odometry messages, cost field positions |
| Protection Levels (HPL, VPL) | meters [m] | `IntegrityReport.hpl`, `IntegrityReport.vpl` (float64, no unit annotation) |
| Alert Limits (HAL, VAL) | meters [m] | `IntegrityReport.hal`, `IntegrityReport.val` |
| Integrity Margin (IM) | meters [m] | `IntegrityReport.im` |
| Velocity | meters/second [m/s] | `Odometry.twist.twist.linear` |
| Angular velocity | radians/second [rad/s] | `Odometry.twist.twist.angular`, `Imu.angular_velocity` |
| Acceleration | meters/second² [m/s²] | `Imu.linear_acceleration` |
| Time | seconds [s] (ROS2 `builtin_interfaces/Time`), UNIX epoch (float64 `stamp_s`) | `header.stamp`, `stamp_s` field |
| Heading/yaw | radians [rad] | `IntegrityPlanner::plan(yaw0)`, `CandidateTrajectory` yaw |
| Cost | unitless (arbitrary) | PI cost, integrity cost |
| Gradient | unitless/meter [1/m] | PI gradient in cost field |
| Probability | unitless [0,1] | P_HMI_req, P_FA_req, fault priors |
| PDOP, TDOP | unitless | `IntegrityReport.pdop`, `IntegrityReport.tdop` |

### H.4 Timestamp Sources

| Data Product | Timestamp Source | Consumer Assumption |
|-------------|-----------------|---------------------|
| `IntegrityReport` | `rclcpp::Node::now()` at publish time | Consumer assumes fresh (no age check in evaluator) |
| `integrity_front_cost_field` | `rclcpp::Node::now()` at publish time, with `stamp_s` field set to same | GridMap compares stamp_s against `now()` for staleness |
| `integrity_cost_field` | Message header stamp | BsplineOptimizer compares against `integrity_field_stale_timeout_s_` |
| Odometry | Estimator timestamp (from LiDAR scan end time) | Consumer trusts estimator time |
| IMU | IMU hardware timestamp (or `imu_time_offset` corrected) | IAP applies `imu_time_offset` from config |
| LiDAR | LiDAR hardware timestamp (or `points_time_offset` corrected) | IAP applies `points_time_offset` from config |
| GNSS measurements | GNSS receiver time-of-week (via `GnssMeasMsg`) | GNSS extension applies `time_tolerance` (0.1s default) |

### H.5 Timestamp Mismatch Risks

| Risk | Detail |
|------|--------|
| IMU/LiDAR time sync | `imu_time_offset` and `points_time_offset` default to 0.0 — must be calibrated for hardware |
| GNSS time vs. ROS time | GNSS ToW may drift from system clock; `time_tolerance` of 0.1s may reject valid measurements |
| Cost field staleness | `stamp_s` is `now()` at publish — if publish is at 2 Hz but planner runs at 10+ Hz, most planner cycles see stale data |
| Simulator time | Sim publishes at wall-clock rate by default; rosbag playback may use different clock |

---

## I. Cost Path Audit

### I.1 Does Integrity/Risk Information Actually Enter the Planner Cost?

**Summary: Code paths exist but are ALL disabled by default.** When enabled, integrity information enters both A* and B-spline optimization through two mechanisms: risk overlay (preferred) and legacy cost field (deprecated).

### I.2 A* Edge Cost

#### Risk Overlay Path (Preferred)

**Enable flag:** `risk_overlay_use_for_astar = true`  
**Default:** `false`  
**Source data:** `/iap/integrity_front_cost_field` → `GridMap::ingestRiskOverlayCloud()` → risk overlay buffer

**Formula:**
$$edge\_cost = static\_cost + \lambda_{integrity\_front} \cdot metric\_length \cdot integrity\_cost$$

Where:
- $static\_cost = \sqrt{dx^2 + dy^2 + dz^2}$ (Euclidean step)
- $integrity\_cost = riskOverlayPiCost(pos, hpl\_adv, vpl\_adv)$
- $\lambda_{integrity\_front} = 2.0$ (default)

**PI Cost formula** (`GridMap::riskOverlayPiCost()`):
$$r_h = \frac{hpl\_adv}{hal + \epsilon}, \quad r_v = \frac{vpl\_adv}{val + \epsilon}, \quad r = \max(r_h, r_v)$$

$$pi\_cost = \begin{cases} 0 & r < 0.75 \\ w_{soft} \cdot (r - 0.75)^2 & 0.75 \leq r < 1.0 \\ w_{hard} \cdot (r - 1.0)^2 + c_{unsafe} & r \geq 1.0 \end{cases}$$

**Evidence:** `path_searching/src/dyn_a_star.cpp` line 305-350, `plan_env/src/grid_map.cpp` line 766+

#### Legacy Cost Field Path

**Enable flag:** `use_integrity_front_search = true`  
**Default:** `false`  
**Source data:** `/iap/integrity_front_cost_field` → `BsplineOptimizer::onFrontIntegrityCostField()` → `front_integrity_samples_`

**Formula:**
$$edge\_cost = static\_cost \cdot (1.0 + \lambda_{integrity\_front} \cdot integrity\_cost)$$

Where integrity_cost is queried from nearest `FrontIntegritySample`.

**Evidence:** `path_searching/src/dyn_a_star.cpp` (~line 330), `bspline_opt/src/bspline_optimizer.cpp` `queryFrontIntegrityCost()`

### I.3 Global A* Waypoints

**Enable flag:** `use_integrity_global_search = true` OR `risk_overlay_use_for_astar = true`  
**Default:** `false`  
**Function:** `EGOPlannerManager::buildIntegrityAwareGlobalWaypoints()` in `plan_manage/src/planner_manager.cpp` line 458  
**Invoked from:** `planGlobalTraj()` and `planGlobalTrajWaypoints()` — with **graceful fallback** to linear interpolation if integrity-aware A* fails.

**Evidence:** `plan_manage/src/planner_manager.cpp` line 560 and 628

### I.4 B-Spline Optimization Gradient

#### Risk Overlay Path (Preferred)

**Enable flag:** `risk_overlay_use_for_bspline = true`  
**Default:** `false`  
**Source data:** Risk overlay buffer in GridMap (same as A* risk overlay path)

**Formula:**
For each B-spline segment and sample point:
$$p(u) = b_0(u) q_i + b_1(u) q_{i+1} + b_2(u) q_{i+2} + b_3(u) q_{i+3}$$

$$cost \mathrel{+}= sample\_cost \cdot weight, \quad \nabla q_j \mathrel{+}= b_j(u) \cdot sample\_gradient \cdot weight$$

where $b_j$ are cubic B-spline basis functions and weight = interval / samples_per_segment.

**Total gradient:**
$$g_{total} = \lambda_{smooth} \cdot g_{smoothness} + \lambda_{dist} \cdot g_{distance} + \lambda_{feas} \cdot g_{feasibility} + \lambda_{swarm} \cdot g_{swarm} + \lambda_{terminal} \cdot g_{terminal} + \lambda_{integrity} \cdot g_{integrity}$$

**Evidence:** `bspline_opt/src/bspline_optimizer.cpp` line 1484 `calcIntegrityCost()`, line 2511 gradient assembly

#### Legacy Cost Field Path

**Enable flag:** `use_integrity_cost = true`  
**Default:** `false`  
**Source data:** `/iap/integrity_cost_field` → `BsplineOptimizer::onIntegrityCostField()` → `integrity_samples_`

**Formula:** Nearest-neighbor: for each control point, find nearest integrity sample within `integrity_nearest_radius_m` (1.0m default), add its cost and gradient.

**Evidence:** `bspline_opt/src/bspline_optimizer.cpp` `onIntegrityCostField()` callback, `calcIntegrityCost()` legacy branch

### I.5 Cost Path Summary Table

| Cost Term | Formula | Weight (Default) | Enable Flag | Source Data | Debug Output | Active by Default? |
|-----------|---------|------------------|-------------|-------------|-------------|---------------------|
| A* edge (overlay) | static + λ·len·pi_cost | λ=2.0 | `risk_overlay_use_for_astar` | `/iap/integrity_front_cost_field` | A* statistics log | ❌ No |
| A* edge (legacy) | static·(1+λ·cost) | λ=2.0 | `use_integrity_front_search` | `/iap/integrity_front_cost_field` | A* statistics log | ❌ No |
| Global A* waypoints | (same as A* edge) | — | `use_integrity_global_search` | Risk overlay or front samples | Log line with stats | ❌ No |
| B-spline gradient (overlay) | Σ b_j·∇sample·w | λ=0.00001 | `risk_overlay_use_for_bspline` | GridMap risk overlay buffer | CSV debug topic | ❌ No |
| B-spline gradient (legacy) | Σ nearest·(cost,grad) | λ=0.00001 | `use_integrity_cost` | `/iap/integrity_cost_field` | CSV debug | ❌ No |

### I.6 IntegrityPlanner Cost (NOT integrated with ego_planner)

The `IntegrityPlanner` class has its own cost function:
$$J(\tau) = w_{int} \sum_k \max(0, PL_k/AL_k - 1)^2 + w_{turn} D_{turn} + w_{mission} \cdot dist_{goal} + w_{eff} \cdot effort + w_{infeas} \cdot I_{infeas}$$

This is **NOT** used by the ego_planner. It exists as an independent planner implementation.

---

## J. Failure Mode and Fallback Matrix

| # | Scenario | Expected Behavior | Actual Code Behavior | Evidence | Status |
|---|----------|-------------------|---------------------|----------|--------|
| J.1 | **GNSS unavailable** | PL falls back to estimator covariance | `IntegrityMonitor::compute()`: if no ARAIM result, PL = K_pl · sqrt(λ_max(Σ_p)) | `integrity_monitor.cpp` | ✅ OK |
| J.2 | **LiDAR observability unavailable** | Advisory PL uses GNSS-only | `FuturePLFieldPredictor`: if no LiDAR FIM, returns GNSS-only advisory PL | `future_pl_field_predictor.cpp` | ✅ OK |
| J.3 | **PL is NaN or infinite** | Should be caught and handled | **UNKNOWN** — no explicit NaN guard found in IntegrityMonitor compute path. `pi_cost_value_valid()` checks exist in evaluator but may not cover all paths. | `phase2_planner_integrity_evaluator.cpp` line 1930 | ⚠️ RISK |
| J.4 | **FGO information matrix singular** | Eigenvalue check, fallback | `FGOInformationManager::extract()`: checks min eigenvalue > eps_degen (1e-10). If singular, `has_data()` returns false. | `fgo_information_manager.cpp` | ✅ OK |
| J.5 | **Risk grid stale** | Exponential decay, UNKNOWN cost penalty | `riskOverlayPiCost()`: if stale > timeout, applies exponential decay with τ = stale_tau_s. UNKNOWN risk cost = λ_unknown (10.0). | `grid_map.cpp` line 766+ | ✅ OK |
| J.6 | **Planner starts before risk grid ready** | Should degrade gracefully (zero cost) | Risk overlay: if no data in buffer, `integrateRiskOnEdge()` returns false, cost = 0. Legacy: if `integrity_samples_` empty, `queryFrontIntegrityCost()` returns false. | `dyn_a_star.cpp`, `bspline_optimizer.cpp` | ✅ OK |
| J.7 | **Topic publisher missing** (e.g., `/iap/integrity` not published) | Should not crash | Subscriber simply never fires callback. IntegrityReport pointer is null in planner. **Silent degradation** — no warning emitted. | `integrity_planner.cpp` | ⚠️ RISK |
| J.8 | **Topic subscriber missing** | Publisher queue full → oldest messages dropped | PointCloud2 publishers use QoS(1).best_effort() — no guarantee of delivery. If no subscriber, messages are silently dropped. | `phase2_planner_integrity_evaluator.cpp` | ⚠️ RISK |
| J.9 | **Frame mismatch** (e.g., `world` vs `map`) | Should reject with warning | `GridMap::ingestRiskOverlayCloud()`: rejects frame if `msg.header.frame_id != mp_.frame_id_`, logs WARN. | `grid_map.cpp` line 618 | ✅ OK |
| J.10 | **Integrity extension not loaded** | No integrity published | If `libintegrity_extension.so` not in extension_modules list, `/iap/integrity` topic not created. `phase2_planner_integrity_evaluator` subscriber never receives data. No error at the evaluator. | `config_ros.json`, `integrity_extension.cpp` | ⚠️ RISK |
| J.11 | **Cost field fields mismatch** | Should reject and warn | `GridMap::ingestRiskOverlayCloud()`: validates required fields (x,y,z,hpl_adv,vpl_adv,flags,stamp_s). Rejects with WARN if missing. | `grid_map.cpp` line 587-602 | ✅ OK |
| J.12 | **Odometry timestamp jumps (bag loop)** | Should be detected | **UNKNOWN** — no explicit timestamp jump detection found in odometry consumer. Evaluator may produce stale cost field. | — | UNKNOWN |
| J.13 | **Phase2 evaluator launched but no /iap/integrity** | Should log warning | **UNKNOWN** — evaluator subscribes to `/iap/integrity` but no explicit check if data ever arrives. | `phase2_planner_integrity_evaluator.cpp` | UNKNOWN |
| J.14 | **B-spline optimizer optimization diverges** | Integrity cost should not cause divergence | Gradient clipping: `integrity_grad_norm_max` (0.1) and `integrity_cost_max` (1000.0). Risk overlay path uses local gradient from GridMap. | `bspline_optimizer.cpp` | ✅ OK |

---

## K. Integration Audit Matrix

| # | Item | Expected Design Behavior | Actual Code Behavior | Status | Evidence | Recommended Fix | Priority |
|---|------|-------------------------|---------------------|--------|----------|-----------------|----------|
| K.1 | **ARAIM → PL computation** | ARAIM computes HPL/VPL from GNSS measurements | ✅ Implemented. `Araim::run()` called by `IntegrityMonitor::compute()`. Supports single-sat + trunk fault hypotheses. | **OK** | `araim.cpp`, `integrity_monitor.cpp` | — | — |
| K.2 | **PL → /iap/integrity topic** | IntegrityReport published on ROS topic | ✅ Implemented. `IntegrityExtensionModule` publishes `/iap/integrity` at smoother update rate. | **OK** | `integrity_extension.cpp` line 233 | — | — |
| K.3 | **/iap/integrity → cost field** | IntegrityReport consumed to generate cost fields for planner | ⚠️ Partial. `phase2_planner_integrity_evaluator` subscribes to `/iap/integrity` and generates cost fields, but: (a) only launched in demo9/10/11, (b) `publish_integrity_front_cost_field` defaults to `false`. | **PARTIAL** | `phase2_planner_integrity_evaluator.cpp` lines 781, 988 | 1. Launch evaluator in all configurations. 2. Set `publish_integrity_front_cost_field=true` by default. 3. Or move cost field publishing into `IntegrityExtensionModule`. | **HIGH** |
| K.4 | **Cost field → A*** | Integrity cost used in A* edge cost | ⚠️ Code exists, disabled by default. Two paths: (a) risk overlay: `risk_overlay_use_for_astar=false`, (b) legacy: `use_integrity_front_search=false`. | **PARTIAL** | `dyn_a_star.cpp` lines 305-350, `advanced_param.launch.py` | Set `risk_overlay_use_for_astar=true` (or `use_integrity_front_search=true`) in default config. Requires K.3 fix first. | **HIGH** |
| K.5 | **Cost field → B-spline** | Integrity cost + gradient used in B-spline optimization | ⚠️ Code exists, disabled by default. Two paths: (a) risk overlay: `risk_overlay_use_for_bspline=false`, (b) legacy: `use_integrity_cost=false`. | **PARTIAL** | `bspline_optimizer.cpp` lines 1484, 2511, `advanced_param.launch.py` | Set `risk_overlay_use_for_bspline=true` (or `use_integrity_cost=true`) in default config. Requires K.3 fix first. | **HIGH** |
| K.6 | **Risk overlay → A*** | Risk overlay buffer used in A* | ⚠️ `risk_overlay_enable=true` (data ingested), but `risk_overlay_use_for_astar=false` (not used). Data is collected but never consumed. | **PARTIAL** | `grid_map.cpp` lines 422, 581; `advanced_param.launch.py` | Set `risk_overlay_use_for_astar=true`. | **HIGH** |
| K.7 | **Risk overlay → B-spline** | Risk overlay buffer used in B-spline optimization | ⚠️ Same as K.6: data ingested but `risk_overlay_use_for_bspline=false`. | **PARTIAL** | `grid_map.cpp`, `bspline_optimizer.cpp`, `advanced_param.launch.py` | Set `risk_overlay_use_for_bspline=true`. | **HIGH** |
| K.8 | **Predicted PL → planner** | Future PL predictions used for trajectory evaluation | ❌ `IntegrityPlanner` uses `PredictedIntegrityComputer` and `PredictedAraimComputer`, but `IntegrityPlanner` is NOT used by ego_planner. The ego_planner does NOT use predicted PL — it only uses current integrity snapshots via cost fields. | **MISSING** | `integrity_planner.cpp`, `planner_manager.cpp` | Integrate `IntegrityPlanner` with ego_planner, OR publish predicted PL in cost field (already partially done: hpl_adv/vpl_adv in front cost field). | **MEDIUM** |
| K.9 | **IntegrityPlanner → ego_planner** | Receding-horizon integrity planner drives the drone | ❌ `IntegrityPlanner` is a standalone C++ class with no ROS integration. `EGOPlannerManager` is the actual flight planner and does NOT use `IntegrityPlanner`. | **MISSING** | `integrity_planner.cpp`, `planner_manager.cpp` | Either: (a) integrate `IntegrityPlanner` as a mode in `EGOPlannerManager`, or (b) accept dual-planner architecture and document the divide. | **LOW** |
| K.10 | **GNSS extension → GNSS handler** | GNSS measurements fed to factor graph | ✅ Implemented. `GnssExtensionModule` subscribes to GNSS topics, `GnssHandler` builds pseudorange/Doppler factors, added to iSAM2. | **OK** | `gnss_extension.cpp`, `gnss_handler.cpp` | — | — |
| K.11 | **Trunk extension → trunk mapping** | Trunk landmarks detected and mapped | ✅ Implemented. `TrunkExtensionModule` detects trunks from LiDAR, `TrunkMap` maintains EKF, used for HAL computation. | **OK** | `trunk_extension.cpp`, `trunk_detector.cpp`, `trunk_map.cpp` | — | — |
| K.12 | **FGO sigma → integrity** | Estimate covariance extracted for integrity | ✅ Implemented. `FGOInformationManager::extract()` called on smoother update. | **OK** | `fgo_information_manager.cpp`, `integrity_extension.cpp` | — | — |
| K.13 | **LiDAR ARAIM → integrity** | LiDAR block-level integrity monitoring | ✅ Implemented. `LidarAraim` called when LiDAR snapshot available. | **OK** | `lidar_araim.cpp`, `integrity_monitor.cpp` | — | — |
| K.14 | **Cost field gradient → B-spline** | Gradient used in optimization | ⚠️ Partial. Legacy path (`/iap/integrity_cost_field`) includes gradients. Front cost field (`/iap/integrity_front_cost_field`) sets `grad_x/y/z=0.0f`. Risk overlay path computes gradients locally from GridMap. | **PARTIAL** | `phase2_planner_integrity_evaluator.cpp` line 4060; `bspline_optimizer.cpp` line 1484 | Option 1: Compute gradients in evaluator. Option 2: Use risk overlay path (computes gradients locally). Both require enabling. | **MEDIUM** |
| K.15 | **Global waypoints → integrity** | Global path respects integrity constraints | ⚠️ `buildIntegrityAwareGlobalWaypoints()` exists and is called from `planGlobalTraj()`, but requires `use_integrity_global_search=true` (default false) AND risk overlay enabled. | **PARTIAL** | `planner_manager.cpp` lines 458, 560, 628 | Enable `use_integrity_global_search=true` in default config. | **MEDIUM** |
| K.16 | **UnifiedRiskGrid → planner** | URG used for fast risk queries | ⚠️ URG exists but `use_unified_risk_grid=false` by default. When enabled, can publish to legacy topics. | **PARTIAL** | `unified_risk_grid.cpp`, `phase2_planner_integrity_evaluator.cpp` | Enable `use_unified_risk_grid=true` and verify integration. | **LOW** |
| K.17 | **CSV logging → debug** | Debug data exported for analysis | ✅ Implemented. Multiple CSV paths: ARAIM debug, trajectory, timing, planner integrity, phase2 summary. | **OK** | `integrity_extension.cpp`, `phase2_planner_integrity_evaluator.cpp` | — | — |
| K.18 | **RViz markers → visualization** | Integrity data visualized in RViz | ✅ Implemented. `/iap/araim_envelopes`, `/iap/planner_integrity_markers`, `/iap/integrity_viz/*`, `/grid_map/risk_overlay_debug`. | **OK** | `integrity_extension.cpp`, `phase2_planner_integrity_evaluator.cpp`, `grid_map.cpp` | — | — |
| K.19 | **Sim → IAP GNSS** | Simulated GNSS used in sim demos | ✅ Implemented. `gnss_sim` package generates GNSS measurements. `demo9/10/11` launch files configure GNSS simulation parameters. | **OK** | `gnss_sim/`, `demo9_ego_planner_closed_loop.launch.py` | — | — |
| K.20 | **Live pipeline cost field** | Cost field published in live (non-sim) pipeline | ❌ `phase2_planner_integrity_evaluator` is NOT launched by `iap_rosnode.launch.py`. Live pipeline has NO cost field bridge. | **MISSING** | `iap_rosnode.launch.py` vs `demo10_ego_planner_pi_lite_eval.launch.py` | Launch evaluator in live pipeline, or integrate cost field publishing into `IntegrityExtensionModule`. | **HIGH** |

---

## L. Test Gap Matrix

### L.1 Existing Tests

| Test File | Module | Type | Coverage |
|-----------|--------|------|----------|
| `test_alert_limit_model.cpp` | AlertLimitModel | Unit | HAL/VAL computation, drone radius, safety buffer |
| `test_araim.cpp` | Araim, IntegrityState, TrunkMap | Unit | Q_inv, 3-term PL formula, HPL/VPL, state transitions, TrunkMap EKF |
| `test_future_pl_field_predictor.cpp` | FuturePLFieldPredictor | Unit | Grid sampling, GNSS epochs, LiDAR observability |
| `test_integrity_snapshot.cpp` | IntegritySnapshot | Unit | Snapshot structure, GNSS/LiDAR fusion |
| `test_lidar_observability_fim.cpp` | LidarObservabilityFim | Unit | FIM computation, trunk observations, TDOP |
| `test_local_occupancy.cpp` | LocalOccupancyGrid | Unit | Rolling occupancy, eviction policies |
| `test_odom_freshness.cpp` | Odometry freshness | Unit | Timestamp validation, age checking |
| `test_phase2_summary_schema.py` | Phase2 summary schema | Unit | Schema validation for phase2 logs |
| `test_pi_cost_adapter.cpp` | PICostAdapter | Unit | Safe/marginal/unsafe risk bands, hinge/ratio terms |
| `test_pl_grid.cpp` | PLGrid | Unit | Voxel queries, HPL/VPL at grid positions |
| `test_predicted_araim.cpp` | PredictedAraimComputer | Unit | Future visibility, HPL/VPL prediction |
| `test_run_log_manager.cpp` | RunLogManager | Unit | Log directory management, CSV schema |
| `test_unified_risk_grid.cpp` | UnifiedRiskGrid | Unit | ESDF, occupancy, AL, PL, PI cost fields |
| `test_bspline_risk_overlay.cpp` (in `bspline_opt/test/`) | B-spline risk overlay | Unit | B-spline optimization with risk overlay |
| `test_astar_risk_overlay.cpp` (in `path_searching/test/`) | A* risk overlay | Unit | A* search with risk overlay |
| `test_risk_overlay.cpp` (in `plan_env/test/`) | Risk overlay | Unit | Comprehensive risk overlay testing |

### L.2 Missing Test Categories

| Category | Description | Priority |
|----------|-------------|----------|
| **Integration: End-to-end pipeline** | Test the full sensor→estimator→integrity→planner→controller chain in simulation with rosbag replay | **CRITICAL** |
| **Integration: Topic contract** | Verify all topic message types, field names, frame_ids match between publishers and subscribers. Validate field count and types for cost fields. | **CRITICAL** |
| **Integration: Cost field round-trip** | Generate cost field → ingest in GridMap → run A* → verify cost is applied correctly → verify B-spline gradient is applied correctly | **CRITICAL** |
| **System: Launch file validation** | Automated launch+checks for each demo: verify all nodes start, all topics connected, no ERROR log lines | **HIGH** |
| **System: Demo-level checks** | Rosbag replay with known GNSS/LiDAR data → verify ARAIM outputs within expected bounds → verify PL < AL for safe trajectories | **HIGH** |
| **Stress: GNSS outage** | Simulate GNSS dropout → verify PL gracefully increases → verify planner avoids high-PL regions | **HIGH** |
| **Stress: LiDAR dropout** | Simulate LiDAR outage → verify odometry covariance grows → verify integrity monitor falls back to GNSS-only | **HIGH** |
| **Stress: Frame mismatch** | Publish cost field with wrong frame_id → verify GridMap rejects with warning → verify planner falls back to non-integrity path | **MEDIUM** |
| **Regression: Cost functions** | Golden-output tests for PI cost, risk overlay cost, B-spline gradient — verify output doesn't change unexpectedly | **MEDIUM** |
| **Regression: Parameter defaults** | Verify that with default parameters, the system behaves identically to a baseline without integrity | **MEDIUM** |
| **Unit: NaN/Inf handling** | Test each module with NaN and Inf inputs → verify graceful handling (not crash) | **MEDIUM** |
| **Unit: Parameter validation** | Test that invalid parameter combinations (e.g., risk_overlay_enable=false but risk_overlay_use_for_astar=true) produce warnings | **LOW** |
| **Unit: Timestamp edge cases** | Test with zero timestamps, future timestamps, very old timestamps → verify staleness logic | **LOW** |
| **Documentation: Integration test guide** | Write step-by-step guide for running integration tests: which launch files, what rosbags, expected outputs | **LOW** |

### L.3 Recommended Test Implementation Order

1. **Topic contract test** — verify field compatibility between `phase2_planner_integrity_evaluator` cost field publishers and `GridMap`/`BsplineOptimizer` subscribers (catches K.3 - K.7 issues immediately)
2. **End-to-end pipeline test** — launch demo9 with integrity enabled, verify data flows through all nodes
3. **Cost field round-trip test** — focused test on A* + B-spline integrity cost computation
4. **GNSS outage stress test** — verify fallback behavior
5. **NaN/Inf unit tests** — prevent crashes in production

---

## Audit Summary

### Key Findings

1. **✅ Integrity monitoring pipeline is functional.** ARAIM, IntegrityMonitor, FGO sigma extraction, LiDAR ARAIM all work and publish `/iap/integrity`.

2. **⚠️ Cost field bridge is a separate node and disabled by default.** `phase2_planner_integrity_evaluator` is the only node that publishes cost fields, but it is not launched in the main pipeline (`iap_rosnode.launch.py`) and its publishers default to OFF.

3. **⚠️ Ego_planner integrity integration is fully coded but fully disabled.** Risk overlay ingestion works, A* and B-spline cost functions are implemented, global waypoint generation with integrity exists — but ALL enable flags default to `false`.

4. **❌ IntegrityPlanner is a parallel system.** The `IntegrityPlanner` class (receding-horizon with motion primitives) is NOT connected to `EGOPlannerManager`. Two separate planners exist, only one flies the drone.

5. **❌ Live pipeline has no integrity→planner data path.** From `iap_rosnode.launch.py`, integrity reports are published but never consumed by any planner.

6. **⚠️ Front cost field gradients are zero.** The `/iap/integrity_front_cost_field` topic sets `grad_x/y/z = 0.0f`, so legacy B-spline integrity path gets no gradient information from the front field. Risk overlay path computes gradients locally — but this path is also disabled.

### Top Recommended Actions

| Priority | Action | Affected Sections |
|----------|--------|-------------------|
| **P0** | Set `publish_integrity_front_cost_field=true` by default in evaluator | K.3, G.8 |
| **P0** | Set `risk_overlay_use_for_astar=true` and `risk_overlay_use_for_bspline=true` by default | K.4-K.7, G.7 |
| **P0** | Launch `phase2_planner_integrity_evaluator` in `iap_rosnode.launch.py` (or integrate cost field publishing into `IntegrityExtensionModule`) | K.3, K.20 |
| **P1** | Write topic contract integration test | L.2 |
| **P1** | Add NaN/Inf guards in integrity computation chain | J.3 |
| **P2** | Integrate `IntegrityPlanner` with `EGOPlannerManager` OR document the dual-planner architecture decision | K.8, K.9 |
| **P2** | Compute and publish gradients in front cost field | K.14 |
| **P3** | Add integration tests for GNSS outage, LiDAR dropout, frame mismatch | L.2 |

---

## M. Simulation Environment Infrastructure

### M.1 Overview

The simulation environment is located at `sim/ego_planner_swarm_ws/src/uav_simulator/` and consists of six packages that together provide a complete closed-loop UAV simulation with GNSS, LiDAR, IMU, dynamics, and environment generation. The simulator integrates with the IAP pipeline via standard ROS 2 topics.

```
┌──────────────────────────────────────────────────────────────────┐
│                    Simulation Data Flow                          │
│                                                                  │
│  mockamap / map_generator                                        │
│    │  (generates 3D obstacle map)                                │
│    ▼                                                             │
│  local_sensing (LiDAR render from map)                           │
│    │  (publishes /sim/drone_0/lidar)                             │
│    ▼                                                             │
│  gnss_sim (GNSS measurement generation)                          │
│    │  (publishes /ublox_driver/*)                                │
│    ├──────────────────────────────────────────┐                  │
│    ▼                                          ▼                  │
│  iap_rosnode (state estimation + integrity)                       │
│    │                                                             │
│    ▼                                                             │
│  ego_planner (path planning)                                     │
│    │                                                             │
│    ▼                                                             │
│  so3_control (geometric controller)                              │
│    │  (publishes SO3Command → motor RPMs)                        │
│    ▼                                                             │
│  so3_quadrotor_simulator (dynamics + IMU)                        │
│    │  (publishes truth_odom, imu, imu_iap)                       │
│    └──────► back to gnss_sim, local_sensing, so3_control         │
└──────────────────────────────────────────────────────────────────┘
```

### M.2 so3_quadrotor_simulator — Quadrotor Dynamics

**Package:** `so3_quadrotor_simulator`
**Executable:** `quadrotor_simulator_so3`
**Source:** `so3_quadrotor_simulator/src/quadrotor_simulator_so3.cpp`

**Dynamics Model (22-state RK4 integration):**

The drone is modeled as a rigid body with 4 independent rotors. State vector:

$$\mathbf{x} = [p_x, p_y, p_z, v_x, v_y, v_z, R_{00}, R_{01}, \ldots, R_{22}, \omega_x, \omega_y, \omega_z, \omega_{m1}, \omega_{m2}, \omega_{m3}, \omega_{m4}]^T$$

**Equations of motion (integrated via Boost odeint RK4):**

Translation:
$$\dot{\mathbf{p}} = \mathbf{v}$$

$$\dot{\mathbf{v}} = -g\hat{\mathbf{z}} + \frac{T}{m} R\mathbf{e}_3 + \frac{\mathbf{F}_{ext}}{m} - \frac{C_d \mathbf{v} \|\mathbf{v}\|}{m}$$

Rotation:
$$\dot{R} = R[\boldsymbol{\omega}]_\times$$

$$\dot{\boldsymbol{\omega}} = J^{-1}(\mathbf{M} - \boldsymbol{\omega} \times J\boldsymbol{\omega} + \mathbf{M}_{ext})$$

Motor dynamics (first-order lag):
$$\dot{\omega}_{m,i} = \frac{u_i - \omega_{m,i}}{\tau}$$

**Motor-to-wrench mapping (X-configuration):**

Thrust: $T = k_f \sum_{i=1}^{4} \omega_{m,i}^2$

Roll moment: $M_x = k_f d (\omega_{m,3}^2 - \omega_{m,4}^2)$

Pitch moment: $M_y = k_f d (\omega_{m,2}^2 - \omega_{m,1}^2)$

Yaw moment: $M_z = k_m (\omega_{m,1}^2 + \omega_{m,2}^2 - \omega_{m,3}^2 - \omega_{m,4}^2)$

**Key Physical Parameters:**

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `mass` | 0.98 | kg | Vehicle mass |
| `Ixx`, `Iyy` | 2.64×10⁻³ | kg·m² | Roll/pitch inertia |
| `Izz` | 4.96×10⁻³ | kg·m² | Yaw inertia |
| `arm_length` | 0.26 | m | Motor-to-center distance |
| `prop_radius` | 0.062 | m | Propeller radius |
| `kf` | 8.98×10⁻⁹ | N·s² | Thrust coefficient |
| `km` | 0.07·(3·r)·kf | N·m·s² | Moment coefficient |
| `motor_tau` | 1/30 | s | Motor time constant |
| `min_rpm` / `max_rpm` | 1200 / 35000 | RPM | Motor limits |
| `drag_coefficient` | 0.1 | — | Aerodynamic drag |
| `gravity` | 9.81 | m/s² | Gravitational acceleration |

**ROS Topics:**

| Topic | Type | Dir | Rate | Content |
|-------|------|-----|------|---------|
| `cmd` (sub) | `SO3Command` | ← | 100 Hz | Force vector + orientation + auxiliary states |
| `odom` (pub) | `nav_msgs/Odometry` | → | 100 Hz | **Ground truth pose** (position, velocity, attitude, angular velocity) |
| `imu` (pub) | `sensor_msgs/Imu` | → | 100 Hz | **Standard IMU**: orientation quaternion + angular velocity + linear acceleration (world frame $\dot{v}$) |
| `imu_iap` (pub) | `sensor_msgs/Imu` | → | 100 Hz | **IAP-specific IMU**: same as above but linear_acceleration = specific force in body frame: $\mathbf{a}_{body} = R^T(\mathbf{a}_{world} - \mathbf{g})$ |
| `force_disturbance` (sub) | `geometry_msgs/Vector3` | ← | on change | External force perturbations |
| `moment_disturbance` (sub) | `geometry_msgs/Vector3` | ← | on change | External moment perturbations |

**IMU Generation Detail:**

Two IMU publishers exist:

1. **Standard IMU** (`/sim/drone_0/imu`): orientation = quaternion from R; angular_velocity = $\boldsymbol{\omega}$; linear_acceleration = $\dot{\mathbf{v}}$ (world-frame acceleration, **not** specific force). This is the default IMU consumed by the controller.

2. **IAP IMU** (`/sim/drone_0/imu_iap`): linear_acceleration = $R^T(\dot{\mathbf{v}} - \mathbf{g})$ (body-frame specific force). This matches what a real IMU accelerometer measures (acceleration minus gravity, expressed in sensor frame). Enabled when `iap_imu/enable=true`.

**Simulation Rate Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rate/simulation` | 1000 Hz | RK4 integration step |
| `rate/odom` | 100 Hz | Odometry + IMU publish rate |
| `simulator/hold_until_cmd` | false | Freeze dynamics until first motor command |

### M.3 so3_control — Geometric Attitude Controller

**Package:** `so3_control`
**Executable:** `SO3ControlComponent` (composable node)
**Source:** `so3_control/src/so3_control.cpp`

**Control Architecture:**

The controller uses a cascaded position-attitude control law on the SO(3) manifold:

**Step 1 — Desired force from position error:**
$$\mathbf{F} = m g \hat{\mathbf{z}} + k_x(\mathbf{x}_d - \mathbf{x}) + k_v(\mathbf{v}_d - \mathbf{v}) + m \mathbf{a}_d$$

**Step 2 — Desired attitude from force + yaw:**
- $b_3^c = \frac{\mathbf{F}}{\|\mathbf{F}\|}$ (thrust direction)
- $b_1^d = [\cos\psi_d, \sin\psi_d, 0]^T$ (desired heading)
- $b_2^c = \frac{b_3^c \times b_1^d}{\|b_3^c \times b_1^d\|}$ (right direction)
- $b_1^c = b_2^c \times b_3^c$
- $R_d = [b_1^c, b_2^c, b_3^c]$

**Step 3 — Attitude error on SO(3):**
$$\mathbf{e}_R = \frac{1}{2}(R_d^T R - R^T R_d)^\vee$$

**Step 4 — Moment command:**
$$\mathbf{M} = -k_R \mathbf{e}_R - k_\omega \boldsymbol{\omega}$$

**Step 5 — Motor allocation (solves for RPMs from F, M):**
$$\begin{bmatrix} \omega_1^2 \\ \omega_2^2 \\ \omega_3^2 \\ \omega_4^2 \end{bmatrix} = \begin{bmatrix} 1/(4k_f) & 0 & -1/(2dk_f) & 1/(4k_m) \\ 1/(4k_f) & 0 & 1/(2dk_f) & 1/(4k_m) \\ 1/(4k_f) & 1/(2dk_f) & 0 & -1/(4k_m) \\ 1/(4k_f) & -1/(2dk_f) & 0 & -1/(4k_m) \end{bmatrix} \begin{bmatrix} F \\ M_x \\ M_y \\ M_z \end{bmatrix}$$

**Controller Gains:**

| Gain | Default (x, y, z) | Purpose |
|------|--------------------|---------|
| `gains/kx` | 5.7, 5.7, 6.2 | Position error → force |
| `gains/kv` | 3.4, 3.4, 4.0 | Velocity error → force |
| `gains/rot` | 1.5, 1.5, 1.0 | Attitude error → moment |
| `gains/ang` | 0.13, 0.13, 0.1 | Angular velocity error → moment |
| `mass` | 0.98 kg | Vehicle mass |

**ROS Topics:**

| Topic | Type | Dir | Purpose |
|-------|------|-----|---------|
| `odom` (sub) | `Odometry` | ← | Current state feedback |
| `position_cmd` (sub) | `PositionCommand` | ← | **Desired trajectory from planner** |
| `imu` (sub) | `Imu` | ← | IMU acceleration feedforward |
| `so3_cmd` (pub) | `SO3Command` | → | Motor commands to dynamics |
| `motors` (sub) | `Bool` | ← | Motor enable/disable |
| `corrections` (sub) | `Corrections` | ← | Thrust/angle bias trim |

### M.4 local_sensing — LiDAR Simulation from Map

**Package:** `local_sensing`
**Executable:** `pcl_render_node`
**Source:** `local_sensing/src/pcl_render_node.cpp`

**Algorithm — KD-tree radius search with FOV filtering:**

1. **Map ingestion:** Subscribe to global map PointCloud2, build KD-tree (0.1 m resolution downsampling)
2. **Per odometry update:**
   - Get drone pose (position $\mathbf{p}$, rotation $R$)
   - KD-tree radius search: all map points within `sensing_horizon` of $\mathbf{p}$
   - **FOV filtering (two constraints):**
     - Elevation: $\tan^{-1}(|\Delta z| / \text{horizon\_xy}) < 30°$ (limit ground returns)
     - Forward-facing: $(\mathbf{p}_i - \mathbf{p}) \cdot \mathbf{b}_1 > 0.5 \cdot \text{horizon}$ (cosine threshold, front hemisphere only)
   - Publish filtered local point cloud as simulated LiDAR

**ROS Topics:**

| Topic | Type | Dir | Rate | Content |
|-------|------|-----|------|---------|
| `global_map` (sub) | `PointCloud2` | ← | once | Complete obstacle map (from mockamap/map_generator) |
| `odometry` (sub) | `Odometry` | ← | ~100 Hz | Drone pose for sensing FOV |
| `pcl_render_node/cloud` (pub) | `PointCloud2` | → | ~10 Hz | **Simulated LiDAR scan** |

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sensing_horizon` | variable | Maximum LiDAR range |
| `sensing_rate` | variable | LiDAR publish frequency |
| `map/resolution` | 0.1 m | KD-tree voxel size |

**Frame Convention:**
- Input map: typically in `world` frame
- Output LiDAR: in `world` frame (NOT body frame — requires `demo4_lidar_body_bridge` for body-frame processing)

**CUDA Depth Render Variant:**
An alternative GPU-accelerated mode renders depth images from the 3D map using CUDA kernels, then back-projects to point cloud. Used when `camera.yaml` is configured with intrinsics. Publishes `/depth_image` (mono16) and `/pcl_world` (PointCloud2).

### M.5 gnss_sim — GNSS Signal Simulation (Detailed)

**Package:** `gnss_sim`
**Executable:** `gnss_sim_node`
**Source:** `gnss_sim/src/gnss_sim_node.cpp`
**Launch:** `gnss_sim/launch/gnss_sim.launch.py`

**Architecture (4 main classes):**

| Class | Responsibility |
|-------|---------------|
| `GnssSimNode` | Main ROS node: parameter management, timing, publisher/subscriber orchestration |
| `SatelliteConstellation` | Ephemeris data management: synthetic generation or RINEX loading |
| `VisibilityModel` | Per-satellite visibility classification (LOS/NLOS/OCCLUDED/FAULTED) |
| `TimeSyncBuffer` | Circular buffer for truth odometry interpolation |

**Pseudorange Measurement Model:**

$$\rho_{meas} = \rho_{geo} + \Delta\rho_{sagnac} + c \cdot (dt_r - dt^s) + I + T + TGD \cdot c + \Delta\rho_{nlos} + \Delta\rho_{mp} + \Delta\rho_{fault} + \eta_{\rho}$$

Where:
- $\rho_{geo} = \|\mathbf{r}_{sat} - \mathbf{r}_{rcv}\|$ — geometric range
- $\Delta\rho_{sagnac}$ — Sagnac effect correction (Earth rotation during signal transit)
- $c \cdot dt_r$ — receiver clock bias (configurable, default 0 m)
- $c \cdot dt^s$ — satellite clock correction (from ephemeris)
- $I$ — ionospheric delay (Klobuchar model, 8 coefficients)
- $T$ — tropospheric delay
- $TGD \cdot c$ — transmit group delay
- $\Delta\rho_{nlos} \sim \mathcal{N}(15, 5^2)$ m — NLOS bias
- $\Delta\rho_{mp} = A \cdot \sin(2\pi t / T_{mp})$ — multipath oscillation
- $\Delta\rho_{fault}$ — injected fault bias (time-varying)
- $\eta_{\rho} \sim \mathcal{N}(0, 1^2)$ m — thermal noise

**Doppler Measurement Model:**

$$f_{dopp} = -\frac{\mathbf{v}_{rel} \cdot \hat{\mathbf{r}}}{c} f_{L1} + \Delta f_{sagnac} + c \cdot \dot{dt}_r - c \cdot \dot{dt}^s + \Delta f_{fault} + \eta_f$$

Where $\mathbf{v}_{rel} = \mathbf{v}_{sat} - \mathbf{v}_{rcv}$ (relative velocity along line-of-sight).

**CN0 (Signal Strength) Model:**

| Condition | CN0 (dBHz) | Notes |
|-----------|-----------|-------|
| LOS (clear) | 45 | Default; degrades ~0.2 dB/degree below 20° elevation |
| NLOS | 28 | Further degraded by multipath |
| LOW_ELEVATION | < 30 | Below min elevation mask |
| FAULTED | 45 − cn0_degrade | Degraded per fault config |

**Satellite Visibility Classification (8 states):**

The visibility model evaluates each satellite in priority order:

| State | Cause | Meas. Used? | CN0 | Can Upgrade? |
|-------|-------|------------|-----|-------------|
| LOS | Clear line-of-sight to open sky | ✅ Yes | 45 dBHz | N/A |
| NLOS | Blocked by map, but NLOS bias applied | ✅ Yes | 28 dBHz | — |
| OCCLUDED | Blocked by map, no NLOS fallback | ❌ No | 0 | — |
| BELOW_HORIZON | Below local horizon (elevation < 0°) | ❌ No | 0 | No |
| LOW_ELEVATION | Below `min_elevation_deg` mask | ❌ No | Degraded | No |
| DROPPED | Fault: complete signal loss | ❌ No | 0 | No |
| FAULTED | Fault: bias/degradation active | ✅ Yes | Degraded | No |
| LOW_CN0 | CN0 < threshold (LOS only) | ❌ No | Low | No |

**Visibility evaluation order:**
1. Horizon gating (elevation < 0 → BELOW_HORIZON, **permanent**)
2. Elevation mask (elevation < min_elevation_deg → LOW_ELEVATION, **permanent**)
3. Sky mask constraints (az/el region gating, configurable)
4. Map occlusion: ray-cast from receiver toward satellite, check against voxelized map
   - If ray intersects map and NLOS enabled → NLOS
   - If ray intersects map and NLOS disabled → OCCLUDED
5. Multipath: sinusoidal modulation on pseudorange (if enabled)
6. Fault injection: per-satellite biases, dropouts, CN0 degradation
7. CN0 thresholding: mark as LOW_CN0 if signal too weak

**Ephemeris Management:**

| Mode | Source | Description |
|------|--------|-------------|
| **Synthetic** (default) | Algorithmic generation | 24 GPS-like satellites in 6 orbital planes × 4 sats/plane. Semi-major axis ~26,560 km, inclination 55°. Refreshed every 3600 s. |
| **RINEX** | RINEX 3.04 NAV file | Loads real ephemeris from file. Supports GPS, BDS, Galileo, GLONASS constellations. Max age 7200 s before re-selection. Falls back to synthetic on error. |

**ROS Topics:**

| Topic | Type | Dir | Rate | Content |
|-------|------|-----|------|---------|
| `/sim/drone_0/truth_odom` (sub) | `Odometry` | ← | 100 Hz | **Receiver truth pose** (interpolated in TimeSyncBuffer) |
| `/sim/drone_0/lidar` (sub, optional) | `PointCloud2` | ← | 10 Hz | Trigger source when `time_source=trigger_topic` |
| `/map_generator/global_cloud` (sub, optional) | `PointCloud2` | ← | once | Map for occlusion ray-casting |
| `/ublox_driver/receiver_lla` (pub) | `NavSatFix` | → | 1 Hz | Receiver LLA position (WGS84) |
| `/ublox_driver/ephem` (pub) | `GnssEphemMsg` | → | 1 Hz | GPS/BDS/Galileo ephemeris |
| `/ublox_driver/glo_ephem` (pub) | `GnssGloEphemMsg` | → | 1 Hz | GLONASS ephemeris |
| `/ublox_driver/iono_params` (pub) | `GnssIonosphereParameter` | → | 1 Hz | Klobuchar iono coefficients |
| `/ublox_driver/range_meas` (pub) | `GnssMeasMsg` | → | 10 Hz | **Pseudorange + Doppler + CN0** for all visible SVs |
| `/gnss_sim/diagnostics` (pub) | `DiagnosticArray` | → | 10 Hz | SV counts, ephemeris status |
| `/gnss_sim/visualization/*` (pub) | `MarkerArray` | → | 10 Hz | Sky dome, SV positions, signal rays, skyplot |

**Key Configuration Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ephemeris_source` | "synthetic" | "synthetic" or "rinex" |
| `num_gps_sats` | 24 | Satellite count (synthetic mode) |
| `enabled_constellations_csv` | "GPS" | e.g., "GPS,BDS,GAL,GLO" |
| `origin_lat/lon_deg` | Shanghai | Reference origin (WGS84) |
| `measurement_rate_hz` | 10.0 | Pseudorange publish rate |
| `pseudorange_noise_std_m` | 1.0 | Thermal noise sigma |
| `doppler_noise_std_mps` | 0.1 | Doppler noise sigma |
| `min_elevation_deg` | 10.0 | Minimum SV elevation |
| `enable_map_occlusion` | false | Enable ray-cast occlusion |
| `enable_nlos` | false | Enable NLOS bias (false = occluded sats dropped) |
| `nlos_bias_mean_m` | 15.0 | NLOS pseudorange bias mean |
| `nlos_bias_std_m` | 5.0 | NLOS pseudorange bias std |
| `enable_multipath` | false | Enable multipath oscillation |
| `multipath_amp_m` | 0.0 | Multipath amplitude |
| `multipath_period_s` | 8.0 | Multipath oscillation period |
| `enable_fault_injection` | false | Enable fault scenarios |
| `scenario_file` | "" | YAML fault scenario path |
| `enable_visualization` | false | Enable RViz sky dome |
| `enable_csv_log` | false | Enable CSV debug logging |

**Fault Injection YAML Format:**

```yaml
anchor:
  lat_deg: 31.2304
  lon_deg: 121.4737
  alt_m: 25.0

skymask:
  enabled: true
  default_min_elevation_deg: 10.0
  az_el_deg:
    - {az_deg: 0.0, min_el_deg: 5.0}
    - {az_deg: 90.0, min_el_deg: 15.0}

faults:
  - constellation: "GPS"
    sat: 1
    start_time_s: 10.0
    duration_s: 20.0
    pseudorange_bias_m: 50.0       # Constant bias
    bias_rate_mps: 2.0             # Ramp rate
    doppler_bias_mps: 10.0         # Doppler error
    drop: false                    # Complete signal loss
    cn0_degrade_dbhz: 5.0          # CN0 reduction
  - constellation: "BDS"
    sat: 5
    start_time_s: 0.0
    duration_s: 100.0
    drop: true                     # SV completely removed
```

### M.6 map_generator — Random Forest Map Generation

**Package:** `map_generator`
**Executable:** `random_forest_sensing`
**Source:** `map_generator/src/random_forest_sensing.cpp`

**Algorithm:**
1. Randomly place N rectangular obstacles within map bounds
2. Each obstacle is voxelized at specified resolution
3. Only surface/exterior voxels are retained (hollow interior)
4. Optional z-anchor features: vertical posts, walls, shelves for structured environments

**ROS Topics:**

| Topic | Type | Dir | Rate | Content |
|-------|------|-----|------|---------|
| `odometry` (sub) | `Odometry` | ← | ~100 Hz | Drone position |
| `/map_generator/global_map` (pub) | `PointCloud2` | → | once | Full obstacle point cloud |
| `/map_generator/local_map` (pub) | `PointCloud2` | → | periodic | Local (FOV-culled) obstacles |

**Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obs_num` | variable | Number of obstacles |
| `width_min/max` | variable | Obstacle width range [m] |
| `height_min/max` | variable | Obstacle height range [m] |
| `resolution` | variable | Voxel size [m] |
| `sensing_range` | variable | Local map culling radius [m] |
| `x/y/z_size` | variable | Map dimensions [m] |

### M.7 mockamap — Procedural Map Generation

**Package:** `mockamap`
**Executable:** `mockamap_node`
**Source:** `mockamap/src/maps.cpp`

**Map Types:**

| Type ID | Name | Method | Typical Use |
|---------|------|--------|-------------|
| 0 | Random Obstacles | Random box placement, hollow voxelization | Generic cluttered scenes |
| 1 | Perlin 3D | 3D Perlin noise + FBM (fractal Brownian motion) summation, threshold-cut | Natural terrain, caves, forest canopy |
| 2 | 2D Maze | Recursive binary space partition with random doorways | 2D navigation challenges |
| 3 | 3D Maze | 3D recursive subdivision (experimental) | Complex 3D path planning |
| 4 | Fixed Map | Predefined obstacle layout | Reproducible benchmarking |

**Perlin 3D Algorithm (Type 1):**
1. Generate 3D Perlin noise: $n(x,y,z)$ at each voxel
2. FBM summation: $N = \sum_{i=1}^{f} a^i \cdot n(2^i x, 2^i y, 2^i z)$
3. Sort all voxel values, retain top $(1 - fill)$ fraction as obstacles
4. Post-process: KD-tree removal of fully enclosed interior points

**ROS Topics:**

| Topic | Type | Dir | Content |
|-------|------|-----|---------|
| `mock_map` (pub) | `PointCloud2` | → | Generated obstacle point cloud |

**Key Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `seed` | 4546 | Random seed for reproducibility |
| `resolution` | 0.38 m | Voxel size |
| `type` | 3 | Map type (0-4) |
| `x/y/z_length` | 100/100/10 m | Map dimensions |
| `complexity` (perlin) | 0.143 | Noise frequency scaling |
| `fill` (perlin) | 0.38 | Obstacle fill ratio |
| `fractal` (perlin) | 1 | FBM octave count |
| `attenuation` (perlin) | 0.5 | Per-octave amplitude decay |
| `update_freq` | 1.0 Hz | Regeneration frequency |

### M.8 iap_phase1_tools — Python Evaluation Tools

**Package:** `iap_phase1_tools`
**Location:** `sim/ego_planner_swarm_ws/src/iap_phase1_tools/`

Two main Python scripts:

| Script | Purpose |
|--------|---------|
| `phase1_closed_loop_logger.py` | Logs closed-loop simulation data: odometry, planner commands, controller outputs, GNSS measurements |
| `phase2_planner_integrity_evaluator.py` | Python counterpart to `phase2_planner_integrity_evaluator.cpp` — evaluates planner integrity metrics offline |

**Supporting Tools:**

| Script | Purpose |
|--------|---------|
| `tools/phase1/check_topic_contract.py` | Validates topic message types, field names, and frame IDs between publishers and subscribers |
| `tools/phase1/validate_phase1_closed_loop.py` | End-to-end validation of phase 1 closed-loop simulation results |
| `tools/phase2/validate_phase2_integrity_eval.py` | Validates phase 2 integrity evaluation outputs |
| `tools/phase2/run_phase2_h_lite_scenarios.py` | Batch runner for phase 2 H-lite scenarios |

### M.9 Simulation ↔ IAP Integration Summary

**Topic Bridge Table (Sim → IAP):**

| Simulator Topic | IAP Consumer | How Connected |
|----------------|-------------|---------------|
| `/sim/drone_0/imu` (or `imu_iap`) | `iap_rosnode` (IMU subscriber) | Direct remap in launch file: `imu_topic` param |
| `/sim/drone_0/lidar` | `iap_rosnode` (LiDAR subscriber) | Direct remap: `points_topic` param |
| `/ublox_driver/range_meas` | `libgnss_extension.so` | GNSS extension auto-subscribes to topics in `config_gnss.json` |
| `/ublox_driver/ephem` | `libgnss_extension.so` | Same as above |
| `/ublox_driver/receiver_lla` | `libgnss_extension.so` | Same as above |
| `/sim/drone_0/truth_odom` | `phase2_planner_integrity_evaluator` (via `/drone_0_visual_slam/odom`) | Planner uses estimator odom, not truth directly |

**Frame Coordination:**

| Component | Frame Published | Frame Expected |
|-----------|----------------|----------------|
| Simulator truth odom + IMU | `world` | `map` (IAP) — **potential mismatch** |
| GNSS sim receiver LLA | N/A (WGS84 LLA) | GNSS extension handles LLA→ECEF→ENU internally |
| Map generator / mockamap | `world` | — |
| Local sensing LiDAR | `world` | `map` (IAP) — **potential mismatch** |
| Cost field publisher | `map` (configurable) | GridMap expects `mp_.frame_id_` to match |

**⚠️ CRITICAL NOTE:** The simulator publishes sensor data in the `world` frame, but IAP operates in the `map` frame. For direct topic remapping to work, either:
- The launch file must publish a static `world→map` TF transform (identity), or
- IAP must be configured to accept `world` as its `map_frame_id`, or
- A bridge node must republish data with corrected frame_id

In practice, demo launch files (demo9/10/11) handle this by setting IAP's `map_frame_id` to match the simulator's `world` frame, or by remapping topics such that frame mismatches are avoided.

---

*Audit generated: 2026-05-20. Simulation analysis appended: 2026-05-21. All claims verified against source code at `/home/dev/ws_iap/src/iap/` and `/home/dev/ws_iap/src/iap/sim/ego_planner_swarm_ws/`.*
*No code was modified during this audit.*


