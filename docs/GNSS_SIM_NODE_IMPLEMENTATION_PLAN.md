# GNSS Sim Node Implementation Plan

## Summary

- Add an independent ROS2 C++ package `gnss_sim` under `src/iap/sim/ego_planner_swarm_ws/src/uav_simulator/gnss_sim`.
- Add executable `gnss_sim_node` that subscribes to UAV truth odometry and publishes the five `/ublox_driver/*` topics required by IAP.
- Implement v1 as a nominal GNSS simulator: no NLOS, no multipath, no map occlusion, and no fault injection.
- Keep `/ublox_driver/*` topic names absolute so the existing IAP `gnss_extension` can consume them without source changes.

## Key Interfaces

- Subscribe:
  - `/sim/drone_0/truth_odom` by default, type `nav_msgs/msg/Odometry`.
- Publish:
  - `/ublox_driver/receiver_lla`, type `sensor_msgs/msg/NavSatFix`.
  - `/ublox_driver/ephem`, type `gnss_comm/msg/GnssEphemMsg`.
  - `/ublox_driver/glo_ephem`, type `gnss_comm/msg/GnssGloEphemMsg` (publisher only in v1).
  - `/ublox_driver/iono_params`, type `gnss_comm/msg/GnssIonosphereParameter`.
  - `/ublox_driver/range_meas`, type `gnss_comm/msg/GnssMeasMsg`.

## Implementation Steps

1. Create package metadata and CMake for `gnss_sim`, depending on `rclcpp`, `nav_msgs`, `sensor_msgs`, `gnss_comm`, and `Eigen3`.
2. Implement `GnssSimNode` with parameters for origin LLA, truth odometry topic, rates, noise, elevation cutoff, receiver clock bias/drift, and random seed.
3. Convert local EGO world coordinates as ENU:
   - `origin_lla -> origin_ecef` with `gnss_comm::geo2ecef`.
   - `R_ecef_enu` with `gnss_comm::geo2rotation`.
   - `receiver_ecef = origin_ecef + R_ecef_enu * position_world`.
   - `receiver_vel_ecef = R_ecef_enu * velocity_world`.
4. Use odometry header stamp as the GNSS epoch source. Convert ROS UTC time to `gnss_comm::gtime_t`, then to GPST with `gnss_comm::utc2gpst`.
5. Build a deterministic GPS-only constellation:
   - 24 GPS L1 satellites, PRN 1-24.
   - Satellite ids generated with `gnss_comm::sat_no(SYS_GPS, prn)`.
   - Broadcast ephemerides stored as `gnss_comm::Ephem` and published through `gnss_comm::ephem2msg`.
6. Generate range measurements with the same propagation utilities used by IAP:
   - Use `gnss_comm::eph2pos` and `gnss_comm::eph2vel`.
   - Evaluate satellite state at approximate transmit time `t_tx = t_rx - pseudorange / c`.
   - Publish L1 pseudorange and Doppler through `gnss_comm::meas2msg`.
7. Publish startup topics in order after the first odometry message:
   - `receiver_lla`.
   - GPS ephemerides.
   - GPS ionosphere parameters.
   - `range_meas` at stable epoch rate.
8. Add launch integration:
   - Standalone `gnss_sim.launch.py`.
   - `demo4.launch` argument `start_gnss_sim`, default `true`.

## Test Plan

- Build:
  - `colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm --packages-select gnss_comm gnss_sim`
- Topic smoke test:
  - Launch demo4 and confirm `/sim/drone_0/truth_odom` is active.
  - Confirm `/ublox_driver/receiver_lla`, `/ublox_driver/ephem`, `/ublox_driver/iono_params`, and `/ublox_driver/range_meas` publish continuously.
  - Confirm `range_meas.meas.size()` is at least 4 in nominal operation.
- IAP integration test:
  - Run IAP with `libgnss_extension.so`.
  - Confirm logs report receiver origin set, Klobuchar parameters received, and GNSS epochs inserted.
  - Confirm `/iap/integrity` publishes when integrity is enabled.

## Current Status and Usage

- Implementation status:
  - `gnss_sim_node` is implemented in `src/iap/sim/ego_planner_swarm_ws/src/uav_simulator/gnss_sim/src/gnss_sim_node.cpp`.
  - It now provides GPS L1 v2 simulation features while preserving the original IAP-facing `/ublox_driver/*` topics.
  - It publishes `/ublox_driver/glo_ephem` as a ROS publisher handle, but GLONASS simulation is still not implemented.
  - Implemented v2 features include trigger-based timing, configurable GPS PRNs, scenario YAML loading, SkyMask, point-cloud raycast occlusion, LOS/NLOS/OCCLUDED/DROPPED/FAULTED states, NLOS bias, multipath, C/N0 degradation, fault injection, RViz markers, diagnostics, and optional CSV logging.

- Satellite count:
  - The default simulated constellation is 24 GPS satellites, PRN 1-24.
  - The PRN range is configurable with `num_gps_sats`, `gps_prn_min`, and `gps_prn_max`.
  - The actual number published in each `/ublox_driver/range_meas` epoch is the usable subset after visibility, occlusion, drop, and IAP-side filtering.
  - If fewer than 4 satellites are visible, the node logs a throttled warning and skips that epoch.

- World-to-GNSS anchor:
  - The node assumes the EGO world frame is local ENU.
  - The ENU origin is anchored to WGS84 by these ROS parameters:
    - `origin_lat_deg`, default `31.2304`
    - `origin_lon_deg`, default `121.4737`
    - `origin_alt_m`, default `25.0`
  - The conversion is:
    - `origin_lla -> origin_ecef` with `gnss_comm::geo2ecef(origin_lla)`
    - `R_ecef_enu = gnss_comm::geo2rotation(origin_lla)`
    - `receiver_ecef = origin_ecef + R_ecef_enu * odom.pose.position`
    - `receiver_vel_ecef = R_ecef_enu * odom.twist.twist.linear`
    - `receiver_lla = gnss_comm::ecef2geo(receiver_ecef)`
  - The initial `/ublox_driver/receiver_lla` therefore equals the configured anchor plus the current local ENU offset from truth odometry.

- Main runtime parameters:
  - `truth_odom_topic`, default `/sim/drone_0/truth_odom`
  - `origin_lat_deg`, `origin_lon_deg`, `origin_alt_m`
  - `measurement_rate_hz`, default `10.0`
  - `ephem_rate_hz`, default `1.0`
  - `navsat_rate_hz`, default `1.0`
  - `min_elevation_deg`, default `10.0`
  - `pseudorange_noise_std_m`, default `1.0`
  - `doppler_noise_std_mps`, default `0.1`
  - `receiver_clock_bias_m`, default `0.0`
  - `receiver_clock_drift_mps`, default `0.0`
  - `random_seed`, default `20260429`
  - `time_source`, default `odom_stamp`; demo7 uses `trigger_topic`
  - `trigger_topic`, default `/sim/drone_0/lidar`
  - `scenario_file`, default empty; demo7 uses `config/gnss_sim/demo7_open_sky.yaml`
  - `num_gps_sats`, default `24`
  - `gps_prn_min`, default `1`
  - `gps_prn_max`, default `24`
  - `ephemeris_source`, default `synthetic`; valid values are `synthetic` and `rinex`
  - `rinex_nav_file`, default empty
  - `rinex_ephem_max_age_s`, default `7200.0`
  - `rinex_gps_only`, default `true`
  - `fallback_to_synthetic_on_rinex_error`, default `true`
  - `enabled_constellations_csv`, default `GPS`; comma-separated values from `GPS`, `BDS`, `GAL`, `GLO`
  - `enable_skymask`, `enable_map_occlusion`, `enable_nlos`, `enable_multipath`, `enable_fault_injection`
  - `enable_visualization`
  - `enable_csv_log`, `csv_log_path`

- Standalone usage:
  - Build first:
    ```bash
    colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm --packages-select gnss_comm gnss_sim
    source install/setup.bash
    ```
  - Run standalone with defaults:
    ```bash
    ros2 launch gnss_sim gnss_sim.launch.py
    ```
  - Run with a custom anchor:
    ```bash
    ros2 run gnss_sim gnss_sim_node --ros-args \
      -p truth_odom_topic:=/sim/drone_0/truth_odom \
      -p origin_lat_deg:=31.2304 \
      -p origin_lon_deg:=121.4737 \
      -p origin_alt_m:=25.0
    ```

- Demo4 usage:
  - `demo4.launch` now starts `gnss_sim_node` by default through `start_gnss_sim:=true`.
  - Disable GNSS sim without changing other demo4 nodes:
    ```bash
    ros2 launch iap demo4.launch start_gnss_sim:=false
    ```
  - Run demo4 with GNSS sim enabled and no RViz:
    ```bash
    ros2 launch iap demo4.launch start_rviz:=false start_gnss_sim:=true
    ```

## GNSS Sim v2 Requirements

### Goals

- Extend v1 from a nominal GPS L1 measurement source into a scenario-driven GNSS simulator for IAP fusion and ARAIM validation.
- Keep the five IAP-facing `/ublox_driver/*` topics backward compatible.
- Add optional simulator-only diagnostic and visualization topics under `/gnss_sim/*`; these topics must not be required by IAP.
- Make GNSS timing stricter so `clock_owner_mode=gnss` can be tested without avoidable timestamp drift.
- Support controlled degradation: SkyMask, map occlusion, LOS/NLOS classification, multipath, satellite drop, C/N0 degradation, and fault injection.
- Provide RViz visualization that makes satellite visibility and signal quality understandable during demo7 runs.

### v2.1 Timing and Configuration

- Add strict timestamp modes:
  - `time_source=odom_stamp`: current behavior, range epochs use the latest truth odom stamp.
  - `time_source=clock`: use `/clock` when simulation time is enabled.
  - `time_source=trigger_topic`: publish GNSS epochs on a trigger topic such as LiDAR frame stamp.
- Add optional truth-state buffering and interpolation:
  - Buffer `/sim/drone_0/truth_odom`.
  - Interpolate receiver pose and velocity at the exact GNSS epoch stamp.
  - Reject epochs outside the buffer instead of silently using stale truth.
- Expose constellation configuration:
  - `num_gps_sats`, default `24`.
  - `gps_prn_min`, default `1`.
  - `gps_prn_max`, default `24`.
  - `enabled_constellations`, default `["GPS"]`.
  - Future optional support for Galileo, BeiDou, and GLONASS.
- Add YAML scenario loading:
  - Anchor LLA.
  - Measurement rates.
  - Noise parameters.
  - Enabled satellites.
  - SkyMask file path.
  - Fault schedule.
  - Visualization scale and enabled layers.

### v2.2 Visibility, SkyMask, and Occlusion

- Add satellite visibility classification for every simulated satellite:
  - `LOS`: satellite is above elevation mask and not occluded.
  - `NLOS`: direct path is blocked but a configured reflected/diffracted path is available.
  - `OCCLUDED`: blocked and not used in measurement generation.
  - `DROPPED`: removed by configured drop model.
  - `FAULTED`: still visible but affected by injected measurement fault.
- Add SkyMask support:
  - Load azimuth/elevation mask from YAML or CSV.
  - Compare each satellite azimuth/elevation against the mask.
  - Allow dynamic masks for scenario playback in later versions.
- Add map occlusion support:
  - Optional subscription to `/map_generator/global_cloud` or `/map_generator/local_cloud`.
  - Optional static mesh/point-cloud map input from a file.
  - Raycast from receiver to satellite direction in the local ENU frame.
  - Mark blocked direct paths as `NLOS` or `OCCLUDED` depending on reflection settings.
- Add minimum implementation path:
  - First implement SkyMask-only visibility.
  - Then add point-cloud raycast.
  - Then add reflected/diffracted NLOS path generation.

### v2.3 NLOS, Multipath, and Measurement Quality

- Add NLOS measurement model:
  - Generate a longer effective path than the direct geometric range.
  - Add positive pseudorange bias for NLOS satellites.
  - Lower C/N0 and increase pseudorange/Doppler noise.
  - Keep NLOS satellites optionally usable so IAP and ARAIM can reject or downweight them.
- Add reflected/diffracted path model:
  - Represent NLOS path as receiver -> reflection/diffraction point -> satellite direction.
  - For point-cloud-only maps, approximate the reflection point by the first raycast hit.
  - For mesh maps, optionally use surface normals for specular reflection.
  - Use a display range scale because real satellites are thousands of kilometers away.
- Add multipath model:
  - Configurable sinusoidal, slowly varying, or random-walk pseudorange bias.
  - Parameters: `multipath_amp_m`, `multipath_period_s`, `multipath_tau_s`.
  - Apply stronger multipath at low elevation or near occluding structures.
- Add C/N0 model:
  - Elevation-dependent nominal C/N0.
  - NLOS and multipath degradation.
  - Fault-specific C/N0 degradation.
  - Use C/N0 to scale pseudorange and Doppler noise.

### v2.4 Fault Injection

- Add scheduled fault injection from YAML:
  - `fault_sat`: target satellite id or PRN.
  - `start_time_s` and `duration_s`.
  - `bias_m`: constant pseudorange bias.
  - `bias_rate_mps`: ramp fault.
  - `doppler_bias_mps`: Doppler fault.
  - `drop_sat`: remove one or more satellites from epochs.
  - `drop_rate`: probabilistic satellite dropout.
  - `cn0_degrade_dbhz`: signal quality degradation.
- Support multiple simultaneous faults:
  - Single-satellite fault.
  - Multi-satellite fault.
  - Orbit-plane or constellation-group fault.
- Add deterministic replay:
  - All random faults must use `random_seed`.
  - The same scenario file and seed must reproduce identical GNSS measurements.

### RViz Visualization Requirements

- Add a visualization publisher group controlled by `enable_visualization`, default `true` for demo7 and `false` for headless tests.
- Publish RViz topics under `/gnss_sim/visualization/*`:
  - `/gnss_sim/visualization/satellite_markers`, type `visualization_msgs/msg/MarkerArray`.
  - `/gnss_sim/visualization/signal_rays`, type `visualization_msgs/msg/MarkerArray`.
  - `/gnss_sim/visualization/nlos_paths`, type `visualization_msgs/msg/MarkerArray`.
  - `/gnss_sim/visualization/sky_dome`, type `visualization_msgs/msg/MarkerArray`.
  - `/gnss_sim/visualization/skyplot`, type `visualization_msgs/msg/MarkerArray`.
  - `/gnss_sim/visualization/status_text`, type `visualization_msgs/msg/MarkerArray`.
  - `/gnss_sim/visualization/occlusion_points`, type `visualization_msgs/msg/MarkerArray`.
- Satellite markers:
  - Draw one marker per simulated satellite in a scaled local ENU display frame.
  - Show PRN text labels with `TEXT_VIEW_FACING`.
  - Use a configurable display radius such as `satellite_display_radius_m=80.0` instead of true orbital distance.
- Signal ray colors:
  - `LOS`: green.
  - `NLOS`: orange or red.
  - `OCCLUDED`: dark gray.
  - `DROPPED`: transparent gray or dashed-style segmented line.
  - `FAULTED`: magenta.
  - `LOW_CN0`: yellow.
- Signal ray geometry:
  - LOS: draw a straight line from receiver to scaled satellite marker.
  - NLOS: draw a bent polyline from receiver to reflection/diffraction point, then toward the scaled satellite marker.
  - Multipath: draw one primary LOS/NLOS path plus a thinner secondary path with lower alpha.
  - Occlusion: draw a small sphere at the first raycast hit point.
- Skyplot visualization:
  - Draw a 2D polar skyplot in a fixed local frame near the UAV or in a fixed map corner.
  - Radius encodes zenith angle; angle encodes azimuth.
  - Draw current `min_elevation_deg` ring.
  - Draw SkyMask boundary if enabled.
  - Color each satellite by the same LOS/NLOS/fault convention.
- Receiver and anchor visualization:
  - Draw the current receiver position.
  - Draw the ENU anchor frame and label with `origin_lat_deg`, `origin_lon_deg`, `origin_alt_m`.
  - Optionally publish a TF frame such as `gnss_origin` if it does not conflict with existing frames.
- Measurement quality visualization:
  - Text panel showing:
    - GNSS epoch stamp.
    - GPS week and TOW.
    - Number of simulated satellites.
    - Number of used measurements.
    - LOS/NLOS/OCCLUDED/DROPPED/FAULTED counts.
    - PDOP or geometry score if available.
  - Optional per-satellite text showing PRN, elevation, azimuth, C/N0, pseudorange bias, and Doppler.
- Demo7 RViz integration:
  - Add GNSS visualization displays to the demo7 RViz config after the publishers exist.
  - Keep existing demo7 trajectory displays:
    - IAP estimated trajectory.
    - Desired path.
    - Truth trajectory.
  - Add GNSS visualization as optional layers that can be toggled in RViz.

### v2 Diagnostic Topics and Logs

- Publish `/gnss_sim/diagnostics`, type `diagnostic_msgs/msg/DiagnosticArray`, with:
  - Time source mode and latest epoch age.
  - Visible satellite count.
  - LOS/NLOS/OCCLUDED/DROPPED/FAULTED counts.
  - Stale odom or stale map warnings.
  - Active fault schedule entries.
- Publish optional CSV logs:
  - Satellite azimuth/elevation.
  - Visibility state.
  - C/N0.
  - Pseudorange truth, noise, NLOS bias, multipath bias, and final raw pseudorange.
  - Doppler truth, noise, bias, and final raw Doppler.
  - Raycast hit position and NLOS path length.

### v2 Acceptance Tests

- Timing:
  - With `time_source=trigger_topic`, GNSS epoch stamps align with LiDAR frame stamps within `config_gnss.time_tolerance`.
  - `clock_owner_mode=gnss` should not drift solely because of stale or mismatched GNSS timestamps.
- Visibility:
  - In an open-sky scenario, most satellites above `min_elevation_deg` are classified as `LOS`.
  - With a SkyMask blocking one azimuth sector, satellites in that sector are classified as `OCCLUDED` or `NLOS`.
- Visualization:
  - RViz shows satellite markers, signal rays, skyplot, and status text.
  - LOS and NLOS signals are visually distinguishable by color.
  - NLOS paths show a bent receiver-to-reflection-to-satellite path.
  - Faulted satellites are visually highlighted.
- Fault injection:
  - A scheduled single-satellite bias appears in CSV logs and visualization at the configured time.
  - IAP GNSS residuals increase for the affected satellite.
  - ARAIM/integrity output reacts to the injected fault when configured thresholds are sensitive enough.
- Regression:
  - With all v2 degradation disabled, measurement output remains compatible with the v1 nominal GNSS behavior.
  - `/ublox_driver/*` topic names and message types remain unchanged.

## GNSS Sim v2 Development Progress and Usage

### Current Development Status

- `[DONE]` v1 nominal GPS L1 GNSS simulator is implemented.
  - The implemented node publishes IAP-compatible `/ublox_driver/receiver_lla`, `/ublox_driver/ephem`, `/ublox_driver/iono_params`, and `/ublox_driver/range_meas`.
  - `/ublox_driver/glo_ephem` has a publisher handle, but GLONASS simulation is not implemented.
  - The current simulator uses truth odometry as the receiver state source and generates nominal GPS L1 pseudorange and Doppler measurements.
- `[DONE]` v2 GPS L1 update is implemented as of 2026-04-29.
  - Implemented features include strict time modes, truth odom interpolation, configurable GPS PRNs, YAML scenarios, SkyMask, point-cloud occlusion, NLOS, multipath, fault injection, RViz GNSS visualization, diagnostics, and optional GNSS simulator CSV logs.
  - The five IAP-facing `/ublox_driver/*` topics must remain backward compatible.
  - All v2-only diagnostics and visualization topics must be published under `/gnss_sim/*` and must not be required by IAP.

### v2 Planning Decisions

- v2 will be delivered in phases instead of as one large change.
- SkyMask and point-cloud raycast occlusion are both included in v2:
  - SkyMask is the deterministic baseline for visibility testing.
  - Point-cloud raycast is used to make demo7 urban/obstacle scenarios visible and testable.
- demo7 is the main integration and visualization entry point for v2.
- demo7 should default to GNSS visualization once the v2 visualization publishers exist.
- The first engineering priority is strict GNSS time synchronization:
  - Add trigger-based GNSS epochs.
  - Interpolate truth odometry at the target GNSS epoch stamp.
  - Reduce timestamp mismatch so `clock_owner_mode=gnss` can be tested without avoidable drift.
- v2 must not require IAP C++ source changes.

### v2 Feature Checklist

- `[DONE]` v1 nominal GPS L1 GNSS.
- `[DONE]` Strict time synchronization with `time_source=odom_stamp|trigger_topic|clock`.
- `[DONE]` Truth odometry buffer and interpolation at the GNSS epoch stamp.
- `[DONE]` Configurable GPS PRN count and PRN range.
- `[DONE]` Scenario YAML loading.
- `[DONE]` SkyMask visibility model.
- `[DONE]` Point-cloud raycast occlusion using `/map_generator/global_cloud` or `/map_generator/local_cloud`.
- `[DONE]` Per-satellite visibility states: `LOS`, `NLOS`, `OCCLUDED`, `DROPPED`, `FAULTED`.
- `[DONE]` NLOS pseudorange bias and lower C/N0.
- `[DONE]` Multipath pseudorange bias model.
- `[DONE]` Elevation-dependent and degradation-dependent C/N0 model.
- `[DONE]` Scheduled fault injection.
- `[DONE]` RViz satellite, signal ray, NLOS path, skyplot, and status visualization.
- `[DONE]` `/gnss_sim/diagnostics`.
- `[DONE]` GNSS simulator CSV debug logs.
- `[DONE]` demo7 RViz integration with GNSS visualization layers.

### Current v2/demo7 Usage

- Build the current implemented GNSS simulator and IAP demo packages:
  ```bash
  colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm src/iap --packages-select gnss_comm gnss_sim iap
  source install/setup.bash
  ```
- Run demo7 with the current v2 GNSS simulator:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=true start_gnss_sim:=true
  ```
- Run demo7 without starting IAP, useful for checking the simulator side first:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=true start_iap:=false start_gnss_sim:=true
  ```
- Disable GNSS simulator for regression comparison:
  ```bash
  ros2 launch iap demo7.launch start_gnss_sim:=false
  ```

### v2 Scenario Usage

- Run demo7 with a specific GNSS scenario:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true \
    gnss_time_source:=trigger_topic \
    gnss_scenario_file:=/home/dev/ws_iap/src/iap/config/gnss_sim/demo7_skymask_nlos.yaml
  ```
- Available scenario files:
  - `demo7_open_sky.yaml`: open-sky nominal GNSS.
  - `demo7_skymask_nlos.yaml`: SkyMask plus NLOS degradation.
  - `demo7_fault_injection.yaml`: single-satellite or multi-satellite fault injection.

### RViz Visualization Usage

- GNSS visualization topics:
  - `/gnss_sim/visualization/satellite_markers`
  - `/gnss_sim/visualization/signal_rays`
  - `/gnss_sim/visualization/nlos_paths`
  - `/gnss_sim/visualization/sky_dome`
  - `/gnss_sim/visualization/skyplot`
  - `/gnss_sim/visualization/status_text`
  - `/gnss_sim/visualization/occlusion_points`
- Color convention:
  - `LOS`: green.
  - `NLOS`: orange or red.
  - `OCCLUDED`: dark gray.
  - `DROPPED`: transparent gray.
  - `FAULTED`: magenta.
  - `LOW_CN0`: yellow.
- demo7 RViz must continue to show the three existing trajectory layers:
  - IAP estimated trajectory.
  - Desired path.
  - Truth trajectory.
- GNSS visualization must be added as optional RViz layers. Turning those layers off must not affect the GNSS measurement topics or the IAP fusion path.

### Current v1 Verification

- Confirm GNSS simulator topics:
  ```bash
  ros2 topic echo /ublox_driver/receiver_lla --once
  ros2 topic echo /ublox_driver/ephem --once
  ros2 topic echo /ublox_driver/iono_params --once
  ros2 topic echo /ublox_driver/range_meas --once
  ```
- Expected current behavior:
  - `/ublox_driver/receiver_lla` publishes continuously after truth odometry is received.
  - `/ublox_driver/ephem` publishes GPS ephemerides.
  - `/ublox_driver/iono_params` publishes ionosphere parameters.
  - `/ublox_driver/range_meas.meas.size()` is at least 4 in nominal open-sky operation.
  - IAP logs should show GNSS epoch insertion and GNSS factor injection when `libgnss_extension.so` is loaded.

### v2 Verification

- Timing:
  - With `time_source=trigger_topic`, GNSS epoch stamps must align with LiDAR frame stamps within `config_gnss.time_tolerance`.
  - `clock_owner_mode=gnss` should not drift because of stale or mismatched GNSS timestamps.
- Visibility:
  - A SkyMask that blocks a known azimuth sector should make satellites in that sector appear as `NLOS` or `OCCLUDED`.
  - Point-cloud raycast hits should produce an occlusion point in RViz.
- Visualization:
  - LOS and NLOS signals must be visually distinguishable in RViz.
  - NLOS signals must show a bent path from receiver to hit/reflection point and then toward the satellite marker.
  - Faulted satellites must be highlighted as `FAULTED`.
- Fault injection:
  - A scheduled fault must appear in GNSS simulator CSV logs at the configured time.
  - The affected PRN should show increased IAP pseudorange or Doppler residuals.
  - ARAIM or integrity output should react when thresholds and scenario geometry make the fault observable.
- CSV logs:
  - Planned CSV rows should include PRN, azimuth, elevation, visibility state, C/N0, pseudorange truth, noise, NLOS bias, multipath bias, final pseudorange, Doppler, and raycast hit position when available.

### Recommended v2 Implementation Order

1. Refactor the current single-file node into small internal modules while preserving v1 behavior.
2. Implement strict time synchronization and truth odometry interpolation.
3. Expose configurable GPS PRN count and range.
4. Add scenario YAML loading.
5. Implement SkyMask visibility.
6. Implement point-cloud raycast occlusion.
7. Add NLOS, multipath, C/N0 degradation, and fault injection.
8. Add RViz visualization publishers.
9. Add diagnostics and CSV logs.
10. Update demo7 launch and RViz config for the new v2 parameters and visualization layers.

## GNSS Sim v2 Implementation Record

### Completed on 2026-04-29

- `[DONE]` Implemented GNSS sim v2 in `gnss_sim_node`.
  - The implementation keeps one executable node and organizes the code internally into dedicated classes for time buffering, scenario loading, constellation management, visibility, measurement generation, visualization, and diagnostics.
  - IAP-facing `/ublox_driver/*` topic names and message types are unchanged.
- `[DONE]` Added strict time modes:
  - `odom_stamp`: v1-compatible latest-odom behavior.
  - `trigger_topic`: publish GNSS epochs from a trigger topic header stamp.
  - `clock`: publish fixed-rate epochs from ROS node time.
- `[DONE]` Added truth odometry buffering and interpolation for trigger/clock epoch stamps.
- `[DONE]` Added configurable GPS PRN parameters:
  - `num_gps_sats`
  - `gps_prn_min`
  - `gps_prn_max`
  - `enabled_constellations`
- `[DONE]` Added scenario YAML loading:
  - `config/gnss_sim/demo7_open_sky.yaml`
  - `config/gnss_sim/demo7_skymask_nlos.yaml`
  - `config/gnss_sim/demo7_fault_injection.yaml`
- `[DONE]` Added SkyMask and point-cloud raycast occlusion.
- `[DONE]` Added per-satellite visibility states:
  - `LOS`
  - `NLOS`
  - `OCCLUDED`
  - `DROPPED`
  - `FAULTED`
  - `LOW_CN0`
- `[DONE]` Added NLOS pseudorange bias, C/N0 degradation, multipath bias, and scheduled fault injection.
- `[DONE]` Added RViz visualization topics:
  - `/gnss_sim/visualization/satellite_markers`
  - `/gnss_sim/visualization/signal_rays`
  - `/gnss_sim/visualization/nlos_paths`
  - `/gnss_sim/visualization/sky_dome`
  - `/gnss_sim/visualization/skyplot`
  - `/gnss_sim/visualization/status_text`
  - `/gnss_sim/visualization/occlusion_points`
- `[DONE]` Added RViz sky dome and status legend:
  - Sky dome draws the local horizon ring, elevation rings, azimuth meridians, `N/E/S/W`, and `UP`.
  - Status text includes color legend: green `LOS`, orange `NLOS`, yellow `LOW_ELEVATION`, gray `BELOW/OCCLUDED`, magenta `FAULTED`.
- `[DONE]` Added Phase 2 GPS RINEX NAV input:
  - `ephemeris_source=synthetic|rinex`.
  - `rinex_nav_file`.
  - `rinex_ephem_max_age_s`.
  - `rinex_gps_only`.
  - `fallback_to_synthetic_on_rinex_error`.
  - RINEX ephemerides are loaded with `gnss_comm::rinex2ephems()`, filtered to GPS in Phase 2, selected by closest healthy TOE, and published through `/ublox_driver/ephem`.
  - Invalid or missing RINEX input can fall back to synthetic GPS while diagnostics report the fallback state.
- `[DONE]` Added Phase 3/4 RINEX-driven multi-constellation support:
  - `enabled_constellations_csv`, default `GPS`.
  - RINEX GPS, BeiDou, and Galileo ephemerides are published through `/ublox_driver/ephem`.
  - RINEX GLONASS ephemerides are published through `/ublox_driver/glo_ephem`.
  - Range measurements support `Gxx`, `Cxx`, `Exx`, and `Rxx` observations with per-system frequency/code and Earth rotation settings.
  - Synthetic mode remains GPS-only.
- `[DONE]` Added `/gnss_sim/diagnostics`.
- `[DONE]` Added optional simulator CSV logging through `enable_csv_log` and `csv_log_path`.
- `[DONE]` Updated `demo7.launch` with GNSS v2 parameters and defaults:
  - `gnss_time_source=trigger_topic`
  - `gnss_trigger_topic=/sim/drone_0/lidar`
  - `gnss_scenario_file=$(find-pkg-share iap)/config/gnss_sim/demo7_open_sky.yaml`
  - `gnss_enable_visualization=true`
  - `gnss_enable_map_occlusion=true`
  - `gnss_enable_sky_dome_visualization=true`
  - `gnss_sky_dome_show_cardinal_labels=true`
- `[DONE]` Added `config/sim_demo7/demo7_gnss.rviz`, based on demo6 RViz, with extra GNSS visualization layers.

### Current Recommended Commands

- Build:
  ```bash
  colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm src/iap --packages-select gnss_comm gnss_sim iap
  source install/setup.bash
  ```
- Run demo7 with GNSS v2:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=true start_gnss_sim:=true
  ```
- Run a SkyMask/NLOS scenario:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true \
    gnss_scenario_file:=/home/dev/ws_iap/src/iap/config/gnss_sim/demo7_skymask_nlos.yaml
  ```
- Run a fault-injection scenario:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true \
    gnss_scenario_file:=/home/dev/ws_iap/src/iap/config/gnss_sim/demo7_fault_injection.yaml
  ```
- Run demo7 with a GPS RINEX NAV file:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true \
    gnss_ephemeris_source:=rinex \
    gnss_rinex_nav_file:=/path/to/valid_rinex_3_04_nav.rnx
  ```
- Run demo7 with RINEX GPS, BeiDou, Galileo, and GLONASS:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true \
    gnss_ephemeris_source:=rinex \
    gnss_enabled_constellations:=GPS,BDS,GAL,GLO \
    gnss_rinex_nav_file:=/path/to/valid_rinex_3_04_nav.rnx
  ```
- Disable map occlusion while keeping other v2 features:
  ```bash
  ros2 launch iap demo7.launch \
    start_gnss_sim:=true \
    gnss_enable_map_occlusion:=false
  ```

### Verification Results

- Build verification passed:
  ```bash
  colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm src/iap --packages-select gnss_comm gnss_sim iap
  ```
- Demo7 simulator smoke test passed:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=false start_iap:=false backend_delay:=0.1 control_viz_start_delay:=20.0
  ```
  - `gnss_sim_node` loaded `demo7_open_sky.yaml`.
  - `/gnss_sim/diagnostics` reported `time_source=trigger_topic`, `simulated_sats=24`, map occlusion enabled, and map ready.
  - `/ublox_driver/range_meas` published GPS L1 measurements.
- Demo7 IAP integration smoke test passed:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=false start_iap:=true backend_delay:=0.1 iap_start_delay:=5.0 control_viz_start_delay:=20.0
  ```
  - IAP loaded `libgnss_extension.so`.
  - Logs showed `Klobuchar iono params received`.
  - Logs showed `GnssExtensionModule: ECEF origin set`.
  - Logs showed `epoch #1 inserted`.
  - Logs showed GNSS factor injection and residual diagnostics.
- RINEX missing-file fallback smoke test passed:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=false \
    start_iap:=false \
    gnss_ephemeris_source:=rinex \
    gnss_rinex_nav_file:=/tmp/missing.rnx
  ```
  - `/gnss_sim/diagnostics` reported `ephemeris_source=rinex`.
  - `/gnss_sim/diagnostics` reported `rinex_loaded=false`.
  - `/gnss_sim/diagnostics` reported `rinex_fallback_active=true`.
  - `/ublox_driver/range_meas` continued publishing synthetic GPS measurements.
- RINEX valid-file smoke test passed with the repository sample RINEX file:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=false \
    start_iap:=false \
    gnss_ephemeris_source:=rinex \
    gnss_rinex_nav_file:=/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx \
    gnss_rinex_ephem_max_age_s:=200000000.0
  ```
  - `/gnss_sim/diagnostics` reported `rinex_loaded=true`.
  - `/gnss_sim/diagnostics` reported `rinex_ephem_count=417`.
  - `/gnss_sim/diagnostics` reported `rinex_selected_ephem_count=31`.
  - `/ublox_driver/ephem` and `/ublox_driver/range_meas` published from selected RINEX GPS ephemerides.
  - The large max-age value was used only because the sample RINEX file is from 2022 while the current demo timestamps are 2026; matching-date RINEX files should use the default max-age.
  - This smoke test verifies the RINEX load/select/publish path, not final orbit numerical accuracy for a matching real-world date.
- RINEX GPS+BDS smoke test passed:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=false \
    start_iap:=false \
    gnss_ephemeris_source:=rinex \
    gnss_enabled_constellations:=GPS,BDS \
    gnss_rinex_nav_file:=/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx \
    gnss_rinex_ephem_max_age_s:=200000000.0
  ```
  - `/gnss_sim/diagnostics` reported `enabled_constellations=GPS,BDS`.
  - `/gnss_sim/diagnostics` reported `selected_gps_ephems=31`.
  - `/gnss_sim/diagnostics` reported `selected_bds_ephems=44`.
  - `/gnss_sim/diagnostics` reported `used_gps_obs=7`.
  - `/gnss_sim/diagnostics` reported `used_bds_obs=17`.
  - `/ublox_driver/range_meas` included GPS and BeiDou satellite ids.
- RINEX all-constellation smoke test passed:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=false \
    start_iap:=false \
    gnss_ephemeris_source:=rinex \
    gnss_enabled_constellations:=GPS,BDS,GAL,GLO \
    gnss_rinex_nav_file:=/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx \
    gnss_rinex_ephem_max_age_s:=200000000.0
  ```
  - `/gnss_sim/diagnostics` reported `selected_gps_ephems=31`.
  - `/gnss_sim/diagnostics` reported `selected_bds_ephems=44`.
  - `/gnss_sim/diagnostics` reported `selected_gal_ephems=22`.
  - `/gnss_sim/diagnostics` reported `selected_glo_ephems=22`.
  - `/gnss_sim/diagnostics` reported mixed used observations: GPS, BDS, GAL, and GLO.
  - `/ublox_driver/ephem` published GPS/BDS/GAL ephemerides and `/ublox_driver/glo_ephem` published GLONASS ephemerides.
- RINEX all-constellation IAP integration smoke test passed:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=false \
    start_iap:=true \
    gnss_ephemeris_source:=rinex \
    gnss_enabled_constellations:=GPS,BDS,GAL,GLO \
    gnss_rinex_nav_file:=/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx \
    gnss_rinex_ephem_max_age_s:=200000000.0
  ```
  - IAP logs showed `GnssExtensionModule: ECEF origin set`.
  - IAP logs showed `Klobuchar iono params received`.
  - IAP logs showed `epoch #1 inserted` with `n_sats=37`.

### Known Limitations

- v2 supports GPS L1 synthetic ephemerides and RINEX-driven GPS/BDS/GAL/GLO measurements.
- Synthetic BDS/GAL/GLO constellation generation is not implemented.
- RINEX numerical cross-check against an external propagation reference still requires a matching-date RINEX file and reference tool.
- Point-cloud occlusion uses voxel/raycast approximation, not mesh-based specular reflection.
- NLOS bent paths use the first raycast hit point as an approximate reflection/diffraction point.
- `/gnss_sim/visualization/occlusion_points` is a `MarkerArray` topic so occlusion hits can be drawn as colored RViz spheres.
- The implementation keeps v2 helper classes in one source file for now; they can be physically split into separate files later without changing ROS interfaces.

## GNSS Satellite Geometry and RViz Sky-Dome Correction Plan

### Problem Statement

- The current simulator has two different satellite representations:
  - `sat_pos_ecef`: satellite ECEF position used for pseudorange, Doppler, azimuth, elevation, and visibility logic.
  - `display_pos_enu`: scaled local ENU position used only for RViz visualization.
- The measurement layer uses synthetic GPS-like broadcast ephemerides:
  - GPS orbit semi-major axis is approximately `26,560,000 m`, close to real GPS orbit scale.
  - The constellation is deterministic and simulated; it is not real historical/current GPS broadcast ephemeris.
- The current RViz visualization scales satellites to a small radius around the UAV, for example `satellite_display_radius_m=80.0`.
  - This is useful for visualization, but it can look like satellites are distributed around the UAV on a complete sphere.
  - Real receiver visibility should be local-sky based: satellites below the local ENU horizon are not visible or usable.

### Correction Goals

- Make the separation between real-orbit-scale simulation data and RViz display data explicit in code, documentation, diagnostics, and RViz status text.
- Ensure satellites below the local horizon are not treated as normal visible GNSS signals.
- Change RViz display from a full sphere around the UAV to an upper-hemisphere sky dome.
- Preserve `/ublox_driver/*` compatibility and avoid changing IAP code.
- Keep synthetic GPS-like ephemerides as the current default; add real RINEX/broadcast ephemeris loading as a later enhancement.

### Implementation Steps

1. `[DONE 2026-04-29]` Add explicit code comments and variable naming around satellite geometry:
   - Document that `sat_pos_ecef` is the measurement-layer satellite position in ECEF meters.
   - Document that `display_pos_enu` is an RViz-only scaled ENU position.
   - Add a short comment near the `satellite_display_radius_m` parameter explaining that it is not orbital altitude.
2. `[DONE 2026-04-29]` Add RViz status text fields:
   - `display_radius_m=<satellite_display_radius_m>`.
   - `synthetic_orbit_radius_m≈26560000`.
   - `display=scaled_local_sky_dome`.
   - This prevents users from reading the RViz marker radius as physical satellite height.
3. `[DONE 2026-04-29]` Add a `BELOW_HORIZON` or equivalent internal visibility state:
   - If `elevation < 0 deg`, classify the satellite as below horizon before SkyMask, map occlusion, NLOS, multipath, or fault logic.
   - Below-horizon satellites must not enter `/ublox_driver/range_meas`.
   - Below-horizon satellites must not draw normal LOS/NLOS signal rays.
   - They may be omitted from RViz or shown in skyplot edge/gray style for debugging.
4. `[DONE 2026-04-29]` Refine low-elevation handling:
   - If `0 deg <= elevation < min_elevation_deg`, classify as `LOW_ELEVATION` or `OCCLUDED`.
   - These satellites should not be normal `LOS`.
   - They should not enter `/ublox_driver/range_meas` unless a future scenario explicitly enables low-elevation degraded measurements.
   - If shown in RViz, use gray/yellow styling and label them clearly.
5. `[DONE 2026-04-29]` Apply SkyMask, map occlusion, NLOS, multipath, and fault logic only after horizon/elevation gating:
   - `elevation < 0`: below horizon, stop.
   - `0 <= elevation < min_elevation_deg`: low elevation or occluded, stop by default.
   - `elevation >= min_elevation_deg`: continue to SkyMask and map occlusion.
6. `[DONE 2026-04-29]` Replace RViz display-sphere semantics with upper-hemisphere sky-dome semantics:
   - Compute display direction from azimuth/elevation:
     ```text
     display_dir_enu = [
       cos(elevation) * sin(azimuth),
       cos(elevation) * cos(azimuth),
       sin(elevation)
     ]
     display_pos_enu = receiver_pos_enu + satellite_display_radius_m * display_dir_enu
     ```
   - This puts high-elevation satellites near the top of the local sky dome.
   - This puts low-elevation satellites near the local horizon.
   - This avoids drawing usable satellites below the UAV ground plane.
7. `[DONE 2026-04-29]` Update RViz signal drawing rules:
   - `LOS`: draw straight green ray from receiver to sky-dome marker.
   - `NLOS`: draw orange/red bent path from receiver to raycast hit/reflection point, then toward sky-dome marker.
   - `OCCLUDED`/`LOW_ELEVATION`: do not draw a normal signal ray; optionally draw dim debug marker.
   - `BELOW_HORIZON`: do not draw signal ray; optionally show in skyplot only.
   - `FAULTED`: draw as magenta only if it is otherwise above horizon and usable.
8. `[DONE 2026-04-29]` Update diagnostics and CSV logs:
   - Add below-horizon count.
   - Add low-elevation count if implemented separately from `OCCLUDED`.
   - Keep azimuth/elevation in CSV so geometry can be audited.
9. `[DONE 2026-04-29]` Update documentation after implementation:
   - Record that RViz now uses a scaled sky dome.
   - Record that physical orbit scale is still used in the measurement layer.
   - Record that synthetic GPS-like ephemerides are not real broadcast ephemerides.

### Acceptance Criteria

- In RViz, usable GNSS satellites appear only on the receiver's upper local sky dome.
- No normal signal ray is drawn to satellites below the ENU horizon.
- `/ublox_driver/range_meas` contains only satellites that pass horizon and elevation gating.
- `/gnss_sim/diagnostics` reports counts for LOS, NLOS, OCCLUDED, DROPPED, FAULTED, and below-horizon or low-elevation satellites.
- RViz status text explicitly shows:
  - display radius.
  - synthetic GPS orbit radius.
  - scaled sky-dome display mode.
- IAP still receives compatible `/ublox_driver/*` messages and continues to insert GNSS epochs.

### Step 1-4 Verification Results

- Build passed:
  ```bash
  colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm src/iap --packages-select gnss_comm gnss_sim iap
  ```
- Demo7 GNSS smoke test passed:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=false start_iap:=false backend_delay:=0.1 control_viz_start_delay:=20.0
  ```
- `/gnss_sim/diagnostics` now reports:
  - `below_horizon`
  - `low_elevation`
  - `display_radius_m`
  - `synthetic_orbit_radius_m`
  - `display_mode=scaled_local_sky_dome`
- `/ublox_driver/range_meas` still publishes compatible GPS L1 measurements after horizon and low-elevation gating.

### Step 5-9 Verification Results

- Build passed:
  ```bash
  colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm src/iap --packages-select gnss_comm gnss_sim iap
  ```
- Demo7 GNSS smoke test passed:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=false start_iap:=false backend_delay:=0.1 control_viz_start_delay:=20.0
  ```
- `/gnss_sim/diagnostics` reported:
  - `time_source=trigger_topic`
  - `simulated_sats=24`
  - `los=5`
  - `nlos=1`
  - `below_horizon=15`
  - `low_elevation=3`
  - `display_mode=scaled_local_sky_dome`
  - `display_radius_m=80.000000`
  - `synthetic_orbit_radius_m=26560000.000000`
- Geometry sanity passed:
  - `5 + 1 + 0 + 0 + 0 + 15 + 3 + 0 = 24`, matching `simulated_sats=24`.
  - `/ublox_driver/range_meas` continued publishing only usable above-horizon/elevation-gated satellites.
- Demo7 IAP integration smoke test passed:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=false start_iap:=true backend_delay:=0.1 iap_start_delay:=5.0 control_viz_start_delay:=20.0
  ```
  - IAP logs showed `Klobuchar iono params received`.
  - IAP logs showed `GnssExtensionModule: ECEF origin set`.
  - IAP logs showed `epoch #1 inserted`.
  - IAP logs showed GNSS factor injection and residual diagnostics.

### Sky Dome and Legend Visualization Add-On

- `[DONE 2026-04-29]` Added `/gnss_sim/visualization/sky_dome`, type `visualization_msgs/msg/MarkerArray`.
- The sky dome visualization publishes:
  - Horizon ring at receiver height.
  - Configurable elevation rings.
  - Configurable azimuth meridians.
  - Optional `N/E/S/W` and `UP` labels.
- Added parameters:
  - `enable_sky_dome_visualization`, default `true`.
  - `sky_dome_show_cardinal_labels`, default `true`.
  - `sky_dome_ring_count`, default `3`.
  - `sky_dome_meridian_count`, default `12`.
- `demo7.launch` exposes matching launch arguments:
  - `gnss_enable_sky_dome_visualization`, default `true`.
  - `gnss_sky_dome_show_cardinal_labels`, default `true`.
  - `gnss_sky_dome_ring_count`, default `3`.
  - `gnss_sky_dome_meridian_count`, default `12`.
- The status text legend now shows:
  - green `LOS`.
  - orange `NLOS`.
  - yellow `LOW_ELEVATION`.
  - gray `BELOW/OCCLUDED`.
  - magenta `FAULTED`.
- `config/sim_demo7/demo7_gnss.rviz` includes a `6 GNSS Sky Dome` MarkerArray display for `/gnss_sim/visualization/sky_dome`.

### Future Real-Ephemeris Enhancement

- Add optional RINEX NAV or broadcast ephemeris input.
- Load real GPS ephemerides from file instead of using deterministic synthetic ephemerides.
- Propagate real PRN positions from the configured simulation time.
- Use the configured receiver anchor LLA to produce realistic visible PRN sets for a specific location and time.
- Keep the existing synthetic constellation as the default fallback for deterministic tests.

## GNSS Sim Next Roadmap: Satellite Parameters, RINEX NAV, and Multi-Constellation

### Current Baseline

- `[DONE]` The current simulator implements a GPS L1 synthetic constellation.
- `[DONE]` The default demo7 constellation remains 24 GPS satellites, PRN 1-24.
- `[DONE]` `gnss_sim_node` already supports `num_gps_sats`, `gps_prn_min`, and `gps_prn_max`.
- `[NEXT]` `demo7.launch` now exposes the same controls:
  - `gnss_num_gps_sats`, default `24`.
  - `gnss_gps_prn_min`, default `1`.
  - `gnss_gps_prn_max`, default `24`.
- `gnss_comm` already provides the common `GnssEphemMsg` path for GPS, Galileo, and BeiDou, plus `eph2pos()` / `eph2vel()` propagation support for those systems.
- GLONASS uses a separate `GnssGloEphemMsg` / `GloEphem` path, so it should be implemented after the GPS/Galileo/BeiDou path is stable.

### Phase 1: Demo7 Satellite Count Parameter Convergence

- `[NEXT]` Keep the default demo7 behavior unchanged:
  - `gnss_num_gps_sats=24`.
  - `gnss_gps_prn_min=1`.
  - `gnss_gps_prn_max=24`.
- Allow runtime experiments without editing source:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true \
    gnss_num_gps_sats:=32 \
    gnss_gps_prn_min:=1 \
    gnss_gps_prn_max:=32
  ```
- Validation:
  - `/gnss_sim/diagnostics` reports `simulated_sats=24` with defaults.
  - `/gnss_sim/diagnostics` reports `simulated_sats=32` with the custom launch arguments above.
  - `/ublox_driver/range_meas` continues to publish only usable above-horizon/elevation-gated satellites.

### Phase 2: Real RINEX NAV Input

- `[DONE 2026-04-29]` Add an ephemeris source switch:
  - `ephemeris_source=synthetic|rinex`, default `synthetic`.
  - `rinex_nav_file=""`.
  - `rinex_ephem_max_age_s=7200.0`.
  - `rinex_gps_only=true`.
  - `fallback_to_synthetic_on_rinex_error=true`.
- Use `gnss_comm::rinex2ephems()` to load real broadcast ephemerides from RINEX NAV files.
- Keep the synthetic constellation as the default fallback for deterministic tests and demos.
- Use the configured anchor LLA and GNSS epoch time to produce realistic PRN azimuth/elevation and visibility.
- Phase 2 is GPS-only RINEX NAV:
  - GPS ephemerides are published through `/ublox_driver/ephem`.
  - BeiDou, Galileo, and GLONASS remain later phases.
- demo7 launch arguments:
  - `gnss_ephemeris_source`, default `synthetic`.
  - `gnss_rinex_nav_file`, default empty.
  - `gnss_rinex_ephem_max_age_s`, default `7200.0`.
- Usage:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true \
    gnss_ephemeris_source:=rinex \
    gnss_rinex_nav_file:=/path/to/brdc_nav.rnx
  ```
- RINEX diagnostics:
  - `ephemeris_source`.
  - `rinex_nav_file`.
  - `rinex_loaded`.
  - `rinex_ephem_count`.
  - `rinex_selected_ephem_count`.
  - `rinex_fallback_active`.
  - `rinex_ephem_max_age_s`.
  - `rinex_error`.
- Fallback behavior:
  - Empty, unreadable, unsupported, or unusable RINEX files trigger a warning.
  - If `fallback_to_synthetic_on_rinex_error=true`, the node continues with synthetic GPS and reports `rinex_fallback_active=true`.
  - If fallback is disabled, range epochs are skipped until valid RINEX ephemerides are available.
- Validation:
  - For the same RINEX file, anchor, and epoch, PRN azimuth/elevation match an external RINEX propagation reference within a small tolerance.
  - RViz skyplot and sky dome show only satellites that are visible at the configured place and time.
  - IAP continues to receive compatible `/ublox_driver/ephem` and `/ublox_driver/range_meas` messages.

### Phase 3: BeiDou Support

- `[DONE 2026-04-29]` Extend constellation selection to support `GPS,BDS`.
- Publish BeiDou ephemerides through `/ublox_driver/ephem` using `gnss_comm::Ephem`.
- Generate BeiDou observations with:
  - `SYS_BDS` satellite ids.
  - BeiDou B1 frequency/code selection using `FREQ1_BDS` and `CODE_L2I`.
  - The same visibility, SkyMask, NLOS, multipath, fault, diagnostics, CSV, and RViz pipelines as GPS.
- Validation:
  - IAP logs can show inserted `Cxx` satellite observations.
  - `/ublox_driver/range_meas` contains both GPS and BeiDou observations when enabled.
  - GPS-only demo7 remains unchanged when `gnss_enabled_constellations:=GPS`.

### Phase 4: Galileo and GLONASS

- `[DONE 2026-04-29]` Add Galileo:
  - Use `SYS_GAL`.
  - Reuse `/ublox_driver/ephem`, `gnss_comm::Ephem`, and `eph2pos()` / `eph2vel()`.
  - Start with E1/L1-compatible single-frequency measurements using `FREQ1` and `CODE_L1C`.
- `[DONE 2026-04-29]` Add GLONASS:
  - Use `SYS_GLO`.
  - Publish `/ublox_driver/glo_ephem` with `gnss_comm::GloEphem`.
  - Handle GLONASS frequency/channel-specific L1 behavior using `FREQ1_GLO + freqo * DFRQ1_GLO`.
- Validation:
  - IAP caches each constellation's ephemerides and inserts GNSS factors without breaking GPS-only operation.
  - Mixed-constellation RViz colors/labels clearly distinguish `Gxx`, `Cxx`, `Exx`, and `Rxx` satellites.

## Demo7 Unified Simulation Epoch for RINEX-Reproducible Fusion

### Current Decision

- `[DONE 2026-04-30]` demo7 uses a fixed simulation epoch by default:
  - `sim_epoch_enabled=true`.
  - `sim_start_utc=2022-07-06T00:00:00Z`.
- The default epoch is intentionally aligned with the repository sample RINEX file:
  - `/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx`.
- The implementation uses source-level timestamp offsetting instead of global `/clock`:
  - Control, timers, and wall-rate simulation continue to run in real wall time.
  - Fusion-critical message stamps use `sim_start_utc + elapsed_wall_time`.
- This is demo7-specific. Other demos keep their existing wall-clock stamp behavior unless they explicitly pass the new simulator parameters.

### Implemented Parameters

- `so3_quadrotor_simulator`:
  - `sim_time/enable`, default `false`.
  - `sim_time/start_utc`, default empty.
- `demo7.launch`:
  - `sim_epoch_enabled`, default `true`.
  - `sim_start_utc`, default `2022-07-06T00:00:00Z`.
- `gnss_sim_node` diagnostics now include:
  - `gnss_epoch_utc`.
  - `rinex_selected_min_age_s`.
  - `rinex_selected_max_age_s`.
  - `rinex_time_consistent`.

### Timestamp Data Flow

- `so3_quadrotor_simulator` records wall start time when the node starts.
- When sim epoch is enabled, every truth odom, raw IMU, and IAP IMU publication uses:
  - `header.stamp = sim_start_utc + (node_now - wall_start)`.
- `local_sensing/pcl_render_node` already publishes LiDAR using the inherited truth odom stamp.
- `gnss_sim_node` in demo7 uses:
  - `time_source=trigger_topic`.
  - `trigger_topic=/sim/drone_0/lidar`.
- Therefore GNSS range epochs use the same stamp as the LiDAR frame that triggered them, while truth odom interpolation uses the same epoch-based time axis.
- IAP receives LiDAR, IMU, and GNSS timestamps in the same UTC epoch, so `config_gnss.time_tolerance=0.1` can match GNSS epochs without large RINEX age hacks.

### Usage

- Default demo7 with the fixed 2022 epoch:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true
  ```
- RINEX sample with GPS, BeiDou, Galileo, and GLONASS:
  ```bash
  ros2 launch iap demo7.launch \
    start_rviz:=true \
    start_gnss_sim:=true \
    gnss_ephemeris_source:=rinex \
    gnss_enabled_constellations:=GPS,BDS,GAL,GLO \
    gnss_rinex_nav_file:=/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx
  ```
- Use a different RINEX date by changing the simulation epoch:
  ```bash
  ros2 launch iap demo7.launch \
    sim_start_utc:=2026-04-30T00:00:00Z \
    gnss_ephemeris_source:=rinex \
    gnss_rinex_nav_file:=/path/to/matching_day_nav.rnx
  ```
- Disable fixed epoch and return to wall-clock message stamps:
  ```bash
  ros2 launch iap demo7.launch sim_epoch_enabled:=false
  ```

### Validation Checklist

- Build:
  ```bash
  colcon build --base-paths src/iap/sim/ego_planner_swarm_ws/src src/gnss_comm src/iap \
    --packages-select so3_quadrotor_simulator gnss_sim iap
  ```
- Timestamp smoke:
  ```bash
  ros2 launch iap demo7.launch start_rviz:=false start_iap:=false
  ```
  - `/sim/drone_0/truth_odom.header.stamp` starts near Unix `1657065600`.
  - `/sim/drone_0/imu_iap.header.stamp` matches truth odom within one odom period.
  - `/sim/drone_0/lidar.header.stamp` is inherited from truth odom.
- RINEX consistency smoke:
  - `/gnss_sim/diagnostics` reports `ephemeris_source=rinex`.
  - `rinex_time_consistent=true`.
  - `rinex_selected_max_age_s <= rinex_ephem_max_age_s`.
  - The old smoke-test workaround `gnss_rinex_ephem_max_age_s:=200000000.0` is not needed when the RINEX file date and `sim_start_utc` match.
- IAP fusion smoke:
  - IAP logs should show the first GNSS epoch delta to the latest LiDAR frame within `0.1s`.
  - Logs should still show `Klobuchar iono params received`, `epoch #... inserted`, and GNSS factor injection.
  - `/drone_0_visual_slam/odom` should continue publishing.

### Known Limits

- This pass does not introduce ROS `/clock` or global `use_sim_time`.
- Position command messages may still carry wall-clock stamps because they are not consumed by IAP as fusion sensor timestamps.
- If `sim_time/start_utc` cannot be parsed, `so3_quadrotor_simulator` logs a warning and falls back to wall-clock stamps.
- RINEX physical correctness still requires a NAV file whose date covers `sim_start_utc`; increasing `rinex_ephem_max_age_s` should be treated as a smoke-test-only override.

## Assumptions

- The stable truth topic is `/sim/drone_0/truth_odom`, type `nav_msgs/msg/Odometry`.
- The GNSS simulator still requires odometry input because Doppler generation needs velocity.
- The current implementation supports GPS synthetic mode and RINEX-driven GPS/BDS/GAL/GLO mode.
- Fault injection, map occlusion, SkyMask, NLOS, and multipath are implemented as GPS L1 v2 features.
