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
  - It currently provides nominal GPS L1 simulation only.
  - It publishes `/ublox_driver/glo_ephem` as a ROS publisher handle, but v1 does not simulate or publish GLONASS ephemerides.
  - SkyMask, NLOS, map occlusion, multipath, C/N0 degradation, satellite drop, and fault injection are not implemented yet.

- Satellite count:
  - The simulated constellation is currently hardcoded as 24 GPS satellites, PRN 1-24.
  - The code uses `for (uint32_t prn = 1; prn <= 24; ++prn)` and maps each satellite with `gnss_comm::sat_no(SYS_GPS, prn)`.
  - The actual number published in each `/ublox_driver/range_meas` epoch is the visible subset after `min_elevation_deg` filtering, not always 24.
  - If fewer than 4 satellites are visible, the node logs a throttled warning and skips that epoch.
  - To change the total constellation size in the current version, edit the PRN loop upper bound in `ensure_ephemerides()`. A future cleanup can expose this as a ROS parameter such as `num_gps_sats`.

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
  - `/gnss_sim/visualization/skyplot`, type `visualization_msgs/msg/MarkerArray`.
  - `/gnss_sim/visualization/status_text`, type `visualization_msgs/msg/MarkerArray`.
  - Optional `/gnss_sim/visualization/occlusion_points`, type `sensor_msgs/msg/PointCloud2`.
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

## Assumptions

- The stable truth topic is `/sim/drone_0/truth_odom`, type `nav_msgs/msg/Odometry`.
- v1 only supports odometry input because Doppler needs velocity.
- v1 only simulates GPS L1. GLONASS publisher exists but does not publish ephemerides.
- Fault injection, map occlusion, SkyMask, NLOS, and multipath are v2 features.
