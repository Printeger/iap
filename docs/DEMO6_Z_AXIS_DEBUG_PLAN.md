# Demo6 Z-Axis Estimation Debug Plan

## Context

In `demo6`, the desired path, truth path, and IAP/control estimated path are visualized separately:

- Desired: `/demo6/desired/odom` and `/demo6/desired/path`
- Truth: `/sim/drone_0/truth_odom` and `/demo6/truth/path`
- IAP/control estimate: `/drone_0_visual_slam/odom` and `/demo6/drone/path`

When control uses truth odometry:

```bash
ros2 launch iap demo6.launch control_odom_topic:=/sim/drone_0/truth_odom
```

the simulator no longer loses height. This indicates that the SO3 controller and simulator dynamics are mostly healthy, and the remaining issue is likely in the IAP estimated `z` / `vz` chain.

## Goal

Find whether the height error comes from:

- the `demo4_lidar_body_bridge` frame conversion,
- an IMU/LiDAR frame or extrinsic mismatch,
- IMU acceleration semantics or bias,
- weak vertical observability in the simulated LiDAR geometry,
- or feedback instability when control closes the loop on IAP `z` / `vz`.

## Step 1: Confirm Which Trajectory Drops

Record these topics at the same time:

```bash
ros2 topic echo /demo6/desired/odom
ros2 topic echo /sim/drone_0/truth_odom
ros2 topic echo /drone_0_visual_slam/odom
```

Check:

- `pose.pose.position.z`
- `twist.twist.linear.z`

Expected outcome:

- If IAP `z` or `vz` diverges before truth loses height, the controller is following a bad estimate.
- If truth drops first while IAP remains stable, investigate control feedback timing or simulator dynamics.

### Result 2026-04-28

Run command:

```bash
ros2 launch iap demo6.launch start_rviz:=false
```

Sample captured after `iap_rosnode`, `poscmd_2_odom`, control, and all three visualization nodes were running:

| Source | Topic | Stamp | Frame | Child | x | y | z | vz |
| --- | --- | ---: | --- | --- | ---: | ---: | ---: | ---: |
| Desired | `/demo6/desired/odom` | `1777386732.855796` | `map` | empty | `-0.916512` | `0.400007` | `2.000000` | `0.000000` |
| Truth | `/sim/drone_0/truth_odom` | `1777386732.857514` | `map` | `drone_0` | `-0.897682` | `0.480198` | `0.802107` | `0.001763` |
| IAP estimate | `/drone_0_visual_slam/odom` | `1777386732.740515` | `map` | `imu` | `-0.902619` | `0.426776` | `2.031309` | `0.081397` |

Observation:

- Desired and IAP estimated height are close (`2.00 m` vs `2.03 m`), while truth is much lower (`0.80 m`).
- XY is comparatively close across all three sources.
- This supports the hypothesis that the controller is being fed an IAP estimate that says the vehicle is near the desired height, while the simulated truth vehicle is actually below it.
- The IAP estimated vertical velocity is positive (`0.081 m/s`) at this sample, while truth vertical velocity is near zero. This points to an estimator/control feedback mismatch in `z` / `vz`, not a desired trajectory issue.

## Step 2: Confirm Raw LiDAR Frame Semantics

Inspect the raw simulated LiDAR:

```bash
ros2 topic echo /sim/drone_0/lidar --once
```

Check:

- `header.frame_id`
- point coordinate ranges, especially `z`

Expected outcome:

- If `header.frame_id` is `map`, `/sim/drone_0/lidar` is a map/world-frame local cloud and should not be fed directly to IAP as a sensor-frame cloud.

### Result 2026-04-28

Sample:

```text
topic=/sim/drone_0/lidar
stamp=1777386756.240513
frame_id=map
width=6432
height=1
fields=[x, y, z]
points_read=6432
x[min,max,mean]=[-7.390, 8.810, 0.534]
y[min,max,mean]=[ 2.383, 9.685, 5.714]
z[min,max,mean]=[-1.940, 2.760, 0.099]
first_points=(0.115,2.621,1.043), (0.124,2.593,0.945), (0.103,2.658,1.122)
```

Observation:

- `header.frame_id` is `map`.
- Coordinates are map/world-frame local map coordinates, not LiDAR/body-frame coordinates.
- This confirms `/sim/drone_0/lidar` should not be fed directly to IAP as a sensor-frame scan except as an isolation experiment.

## Step 3: Confirm `lidar_body` Output Semantics

Inspect bridge output:

```bash
ros2 topic echo /sim/drone_0/lidar_body --once
```

Check:

- `header.frame_id`, expected `lidar`
- whether points are in a body/lidar-relative coordinate system
- whether the cloud changes with vehicle yaw/roll/pitch as a sensor-frame scan should

Expected outcome:

- `/sim/drone_0/lidar_body` should behave like an instantaneous sensor-frame point cloud, not a delayed or map-frame cloud.

### Result 2026-04-28

The first bridge log during this run was:

```text
first body cloud stamp=1777386681.070529 points=6544 frame_id=lidar input_frame=map
```

Sensor-data QoS sample:

```text
topic=/sim/drone_0/lidar_body
stamp=1777386756.070520
frame_id=lidar
width=6475
height=1
fields=[x, y, z]
points_read=6475
x[min,max,mean]=[ 2.372, 9.529, 5.541]
y[min,max,mean]=[-8.235, 7.995,-0.114]
z[min,max,mean]=[-3.812, 0.939,-1.740]
first_points=(2.534,0.725,-0.801), (2.506,0.719,-0.899), (2.572,0.734,-0.722)
```

Observation:

- `header.frame_id` is `lidar`, as intended.
- The coordinate ranges are clearly different from `/sim/drone_0/lidar`, which is consistent with the bridge converting map-frame points into a body/lidar-relative frame.
- The output is being published as a sensor stream with best-effort sensor QoS; a default reliable subscriber did not receive it. Use sensor-data QoS for future debug samplers.
- This step confirms the bridge is at least producing a transformed sensor-frame-like cloud. It does not yet prove the transform is mathematically correct or that the bridge's body frame is equivalent to IAP's `lidar` frame; that remains Step 4.

## Step 4: Verify Bridge Math

For one synchronized sample, capture:

- raw cloud: `/sim/drone_0/lidar`
- truth odom: `/sim/drone_0/truth_odom`
- bridge cloud: `/sim/drone_0/lidar_body`

Verify:

```text
p_body ~= T_body_map * p_map
```

Expected outcome:

- If this does not hold, fix `demo4_lidar_body_bridge`.
- If it holds, continue checking whether the bridge's body frame is truly equivalent to IAP's `lidar` frame.

### Result 2026-04-28

Run command:

```bash
ros2 launch iap demo6.launch start_rviz:=false
```

Synchronized sample:

```text
matched_stamp=1777386965.890471
raw_frame=map
body_frame=lidar
truth_odom_stamp=1777386965.871509
odom_dt=0.018962 s
raw_points=5473
body_points=5473
compared=5473
truth_pose_xyz=[-0.855282, 0.599275, 0.602239]
```

Verification used:

```text
p_pred_body = R_map_body^T * (p_map - t_map_body)
error = p_pred_body - p_lidar_body
```

Error summary:

```text
error_norm min  = 0.007339394 m
error_norm mean = 0.023449324 m
error_norm max  = 0.036360478 m
error_norm p95  = 0.035010160 m
```

First point examples:

```text
point0 raw=[-0.703494,-2.179275,0.942842]
       pred_body=[2.238287,1.642779,0.388251]
       actual_body=[2.240732,1.635809,0.386708]
       err=[-2.445e-03,6.970e-03,1.543e-03]

point1 raw=[-0.708983,-2.189898,0.844878]
       pred_body=[2.251410,1.645159,0.290465]
       actual_body=[2.253757,1.638257,0.288911]
       err=[-2.347e-03,6.901e-03,1.555e-03]

point2 raw=[-0.717878,-2.207117,0.748964]
       pred_body=[2.271892,1.648270,0.194831]
       actual_body=[2.274144,1.641413,0.193257]
       err=[-2.253e-03,6.857e-03,1.573e-03]
```

Observation:

- The bridge math is consistent with `p_body ~= T_body_map * p_map`.
- The remaining centimeter-level error is consistent with using the nearest truth odom sample, which was about `19 ms` away from the cloud stamp.
- This does not point to a gross sign, axis, or inverse-transform bug in `demo4_lidar_body_bridge`.
- The next suspicion is whether the bridge's body-frame output should truly be treated as IAP's `lidar` frame with identity `imu -> lidar`.

## Step 5: Confirm IAP Frame Assumptions

Read IAP startup logs for:

- `meta/imu_frame_id`
- `meta/lidar_frame_id`
- `meta/base_frame_id`
- `publish_imu2lidar`

Key question:

```text
Does IAP assume imu and lidar are the same frame?
```

Expected outcome:

- If bridge output is actually body/IMU frame but labeled `lidar`, either make that explicit or configure the correct `imu -> lidar` extrinsic.

### Result 2026-04-28

Relevant IAP startup logs:

```text
iap subscribed imu_topic=/sim/drone_0/imu_iap points_topic=/sim/drone_0/lidar_body
iap input first imu stamp=1777386917.716419 frame_id=imu
meta/imu_frame_id=imu
meta/base_frame_id=imu
iap input first points stamp=1777386917.730478 frame_id=lidar width=6544 height=1 fields=3
meta/lidar_frame_id=lidar
base_frame_id is not set. using IMU frame ID 'imu' as base frame ID.
```

Observation:

- IAP auto-detected IMU frame as `imu`.
- IAP auto-detected LiDAR frame as `lidar`.
- IAP base frame is `imu`.
- The configuration has `publish_imu2lidar=true`, and TF inspection in Step 6 shows the effective `imu -> lidar` transform is identity.
- Therefore, current IAP assumptions are:

```text
base frame = imu
lidar frame = lidar
T_imu_lidar = identity
```

Implication:

- This is only correct if `demo4_lidar_body_bridge` output is exactly in the IMU/body frame and if that frame's axes match IAP's `imu` frame convention.
- If the bridge output is body-like but not exactly IMU-frame-equivalent, z/roll/pitch estimation can be biased even though the bridge math in Step 4 is correct.

## Step 6: Inspect TF Tree

Generate or inspect the TF tree:

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo imu lidar
ros2 run tf2_ros tf2_echo map imu
```

Check:

- duplicate TF publishers,
- unexpected `imu -> lidar` rotation or translation,
- conflicts from multiple `odom_visualization` nodes,
- whether `map -> imu` from IAP is consistent with truth.

### Result 2026-04-28

TF publisher inspection:

```text
/tf publisher count: 4

publishers:
- /demo6_iap_rosnode
- /demo6_odom_visualization
- /demo6_truth_odom_visualization
- /demo6_desired_odom_visualization
```

`/tf_static`:

```text
WARNING: topic [/tf_static] does not appear to be published yet
```

Sampled dynamic TF edges:

```text
imu -> lidar
map -> odom
odom -> imu
```

`tf2_echo imu lidar`:

```text
Translation: [0.000, 0.000, 0.000]
Rotation quaternion xyzw: [0.000, 0.000, 0.000, 1.000]
Rotation RPY degree: [0.000, -0.000, 0.000]
```

`tf2_echo map imu` samples:

```text
t=1777386979.730500
Translation: [0.077, -0.982, 2.037]
RPY degree: [1.742, -0.927, 1.236]

t=1777386980.730527
Translation: [0.225, -0.968, 2.032]
RPY degree: [0.461, -0.540, 10.306]

t=1777386981.730497
Translation: [0.383, -0.931, 2.055]
RPY degree: [-0.584, -1.094, 19.290]
```

Observation:

- The effective `imu -> lidar` transform is identity.
- No `/tf_static` publisher was present during this run; the IMU/LiDAR relation is coming through dynamic TF.
- The sampled IAP TF tree is `map -> odom -> imu -> lidar`.
- There are four `/tf` publishers because each `odom_visualization` node also creates a TF broadcaster. In the sampled TF edges, only the IAP chain appeared, but the extra publishers are still worth watching for conflicts if RViz reports TF warnings.
- The IAP `map -> imu` z in this sample is around `2.03-2.06 m`, matching the IAP estimated odometry height rather than the lower truth height seen in Step 1.

Implication:

- Steps 5 and 6 agree: IAP is treating `lidar` as coincident with `imu`.
- Since Step 4 shows the bridge transform is mathematically reasonable, the next check should focus on whether the bridge's output axes are truly the same as IAP's IMU axes, and then on IMU acceleration semantics/bias in Steps 7-8.

## Step 7: Check IMU Semantics

Compare:

```bash
ros2 topic echo /sim/drone_0/imu --once
ros2 topic echo /sim/drone_0/imu_iap --once
```

Check while the vehicle is static:

- `linear_acceleration.x`
- `linear_acceleration.y`
- `linear_acceleration.z`

Expected outcome:

- IAP should receive the acceleration convention it expects. For specific force, static `z` should be close to gravity magnitude, with sign depending on the frame convention.
- If IAP receives world acceleration or gravity is handled with the wrong sign, vertical velocity will drift.

### Result 2026-04-28

Run command:

```bash
ros2 launch iap demo6.launch start_rviz:=false
```

Sample captured during the pre-control hold/static phase:

```text
raw: topic=/sim/drone_0/imu
     stamp=1777387271.022793
     frame=imu
     acc=[0.000000, 0.000000, 1.0788590662033708e+225]
     gyro=[0.000000, 0.000000, 0.000000]
     quat_xyzw=[0.000000, 0.000000, 0.000000, 1.000000]

iap: topic=/sim/drone_0/imu_iap
     stamp=1777387271.022793
     frame=imu
     acc=[0.000000, 0.000000, 9.810000]
     gyro=[0.000000, 0.000000, 0.000000]
     quat_xyzw=[0.000000, 0.000000, 0.000000, 1.000000]
```

Observation:

- `/sim/drone_0/imu_iap` is consistent with a static specific-force convention: `+9.81 m/s^2` on z.
- `/sim/drone_0/imu` is invalid during the hold phase; its z acceleration is an uninitialized huge value.
- This confirms it was correct to switch both IAP and SO3 control to `/sim/drone_0/imu_iap`.
- Static IMU semantics do not explain the remaining z mismatch by themselves. The next suspect is estimator initialization/bias and LiDAR/frame observability.

## Step 8: Inspect IAP Initialization

From IAP logs, inspect:

- `T_world_imu`
- `v_world_imu`
- `imu_bias`

Check:

- initial `z` offset relative to truth,
- nonzero initial `vz`,
- unusual accelerometer bias, especially `ba_z`.

Expected outcome:

- A significant initial `vz` or rapidly changing `ba_z` points to IMU initialization or acceleration semantics.

### Result 2026-04-28

From the truth-control isolation run:

```text
initial IMU state estimation result
T_world_imu=se3(0.003245,-0.004826,0.036311,-0.000604,-0.000379,-0.000078,1.000000)
v_world_imu=vec(-0.003111,0.004466,0.016618)
imu_bias=vec(-0.000331,0.000439,0.003532,-0.000115,-0.000084,-0.000000)

[sim_ext] initialized planner odom SE3 alignment
truth_stamp=1777387301.827011
est_stamp=1777387301.827011
dt=0.000000s
translation=[-0.003,0.005,-0.037]
rotation_deg=0.082
```

Observation:

- Initial pose is near the origin and has only a small z offset before `sim_ext` truth alignment.
- Initial vertical velocity is small but nonzero: `vz ~= 0.0166 m/s`.
- Initial accelerometer z bias is small: `ba_z ~= 0.0035`.
- First SE3 alignment is timestamp-exact and small, so the first alignment is not the source of the large height mismatch.
- The z mismatch appears after the system evolves, not as a large initial alignment error.

## Step 9: Run Isolation Experiments

### Experiment A: Truth Control, IAP Observe Only

```bash
ros2 launch iap demo6.launch control_odom_topic:=/sim/drone_0/truth_odom
```

Purpose:

- Confirm whether IAP `z` drifts even when it is not controlling the simulator.

### Experiment B: Compare LiDAR Inputs

Try IAP with:

- current `/sim/drone_0/lidar_body`
- temporary direct `/sim/drone_0/lidar`

Purpose:

- Determine whether z drift is caused by the bridge/frame conversion or by the estimator/IMU chain.

Note:

- Direct `/sim/drone_0/lidar` is expected to be semantically wrong if it is map-frame data, so use it only as an isolation experiment.

### Result 2026-04-28

#### Experiment A: Truth Control, IAP Observe Only

Run command:

```bash
ros2 launch iap demo6.launch start_rviz:=false control_odom_topic:=/sim/drone_0/truth_odom
```

Sample after takeoff/transition:

| Source | Topic | Stamp | x | y | z | vx | vy | vz |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| Desired | `/demo6/desired/odom` | `1777387330.893367` | `0.876635` | `0.000000` | `2.000000` | `0.230874` | `0.000000` | `0.000000` |
| Truth | `/sim/drone_0/truth_odom` | `1777387330.889967` | `0.880777` | `0.000033` | `1.999791` | `0.225564` | `-0.000012` | `0.000156` |
| IAP estimate | `/drone_0_visual_slam/odom` | `1777387330.656959` | `0.809620` | `-0.003940` | `2.106294` | `0.263159` | `-0.013163` | `-0.032112` |

Observation:

- With truth odometry driving the controller, truth tracks desired very closely in z (`1.9998 m` vs `2.0000 m`).
- IAP, while only observing, estimates `z=2.1063 m`, about `+0.1065 m` higher than truth at this sample.
- This shows the IAP z estimate has a modest positive bias even when it is not controlling the simulator.
- The severe truth height drop seen in closed-loop IAP control is therefore a feedback effect: a biased IAP z estimate is used by the controller, which can make the real simulated vehicle descend while IAP still believes it is near the desired height.

#### Experiment B: Direct Raw `/sim/drone_0/lidar`

Method:

- Started demo with `start_iap:=false` to avoid launching `demo4_lidar_body_bridge`.
- Started `iap_rosnode` manually with:

```bash
ros2 run iap iap_rosnode --ros-args \
  -p config_path:=/home/dev/ws_iap/install/iap/share/iap/config/sim_demo4 \
  -p imu_topic:=/sim/drone_0/imu_iap \
  -p points_topic:=/sim/drone_0/lidar
```

Relevant logs:

```text
iap input first imu stamp=1777387370.118768 frame_id=imu
meta/imu_frame_id=imu
meta/base_frame_id=imu
iap input first points stamp=1777387370.221844 frame_id=map width=6544 height=1 fields=3
meta/lidar_frame_id=map

initial IMU state estimation result
T_world_imu=se3(0.003264,-0.004852,0.036189,-0.000971,-0.000642,-0.000078,0.999999)
v_world_imu=vec(-0.004216,0.005767,0.022432)
imu_bias=vec(-0.000400,0.000497,0.004980,-0.000142,-0.000112,-0.000000)

[sim_ext] initialized planner odom SE3 alignment
truth_stamp=1777387371.381860
est_stamp=1777387371.381860
dt=0.000000s
```

Failure symptoms:

```text
large time difference between points and imu!!
points=1777387375.051816 imu=1777387376.051826 diff=1.000010
...
points=1777387375.051816 imu=1777387403.933826 diff=28.882010
```

Final sample:

```text
truth: topic=/sim/drone_0/truth_odom
       stamp=1777387403.835855
       frame=map
       child=drone_0
       x=-857.573132
       y=-366.808314
       z=632.184841
       vx=-24.933554
       vy=-11.537439
       vz=27.451541

iap: missing
```

Observation:

- IAP detected the raw cloud frame as `map`, confirming again that raw `/sim/drone_0/lidar` is not sensor-frame data.
- The direct raw-lidar setup became unstable: points stopped advancing while IMU continued, producing persistent time-difference warnings.
- Truth also flew away in this isolation setup because the normal demo control/IAP assumptions were broken.
- This experiment is not a valid operating mode, but it is useful diagnostically: direct `/sim/drone_0/lidar` is semantically wrong for IAP and cannot replace `/sim/drone_0/lidar_body`.

## Step 10: Decide the Fix

Use the observations to choose the fix:

- If bridge math is wrong: fix `demo4_lidar_body_bridge`.
- If bridge math is correct but frame labels are wrong: relabel output or configure explicit `imu -> lidar`.
- If frames are correct but z still drifts: inspect IMU acceleration semantics, IMU bias estimation, and LiDAR vertical observability.
- If only closed-loop IAP control drops height: improve control robustness to IAP `z` / `vz`, or temporarily use truth odom for demo stability.

### Result 2026-04-28

Current decision:

- Bridge math is not the primary issue. Step 4 showed `p_body ~= T_body_map * p_map` with centimeter-level residuals.
- Raw `/sim/drone_0/lidar` is not a valid replacement. It is map-frame data, and direct use caused IAP/time-sync instability.
- IAP's static IMU input is sane when using `/sim/drone_0/imu_iap`.
- Initial IAP state and first truth alignment are small and timestamp-consistent.
- The remaining issue is an IAP z estimate bias/drift that is modest when IAP only observes, but becomes severe when the controller closes the loop on IAP `z` / `vz`.

Recommended next fixes/investigations:

1. Keep `/sim/drone_0/lidar_body` and `/sim/drone_0/imu_iap`; do not feed raw `/sim/drone_0/lidar` directly.
2. Make the frame contract explicit:
   - either set `lidar_body_bridge` output frame to `imu` if the output is truly IMU/body-frame,
   - or explicitly configure/publish the correct `imu -> lidar` extrinsic instead of relying on identity.
3. Add a diagnostic CSV comparing truth and IAP over time:
   - `truth_z`, `iap_z`, `z_error`
   - `truth_vz`, `iap_vz`, `vz_error`
   - IAP roll/pitch and `ba_z`
4. For stable demo visualization/control, use `control_odom_topic:=/sim/drone_0/truth_odom` until IAP z feedback is fixed.
5. For IAP closed-loop validation, add controller-side protection against z estimate bias, such as limiting vertical correction from IAP `z/vz` or gating control until IAP/truth z error is within a threshold in simulation.
