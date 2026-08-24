# ICRA-050 executed commands

All commands ran from `/home/dev/ws_iap/src/iap`. Environment inventories and
full stdout/stderr are retained in `results/icra27/icra050/preflight/`.

## Fresh build — exit 0

The command ran under `env -i` with only the recorded ROS Jazzy base. The
effective colcon invocation was:

```bash
colcon --log-base results/icra27/icra050/log build \
  --base-paths \
    /home/dev/ws_iap/src/iap \
    /home/dev/ws_iap/src/iap/src/iap/planner/bspline_opt \
    /home/dev/ws_iap/src/iap/src/iap/planner/path_searching \
    /home/dev/ws_iap/src/iap/src/iap/planner/plan_env \
    /home/dev/ws_iap/src/iap/src/iap/planner/plan_manage \
    /home/dev/ws_iap/src/iap/src/iap/planner/traj_utils \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/cmake_utils \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/odom_visualization \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/pose_utils \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/quadrotor_msgs \
    /home/dev/ws_iap/src/iap/src/uav_simulator/Utils/uav_utils \
    /home/dev/ws_iap/src/iap/src/uav_simulator/fake_drone \
    /home/dev/ws_iap/src/iap/src/uav_simulator/gnss_sim \
    /home/dev/ws_iap/src/iap/src/uav_simulator/local_sensing \
    /home/dev/ws_iap/src/iap/src/uav_simulator/so3_control \
    /home/dev/ws_iap/src/iap/src/uav_simulator/so3_quadrotor_simulator \
    /home/dev/ws_iap/src/gnss_comm \
  --packages-select \
    iap bspline_opt path_searching plan_env ego_planner traj_utils \
    cmake_utils odom_visualization pose_utils quadrotor_msgs uav_utils \
    poscmd_2_odom gnss_sim local_sensing so3_control \
    so3_quadrotor_simulator gnss_comm \
  --build-base results/icra27/icra050/build \
  --install-base results/icra27/icra050/install \
  --merge-install \
  --executor sequential \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DBUILD_WITH_CUDA=OFF \
    -DBUILD_WITH_OPENCV=OFF \
    -DBUILD_WITH_VIEWER=OFF
```

## Standalone dependency preflight — exit 2

After sourcing `/opt/ros/jazzy/setup.bash` and the task merged
`local_setup.bash`, both prefix variables were explicitly set to the same
ordered two-prefix value.

```bash
AMENT_PREFIX_PATH="$PWD/results/icra27/icra050/install:/opt/ros/jazzy" \
P4_G0C_ALLOWED_PREFIXES="$PWD/results/icra27/icra050/install:/opt/ros/jazzy" \
python3 scripts/dev_planner/run_p4_g0c_calibration.py \
  --dependency-preflight-only \
  --runs-root "$PWD/results/icra27/icra050/dependency_preflight"
```

Typed result:
`DEPENDENCY_RUNTIME_LIBRARY_MISSING:iap:lib/libodometry_estimation_gpu.so`.

## Commands not invoked

- Full registered runner: not invoked; exit code N/A.
- GPU preflight: not invoked; exit code N/A.
- ROS/launch live matrix: not invoked; exit code N/A.
- Analyzer: not invoked; exit code N/A.
