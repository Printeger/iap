# ICRA-051 executed commands

All commands ran from `/home/dev/ws_iap/src/iap`. Full environment, command,
stdout/stderr and exit evidence remains below `results/icra27/icra051/`.

## Fresh CUDA build — exit 0

The build ran under `env -i` with the ROS Jazzy base, CUDA 12.5 compiler and
the exact 17-package source/selection inventory recorded in `preflight/`.

```bash
colcon --log-base results/icra27/icra051/log build \
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
  --build-base results/icra27/icra051/build \
  --install-base results/icra27/icra051/install \
  --merge-install --executor sequential \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    -DBUILD_WITH_CUDA=ON \
    -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
    -DBUILD_WITH_OPENCV=OFF \
    -DBUILD_WITH_VIEWER=OFF
```

## Static CUDA closure — PASS

Before any runner call, the fresh install was checked for exact package index,
six non-symlink ELF runtime libraries, hashes, complete `ldd` resolution and
dynamic loading of `libodometry_estimation_gpu.so`. The recorded check returned
`result=PASS`.

## Standalone dependency preflight — exit 0

After sourcing ROS Jazzy and the task-local merged install, both prefix
variables were explicitly reset to the same ordered two-prefix value.

```bash
AMENT_PREFIX_PATH="$PWD/results/icra27/icra051/install:/opt/ros/jazzy" \
P4_G0C_ALLOWED_PREFIXES="$PWD/results/icra27/icra051/install:/opt/ros/jazzy" \
PYTHONDONTWRITEBYTECODE=1 \
TMPDIR="$PWD/results/icra27/icra051/tmp" \
python3 scripts/dev_planner/run_p4_g0c_calibration.py \
  --dependency-preflight-only \
  --runs-root "$PWD/results/icra27/icra051/dependency_preflight"
```

## Full registered runner — exit 2

The full runner used the identical sanitized environment and prefixes. Its
built-in dependency and GPU gates passed, then the first registered launch
exited before either required process started.

```bash
AMENT_PREFIX_PATH="$PWD/results/icra27/icra051/install:/opt/ros/jazzy" \
P4_G0C_ALLOWED_PREFIXES="$PWD/results/icra27/icra051/install:/opt/ros/jazzy" \
PYTHONDONTWRITEBYTECODE=1 \
TMPDIR="$PWD/results/icra27/icra051/tmp" \
python3 scripts/dev_planner/run_p4_g0c_calibration.py \
  --runs-root "$PWD/results/icra27/icra051/runs"
```

Typed runner reason: `launch_exit_1`. The launch console records
`rcutils_expand_user failed` followed by `Failed to get logging directory`.
Exactly one ID was attempted, zero completed and no retry occurred.
Because `ROS_LOG_DIR` was not set repository-locally, launch also created
`/root/.ros/log/2026-08-24-14-34-21-049171-mint-X-965267/launch.log` outside the
authorized ICRA-051 root. This is retained as a Spec blocker, not repaired.

## Command not invoked

- Analyzer: not invoked; exit code N/A because runner state was not COMPLETE.
- No threshold action, G0D, P5 or formal campaign command was invoked.
