#!/usr/bin/env bash

set -eo pipefail

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/jazzy/setup.bash
source /home/dev/ws_iap/install/setup.bash
set -u

cd /home/dev/ws_iap
colcon --log-base /home/dev/ws_iap/log build \
  --paths /home/dev/ws_iap/src/iap \
    /home/dev/ws_iap/src/iap/src/iap/planner/plan_env \
    /home/dev/ws_iap/src/iap/src/iap/planner/traj_utils \
    /home/dev/ws_iap/src/iap/src/iap/planner/path_searching \
    /home/dev/ws_iap/src/iap/src/iap/planner/bspline_opt \
    /home/dev/ws_iap/src/iap/src/iap/planner/plan_manage \
  --packages-select iap plan_env traj_utils path_searching bspline_opt ego_planner \
  --build-base /home/dev/ws_iap/build \
  --install-base /home/dev/ws_iap/install \
  --symlink-install \
  --cmake-args -DBUILD_TESTING=ON
