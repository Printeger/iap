# ICRA-042 verification summary

IAP-RQ-423. ICRA-042 registers a pre-data P4-G0C protocol; it does not execute
calibration or decide G0C. Canonical hashes are protocol
`496b2af570c0491ab4d35a84e32309608cc59a1784191842c5b055abb840617a`,
proposed registry
`77462979a0ac691a804dd0077b3b5da0dcf508c0eaa4551a884cc57645945784`
and live fixture
`985aabcd486186a4430305b409669422499f891d529369c6f0bfe8e7dfe0d710`.

The fresh product chain is rooted entirely below `results/icra27/icra042/`.
Configure each source in this order with the install prefixes recorded in
`preflight/build_identity.json`, always `-DBUILD_TESTING=ON` except the
task-local `quadrotor_msgs` bootstrap, then build `--target install -j2`:

```bash
bash results/icra27/icra042/preflight/task_env.bash cmake -S src/uav_simulator/Utils/quadrotor_msgs -B results/icra27/icra042/build_quadrotor_msgs -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra042/install_quadrotor_msgs"
bash results/icra27/icra042/preflight/task_env.bash cmake -S . -B results/icra27/icra042/build_iap -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON -DBUILD_WITH_CUDA=OFF -DBUILD_WITH_OPENCV=OFF -DBUILD_WITH_VIEWER=OFF -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra042/install_iap"
bash results/icra27/icra042/preflight/task_env.bash cmake -S src/iap/planner/plan_env -B results/icra27/icra042/build_plan_env -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra042/install_plan_env"
bash results/icra27/icra042/preflight/task_env.bash cmake -S src/iap/planner/path_searching -B results/icra27/icra042/build_path_searching -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra042/install_path_searching"
bash results/icra27/icra042/preflight/task_env.bash cmake -S src/iap/planner/bspline_opt -B results/icra27/icra042/build_bspline -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra042/install_bspline"
bash results/icra27/icra042/preflight/task_env.bash cmake -S src/iap/planner/plan_manage -B results/icra27/icra042/build_plan_manage -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra042/install_plan_manage"
# For each matching build_<product> directory, in the same environment:
bash results/icra27/icra042/preflight/task_env.bash cmake --build results/icra27/icra042/build_<product> --target install -j2
```

The task environment supplies exact task-local package prefixes and only the
unchanged workspace `traj_utils`/`gnss_comm` dependencies. Reproduce the
authorized non-live checks with:

```bash
python3 -m unittest discover -s test -p 'test_*.py'
python3 scripts/dev_planner/run_p4_g0c_calibration.py --plan-only --runs-root results/icra27/icra042/synthetic_plan_runs
bash results/icra27/icra042/preflight/task_env.bash results/icra27/icra042/build_bspline/test_p4_collision_guide
bash results/icra27/icra042/preflight/task_env.bash results/icra27/icra042/build_bspline/test_p4_collision_guide_integration
bash results/icra27/icra042/preflight/task_env.bash results/icra27/icra042/build_bspline/test_p4_collision_scan_contract
bash results/icra27/icra042/preflight/task_env.bash results/icra27/icra042/build_path_searching/test_p4_risk_astar
bash results/icra27/icra042/preflight/task_env.bash results/icra27/icra042/build_plan_env/test_grid_map_occupancy_epoch
bash results/icra27/icra042/preflight/task_env.bash ctest --test-dir results/icra27/icra042/build_plan_manage -L gtest --output-on-failure
```

Results are decision 15/15, integration 5/5, collision 17/17, path-searching
5/5, occupancy 6/6 and plan-manager 9/9 with 186 active cases plus one existing
disabled case. Python discovery is 376/376; final focused protocol, launch,
runner and analyzer tests are 21/21 and the launch golden suite is 16/16.
Task-environment linkage has zero missing, historical, workspace-default
IAP/planner or build-tree product resolutions. Source/install hashes match.

No GPU preflight, ROS, launch, calibration, smoke, benchmark, bag, RViz,
threshold freeze/application, G0D or P5 execution occurred. The proposed
registry retains four null gates and application disabled. Result:
`P4_G0C_PROTOCOL_READY_FOR_REVIEW`, not G0C PASS.
