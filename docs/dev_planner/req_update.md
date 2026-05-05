You need to harden demo9 before Phase 2.

Do not implement PI-lite or integrity-aware planning yet.

Fix these blockers:

1. Enforce phase1_closed_loop_logger packaging and build checks.
   - The logger package already exists at `sim/ego_planner_swarm_ws/src/iap_phase1_tools`.
   - `setup.py` already registers `phase1_closed_loop_logger = iap_phase1_tools.phase1_closed_loop_logger:main`.
   - The blocker is not that the package is missing from source; the blocker is that `tools/build_phase1_ego_planner_closed_loop.sh` silently skips desired packages that are not discovered by `colcon list`.
   - Split demo9 packages into required and optional groups.
   - Required packages must include at least `iap`, `iap_phase1_tools`, `ego_planner`, `traj_utils`, `so3_quadrotor_simulator`, `so3_control`, `poscmd_2_odom`, `odom_visualization`, `map_generator`, `local_sensing`, `gnss_sim`, and their required message/helper packages.
   - The build script must fail fast if any required package is missing.
   - After build, verify that `ros2 pkg executables iap_phase1_tools` contains `phase1_closed_loop_logger`; fail if it does not.

2. Decouple planner_use_dynamic from use_so3_dynamics.
   - Current demo9 reads `use_so3_dynamics` and then changes it with `use_so3_dynamics = use_so3_dynamics and planner_use_dynamic`.
   - This can make `use_so3_dynamics:=true` still launch the debug `poscmd_2_odom` plant if `planner_use_dynamic:=false`.
   - Remove: `use_so3_dynamics = use_so3_dynamics and planner_use_dynamic`.
   - use_so3_dynamics should only decide whether the plant is SO3 simulator or debug poscmd_2_odom.
   - Treat `planner_use_dynamic` as deprecated compatibility only; it may be accepted as a launch argument, but it must not control the plant.
   - If `planner_use_dynamic` is kept, print/log a clear compatibility warning when it is set.
   - Phase 1 official validation must use SO3 dynamics.

3. Clarify and harden demo9 waypoint defaults.
   - EGO's `point_num` counts `point0`; in demo9, `goal_x/y/z` is passed as `point0_x/y/z`.
   - The intended default route is a loop: start from the simulator init pose, visit the square waypoints, and return to the goal point when enough points are provided.
   - The current default `point_num:=7` is only valid if demo9 provides `point0..point6`.
   - Today demo9 only provides `point0..point5`, so the intent is unclear and may silently fall back to default/invalid `point6` values inside EGO.
   - EGO itself can hold more than 6 waypoints (`waypoints_[50][3]`), so the issue is the launch argument/parameter chain, not EGO's storage capacity.
   - `advanced_param.launch.py` currently exposes only `point0..point5`.
   - Either set `point_num:=6` and document that the default route ends at `point5`, or add explicit `point6_x/y/z` through demo9 and `advanced_param.launch.py`, set it equal to `goal_x/y/z`, and document that the default route returns to `point0`.
   - Prefer the second option if the desired default behavior is a closed loop back to the goal point; verify EGO receives all configured waypoints.

4. Fix GNSS default robustness.
   - Current demo9 defaults to `gnss_ephemeris_source:=rinex`.
   - The default RINEX path is likely invalid: `/home/dev/ws_iap/src/LIGO./Data/BRDM00DLR_S_20221870000_01D_MN.rnx`.
   - Current `gnss_fallback_to_synthetic_on_rinex_error:=false`, so default smoke tests can fail due to a local file path.
   - Prefer changing the default smoke-test mode to `gnss_ephemeris_source:=synthetic`.
   - If official validation requires RINEX, make the official command pass `gnss_ephemeris_source:=rinex` and an explicit `gnss_rinex_nav_file:=...`.
   - Avoid relying on silent fallback for official validation, because it makes the ephemeris source ambiguous.
   - If RINEX remains a supported mode, add a file-exists guard with a clear launch/runtime error.

5. Separate debug vs official validation.
   - allow_truth_alignment=true is acceptable only for debug.
   - Official Phase 1 validation should run with allow_truth_alignment=false.
   - `phase1_summary.json` already records `allow_truth_alignment`; keep this behavior.
   - Add enough runtime metadata for the validator to prove that official runs did not use truth feedback:
     - `use_so3_dynamics`
     - `use_iap_odom_for_planner`
     - actual `planner_odom_topic`
     - actual SO3 controller odom feedback topic
     - actual plant mode (`so3_quadrotor_simulator` or debug `poscmd_2_odom`)
   - Store these fields in `phase1_summary.json` and/or `topic_contract.json`.
   - Official validation must reject runs that used truth odom as planner or controller feedback.

6. Update validation.
   - Add an explicit `--official` mode to `tools/phase1/validate_phase1_closed_loop.py`.
   - In official mode, fail if `allow_truth_alignment != false`.
   - In official mode, fail if `use_so3_dynamics != true`.
   - In official mode, fail if `use_iap_odom_for_planner != true`.
   - In official mode, fail if the planner or SO3 controller odom feedback topic is `/sim/drone_0/truth_odom`.
   - Add `planner_cmd.csv` to the required file checks; current validation requires `desired_vs_truth.csv`, `planner_traj.csv`, `iap_sim_truth_vs_est.csv`, and `phase1_summary.json`, but misses `planner_cmd.csv`.
   - Required official files: `desired_vs_truth.csv`, `planner_traj.csv`, `planner_cmd.csv`, `phase1_summary.json`.
   - When GNSS+ARAIM are enabled, `iap_araim.csv` must also exist and be non-empty.
   - Keep movement, final-distance-to-goal, finite-value, and count checks.

7. Update README.
   - Add demo8 and demo9 sections.
   - README currently documents demo1..demo7; add demo8 and demo9 usage and validation notes.
   - Sync `docs/phase1_ego_planner_integration/topic_contract.md` with the actual waypoint chain. It currently documents fewer waypoint args than demo9 provides.
   - Document the official Phase 1 command:
     ros2 launch iap demo9_ego_planner_closed_loop.launch.py start_rviz:=false run_duration_s:=60 allow_truth_alignment:=false use_so3_dynamics:=true
   - Document the official validation command:
     python3 tools/phase1/validate_phase1_closed_loop.py --run-dir <latest_run_dir> --official

After changes, run:
bash tools/build_phase1_ego_planner_closed_loop.sh
ros2 launch iap demo9_ego_planner_closed_loop.launch.py start_rviz:=false run_duration_s:=60 allow_truth_alignment:=false use_so3_dynamics:=true
python3 tools/phase1/validate_phase1_closed_loop.py --run-dir <latest_run_dir> --official

Report:
- build result
- launch result
- generated files
- validation result
- whether planner/controller used IAP odom, not truth
- whether simulator movement and final distance-to-goal improved
