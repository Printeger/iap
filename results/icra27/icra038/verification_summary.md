# ICRA-038 verification summary

Requirement: `IAP-RQ-423`

Result: `P4_G0A_REBOUND_REPAIR_READY_FOR_REVIEW`

## Repair invariant

The shared scanner is unchanged. At the rebound consumption boundary, every
closed segment must contain at least one occupied integer control point before
the legacy direction/base-point filter can truthfully classify it as actionable
or already represented. If any segment lacks that evidence, rebound preserves
the scanner's complete `CLOSED_SEGMENTS` status/endpoints, sets the existing
error-stop state and returns before A*/guide work. A multi-segment result is
precluded from exposing or consuming an actionable partial subset.

The existing public test wrapper was only extended with an optional error-stop
output and deterministic normal precondition. No status, scanner geometry,
planner-manager, A*, guide, FSM, CMake or other product behavior changed.

## TDD and tests

- Fresh ICRA-038 bspline configure and focused compile: exit 0.
- Adjacent-endpoint RED: exit 1; scanner returned `(2,3)`, but rebound exposed
  `NO_COLLISION` and did not set error stop.
- Ordinary-then-adjacent multi-segment RED: exit 139/SIGSEGV after the ordinary
  partial subset reached the deliberately absent A* dependency.
- After the bounded production repair, the two new tests pass 2/2 and the full
  collision target passes 17/17.
- Final selected bspline CTest passes 2/2 targets: collision 17/17 and P1 39/39.
- Retained read-only direct tests pass: path-searching P4 4/4 and occupancy
  epoch 6/6.
- Fresh linked plan-manager CTest passes 9/9 targets, 186/186 active cases;
  one pre-existing profile test remains disabled.

## Build, linkage and identity

Fresh ICRA-038 bspline and plan-manager configure/build/install commands all
exit 0. Cache, link-line, installed-node and direct focused-test audits resolve
ICRA-038 bspline, ICRA-037 IAP/typesupport and the intended read-only ICRA-026
plan-env/path-searching only. Workspace-default IAP, ICRA-037 bspline and
missing product-library matches are zero. The installed node has no
non-toolchain RUNPATH and the installed optimizer header is byte-identical to
source.

All six ICRA-037 build/install tree identities are unchanged before/after:

| Tree | Files | Bytes | Manifest SHA-256 |
|---|---:|---:|---|
| `build_iap` | 1251 | 2461214970 | `29b225df85297e882e17a0ec25e61ffdcd9b3dc2b7af6700501bed1e938f6ad6` |
| `install` | 457 | 692115561 | `ee2c6682f95cbb528d23d5f3c196e2da2d9d1235010ab1466dea4c44e368f393` |
| `build_bspline` | 153 | 260651787 | `41d3407b124685257f9227a14018473dd3fe218c1c3b1e57e3a253ab55efdc93` |
| `install_bspline` | 25 | 54013614 | `c107f1b86a51400bb754f993d6d781266eb3c5c66bd28fa2b92ed726e19a88b9` |
| `build_plan_manage` | 320 | 1188819353 | `54cbd594861ef05ffd638b3ef46caca50e25a77fff066530fa558654008343ac` |
| `install_plan_manage` | 26 | 165097579 | `7cee1923301da3e49d1a9d1897559890ed40cec52409982eb0f7037e4726d10e` |

Final identities:

- Optimizer header: `61bd3f096644661413ed4ea4fa77cc6d4b6a8072ca0fcb4aebcb281a6a29fbd0`.
- Optimizer source: `bc2b559ba1d12c585a4432cc04a43cde11f18085919d36b3c8243850662c124e`.
- Focused test: `403e26fe95befe15873b44c0de693e55a37ac706d8f333fb0fd7204d0926cb35`.
- Frozen fixture: `49a676a5ff51538ab961c814409f6c2dfb7ba4679a861d4e8e94cc7d5679c788`.
- Protected PDF: `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Focused executable: `a624ad58a7ced9b9b3accb9a904ecee9bedc346c18a88c102b05ffb76e77e973`.
- Installed bspline archive: `a0b1cf2a5f1cb55574f43411323584fb6bf5cdc44cf52b3a38ff9e88514c909b`.
- Installed planner node: `9e0ba821b1868e70510407ae560a8479146dc61fb09a3b3c82698149c27e506b`.

## Exact command ledger

All final commands below exited 0 unless an explicit RED exit is shown.

```bash
git status --short --branch
git fetch origin
git rev-list --left-right --count HEAD...origin/dev/icra
sha256sum src/iap/planner/bspline_opt/test/p4_collision_scan_fixture.hpp \
  docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf

source /opt/ros/jazzy/setup.bash
source /home/dev/ws_iap/install/setup.bash
cmake -S src/iap/planner/bspline_opt \
  -B results/icra27/icra038/build_bspline \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra038/install_bspline" \
  -Diap_DIR="$PWD/results/icra27/icra037/install/share/iap" \
  -Dpath_searching_DIR="$PWD/results/icra27/icra026/install_path_searching/share/path_searching/cmake" \
  -Dplan_env_DIR="$PWD/results/icra27/icra026/install_plan_env/share/plan_env/cmake"
cmake --build results/icra27/icra038/build_bspline --parallel 2
cmake --install results/icra27/icra038/build_bspline

# RED exit 1: false error-stop and NO_COLLISION rewrite.
results/icra27/icra038/build_bspline/test_p4_collision_scan_contract \
  --gtest_filter='P4CollisionScanFailClosedIntegration.ReboundAdjacentEndpointsPreserveTruthAndStopBeforeAStar'
# RED exit 139/SIGSEGV: partial subset reached absent A*.
results/icra27/icra038/build_bspline/test_p4_collision_scan_contract \
  --gtest_filter='P4CollisionScanFailClosedIntegration.ReboundUnclassifiableSegmentRejectsEntireMultiSegmentResult'

cmake -S src/iap/planner/plan_manage \
  -B results/icra27/icra038/build_plan_manage \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra038/install_plan_manage" \
  -Diap_DIR="$PWD/results/icra27/icra037/install/share/iap" \
  -Dbspline_opt_DIR="$PWD/results/icra27/icra038/install_bspline/share/bspline_opt/cmake" \
  -Dpath_searching_DIR="$PWD/results/icra27/icra026/install_path_searching/share/path_searching/cmake" \
  -Dplan_env_DIR="$PWD/results/icra27/icra026/install_plan_env/share/plan_env/cmake" \
  -DIAP_MSGS_TYPESUPPORT_CPP="$PWD/results/icra27/icra037/install/lib/libiap__rosidl_typesupport_cpp.so"
cmake --build results/icra27/icra038/build_plan_manage --parallel 2
cmake --install results/icra27/icra038/build_plan_manage

export LD_LIBRARY_PATH="$PWD/results/icra27/icra038/install_plan_manage/lib:$PWD/results/icra27/icra038/install_bspline/lib:$PWD/results/icra27/icra037/install/lib:$PWD/results/icra27/icra026/install_path_searching/lib:$PWD/results/icra27/icra026/install_plan_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ctest --test-dir results/icra27/icra038/build_bspline --output-on-failure \
  -R '^(test_p1_integrity_cost|test_p4_collision_scan_contract)$'
results/icra27/icra026/build_path_searching/test_p4_risk_astar
results/icra27/icra026/build_plan_env/test_grid_map_occupancy_epoch
ctest --test-dir results/icra27/icra038/build_plan_manage --output-on-failure \
  -R '^(test_gate0_qualification_writer|test_p0_risk_grid_runtime|test_p0_occupancy_epoch_adapter|test_p1_replan_admission|test_p1_candidate_selection|test_p5_runtime_integrity_gate|test_p2_candidate_ranking|test_p3_reference_bias|test_planning_risk_context)$'

rg -n '^(iap_DIR|bspline_opt_DIR|path_searching_DIR|plan_env_DIR|IAP_MSGS_TYPESUPPORT_CPP):' \
  results/icra27/icra038/build_bspline/CMakeCache.txt \
  results/icra27/icra038/build_plan_manage/CMakeCache.txt
readelf -d results/icra27/icra038/install_plan_manage/lib/ego_planner/ego_planner_node
ldd results/icra27/icra038/install_plan_manage/lib/ego_planner/ego_planner_node
ldd results/icra27/icra038/build_bspline/test_p4_collision_scan_contract
cmp src/iap/planner/bspline_opt/include/bspline_opt/bspline_optimizer.h \
  results/icra27/icra038/install_bspline/include/bspline_opt/bspline_optimizer.h
```

No GPU, ROS, live map, launch, runner, analyzer, smoke, benchmark,
qualification, tuning, cleanup, dual-guide/G0B or P5 work ran. Build/install
trees are retained and only compact evidence will be staged.
