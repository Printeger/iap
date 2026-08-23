# ICRA-037 verification summary

Requirement: `IAP-RQ-423`

Gate: `P4_G0A`

Result: `P4_G0A_COLLISION_SCAN_GREEN_READY_FOR_REVIEW`

## Implementation boundary

One production `CollisionScanResult` carries exactly `NO_COLLISION`,
`CLOSED_SEGMENTS`, `OPEN_ENDED_COLLISION` or `INVALID_INPUT`, plus ordered
closed endpoint pairs. Both `initControlPoints()` and
`check_collision_and_rebound()` consume the same scanner. A run may enter only
inside the legacy trigger window, but an active run continues through the seed
tail. Multiple interpolated runs that map to overlapping control-point endpoint
pairs are merged, so returned segments remain ordered and non-overlapping.
Open-ended and invalid outcomes expose no segments and return before the
existing A*/guide path. The planner-manager initial caller returns failure for
those outcomes before candidate fanout or guide publication. One shared
`collisionScanFailsClosed()` predicate owns that propagation rule.

No original/risk guide generation, A* behavior, profile, scoring, selection,
fallback, lineage, P5 behavior, GPU, ROS/live flow, smoke, benchmark,
qualification or Gate promotion was added or run.

## TDD and builds

- Fresh IAP configure/build/install: exit 0.
- Fresh bspline configure: exit 0 against ICRA-037 IAP and retained read-only
  ICRA-026 plan-env/path-searching.
- RED compile attempt: exit 2 on the deliberately missing production scan
  result/status and test hooks.
- First GREEN focused compile: exit 0. Focused attempts progressed from 13/13
  to 14/14 after adding the closed-path integration assertion. Two-axis review
  then identified an untested overlap case; its focused test first failed 0/1
  with two duplicate pairs and passes after the merge rule. Final coverage is
  15/15.
- Final bspline rebuild/install: exit 0. Final selected CTest: 2/2 targets,
  comprising P1 39/39 and collision contract 15/15.
- Retained direct regressions: path-searching P4 4/4 and occupancy epoch 6/6.
- Corrected plan-manager build/install: exit 0. Final affected CTest: 9/9
  targets, zero failures.

The first plan-manager configure found workspace-default IAP typesupport and
reported a runtime-path cycle. No planner test was accepted from that linkage.
The same task-local tree was reconfigured with the exact ICRA-037 typesupport
file, rebuilt and reinstalled; all nine affected targets and the final direct
linkage audit then passed. `preflight/linkage.json` records the corrected
closure.

## Frozen contract and final artifacts

- Frozen fixture SHA-256 (unchanged):
  `49a676a5ff51538ab961c814409f6c2dfb7ba4679a861d4e8e94cc7d5679c788`.
- Protected PDF SHA-256 (unchanged and unstaged):
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Optimizer header:
  `5b751d5358095122ad5b959276074041aa397d297c9ff2f2e8f7668524817e9a`.
- Optimizer source:
  `0f032c38f01b2e93a434e0bf015471d178311c67738e3004399a18d928c192bb`.
- Focused test:
  `9291317425289475ac33618dc9cb56011fb589a8d0a6ee0112898ab91700d9a8`.
- Planner-manager source:
  `ef4bd0ecdc5029900a7bb33607a1418a7d167fca68c0e1dfae4a560883a4b5ac`.
- Focused executable:
  `8aa8b590e8773964a92f1e242e3133afaad5cf93ea2c351776d7aeddaf4c1c01`.
- Installed IAP library:
  `1dd661eee9887f3b0486058f4e16e18e0b6d5bdc07e9fec02e035bd8b33c2f92`.
- Installed bspline archive:
  `5158b23f82691ccf2511dacac9efb97cf8913af3644d3bbf70907d9438ef3ced`.
- Installed planner node:
  `11ea5afb4674755e6672e55c2e7fadb6ffee092a3fd01c4335946801900cd74b`.

`test/green_result.json` records the exact seven former RED outcomes and final
counts. Final XML is retained as `test/bspline_final.xml` and
`test/plan_manage_final.xml`; prior focused attempts and the direct retained
regressions remain in the same task tree. Build/install trees are retained for
Supervisor review and are not staged.

## Exact command ledger

All paths below are repository-local. Each listed final command exited 0;
counts are recorded above. The historical RED compile exited 2, and the
review-added overlap regression exited 1 before its production fix.

```bash
git status --short --branch
git fetch origin
git rev-list --left-right --count HEAD...origin/dev/icra
sha256sum src/iap/planner/bspline_opt/test/p4_collision_scan_fixture.hpp \
  docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf

cmake -S . -B results/icra27/icra037/build_iap \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra037/install"
cmake --build results/icra27/icra037/build_iap --parallel 2
cmake --install results/icra27/icra037/build_iap

cmake -S src/iap/planner/bspline_opt \
  -B results/icra27/icra037/build_bspline \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra037/install_bspline" \
  -Diap_DIR="$PWD/results/icra27/icra037/install/share/iap" \
  -Dpath_searching_DIR="$PWD/results/icra27/icra026/install_path_searching/share/path_searching/cmake" \
  -Dplan_env_DIR="$PWD/results/icra27/icra026/install_plan_env/share/plan_env/cmake"
cmake --build results/icra27/icra037/build_bspline --parallel 2
cmake --install results/icra27/icra037/build_bspline

cmake -S src/iap/planner/plan_manage \
  -B results/icra27/icra037/build_plan_manage \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DCMAKE_INSTALL_PREFIX="$PWD/results/icra27/icra037/install_plan_manage" \
  -Diap_DIR="$PWD/results/icra27/icra037/install/share/iap" \
  -Dbspline_opt_DIR="$PWD/results/icra27/icra037/install_bspline/share/bspline_opt/cmake" \
  -Dpath_searching_DIR="$PWD/results/icra27/icra026/install_path_searching/share/path_searching/cmake" \
  -Dplan_env_DIR="$PWD/results/icra27/icra026/install_plan_env/share/plan_env/cmake" \
  -DIAP_MSGS_TYPESUPPORT_CPP="$PWD/results/icra27/icra037/install/lib/libiap__rosidl_typesupport_cpp.so"
cmake --build results/icra27/icra037/build_plan_manage --parallel 2
cmake --install results/icra27/icra037/build_plan_manage

source /opt/ros/jazzy/setup.bash
export LD_LIBRARY_PATH="$PWD/results/icra27/icra037/install_plan_manage/lib:$PWD/results/icra27/icra037/install_bspline/lib:$PWD/results/icra27/icra037/install/lib:$PWD/results/icra27/icra026/install_path_searching/lib:$PWD/results/icra27/icra026/install_plan_env/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ctest --test-dir results/icra27/icra037/build_bspline --output-on-failure \
  -R '^(test_p1_integrity_cost|test_p4_collision_scan_contract)$'
results/icra27/icra026/build_path_searching/test_p4_risk_astar
results/icra27/icra026/build_plan_env/test_grid_map_occupancy_epoch
ctest --test-dir results/icra27/icra037/build_plan_manage --output-on-failure \
  -R '^(test_gate0_qualification_writer|test_p0_risk_grid_runtime|test_p0_occupancy_epoch_adapter|test_p1_replan_admission|test_p1_candidate_selection|test_p5_runtime_integrity_gate|test_p2_candidate_ranking|test_p3_reference_bias|test_planning_risk_context)$'

ament_uncrustify src/iap/planner/bspline_opt/test/test_p4_collision_scan_contract.cpp
git diff --check
python3 -m json.tool results/icra27/icra037/test/green_result.json
python3 -m json.tool results/icra27/icra037/preflight/linkage.json
```

The initial plan-manager configure omitted the explicit
`IAP_MSGS_TYPESUPPORT_CPP` line above. It exited 0 but was rejected because it
selected workspace-default IAP typesupport and emitted a runtime-path cycle;
no planner result was accepted before the corrected configure/build/install.
