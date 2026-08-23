# ICRA-035 qualification evidence summary

Scope: IAP-RQ-320 / IAP-RQ-321 / IAP-RQ-322. This qualification changed no
source, header, test, analyzer, runner, capture, launch, configuration, CMake or
product file.

## Fresh build and tests

All commands exited 0 using the ordered task-local ICRA-035 IAP/EGO prefixes,
read-only ICRA-026 `plan_env`, `path_searching` and `bspline_opt`, the retained
workspace dependencies, and ROS Jazzy. Full ordered package/linkage evidence is
in `preflight/static_preflight.json`.

```text
cmake -S . -B results/icra27/icra035/build_iap -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/home/dev/ws_iap/src/iap/results/icra27/icra035/install
cmake --build results/icra27/icra035/build_iap -j2
cmake --install results/icra27/icra035/build_iap
cmake -S src/iap/planner/plan_manage -B results/icra27/icra035/build_ego -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=/home/dev/ws_iap/src/iap/results/icra27/icra035/install_ego -Diap_DIR=/home/dev/ws_iap/src/iap/results/icra27/icra035/install/share/iap -Dplan_env_DIR=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_plan_env/share/plan_env/cmake -Dpath_searching_DIR=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_path_searching/share/path_searching/cmake -Dbspline_opt_DIR=/home/dev/ws_iap/src/iap/results/icra27/icra026/install_bspline_opt/share/bspline_opt/cmake -U IAP_MSGS_TYPESUPPORT_CPP
cmake --build results/icra27/icra035/build_ego -j2
cmake --install results/icra27/icra035/build_ego
ctest --test-dir results/icra27/icra035/build_iap --output-on-failure -R '^(test_rolling_spatial_advisory_window|test_risk_grid_map|test_gate0_analyzer|test_gate0_runner|test_gate0_capture_p0_health|test_test_planner_launch)$'
ctest --test-dir results/icra27/icra035/build_ego --output-on-failure -R '^(test_p0_risk_grid_runtime|test_p0_occupancy_epoch_adapter)$'
```

The IAP selection passed 6/6: rolling 23, RiskGrid 43, analyzer 42, runner 27,
capture 1 and launch 16 tests. EGO passed 2/2: P0 runtime 79 tests (one existing
disabled profile not invoked) and occupancy adapter 7 tests.

## Disclosed pre-live corrections

1. One read-only `rg` lookup omitted `--` before a pattern beginning with
   `--benchmark`; it exited nonzero and was immediately corrected. It made no
   change.
2. The first EGO configure exited 0 but warned that its generated runtime path
   included both ICRA-035 IAP and the workspace-default IAP. Before any EGO
   build, the environment was narrowed and the inherited
   `IAP_MSGS_TYPESUPPORT_CPP` cache entry was removed with `-U`; the corrected
   configure/build/install and final linkage all passed.
3. The first task-local static helper passed the actual ament/linkage checks but
   supplied `preflight/dependencies` instead of the helper's expected
   `preflight` root, so its dependency-record path assertion exited 1. The
   retained failed record is `preflight/dependencies/launch_dependency_preflight.json`.
   The path argument and overly verbose linkage rendering were corrected before
   live execution; the final static preflight exited 0 with `ready=true`.
4. A post-live read-only inspection attempted `jq`, which is not installed. It
   exited 127; a read-only Python JSON inspection replaced it. No live command
   was repeated.
5. The first final JSON syntax sweep recursively included the runtime's
   JSON5/comment-bearing configuration copies and therefore raised
   `JSONDecodeError` on those non-strict-JSON inputs. The same shell continued
   to run `git diff --check` and the final affected CTest selections, which
   passed. The corrected syntax sweep is restricted to staged strict-JSON
   evidence; no runtime file was changed and no live command was repeated.

## Frozen preflight and one-shot result

Static preflight SHA-256 is
`08878746a4778ee6ca7b4c34913e10ba8487102b7fc17e26edd61dc2707b1244` and
effective-config SHA-256 is
`97b4ccb8bbb348ef285771e9d29f735188477b568ad53fb957dfeca612b211e5`.
It proves CPU, worker 4, 60/55 seconds, 30 x 30 x 6 m, 0.75 m, six 0.5-second
horizons through 2.5 seconds, 0.5-second refresh, occupied skip, no bag/RViz,
safety off, P1-P5 disabled and exact provisional `0.01` /
`legacy_iap_rq320_baseline_v1`. Five direct IAP links resolve only the ICRA-035
install and the sole plan-env link resolves ICRA-026. Ament resolves current
ICRA-035 IAP/EGO and intended ICRA-026 dependencies. Source and installed
runner/analyzer/capture/launch inputs match byte-for-byte.

The guarded runner command ran exactly once and exited 0:

```text
python3 scripts/dev_planner/run_gate0_qualification.py --output-root results/icra27/icra035/runs --benchmark
```

Its mandatory GPU preflight passed on one NVIDIA GeForce RTX 4070 Ti SUPER:
both `nvidia-smi` calls exited 0, `cuInit(0)=0`, device query returned 0 and
device count was 1. Dependency/config/log/capture readiness passed. Required
`iap_rosnode` was seen, had no runtime failure and stopped only during controlled
shutdown. Capture produced 209 health observations and 607 integrity reports.

The guarded analyzer command ran exactly once and exited 0/PASS:

```text
python3 scripts/dev_planner/gate0_analyzer.py --gate0-root results/icra27/icra035/runs --output-dir results/icra27/icra035/runs/benchmark/analyzer
```

Gate-0B has 105 completed attempts, 103 strict successful generations, two
typed completed failures, 18 in-progress observations, 86 equivalent completed
duplicates, zero conflicts and 607/607 valid integrity reports. Every success
has the exact 76,800-query shape. Refresh p50/p95/max is
175.482122/184.1007665/199.520467 ms; provider p50/p95 is
146.82252/150.8886328 ms; generation interval p50/p95 is
500.135382/511.2421743 ms. Refresh p95 / 500 ms is 0.368201533 and both failed
and stale ratios are 0.019047619. The acceptance threshold is 400 ms.

## Final boundary

No task process remains, no bag exists, and all 38 IAP runtime log files plus
17 ROS log files are task-local. External `log/` is byte-identical before and
after: manifest `a07fbf7945ec9800e95f6ef49d0d9c8bbdee8e2e8ff1500f919e1037cc4221f0`,
43,763 files and 15,834,674,845 bytes. The protected untracked PDF remains
unstaged and unchanged at
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No retry, smoke, campaign, P1-P5 execution, tuning, cleanup or Gate promotion
occurred. The exact sigma/profile remains provisional and is not empirical
calibration or full IAP-RQ-322 completion. Gate-0B PASS remains subject to
Supervisor review.
