# ICRA-057 executed-boundary ledger

All repository evidence is redacted. No inherited or retained full environment
is copied here, and no credential variable name or value is reproduced.

## Phase A — complete

All commands ran from `/home/dev/ws_iap/src/iap`; `$ROOT` below was the exact
absolute path `/home/dev/ws_iap/src/iap/results/icra27/icra057`.

```bash
ROOT=/home/dev/ws_iap/src/iap/results/icra27/icra057
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" unittest -- discover -s test -p 'test_p4_g0c_dependency_preflight.py'
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" unittest -- discover -s test -p 'test_p4_g0c_hermetic_tests.py'
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" unittest -- discover -s test -p 'test_p4_g0c_surface_classifier.py'
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" unittest -- discover -s test -p 'test_p4_g0c_*.py'
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" unittest -- discover -s test -p 'test_p4_g0c_launch_contract.py'
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" unittest -- discover -s test -p 'test_test_planner_launch.py'
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" unittest -- discover -s test -p 'test_*.py'
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" syntax -- scripts/dev_planner/run_p4_g0c_calibration.py scripts/dev_planner/run_p4_g0c_tests.py test/test_p4_g0c_dependency_preflight.py test/test_p4_g0c_hermetic_tests.py scripts/dev_planner/p4_g0c_surface_classifier.py test/test_p4_g0c_surface_classifier.py test/test_p4_g0c_launch_contract.py test/test_test_planner_launch.py
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" flake8 -- scripts/dev_planner/run_p4_g0c_calibration.py scripts/dev_planner/run_p4_g0c_tests.py test/test_p4_g0c_dependency_preflight.py test/test_p4_g0c_hermetic_tests.py scripts/dev_planner/p4_g0c_surface_classifier.py test/test_p4_g0c_surface_classifier.py test/test_p4_g0c_launch_contract.py test/test_test_planner_launch.py
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root "$ROOT" canonical-json -- results/icra27/icra057/compact/phase_a.json results/icra27/icra057/compact/results.json config/icra27/p4_g0c_protocol_v3.json config/icra27/p4_threshold_registry_v3.json config/icra27/p4_g0c_runtime_dependencies_v3.json config/icra27/p4_g0c_replacement_lineage_v3.json
git diff --check
```

Phase-A source/docs/compact were first committed and pushed as `9decf92` with
`IAP-RQ-423`; the final counts include the post-review fail-closed regression.

## Phase B — blocked before adoption decision

The exact read-only command whose scope triggered the incident was:

```bash
rg -n -m 20 '/usr/local/cuda/bin/nvcc|CMAKE_CUDA_COMPILER_VERSION|CUDA compiler identification|CUDA_VERSION' results/icra27/icra056/build/iap/CMakeFiles results/icra27/icra056/log
```

Its path list mistakenly included `results/icra27/icra056/log`. A historical
event record in that directory contains a serialized full build environment,
so the tool returned credential-like values. The values are intentionally not
repeated here.

Per `NEXT_TASK.md` output and credential rules, work stopped immediately as
`BLOCKED_CREDENTIAL_VALUE_OUTPUT_EXPOSURE`. The historical log was not changed,
deleted or hidden. No ICRA-057 preflight file or environment dump was written.

## Commands not invoked

- `colcon`: zero invocations; no ICRA-057 build/install exists.
- Standalone dependency runner: zero invocations; root absent.
- Full runner and built-in GPU preflight: zero invocations; runs root absent.
- Registered r3 identity: zero attempted, zero completed, zero retries.
- Analyzer and threshold action: zero invocations.
