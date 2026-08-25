# ICRA-059 Phase-A command ledger

All formal commands used cwd `/home/dev/ws_iap/src/iap` and the explicit task
root `/home/dev/ws_iap/src/iap/results/icra27/icra059/phase_a_tests`. The
launcher bound `HOME`, `ROS_HOME`, `ROS_LOG_DIR`, `TMPDIR` and mode-0700
`XDG_RUNTIME_DIR` below that root. Each command exited 0 with an empty external
ROS-log inventory delta.

```text
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root /home/dev/ws_iap/src/iap/results/icra27/icra059/phase_a_tests unittest discover -s test -p test_p4_g0c_*.py
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root /home/dev/ws_iap/src/iap/results/icra27/icra059/phase_a_tests unittest discover -s test -p test_*.py
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root /home/dev/ws_iap/src/iap/results/icra27/icra059/phase_a_tests syntax scripts/dev_planner/p4_g0c_protocol.py scripts/dev_planner/run_p4_g0c_calibration.py scripts/dev_planner/analyze_p4_g0c_calibration.py scripts/dev_planner/p4_g0c_surface_classifier.py scripts/dev_planner/run_p4_g0c_tests.py launch/test_planner.launch.py test/test_p4_g0c_protocol.py test/test_p4_g0c_runner.py test/test_p4_g0c_dependency_preflight.py test/test_p4_g0c_analyzer.py test/test_p4_g0c_surface_classifier.py test/test_p4_g0c_launch_contract.py test/test_p4_g0c_launch_environment.py test/test_p4_g0c_hermetic_tests.py
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root /home/dev/ws_iap/src/iap/results/icra27/icra059/phase_a_tests flake8 scripts/dev_planner/p4_g0c_protocol.py scripts/dev_planner/run_p4_g0c_calibration.py scripts/dev_planner/analyze_p4_g0c_calibration.py scripts/dev_planner/p4_g0c_surface_classifier.py scripts/dev_planner/run_p4_g0c_tests.py launch/test_planner.launch.py test/test_p4_g0c_protocol.py test/test_p4_g0c_runner.py test/test_p4_g0c_dependency_preflight.py test/test_p4_g0c_analyzer.py test/test_p4_g0c_surface_classifier.py test/test_p4_g0c_launch_contract.py test/test_p4_g0c_launch_environment.py test/test_p4_g0c_hermetic_tests.py
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root /home/dev/ws_iap/src/iap/results/icra27/icra059/phase_a_tests canonical-json config/icra27/p4_g0c_protocol_v4.json config/icra27/p4_threshold_registry_v4.json config/icra27/p4_g0c_runtime_dependencies_v4.json config/icra27/p4_g0c_replacement_lineage_v4.json
python3 scripts/dev_planner/run_p4_g0c_tests.py --task-root /home/dev/ws_iap/src/iap/results/icra27/icra059/phase_a_tests git-diff-check
```

Development RED was the missing `PROTOCOL_SCHEMA_V4`; subsequent focused REDs
identified the v4 XDG classifier multiset and frozen-v3 dependency fixture
seams. These were corrected before the final formal invocations above. No
development command invoked GPU, ROS, readiness, runner, analyzer or a
registered identity.
