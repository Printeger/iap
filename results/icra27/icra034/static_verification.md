# ICRA-034 static verification

Requirements: IAP-RQ-320, IAP-RQ-321, IAP-RQ-322.

- The first focused test launcher,
  `python3 -m unittest test.test_gate0_analyzer.Gate0AnalyzerTest.test_message_stamp_unavailable_completed_failures_accept_full_startup_shape`,
  exited 1 because `test/` is not a Python package. It did not import or execute
  the test.
- The direct focused RED invocation,
  `python3 test/test_gate0_analyzer.py -k message_stamp_unavailable_completed_failures_accept_full_startup_shape`,
  exited 1 against the old analyzer with the expected
  `P0_EVIDENCE_CONTRACT_FAIL` instead of `PASS`.
- Focused GREEN invocations were rerun while implementing the enumerated typed
  fail-closed cases. They all passed before the full suite.
- Final static command:
  `python3 -m py_compile scripts/dev_planner/gate0_analyzer.py test/test_gate0_analyzer.py && python3 test/test_gate0_analyzer.py && git diff --check`
  exited 0; the complete direct analyzer suite passed 42/42.
- No engineering test invoked the analyzer CLI on ICRA-033 evidence or started
  GPU, ROS, launch, runner, capture, smoke, qualification or a build/install.
