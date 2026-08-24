# ICRA-043 verification summary

IAP-RQ-423 repairs the ICRA-042 G0C evidence protocol without executing a live
flow. The authoritative runner state is now `p4_g0c_runner_state_v2`: every
attempt is persisted before its executor, completion is recorded only after
artifact validation, and the analyzer requires the exact ordered 15-attempt
COMPLETE ledger with zero retry.

The analyzer rejects any extra root/run-like/retry directory, alternate G0C
manifest or P4 decision CSV, and requires at least one row in every registered
run. Runner and analyzer share the exact ordered 36-column production schema,
canonical typed identity, finite measurements, positive path lengths,
duplicate decision identity rejection, and path-ratio arithmetic. The
pre-data absolute ratio tolerance is frozen at `2e-5`, derived from six
significant-digit production serialization and the pre-existing 1.30 eligible
ratio cap. No threshold value changed.

Verification:

- red protocol/runner/analyzer tests each exited 1 against the old behavior;
- focused protocol 4/4, runner 11/11, analyzer 13/13, launch contract 6/6 and
  launch golden 16/16 passed (50/50), including review-remediation red tests;
- initial full discovery passed 384/384, first-remediation discovery passed
  388/388 and the final non-object-manifest remediation discovery passed
  389/389;
- Python syntax, fatal-only flake8, canonical JSON and `git diff --check`
  passed;
- all 3,829 files in the 12 retained ICRA-042 build/install trees have exact
  before/after manifest SHA-256
  `6836841bc7ee74594ff80926bfd67c8531ea2d26076b27406cb9aeea3d784d34`;
- protected fixture/PDF hashes remain exact.

Two-axis review initially found overwriteable preflight failure state,
alternate-name inventory bypass, uncaught finalization I/O and missing direct
attempt-list adversaries. Remediation makes any existing state/preflight/run
path fail before preflight, persists `PREFLIGHT_RUNNING` before preflight,
converts boundary errors into retained FAILED state, rejects every alternate
manifest/CSV name, and directly corrupts each authoritative attempt-list form
in tests. A syntactically valid non-object manifest also rejects as malformed
and persists FAILED. Decision identity extraction is centralized in the shared
schema.

Canonical SHA-256 values are protocol
`9e89ea42675459a63853d98845f02b7fe5b9434a9f28fcbd6ef5ba1bc5bd906d`,
registry
`1a9e206c12133035b29dd4ff573cf3868cf4765f3b9213362e507d85c24deaff`
and launch
`26f914f749758745b9c031819df0e969def46bd7fd15bb3caac831921df2dd65`.

Reproduce only the authorized synthetic Python checks:

```bash
python3 test/test_p4_g0c_protocol.py
python3 test/test_p4_g0c_runner.py
python3 test/test_p4_g0c_analyzer.py
python3 test/test_p4_g0c_launch_contract.py
python3 test/test_test_planner_launch.py
python3 -m unittest discover -s test -p 'test_*.py'
```

No GPU preflight, ROS, launch, calibration, CTest/retained binary, smoke,
benchmark, threshold draft/freeze/application, G0D or P5 ran. This evidence is
`P4_G0C_PROTOCOL_REPAIR_READY_FOR_REVIEW`, not G0C PASS.
