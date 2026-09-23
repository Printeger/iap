# Observation execution closure blocked checkpoint (2026-09-23)

This development checkpoint records the single permitted compact live run. It
does not claim observation execution closure, qualification, or scientific
effect.

- Command: `python3 scripts/dev_planner/run_icra_interface_integration.py --stage full --repetitions 1`
- Scenario/mode: `icra072_p4_selection_trigger_v1`, BDS,
  `mission_best_effort`, one 75-second repetition
- Result: `FAIL`
- Local ignored evidence root:
  `results/icra27/dev_runs/interface_integration/run-20260923T155754Z-1090476`
- Last successful required state: P0 passed with healthy generations 1 through
  31 (61 windows; first healthy delay 13.785 s). The route worker enumerated
  two channels.
- First failed required state: no valid closed-collision trajectory lineage was
  established (`lineage_trajectory_identity_missing_or_invalid`). The earliest
  explicit planner error was `P4-v2 final lineage write failed before P5` at
  planner timestamp 1790179088.838306146. After P0 became healthy, planning
  repeatedly deferred with `OBSERVATION_UNAVAILABLE_SENSOR_GEOMETRY`.
- Trajectory identity: none was published or activated. `bspline_count=0`,
  `published_group_count=0`, and all 6255 captured PositionCommand samples had
  trajectory ID 0, execution instance 0, and an empty curve hash.
- Observation certificate: none was produced; no certified observation segment
  executed.
- P5 reason/status: P5 was not reached for a publishable observation;
  `p5_final_ok_count=0` and `p5_runtime_committed_count=0`.
- Evidence improvement: none. No new observation was executed and no formal P4
  selection occurred (`formal_v5_selection_count=0`, `selected_count=0`).

Per the hard-stop rule, this live was not rerun and production code was not
changed after it failed. Raw capture, stdout, exported CSV files, and other run
artifacts remain local and ignored; they are not part of this commit.
