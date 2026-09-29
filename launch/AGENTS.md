# Launch development rules

- The only canonical user entrypoints are `glio.launch.py`,
  `glio_integrity.launch.py`, `iap_sim.launch.py`, and `iap_flight.launch.py`.
- Do not add `demo12`, `demo13`, or another numbered root launch.
- Module launches start no environment, simulator, map generator, fake odom,
  truth adapter, or rosbag process.
- Simulation and real flight remain separate files. Never add a boolean that
  turns `iap_sim.launch.py` into a flight launch.
- `iap_flight.launch.py` must reject simulator/truth extensions and must remain
  fail-closed on missing deployment calibration.
- Current Integrity Monitor remains an `iap_rosnode` extension until a real
  independent process/interface exists.
- Advisory Integrity uses the maintained P0 path. Do not start or restore
  `phase2_planner_integrity_evaluator` in a canonical graph.
- Root `test_planner.launch.py` is a temporary compatibility link for
  validation infrastructure, not a canonical entrypoint. Maintained shared
  graph code belongs under `_includes/full_stack_runtime.py`.
- Add scenarios to `config/scenarios/catalog.json`; do not grow one launch
  argument per scenario property.
- Every run requires an explicit absolute output directory.
- Keep `bp/` until all four canonical launches have recorded integration/live
  evidence. Moving a file into `legacy/` or deleting it requires checking all
  scripts, tests, documentation, and frozen evidence references first.
