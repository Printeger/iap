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
- Historical launch files exist only under `bp/`; do not restore root-level
  compatibility copies or links. Maintained shared graph code belongs under
  `_includes/full_stack_runtime.py`. The build may install `bp/` for frozen
  script compatibility, but new code must not invoke those launch files.
- Add scenarios to `config/scenarios/catalog.json`; do not grow one launch
  argument per scenario property.
- Canonical launches allocate a timestamped directory automatically below
  `IAP_RUN_ROOT` or the repository-local `log/runs`. An explicit `output_dir`
  remains an absolute-path override; never introduce a shared, non-versioned
  output directory that lets separate runs overwrite one another.
- Keep `bp/` until all four canonical launches have recorded integration/live
  evidence. Moving a file into `legacy/` or deleting it requires checking all
  scripts, tests, documentation, and frozen evidence references first.
