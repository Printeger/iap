# Pre-refactor launch backup

This directory is an exact transition copy of the root launch entrypoints that
existed before the canonical launch refactor. Do not develop new behavior here.

The source-root compatibility entrypoints are intentionally retained for now
because tests, experiment scripts, and frozen qualification manifests still
reference their exact historical paths. `test_planner.launch.py` is now only a
compatibility link to the private maintained runtime. After all four canonical
launches pass their runtime contracts, those references can be migrated and the
remaining root files can move to `legacy/` or be removed with this backup.
