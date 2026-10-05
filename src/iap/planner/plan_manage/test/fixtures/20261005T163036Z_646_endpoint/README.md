# Captured v2 endpoint regression

Source: `log/20261005T163036Z_646/export/planner/failure_map/endpoint/`.
This is the near stationary run, separate from `20261005T162512Z_204`.

`snapshot.json` is copied verbatim. `cells.bin.z` is zlib compression of
`cells.bin` (7,392,000 bytes, SHA-256
`3a286290f6ad231986846fcd8304b17f9d2b842c83a9416c8d9d9445cc5dbb8c`).
`control_points.csv` contains the 16 `control_points_m` rows from that JSON.
The test reconstructs the exact raw, inflated and observed cell flags and uses
the saved motion/time values with `GridMap::queryPlanningCell`.
