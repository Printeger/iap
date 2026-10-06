# Current replacement regression

`20261006T033519Z_009_curve_hits.csv` contains all 3,600 map endpoints from
`export/planner/failure_map/curve_unobserved/current_frame_hits.csv`, v3
occupancy generation 43, current frame 89, stamp 1791257733.4700968.
Only the three map coordinate columns were retained; no points were added.
The test uses the saved sensor position, map origin, dimensions and resolution.
Subtracting the saved sensor position expresses these transformed endpoints
in an identity-oriented source frame, preserving the actual map rays.

This is the current hit input, **not** a complete observed mask or complete
beam set. The run has no bound complete beams. The prior frame in the removal
test and the explicit no-return beams in the upgrade test are synthetic and
labelled as such; neither establishes real LiDAR coverage of the hole.
