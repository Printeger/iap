# ICRA-036 deterministic P4 collision-scan RED

Requirement: IAP-RQ-423. Result:
`P4_G0A_RED_READY_FOR_REVIEW`. This task adds only a test fixture, one focused
test target and compact evidence. Production optimizer header/source remain
byte-identical.

## Frozen fixture

The fixture contains 15 ordered samples at integer `(x, 0, 0)`, `x=0..14`,
with a deterministic 0.25 m task-local occupancy grid. There is no randomness,
wall-clock, ROS message, GPU, live map, P0 query or external file input.

| Case | Occupied indices | Expected status | Expected free endpoints |
|---|---|---|---|
| no collision | none | `NO_COLLISION` | none |
| one closed | 4, 5 | `CLOSED_SEGMENTS` | `(3,6)` |
| entry before old two-thirds, late exit | 8, 9, 10 | `CLOSED_SEGMENTS` | `(7,11)` |
| open ended | 7..14 | `OPEN_ENDED_COLLISION` | none |
| empty, non-finite, structural invalid, unavailable truth | n/a | `INVALID_INPUT` | none |
| multiple closed | 3, 4 and 7 | `CLOSED_SEGMENTS` | `(2,5)`, `(6,8)` |
| closed then open | 3, 4 and 7..14 | `OPEN_ENDED_COLLISION` | none consumable |

Fixture integrity proves every expected endpoint sample is free, every closed
segment contains a strictly interior occupied sample and multiple endpoints are
ordered/non-overlapping. The test-local legacy observer calls only existing
`BsplineOptimizer::initControlPoints()`. For valid input it translates an empty
legacy vector to `NO_COLLISION` and a nonempty vector to `CLOSED_SEGMENTS`; for
unsafe invalid inputs it exposes no result. It does not scan independently,
infer open/invalid status, synthesize endpoints or implement guide behavior.

## Builds and attempts

Fresh current IAP configure/build/install below ICRA-036 exited 0. The first
bspline configure exited 1 before compilation because `iap_DIR` incorrectly
named the ament `share/iap/cmake` directory, whose generated config looks for a
different export filename and an exported `iap_status` library. The corrected
consumer path is the project's installed custom config directory
`results/icra27/icra036/install/share/iap`; configure, full build and install
then exited 0 with ICRA-026 plan-env/path-searching read-only.

The focused target compiled and linked on its first build. RED attempt 1 ran 11
tests: three passed; seven intended missing-contract tests failed as expected;
the multiple-obstacle green case also failed because its first obstacle began
at the spline's unscannable prefix, making the observed start index occupied.
Only that fixture placement was corrected from `{2,3},{6,7}` to `{3,4},{7}`
with free expected endpoints `(2,5),(6,8)`. RED attempt 2 then produced the
intended stable 4-pass/7-fail split. `ament_uncrustify --reformat` was applied
only to the two new files, their focused style check passed, and RED attempt 3
reproduced the exact same 4-pass/7-fail split. No assertion was weakened.

The final intentional failures are:

- late free exit: observed `NO_COLLISION`/zero segments instead of
  `CLOSED_SEGMENTS`/`(7,11)`;
- open ended: observed `NO_COLLISION` instead of `OPEN_ENDED_COLLISION`;
- empty, non-finite, structurally invalid and unavailable occupancy: legacy
  exposes no safe explicit result instead of `INVALID_INPUT`;
- closed then open: observed `CLOSED_SEGMENTS` with one consumable partial
  segment instead of overall `OPEN_ENDED_COLLISION` with none.

Exact final names, statuses, reasons and hashes are in `test/red_result.json`.

## Green baseline and static boundary

The existing functional suites remain green when run separately:

```text
ctest --test-dir results/icra27/icra036/build_bspline -R '^test_p1_integrity_cost$' --output-on-failure
results/icra27/icra026/build_path_searching/test_p4_risk_astar
results/icra27/icra026/build_plan_env/test_grid_map_occupancy_epoch
```

They pass respectively 1 target / 39 tests, 4/4 and 6/6. The new fixture's four
green tests also pass when filtered separately. A package-wide linter attempt
is disclosed separately from this functional baseline: existing
`CMakeLists.txt` lines 46–65 fail `lint_cmake` on historical trailing whitespace;
`uncrustify` reports broad historical production/old-test divergence (the two
new files now pass their focused check); and `xmllint` timed out at the existing
60-second CTest limit. `test_p1_integrity_cost` and `cppcheck` passed within that
attempt. Forbidden historical/production formatting was not changed.

Ament and direct linkage resolve only the ICRA-036 IAP/bspline installs and
intended retained ICRA-026 plan-env/path-searching prefixes; no workspace
default, deleted ICRA-035, build-tree or missing product library appears.
Optimizer header/source SHA-256 remain exactly
`6c52f424248fafa7ace27bdd9a7500fb7933311826b447873ff0420023652656` and
`288d4cfb3a71306b87e994aead0df0621bcdab05fe4a161bbe8dedbfb4ad45d3`.
The protected PDF remains unstaged at
`1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
No GPU, ROS, live flow, runner/analyzer, smoke, benchmark, guide/risk/fallback/
threshold/lineage/P5 implementation, cleanup or Gate promotion occurred.
