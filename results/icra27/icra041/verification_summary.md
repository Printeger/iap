# ICRA-041 clean-room P4-G0B requalification

Requirement: `IAP-RQ-423`

Builder result: `P4_G0B_CLEAN_REQUALIFICATION_READY_FOR_REVIEW`

## Product identity and dependency boundary

Reviewed product source is unchanged from `57ea9263`; authorization HEAD
`8f75dabc` changes only Supervisor-owned control documents. A sanitized wrapper
unsets inherited build/runtime prefixes, sources ROS Jazzy, then admits only
fresh ICRA-041 products and immutable workspace `traj_utils`/`gnss_comm`.
Because `quadrotor_msgs` is required by unchanged CMake but absent from Jazzy,
it is freshly bootstrapped below ICRA-041 from unchanged repository source;
the workspace-default product is not consumed.

Fresh configure/build/install exits are zero for quadrotor messages, IAP,
plan-env, path-searching, bspline-opt and plan-manager. CMake caches bind every
IAP/planner dependency to ICRA-041. Direct dynamic resolution across all
relevant tests and the installed node has zero missing, historical,
workspace-default product or build-tree resolutions. Installed node RUNPATH is
empty. Five relevant installed headers are byte-identical to source.

## Deterministic matrix

Every prescribed command ran once with no retry. Exit zero and exact results:

- focused identity precedence 3/3, then decision 15/15;
- focused non-G0B false boundary 1/1, then integration 5/5;
- collision contract 17/17 and P1 integrity cost 39/39;
- fresh path-searching P4 5/5 and fresh occupancy epoch 6/6;
- fresh plan-manager CTest 9/9, comprising 186 active cases, one existing
  disabled case and zero failures.

The production `p4_collision_guide_v1` run repeats request/original/risk/
selected hashes `1c8abe0fa4e4136a` / `2a3380ee05f43a1f` /
`b3789ad7a8e50365` / `2a3380ee05f43a1f`. Both profiles are 200/200 valid;
original mean/max is `2.0295422607088973/10.500000000000002`, risk mean/max is
`1/1.0000000000000002`, ratio is `1.0`, original remains selected and no risk
guide is applied. The separate false-boundary case preserves
`metrics_only=false`, reports `SELECTION_NOT_AUTHORIZED`, selects original and
keeps `selection_applied=false`.

## Retained provenance

Before configure and after all tests, the same sorted manifest command hashed
every regular file and symlink in all 14 retained ICRA-039/040 build/install
trees. Both 3,124-line, 528,446-byte manifests have canonical SHA-256
`d18c1c89ef585ef42a31eb9b1f944c8eecbe7d6f1da98ecf567e3816357e3162` and
`cmp` exited zero. This proves no further write by ICRA-041; it does not claim
to repair or reinterpret the historical ICRA-040 incident. Full manifests
remain task-local and unstaged; compact schema/hash/comparison evidence is in
`preflight/retained_manifest.json`.

## Reproduction and limitations

Configure/build commands and exact options are recorded in the ICRA-041 START
entry. The literal executable paths, filter strings, XML paths and manifest
generator are recorded in the ICRA-041 REVIEW REMEDIATION entry. Plan-manager
uses `bash results/icra27/icra041/preflight/task_env.bash ctest --test-dir
results/icra27/icra041/build_plan_manage -L gtest --output-on-failure
--output-junit /home/dev/ws_iap/src/iap/results/icra27/icra041/test/plan_manage.xml`.

This evidence is deterministic unit/integration requalification only. It is
not G0B PASS and does not authorize G0C/G0D, threshold/calibration, risk-guide
application or P5. No GPU, ROS/live map, launch, main flow, runner, analyzer,
capture, smoke, benchmark, campaign, tuning or cleanup occurred.
