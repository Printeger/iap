# NONE — ICRA-075 blocked awaiting user decision

> Active gate: `ICRA-075_EXPLORATORY_AND_POWER_INPUTS`
> Owner: `SUPERVISOR`
> Activation: `BLOCKED_AWAITING_USER_RESEARCH_DECISION`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-075_LAYER3_EXPLORATORY_ABLATION_AND_POWER_INPUTS`
> Review base: `7752255987504bdf961d01a32029769eaf512d80`
> Reviewed Builder HEAD: `e5d625ab6f3d922d563425fcf01969c3d1b4b0a4`
> User decision: `USER-ICRA-ROUTE-20260827-004`
> Next task: `NONE`

## Supervisor verdict

ICRA-075 is `REQUEST_CHANGES / BLOCKED / NOT PASS`. Builder correctly stopped the development matrix after the
first PRIMARY seed-75001 `P0_P5_CONTROL` row. GPU, source check, all 15 required processes, 134 ready/non-stale P0
records and owned cleanup passed, but all 2,137 P5 final observations rejected `current_low_margin`. Current fused
GNSS-dominant HPL was `24.3673612..27.733391 m` and VPL `68.8205779..86.6898998 m` against P5 current alert limits
`10/20 m`; observed minimum margin reached about `-52.256574 m`. No EGO final, normal publication or P5 runtime
identity exists. The exact matrix is 0/40 complete and `icra075_exploratory_power_inputs_v1` was not produced.

Two independent engineering blockers also remain:

1. `recordP4VerticalSliceLineage()` treats metrics-only identity/guide/CSV writer failures as success. The FSM uses
   that return value before P5/publication, so this is a decision-path fail-open change, not evidence-only
   instrumentation.
2. The runner performs its last source capture before analyzer execution and does not recheck afterward. A source
   change during analysis can escape the required typed batch fail-closed record.

The offline ICRA-075 suite independently passes 14/14, matrix construction/hashes/build declarations and retained
GPU/process/cleanup evidence conform. Those facts do not satisfy the mandatory complete matrix, identity chain or
power exit. ICRA-073 and ICRA-072B remain blocked/user-bypassed/NOT PASS.

## Required user decision

No Builder task is active. The route owner must explicitly choose one path:

1. Recommended: continue ICRA-075 with one bounded repair task. Restore fail-closed metrics-only lineage, add the
   missing post-analyzer source check, and diagnose the P5/fixture compatibility without tuning provider truth,
   AL/PL, P5 thresholds or the protected research route. Resume the exact non-overwriting matrix only if a genuine
   implementation/configuration defect can be repaired within the frozen contract.
2. Bypass ICRA-075 and issue ICRA-076. This keeps ICRA-075 BLOCKED/NOT PASS and accepts proceeding to formal
   preregistration without the planned 40-row exploratory matrix, effect/variance estimates, safety summaries or
   power inputs, plus the two unresolved fail-closed defects. It materially weakens the basis for freezing SESOI
   and the confirmatory sample size.

Either choice requires a distinct user decision. Until then, do not build, run ROS/GPU/live, modify product or
fixture bytes, tune, access held-out seeds, freeze formal parameters, issue ICRA-076, or alter retained evidence.

## Retention

Preserve `/home/dev/ws_iap/{build,install,log}`, all raw/compact/live/scientific evidence and ordinary logs,
every ICRA-072..075 artifact, `.claude/settings.local.json`,
`src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~`, and untracked
`docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` unchanged and unstaged.
