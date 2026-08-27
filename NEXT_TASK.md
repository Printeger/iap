# NONE — ICRA-073 blocked awaiting user fixture decision

> Active gate: `ICRA-073_EFFECT_DIAGNOSTICS`
> Owner: `SUPERVISOR`
> Activation: `BLOCKED_AWAITING_USER_RESEARCH_DECISION`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-073_LAYER3_INVERSE_CORRIDOR_EFFECT_DIAGNOSTICS`
> Review base: `347c9111f1f6618b678a0c62942b561ac94e7814`
> Reviewed Builder HEAD: `4f5bb302920f869e45a8ac240f192a81f28162c7`
> User decision: `USER-ICRA-ROUTE-20260827-003`
> User approval anchor: `a30468e4ca991dacfe24a10c45040c51efd74ce7`
> Next task: `NONE`

## Supervisor verdict

ICRA-073 is `BLOCKED_FROZEN_GUARD_INFLATION_CONFLICT / NOT PASS`. The exact frozen PRIMARY/MIRROR/FLAT_NULL
descriptors and source binding are implemented and focused tests pass 3/3. Canonical `preflight-001.json` is
source-bound and fails only at the first mandatory geometry gate: risky-centre-line clearance to the central
cuboid is `1.275072583 m`, below the required `0.75 + 0.50 + 0.099 = 1.349 m` by `0.073927417 m`.

The Builder correctly stopped before shared build, GPU preflight, ROS/live work or tuning. Complete-scene
reachability and the later polyline, LiDAR, outer-tree, provider, mirror/null and oracle-isolation gates remain
not evaluated. The independent 200-sample committed-final oracle, paired runner, all three control/treatment
pairs and P5/publication/runtime identity evidence do not exist.

Standards Review additionally found two High defects in the preflight tool: source admission compares only four
hard-coded retained paths and can miss a new ignored fifth file; `--variant --output` can write outside the
repository and overwrite existing files. If ICRA-073 continues, both require adversarial tests and repair. The
dormant mirror check also needs full-geometry coverage before it can support later diagnostics.

## Required user decision

No Builder work is authorized until the route owner chooses one of these paths against the latest pushed
Supervisor handoff:

1. Revise the frozen fixture and continue ICRA-073. A new bounded task will repair the two preflight guards,
   compute and freeze the smallest scientifically defensible geometry/guard revision, retain
   PRIMARY/MIRROR/FLAT_NULL causality, then complete the original preflight, oracle and paired diagnostics.
2. Accept the blocker and bypass ICRA-073 to ICRA-074. This keeps ICRA-073 BLOCKED/NOT PASS and explicitly
   accepts that targeted optimization starts without the retained effect diagnostic that the roadmap requires.

Either choice requires a distinct user decision record. Decision 003 bypassed ICRA-072B only and cannot be
reused to infer this new choice. Until then, `active_role=SUPERVISOR`, `next_task=NONE`; do not build, run ROS/GPU/
live, modify fixture or algorithm bytes, tune, issue ICRA-074, or alter retained evidence.

## Retention

Preserve `/home/dev/ws_iap/{build,install,log}`, all raw/compact/live/scientific/Supervisor evidence, ordinary
logs, every ICRA-072/073 artifact, `.claude/settings.local.json`,
`src/uav_simulator/local_sensing/CMakeModules/FindEigen.cmake~`, and untracked
`docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` unchanged and unstaged.
