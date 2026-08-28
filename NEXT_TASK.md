# NONE — ICRA-076 Review blocked awaiting user decision

> Active gate: `ICRA-076_PREREGISTRATION_FREEZE`
> Owner: `SUPERVISOR`
> Activation: `BLOCKED_AWAITING_USER_RESEARCH_DECISION`
> Fixed Review base: `f66fb7344798ba0c04d2d5b59d0be05183f3bfb1`
> Reviewed Builder HEAD: `ae79bc7c49928f8e7e5cc87ae5f0f33de39381fd`
> User decision: `USER-ICRA-ROUTE-20260827-007`

No Builder task is active. ICRA-076 remains `REQUEST_CHANGES / BLOCKED / NOT PASS`; ICRA-077 and held-out access
are unauthorized.

Two High blockers remain:

1. `icra076_flat_null_snapshot_v1.json` uses `resolution_m=0.5`, while the frozen estimand requires `r=0.75` and
   `b=2r=1.5 m`. Production profiling therefore measured a different endpoint exclusion and emitted 154
   controllable samples while labelling the result as the frozen domain. Validation does not cross-bind snapshot
   resolution, endpoint buffer, sample identity and protocol authority.
2. The replay probe compiles against tracked `thirdparty/json/include` headers, but freeze-005 omits `thirdparty`
   from source inventory roots. Those build bytes can change without invalidating the freeze.

The user must choose bounded same-Gate repair (recommended) or explicitly accept both invalid-freeze risks and
bypass to ICRA-077. A repair should align the serialized snapshot with frozen `r=0.75/b=1.5 m`, add cross-binding
and wrong-resolution adversaries, freeze the complete `thirdparty/json/include` dependency with mutation coverage,
then push before creating fresh replay/verification/freeze identities. Preserve all earlier attempts.

Until the user decides, do not edit product/task files, run ROS/GPU/live, access held-out inputs, create ICRA-077
results, tune thresholds, or issue qualification/campaign claims. Preserve shared build/install/log, all evidence
and logs, hidden artifacts and untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`.
