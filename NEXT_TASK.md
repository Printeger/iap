# NONE — ICRA-077A Review blocked awaiting user decision

> Active gate: `ICRA-077_HELD_OUT_CONFIRMATION`
> Owner: `SUPERVISOR`
> Activation: `BLOCKED_AWAITING_USER_RESEARCH_DECISION`
> Fixed Review base: `e9aacf4a2a264f8c7f050db62ddc60ea170b906d`
> Reviewed Builder HEAD: `bdded38ddec75e3fd1365da5cf7b7ceb35f9c7b8`
> User decision: `USER-ICRA-ROUTE-20260828-009`

No Builder task is active. ICRA-077A is `REQUEST_CHANGES / BLOCKED / NOT PASS`; ICRA-077B, held-out access,
GPU, ROS, live execution and ICRA-078 remain unauthorized.

Two blockers remain:

1. **Critical — protected-route fingerprint is not independently anchored.** The validator parses the current
   route document and compares it only with a SHA stored in the same mutable protocol config. A coordinated
   change to a protected route field, its transition `new` value and that config SHA is accepted. Read-only
   Supervisor reproduction changed `active_route` to `P0_P5_CONTROL` and returned
   `JOINT_TAMPER_ACCEPTED`. The validator reads the frozen roadmap Git blob for byte identity but never derives
   and compares its protected fields. A repair must derive the protected-field identity from the immutable
   frozen blob or an explicit authorized re-freeze transition, reject route+config paired tampering and create
   fresh non-overwriting evidence.
2. **High — current evidence has ambiguous task/result identity.** The `icra077a_preregistration_freeze_v2`
   record is created under the active ICRA-077A handoff but forces `task=ICRA-076`, generic `result=PASS` and
   `icra077_authorized=false`, while the same record correctly says ICRA-076 remains blocked/user-bypassed/NOT
   PASS. Before formal outcome access, the schema must distinguish the active repair task, the historical
   preregistration artifact, validation result and downstream ICRA-077B authorization. Preserve freeze-007 and
   generate a fresh identity after repair.

The user must choose bounded same-Gate repair (recommended) or explicitly accept both formal freeze risks and
bypass to debt-bearing ICRA-077B. A repair may change only these two boundaries, add paired-tamper and exact
task/result/authorization adversaries, push source before fresh replay/verification/freeze evidence, and retain
all prior attempts.

Until the user decides, do not edit Builder task files, access held-out inputs, run GPU/ROS/live/360 rows, tune
the contract, issue ICRA-077B/078, or make a qualification/campaign/effect claim. Preserve shared
`/home/dev/ws_iap/{build,install,log}`, all evidence/logs, hidden artifacts and untracked
`docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`.
