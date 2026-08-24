# ICRA-048 two-axis review

Fixed point: `8657412bc5fcbc6b727ca186b7d642ad3b0d5b49`

Review command: `git diff --cached
8657412bc5fcbc6b727ca186b7d642ad3b0d5b49`

## Standards

Initial review found one blocking fail-open condition: shared-loader v2 trust
mode was selected from the untrusted protocol's own schema. Remediation makes
expected schema a trusted caller input, defaults to v2 and retains explicit
legacy mode only for the exact protected v1 path. Schema downgrade now rejects
before dependency validation.

Code re-review reports 0 hard/blocking findings and 0 judgement-call smells.
The independent launch/shared scientific constants are required by the
specified acyclic trust split and are not counted as duplicated-code smell.
The final procedural audit confirms all 18 staged paths are allowlisted,
unstaged diff is empty, the protected PDF is untracked/unstaged, branch
divergence is `0 0`, task-process count is zero, and protected hashes match.

Final Standards count: **0 blocking / 0 nonblocking**.

## Spec

Initial review found loose Python equality and stale-hash gaps in test-planner
and run manifests, plus a v1 compatibility concern. Remediation uses canonical
exact-type JSON equality, recomputes each supplied effective-value hash, rejects
before runner COMPLETE/analyzer draft, and keeps registered-v1 mode only for
the exact protected v1 path. Adversaries cover bool/int and int/float
substitutions, stale hashes and schema downgrade.

Final re-review reports no missing/partial requirement, scope creep or
incorrect behavior. Trust anchors, launch/runtime effective values, secondary
manifest rejection, minimal hash cascade, retained/protected state, no-live
boundary, focused 74/74 and full 429/429 align with `NEXT_TASK.md`.

Final Spec count: **0 blocking / 0 nonblocking**.

## Aggregate

Standards: 0 findings, worst issue none. Spec: 0 findings, worst issue none.
The correct handoff boundary is
`P4_G0C_V2_CONTRACT_REPAIR_READY_FOR_REVIEW`, never G0C PASS.
