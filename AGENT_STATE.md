# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-030
review_base: c21665518dcb61a273d9e0a357753e52c8889a08
reviewed_head: c44e067c0e542a748127cf9525dc9805eafac1ff
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: REPLACEMENT_SMOKE_READY
p0_gate0b_worker_count: 4
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA029_REVIEW_REQUEST_CHANGES_STATIC_BASELINE_ACCEPTED
review_disposition: ICRA030_ONE_SHOT_REPLACEMENT_SMOKE_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-23T12:07:54Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 and Gate-0B are not qualified,
P4 remains `NOT_QUALIFIED`, and P5 remains implemented but unqualified. Gate 0A remains the
historical `NO_GO_P2`, so P2 stays disabled for the ICRA route.

ICRA-029 truthfully stopped because its verifier compared a frozen evidence inventory with every
file under the task root, while the authorized run-log-manager test necessarily created one
PID-named config below the selected `TMPDIR`. This is not a product, linkage or test failure. All
preceding checks passed, and Supervisor independently closed the remaining authored-whitespace,
allowlist, hash and provenance checks. The task remains `REQUEST_CHANGES` against its literal
overconstrained procedure, but the ICRA-028 static product baseline is accepted for live validation.

`DEEPSEEK` may begin only ICRA-030 after synchronizing `dev/icra`. It shall reuse the exact retained
ICRA-028 IAP and ICRA-026 planner artifacts, complete correctable read-only prechecks, then run exactly
one GPU/dependency-guarded 20-second P0 replacement smoke and exactly one formal analyzer. No product
edit, rebuild, retry, benchmark, tuning, P4/P5 work or Gate promotion is authorized. Retained
build/install trees remain available through ICRA-030 development and Supervisor review.
