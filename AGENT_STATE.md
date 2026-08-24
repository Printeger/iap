# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: P4_G0B
task_id: ICRA-041
review_base: d9e9e45db24d9a386578f544758aa829b6080cae
reviewed_head: 57ea9263b90987245e352033a82241139d3ac2f1
conference_route: P0_P4_P5
route_status: P4_G0B_CLEAN_REQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: PASS
p0_gate0b_worker_count: 4
p4_status: G0A_PASS_G0B_FUNCTIONAL_REPAIR_PASS_PROVENANCE_REQUEST_CHANGES
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA040_REVIEW_REQUEST_CHANGES_RETAINED_ARTIFACT_PROVENANCE
review_disposition: ICRA041_SELF_CONTAINED_REQUALIFICATION_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-24T06:33:43Z
```

The conditional conference route remains `P0 -> P4 -> P5`. P0 Gate-0B remains `PASS`; the historical
Gate-0A verdict remains `NO_GO_P2`, so P2 stays disabled. P4-G0A remains `PASS`. ICRA-040's two code
repairs pass functional review and every prescribed regression, but P4-G0B is not qualified because an
accidental CTest rewrote retained ICRA-039 build-tree logs, violating the immutable-artifact provenance
contract.

The route remains at P4-G0B. `DEEPSEEK` may begin only ICRA-041 after synchronization. ICRA-041 is a
self-contained evidence-only requalification: rebuild every required package from current source under a
new task root, consume no ICRA-039/040 product, rerun the complete deterministic matrix and prove the old
trees received no further write. No source/test/config change, calibration, G0C/G0D, risk application,
P5 or live execution is authorized.

All ten ICRA-039 and four ICRA-040 build/install trees remain retained through ICRA-041 development and
Supervisor review. Cleanup is Supervisor-only after a future Review PASS and pushed documentation.
