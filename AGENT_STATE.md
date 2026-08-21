# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-012
review_base: c865c74317e23b9cb5339174e662d1fc7e87a4ec
reviewed_head: 9faf12139d49b93c259af014249c3c1b447e179c
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PHASE2_REPAIR_PERFORMANCE_AND_CALIBRATION_PENDING
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA011_REQUEST_CHANGES
review_disposition: ICRA012_LEGACY_DIAGNOSTIC_SEMANTICS_AND_REPRO_DOC_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T10:52:38Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not qualify any of those stages.

Gate 0A remains the historical `NO_GO_P2`. P2 stays configuration-disabled for ICRA while its source and tests remain available.

The Supervisor reviewed ICRA-011 over `c865c74..9faf121`. The private call-local SpatialAdvisory Seam, coherent source identity, scalar-equivalent per-horizon growth/fusion, exact new counters, worker aggregation and canonical offline profile are accepted. Supervisor independently rebuilt the affected targets; three exact Predictor regressions, three exact runtime regressions, the profile contract and all six retained suites passed (137/137 retained tests).

ICRA-011 remains review-failed on two bounded findings. Spec: `unique_positions` and production `predictor_unique_positions` were redefined from the legacy LiDAR-cache position count to the generalized spatial-cache size, so GNSS-only incorrectly reports nonzero legacy positions; neighboring legacy LiDAR counters require explicit preservation tests. Standards: the ICRA-011 `docs/CHANGES.md` entry omits the reproducible command required by `AGENTS.md` Definition of Done. No core scientific-result defect or phase-2 performance-evidence defect was found.

`DEEPSEEK` may begin only ICRA-012 after synchronizing `dev/icra`. It restores the legacy LiDAR diagnostic semantics while leaving the new generalized spatial/invocation counters and accepted internal Seam intact, adds source-mode/non-cacheable/early-invalid regressions, strengthens GNSS-only production worker evidence, and adds the required reproduction commands to `docs/CHANGES.md`.

P1/P2/P3/P4/P5 remain disabled. No phase-3 fixed lattice/ring window, cross-refresh reuse, smoke, qualification, production calibration, GPU port, worker/default change, threshold change or P4 work is authorized in ICRA-012. Gate-0B remains blocked on phase-2 review closure, the later staged performance refactor, production calibration/activation and qualification.
