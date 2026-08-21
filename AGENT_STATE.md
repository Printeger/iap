# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-016
review_base: eb66c078a97d00360e542bfd28bea897a66510e6
reviewed_head: eb1cb67889960d995f7ca8dab318da649af82cb4
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PHASE4A_VERSION_TTL_WATCHDOG_PHASE4B_DELTA_AND_CALIBRATION_PENDING
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA015_PASS_PHASE3B_CLOSED
review_disposition: ICRA016_PHASE4A_VERSION_TTL_WATCHDOG_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-21T15:59:52Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not
qualify any of those stages. Gate 0A remains the historical `NO_GO_P2`; P2 stays disabled for the
ICRA route while its source and retained tests remain available.

The Supervisor reviewed ICRA-015 over `eb66c07..eb1cb67`. Active-source identity now contains only
the spatial evidence actually consumed by each source mode; `current.stamp/valid` still runs through
per-horizon validation without invalidating or restamping retained spatial advice. Legacy LiDAR
counters again describe only same-call population/reuse, while rolling counters carry cross-refresh
work. Reproduction commands are present. Standards and Spec both pass. Independent current builds
pass 271/271 active GTests plus 2/2 retained profile tests, and all retained artifacts remain exact.

ICRA-015 closes the ICRA-014 repair and phase 3B/phase 3 implementation stage. It does not qualify
P0 or Gate-0B. `DEEPSEEK` may begin only ICRA-016 after synchronizing `dev/icra`: add atomic,
versioned provenance for active spatial sources, bounded continuous-source TTL retention and a
successful-full-refresh watchdog while keeping all policy disabled by default and preserving the
existing RiskGrid/P4/P5 consumer Seam.

Occupancy delta/reverse-ray work remains a separate Phase-4B task. Production TTL/watchdog values,
CPU calibration/scaling, launch defaults, P1/P2/P3/P4/P5 behavior, main-flow smoke, qualification,
formal benchmark, analyzer and GPU/CUDA work remain disabled. Gate-0B stays blocked until phase 4,
CPU scaling, calibration/activation and an explicitly authorized qualification sequence complete.
