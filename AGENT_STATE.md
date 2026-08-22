# ICRA Agent State

```yaml
schema_version: icra_single_branch_two_agent_v2
branch: dev/icra
active_role: DEEPSEEK
status: TASK_READY
gate: GATE_0B
task_id: ICRA-017
review_base: 6686b917c090bbe39bd1edfba30b1693cfe77082
reviewed_head: c7841ba78d333d1e11f625fbd4b61c2ebb02ce68
conference_route: P0_P4_P5
route_status: PREQUALIFICATION
historical_gate0a_verdict: NO_GO_P2
p0_gate0b_status: BLOCKED_PHASE4A_PROVENANCE_REVIEW_REPAIR_PHASE4B_DELTA_AND_CALIBRATION_PENDING
p4_status: NOT_QUALIFIED
p5_status: IMPLEMENTED_BUT_UNQUALIFIED
supervisor_verdict: ICRA016_REQUEST_CHANGES
review_disposition: ICRA017_PHASE4A_PROVENANCE_AND_IDENTITY_REPAIR_AUTHORIZED
handoff_status: TASK_READY
next_task: NEXT_TASK.md
updated_utc: 2026-08-22T02:53:15Z
```

The conference development target remains conditional `P0 -> P4 -> P5`, but this state does not
qualify any of those stages. Gate 0A remains the historical `NO_GO_P2`; P2 stays disabled for the
ICRA route while its source and retained tests remain available.

The Supervisor reviewed ICRA-016 over `6686b91..c7841ba`. The central source projection, per-slot
original provenance, bounded TTL, commit-only watchdog and additive diagnostics are substantially
present. Standards passes with no hard violations. Independent current builds pass 287/287 required
active GTests, 2/2 retained profile tests and an additional 39/39 retained P1 integrity-cost tests;
all checked consumers link the current ICRA-016 library and protected artifacts remain exact.

ICRA-016 is not accepted because an empty/unusable GNSS measurement callback does not advance the
GNSS generation or clear the prior accepted epoch. The old epoch may therefore remain eligible for
P0 reuse and an in-flight refresh cannot observe the source update. The production occupancy guard
also substitutes sampled semantic comparison for the authorized stable owner identity, and rejected
`beginRefresh` provenance is lost before the typed P0 health edge.

`DEEPSEEK` may begin only ICRA-017 after synchronizing `dev/icra`. It is a narrow Phase-4A review
repair: make every non-null GNSS callback atomically publish either a new valid epoch or an explicit
invalid/absent generation; replace repeated occupancy re-capture/visibility replay with one stable
source-token-plus-generation Seam; and preserve typed provenance failure at P0 without reporting
aborted retention as committed work.

Phase-4B occupancy delta/reverse-ray, production TTL/watchdog values, CPU calibration/scaling,
launch defaults, P1/P2/P3/P4/P5 behavior, main-flow smoke, qualification, formal benchmark, analyzer
and GPU/CUDA work remain disabled. Gate-0B stays blocked until ICRA-017 review, Phase-4B, CPU scaling,
calibration/activation and an explicitly authorized qualification sequence complete.
