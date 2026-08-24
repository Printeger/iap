# ICRA-039 two-axis review

Requirement: `IAP-RQ-423`

Fixed point: `b45ff3ad633fc7ce3ab2418f774073a6eb3a2d16`

Initial implementation: `4086ce5`

Reviewed repair: `05a9a36`

## Initial findings and resolution

Standards initially reported two maintainability findings: duplicated
initial/rebound decision consumption and a five-value injection identity data
clump. Spec initially reported four gaps: the production attempt ID could come
from P1 or a local surrogate, injection did not recheck the complete request
identity, the reported positive fixture used scripted paths, and enabled G0B
did not force metrics-only.

Commit `05a9a36` resolves all six findings. Initial and rebound share
`collectP4GuidesForSegments()`; injection accepts one reconstructed
`P4GuideRequest`. The manager passes its real nonzero attempt ID independently
of P1, complete canonical request identity is rehashed between searches and
before injection, an injection mismatch records invalid/replan and clears the
selected guide, enabled P4 attempts force G0B metrics-only, and the
authoritative central-obstacle fixture runs both searches through production
`P4AStarGuideSearch`.

## Final Standards review

Verdict: **PASS — 0 findings**.

The reviewer checked `b45ff3a..05a9a36` against `AGENTS.md`,
`docs/spec/conventions.md` and `docs/spec/talk_spec.md`. The two prior findings
are resolved and no new hard violation or judgement-call smell remains.
Documentation/traceability, JSON/XML parsing and `git diff --check` pass.

## Final Spec review

Verdict: **PASS — 0 findings**.

The reviewer checked the complete committed diff against `NEXT_TASK.md` and
`docs/REQS.md` IAP-RQ-423, explicitly rechecking all four prior gaps. The
review confirms the real manager attempt binding, complete identity
revalidation and invalidation, production-A* deterministic fixture with
200/200 strict dominance and repeat hashes, forced G0B metrics-only with a
false general default, allowlist compliance, exact fallback behavior,
unchanged timeout/ratio constants, preserved pre-stops, compact evidence and
test counts.

## Final verification state

- decision: 11/11;
- integration: 4/4;
- collision: 17/17;
- P1: 39/39;
- path-searching P4: 5/5;
- occupancy epoch: 6/6;
- affected plan-manager CTest: 9/9, 186 active cases passed, one pre-existing
  disabled case;
- no GPU, ROS/live flow, launch, runner, analyzer, capture, smoke, benchmark,
  qualification, calibration, G0C/G0D, risk application or P5 execution.

Final builder state is `P4_G0B_METRICS_ONLY_READY_FOR_REVIEW`, not G0B PASS.
