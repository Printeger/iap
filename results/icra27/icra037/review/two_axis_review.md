# ICRA-037 two-axis review

Requirement: `IAP-RQ-423`

Fixed point: `cc6a58a82befd23758b9ed2d0661253df34a0594`

Diff: `git diff cc6a58a82befd23758b9ed2d0661253df34a0594...HEAD`

## Standards

Initial review found one hard documentation issue (no reproducible command in
`docs/CHANGES.md`) and one judgement-call duplication smell (four copies of the
open/invalid fail-closed predicate). The implementation now provides the
repository-root reproduction block and all consumers call the shared
`collisionScanFailsClosed()` predicate.

Final re-review: **PASS — zero findings**. AGENTS.md requirement mapping,
allowlist, documentation, evidence, commit messages, frozen fixture and
protected-PDF boundary are consistent.

## Spec

Initial review found one high defect: separate interpolated occupied runs in a
single control-point interval could emit overlapping endpoint pairs. The new
production-facing regression first failed with two `(2,3)` pairs; the scanner
now merges overlapping pairs and the test passes. Initial and follow-up review
also required command-level evidence. The final ledger records the exact
compile RED and overlap RED calls/exits/counts, rejected configure provenance,
final build/test commands and CMakeCache/workspace-default/readelf/ldd linkage
audits.

Final re-review: **PASS — zero findings**. Shared scanner, legacy entry/full
tail behavior, non-overlap, open/invalid fail-closed behavior, frozen 11-case
contract, final 15/15 focused suite, regressions, linkage and forbidden scope
all satisfy `NEXT_TASK.md`.

Final summary: Standards 0 findings; Spec 0 findings; no worst remaining issue
on either axis.
