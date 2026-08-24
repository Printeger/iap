# ICRA-042 two-axis review

Fixed point: `dc99af894eb9e49d511238e6096932c13a7a70df`.
Reviewed staged WIP with `git diff --cached <fixed-point>` because HEAD still
equaled the fixed point. Standards sources were `AGENTS.md`, `CONTEXT.md`,
`docs/spec/conventions.md` and `docs/spec/talk_spec.md`; the spec source was
`NEXT_TASK.md`.

## Standards

Initial review found three hard fail-closed gaps: analyzer did not recompute
the complete frozen effective-config hash; runner did not require all bound
manifest fields or the complete typed CSV schema; and launch accepted
self-consistent but unregistered protocol/registry/fixture artifacts. It also
reported judgement-call duplication/data-clump smells around the intentionally
repeated frozen binding contract. Allowlist, repository-local evidence,
protected/Supervisor-owned files and historical paths had no finding.

Remediation binds launch to the three registered artifact hashes and validates
canonical/schema/proposed-registry truth; analyzer compares the exact complete
effective map and its deterministic hash; runner requires the complete binding
and typed finite CSV structure. Shared script-side effective hashing now lives
in `p4_g0c_protocol.py`.

## Spec

Initial review found four gaps: explicit scenario-fixture overrides could
silently change live geometry; P1/P2 metrics-only values resolved true in the
node despite the G0C profile recording false; analyzer accepted partial/arbitrary
effective config; and runner accepted partial manifest truth. It found no scope
creep.

Remediation validates every registered scenario-preset value after preset
application and rejects conflicting explicit overrides; the G0C node and
general manifest now consume the configured false P1/P2 metrics-only values;
and the analyzer/runner changes above close the remaining findings. Synthetic
regressions cover the fixture override, G0C metrics resolution, exact config
binding and typed malformed CSV.

Summary before remediation: Standards 3 hard + 2 judgement findings (worst:
config-mismatched bundle eligible); Spec 4 findings (worst: live fixture
override accepted). Final re-review is recorded below before commit.

## Final re-review

Standards: **0 findings / NO BLOCKING FINDING**. The reviewer confirmed the
registered hash/canonical trust root, exact analyzer config binding, complete
runner manifest/CSV validation and ownership/allowlist boundaries. Remaining
private-helper data clumps are acceptable at this audited scale.

Spec: **0 findings / NO BLOCKING FINDING**. All four initial findings are
closed; G0C P1/P2 truth is consistent across node/general/special manifests,
all scenario conflicts fail closed, and exact config plus complete typed
artifacts are enforced. No new scope creep or behavior error was found.

Final summary: Standards 0, Spec 0; worst issue on either axis: none.
