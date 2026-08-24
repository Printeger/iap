# ICRA-040 two-axis review

Fixed point: `d9e9e45db24d9a386578f544758aa829b6080cae`

Reviewed commit: `70131a1 fix(p4): repair G0B review findings [IAP-RQ-423]`

Diff: `git diff d9e9e45db24d9a386578f544758aa829b6080cae...70131a1`

## Standards

No hard violations. `AGENTS.md` requirements are met: valid requirement ID,
synchronized CHANGES/TRACEABILITY, repository-local evidence, and no protected
or Supervisor-owned tracked file changes. `CONTEXT.md` terminology is
consistent.

The reviewer recorded four judgement-call smells:

- Duplicated Code in the post-search identity/epoch validation shape in
  `p4_collision_guide.cpp`.
- Duplicated Code across the new epoch-precedence assertions in
  `test_p4_collision_guide.cpp`.
- Primitive Obsession in the integration helper's two Boolean authorization
  arguments.
- Mysterious Name for `guideSeedMatrix()` in the integration test.

These are non-blocking heuristics rather than documented-standard breaches.
The task explicitly forbids the existing Low design-debt refactor and interface
redesign; no unrelated refactor was added to this narrowly authorized repair.

## Spec

Review verdict: **REQUEST_CHANGES**.

1. **High — retained ICRA-039 trees were modified.** The task requires all ten
   retained trees to stay untouched throughout development and review. During
   loader diagnosis, an old ICRA-039 CTest rewrote retained test logs. The
   complete four-target run re-established every recorded START path/size
   manifest, but that cannot prove byte-for-byte restoration and cannot undo
   the process violation. This finding is not retroactively repairable.
2. **Medium — prescribed exit-code evidence was missing.** The committed XML
   and compact JSON recorded exact cases and disabled tests but omitted command
   exit codes. This review follow-up adds exit code zero for both focused runs
   and every prescribed regression to `test/result.json` and the verification
   summary.

The reviewer found both requested code repairs spec-correct: post-original
identity/epoch validation precedes outcome inspection, the silent
`metrics_only` rewrite is removed, registered G0B tests opt in explicitly, and
the false-boundary test proves preserved false plus the authorization stop.

Summary: Standards has 0 hard findings and 4 judgement-call smells (worst:
duplicated validation shape). Spec has 2 findings, of which 1 is repaired and 1
remains an irreversible process nonconformance (worst: retained ICRA-039 trees
were not untouched throughout development).
