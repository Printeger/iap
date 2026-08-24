# ICRA-049 verification summary

Task HEAD: `d828802c89d6dae1dfc969d7a1f625b9ef26b0b0`

Result boundary: `P4_G0C_TOP_LEVEL_EVIDENCE_BINDING_READY_FOR_REVIEW`; this is
a synthetic evidence-binding repair only, never G0C PASS or live authorization.

## Repaired evidence seam

- The shared protocol module freezes exactly 28 protocol-effective keys that
  the production test-planner manifest materializes at top level.
- Every mapped key must exist and must equal the protocol value as canonical
  JSON bytes. Boolean/integer and integer/float substitutions therefore reject.
- Complete nested `p4.g0c` hash, effective-value and scientific bindings remain
  validated independently.
- Runner consumes the shared validator before COMPLETE/final inventory;
  analyzer consumes it before threshold-draft eligibility.

## Parameterized evidence

- Synthetic fixtures add all 28 required top-level fields and retain the full
  nested binding. Unchanged production-shaped v2 evidence remains runner
  COMPLETE and analyzer `DRAFT_ELIGIBLE`.
- Runner covers 84 top-level adversaries: remove/change/wrong-type for every
  mapped key. Each rejects before COMPLETE and final inventory.
- Analyzer covers the same 84 adversaries with nested values unchanged and
  legitimate inventory/state hashes refreshed. Each is REJECTED with no draft.
- P1/P2 metrics-only top-level-only drift, bool/int substitution and the
  manager-clearance float/int substitution are included.

## Tests and boundaries

- Regression-first: protocol 1 expected error; runner 84 expected failures;
  analyzer 84 expected failures.
- Final direct: protocol 14/14, runner 17/17, analyzer 29/29, launch contract
  9/9 and launch golden 16/16. Focused discovery passes 77/77; the one full
  repository discovery passes 432/432.
- Python syntax, fatal-only flake8, canonical JSON and `git diff --check` pass.
- Final two-axis review: Standards 0 blocking / 1 nonblocking independent-test
  duplication smell; Spec 0 blocking / 0 nonblocking. No remediation required.
- No build, GPU preflight, ROS/launch, runner/analyzer CLI, calibration,
  CTest/retained binary, bag/RViz, threshold action, G0C verdict, G0D, P5 or
  cleanup ran.

## Protected state

- ICRA-046 remains 3,815 files, 759 directories and 4,884,473,805 bytes with
  aggregate SHA-256
  `823d41bf0e9f5e17ede8b538624ba71d46626d5e1369451f4989a9a0e4cd96b1`.
- ICRA-047 and ICRA-048 evidence aggregates remain respectively
  `b411cfd9b251cfd6de31bd250d39ce63414a8e6df2c8f40f4593041fb28def81`
  and `561edd73851ebcf12ae17ff3642d80631b25d587d2aaf43a20a788f438fc24b1`.
- V1/v2 artifacts, replacement lineage, launch and the protected PDF remain
  unchanged. The PDF remains untracked and unstaged.
