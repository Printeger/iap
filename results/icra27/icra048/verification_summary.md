# ICRA-048 verification summary

Task HEAD: `8657412bc5fcbc6b727ca186b7d642ad3b0d5b49`

Result boundary: `P4_G0C_V2_CONTRACT_REPAIR_READY_FOR_REVIEW`; this is a
synthetic contract repair only and is never a G0C PASS or live authorization.

## Three repaired review findings

- Every schema in `P4_G0C_EXPERIMENTS` now uses explicit frozen effective
  values. A real `_launch_setup` unit path proves v2 ego-planner parameters,
  test-planner manifest, run manifest and protocol agree on false P1/P2
  metrics-only values.
- The shared loader pins exact full-file v2 protocol/registry hashes before
  dependency validation or output work. The launch independently freezes the
  complete scientific/effective contract and requires declared actual hashes,
  avoiding a protocol -> dependency -> launch -> protocol hash cycle.
- Inventory rejects secondary v1 and v2 run manifests. Analyzer eligibility
  also fails closed on test-planner/protocol effective-contract disagreement,
  before a threshold draft can be created.

## Final identities

- Launch SHA-256:
  `162f19384112eeeccd02cd8228d05cd4a5758a72fb9fdeb4a738081777aefe03`.
- Runtime-dependency manifest SHA-256:
  `d347896447ff27fd332b4b8764e1fa4368a7410b3080b49c77bc1b5f280d7652`.
- Protocol v2 SHA-256:
  `8b0b2c3ed531680c6c8268738cb1bcb9136f39d2b97e68769e54a53afe59de79`.
- Registry v2 SHA-256:
  `99ccf38c317d45d8605a7e382628a8f0afd32c8097a763d05bfdcc5807beb94f`.
- Replacement-lineage SHA-256 remains
  `9268ec4df0994fde82a8a7b07a07cd26f813356a642901576a7ac2703e59c6d5`.

## Tests and static checks

- Regression-first evidence: protocol 6 expected failures, real launch path 1
  expected failure, analyzer 1 expected failure.
- Pre-review repository-local direct suites passed protocol 11/11, runner
  15/15, analyzer 26/26, launch contract 9/9 and launch golden 16/16; focused
  discovery passed 69/69 and full discovery passed 424/424.
- Review remediation adds trusted-mode schema-downgrade and exact JSON-type/
  recomputed-effective-hash adversaries for both test-planner and run
  manifests. Final direct suites pass protocol 13/13, runner 16/16, analyzer
  28/28, launch contract 9/9 and launch golden 16/16; focused discovery passes
  74/74. Four repository-local full discoveries ran; the final one passes
  429/429. The full suite emits one pre-existing unrelated `ResourceWarning`
  and expected diagnostic stdout.
- Python syntax, fatal-only flake8, canonical JSON and `git diff --check` pass.
- Final two-axis review: Standards 0 blocking / 0 nonblocking; Spec 0 blocking
  / 0 nonblocking. Initial fail-open findings and their remediation are retained
  in `review/two_axis_review.md`.

## Protection and forbidden-boundary audit

- ICRA-046 before/after: 3,815 files, 759 directories, 4,884,473,805 bytes,
  aggregate SHA-256
  `823d41bf0e9f5e17ede8b538624ba71d46626d5e1369451f4989a9a0e4cd96b1`.
- Existing ICRA-047 evidence before/after aggregate SHA-256:
  `b411cfd9b251cfd6de31bd250d39ce63414a8e6df2c8f40f4593041fb28def81`.
- V1 protocol/registry/fixture, replacement lineage and the protected PDF are
  unchanged. The PDF remains untracked and unstaged.
- GPU preflight, ROS/launch, runner/analyzer CLI, calibration, CTest/retained
  binaries, smoke, benchmark, bag/RViz, threshold action, G0C verdict, G0D,
  P5 and cleanup invocation counts are all zero.
