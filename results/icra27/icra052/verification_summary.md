# ICRA-052 synthetic verification

IAP-RQ-423. This task registered the disjoint 15-ID r3 matrix and made the
launch environment/output inventory a runner-owned pre-attempt gate. It did
not execute the runner or analyzer CLI, GPU preflight, ROS, a build, CTest,
main flow, smoke or qualification.

Formal verification commands and results:

- `env TMPDIR=$PWD/results/icra27/icra052/tmp python3 -m unittest discover -s test -p 'test_p4_g0c_*.py' -v` — exit 0, 84 tests.
- `env TMPDIR=$PWD/results/icra27/icra052/tmp python3 -m unittest discover -s test -p 'test_*.py'` — exit 0, 439 tests.
- in-memory `compile()` over the four production Python files and five focused
  tests — exit 0, 9 files.
- `python3 -m flake8 --select=E9,F63,F7,F82 ...` — exit 0.
- canonical JSON comparison over all four r3 JSON files — exit 0.
- `git diff --check` — exit 0.

Development history is retained truthfully. An initial package-style unittest
invocation exited 1 because `test/` is not a Python package. The first focused
discovery then exposed eight dependency-loader compatibility errors, and an
intermediate dependency-only run exposed two stale v2 fake-install launch-hash
failures; both defects were fixed before the formal runs above. One analyzer
development run exposed a missing test constant and was corrected. Early
focused invocations also omitted explicit `TMPDIR`; their auto-cleaned
`TemporaryDirectory` products did not persist, and both formal suites were
rerun with the repository-local ICRA-052 TMPDIR. No product or retained
artifact was executed or changed.

Adversarial coverage includes 7 runner pre-attempt path/inventory failures,
all 4 required child environment keys, all 8 mutable-output keys including the
disabled-bag destination and unknown-output fail-closed behavior. Analyzer
coverage applies remove/change/wrong-type to every new binding (12 x 3 = 36),
refreshes the legitimate launch/inventory hashes, and proves no threshold
draft is emitted.

The first Spec review found that the existing dependency-preflight test class
had been repointed from v2 to v3. Remediation restored its v2 protocol,
registry, dependency and schema semantics against the immutable historical
launch bytes, added separate v3 complete-closure coverage, gave production r3
dependency results their v3 schema, and added all four absent-caller-key
cases. The 84/84 and 439/439 commands above are the post-remediation reruns.

Protected SHA-256 values remain: PDF `1f07da56...44f6`, ICRA-051 state
`7c3cafc5...46a7`, and external launch log `f506e556...58e7`. The external log
is preserved but never read by a live process or modified. ICRA-051 carries
both one High Standards blocker and one High Spec blocker because that live
attempt created the repository-external ROS log.
