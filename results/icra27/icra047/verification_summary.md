# ICRA-047 verification summary

Task HEAD: `f7d60bd3d8a3dab048986ea821b6e8e8b3e50361`

Result boundary: `P4_G0C_REPLACEMENT_PROTOCOL_READY_FOR_REVIEW`; this is
synthetic protocol/dependency readiness only and is never a G0C PASS.

## Replacement identity and lineage

- Protocol v2 SHA-256: `b7a6abe09cf4a5030366d212b9fc74e46c4409d095e9313d6a13af69d75ef920`.
- Registry v2 SHA-256: `7be0f4a02e8b949bd4429cf5c61113f05258f511e12100941ffebbcc3c2d4ae9`;
  it remains proposed, null and disabled.
- Lineage SHA-256: `9268ec4df0994fde82a8a7b07a07cd26f813356a642901576a7ac2703e59c6d5`.
- The 15 exact unique `p4-g0c-r2-*` identities preserve v1 science and exclude
  the consumed v1 namespace.
- ICRA-046 remains bound as 1 attempted / 0 complete / 0 retry, zero analyzer,
  missing `so3_control`, and
  `PRELIVE_DEPENDENCY_GATE_VIOLATION_NO_CALIBRATION_DATA`.

## Executable dependency gate

Runtime manifest SHA-256 is
`1af4d640f69637437f61181e81d45c2f5f1f72a8cafd2dc63a6a10ae63f3c1d6`.
It declares 18 packages, 13 loadable script/full-native-ELF executables, the
SO3 component/resource and full native ELF library, 14 exact SHA-256-bound
config files, six config-selected IAP ELF shared libraries, and the hash-bound
launch contract. ELF inputs require complete headers, native architecture and
successful non-ROS dynamic-link resolution. The same validator
drives dependency-only and full modes before any GPU-running persistence or
GPU call. Synthetic tests cover each missing item plus config drift, malformed
executables/libraries, truncated and wrong-architecture ELF, unresolved
`DT_NEEDED` dependencies, duplicate identity, historical/undeclared prefixes,
aliases/symlinks, registration mismatch, launch drift and manifest drift.

## Tests and static checks

- Initial red: 5/5 expected failures before v2 artifacts existed.
- Pre-review: focused 61/61, launch 16/16 and full 416/416 passed, but temporary
  directories were not explicitly repository-local. This policy finding is
  retained rather than concealed.
- Review remediation uses
  `TMPDIR=$PWD/results/icra27/icra047/tmp`: focused 62/62 and launch 16/16 pass;
  the final full discovery passes 417/417. There were five full discovery
  invocations total: one unconstrained and four repository-local. One
  attempted module-form focused command had
  five loader errors because `test/` is not a package; corrected discovery
  syntax passed.
- Python syntax, fatal-only flake8, canonical v2 JSON and `git diff --check`
  pass. The full suite emits one pre-existing unrelated `ResourceWarning`.

## Protection and forbidden-boundary audit

- ICRA-046 before/after: 3,815 files, 759 directories, 4,884,473,805 bytes,
  aggregate SHA-256
  `823d41bf0e9f5e17ede8b538624ba71d46626d5e1369451f4989a9a0e4cd96b1`.
- PDF before/after SHA-256:
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`;
  it remains untracked and unstaged.
- Pre-commit fetch/divergence is `0 0`; exact task-process audit count is 0;
  available capacity is 122,522,521,600 bytes.
- GPU preflight, ROS/launch, runner CLI, analyzer CLI, calibration, CTest,
  retained binaries, smoke, benchmark, bag/RViz, threshold action, G0D and P5
  invocation counts are all zero.
