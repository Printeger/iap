# ICRA-050 two-axis review

Fixed point: `7cecd16f710ec5cad8378117ceb7cf8a40dc6e72`

Candidate: pre-commit staged ICRA-050 documentation and compact evidence.

## Standards

Hard/blocking: **0**. Nonblocking/judgement: **0**.

No documented-standard violations or baseline smells were found. The candidate
binds the work to `IAP-RQ-423`, records exact commands/exits, updates required
documentation, discloses `BUILD_WITH_CUDA=OFF` and the missing GPU library,
stages only allowed compact evidence, retains all raw products and reports a
truthful BLOCKED result. Repeated commands in CHANGES and compact evidence are
audit-required representations rather than a Duplicated Code judgement.

## Spec

Blocking: **1**. Nonblocking: **0**.

**High — the sole build intentionally excluded a mandatory member of the
declared closure.** `NEXT_TASK.md` requires “Fresh-build the complete declared
closure” and requires all six IAP runtime libraries to validate. The immutable
dependency manifest includes `iap:lib/libodometry_estimation_gpu.so`, while the
build explicitly used `-DBUILD_WITH_CUDA=OFF`, guaranteeing that library could
not be produced. The resulting dependency failure is therefore self-induced,
not evidence that a conforming complete build failed. Disclosure and correct
no-retry handling do not cure the build-spec violation. A CUDA-enabled build
failure would instead have required `BLOCKED_BUILD_OR_LINKAGE` before the
dependency runner.

Everything after the typed dependency failure is otherwise fail-closed and
truthful: one fresh 17-package non-symlink merged build, one standalone
preflight, zero GPU/ROS/full-runner/analyzer calls, no retry or cleanup,
retained products, allowlisted staged files, protected identities unchanged and
exact command/exit records. No retry is permitted now; Supervisor disposition
is required.

## Aggregate

- Standards: 0 blocking, 0 nonblocking.
- Spec: 1 blocking, 0 nonblocking.
- Disposition: `BLOCKED_DEPENDENCY_RUNTIME_LIBRARY_MISSING` with an explicit
  preceding build-spec violation; never G0C PASS.
