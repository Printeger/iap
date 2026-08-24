# ICRA-050 verification summary

Result: `BLOCKED_DEPENDENCY_RUNTIME_LIBRARY_MISSING`.

- Initial synchronization passed at `7cecd16f710ec5cad8378117ceb7cf8a40dc6e72`
  with divergence `0 0`; the only pre-existing worktree entry was the protected
  untracked PDF. Capacity was 122,372,354,048 bytes, above the 20 GiB floor.
- Exact package discovery resolved all 17 authorized source identities. The
  external `/home/dev/ws_iap/src/gnss_comm` remained read-only and its aggregate
  stayed `de422a4…16a`.
- The one fresh sanitized non-symlink merged build exited 0 with 17/17 packages
  in 4m58s. All build, install and log outputs remain below ICRA-050.
- The build used `BUILD_WITH_CUDA=OFF`. Its install contains
  `libodometry_estimation_cpu.so` and `libodometry_estimation_ct.so`, but not the
  dependency-manifest-required `lib/libodometry_estimation_gpu.so`.
- The sole standalone `--dependency-preflight-only` invocation used identical
  ordered `AMENT_PREFIX_PATH` and `P4_G0C_ALLOWED_PREFIXES` containing only the
  ICRA-050 merged install and `/opt/ros/jazzy`. It exited 2 with runner state
  `FAILED` and typed reason
  `DEPENDENCY_RUNTIME_LIBRARY_MISSING:iap:lib/libodometry_estimation_gpu.so`.
- The dependency state is retained at SHA-256
  `701c37b87cb04fee6ec61692764ae4ff8be06442385afcc2f40645536c59a8bd`.
  It records zero GPU preflight, zero launch, and `launch_started=false`.
- Fail-closed handling therefore made zero full-runner, GPU, ROS/launch and
  analyzer calls. The registered live `runs` root was never created; no
  analysis, threshold draft, registry change, threshold action, G0C verdict,
  G0D, P5, formal campaign or cleanup occurred.
- Post-failure task-process counts are exactly zero. The complete task build,
  install, log and dependency evidence remain retained for Supervisor review.
- All v1/v2 protocol, registry, fixture, dependency, lineage and launch hashes,
  the protected PDF, ICRA-046/047/048/049 trees and external GNSS source remain
  unchanged.
- Independent review reports Standards 0 blocking / 0 nonblocking and Spec 1
  blocking / 0 nonblocking. The Spec blocker is the deliberate CUDA-off build:
  it could not produce the mandatory GPU runtime library, so the dependency
  failure does not prove that a conforming complete closure failed. No retry is
  permitted inside the consumed ICRA-050 one-shot boundary.

The full Python test suite was not run after the typed dependency failure:
`NEXT_TASK.md` requires an immediate fail-closed stop and authorizes no attempt
to repair or bypass the consumed standalone gate.
