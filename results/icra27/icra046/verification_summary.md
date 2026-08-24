# ICRA-046 verification summary

Requirement: `IAP-RQ-423`

Builder result: `BLOCKED_LAUNCH_DEPENDENCY_SO3_CONTROL_NOT_FOUND`

## Fresh products and pre-live evidence

All six task-local configure/install pairs completed with exit 0 below
`results/icra27/icra046/`. Source and installed protocol, registry, fixture and
launch bytes match. Six relevant binaries have zero missing, historical,
workspace-default IAP/planner or build-tree product resolutions; only task
installs plus authorized external `traj_utils`/`gnss_comm` are admitted.

Focused Python suites pass 66/66 and the one full discovery passes 405/405.
Fresh decision/integration/collision/path/occupancy tests pass 15/15, 5/5,
17/17, 5/5 and 6/6. Plan-manager passes 9/9 CTest targets comprising 186 active
cases and one existing disabled case. Pre-live capacity was 122,578,186,240
bytes and source registry truth remained `PROPOSED_UNCALIBRATED` with null
bundle/gates and application disabled.

## One-shot runner outcome

The sole full runner invocation consumed the exact registered live root. Its
built-in GPU preflight passed: both `nvidia-smi` commands exited 0,
`cuInit(0)=0`, `cuDeviceGetCount=0` and `device_count=1` on an NVIDIA GeForce
RTX 4070 Ti SUPER.

The first launch then exited 1 before either required process started because
ROS package `so3_control` was not found in the sanitized task/authorized
prefixes. The authoritative state is FAILED with one attempted run, zero
complete runs, zero retry and failure `launch_exit_1`. The prior read-only
`ros2 launch ... --show-args` check had exited 0 but did not resolve this
runtime Node package, so it was not a sufficient dependency proof. Entering the
runner, GPU preflight and ROS launch without that proof violated the explicit
pre-live dependency gate. The one-shot call is consumed, so the violation is
irreversible within ICRA-046 and is reported rather than repaired.

Fail-closed handling stopped immediately. Runner invocation count is one;
analyzer invocation count is zero; analysis/draft outputs do not exist. No
alternate root, repair, exclusion, data rewrite, threshold freeze, registry
mutation, G0D or P5 occurred. The four-file raw runs tree is retained unchanged
with manifest SHA-256
`f307e61a90707d6da5a38138558a97447c5267ef9a5184f3df92ca8b97079438`.
