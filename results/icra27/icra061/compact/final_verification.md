# ICRA-061 final verification

- Canonical v2/v5 fixture, protocol, registry, dependency and lineage were registered without changing v1-v4 bytes; all 15 r5 identities remain unconsumed.
- Exact production scanner coverage passes: r5 is `CLOSED_SEGMENTS` with free endpoints and a free tail; r4 remains `OPEN_ENDED_COLLISION`.
- Installed no-ROS preflight passes exact start/horizon/control spacing and effective central obstacle enabled/x/y/z geometry. A y-drift adversary now fails before scanner invocation.
- Fresh CUDA build and static closure pass 17/17 packages, 17 indexes, six ordinary ELF libraries and exact installed/source launch equality.
- The sole nonregistered readiness passes GPU, process health, admission release-once and closed-segment requirements: 848 deferrals, zero pre-release P4 rows, 12 positive-identity post-release rows and controlled shutdown.
- The sole standalone dependency preflight passes exact 18/13/1/14/6 counts with zero GPU, launch, identity or retry.
- Readiness also exposes a pre-identity scientific blocker: all 12 rows are `incomplete_profile`; original validity is 17/200 for ten rows and 0/200 for two, while risk validity is 103-147/200. The stable final RiskGrid is generation 17 with global valid/unknown ratios 0.984/0.016 and dominant unknown `occupied_skip`, so this is not startup warm-up.
- The frozen analyzer contract retains every failed row and requires `METRICS_ONLY` plus 200/200 validity on both arms. The registered matrix is therefore not invoked: full runner/analyzer/registered attempts/retries are 0/0/0/0, and no draft, threshold action or G0C PASS claim exists.
- Final verification passes admission C++ 4/4, hermetic Python 485/485, syntax, fatal-only flake8, JSON parsing and diff checks with empty external ROS-log delta.
- Result: `BLOCKED_R5_READINESS_PROFILE_INCOMPLETE_BEFORE_REGISTERED_IDENTITY`; control returns to SUPERVISOR review without changing science or consuming one-shot identities.
