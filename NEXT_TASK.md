# NONE — ICRA-076 blocked awaiting user decision

> Active gate: `ICRA-076_PREREGISTRATION_FREEZE`
> Owner: `SUPERVISOR`
> Activation: `BLOCKED_AWAITING_USER_RESEARCH_DECISION`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-076_LAYER4_PREREGISTRATION_FREEZE`
> Fixed Review base: `8105a16aa5801a1f4c373d842a4e8598594596cf`
> Reviewed Builder HEAD: `aeb5eb0ef26566c62c18aa8dfdae6d03293a8803`
> User decision: `USER-ICRA-ROUTE-20260827-006`

No Builder task is active. ICRA-076 remains `BLOCKED / NOT PASS`; canonical candidate
`preregistration-freeze-003.json` is retained but rejected for formal authority. ICRA-077 is not authorized.

The blocking findings are:

1. The repeatability runner executes a generic PASS GTest, then synthesizes `B_original/B_risk` from fixture
   constants rather than consuming production-emitted serialized snapshot measurements. Its transcript contains
   no measured B values. The validator also computes `abs(D_replay-D_reference)` instead of the route-locked
   `|D_peak|` U95. Therefore `U95=0`, `delta_peak=0.3 m` and freeze PASS are not established.
2. The canonical README command uses `/tmp/icra076-verification.json`, and the implementation admits that external
   evidence manifest, violating the repository-only evidence boundary.
3. Tests 073/074/075 are mandatory verification commands but their source bytes are absent from the frozen source
   inventory, so relevant verification semantics can drift without invalidating the freeze.

The user must explicitly choose either bounded same-Gate repair (recommended and offline-only), or acceptance of
the invalid repeatability/threshold basis and Standards debt followed by bypass to ICRA-077. Until then, do not
execute Builder work, alter the route lock, access held-out data, run GPU/ROS/live, create ICRA-077 results, or
issue any qualification/campaign claim.

Preserve `/home/dev/ws_iap/{build,install,log}`, all raw/compact/live/scientific evidence and ordinary logs,
all ICRA-076 freeze/replay attempts, hidden user artifacts, ignored backup, and untracked
`docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` unchanged.
