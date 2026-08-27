# NONE — ICRA-075 blocked awaiting user decision

> Active gate: `ICRA-075_EXPLORATORY_AND_POWER_INPUTS`
> Owner: `SUPERVISOR`
> Activation: `BLOCKED_AWAITING_USER_RESEARCH_DECISION`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-075_LAYER3_EXPLORATORY_ABLATION_AND_POWER_INPUTS`
> Fixed Review base: `32283d0fee784089896895f2cf907363561bbaa2`
> Reviewed Builder HEAD: `6678e7d6afc3f0663e33179bea41516bebed9bb9`
> User decision: `USER-ICRA-ROUTE-20260827-005`

No Builder task is active. ICRA-075 remains `BLOCKED / NOT PASS`: the bounded repair correctly diagnosed
`FROZEN_CONTRACT_INCOMPATIBLE` and stopped before GPU/ROS/live, so the exact matrix remains 0/40 and no power
record exists. Passing P5 would require changing protected `max_pl`, provider/PL, HAL/VAL or threshold authority.

Standards Review also found two P1 fail-closed defects that must remain visible unless the user explicitly accepts
them: the integrated runner overwrites an earlier `SOURCE_CHANGED_BEFORE_ROW` or `SOURCE_CHANGED_DURING_ROW` with
`SOURCE_CHANGED_AFTER_ANALYZER`, and the compatibility tool permits arbitrary `--output` paths outside the
required repository-local evidence root. The canonical diagnosis-003 itself is retained, repository-local and
hash-bound; these findings concern the implementation boundary.

The route owner must choose one of the following before any next task is issued:

1. repair the two engineering defects and separately authorize a revision/re-freeze of the incompatible scientific
   contract; or
2. explicitly accept the frozen incompatibility, missing 40-row exploratory/power basis, and stated engineering
   debt, then bypass ICRA-075 and issue ICRA-076.

Until that choice, do not execute Builder work, change the route lock, tune protected values, run GPU/ROS/live,
create matrix-003, freeze a formal protocol, access held-out data, or issue ICRA-076. Preserve shared
`/home/dev/ws_iap/{build,install,log}`, all raw/compact/live/scientific evidence and ordinary logs, hidden user
artifacts, ignored backup, and untracked `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf` unchanged.
