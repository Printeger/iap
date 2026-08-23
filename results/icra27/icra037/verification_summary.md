# ICRA-037 verification summary

Requirement: `IAP-RQ-423`

Gate: `P4_G0A`

Result: `P4_G0A_COLLISION_SCAN_GREEN_READY_FOR_REVIEW`

## Implementation boundary

One production `CollisionScanResult` carries exactly `NO_COLLISION`,
`CLOSED_SEGMENTS`, `OPEN_ENDED_COLLISION` or `INVALID_INPUT`, plus ordered
closed endpoint pairs. Both `initControlPoints()` and
`check_collision_and_rebound()` consume the same scanner. A run may enter only
inside the legacy trigger window, but an active run continues through the seed
tail. Open-ended and invalid outcomes expose no segments and return before the
existing A*/guide path. The planner-manager initial caller returns failure for
those outcomes before candidate fanout or guide publication.

No original/risk guide generation, A* behavior, profile, scoring, selection,
fallback, lineage, P5 behavior, GPU, ROS/live flow, smoke, benchmark,
qualification or Gate promotion was added or run.

## TDD and builds

- Fresh IAP configure/build/install: exit 0.
- Fresh bspline configure: exit 0 against ICRA-037 IAP and retained read-only
  ICRA-026 plan-env/path-searching.
- RED compile attempt: exit 2 on the deliberately missing production scan
  result/status and test hooks.
- First GREEN focused compile: exit 0. Focused attempts progressed from 13/13
  to the final 14/14 after adding the closed-path integration assertion.
- Final bspline rebuild/install: exit 0. Final selected CTest: 2/2 targets,
  comprising P1 39/39 and collision contract 14/14.
- Retained direct regressions: path-searching P4 4/4 and occupancy epoch 6/6.
- Corrected plan-manager build/install: exit 0. Final affected CTest: 9/9
  targets, zero failures.

The first plan-manager configure found workspace-default IAP typesupport and
reported a runtime-path cycle. No planner test was accepted from that linkage.
The same task-local tree was reconfigured with the exact ICRA-037 typesupport
file, rebuilt and reinstalled; all nine affected targets and the final direct
linkage audit then passed. `preflight/linkage.json` records the corrected
closure.

## Frozen contract and final artifacts

- Frozen fixture SHA-256 (unchanged):
  `49a676a5ff51538ab961c814409f6c2dfb7ba4679a861d4e8e94cc7d5679c788`.
- Protected PDF SHA-256 (unchanged and unstaged):
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- Optimizer header:
  `4a70f4e35075a8f0bec3087e88e512e336d85af650bc4c6c85409850bdd30879`.
- Optimizer source:
  `e0be3918b7095189a181baa11439f1379312f661ee3b304fb0f7796a84a15529`.
- Focused test:
  `04bddf04b95cd6438701be329ee9dfa8e04647a902715e903a47f9a2f6292573`.
- Planner-manager source:
  `5e1b3379b729040d0e1a32c4737dd80f22028a78775e15d28b9542689f0eb494`.
- Focused executable:
  `e0d5b69a612dcb2d802df3628d795ed608b2030b16d6a9ee6d150fb98ffd8256`.
- Installed IAP library:
  `1dd661eee9887f3b0486058f4e16e18e0b6d5bdc07e9fec02e035bd8b33c2f92`.
- Installed bspline archive:
  `c74fe565c08d0f06611ca2fb33a9467a5ec094658e6c4d28ea927cbf65c0600b`.
- Installed planner node:
  `4f59f5d56b2426a5550841665a6e9f2893175aa3157f6a48868e1e27c447eaa4`.

`test/green_result.json` records the exact seven former RED outcomes and final
counts. Final XML is retained as `test/bspline_final.xml` and
`test/plan_manage_final.xml`; prior focused attempts and the direct retained
regressions remain in the same task tree. Build/install trees are retained for
Supervisor review and are not staged.
