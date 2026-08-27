# ICRA-072 — exact local-control artifact admission and canonical closure (ICRA-072B)

> Active gate: `P4_V2_END_TO_END_VERTICAL_SLICE_AND_LIVE_SMOKE`
> Owner: `DEEPSEEK`
> Activation: `TASK_READY`
> Conference route: `P0_P4_V2_P5`
> Milestone: `ICRA-072B_LAYER2_STABILIZATION`
> Review base: `2e0c2b3930a37b99b1bdaf9d9e08a950117b37f4`
> Reviewed Builder HEAD: `af7c80480a64eb45828e035f3725c18056ba10b8`
> User decision: `USER-ICRA-ROUTE-20260826-002`
> Workflow decision: `USER-ICRA-WORKFLOW-20260826-001`
> Requirement mapping: `IAP-RQ-320`, `IAP-RQ-321`, `IAP-RQ-400`, `IAP-RQ-410`, `IAP-RQ-421`, `IAP-RQ-422`, `IAP-RQ-423`, `IAP-RQ-424`
> One task: admit exactly the retained local agent-control artifact without hiding arbitrary source, then produce the required canonical PASS

## Accepted repair and remaining blocker

The bounded harness repair at `7a5aa5840cb65fc17067fed993019d2a6cc9118d` is accepted. Exact command-local
Git trust, isolated HOME/XDG, absence/wrong-trust rejection, no Git-config mutation, and Python/gtest
skipped/disabled fail-closed behavior pass focused Review. Supervisor reruns pass runner 5/5 and tools 17/17.
Do not rework these seams or touch product C++, production tests or shared build roots.

The attempted `repair-001` command exited 2 at source admission before output/log-root creation and before any
suite. It discovered two untracked paths in the exact isolated environment:

- `docs/icra27/dev/ICRA_SYSTEM_FLOW.pdf`, SHA-256
  `1f07da5631a6551a2f98c02d46fd45bc87f2f1e3e7c14e95f9a7f4a0bac844f6`.
- `.claude/settings.local.json`, 72-byte retained local agent-control file, SHA-256
  `27aac0ccca0ad0ab573578864cf27b9560d3f819bdeeae62378f8c20e62a8f64`.

Layers 1–3 are repeatable and neither `repair-001_summary.json` nor `repair-001_logs/` exists. Therefore no
canonical result identity was consumed. Correct Builder documentation that asserted a retry prohibition, without
rewriting or concealing the truthful historical source-admission stop.

## Required exact-admission repair

1. Extend source admission to accept exactly the two paths and hashes above. Classify the JSON as a local agent
   control artifact that is not runtime source. Require both artifacts to be ordinary non-symlink files with the
   exact path, byte size and SHA-256; record their classification and observed metadata in source binding and the
   canonical summary.
2. Reject any missing/changed artifact, symlink, non-regular file, third untracked path, tracked/staged change,
   rename or deletion. Do not use `.gitignore`, ambient excludes, wildcard allowlists, `--untracked-files=no` or
   any mechanism that hides arbitrary source/config paths.
3. Add focused RED/GREEN tests for exact two-artifact acceptance and every rejection class above. Preserve all
   existing trust, skip/disabled, row/suite cardinality, count and pushed-source checks.
4. Do not edit, chmod, track, move or delete `.claude/settings.local.json` or the PDF. Do not expose their content
   in evidence; path/classification/size/hash are sufficient.
5. Correct README and Builder-owned adjudication docs: the earlier invocation stopped before result creation;
   `repair-001` remains the authorized fresh non-overwriting identity.
6. Commit and push the repair first, fetch and prove divergence `0 0`, then run the canonical command once using:
   `results/icra27/icra072b/repair-001_summary.json` and
   `results/icra27/icra072b/repair-001_logs/`.

## Allowed scope

- `scripts/dev_planner/run_icra072b_stabilization.py`.
- `test/test_icra072b_stabilization.py`.
- `README.md`, `DEV_LOG.md`, `docs/CHANGES.md`, `docs/TRACEABILITY.md`, `docs/REQS.md`.
- New immutable `repair-001_summary.json` and `repair-001_logs/` under `results/icra27/icra072b/`.

Do not edit product C++, production tests, `.claude/settings.local.json`, the PDF, Supervisor files, route/scope/
plan/workflow/system-flow authority or shared workspace roots. If exact admission cannot be proven without a
broader allowlist or artifact mutation, stop and return the blocker.

## Forbidden and retention

- No shared rebuild, ROS launch, GPU preflight, live run, `run-025`, bag, algorithm, inverse-corridor, effect,
  optimization, qualification or campaign work.
- Preserve immutable `final_summary.json`, `final_logs`, all `run-001` through `run-024`, raw/compact/live/
  scientific/Supervisor evidence, ordinary logs and `/home/dev/ws_iap/{build,install,log}`.
- No deletion, cleanup, chmod, move, archive, backup or external output is authorized.
- Do not weaken source, occupancy, EGO, fused-P5, required-process, GPU or cleanup fail-closed behavior.

## Exit and handoff

Layer 2 exits only when pushed-source-bound `repair-001_summary.json` exists and reports exactly five suites at
8 + 2 + 2 + 4 + 17, all eight required rows PASS, zero skip/disabled, exact two-artifact admission, and no
missing/duplicate/nonzero/count/source condition. Focused runner tests must pass; retained shared build remains
6/6; tracked state is clean; exact isolated source status sees only the two admitted artifacts; pushed divergence
is `0 0`.

Return `ICRA072B_EXACT_ADMISSION_READY_FOR_REVIEW` with pushed HEAD, test counts, summary/log path and hash,
suite/row verdicts, exact admitted-artifact metadata and source-binding result. Only a later Supervisor Review
PASS may close ICRA-072 and issue ICRA-073.
