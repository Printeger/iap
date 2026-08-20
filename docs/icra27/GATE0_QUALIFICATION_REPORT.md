# ICRA Gate 0 Qualification Report

> Run date: 2026-08-16 UTC
> Branch: `dev/icra`
> Documentation reconciliation commit: `8d4ec35ac80445bfeb5998f37bef3efd7654e7ab`
> Nature: read-only qualification evidence, not a P2/P5 algorithm result or certification claim.

## 1. Outcome

- **Gate 0A: `NO-GO-P2`** — all 378 recorded planning attempts had exactly one base `distinctiveTrajs()` candidate and exactly one rebound optimizer success. No run contained a same-attempt `generated>=2 && optimizer_success>=2` qualification set.
- **Gate 0B: `P0_PERFORMANCE_GATE_FAIL`** — 100 distinct refresh-callback records were captured, but none produced a successful generation. The runtime first reported `message_stamp_unavailable` and then `snapshot_unavailable`; `refresh_query_count` remained zero, so the fixed 76,800-query workload and its latency percentiles were not measured.
- **Overall: `NO-GO-P2`** — Gate 0A has highest priority. Stop the P2 conference mainline and revise the next task to a P0+P5 backup-paper plan.
- **Disk: `CAMPAIGN_DISK_NO_GO`** — the workspace has 32 GiB available, below the frozen 40 GiB campaign threshold. This independently blocks any formal campaign.

## 2. Frozen protocol

Gate 0A ran the existing `p1_fork_fused_v1`, `p1_fork_fused_mirror_v1`, and `p1_fork_symmetric_null_v1` scenarios three times each. Every run used logical seed 11 (`forest=11`, `GNSS=20260011`, terminal feature seed `11022`), no bag, no RViz, a 90 s launch duration, and an 85 s validation window.

The effective manifests record P0/P1/P2/P3/P4/P5 false, all P1 collision fanout/supplement controls false or zero, and `manager/use_distinctive_trajs=true`. The mirror runs retain `p1_fixture_mirror_y=true` for geometry while the explicit manager override keeps `manager/p1_collision_fanout_mirror_y=false`.

Gate 0B ran one no-bag `gnss_open_sky` P0-only launch for 60 s with `30x30x6 m`, `0.75 m`, six horizons from `0.0` through `2.5 s`, 0.5 s refresh, worker count 1, and occupied-voxel skip enabled. A separate subscriber captured only `/planning/risk_grid_health` JSON.

## 3. Gate 0A evidence

| Run | Attempts | Generated per attempt | Optimizer inputs per attempt | Successes per attempt | Selected reached EGO/update/publish | Qualified sets |
|---|---:|---:|---:|---:|---:|---:|
| primary-r1 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| primary-r2 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| primary-r3 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| mirror-r1 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| mirror-r2 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| mirror-r3 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| flat-null-r1 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| flat-null-r2 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| flat-null-r3 | 42 | 1 | 1 | 1 | 42/42 | 0 |
| **Total** | **378** | **378 candidates** | **378 inputs** | **378 successes** | **378/378** | **0** |

There was no P1 fanout/supplement event, optimizer input count matched base generated count, all attempts had nonzero identity, and all selected singleton candidates continued through refinement/feasibility, `updateTrajInfo`, and normal B-spline publication. The failure is therefore specifically the absence of a comparable natural base candidate set, not optimizer failure or lineage loss.

Per-candidate rows are in `results/icra27/gate0/candidate_qualification.csv`. Full generated, optimizer-input, optimized, selected, and post-refinement matrices are in `candidate_control_points.csv`, with canonical `.17g` Python serialization and SHA256.

## 4. Gate 0B evidence

| Metric | Result |
|---|---:|
| Captured distinct callback records | 100 |
| Successful distinct generations | 0 |
| Failed refresh records | 100 |
| Ready ratio | 0.0 |
| Stale ratio | 1.0 |
| Expected query count | 76,800 |
| Observed refresh query count | 0 |
| p50 / p95 / max | unavailable |

Because the workload never reached provider evaluation, this report does not interpret the small failed-callback durations as full-grid latency. The frozen gate fails on successful-generation count and query shape before any latency comparison. Per the protocol, only these follow-up levers are proposed: diagnose snapshot availability first, then separately evaluate worker count, ROI, horizons, or refresh period; no parameter was changed and no automatic retry was performed.

## 5. Instrumentation isolation

`Gate0QualificationWriter` is disabled by default and accepts copied scalar/matrix evidence. It does not reference the candidate generator, optimizer APIs, P2 comparator, refinement algorithm, or P5 decision/action. Hooks only append after existing events. The launch change separately resolves geometry mirror and manager fanout mirror, with a regression preserving legacy fallback when no manager override is explicit.

No P2 scoring/winner rule, P5 decision, candidate generation algorithm, P0 update algorithm, collision rule, dynamics rule, or safety action was modified.

## 6. Disk and external dependency closure

The refreshed disk snapshot is: workspace 648 GiB total, 583 GiB used, 32 GiB available (95%); repository results 104 GiB; workspace log 180 MiB; build 3.8 GiB; install 353 MiB. `disk_archive_candidates.csv` contains 163 candidate/aggregate rows. No existing data was deleted, moved, or compressed.

`/home/dev/ws_iap/src/gnss_comm` remains unchanged. Its 43 files (249,746 logical bytes) were archived outside Git to `backups/icra27-baseline-20260816/gnss_comm-closure/gnss_comm-20260816.tar` (296,960 bytes, SHA256 `821f3fadc9b46567442fc7765e68183d0dcd8710f55a0c591e707e26ea87011d`). Tar listing, source-list equality, SHA256, environment, and `ros2 doctor --report` checks passed; all closure artifacts are read-only.

## 7. Artifact hashes

| Artifact | SHA256 |
|---|---|
| `candidate_qualification.csv` | `06cf1273bc6fb7610f9f7959db0bc4eea14e34bbee4223e8eaf992cadae66b1c` |
| `candidate_control_points.csv` | `7833d58ff055b9c932327296748832ac0de5445507163ae35e8ad811f051c39b` |
| `effective_config.json` | `61b9a8ca08fa273e2c9dc3adfb395dd4ad2528b9718ba8b3e6955ad5d1dbdf13` |
| `p0_full_grid_benchmark.csv` | `94e083ffe9e69570516fecb49d928b45d9162862a87fd9c4e9644225af6c0e5f` |
| `p0_full_grid_summary.json` | `698a75211c9b48a50318697ba778238b701137d852ef420f45b05e78cf881715` |
| `disk_archive_candidates.csv` | `66aefad046fad33108cf5a8ef4473befabac5c904875d5b4bde7dbefef781095` |

Raw stdout, runner/resolved-runtime manifests, validator output, runtime/export directories, capture JSONL, and raw event/control-point CSVs remain under `results/icra27/gate0/raw-20260816-v3/` (548 files, 41 MiB) and are intentionally not bulk-added to Git. The failed preflight `raw-20260816-v2/` is retained separately and is not used by the aggregate evidence.

## 8. Go/No-Go action

Do not implement immutable P2 batch identity or alter P2 scoring on this branch after this result. The unique next task is to revise the conference route to P0+P5, while treating Gate 0B snapshot availability and the independent campaign disk gate as explicit blockers. Any proposal to retain P2 requires human review of an explicitly disclosed upstream controlled fixture; this task did not create one.

## 9. 2026-08-20 post-report disposition

本节记录报告后的范围迁移，不改写第 1–8 节的运行事实。`378/378 singleton`、`NO-GO-P2`、P0 的零成功 generation 和 `CAMPAIGN_DISK_NO_GO` 仍是历史有效结论。

后续源码审计表明，Gate 0A 的早期碰撞扫描约在 seed 前 2/3 停止。已进入障碍但尚未观察到出口的情况被留作零个闭合 segment；后续 optimizer collision recheck 仍可能触发 rebound。

因此，`collision_segment_count=0` 不能单独证明 seed 没有碰撞。该解释不改变 P2 结论：378 个 attempt 仍全部只有一个可比较候选，P2 的 treatment domain 没有形成。

ICRA 的新开发目标是条件式 `P0 -> P4 -> P5`，同时保留 `P0+P5` 作为须经新 Supervisor 裁定才能启用的 contingency。该目标不是本报告测得的新结果。

当前状态为 `P0 BLOCKED/UNQUALIFIED`、`P4 NOT_QUALIFIED`、`P5 IMPLEMENTED-BUT-UNQUALIFIED`。P4 Gate-0、P4 到 B-spline lineage 和 P5 final/runtime 均尚未获得运行资格证据。

ICRA-004 仍只处理 GPU preflight 与 P0 smoke，且在该 smoke 中关闭 P4/P5。P0 Gate-0B 通过并经 Supervisor review 前，不开始 P4 生产代码。
