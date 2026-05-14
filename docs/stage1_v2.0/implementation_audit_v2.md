# IAP v2.0 Implementation Audit v2

Date: 2026-05-13

Scope: 只读审计 (read-only audit)。依据 `src/iap` 当前代码，对照 v2.0 规划
(`iap_codex_implementation_plan_v2_3.tex`)、上次 audit
(`implementation_audit.md`) 及问题映射 PDF
(`iap_next_step_direction_with_issue_mapping.pdf`)。

**代码为绝对真理**——当文档与实现有差异时，以代码为准。

---

## 1. Executive Verdict

Overall status: **PARTIAL → PARTIAL (improved)**.

上次 audit 的 5 个 must-fix 项中，4 个已解决，1 个部分解决。关键改进：

| Issue | Audit v1 | Audit v2 |
|---|---|---|
| Phase 2 validator `fim_add` assumption | BROKEN | **FIXED** |
| `phase2_summary.json` schema mismatch | BROKEN | **FIXED** |
| Demo11 IAP odom freshness/alignment | BROKEN | **FIXED** |
| URG update cost too high (~8881ms) | HIGH RISK | **IMPROVED** (~1230ms) |
| Full official demo11 validation | PENDING | **STILL PENDING** |

新发现: `planner_trajectory_count=0` 是当前阻断 full v2.0 acceptance 的根因,
而不是之前 audited 的 odom/schema/validator 问题。

Per-stage status:

| Stage | Status |
|---|---|
| Stage 0: LiDAR ARAIM PL Stability Repair | COMPLETE |
| Stage 1: Naming Clarification | COMPLETE |
| Stage 2: Advisory FIM Predictor | COMPLETE + diagnostic cleanup |
| Stage 3: Unified PI Cost Adapter | COMPLETE |
| Stage 4: URG + Stage 4.1 perf | IMPLEMENTED, perf ~1230ms |
| v2.0 Acceptance Automation | IMPLEMENTED |
| Full v2.0 Demo11 Acceptance | **FAILED** (planner trajectory) |

---

## 2. Module-by-Module Audit

### Module A: Certified ARAIM Monitor (Stage 0 + GNSS path)

**v2.0 plan coverage**: Section 5 (formulas), Section 7 (Stage 0 tasks).

**Implementation**: Code in `src/iap/integrity/araim.cpp`, `src/iap/integrity/lidar_araim.cpp`,
`src/iap/integrity/integrity_monitor.cpp`。

**Code verification**:

- Bounded LiDAR age risk (`1-e^(-A/tau)` or capped linear): `compute_risk_components()`
  in `lidar_araim.cpp` — **PRESENT**.
- Target keyframe window cap (`K_T=10`): `filter_target_window()` in
  `lidar_araim.cpp` — **PRESENT**.
- Solution-separation variance fallback: `computeSigmaSs()` flow with raw variance,
  floor, fallback diagnostics — **PRESENT** (verified in `lidar_araim.cpp`).
- PL component CSV logging: `lidar_araim_debug.hpp` header includes `sep_term_m`,
  `sigma_ss_term_m`, `sigma_subset_term_m`, `bias_term_m`, gamma components,
  `ss_variance_fallback_flag` — **PRESENT**.
- Certified monitor fusion `PL_mon = max(PL_G, PL_L)`: `integrity_monitor.cpp`
  `run_lidar_araim()` — **UNCHANGED**.
- New invariant test: `test_araim.cpp` now tests `PL_E/PL_N/PL_U/HPL/VPL`
  equals `max(GNSS_certified, LiDAR_certified)` — **ADDED** since last audit.

**Previous audit issues**: None outstanding for this module.

**Status**: **COMPLETE**. Stage 0 implementation matches v2.0 spec. Invariant
regression test added.

---

### Module B: Stage 1 — Naming/Interface Clarification

**v2.0 plan coverage**: Section 2.3 (naming table), Section 7 (Stage 1 tasks).

**Implementation**:

- Semantic alias accessors: `gnss_certified_*`, `lidar_certified_*`,
  `monitor_fused_*`, `monitor_integrity_margin()`, `gnss_advisory_*_proxy`,
  `advisory_predicted_*` — **PRESENT** in `integrity_types.hpp`,
  `integrity_snapshot.hpp`, `future_pl_query_result.hpp`.
- Log labels updated in monitor, extension, evaluator — **PRESENT**.
- Advisory fusion mode now uses `AdvisoryFusionMode { Legacy, FimAdd, Unknown }`
  enum instead of raw strings (CSV/JSON boundaries still emit `legacy`/`fim_add`
  strings) — **IMPROVED** since last audit.

**Previous audit issues**: None outstanding. Stage 1 was already accepted.

**Status**: **COMPLETE**. No behavior changes, only naming/clarity improvements.

---

### Module C: Stage 2 — Advisory FIM Predictor

**v2.0 plan coverage**: Section 5 (GNSS FIM, LiDAR FIM, FIM-add formulas),
Section 7 (Stage 2 tasks), Section 4 (config defaults).

**Sub-module C1: GNSS Advisory FIM**

- `PredictedAraimComputer::predict_advisory_fim()` in `predicted_araim.cpp`:
  4D ENU position + meter-equivalent clock state — **PRESENT**.
- Clock Schur complement: `H_pp - H_pc * (H_cc + lambda_c)^-1 * H_cp` — **PRESENT**.
- Degeneracy handling: `<4 satellites` or non-PSD → `GNSS_DEGENERATE` flag — **PRESENT**.
- PSD check and eigenvalue clamp/zero policy — **PRESENT**.

**Sub-module C2: LiDAR Advisory FIM**

- `LidarObservabilityFim::evaluate_advisory_fim()`: 3x3 translational FIM from
  normals — **PRESENT**.
- `LidarFimPrimitive` struct — **PRESENT**.
- PCA fallback: deterministic voxel-bucket sampling → bounded uniform capping
  (replaced old index-stride sampling) — **IMPROVED** since last audit.
- Configurable PCA parameters: `pca_radius_m`, `pca_max_points`,
  `pca_min_support`, `pca_voxel_sample_m`, `pca_max_primitives`,
  `use_cloud_normals_first` — **PRESENT**.
- Cloud-provided `normal_x/y/z` used first; PCA runs only for points without
  valid normals — **PRESENT**.
- Summary diagnostics: `lidar_pca_primitives_total`, `lidar_pca_valid_normals`,
  `lidar_pca_invalid_normals`, `lidar_pca_support_mean/min` — **PRESENT**.

**Sub-module C3: FIM-add Fusion**

- `Lambda_adv = Lambda_prior + Lambda_G + Lambda_L` — **PRESENT** in
  `future_pl_field_predictor.cpp`.
- `Sigma_adv = inv(Lambda_adv + fim_epsilon * I)` — **PRESENT**.
- HPL from `lambda_max(Sigma_xy)`, VPL from `Sigma_zz` — **PRESENT**.
- Disabled by default (`use_advisory_fim_add=false`) — **PRESENT**.
- Legacy `predict_araim_result()` path preserved — **PRESENT**.

**FIM diagnostic improvements since last audit**:

- Split `fim_regularized` into `fim_epsilon_applied` vs `fim_degeneracy_regularized`.
- Added `fim_fallback_reason` field.
- Summary: `epsilon_applied_count`, `degeneracy_regularized_count`,
  `fallback_reason_histogram` (old `regularized_count` kept as compat alias).
- All propagated through `FuturePLQueryResult` → PL grid interpolation → CSV →
  `phase2_summary.json`.

**Previous audit issues**: None unresolved. Old validator check `PL_H_pred >= gnss_hpl`
was blocking Stage 2 validation — this was a validator bug, not a Stage 2 code bug,
and has been fixed (see Module G).

**Status**: **COMPLETE**. All v2.0 plan Stage 2 requirements met. FIM diagnostics
improved beyond original plan.

---

### Module D: Stage 3 — Unified PI Cost Adapter

**v2.0 plan coverage**: Section 5 (PI formulas), Section 7 (Stage 3 tasks).

**Implementation**:

- `PICostAdapter::evaluate()`: hinge cost `max(0, PL - AL + margin)^2`,
  optional ratio term, unknown penalty — **PRESENT** in `pi_cost_adapter.cpp`.
- PI input selection: `select_pi_advisory_pl()` prefers `fim_add` HPL/VPL,
  falls back to advisory predicted PL, constant_current only with compat flag —
  **PRESENT** in `phase2_planner_integrity_evaluator.cpp`.
- Legacy PI behavior preserved when `use_unified_advisory_pl=false` — **PRESENT**.
- A* front-end and B-spline back-end consume same PI adapter output — **PRESENT**.
- CSV/summary fields: `advisory_hpl_used`, `advisory_vpl_used`,
  `advisory_pl_source`, `im_h_adv`, `im_v_adv`, `im_min_adv`, `pi_hinge_cost`,
  `pi_ratio_cost`, `pi_total_cost`, etc. — **PRESENT**.
- `penalize_unknown_advisory` defaults `true` in evaluator — **PRESENT**.

**Previous audit issues**: None.

**Status**: **COMPLETE**.

---

### Module E: Stage 4 — Unified Risk Grid (URG)

**v2.0 plan coverage**: Section 5 (URG struct, query API, stale handling),
Section 7 (Stage 4 tasks).

**Sub-module E1: URG Core**

- `UnifiedRiskVoxel` struct: `esdf_m`, `occ_prob`, `al_h_m`, `al_v_m`,
  `hal_m`, `val_m`, `hpl_adv_m`, `vpl_adv_m`, `pl_adv_m`, integrity margin
  fields, `pi_cost`, gradient fields, `updated_time_s`, `age_s`, `flags` —
  **ALL PRESENT** in `unified_risk_grid.hpp:31-55`.
- Flags: `VALID_ESDF`, `VALID_OCCUPANCY`, `VALID_AL`, `VALID_ADVISORY_PL`,
  `VALID_PI`, `STALE_PL`, `UNKNOWN_RISK`, `OCCUPIED`, `OUT_OF_RANGE`,
  `FIM_ADD_USED`, `LIDAR_FIM_VALID`, `GNSS_FIM_VALID`, `PI_INPUT_VALID` —
  **PRESENT**.
- `queryRisk()` API with gradient support — **PRESENT**.
- Disabled mode (`phase2_use_unified_risk_grid=false`): zero URG
  queries/updates, no `urg_grid_voxels.csv`, legacy-compatible — **PRESENT**.

**Sub-module E2: URG Staleness (fixed since last audit)**

上次 audit 发现 URG 所有 voxel 都被标记为 stale。现在：
- Per-voxel `updated_time_s` 是 staleness 的 primary source
  (`unified_risk_grid.cpp:436-438`)。
- Interpolation 传播 oldest finite timestamp
  (`oldest_finite_timestamp()` at line 54-65)。
- Global `stamp_s_` 仅作为 fallback。
- Stale penalty ramp `c_unknown * (1 - exp(-dt/tau_stale))` — **PRESENT**.
- 已确认: enabled smoke 中 `urg_stale_count=2601` 而 `urg_query_count=7807`，
  不再是全部 stale。

**Sub-module E3: Stage 4.1 Performance (new since last audit)**

上次 audit 记录的 URG update 瓶颈及修复状态：

| 瓶颈 (上次 audit) | 本版状态 |
|---|---|
| Grid size 25m×25m 过大 | **UNCHANGED** (仍为默认,但可通过 launch arg 缩小) |
| Per-voxel direct PI/FIM/PL queries | **PARTIALLY FIXED**: grid_difference 梯度替代 per-voxel finite difference |
| Local PCA/FIM recomputation | **PARTIALLY FIXED**: improved sampling, but still per-rebuild |
| No incremental update | **NOT YET** |
| CSV export synchronous | **FIXED**: export gated by `phase2_urg_export_voxels` |
| No cancellation checks | **FIXED**: `urg_rebuild_cancel_check_interval` added |
| No timing breakdown | **FIXED**: `urg_time_pl_query_ms`, `urg_time_al_esdf_ms`, `urg_time_pi_ms`, `urg_time_gradient_ms`, `urg_time_csv_ms`, `urg_time_total_ms` |

Grid-difference gradient mode (`phase2_urg_gradient_mode:=grid_difference`):
post-build central differences on already-computed PI costs, replacing per-voxel
finite differences — **PRESENT** in `unified_risk_grid.cpp:321-365`。

**Performance data**:

| Metric | Audit v1 (enabled) | Audit v2 (25m grid) | Audit v2 (11m grid) |
|---|---|---|---|
| `urg_mean_update_ms` | ~8881 | ~1230 | ~39 |
| `urg_p95_update_ms` | ~11893 | ~1627 | ~67 |
| `urg_time_pl_query_ms` | (not available) | ~1134 | ~0.1 |
| `urg_time_al_esdf_ms` | (not available) | ~480 | ~21 |

25m grid update 从 ~8881ms 降到 ~1230ms (7.2x improvement)，但仍超过
1000ms 初步目标。时间主要花在 PL query (1134ms) 和 AL/ESDF (480ms)。

**Previous audit issues resolved**:
- All-voxels-stale bug → **FIXED** (per-voxel timestamps).
- No cancellation checks → **FIXED**.
- No timing breakdown → **FIXED**.
- Heavy per-voxel finite difference → **IMPROVED** (grid_difference mode).
- CSV export blocking → **FIXED** (gated + optional on-update/on-shutdown).

**Remaining E2 gaps**:
- Default 25m grid still >1000ms (v2.0 acceptance `--urg-mean-update-ms-max 1000` FAILS).
- No incremental/adaptive grid update.
- PL query time (1134ms) dominates; AL/ESDF (480ms) secondary.
- Still 100% of voxels rebuilt each update.

**Status**: **IMPLEMENTED + PERFORMANCE IMPROVED** (7x faster), but not yet
meeting 1000ms target at 25m default grid.

---

### Module F: Odom & Preprocessing Infrastructure

**Sub-module F1: Demo11 Odom Freshness Fix (NEW since last audit)**

上次 audit 标记 `demo9_preflight_control_odom_mux` 因跨时钟域年龄计算错误
(121573785s) 拒绝 IAP odometry。现在：

- **Root cause**: IAP odom stamp 在 simulator epoch (2022-07-06)，mux 用
  node wall clock (2026) 计算 freshness。
- **Fix**: `demo3_odom_mux.cpp` 现在用 truth odom stamp 作为 freshness
  reference (`ref_source=truth_odom_stamp`)，只在 truth 不可用时 fallback
  到 `node_now_no_truth`。
- `include/iap/sim/odom_freshness.hpp`: header-only freshness decision helper。
- `test/test_odom_freshness.cpp`: 5 个测试 (truth-stamp acceptance,
  node-now fallback, stale rejection, non-increasing stamp rejection,
  zero-stamp rejection)。
- `demo11` launch default: `iap_odom_freshness_sec=1.0` (mux 自身默认 0.3s)。

**Recorded evidence** (from status.md):

```text
mode=iap_locked valid_iap_streak=77 accepted_iap=77
hist_accepted=77 hist_stale=0 hist_non_increasing=0 hist_zero_stamp=0
```

**Status**: **FIXED**. IAP odom 被持续接受，`valid_iap_streak` 为正。

**Sub-module F2: LocalOccupancy Rolling Eviction (NEW)**

`LocalOccupancyGrid` 增加 rolling eviction，防止 map 在 `max_voxels` 之后
冻结不再接受新 voxel：

- `enable_eviction`, `local_radius_m`, `max_age_s`, `eviction_policy`
  (distance_then_age) — **PRESENT**.
- `test/test_local_occupancy.cpp` — **PRESENT**.
- `phase2_summary.json` diagnostics: `local_occupancy_voxel_count`,
  `local_occupancy_inserted_count`, `local_occupancy_evicted_count`,
  `local_occupancy_rejected_count` — **PRESENT**.

**Status**: **COMPLETE**. Does not modify certified ARAIM, Stage 2/3/4 math,
ROS message schemas, or existing query APIs.

**Sub-module F3: Advisory LiDAR FIM PCA Primitive Sampling (NEW)**

PCA normal generation 从 index-stride sampling 升级为 deterministic
voxel-bucket sampling + bounded uniform capping：

- `include/iap/planner/lidar_observability_fim.hpp`: reusable/testable
  `generate_lidar_fim_primitives()` API — **PRESENT**.
- Cloud normals first, PCA fallback for invalid normals — **PRESENT**.
- Tests: configurable PCA radius, min support boundary, voxel sampling
  dedup, empty cloud handling, cloud normal path — **PRESENT**.

**Status**: **COMPLETE**.

---

### Module G: Phase 2 Summary Schema & Validator

**Previous audit issues**: 这是上次 audit 最大的阻断项。

**Issue G1: Phase 2 validator `fim_add` assumptions → FIXED**

上次 audit: validator 执行 `PL_H_pred >= gnss_hpl`，但 FIM-add 可以有意
产生更 tighter 的 advisory PL。现在:

- `validate_phase2_integrity_eval.py:626`: check 被 `if not fim_add_mode`
  guard 住。
- `--expect-mode legacy|fim_add_urg`: 支持两种 validation mode。
- `fim_add_urg` mode 检查: `advisory_fim.fusion_mode == fim_add`,
  `gnss_fim_valid_count > 0`, finite `hpl_adv/vpl_adv/lambda_adv_trace`。
- Tests in `test_phase2_summary_schema.py:118`: `test_fim_add_allows_advisory_pl_below_gnss_proxy`
  confirms zero failures with `PL_H_pred=1.0 < gnss_hpl=20.0`.

**Issue G2: `phase2_summary.json` schema mismatch → FIXED**

上次 audit: `ana_log.py` 重写 summary 后丢失 validator 需要的 online blocks。
现在:

- `phase2_summary_schema.py`: shared schema with `REQUIRED_ONLINE_SUMMARY_FIELDS`,
  `ADVISORY_FIM_FIELDS`, `URG_FIELDS`, `merge_online_and_offline_summary()`.
- `ana_log.py:2136`: calls `merge_online_and_offline_summary()` — deep merge
  preserves `advisory_fim`, `pi_stage3`, `urg`, `current_consistency_raw`,
  `stage1_capabilities`, etc.
- Test `test_ana_log_merge_preserves_online_validator_schema`: confirms
  `missing_required_online_fields=[]`.

**Issue G3: Validator URG checks → ADDED**

- `check_urg()` checks `urg_query_count`, `urg_front_field_points`,
  `urg_backend_field_points`, finite non-negative values, `urg_active==true`
  when `urg_enabled==true`.
- `--urg-mean-update-ms-max`: enforcement of URG performance threshold.

**Status**: **FIXED**. All three previous audit validator/schema issues resolved.
Validator now correctly understands `fim_add` semantics, schema is stable across
online evaluator → `ana_log.py` → validator pipeline, and URG validation is
supported.

---

### Module H: Planner Integration (A* + B-spline)

**v2.0 plan requirement**: A* front-end and B-spline back-end must consume same
`c_PI` from the unified PI adapter / URG.

**Code verification**:

- Front-field samples store provided `cost` field;
  `queryFrontIntegrityCost()` uses it when finite, falls back to
  `normalizedFrontIntegrityCost(hpl,vpl,hal,val)` — **PRESENT** in
  `sim/.../bspline_optimizer.cpp`.
- Back-end `onIntegrityCostField()` reads `cost`, `grad_x/y/z`, `risk_band`;
  `risk_band == 0` samples skipped — **PRESENT**.
- Both paths receive URG-derived samples when URG is enabled;
  legacy topics `/iap/integrity_cost_field` and
  `/iap/integrity_front_cost_field` preserved — **PRESENT**.

**Previous audit issues**: No code issues. Topic/field backward compatibility
preserved.

**Critical blocking issue**: The most recent v2.0 acceptance runs
(legacy and enabled) both produce `planner_trajectory_count=0`. The terminal
logs show EGO planner errors (`AstarSearch`, `Coord2Index`, "drone is in
obstacle"). This is the reason full demo11 acceptance fails, not the Stage 2-4
integrity pipeline itself.

**Status**: **CODE-COMPLETE, RUNTIME-BLOCKED**. Planner integration code is
correct. The `planner_trajectory_count=0` runtime issue is the current v2.0
acceptance blocker.

---

### Module I: v2.0 Acceptance Automation (NEW)

**New tool**: `tools/stage_v2/run_v2_acceptance.py` (~580 lines).

Features:
- `--mode legacy|enabled|both`: 控制运行哪些模式。
- `--run-duration-s`: 最低 90s (低于 90 直接 FAIL)。
- `--urg-mean-update-ms-max`: URG 性能门限 (default 1000ms)。
- `--reuse-run-dir MODE=PATH`: 对已有 run 做 report-only revalidation。
- Per mode: runs `ana_log.py` → Phase 1 validator → Phase 2 validator.
- Output: `acceptance_report.md` + `acceptance_report.json`.
- Final print: `v2.0 full demo11 acceptance: PASS` or `FAIL` + exact checks.

**Recent full acceptance run result** (from status.md):

```text
v2.0 full demo11 acceptance: FAIL
  - legacy: Phase 1 official validator failed
  - legacy: Phase 2 validator failed
  - legacy: run_duration_s 89.95 < 90.00
  - enabled: Phase 1 official validator failed
  - enabled: Phase 2 validator failed
  - enabled: urg.urg_mean_update_ms 1877.573 > 1000.000
```

失败根因分析:
- **Legacy 和 enabled 都无 planner trajectory** (`planner_trajectory_count=0`).
  这是 Phase 1 failure 的直接原因。
- **Enabled URG 1877ms > 1000ms**. 比之前 smoke 的 1230ms 高，可能是 90s
  full run 有更多 rebuild 或者资源竞争。
- **Phase 2 `current_consistency_raw max_pl_ratio` 超标**:
  legacy 20983532.458, enabled 0.990. Legacy 的极端值 suggest 可能是 PL
  sentinel fallback (1e9) 参与计算。

Enabled run 的 integrity pipeline 本身在工作:
- `advisory_fim.query_count=94877`, `gnss_fim_valid=90856`,
  `lidar_fim_valid=41587`.
- `pi_stage3.selected_source_histogram = {"fim_add": 18}`.
- `urg.urg_query_count=46854`, `urg_active=true`.

**Status**: **IMPLEMENTED**. Acceptance automation 本身完成，但自动化
暴露了 planner trajectory 的 runtime 问题。

---

### Module J: Cross-Stage Invariants

**v2.0 plan requirement** (Section 6): 12 invariants across stages.

**Invariant tests added since last audit**:

| Test | Coverage |
|---|---|
| `test_araim`: monitor fusion invariant | `PL_E/PL_N/PL_U/HPL/VPL == max(GNSS_cert, LiDAR_cert)` |
| `test_future_pl_field_predictor`: Stage 2 disabled → legacy path | `fusion_mode=legacy`, legacy advisory path active |
| `test_future_pl_field_predictor`: enum-to-string | `to_string()` produces `legacy`/`fim_add` |
| `test_future_pl_field_predictor`: epsilon vs degeneracy split | `fim_epsilon_applied` ≠ `fim_degeneracy_regularized` |
| `test_unified_risk_grid`: disabled mode | URG counters zero when disabled |
| `test_phase2_summary_schema.py`: URG disabled | zero counters + no `urg_grid_voxels.csv` |

**Invariant verification** (code-level):

| Invariant | Status |
|---|---|
| Certified monitor fusion = max(GNSS, LiDAR) | **PASS** |
| Advisory output labeled as adv/proxy/advisory | **PASS** |
| FIM-add limited to advisory predictor path | **PASS** |
| Stage 2 does not alter certified monitor math | **PASS** |
| Stage 3 does not alter certified monitor or Stage 2 FIM | **PASS** |
| Stage 4 does not alter certified monitor, Stage 2, or Stage 3 | **PASS** |
| Old topics/messages/CSV fields preserved | **PASS** |
| Disabled modes remain legacy-compatible | **PASS** |
| Stage 2 FIM-add output feeds Stage 3 PI selection | **PASS** |
| Stage 3 PI output feeds Stage 4 URG | **PASS** |
| Stale field does not become zero risk silently | **PASS** (per-voxel fix) |
| Planner A* and B-spline consume same c_PI | **PASS** |

**Status**: **ALL INVARIANTS PASS** at code level. New invariant regression
tests added.

---

## 3. Previous Audit Issues: Resolution Tracking

### Must-Fix Items (from Audit v1 Section 7)

| # | Issue | Status | Evidence |
|---|---|---|---|
| 1 | Update Phase 2 validator for `fim_add` semantics | **RESOLVED** | `validate_phase2_integrity_eval.py`: `fim_add` mode skips conservative check; tests pass |
| 2 | Stabilize `phase2_summary.json` schema | **RESOLVED** | `phase2_summary_schema.py`: shared schema; `merge_online_and_offline_summary()` deep-merges |
| 3 | Fix demo11 IAP odometry freshness/alignment | **RESOLVED** | `demo3_odom_mux.cpp`: truth-domain freshness; 5 unit tests; `valid_iap_streak=77` |
| 4 | Reduce URG update time / make cancellable | **PARTIALLY RESOLVED** | ~8881ms → ~1230ms (7.2x); cancellable; timing breakdown; still >1000ms |
| 5 | Full-duration official demo11 validation | **NOT RESOLVED** | Acceptance automation exists; runs FAIL due to planner_trajectory_count=0 |

### Prioritized Follow-Ups (from Audit v1 Section 7)

| # | Item | Status |
|---|---|---|
| 1 | Validator/schema unification | **DONE** |
| 2 | URG Stage 4.1 performance + shutdown cancellation | **DONE** (perf improved, not final) |
| 3 | Odom freshness/alignment fix | **DONE** |
| 4 | Stage 0 target-window cap safety validation | **NOT DONE** (needs separate safety review) |
| 5 | Invariant regression tests for certified monitor + disabled modes | **DONE** |

---

## 4. Test Coverage (Current)

| Test suite | Tests | Status |
|---|---|---|
| `test_araim` | includes monitor fusion invariant | PASS |
| `test_predicted_araim` | GNSS FIM + clock Schur complement | PASS |
| `test_lidar_observability_fim` | PCA sampling, normal handling | PASS |
| `test_future_pl_field_predictor` | FIM-add, epsilon vs degeneracy | PASS |
| `test_pl_grid` | FIM diagnostic interpolation | PASS |
| `test_pi_cost_adapter` | hinge, ratio, unknown, legacy | PASS |
| `test_unified_risk_grid` | core, staleness, disabled mode | PASS |
| `test_local_occupancy` | rolling eviction | PASS |
| `test_odom_freshness` | 5 freshness scenarios | PASS |
| `test_phase2_summary_schema` (Python) | 5 validation modes | PASS |

Full CTest: **13/13 passed** (last recorded).

---

## 5. Remaining Risks / Gaps

### Blocking v2.0 Acceptance

1. **Planner trajectory count = 0**. Both legacy and enabled 90s runs produce
   no B-spline planner trajectories. Terminal logs show EGO planner
   `AstarSearch`/`Coord2Index`/"drone is in obstacle" errors. This is the
   immediate blocker — without planner trajectories, Phase 1 cannot pass.

2. **URG 25m grid > 1000ms**. Even at ~1230ms (best smoke), the default grid
   exceeds the 1000ms acceptance threshold. PL query time (1134ms) is the
   dominant cost, suggesting per-voxel advisory PL/FIM recomputation is still
   the bottleneck.

### Non-Blocking

3. **Stage 0 target-window safety validation**: The target-window hypothesis
   omission is safety-sensitive but has not been separately validated with a
   dedicated safety test case. This is a process gap, not a code gap.

4. **No incremental URG update**: Full rebuild every time, even when only a
   small region changed. This is a Stage 5 optimization item per the v2.0 plan.

5. **`phase2_integrity_eval_aligned.csv` consistently missing**: The Phase 2
   validator still reports this file as missing across all runs. This may be
   a CSV naming/export issue or an alignment requirement mismatch.

6. **Demo11 shutdown errors in non-core nodes**: `poscmd_2_odom`,
   `pcl_render_node`, `traj_server`, odom visualization nodes still throw
   shutdown errors. These are cosmetic for integrity validation but clutter
   CI/demo logs.

---

## 6. Final Assessment

### What improved since Audit v1

- Validator/schema mismatch: **fully resolved**.
- Odom freshness: **fully resolved** with tests.
- URG staleness bug: **fully resolved**.
- URG performance: **7.2x faster** (~8881ms → ~1230ms), with timing breakdown
  and cancellation support.
- FIM diagnostics: **refined** (epsilon vs degeneracy split, fallback reasons).
- LiDAR FIM PCA: **improved** (deterministic voxel-bucket sampling).
- LocalOccupancy: **new** rolling eviction.
- Invariant tests: **added** for monitor fusion, disabled modes, enum semantics.
- Acceptance automation: **new** with full v2.0 pipeline.

### What did not improve

- Full v2.0 demo11 acceptance still **FAILS**.
- URG still above 1000ms at default 25m grid.
- Planner trajectory production still broken (zero trajectories).
- Stage 0 safety validation not performed.

### Recommendation

The implementation is **stage-complete** for Stages 0-4 at code level, with
the validator/schema/odom gaps from Audit v1 all resolved. The immediate next
step should be:

1. **Debug planner trajectory production** (EGO `AstarSearch` failures) — this
   gates both Phase 1 and Phase 2 acceptance and is unrelated to the integrity
   pipeline itself.
2. **Reduce URG PL query time** — either through caching, batched FIM evaluation,
   or adaptive grid sizing.
3. **Re-run v2.0 acceptance** after planner fix, targeting `<1000ms` URG mean
   update time.
