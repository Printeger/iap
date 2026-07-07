# Safety Planner P0-P5 Integration Test Report

> 本报告记录 Safety Planner 自主验证第一步：L0 预检查 + Phase 0 `B0-1` fused nominal baseline lock；追加记录 `B0-2` open-sky baseline lock。

## 1. 测试环境

| 字段 | 值 |
|---|---|
| 日期 | `2026-07-07` UTC |
| 操作者 / Agent | Codex |
| 机器 / 容器 | `mint-X` |
| OS / ROS | Linux `6.17.0-14-generic`; ROS `jazzy` (`ros2 --version` is not a valid CLI flag in this environment) |
| Workspace | `/home/dev/ws_iap` |
| Package | `iap` |
| GPU/CPU 备注 | CPU cores: `20`; GPU not inspected |

## 2. 代码与构建

| 字段 | 值 |
|---|---|
| Code-under-test commit hash | `9ba840648f64e8c958bceb0237d7ecc8bf070fc7` |
| Branch | `dev/iap` |
| Precheck dirty worktree? | `no`; initial status was `## dev/iap...origin/dev/iap` |
| Recent commits | `9ba8406 chore: save iap workspace state`; `e244904 docs: make safety planner validation autonomous`; `15ef47d fix: stabilize P0 risk grid health freshness` |
| Build command | `source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --packages-select iap --event-handlers console_direct+` |
| Build result | `PASS` |
| Unit test command | See commands below |
| Unit test result | `PASS`: `iap` 4/4, `ego_planner` 5/5, `bspline_opt` 1/1, `path_searching` 1/1 |

```bash
ctest --test-dir build/iap -R "test_(risk_grid_map|predictor_module|unified_risk_grid|future_pl_field_predictor)" --output-on-failure
ctest --test-dir build/ego_planner -R "test_(p0_risk_grid_runtime|p5_runtime_integrity_gate|p2_candidate_ranking|p3_reference_bias|planning_risk_context)" --output-on-failure
ctest --test-dir build/bspline_opt -R test_p1_integrity_cost --output-on-failure
ctest --test-dir build/path_searching -R test_p4_risk_astar --output-on-failure
```

## 3. Launch 与参数

| 字段 | 值 |
|---|---|
| Experiment ID | `B0-1` |
| Experiment preset | `baseline_fused_nominal_off` |
| Scenario | `fused_nominal` |
| Launch command | `ros2 launch iap test_planner.launch.py experiment:=baseline_fused_nominal_off run_duration_s:=90 validation_duration_s:=90 start_rviz:=false run_validator:=true record_bag:=true` |
| Run duration | `90s` |
| Validation duration | `90s` |
| Export dir | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fused_nominal_1783429126172` |
| Bag dir | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_fused_nominal_20260707T125846Z` |
| Analyzer command | `python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py --experiment-id B0-1 --export-dir src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fused_nominal_1783429126172 --bag-dir src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_fused_nominal_20260707T125846Z --fail-on-threshold` |
| Analyzer summary | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fused_nominal_1783429126172/metadata/safety_planner_analysis_summary.json`; status `PASS` |
| Rosbag summary | Storage `mcap`; size `1.4 GiB`; duration `90.026153198s`; messages `521753` |

### 参数表

| 参数 | 值 | 说明 |
|---|---:|---|
| `planner_safety_profile` | `off` | B0-1 baseline lock |
| `p0.enable_risk_grid` | `false` | P0 disabled |
| `p0.gnss_epoch_max_age_s` | not recorded in manifest | Not part of B0-1 hard criteria |
| `planner_enable_p1` | `false` | P1 disabled |
| `planner_enable_p2` | `false` | P2 disabled |
| `planner_enable_p3_local` | `false` | P3 local disabled |
| `planner_enable_p3_global` | `false` | P3 global disabled |
| `planner_enable_p4` | `false` | P4 disabled |
| `planner_enable_p5_runtime` | `false` | P5 runtime gate disabled |
| `planner_enable_p5_final` | `false` | P5 final gate disabled |
| `p1.metrics_only` | not recorded in manifest | P1 disabled |
| `p2.metrics_only` | not recorded in manifest | P2 disabled |
| `p5.pred_alert_limit_mode` | not recorded in manifest | P5 disabled |

## 4. Topic Health

| Topic | Expected | Observed count / Hz | Status | Notes |
|---|---|---:|---|---|
| `/iap/integrity` | continuous | `844 / 9.375 Hz` | `PASS` | Validator saw `842` messages and passed |
| `/sim/drone_0/lidar_body` | continuous | `846 / 9.397 Hz` | `PASS` | Bag count from metadata |
| `/drone_0_planning/bspline` | planner-dependent | `24 / 0.267 Hz` | `PASS` | Planner published trajectory messages |
| `/planning/risk_grid_health` | P0 debug on | `0` | `N/A` | Expected with P0 disabled |
| `/planning/integrity_gate_status` | P5 debug on | `0` | `PASS` | Expected absent/zero with P5 disabled; no P5 action rows |
| `/iap/rviz/predicted_pl_cloud` | P0 on | `0` | `N/A` | Expected with P0 disabled |
| `/iap/rviz/risk_validity_cloud` | P0 on | `0` | `N/A` | Expected with P0 disabled |
| `/iap/rviz/p5_gate_status` | P5 on | `0` | `PASS` | Expected with P5 disabled |
| `/iap/rviz/p1_integrity_metrics` | P1 viz on | not in bag metadata | `N/A` | Expected with P1 disabled |
| `/iap/rviz/p2_candidate_trajectories` | P2 viz on | not in bag metadata | `N/A` | Expected with P2 disabled |
| `/iap/rviz/p3_reference_bias` | P3 viz on | not in bag metadata | `N/A` | Expected with P3 disabled |
| `/iap/rviz/p4_astar_guides` | P4 viz on | not in bag metadata | `N/A` | Expected with P4 disabled |

## 5. 实验结果表

| Experiment ID | Status | Validator | Analyzer | Key evidence | Failure / warning | Next branch |
|---|---|---|---|---|---|---|
| B0-1 | `PASS` | `passed=true`, failures `[]`, message_count `842` | `PASS`, failures `[]`, warnings `[]`, inconclusive `[]` | Manifest safety switches all false; core topics non-zero; P5 status rows `0`; integrity CSV rows `842` | No failure observed | `B0-2 open-sky baseline` |

## 6. B0-1 / Phase 0 Baseline Lock 验收

| Criterion | Result | Evidence |
|---|---|---|
| Validator summary `passed=true` | `PASS` | `test_planner_validation_summary.json` reports `passed: true` |
| `planner_safety_profile=off` | `PASS` | Manifest |
| `p0.enable_risk_grid=false` | `PASS` | Manifest |
| P1-P5 planner switches all false | `PASS` | Manifest records all `planner_enable_*` fields as `false` |
| `/iap/integrity` continuous | `PASS` | Bag count `844`, analyzer Hz `9.375` |
| `/sim/drone_0/lidar_body` continuous | `PASS` | Bag count `846`, analyzer Hz `9.397` |
| `/drone_0_planning/bspline` present after planner run | `PASS` | Bag count `24` |
| No P5 replan/emergency/final gate behavior | `PASS` | `/planning/integrity_gate_status` count `0`, analyzer `bad_action_count=0` |
| Shutdown SIGINT teardown noise is not hard fail | `PASS` | Launch command exited `0`; validator passed; teardown exceptions occurred after SIGINT |

## 7. 关键图表清单

| 图表 | 路径 | 结论 |
|---|---|---|
| B0-1 integrity HPL/VPL timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fused_nominal_1783429126172/figures/b0_1_integrity_hpl_vpl_timeline.png` | 842 validator samples plotted; HPL mean `5.103m`, max `10.319m`; VPL mean `14.515m`, max `17.268m` |
| B0-1 topic counts CSV | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fused_nominal_1783429126172/csv/b0_1_topic_counts.csv` | Core topic health evidence for pass/fail decision |

## 8. 失败案例分析

No failure observed.

Non-blocking teardown note: after the validator passed and the launch command began SIGINT shutdown, several visualization/control helper nodes reported `RCLError`, `std::system_error`, or segfault-style exit codes. Per the test plan, this is not a B0-1 hard fail because the launch command exited `0` and validator passed.

## 9. Remaining Issues

| Issue | Severity | Owner | Blocking experiment | Next action |
|---|---|---|---|---|
| SIGINT teardown exceptions in helper nodes | Low | Planner/sim runtime | None for B0-1 | Track separately if teardown stability becomes a CI requirement |
| Analyzer is newly implemented for B0-1 evidence | Low | dev_planner | None for B0-1 | Extend in later phases for P0/P5/P1-P4 debug CSV and richer action timelines |

## 10. Next Actions

1. B0-1 satisfies Phase 0 baseline lock criteria.
2. Proceed to B0-2 open-sky baseline when ready.
3. Do not run later phases until B0-2/B0-3/B0-4 complete in order.

## 11. B0-2 Open-Sky Baseline Validation

### 11.1 Repo State

| Field | Value |
|---|---|
| Initial status before B0-2 | `## dev/iap...origin/dev/iap [ahead 1]` |
| Initial HEAD | `1592da25a6a2cd066978481d8fbcf12a3fa421df` |
| Recent commits | `1592da2 docs: record safety planner B0-1 baseline validation`; `9ba8406 chore: save iap workspace state`; `e244904 docs: make safety planner validation autonomous` |
| Accepted rerun code-under-test | Initial HEAD plus local working tree changes in `apps/demo4_lidar_body_bridge.cpp`, `launch/test_planner.launch.py`, and `scripts/dev_planner/analyze_safety_planner_run.py` |
| Build after runtime/analyzer/map updates | `PASS`: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --packages-select iap --event-handlers console_direct+` |

### 11.2 Launch And Artifacts

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch iap test_planner.launch.py \
  experiment:=baseline_fused_nominal_off \
  scenario:=gnss_open_sky \
  run_duration_s:=90 \
  validation_duration_s:=90 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

| Field | Value |
|---|---|
| Experiment ID | `B0-2` |
| Experiment preset | `baseline_fused_nominal_off` |
| Scenario | `gnss_open_sky` |
| Launch result | `PASS`: command exited `0` |
| Export dir | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_gnss_open_sky_1783432749389` |
| Bag dir | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_gnss_open_sky_20260707T135909Z` |
| Rosbag summary | Storage `mcap`; size `339.2 MiB`; duration `89.924103482s`; messages `521522` |

### 11.3 Validator And Analyzer

| Check | Result | Evidence |
|---|---|---|
| Validator summary | `PASS` | `passed=true`, `failures=[]`, `message_count=883` |
| Required fusion mode | `PASS` | `required_fusion_mode=gnss_only` |
| Required final source | `PASS` | `required_final_source=GNSS` |
| GNSS/fallback validity | `PASS` | `gnss_valid_seen=true`, `fallback_valid_seen=true`, `lidar_valid_seen=false` |
| Safety profile | `PASS` | `planner_safety_profile=off` |
| P0/P1/P2/P3/P4/P5 switches | `PASS` | `p0.enable_risk_grid=false`; all `planner_enable_*` fields false |

Analyzer command:

```bash
python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py \
  --experiment-id B0-2 \
  --export-dir src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_gnss_open_sky_1783432749389 \
  --bag-dir src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_gnss_open_sky_20260707T135909Z \
  --fail-on-threshold
```

| Field | Value |
|---|---|
| Analyzer status | `PASS` |
| Analyzer summary | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_gnss_open_sky_1783432749389/metadata/safety_planner_analysis_summary.json` |
| Failures / inconclusive / warnings | `[]` / `[]` / `[]` |
| Integrity CSV rows | `883` |
| Integrity HPL | min `4.876m`; mean `5.081m`; max `5.647m` |
| Integrity VPL | min `13.753m`; mean `14.518m`; max `17.265m` |
| P5 summary | `status_rows=0`, `bad_action_count=0`, `action_counts={}` |
| Analyzer limitation | Generated artifact filenames still contain `b0_1`; `next_debug_branch` still returns `continue_to_B0-2_open_sky_baseline` on pass |
| Manual next branch | `B0-3 corridor baseline` |

### 11.4 Topic Health

| Topic | Expected | Count | Hz | Span | Coverage | Max gap | Status |
|---|---|---:|---:|---:|---:|---:|---|
| `/iap/integrity` | continuous | `885` | `9.841633` | `88.410846s` | `0.983172` | `0.161068s` | `PASS` |
| `/sim/drone_0/lidar_body` | continuous | `897` | `9.975079` | `89.599674s` | `0.996392` | `0.101106s` | `PASS` |
| `/drone_0_planning/bspline` | planner-dependent | `18` | `0.200169` | `17.308410s` | `0.192478` | `1.019219s` | `PASS` |
| `/planning/integrity_gate_status` | absent-or-zero when P5 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |

### 11.5 Result Row

| Experiment ID | Status | Validator | Analyzer | Key evidence | Failure / warning | Next branch |
|---|---|---|---|---|---|---|
| B0-2 | `PASS` | `passed=true`, failures `[]`, message_count `883` | `PASS`, failures `[]`, warnings `[]`, inconclusive `[]` | Manifest safety switches all false; `/iap/integrity` and `/sim/drone_0/lidar_body` continuous; `/drone_0_planning/bspline` present; P5 status rows `0` | Pre-fix attempts exposed continuity failures; accepted rerun shows no remaining B0-2 failure | `B0-3 corridor baseline` |

### 11.6 B0-2 / Phase 0 Baseline Lock Acceptance

| Criterion | Result | Evidence |
|---|---|---|
| Launch exits `0` | `PASS` | 90s B0-2 launch completed with exit code `0` |
| Validator summary `passed=true` | `PASS` | `test_planner_validation_summary.json` reports `passed: true` |
| `planner_safety_profile=off` | `PASS` | Manifest |
| `p0.enable_risk_grid=false` | `PASS` | Manifest |
| P1-P5 planner switches all false | `PASS` | Manifest records all `planner_enable_*` fields as `false` |
| `/iap/integrity` continuous | `PASS` | Analyzer coverage `0.983172`, max gap `0.161068s` |
| `/sim/drone_0/lidar_body` continuous | `PASS` | Analyzer coverage `0.996392`, max gap `0.101106s` |
| `/drone_0_planning/bspline` present after planner run | `PASS` | Bag count `18` |
| No P5 replan/emergency/final gate behavior | `PASS` | `/planning/integrity_gate_status` count `0`, analyzer `bad_action_count=0` |
| Shutdown SIGINT teardown noise is not hard fail | `PASS` | Launch command exited `0`; validator and analyzer passed; teardown exceptions occurred after SIGINT |

### 11.7 Key Artifacts

| Artifact | Path | Conclusion |
|---|---|---|
| Validator summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_gnss_open_sky_1783432749389/test_planner_validation_summary.json` | Validator passed with `883` messages |
| Analyzer summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_gnss_open_sky_1783432749389/metadata/safety_planner_analysis_summary.json` | Analyzer status `PASS` |
| Integrity HPL/VPL timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_gnss_open_sky_1783432749389/figures/b0_1_integrity_hpl_vpl_timeline.png` | 883 integrity samples plotted; filename still uses `b0_1` due analyzer limitation |
| Topic counts CSV | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_gnss_open_sky_1783432749389/csv/b0_1_topic_counts.csv` | Continuity timing evidence for B0-2 acceptance; filename still uses `b0_1` due analyzer limitation |
| Rosbag | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_gnss_open_sky_20260707T135909Z` | Core topics recorded across the run |

### 11.8 Failure Analysis

Two earlier B0-2 attempts were not accepted:

| Attempt | Export / bag | Observed result | Analysis |
|---|---|---|---|
| Pre-fix open-sky run | Export `..._1783432121137`; bag `..._20260707T134841Z` | Launch `0` and validator passed, but post-fix analyzer marks `FAIL` | `/iap/integrity` coverage `0.170`; `/sim/drone_0/lidar_body` coverage `0.182`; stream continuity was not sufficient for B0-2 |
| Bridge-only hardening rerun | Export `..._1783432572282`; bag `..._20260707T135612Z` | Launch `0` and validator passed, but analyzer marks `FAIL` | Raw LiDAR had `897` scans but only the first `165` were nonempty; the open-sky map allowed the vehicle to leave all LiDAR-visible structure |

Fixes applied before the accepted rerun:

1. Added pending-cloud buffering in `demo4_lidar_body_bridge` so clouds slightly newer than the latest odometry are converted when matching odometry arrives.
2. Added analyzer continuity gates for required continuous topics using bag timestamp span, coverage ratio, and max gap.
3. Added a low corridor floor to the `gnss_open_sky` map preset so the scenario remains open-sky for planner safety but keeps LiDAR odometry fed for the full run.

No failure observed in the accepted B0-2 rerun.

### 11.9 Remaining Issues

| Issue | Severity | Owner | Blocking experiment | Next action |
|---|---|---|---|---|
| SIGINT teardown exceptions in helper nodes | Low | Planner/sim runtime | None for B0-2 | Track separately if teardown stability becomes a CI requirement |
| Analyzer artifact filenames still contain `b0_1` | Low | dev_planner | None for B0-2 | Rename artifacts when generalizing the analyzer beyond B0-1/B0-2 |
| Analyzer `next_debug_branch` still returns `continue_to_B0-2_open_sky_baseline` on B0-2 pass | Low | dev_planner | None for B0-2 | Treat manual next branch as `B0-3 corridor baseline` |
| LiDAR bridge race hardening has no standalone unit test | Medium | dev_planner/runtime | None for B0-2 after accepted rerun | Add a focused bridge timing test if this bridge becomes CI-critical |

### 11.10 Next Actions

1. B0-2 satisfies Phase 0 open-sky baseline lock criteria.
2. Manual next branch is `B0-3 corridor baseline`.
3. Do not run B0-3, P0, P5, or `all` until explicitly requested.
