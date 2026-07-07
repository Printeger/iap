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

## B0-3 Corridor Baseline Validation

### Repo State And Environment

| Field | Value |
|---|---|
| Date | `2026-07-07` UTC |
| Operator / Agent | Codex |
| Machine / container | `mint-X` |
| Workspace | `/home/dev/ws_iap` |
| Package repo | `/home/dev/ws_iap/src/iap` |
| Branch | `dev/iap` |
| Repo protection action | Amended unpushed B0-2 commit message to `docs: record safety planner B0-2 baseline validation`, then pushed `dev/iap` to `origin/dev/iap` before running B0-3 |
| Commit after repo protection | `72d31edb1cafb1db5e3a4c7cdd5fe6a5f50449f2` |
| Clean/dirty status after repo protection | `clean`; `git status -sb` reported `## dev/iap...origin/dev/iap` |
| Recent commits after repo protection | `72d31ed docs: record safety planner B0-2 baseline validation`; `1592da2 docs: record safety planner B0-1 baseline validation`; `9ba8406 chore: save iap workspace state` |
| Source command | `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| Build command | Not rerun for B0-3; reused existing install from the pushed B0-2 baseline state |

### Launch And Artifacts

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch iap test_planner.launch.py \
  experiment:=baseline_corridor_off \
  scenario:=lidar_corridor_degenerate \
  run_duration_s:=90 \
  validation_duration_s:=90 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

| Field | Value |
|---|---|
| Experiment ID | `B0-3` |
| Experiment preset | `baseline_corridor_off` |
| Scenario | `lidar_corridor_degenerate` |
| Launch result | `FAIL` for B0-3 criteria: launch process exited `0`, but validator exited `2` and failed |
| Export dir | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783435708588` |
| Bag dir | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_corridor_off_lidar_corridor_degenerate_20260707T144828Z` |
| Rosbag summary | Storage `mcap`; size `569.3 MiB`; duration `90.020732042s`; messages `506647` |
| Runtime note | `iap_rosnode` died early with exit code `-6` after `cudaErrorNoDevice`, `frame doesn't have points on GPU`, and `GPU points/covs not allocated!!` messages |

### Parameter And Manifest Evidence

| Parameter | Manifest value | Result |
|---|---:|---|
| `planner_safety_profile` | `off` | `PASS` |
| `p0.enable_risk_grid` | `false` | `PASS` |
| `planner_enable_p1` | `false` | `PASS` |
| `planner_enable_p2` | `false` | `PASS` |
| `planner_enable_p3_local` | `false` | `PASS` |
| `planner_enable_p3_global` | `false` | `PASS` |
| `planner_enable_p4` | `false` | `PASS` |
| `planner_enable_p5_runtime` | `false` | `PASS` |
| `planner_enable_p5_final` | `false` | `PASS` |

### Validator And Analyzer

| Check | Result | Evidence |
|---|---|---|
| Validator summary | `FAIL` | `passed=false`, `message_count=0` |
| Validator failures | `FAIL` | `received 0 integrity messages, expected >= 10`; `lidar_valid was never true`; `fallback_valid was never true` |
| Required fusion mode | `PASS` configuration only | `required_fusion_mode=lidar_only` |
| Required final source | `PASS` configuration only | `required_final_source=LIDAR` |
| Analyzer status | `FAIL` | `status=FAIL`, `passed=false`, command exited `2` with `--fail-on-threshold` |
| Analyzer summary | `FAIL` | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783435708588/metadata/safety_planner_analysis_summary.json` |
| Analyzer failures | `FAIL` | `validator summary passed is not true`; `required topic /iap/integrity is missing or not continuous`; `test_planner_integrity_validation.csv has no data rows` |
| Analyzer warnings | `WARN` | `integrity HPL/VPL timeline was not generated because no plottable rows were available` |
| P5 summary | `PASS` | `status_rows=0`, `bad_action_count=0`, `action_counts={}` |
| Analyzer artifact naming | `WARN` | Topic-count artifact still named `csv/b0_1_topic_counts.csv`; recorded as tool naming limitation, not the hard failure |

Analyzer command:

```bash
python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py \
  --experiment-id B0-3 \
  --export-dir /home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783435708588 \
  --bag-dir /home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_corridor_off_lidar_corridor_degenerate_20260707T144828Z \
  --fail-on-threshold
```

### Topic Health

| Topic | Expected | Count | Hz | Span | Coverage | Max gap | Status |
|---|---|---:|---:|---:|---:|---:|---|
| `/iap/integrity` | continuous | `0` | `0.000000` | n/a | n/a | n/a | `FAIL` |
| `/sim/drone_0/lidar_body` | continuous | `897` | `9.964371` | `89.599606s` | `0.995322` | `0.100579s` | `PASS` |
| `/drone_0_planning/bspline` | planner-dependent | `18` | `0.199954` | `17.264801s` | `0.191787` | `1.020563s` | `PASS` |
| `/planning/integrity_gate_status` | absent-or-zero when P5 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/drone_0_visual_slam/odom` | IAP odometry available | `0` | `0.000000` | n/a | n/a | n/a | `FAIL` |
| `/iap/rviz/risk_grid_health` | P0 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/predicted_pl_cloud` | P0 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/risk_validity_cloud` | P0 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/p5_gate_status` | P5 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |

### Result Row

| Experiment ID | Status | Validator | Analyzer | Key evidence | Failure / warning | Next branch |
|---|---|---|---|---|---|---|
| B0-3 | `FAIL` | `passed=false`, failures include zero integrity messages, message_count `0` | `FAIL`, failures include missing/non-continuous `/iap/integrity` and empty integrity CSV | Manifest safety switches all false; `/sim/drone_0/lidar_body` continuous; `/drone_0_planning/bspline` present; P5 status rows `0` | `iap_rosnode` died early; `/iap/integrity` and `/drone_0_visual_slam/odom` recorded `0` messages | Debug corridor baseline / LiDAR / odom / planner / analyzer first |

### B0-3 Acceptance Matrix

| Criterion | Result | Evidence |
|---|---|---|
| Launch exits `0` | `PASS` | `ros2 launch` command returned exit code `0` |
| Validator summary `passed=true` | `FAIL` | `test_planner_validation_summary.json` reports `passed: false` |
| Analyzer status `PASS` or only non-blocking warning | `FAIL` | Analyzer status `FAIL`; command exited `2` |
| `planner_safety_profile=off` | `PASS` | Manifest |
| `p0.enable_risk_grid=false` | `PASS` | Manifest |
| P1-P5 planner switches all false | `PASS` | Manifest records all `planner_enable_*` fields as `false` |
| `/iap/integrity` continuous | `FAIL` | Bag count `0`; analyzer topic health `FAIL` |
| `/sim/drone_0/lidar_body` continuous and nonempty | `PASS` | Analyzer count `897`, coverage `0.995322`, max gap `0.100579s` |
| `/drone_0_planning/bspline` exists | `PASS` | Bag count `18` |
| `/planning/integrity_gate_status` has `0` rows or no P5 action | `PASS` | Analyzer `status_rows=0`, `bad_action_count=0` |
| No P0 risk-grid behavior appears | `PASS` | P0 disabled in manifest; P0/RViz risk topics recorded `0` messages |
| P2 behavior disabled | `PASS` | `planner_enable_p2=false`; no P2 evidence required for this baseline |
| P1 behavior disabled | `PASS` | `planner_enable_p1=false`; no P1 evidence required for this baseline |

### Key Artifacts

| Artifact | Path | Conclusion |
|---|---|---|
| Manifest JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783435708588/test_planner_manifest.json` | Safety Planner fully off |
| Validator summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783435708588/test_planner_validation_summary.json` | Validator failed with `0` integrity messages |
| Integrity validation CSV | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783435708588/test_planner_integrity_validation.csv` | Header only; `0` data rows |
| Analyzer summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783435708588/metadata/safety_planner_analysis_summary.json` | Analyzer status `FAIL` |
| Topic counts CSV | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783435708588/csv/b0_1_topic_counts.csv` | Continuity evidence; filename still uses `b0_1` due analyzer limitation |
| Rosbag | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_corridor_off_lidar_corridor_degenerate_20260707T144828Z` | LiDAR body and planner bspline present; integrity and IAP odom absent |

### Failure Analysis

B0-3 is a hard `FAIL`, not `INCONCLUSIVE`, because the export, bag, validator summary, and analyzer summary were all present and readable. The launch emitted the expected off-profile manifest, and `/sim/drone_0/lidar_body` remained continuous, so the failure is not a Safety Planner activation issue or a missing LiDAR input issue.

The decisive failure is that `iap_rosnode` died early with exit code `-6`, after no-GPU/GPU-allocation messages, leaving `/iap/integrity` and `/drone_0_visual_slam/odom` with `0` recorded messages. The validator therefore received no integrity reports and failed all lidar/fallback validity requirements. Since validator and analyzer failed, teardown SIGINT exceptions are not used as a non-blocking pass rationale for this run.

### Remaining Issues

| Issue | Severity | Owner | Blocking experiment | Next action |
|---|---|---|---|---|
| `iap_rosnode` crashes in `lidar_corridor_degenerate` without GPU-backed points/covs | High | IAP odometry/runtime | Blocks B0-3 and B0-4 | Debug CPU/no-GPU handling or corridor LiDAR preprocessing path before rerunning B0-3 |
| `/iap/integrity` absent for entire B0-3 run | High | Integrity/runtime | Blocks B0-3 | Restore IAP odometry/integrity publication in corridor scenario |
| `/drone_0_visual_slam/odom` absent for entire B0-3 run | High | IAP odometry/runtime | Blocks B0-3 | Investigate IAP node crash root cause and odometry publication path |
| Analyzer artifact filenames still contain `b0_1` | Low | dev_planner | None for interpreting B0-3 failure | Rename artifacts when generalizing the analyzer beyond B0-1 naming |
| SIGINT teardown exceptions in helper nodes | Low | Planner/sim runtime | Not the B0-3 root cause | Track separately after validator/analyzer failures are resolved |

### Next Actions

1. Do not proceed to `B0-4 fallback baseline`, P0, P5, or `all`.
2. Debug the corridor baseline first, focusing on the `iap_rosnode` crash, no-GPU point/cov allocation path, `/drone_0_visual_slam/odom`, and `/iap/integrity` publication.
3. Rerun only B0-3 after the corridor/IAP failure is fixed.

## B0-3 Corridor Baseline Validation Rerun

This rerun supersedes the previous B0-3 failure attempt as the accepted B0-3 evidence. The earlier attempt is retained above because it documents the Docker/GPU outage failure mode; this run was executed after GPU access was restored.

### Repo State And Environment

| Field | Value |
|---|---|
| Date | `2026-07-07` UTC |
| Operator / Agent | Codex |
| Machine / container | `mint-X` |
| Workspace | `/home/dev/ws_iap` |
| Package repo | `/home/dev/ws_iap/src/iap` |
| Branch | `dev/iap` |
| Repo protection action | B0-2 amend/push was already complete; no additional amend was needed |
| Commit under test before rerun | `52f4c8f0718c7f11273ac7b32ff33636a7f4f199` |
| Clean/dirty status before rerun | `clean`; `git status -sb` reported `## dev/iap...origin/dev/iap` |
| Recent commits before rerun | `52f4c8f docs: record safety planner B0-3 corridor baseline validation`; `72d31ed docs: record safety planner B0-2 baseline validation`; `1592da2 docs: record safety planner B0-1 baseline validation` |
| GPU check | `nvidia-smi` detected `NVIDIA GeForce RTX 4070` before launch |
| Source command | `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| Build command | Not rerun for this evidence-only rerun; reused current installed workspace |

### Launch And Artifacts

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch iap test_planner.launch.py \
  experiment:=baseline_corridor_off \
  scenario:=lidar_corridor_degenerate \
  run_duration_s:=90 \
  validation_duration_s:=90 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

| Field | Value |
|---|---|
| Experiment ID | `B0-3` |
| Experiment preset | `baseline_corridor_off` |
| Scenario | `lidar_corridor_degenerate` |
| Launch result | `PASS`: command exited `0` |
| Export dir | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275` |
| Bag dir | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_corridor_off_lidar_corridor_degenerate_20260707T151813Z` |
| Rosbag summary | Storage `mcap`; size `581.7 MiB`; duration `90.002003329s`; messages `509091` |

### Parameter And Manifest Evidence

| Parameter | Manifest value | Result |
|---|---:|---|
| `planner_safety_profile` | `off` | `PASS` |
| `p0.enable_risk_grid` | `false` | `PASS` |
| `planner_enable_p1` | `false` | `PASS` |
| `planner_enable_p2` | `false` | `PASS` |
| `planner_enable_p3_local` | `false` | `PASS` |
| `planner_enable_p3_global` | `false` | `PASS` |
| `planner_enable_p4` | `false` | `PASS` |
| `planner_enable_p5_runtime` | `false` | `PASS` |
| `planner_enable_p5_final` | `false` | `PASS` |

### Validator And Analyzer

| Check | Result | Evidence |
|---|---|---|
| Validator summary | `PASS` | `passed=true`, `failures=[]`, `message_count=883` |
| Required fusion mode | `PASS` | `required_fusion_mode=lidar_only` |
| Required final source | `PASS` | `required_final_source=LIDAR` |
| LiDAR/fallback validity | `PASS` | `lidar_valid_seen=true`, `fallback_valid_seen=true`, `gnss_valid_seen=false` |
| Analyzer status | `PASS` | `status=PASS`, `passed=true`, command exited `0` with `--fail-on-threshold` |
| Analyzer summary | `PASS` | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275/metadata/safety_planner_analysis_summary.json` |
| Failures / inconclusive / warnings | `PASS` | `[]` / `[]` / `[]` |
| Integrity CSV rows | `PASS` | `883` data rows; CSV file has `884` lines including header |
| Integrity HPL | `PASS` | min `2.045466m`; mean `2.639436m`; max `2.842882m` |
| Integrity VPL | `PASS` | min `2.013485m`; mean `2.619819m`; max `2.823732m` |
| P5 summary | `PASS` | `status_rows=0`, `bad_action_count=0`, `action_counts={}` |
| Analyzer limitation | `WARN` | Generated artifact filenames and `next_debug_branch` still contain older B0 naming; artifact contents and thresholds are valid |

Analyzer command:

```bash
python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py \
  --experiment-id B0-3 \
  --export-dir /home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275 \
  --bag-dir /home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_corridor_off_lidar_corridor_degenerate_20260707T151813Z \
  --fail-on-threshold
```

### Topic Health

| Topic | Expected | Count | Hz | Span | Coverage | Max gap | Status |
|---|---|---:|---:|---:|---:|---:|---|
| `/iap/integrity` | continuous | `885` | `9.833114` | `88.389331s` | `0.982082` | `0.111065s` | `PASS` |
| `/sim/drone_0/lidar_body` | continuous | `897` | `9.966445` | `89.599423s` | `0.995527` | `0.100615s` | `PASS` |
| `/drone_0_planning/bspline` | planner-dependent | `18` | `0.199996` | `17.252677s` | `0.191692` | `1.020679s` | `PASS` |
| `/planning/integrity_gate_status` | absent-or-zero when P5 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/drone_0_visual_slam/odom` | IAP odometry available | `885` | bag metadata | bag metadata | bag metadata | bag metadata | `PASS` |
| `/iap/rviz/risk_grid_health` | P0 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/predicted_pl_cloud` | P0 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/risk_validity_cloud` | P0 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/p5_gate_status` | P5 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |

### Result Row

| Experiment ID | Status | Validator | Analyzer | Key evidence | Failure / warning | Next branch |
|---|---|---|---|---|---|---|
| B0-3 rerun | `PASS` | `passed=true`, failures `[]`, message_count `883` | `PASS`, failures `[]`, warnings `[]`, inconclusive `[]` | Manifest safety switches all false; `/iap/integrity` and `/sim/drone_0/lidar_body` continuous; `/drone_0_planning/bspline` present; P5 status rows `0` | Non-blocking SIGINT teardown exceptions after validator/analyzer pass; analyzer filename/next-branch naming limitation | `B0-4 fallback baseline` |

### B0-3 Acceptance Matrix

| Criterion | Result | Evidence |
|---|---|---|
| Launch exits `0` | `PASS` | `ros2 launch` command returned exit code `0` |
| Validator summary `passed=true` | `PASS` | `test_planner_validation_summary.json` reports `passed: true` |
| Analyzer status `PASS` or only non-blocking warning | `PASS` | Analyzer status `PASS`; command exited `0` |
| `planner_safety_profile=off` | `PASS` | Manifest |
| `p0.enable_risk_grid=false` | `PASS` | Manifest |
| P1-P5 planner switches all false | `PASS` | Manifest records all `planner_enable_*` fields as `false` |
| `/iap/integrity` continuous | `PASS` | Analyzer count `885`, coverage `0.982082`, max gap `0.111065s` |
| `/sim/drone_0/lidar_body` continuous and nonempty | `PASS` | Analyzer count `897`, coverage `0.995527`, max gap `0.100615s` |
| `/drone_0_planning/bspline` exists | `PASS` | Bag/analyzer count `18` |
| `/planning/integrity_gate_status` has `0` rows or no P5 action | `PASS` | Analyzer `status_rows=0`, `bad_action_count=0` |
| No P0 risk-grid behavior appears | `PASS` | P0 disabled in manifest; P0/RViz risk topics recorded `0` messages |
| P2 behavior disabled | `PASS` | `planner_enable_p2=false`; no P2 evidence required for this baseline |
| P1 behavior disabled | `PASS` | `planner_enable_p1=false`; no P1 evidence required for this baseline |

### Key Artifacts

| Artifact | Path | Conclusion |
|---|---|---|
| Manifest JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275/test_planner_manifest.json` | Safety Planner fully off |
| Validator summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275/test_planner_validation_summary.json` | Validator passed with `883` messages |
| Integrity validation CSV | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275/test_planner_integrity_validation.csv` | `883` data rows |
| Analyzer summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275/metadata/safety_planner_analysis_summary.json` | Analyzer status `PASS` |
| Integrity HPL/VPL timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275/figures/b0_1_integrity_hpl_vpl_timeline.png` | 883 integrity samples plotted; filename still uses `b0_1` due analyzer limitation |
| Topic counts CSV | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_corridor_off_lidar_corridor_degenerate_1783437493275/csv/b0_1_topic_counts.csv` | Continuity evidence for B0-3 acceptance; filename still uses `b0_1` due analyzer limitation |
| Rosbag | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_corridor_off_lidar_corridor_degenerate_20260707T151813Z` | Core topics recorded across the run |

### Failure Analysis

No failure observed in the accepted B0-3 rerun. The previous B0-3 failure was caused by the Docker/GPU outage path: `iap_rosnode` exited before publishing integrity or IAP odometry. With GPU access restored, `iap_rosnode` stayed alive through the validation window, produced `/drone_0_visual_slam/odom`, and published continuous `/iap/integrity`.

Non-blocking teardown note: after the validator passed and the launch command began SIGINT shutdown, several helper nodes reported `RCLError`, `std::system_error`, or signal-based exit codes. Per the test plan, this is not a B0-3 hard fail because the launch command exited `0`, validator passed, and analyzer passed.

### Remaining Issues

| Issue | Severity | Owner | Blocking experiment | Next action |
|---|---|---|---|---|
| SIGINT teardown exceptions in helper nodes | Low | Planner/sim runtime | None for B0-3 | Track separately if teardown stability becomes a CI requirement |
| Analyzer artifact filenames still contain `b0_1` | Low | dev_planner | None for B0-3 | Rename artifacts when generalizing the analyzer beyond B0-1 naming |
| Analyzer `next_debug_branch` still returns `continue_to_B0-2_open_sky_baseline` on B0-3 pass | Low | dev_planner | None for B0-3 | Treat manual next branch as `B0-4 fallback baseline` |

### Next Actions

1. B0-3 satisfies Phase 0 corridor baseline criteria.
2. Manual next branch is `B0-4 fallback baseline`.
3. Do not run P0, P5, or `all` until B0-4 is explicitly requested and completed.
