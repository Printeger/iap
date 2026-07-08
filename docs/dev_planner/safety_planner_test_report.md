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

## B0-4 Fallback Baseline Validation

### Repo State And Environment

| Field | Value |
|---|---|
| Date | `2026-07-07` UTC |
| Operator / Agent | Codex |
| Machine / container | `mint-X` |
| Workspace | `/home/dev/ws_iap` |
| Package repo | `/home/dev/ws_iap/src/iap` |
| Branch | `dev/iap` |
| Commit under test | `9bbbd84e685c5544ce2588ad573bb1df3dc8ebe4` |
| Clean/dirty status before B0-4 | `clean`; `git status -sb` reported `## dev/iap...origin/dev/iap` |
| Recent commits before B0-4 | `9bbbd84 docs: require default safety planner validation figures`; `80888ef docs: record safety planner B0-3 corridor baseline validation`; `52f4c8f docs: record safety planner B0-3 corridor baseline validation` |
| GPU check | `nvidia-smi` detected `NVIDIA GeForce RTX 4070`, driver `580.126.09`, CUDA `13.0` |
| Source command | `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| Analyzer precheck | `python3 -m py_compile src/iap/scripts/dev_planner/analyze_safety_planner_run.py` passed; B0-3 compatibility smoke returned `PASS` with `b0_3_*` artifacts |

### Launch And Artifacts

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch iap test_planner.launch.py \
  experiment:=baseline_fused_nominal_off \
  scenario:=fallback_only \
  run_duration_s:=90 \
  validation_duration_s:=90 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true \
  validator_require_gnss_valid:=false \
  validator_require_lidar_valid:=false \
  validator_require_fallback_valid:=true \
  validator_required_final_source:=FALLBACK
```

| Field | Value |
|---|---|
| Experiment ID | `B0-4` |
| Experiment preset | `baseline_fused_nominal_off` |
| Scenario | `fallback_only` |
| Launch result | `PASS`: command exited `0` |
| Export dir | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703` |
| Bag dir | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_fallback_only_20260707T161428Z` |
| Rosbag summary | Storage `mcap`; size `328.7 MiB`; duration `90.022062487s`; messages `509729` |

### Parameter And Manifest Evidence

| Parameter | Value | Result |
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
| `integrity_fusion_mode` | `fallback_only` | `PASS`; validator required this fusion mode |
| `validator_require_gnss_valid` | `false` | `PASS` |
| `validator_require_lidar_valid` | `false` | `PASS` |
| `validator_require_fallback_valid` | `true` | `PASS` |
| `validator_required_final_source` | `FALLBACK` | `PASS` |

### Validator And Analyzer

| Check | Result | Evidence |
|---|---|---|
| Validator summary | `PASS` | `passed=true`, `failures=[]`, `message_count=884` |
| Required fusion mode | `PASS` | `required_fusion_mode=fallback_only` |
| Required final source | `PASS` | `required_final_source=FALLBACK` |
| Source validity | `PASS` | `gnss_valid_seen=false`, `lidar_valid_seen=false`, `fallback_valid_seen=true` |
| Analyzer status | `PASS` | `status=PASS`, `passed=true`, command exited `0` with `--fail-on-threshold` |
| Analyzer summary | `PASS` | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/metadata/safety_planner_analysis_summary.json` |
| Failures / inconclusive / warnings | `PASS` | `[]` / `[]` / `[]` |
| Integrity CSV rows | `PASS` | `884` data rows |
| Fusion mode counts | `PASS` | `fallback_only: 884` |
| Final HPL/VPL source counts | `PASS` | `FALLBACK: 884` for both HPL and VPL |
| Fallback validity | `PASS` | `fallback_valid_count=884`, `fallback_valid_seen=true` |
| Integrity HPL/VPL range | `PASS` | HPL min/mean/max `0.299468m` / `1646.596795m` / `4785.288600m`; VPL min/mean/max same |
| P5 summary | `PASS` | `status_rows=0`, `bad_action_count=0`, `action_counts={}` |
| Analyzer next branch | `PASS` | `continue_to_P0-1_open_sky_data_only_validation` |

Analyzer command:

```bash
python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py \
  --experiment-id B0-4 \
  --export-dir src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703 \
  --bag-dir src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_fallback_only_20260707T161428Z \
  --fail-on-threshold
```

### Topic Health

| Topic | Expected | Count | Hz | Span | Coverage | Max gap | Status |
|---|---|---:|---:|---:|---:|---:|---|
| `/iap/integrity` | continuous | `886` | `9.842032` | `88.494722s` | `0.983034` | `0.118564s` | `PASS` |
| `/sim/drone_0/lidar_body` | continuous | `897` | `9.964224` | `89.599916s` | `0.995311` | `0.100427s` | `PASS` |
| `/drone_0_visual_slam/odom` | IAP odometry available | `886` | bag metadata | bag metadata | bag metadata | bag metadata | `PASS` |
| `/drone_0_planning/bspline` | planner-dependent | `18` | `0.199951` | `17.307547s` | `0.192259` | `1.019319s` | `PASS` |
| `/planning/integrity_gate_status` | absent-or-zero when P5 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |
| `/planning/risk_grid_health` | P0 disabled | `0` | n/a | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/risk_grid_health` | P0 disabled | `0` | n/a | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/predicted_pl_cloud` | P0 disabled | `0` | n/a | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/risk_validity_cloud` | P0 disabled | `0` | n/a | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/trajectory_integrity_samples` | P0/P5 disabled | `0` | n/a | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/current_traj_integrity_colored` | P5 disabled | `0` | n/a | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/p5_gate_status` | P5 disabled | `0` | n/a | n/a | n/a | n/a | `PASS` |
| `/iap/rviz/p5_current_im_bars` | P5 disabled | `0` | n/a | n/a | n/a | n/a | `PASS` |

### Result Row

| Experiment ID | Status | Validator | Analyzer | Key evidence | Failure / warning | Next branch |
|---|---|---|---|---|---|---|
| B0-4 | `PASS` | `passed=true`, failures `[]`, message_count `884` | `PASS`, failures `[]`, warnings `[]`, inconclusive `[]` | Manifest safety switches all false; `/iap/integrity` continuous; fallback valid and final source `FALLBACK`; P5 status rows `0`; P0/P5 topics `0` | Non-blocking SIGINT teardown exceptions after validator/analyzer pass; high fallback PL is expected evidence of fallback-only integrity, not a safety-action failure while Safety Planner is off | `P0-1 open-sky data-only validation` |

### B0-4 Acceptance Matrix

| Criterion | Result | Evidence |
|---|---|---|
| Launch exits `0` | `PASS` | `ros2 launch` command returned exit code `0` |
| Validator summary `passed=true` | `PASS` | `test_planner_validation_summary.json` reports `passed: true` |
| Analyzer status `PASS` | `PASS` | Analyzer command exited `0` with `--fail-on-threshold` |
| `planner_safety_profile=off` | `PASS` | Manifest |
| `p0.enable_risk_grid=false` | `PASS` | Manifest |
| P1-P5 planner switches all false | `PASS` | Manifest records all `planner_enable_*` fields as `false` |
| `/iap/integrity` continuous | `PASS` | Analyzer count `886`, coverage `0.983034`, max gap `0.118564s` |
| `fallback_valid_seen=true` | `PASS` | Validator and analyzer integrity summary |
| Final HPL/VPL source is `FALLBACK` | `PASS` | `final_hpl_source_counts={"FALLBACK":884}`, `final_vpl_source_counts={"FALLBACK":884}` |
| `/planning/integrity_gate_status` has zero rows or no P5 action | `PASS` | Bag count `0`; analyzer `status_rows=0`, `bad_action_count=0` |
| P0/P5 risk/RViz topics absent or count `0` | `PASS` | Analyzer `safety_off_topic_counts` reports `0` for all checked P0/P5 topics |
| Required figures are present and non-empty | `PASS` | `test -s` passed for scenario top-down, topic activity, integrity source, and HPL/VPL timelines |

### Key Artifacts

| Artifact | Path | Conclusion |
|---|---|---|
| Manifest JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/test_planner_manifest.json` | Safety Planner fully off |
| Validator summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/test_planner_validation_summary.json` | Validator passed with `884` messages |
| Integrity validation CSV | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/test_planner_integrity_validation.csv` | `884` data rows; final source always `FALLBACK` |
| Analyzer summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/metadata/safety_planner_analysis_summary.json` | Analyzer status `PASS` |
| Topic counts CSV | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/csv/b0_4_topic_counts.csv` | Continuity evidence for B0-4 acceptance |
| Scenario top-down | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/figures/b0_4_scenario_topdown.png` | Bag-derived map, truth odom, visual-slam odom, and bspline overlay |
| Topic activity timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/figures/b0_4_topic_activity_timeline.png` | Core topic activity/gap visualization |
| Integrity source timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/figures/b0_4_integrity_source_timeline.png` | `fallback_only`, final source `FALLBACK`, fallback valid throughout |
| Integrity HPL/VPL timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/figures/b0_4_integrity_hpl_vpl_timeline.png` | Fallback PL timeline plotted; high values expected under fallback-only source |
| Rosbag | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_fallback_only_20260707T161428Z` | Core topics recorded across the run |

![B0-4 scenario top-down](../../results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/figures/b0_4_scenario_topdown.png)

![B0-4 topic activity timeline](../../results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/figures/b0_4_topic_activity_timeline.png)

![B0-4 integrity source timeline](../../results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/figures/b0_4_integrity_source_timeline.png)

![B0-4 integrity HPL/VPL timeline](../../results/planner_validation/exports/test_planner_baseline_fused_nominal_off_fallback_only_1783440868703/figures/b0_4_integrity_hpl_vpl_timeline.png)

### Failure Analysis

No B0-4 failure observed. The run emitted repeated `INTEGRITY UNSAFE` warnings because fallback-only HPL/VPL rose above the alert limits, with maximum HPL/VPL `4785.288600m`. This is expected fallback-only baseline evidence and is not a Safety Planner action because all P0-P5 switches were off. `/planning/integrity_gate_status` remained at `0` messages and P0/P5 risk/RViz topics remained at `0`.

Shutdown note: after the validator passed and the launch command began SIGINT shutdown, several helper nodes reported `RCLError`, signal-based exit codes, or delayed SIGTERM. This matches prior B0 runs and is not a B0-4 hard fail because the launch command exited `0`, validator passed, analyzer passed, and the evidence chain is complete.

### Remaining Issues

| Issue | Severity | Owner | Blocking experiment | Next action |
|---|---|---|---|---|
| SIGINT teardown exceptions in helper nodes | Low | Planner/sim runtime | None for B0-4 | Track separately if teardown stability becomes a CI requirement |
| Fallback-only PL can greatly exceed alert limits | Informational | Integrity/runtime | None for B0-4 | Use as baseline evidence for later P0/P5 fallback semantics rather than as a B0 failure |

### Verification Commands

| Command | Result |
|---|---|
| `git -C src/iap status -sb` | `PASS`; clean before B0-4 |
| `git -C src/iap log --oneline -5` | `PASS`; latest pre-run commit `9bbbd84` |
| `git -C src/iap rev-parse HEAD` | `PASS`; `9bbbd84e685c5544ce2588ad573bb1df3dc8ebe4` |
| `nvidia-smi` | `PASS`; RTX 4070 visible |
| `python3 -m py_compile src/iap/scripts/dev_planner/analyze_safety_planner_run.py` | `PASS` |
| B0-4 launch command above | `PASS`; exit code `0` |
| Artifact inspection commands | `PASS`; manifest, validator summary, and rosbag metadata readable |
| Analyzer command above | `PASS`; status `PASS` |
| `test -s .../figures/b0_4_scenario_topdown.png` | `PASS` |
| `test -s .../figures/b0_4_topic_activity_timeline.png` | `PASS` |
| `test -s .../figures/b0_4_integrity_source_timeline.png` | `PASS` |
| `test -s .../figures/b0_4_integrity_hpl_vpl_timeline.png` | `PASS` |
| `git -C src/iap diff --check` | `PASS` |

### Next Actions

1. B0-4 satisfies Phase 0 fallback baseline criteria.
2. Phase 0 `B0-1` through `B0-4` baseline lock is complete.
3. Next planned experiment is `P0-1 open-sky data-only validation`, but it was not run in this B0-4 execution.

## P0-1 Open-Sky Data-Only Validation

### Repo State And Environment

| Field | Value |
|---|---|
| Date | `2026-07-08` UTC |
| Operator / Agent | Codex |
| Machine / container | `mint-X` |
| Workspace | `/home/dev/ws_iap` |
| Package repo | `/home/dev/ws_iap/src/iap` |
| Branch | `dev/iap` |
| Launch precheck commit | `64bfe5d docs: support P0-1 safety planner validation figures` |
| Final analyzer commit before report | `ac238e6e61040ced02b1a4ec59c60fa3810cff92` |
| Clean/dirty status before P0-1 | `clean`; `git status -sb` reported `## dev/iap...origin/dev/iap` |
| Recent commits before launch | `64bfe5d`, `17a3195`, `9bbbd84`, `80888ef`, `52f4c8f` |
| GPU check | `nvidia-smi` detected `NVIDIA GeForce RTX 4070`, driver `580.126.09`, CUDA `13.0` |
| Source command | `source /opt/ros/jazzy/setup.bash && source install/setup.bash` |
| Analyzer precheck | `python3 -m py_compile src/iap/scripts/dev_planner/analyze_safety_planner_run.py` passed |

Analyzer note: the launch artifact was produced from clean `64bfe5d`. The final analyzer pass was rerun from `ac238e6`, which only refines P0-1 post-processing thresholds for planner-start-delay and partial occupied-skip semantics.

### Launch And Artifacts

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  scenario:=gnss_open_sky \
  run_duration_s:=60 \
  validation_duration_s:=60 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

| Field | Value |
|---|---|
| Experiment ID | `P0-1` |
| Experiment preset | `p0_open_sky` |
| Scenario | `gnss_open_sky` |
| Launch result | `PASS`: command exited `0` |
| Export dir | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509` |
| Bag dir | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_open_sky_20260708T033119Z` |
| Rosbag summary | Storage `mcap`; size `324.7 MiB`; duration `60.050683803s`; messages `343715` |
| Analyzer summary | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/metadata/safety_planner_analysis_summary.json` |

Analyzer command:

```bash
python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py \
  --experiment-id P0-1 \
  --export-dir src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509 \
  --bag-dir src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_open_sky_20260708T033119Z \
  --baseline-export-dir src/iap/results/planner_validation/exports/test_planner_baseline_fused_nominal_off_gnss_open_sky_1783432749389 \
  --baseline-bag-dir src/iap/results/planner_validation/bags/test_planner_baseline_fused_nominal_off_gnss_open_sky_20260707T135909Z \
  --fail-on-threshold
```

### Parameter And Manifest Evidence

| Parameter | Value | Result |
|---|---:|---|
| `planner_safety_profile` | `off` | `PASS`; baseline safety profile unchanged |
| `p0.enable_risk_grid` | `true` | `PASS` |
| `p0.debug_metrics_enable` | `true` | `PASS`; preset enables P0 debug metrics |
| `planner_enable_p1` | `false` | `PASS` |
| `planner_enable_p2` | `false` | `PASS` |
| `planner_enable_p3_local` | `false` | `PASS` |
| `planner_enable_p3_global` | `false` | `PASS` |
| `planner_enable_p4` | `false` | `PASS` |
| `planner_enable_p5_runtime` | `false` | `PASS` |
| `planner_enable_p5_final` | `false` | `PASS` |
| `validator_require_gnss_valid` | `true` | `PASS` |
| `validator_require_lidar_valid` | `false` | `PASS` |
| `validator_require_fallback_valid` | `true` | `PASS` |
| `validator_required_final_source` | `GNSS` | `PASS` |

### Validator And Analyzer

| Check | Result | Evidence |
|---|---|---|
| Validator summary | `PASS` | `passed=true`, `failures=[]`, `message_count=584` |
| Required fusion mode | `PASS` | `required_fusion_mode=gnss_only` |
| Required final source | `PASS` | `required_final_source=GNSS` |
| Source validity | `PASS` | `gnss_valid_seen=true`, `lidar_valid_seen=false`, `fallback_valid_seen=true` |
| Analyzer status | `PASS` | `status=PASS`, `passed=true`, command exited `0` with `--fail-on-threshold` |
| Failures / inconclusive / warnings | `PASS` | `[]` / `[]` / `[]` |
| Integrity CSV rows | `PASS` | `584` data rows |
| Fusion mode counts | `PASS` | `gnss_only: 584` |
| Final HPL/VPL source counts | `PASS` | `GNSS: 584` for both HPL and VPL |
| Integrity HPL range | `PASS` | min/mean/max `4.875940m` / `5.077726m` / `5.530642m` |
| Integrity VPL range | `PASS` | min/mean/max `13.752337m` / `14.510498m` / `17.265101m` |
| P5 summary | `PASS` | `status_rows=0`, `bad_action_count=0`, `action_counts={}` |
| Analyzer next branch | `PASS` | `continue_to_P0-2_degraded_gnss_lidar_good_validation` |

### Topic Health

| Topic | Expected | Count | Hz | Span | Coverage | Max gap | Status |
|---|---|---:|---:|---:|---:|---:|---|
| `/iap/integrity` | continuous | `586` | `9.758423` | `58.525784s` | `0.974606` | `0.129738s` | `PASS` |
| `/sim/drone_0/lidar_body` | continuous | `597` | `9.941602` | `59.600263s` | `0.992499` | `0.100458s` | `PASS` |
| `/drone_0_visual_slam/odom` | continuous | `586` | `9.758423` | `58.525669s` | `0.974605` | `0.129718s` | `PASS` |
| `/drone_0_planning/bspline` | planner-dependent | `16` | `0.266442` | `16.136956s` | `0.268722` | `1.088360s` | `PASS` |
| `/planning/risk_grid_health` | active-periodic after planner start | `44` | `0.732714` | `46.172395s` | `0.768890` | `1.083265s` | `PASS` |
| `/iap/rviz/predicted_pl_cloud` | present | `44` | `0.732714` | `46.171567s` | `0.768877` | `1.083238s` | `PASS` |
| `/iap/rviz/risk_validity_cloud` | present | `44` | `0.732714` | `46.171628s` | `0.768878` | `1.083061s` | `PASS` |
| `/planning/integrity_gate_status` | absent-or-zero when P5 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |

### P0 Health Summary

| Metric | Value | Result |
|---|---:|---|
| Health rows | `44` | `PASS` |
| `ready_false_count` / max consecutive | `0` / `0` | `PASS` |
| `stale_true_count` / max consecutive | `0` / `0` | `PASS` |
| `valid_ratio` min/mean/max | `0.996484` / `0.997615` / `1.000000` | `PASS` |
| `unknown_ratio` min/mean/max | `0.000000` / `0.002385` / `0.003516` | `PASS` |
| Full-frame unknown count / max consecutive | `0` / `0` | `PASS` |
| Refresh elapsed mean | `573.557ms` | `PASS` |
| Provider query count max | `64000` | `PASS` |
| Provider stale count max | `0` | `PASS` |
| Provider invalid count max | `0` | `PASS` |
| Occupied skip count max | `225` of `64000` cells (`0.3516%`) | `PASS`; low occupied overlap explains the small unknown ratio |

### Reason Histogram

| Reason / counter | Count or max | Conclusion |
|---|---:|---|
| `ok` health rows | `44` | Dominant and only health reason; expected for open-sky P0-1 |
| Empty reason rows | `0` | `PASS` |
| `provider_stale_count_max` | `0` | No stale provider evidence |
| `provider_invalid_count_max` | `0` | No invalid provider evidence |
| `occupied_skip_count_max` | `225` | Low partial occupied skip; not a full-frame or material unknown failure |

### PL / Cost Distribution

| Metric | Value | Conclusion |
|---|---:|---|
| PL min/mean/max | `11.181885m` / `11.181885m` / `11.181885m` | Compact open-sky distribution; no high/unknown tail |
| Cost min/mean/max | `11.181885` / `11.181885` / `11.181885` | Bounded low-cost field relative to fallback/unknown behavior |
| Valid cells | `3180` of `3200` (`99.375%`) | `PASS` |
| Unknown cells | `20` of `3200` (`0.625%`) | `PASS`; small localized unknown area |
| Stale cells | `0` | `PASS` |

### Baseline Vs P0-1 Comparison

| Metric | B0-2 open-sky baseline | P0-1 | Conclusion |
|---|---:|---:|---|
| Truth sample count | `1998` | `1937` | Both bags have dense truth odom |
| Truth path length | `24.943657m` | `24.389676m` | Similar path length; P0-1 is `2.2%` shorter |
| Resampled truth RMS distance | n/a | `9.551728m` | Numeric metric is retained but not used as a hard gate because the baseline artifact is a longer `90s` run while P0-1 is `60s` |
| Top-down trajectory overlay | Straight open-sky traversal | Straight open-sky traversal | No visible trajectory abnormality or P5 action; P0 stayed data-only/debug |

### Result Row

| Experiment ID | Status | Validator | Analyzer | Key evidence | Failure / warning | Next branch |
|---|---|---|---|---|---|---|
| P0-1 | `PASS` | `passed=true`, failures `[]`, message_count `584` | `PASS`, failures `[]`, warnings `[]`, inconclusive `[]` | Manifest has `p0.enable_risk_grid=true`; P1-P5 disabled; risk health ready and non-stale; PL cloud and validity cloud present; P5 status rows `0` | Non-blocking SIGINT teardown exceptions after validator/analyzer pass; baseline RMS comparison is informational due duration mismatch | `P0-2 degraded GNSS + LiDAR good validation` |

### P0-1 Acceptance Matrix

| Criterion | Result | Evidence |
|---|---|---|
| Launch exits `0` | `PASS` | `ros2 launch` command returned exit code `0` |
| Validator summary `passed=true` | `PASS` | `test_planner_validation_summary.json` reports `passed: true` |
| Analyzer status `PASS` | `PASS` | Analyzer command exited `0` with `--fail-on-threshold` |
| Manifest has `p0.enable_risk_grid=true` | `PASS` | Manifest |
| P1-P5 planner switches all disabled | `PASS` | Manifest records all `planner_enable_*` fields as `false` |
| Baseline safety profile unchanged | `PASS` | `planner_safety_profile=off` |
| `/planning/risk_grid_health` continuous/data present | `PASS` | `44` active-periodic messages, max gap `1.083265s` after planner activation |
| P0 health `ready=true` | `PASS` | `ready_false_count=0` |
| P0 health `stale=false` | `PASS` | `stale_true_count=0` |
| Dominant reason `ok` or explainable | `PASS` | `ok: 44`; no stale/invalid counters; low occupied skip explains small unknown ratio |
| No periodic full-frame unknown | `PASS` | `full_unknown_count=0`, max consecutive `0` |
| Unknown ratio low and explained | `PASS` | mean `0.002385`, max `0.003516`; occupied skip max `225/64000` |
| `/iap/rviz/predicted_pl_cloud` has messages | `PASS` | Bag/analyzer count `44` |
| `/iap/rviz/risk_validity_cloud` has messages | `PASS` | Bag/analyzer count `44` |
| `/planning/integrity_gate_status` has zero rows or no P5 action | `PASS` | Bag count `0`; analyzer `status_rows=0`, `bad_action_count=0` |
| Required figures are present and non-empty | `PASS` | `test -s` passed for all six P0-1 required figures |
| P0-2/P5/P1/P2 not run | `PASS` | Only `experiment:=p0_open_sky scenario:=gnss_open_sky` was launched |

### Key Artifacts

| Artifact | Path | Conclusion |
|---|---|---|
| Manifest JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/test_planner_manifest.json` | P0 enabled, baseline safety profile `off`, P1-P5 disabled |
| Validator summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/test_planner_validation_summary.json` | Validator passed with `584` messages |
| Analyzer summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/metadata/safety_planner_analysis_summary.json` | Analyzer status `PASS` |
| Scenario top-down | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_scenario_topdown.png` | Map, truth odom, visual-slam odom, bspline, and baseline truth overlay show a normal straight traversal with no visible baseline deviation |
| Topic activity timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_topic_activity_timeline.png` | Integrity, LiDAR, odom, bspline, and risk-grid health have the expected activity; P0 health starts after planner activation |
| P0 health timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_p0_health_timeline.png` | Ready stays true, stale stays false, and there is no full-frame unknown or stale cycle |
| P0 reason histogram | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_p0_reason_histogram.png` | Dominant reason is `ok`; stale/invalid provider counters are zero, with only low occupied-skip counts |
| PL/cost distribution | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_pl_cost_distribution.png` | Distribution is compact at PL/cost `11.181885` with `99.375%` valid cells and no stale tail |
| Risk grid snapshot overview | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_risk_grid_snapshot_overview.png` | Latest predicted PL and validity clouds render; field is mostly valid with a small localized invalid/unknown patch |
| Rosbag | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_open_sky_20260708T033119Z` | Required P0-1 topics recorded |

![P0-1 scenario top-down](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_scenario_topdown.png)

![P0-1 topic activity timeline](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_topic_activity_timeline.png)

![P0-1 P0 health timeline](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_p0_health_timeline.png)

![P0-1 P0 reason histogram](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_p0_reason_histogram.png)

![P0-1 PL/cost distribution](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_pl_cost_distribution.png)

![P0-1 risk grid snapshot overview](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/figures/p0_1_risk_grid_snapshot_overview.png)

### Failure Analysis

No P0-1 hard failure observed. P0 RiskGridMap/RiskGridSnapshot produced health and RViz cloud evidence, stayed `ready=true`, stayed `stale=false`, avoided full-frame unknown cycles, and did not produce a P5 action. `/planning/integrity_gate_status` remained at `0` messages with P5 disabled.

The launch log contains shutdown-time `RCLError`, `std::system_error`, and signal-based exits for helper visualization/bridge nodes after the validator had passed and rosbag recording stopped. This matches prior baseline runs and is not a P0-1 hard failure because the launch command exited `0`, validator passed, analyzer passed, and the evidence chain is complete.

### Remaining Issues

| Issue | Severity | Owner | Blocking experiment | Next action |
|---|---|---|---|---|
| SIGINT teardown exceptions in helper nodes | Low | Planner/sim runtime | None for P0-1 | Track separately if teardown stability becomes a CI requirement |
| Baseline trajectory RMS metric is duration-sensitive | Low | dev_planner analyzer | None for P0-1 | Prefer top-down overlay and path-length comparison until analyzer normalizes different run durations |
| Validity cloud scalar CSV summary has `row_count=0` although topic messages and overview plot exist | Low | dev_planner analyzer | None for P0-1 | Extend validity-cloud scalar extraction if future reports need numeric validity-cloud statistics |

### Verification Commands

| Command | Result |
|---|---|
| `git -C src/iap status -sb` | `PASS`; clean before P0-1 launch and before report editing |
| `git -C src/iap log --oneline -5` | `PASS`; latest pre-launch commit `64bfe5d` |
| `nvidia-smi` | `PASS`; RTX 4070 visible |
| `python3 -m py_compile src/iap/scripts/dev_planner/analyze_safety_planner_run.py` | `PASS` |
| P0-1 launch command above | `PASS`; exit code `0` |
| Artifact inspection commands | `PASS`; manifest, validator summary, and rosbag metadata readable |
| Analyzer command above | `PASS`; status `PASS` |
| `test -s .../figures/p0_1_scenario_topdown.png` | `PASS` |
| `test -s .../figures/p0_1_topic_activity_timeline.png` | `PASS` |
| `test -s .../figures/p0_1_p0_health_timeline.png` | `PASS` |
| `test -s .../figures/p0_1_p0_reason_histogram.png` | `PASS` |
| `test -s .../figures/p0_1_pl_cost_distribution.png` | `PASS` |
| `test -s .../figures/p0_1_risk_grid_snapshot_overview.png` | `PASS` |
| `git -C src/iap diff --check` | `PASS` |

### Next Actions

1. P0-1 satisfies Phase 1 open-sky data-only validation criteria.
2. Next planned experiment is `P0-2 degraded GNSS + LiDAR good validation`.
3. Do not run P0-2, P5, P1, or P2 until explicitly requested.

## P0-2 Degraded GNSS + LiDAR Good Validation

### Repo And Environment

| Field | Value |
|---|---|
| Date | `2026-07-08` |
| Machine / container | `mint-X` |
| Workspace | `/home/dev/ws_iap` |
| Package repo | `/home/dev/ws_iap/src/iap` |
| Branch | `dev/iap` |
| Analyzer support commit | `13f9a5e docs: support P0-2 safety planner validation figures` |
| Repo state before launch | `clean`; `git status -sb` reported `## dev/iap...origin/dev/iap` |
| Recent commits before launch | `13f9a5e`, `2c464e2`, `ac238e6`, `64bfe5d`, `17a3195` |
| GPU check | `NVIDIA GeForce RTX 4070 Ti SUPER`, driver `580.126.09`, memory `16376 MiB` |
| Analyzer compile check | `python3 -m py_compile scripts/dev_planner/analyze_safety_planner_run.py` passed |

### Launch And Artifacts

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch iap test_planner.launch.py \
  experiment:=p0_open_sky \
  scenario:=gnss_degraded_lidar_good \
  run_duration_s:=60 \
  validation_duration_s:=60 \
  start_rviz:=false \
  run_validator:=true \
  record_bag:=true
```

| Field | Value |
|---|---|
| Experiment ID | `P0-2` |
| Experiment preset | `p0_open_sky` |
| Scenario | `gnss_degraded_lidar_good` |
| Launch result | `PASS`: command exited `0` |
| Overall validation result | `FAIL`: analyzer hard-failed P0 health thresholds |
| Export dir | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186` |
| Bag dir | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_degraded_lidar_good_20260708T040721Z` |
| Rosbag summary | Storage `mcap`; size `810.9 MiB`; duration `59.939377915s`; messages `342584` |
| Analyzer summary | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/metadata/safety_planner_analysis_summary.json` |

Analyzer command:

```bash
python3 src/iap/scripts/dev_planner/analyze_safety_planner_run.py \
  --experiment-id P0-2 \
  --export-dir src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186 \
  --bag-dir src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_degraded_lidar_good_20260708T040721Z \
  --baseline-export-dir src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509 \
  --baseline-bag-dir src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_open_sky_20260708T033119Z \
  --fail-on-threshold
```

### Parameter And Manifest Evidence

| Parameter | Value | Result |
|---|---:|---|
| `planner_safety_profile` | `off` | `PASS` |
| `p0.enable_risk_grid` | `true` | `PASS` |
| `planner_enable_p1` | `false` | `PASS` |
| `planner_enable_p2` | `false` | `PASS` |
| `planner_enable_p3_local` | `false` | `PASS` |
| `planner_enable_p3_global` | `false` | `PASS` |
| `planner_enable_p4` | `false` | `PASS` |
| `planner_enable_p5_runtime` | `false` | `PASS` |
| `planner_enable_p5_final` | `false` | `PASS` |
| Validator required fusion mode | `max_pl` | `PASS` |
| Validator required final source | empty | `PASS`; no final-source restriction for this fused scenario |
| Validator source validity | GNSS, LiDAR, fallback all seen | `PASS` |

### Validator And Analyzer

| Check | Result | Evidence |
|---|---|---|
| Validator summary | `PASS` | `passed=true`, `failures=[]`, `message_count=537` |
| Launch command | `PASS` | exit code `0` |
| Analyzer status | `FAIL` | command exited `2` with `--fail-on-threshold` |
| Analyzer failures | `FAIL` | sustained `ready=false`, sustained `stale=true`, and sustained full-frame unknown |
| Analyzer inconclusive / warnings | `PASS` | `[]` / `[]`; evidence is complete |
| Integrity CSV rows | `PASS` | `537` data rows |
| Fusion mode counts | `PASS` | `max_pl: 537` |
| Final HPL/VPL source counts | `PASS` | `GNSS: 537` for both HPL and VPL |
| Integrity HPL range | `PASS` | min/mean/max `24.381667m` / `28.219650m` / `37.281610m` |
| Integrity VPL range | `PASS` | min/mean/max `68.775856m` / `81.351300m` / `118.712729m` |
| P5 summary | `PASS` | `status_rows=0`, `bad_action_count=0`, `action_counts={}` |
| Analyzer next branch | `FAIL` | `debug_P0_risk_grid_health` |

### Topic Health

| Topic | Expected | Count | Hz | Span | Coverage | Max gap | Status |
|---|---|---:|---:|---:|---:|---:|---|
| `/iap/integrity` | continuous | `539` | `8.992419` | `57.691131s` | `0.962491` | `0.390889s` | `PASS` |
| `/sim/drone_0/lidar_body` | continuous | `541` | `9.025786` | `59.399587s` | `0.990994` | `0.500899s` | `PASS` |
| `/drone_0_visual_slam/odom` | continuous | `539` | `8.992419` | `57.690988s` | `0.962489` | `0.390895s` | `PASS` |
| `/drone_0_planning/bspline` | planner-dependent | `26` | `0.433772` | `17.432501s` | `0.290836` | `1.515381s` | `PASS` |
| `/planning/risk_grid_health` | active-periodic | `50` | `0.834176` | `49.000540s` | `0.817502` | `1.071119s` | `PASS` |
| `/iap/rviz/predicted_pl_cloud` | present | `45` | `0.750759` | `46.481041s` | `0.775468` | `1.070947s` | `PASS` |
| `/iap/rviz/risk_validity_cloud` | present | `45` | `0.750759` | `46.481000s` | `0.775467` | `1.070974s` | `PASS` |
| `/planning/integrity_gate_status` | absent-or-zero when P5 disabled | `0` | `0.000000` | n/a | n/a | n/a | `PASS` |

### P0 Health Summary

| Metric | Value | Result |
|---|---:|---|
| Health rows | `50` | `PASS` |
| `ready_false_count` / ratio / max consecutive | `5` / `0.100000` / `5` | `FAIL`; max consecutive is above the 2-sample limit |
| `stale_true_count` / ratio / max consecutive | `5` / `0.100000` / `5` | `FAIL`; max consecutive is above the 2-sample limit |
| `valid_ratio` min/mean/max | `0.000000` / `0.851816` / `0.987188` | `PASS` for mean `>0.6`; startup rows still include full-frame unknown |
| `unknown_ratio` min/mean/max | `0.012813` / `0.148184` / `1.000000` | `FAIL`; max reaches full-frame unknown |
| Full-frame unknown count / ratio / max consecutive | `6` / `0.120000` / `6` | `FAIL`; max consecutive is above the 2-sample limit |
| Refresh elapsed mean | `489.632ms` | `PASS` |
| Provider query count max | `63290` | `PASS` |
| Provider stale count max | `63290` | `FAIL` evidence for stale provider coverage during the failed interval |
| Provider invalid count max | `0` | `PASS` |
| Occupied skip count max | `2975` | `PASS` as partial unknown evidence after startup; not the hard failure source |

### Reason Histogram

| Reason / counter | Count or max | Conclusion |
|---|---:|---|
| `ok` health rows | `44` | Dominant steady-state reason |
| `snapshot_unavailable` health rows | `5` | `FAIL`; first five health rows are not ready/stale with `valid_ratio=0`, `unknown_ratio=1` |
| `stale_gnss_epoch` health rows | `1` | `FAIL`; extends the full-frame unknown sequence to six samples |
| Empty reason rows | `0` | `PASS` |
| `provider_stale_count_max` | `63290` | `FAIL` evidence during the stale/unknown interval |
| `provider_invalid_count_max` | `0` | `PASS` |
| `occupied_skip_count_max` | `2975` | Steady-state unknown area remains partial, but startup full-frame unknown already fails |

### PL / Cost Distribution

| Metric | Value | Conclusion |
|---|---:|---|
| PL min/mean/max | `19.596582m` / `19.596582m` / `19.596582m` | Higher than P0-1 as expected |
| Cost min/mean/max | `19.596582` / `19.596582` / `19.596582` | Higher than P0-1 as expected |
| Valid cells | `3091` of `3200` (`96.593750%`) | `PASS` for latest PL cloud |
| Unknown cells | `109` of `3200` (`3.406250%`) | `PASS` for latest PL cloud |
| Stale cells | `0` | `PASS` for latest PL cloud |
| Cloud summary rows | `45` | `PASS`; one early row was full unknown before valid PL/cost values appeared |

### P0-1 Vs P0-2 Comparison

| Metric | P0-1 open-sky baseline | P0-2 degraded GNSS + LiDAR good | Delta | Result |
|---|---:|---:|---:|---|
| PL mean | `11.181885m` | `19.596582m` | `+8.414698m` | `PASS`; exceeds `0.5m` threshold |
| Cost mean | `11.181885` | `19.596582` | `+8.414698` | `PASS`; exceeds `0.5` threshold |
| Valid ratio | `0.993750` | `0.965938` | `-0.027813` | `PASS`; still high in latest cloud |
| Unknown ratio | `0.006250` | `0.034063` | `+0.027813` | Informational; latest cloud is not full-frame unknown |
| Stale ratio | `0.000000` | `0.000000` | `0.000000` | `PASS` for latest cloud |
| Truth sample count | `1937` | `1995` | `+58` | Informational |
| Truth path length | `24.389676m` | `26.756694m` | `+2.367018m` | Informational |
| Resampled truth RMS distance | n/a | `0.651968m` | n/a | Informational |

Baseline source for the PL/cost comparison: `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_open_sky_1783481479509/csv/p0_1_pl_cloud.csv`.

### Acceptance Matrix

| Criterion | Result | Evidence |
|---|---|---|
| Launch exits `0` | `PASS` | `ros2 launch` returned exit code `0` |
| Validator summary `passed=true` | `PASS` | `test_planner_validation_summary.json` reports `passed: true` |
| Analyzer status `PASS` | `FAIL` | Analyzer exited `2`; status `FAIL` |
| Manifest has `p0.enable_risk_grid=true` | `PASS` | Manifest |
| P1-P5 planner switches all disabled | `PASS` | Manifest records all `planner_enable_*` fields as `false` |
| Safety profile `off` | `PASS` | Manifest |
| Risk-grid health rows present | `PASS` | `50` rows |
| `ready_false_ratio <= 0.10` | `PASS` | ratio is exactly `0.10` |
| No 3-sample consecutive `ready=false` | `FAIL` | max consecutive `5` |
| `stale_true_ratio <= 0.10` | `PASS` | ratio is exactly `0.10` |
| No 3-sample consecutive `stale=true` | `FAIL` | max consecutive `5` |
| `valid_ratio_mean > 0.6` | `PASS` | mean `0.851816` |
| No periodic/full-frame unknown cycle | `FAIL` | full-frame unknown max consecutive `6` |
| Full-frame unknown ratio <= 25% | `PASS` | ratio `0.12` |
| Non-empty health reasons | `PASS` | no empty reason rows |
| Counters explain non-ok health | `PASS` | `snapshot_unavailable`, `stale_gnss_epoch`, provider stale count max `63290` |
| P0 RViz predicted PL cloud has messages | `PASS` | bag/analyzer count `45` |
| P0 RViz risk validity cloud has messages | `PASS` | bag/analyzer count `45` |
| `/planning/integrity_gate_status` absent or no P5 action | `PASS` | count `0`; p5 summary has no actions |
| PL/cost higher than P0-1 | `PASS` | PL/cost mean delta `+8.414698`, threshold `0.5` |
| Required figures are present and non-empty | `PASS` | `test -s` passed for all seven requested P0-2 figures |
| No P0-3/P5/P1/P2 or aggregate run | `PASS` | Only the P0-2 launch command above was run after analyzer support |

### Failure Analysis

P0-2 fails because risk-grid health is not continuously ready after planner activation. The first five health rows are `snapshot_unavailable` with `ready=false`, `stale=true`, `valid_ratio=0`, and `unknown_ratio=1`. The next non-ok row is `stale_gnss_epoch`, which extends the full-frame unknown sequence to six samples. This violates the P0-2 hard gates for consecutive readiness, consecutive staleness, and consecutive full-frame unknown frames.

The steady-state behavior after that startup interval is healthier: the final rows are `ok`, `ready=true`, `stale=false`, with `valid_ratio=0.967266` and `unknown_ratio=0.032734`. The latest PL/cost cloud is materially higher than P0-1, so the degraded-GNSS cost response appears visible. The failure is specifically the sustained startup health gap, not missing artifacts, missing clouds, absent PL/cost differentiation, or P5 leakage.

The launch log contains shutdown-time ROS exceptions and signal-based exits for helper nodes after the validator passed and rosbag recording stopped. They are recorded as non-blocking for this run because the launch command returned `0`, the validator passed, the bag/export evidence is complete, and the analyzer failure is already explained by P0 health data.

### Remaining Issues

| Issue | Severity | Owner | Blocking experiment | Next action |
|---|---|---|---|---|
| P0 risk-grid health has a sustained startup gap | High | P0 risk-grid / planner integration | Blocks P0-2 acceptance | Debug why planner-visible health publishes `snapshot_unavailable` for five samples and then `stale_gnss_epoch` before settling |
| Full-frame unknown sequence during startup | High | P0 risk-grid provider lifecycle | Blocks P0-2 acceptance | Gate publication until first valid snapshot or mark startup separately if that is intended semantics |
| Provider stale counter spikes to `63290` | Medium | P0 provider/fusion health accounting | Related to P0-2 failure | Trace GNSS epoch freshness and provider timestamp alignment for the first valid risk-grid cycles |
| Validity cloud scalar CSV summary has `row_count=0` | Low | dev_planner analyzer | Not blocking P0-2 evidence because topic messages and overview figure exist | Extend scalar extraction for validity-only cloud fields if future reports need numeric validity-cloud statistics |
| SIGINT teardown exceptions in helper nodes | Low | Planner/sim runtime | Not blocking this evidence | Track separately if teardown stability becomes a CI requirement |

### Key Artifacts

| Artifact | Path | Conclusion |
|---|---|---|
| Manifest JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/test_planner_manifest.json` | P0 enabled, safety profile `off`, P1-P5 disabled |
| Validator summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/test_planner_validation_summary.json` | Validator passed with `537` messages |
| Analyzer summary JSON | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/metadata/safety_planner_analysis_summary.json` | Analyzer status `FAIL` |
| Scenario top-down | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_scenario_topdown.png` | Includes P0-1 baseline truth overlay |
| Topic activity timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_topic_activity_timeline.png` | Includes P0 health plus predicted PL and risk-validity cloud topics |
| P0 health timeline | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_p0_health_timeline.png` | Shows startup full-frame unknown and later recovery |
| P0 reason histogram | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_p0_reason_histogram.png` | Shows `snapshot_unavailable`, `stale_gnss_epoch`, and dominant `ok` |
| PL/cost distribution | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_pl_cost_distribution.png` | Latest PL/cost distribution centered at `19.596582` |
| Risk grid snapshot overview | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_risk_grid_snapshot_overview.png` | Latest predicted PL and validity clouds render |
| P0-2 vs P0-1 delta | `/home/dev/ws_iap/src/iap/results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_vs_p0_1_delta.png` | PL/cost mean increases by `8.414698` |
| Rosbag | `/home/dev/ws_iap/src/iap/results/planner_validation/bags/test_planner_p0_open_sky_gnss_degraded_lidar_good_20260708T040721Z` | Required P0-2 topics recorded |

![P0-2 scenario top-down](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_scenario_topdown.png)

![P0-2 topic activity timeline](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_topic_activity_timeline.png)

![P0-2 P0 health timeline](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_p0_health_timeline.png)

![P0-2 P0 reason histogram](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_p0_reason_histogram.png)

![P0-2 PL/cost distribution](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_pl_cost_distribution.png)

![P0-2 risk grid snapshot overview](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_risk_grid_snapshot_overview.png)

![P0-2 vs P0-1 delta](../../results/planner_validation/exports/test_planner_p0_open_sky_gnss_degraded_lidar_good_1783483641186/figures/p0_2_vs_p0_1_delta.png)

### Verification Commands

| Command | Result |
|---|---|
| `git -C src/iap status -sb` | `PASS`; clean before P0-2 launch |
| `git -C src/iap log --oneline -5` | `PASS`; latest pre-launch commit `13f9a5e` |
| `nvidia-smi` | `PASS`; RTX 4070 Ti SUPER visible |
| `python3 -m py_compile src/iap/scripts/dev_planner/analyze_safety_planner_run.py` | `PASS` |
| P0-2 launch command above | `PASS`; exit code `0` |
| Artifact inspection commands | `PASS`; manifest, validator summary, and rosbag metadata readable |
| Analyzer command above | `FAIL`; status `FAIL`, exit code `2` |
| `test -s .../figures/p0_2_scenario_topdown.png` | `PASS` |
| `test -s .../figures/p0_2_topic_activity_timeline.png` | `PASS` |
| `test -s .../figures/p0_2_p0_health_timeline.png` | `PASS` |
| `test -s .../figures/p0_2_p0_reason_histogram.png` | `PASS` |
| `test -s .../figures/p0_2_pl_cost_distribution.png` | `PASS` |
| `test -s .../figures/p0_2_risk_grid_snapshot_overview.png` | `PASS` |
| `test -s .../figures/p0_2_vs_p0_1_delta.png` | `PASS` |

### Next Actions

1. Do not proceed to `P0-3` until P0-2 risk-grid startup health is fixed or the acceptance criteria are explicitly revised.
2. Debug branch: `debug_P0_risk_grid_health`.
3. Start with the first six P0 health samples: five `snapshot_unavailable` rows followed by one `stale_gnss_epoch`, all contributing to full-frame unknown.
4. Re-run only P0-2 after the fix; do not run P0-3, P5, P1, P2, or aggregate experiments as part of this evidence chain.
