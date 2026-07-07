# Safety Planner Integration Test Report Template

> 每次 Safety Planner P0-P5 验证后复制本模板填写。不要只写“通过/失败”；必须附上 commit、命令、topic health、自动分析 summary、关键图表和 next actions。

## 1. 测试环境

| 字段 | 值 |
|---|---|
| 日期 | `YYYY-MM-DD` |
| 操作者 / Agent |  |
| 机器 / 容器 |  |
| OS / ROS |  |
| Workspace | `/home/dev/ws_iap` |
| Package | `iap` |
| GPU/CPU 备注 |  |

## 2. 代码与构建

| 字段 | 值 |
|---|---|
| Commit hash |  |
| Branch |  |
| Dirty worktree? | `yes/no` |
| Build command |  |
| Build result | `PASS/FAIL` |
| Unit test command |  |
| Unit test result | `PASS/FAIL` |

## 3. Launch 与参数

| 字段 | 值 |
|---|---|
| Experiment ID |  |
| Experiment preset |  |
| Scenario |  |
| Launch command |  |
| Run duration |  |
| Validation duration |  |
| Export dir |  |
| Bag dir |  |
| Analyzer command |  |
| Analyzer summary |  |

### 参数表

| 参数 | 值 | 说明 |
|---|---:|---|
| `planner_safety_profile` |  |  |
| `p0.enable_risk_grid` |  |  |
| `p0.gnss_epoch_max_age_s` |  |  |
| `planner_enable_p1` |  |  |
| `planner_enable_p2` |  |  |
| `planner_enable_p3_local` |  |  |
| `planner_enable_p3_global` |  |  |
| `planner_enable_p4` |  |  |
| `planner_enable_p5_runtime` |  |  |
| `planner_enable_p5_final` |  |  |
| `p1.metrics_only` |  |  |
| `p2.metrics_only` |  |  |
| `p5.pred_alert_limit_mode` |  |  |

## 4. Topic Health

| Topic | Expected | Observed count / Hz | Status | Notes |
|---|---|---:|---|---|
| `/iap/integrity` | continuous |  |  |  |
| `/sim/drone_0/lidar_body` | continuous |  |  |  |
| `/drone_0_planning/bspline` | planner-dependent |  |  |  |
| `/planning/risk_grid_health` | P0 debug on |  |  |  |
| `/planning/integrity_gate_status` | P5 debug on |  |  |  |
| `/iap/rviz/predicted_pl_cloud` | P0 on |  |  |  |
| `/iap/rviz/risk_validity_cloud` | P0 on |  |  |  |
| `/iap/rviz/p5_gate_status` | P5 on |  |  |  |
| `/iap/rviz/p1_integrity_metrics` | P1 viz on |  |  |  |
| `/iap/rviz/p2_candidate_trajectories` | P2 viz on |  |  |  |
| `/iap/rviz/p3_reference_bias` | P3 viz on |  |  |  |
| `/iap/rviz/p4_astar_guides` | P4 viz on |  |  |  |

## 5. 实验结果表

| Experiment ID | Status | Validator | Analyzer | Key evidence | Failure / warning | Next branch |
|---|---|---|---|---|---|---|
| B0-1 |  |  |  |  |  |  |
| B0-2 |  |  |  |  |  |  |
| B0-3 |  |  |  |  |  |  |
| B0-4 |  |  |  |  |  |  |
| P0-1 |  |  |  |  |  |  |
| P0-2 |  |  |  |  |  |  |
| P0-3 |  |  |  |  |  |  |
| P0-4 |  |  |  |  |  |  |
| P0-5 |  |  |  |  |  |  |
| P0-6 |  |  |  |  |  |  |
| P5-1 |  |  |  |  |  |  |
| P5-2 |  |  |  |  |  |  |
| P5-3 |  |  |  |  |  |  |
| P5-4 |  |  |  |  |  |  |
| P5-5 |  |  |  |  |  |  |
| P5-6 |  |  |  |  |  |  |
| P5-7 |  |  |  |  |  |  |
| P5-8 |  |  |  |  |  |  |
| P1-1 |  |  |  |  |  |  |
| P1-2 |  |  |  |  |  |  |
| P1-3 |  |  |  |  |  |  |
| P1-4 |  |  |  |  |  |  |
| P1-5 |  |  |  |  |  |  |
| P1-6 |  |  |  |  |  |  |
| P2-1 |  |  |  |  |  |  |
| P2-2 |  |  |  |  |  |  |
| P2-3 |  |  |  |  |  |  |
| P2-4 |  |  |  |  |  |  |
| P2-5 |  |  |  |  |  |  |
| P2-6 |  |  |  |  |  |  |
| P3-1 |  |  |  |  |  |  |
| P3-2 |  |  |  |  |  |  |
| P3-3 |  |  |  |  |  |  |
| P3-4 |  |  |  |  |  |  |
| P3-5 |  |  |  |  |  |  |
| P3-6 |  |  |  |  |  |  |
| P3-7 |  |  |  |  |  |  |
| P4-1 |  |  |  |  |  |  |
| P4-2 |  |  |  |  |  |  |
| P4-3 |  |  |  |  |  |  |
| P4-4 |  |  |  |  |  |  |
| P4-5 |  |  |  |  |  |  |
| P4-6 |  |  |  |  |  |  |
| A-0 |  |  |  |  |  |  |
| A-P0 |  |  |  |  |  |  |
| A-P5 |  |  |  |  |  |  |
| A-P1 |  |  |  |  |  |  |
| A-P2 |  |  |  |  |  |  |
| A-P3 |  |  |  |  |  |  |
| A-P4 |  |  |  |  |  |  |
| A-P1P5 |  |  |  |  |  |  |
| A-P123 |  |  |  |  |  |  |
| A-ALL |  |  |  |  |  |  |
| R-1 |  |  |  |  |  |  |
| R-2 |  |  |  |  |  |  |
| R-3 |  |  |  |  |  |  |
| R-4 |  |  |  |  |  |  |
| R-5 |  |  |  |  |  |  |
| R-6 |  |  |  |  |  |  |
| R-7 |  |  |  |  |  |  |
| R-8 |  |  |  |  |  |  |
| R-9 |  |  |  |  |  |  |
| R-10 |  |  |  |  |  |  |

## 6. P0 验收

| Criterion | Result | Evidence |
|---|---|---|
| `ready=true` in normal runs |  |  |
| `stale=false` in normal runs |  |  |
| No periodic all-unknown frame |  |  |
| Unknown/stale/out-of-range not mapped to zero risk |  |  |
| Reason histogram explains failures |  |  |
| PL/cost distribution distinguishes open-sky/degraded/corridor |  |  |
| Occupied skip counters/reason are consistent |  |  |

## 7. P5 验收

| Criterion | Result | Evidence |
|---|---|---|
| Nominal/open-sky action stays `OK` |  |  |
| Future bad triggers `REQUEST_REPLAN` |  |  |
| Near bad triggers emergency candidate |  |  |
| Current stale escalates with duration |  |  |
| Future unknown escalates with duration |  |  |
| Final gate reject prevents unsafe trajectory publish |  |  |
| P5 uses PL/AL/IM, not `c_pi` |  |  |

## 8. P1/P2/P3/P4 验收

| Module | Criterion | Result | Evidence |
|---|---|---|---|
| P1 | metrics-only does not alter objective |  |  |
| P1 | enabled lowers mean/max risk without feasibility loss |  |  |
| P2 | metrics-only does not change winner |  |  |
| P2 | enabled uses `original_cost`, avoids double count |  |  |
| P3 | local/global bias respects coverage and detour gates |  |  |
| P4 | only runs on collision segment |  |  |
| P4 | occupied remains hard rejection |  |  |

## 9. 关键图表清单

| 图表 | 路径 | 结论 |
|---|---|---|
| P0 health timeline |  |  |
| P0 PL/cost distribution |  |  |
| P0 reason histogram |  |  |
| P5 action timeline |  |  |
| Future IM profile |  |  |
| Trajectory overlay |  |  |
| Baseline vs module metrics |  |  |
| Runtime profile |  |  |

## 10. 失败案例分析

### Failure ID

| 字段 | 内容 |
|---|---|
| Experiment |  |
| Symptom |  |
| First failing timestamp |  |
| Relevant topic/topic gap |  |
| Relevant CSV rows |  |
| Dominant reason/action |  |
| Hypothesis |  |
| Probe performed |  |
| Conclusion |  |
| Fix / next action |  |

## 11. Remaining Issues

| Issue | Severity | Owner | Blocking experiment | Next action |
|---|---|---|---|---|
|  |  |  |  |  |

## 12. Next Actions

1.
2.
3.
