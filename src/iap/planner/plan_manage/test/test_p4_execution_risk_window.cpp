#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "ego_planner/p4_execution_risk_window.h"

namespace ego_planner {
namespace {

std::vector<P4ExecutionRiskSample> nominalSamples(const double duration_s) {
  std::vector<P4ExecutionRiskSample> samples;
  const int count = static_cast<int>(std::lround(duration_s / 0.2));
  for (int index = 0; index <= count; ++index) {
    const double time_s = duration_s * static_cast<double>(index) /
        static_cast<double>(count);
    samples.push_back({Eigen::Vector3d(time_s, 0.1 * time_s * time_s, 1.0),
                       time_s});
  }
  return samples;
}

std::vector<P4BrakingRiskCurveSamples> brakingCurves(
    const double duration_s) {
  std::vector<P4BrakingRiskCurveSamples> curves;
  const int anchor_count = static_cast<int>(std::ceil(duration_s / 0.2));
  for (int index = 0; index < anchor_count; ++index) {
    const double anchor = duration_s * static_cast<double>(index) /
        static_cast<double>(anchor_count);
    P4BrakingRiskCurveSamples curve;
    curve.anchor_time_s = anchor;
    const double braking_duration = std::min(0.6, duration_s - anchor);
    const int sample_count = std::max(
        1, static_cast<int>(std::ceil(braking_duration / 0.2)));
    for (int sample = 0; sample <= sample_count; ++sample) {
      const double dt = braking_duration * static_cast<double>(sample) /
          static_cast<double>(sample_count);
      curve.samples.push_back({
          Eigen::Vector3d(anchor + 0.5 * dt, 0.1 * anchor * anchor, 1.0),
          anchor + dt});
    }
    curves.push_back(std::move(curve));
  }
  return curves;
}

TEST(P4ExecutionRiskWindowTest,
     DynamicWindowsCoverNominalCurveBrakesAndDualCertifiedTransitions) {
  const auto nominal = nominalSamples(4.0);
  const auto brakes = brakingCurves(4.0);
  P4ExecutionRiskWindowParams params;
  params.reaction_time_s = 1.2;
  params.transition_overlap_s = 0.4;
  params.maximum_anchor_gap_s = 0.2;

  const auto layout = buildP4ExecutionRiskWindowLayout(
      nominal, brakes, params);

  ASSERT_TRUE(layout.valid) << layout.reason;
  ASSERT_GE(layout.windows.size(), 3u);
  EXPECT_EQ(layout.transition_count, layout.windows.size() - 1u);
  std::set<std::uint64_t> unique_evidence_ids;
  for (const auto& row : layout.rows) {
    unique_evidence_ids.insert(row.evidence_point_id);
  }
  EXPECT_EQ(layout.unique_evidence_point_count,
            unique_evidence_ids.size());

  std::set<double> covered_nominal_times;
  std::vector<std::set<std::uint64_t>> evidence_windows(
      layout.unique_evidence_point_count + 1u);
  for (const auto& row : layout.rows) {
    EXPECT_GT(row.evidence_point_id, 0u);
    EXPECT_GT(row.satellite_window_id, 0u);
    if (row.nominal) covered_nominal_times.insert(row.sample.relative_time_s);
    if (row.evidence_point_id < evidence_windows.size()) {
      evidence_windows[row.evidence_point_id].insert(
          row.satellite_window_id);
    }
  }
  for (const auto& sample : nominal) {
    EXPECT_EQ(covered_nominal_times.count(sample.relative_time_s), 1u);
  }
  EXPECT_TRUE(std::any_of(
      evidence_windows.begin(), evidence_windows.end(),
      [](const auto& windows) { return windows.size() == 2u; }));

  std::set<std::size_t> covered_brakes;
  for (const auto& row : layout.rows) {
    covered_brakes.insert(
        row.braking_curve_indices.begin(), row.braking_curve_indices.end());
  }
  EXPECT_EQ(covered_brakes.size(), brakes.size());
  EXPECT_FALSE(layout.identity_hash.empty());
}

TEST(P4ExecutionRiskWindowTest, RejectsMissingCertifiedBrakingAnchorGap) {
  const auto nominal = nominalSamples(2.0);
  auto brakes = brakingCurves(2.0);
  brakes.erase(brakes.begin() + 3);

  const auto layout = buildP4ExecutionRiskWindowLayout(
      nominal, brakes, P4ExecutionRiskWindowParams{});

  EXPECT_FALSE(layout.valid);
  EXPECT_EQ(layout.reason, "braking_anchor_gap_exceeds_limit");
}

TEST(P4ExecutionRiskWindowTest,
     CommittedPlanKeepsLayoutIdentityAndWindowIdsAsExecutionAdvances) {
  const auto nominal = nominalSamples(4.0);
  const auto brakes = brakingCurves(4.0);
  const auto plan = buildP4CommittedRiskWindowPlan(
      42, 123456789, "control-hash", "knot-hash", nominal, brakes,
      P4ExecutionRiskWindowParams{});
  ASSERT_TRUE(plan.valid) << plan.reason;
  ASSERT_GE(plan.layout.windows.size(), 3u);

  const auto early = selectP4CommittedRiskWindowRows(plan, 0.31);
  const auto later = selectP4CommittedRiskWindowRows(plan, 1.41);
  ASSERT_TRUE(early.valid) << early.reason;
  ASSERT_TRUE(later.valid) << later.reason;
  EXPECT_EQ(early.window_layout_hash, plan.layout.identity_hash);
  EXPECT_EQ(later.window_layout_hash, plan.layout.identity_hash);
  EXPECT_EQ(early.current_window_id, plan.layout.windows.front().window_id);
  EXPECT_EQ(later.current_window_id, plan.layout.windows[1].window_id);
  EXPECT_EQ(early.next_window_id, plan.layout.windows[1].window_id);
  EXPECT_EQ(later.next_window_id, plan.layout.windows[2].window_id);

  const std::set<std::uint64_t> original_ids = [&plan] {
    std::set<std::uint64_t> ids;
    for (const auto& window : plan.layout.windows) ids.insert(window.window_id);
    return ids;
  }();
  for (const auto& selection : {early, later}) {
    for (const auto& window : selection.windows)
      EXPECT_EQ(original_ids.count(window.window_id), 1u);
    ASSERT_EQ(selection.rows.size(), selection.source_row_indices.size());
    for (std::size_t index = 0; index < selection.rows.size(); ++index) {
      ASSERT_LT(selection.source_row_indices[index], plan.layout.rows.size());
      const auto& source = plan.layout.rows[selection.source_row_indices[index]];
      EXPECT_EQ(selection.rows[index].evidence_point_id,
                source.evidence_point_id);
      EXPECT_EQ(selection.rows[index].satellite_window_id,
                source.satellite_window_id);
      EXPECT_DOUBLE_EQ(selection.rows[index].sample.relative_time_s,
                       source.sample.relative_time_s);
    }
  }
}

TEST(P4ExecutionRiskWindowTest,
     SelectionKeepsDualTransitionRowsAndDropsUnreachableBrakeAnchors) {
  const auto nominal = nominalSamples(4.0);
  const auto brakes = brakingCurves(4.0);
  const auto plan = buildP4CommittedRiskWindowPlan(
      7, 77, "cp", "knots", nominal, brakes,
      P4ExecutionRiskWindowParams{});
  ASSERT_TRUE(plan.valid) << plan.reason;

  const auto selected = selectP4CommittedRiskWindowRows(plan, 1.21);
  ASSERT_TRUE(selected.valid) << selected.reason;
  std::map<std::uint64_t, std::set<std::uint64_t>> memberships;
  for (const auto& row : selected.rows) {
    EXPECT_GE(row.sample.relative_time_s, 1.21 - 1.0e-9);
    if (row.braking) {
      ASSERT_FALSE(row.braking_curve_indices.empty());
      EXPECT_TRUE(row.nominal || std::any_of(
          row.braking_curve_indices.begin(), row.braking_curve_indices.end(),
          [&plan](const std::size_t curve_index) {
            return curve_index < plan.braking_curves.size() &&
                plan.braking_curves[curve_index].anchor_time_s >=
                    1.21 - 1.0e-9;
          }));
    }
    memberships[row.evidence_point_id].insert(row.satellite_window_id);
  }
  EXPECT_TRUE(std::any_of(
      memberships.begin(), memberships.end(),
      [](const auto& item) { return item.second.size() == 2u; }));
}

TEST(P4ExecutionRiskWindowTest,
     ReachableSelectionCacheIdentityAdvancesInsideOneWindow) {
  const auto plan = buildP4CommittedRiskWindowPlan(
      17, 177, "cp", "knots", nominalSamples(4.0), brakingCurves(4.0),
      P4ExecutionRiskWindowParams{});
  ASSERT_TRUE(plan.valid) << plan.reason;

  const auto before_sample = selectP4CommittedRiskWindowRows(plan, 0.31);
  const auto after_sample = selectP4CommittedRiskWindowRows(plan, 0.51);
  ASSERT_TRUE(before_sample.valid) << before_sample.reason;
  ASSERT_TRUE(after_sample.valid) << after_sample.reason;
  ASSERT_EQ(before_sample.current_window_id, after_sample.current_window_id);
  ASSERT_EQ(before_sample.next_window_id, after_sample.next_window_id);

  EXPECT_NE(p4CommittedRiskWindowSelectionCacheIdentity(before_sample),
            p4CommittedRiskWindowSelectionCacheIdentity(after_sample));
  EXPECT_GT(before_sample.rows.size(), after_sample.rows.size());
  for (const auto& row : after_sample.rows) {
    EXPECT_GE(row.sample.relative_time_s, 0.51 - 1.0e-9);
  }
}

TEST(P4ExecutionRiskWindowTest,
     NominalEndpointSurvivesAColocatedExpiredBrakeEndpoint) {
  const auto nominal = nominalSamples(4.0);
  auto brakes = brakingCurves(4.0);
  auto merged_curve = std::find_if(
      brakes.begin(), brakes.end(), [](const auto& curve) {
        return curve.anchor_time_s > 3.3 &&
            std::abs(curve.samples.back().relative_time_s - 4.0) < 1.0e-9;
      });
  ASSERT_NE(merged_curve, brakes.end());
  merged_curve->samples.back() = nominal.back();
  const std::size_t expired_curve_index = static_cast<std::size_t>(
      std::distance(brakes.begin(), merged_curve));
  auto reachable_curve = std::find_if(
      std::next(merged_curve), brakes.end(), [](const auto& curve) {
        return curve.anchor_time_s > 3.5 &&
            std::abs(curve.samples.back().relative_time_s - 4.0) < 1.0e-9;
      });
  ASSERT_NE(reachable_curve, brakes.end());
  reachable_curve->samples.back() = nominal.back();
  const std::size_t reachable_curve_index = static_cast<std::size_t>(
      std::distance(brakes.begin(), reachable_curve));
  const auto plan = buildP4CommittedRiskWindowPlan(
      8, 88, "cp", "knots", nominal, brakes,
      P4ExecutionRiskWindowParams{});
  ASSERT_TRUE(plan.valid) << plan.reason;

  const auto selected = selectP4CommittedRiskWindowRows(plan, 4.0);

  ASSERT_TRUE(selected.valid) << selected.reason;
  ASSERT_FALSE(selected.rows.empty());
  EXPECT_TRUE(std::any_of(
      selected.rows.begin(), selected.rows.end(), [](const auto& row) {
        return row.nominal &&
            std::abs(row.sample.relative_time_s - 4.0) < 1.0e-9;
      }));
  const auto reachable = reachableP4CommittedBrakingCurveIndices(
      plan, selected, 3.5);
  EXPECT_EQ(std::count(
      reachable.begin(), reachable.end(), expired_curve_index), 0);
  EXPECT_EQ(std::count(
      reachable.begin(), reachable.end(), reachable_curve_index), 1);
}

TEST(P4ExecutionRiskWindowTest,
     RejectedAndIncompleteBatchesStillProduceWindowEvidence) {
  const auto plan = buildP4CommittedRiskWindowPlan(
      11, 99, "cp", "knots", nominalSamples(4.0), brakingCurves(4.0),
      P4ExecutionRiskWindowParams{});
  ASSERT_TRUE(plan.valid) << plan.reason;
  const auto selection = selectP4CommittedRiskWindowRows(plan, 0.31);
  ASSERT_TRUE(selection.valid) << selection.reason;

  iap::ForwardRiskBatchResult unsafe;
  unsafe.complete = true;
  unsafe.points.resize(selection.rows.size());
  for (std::size_t index = 0; index < unsafe.points.size(); ++index) {
    unsafe.points[index].safety_state = iap::ForwardRiskSafetyState::SAFE;
    unsafe.points[index].ranking_state =
        iap::ForwardRiskRankingState::COMPARABLE;
    unsafe.points[index].failure_reason =
        iap::ForwardRiskFailureReason::NONE;
    unsafe.points[index].safety_ratio = 0.5 + 0.001 * index;
  }
  const std::size_t rejected_index = unsafe.points.size() / 2u;
  unsafe.points[rejected_index].safety_state =
      iap::ForwardRiskSafetyState::UNSAFE;
  unsafe.points[rejected_index].failure_reason =
      iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
  unsafe.points[rejected_index].safety_ratio = 1.25;
  for (const auto& window : selection.windows) {
    iap::ForwardRiskWindowResult result;
    result.satellite_window_id = window.window_id;
    result.satellite_ids = {3, 7, 12, 18, 23};
    result.satellite_set_hash = 1234;
    result.complete = true;
    result.point_count = window.request_row_indices.size();
    unsafe.windows.push_back(std::move(result));
  }

  const auto captured = buildP4RuntimeWindowEvidence(
      5, plan, selection, 31, 41, 42, 51, 61, 10.0, unsafe);
  EXPECT_TRUE(captured.complete);
  EXPECT_EQ(captured.sequence_id, 5u);
  EXPECT_EQ(captured.window_layout_hash, plan.layout.identity_hash);
  EXPECT_EQ(captured.windows.size(), selection.windows.size());
  EXPECT_EQ(captured.global_worst_nominal_index, rejected_index);
  EXPECT_FALSE(captured.per_window_worst.empty());

  iap::ForwardRiskBatchResult incomplete;
  incomplete.complete = false;
  incomplete.failure_reason =
      iap::ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED;
  const auto failed = buildP4RuntimeWindowEvidence(
      6, plan, selection, 32, 43, 44, 52, 62, 10.1, incomplete);
  EXPECT_FALSE(failed.complete);
  EXPECT_EQ(failed.active_windows.size(), selection.windows.size());
  EXPECT_EQ(failed.rows.size(), selection.rows.size());
  EXPECT_EQ(failed.reason, iap::forwardRiskFailureReasonName(
      iap::ForwardRiskFailureReason::COMPUTE_BUDGET_EXCEEDED));
}

}  // namespace
}  // namespace ego_planner
