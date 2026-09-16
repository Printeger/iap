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
    if (row.braking) covered_brakes.insert(row.braking_curve_index);
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

}  // namespace
}  // namespace ego_planner
