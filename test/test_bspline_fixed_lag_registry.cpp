// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the unified fixed-lag control/segment lifecycle registry.

#include <gtest/gtest.h>

#include <iap/odometry/bspline_fixed_lag_registry.hpp>

namespace {

iap::BSplineFixedLagSegmentState make_segment_state(const iap::BSplineControlWindow& window) {
  iap::BSplineFixedLagSegmentState state;
  state.stamp = window.segment_start();
  state.scan_end = window.segment_end();
  const auto control_states = window.states();
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    state.control_indices[i] = control_states[i].index;
  }
  state.auxiliary_index = control_states[1].index;
  return state;
}

}  // namespace

TEST(BSplineFixedLagRegistryTest, PruneKeepsControlSupportAndSegmentLifecycleAligned) {
  iap::BSplineControlWindow window;
  window.initialize(0.0, 0.1, gtsam::Pose3());

  iap::BSplineFixedLagStateRegistry registry;
  registry.reset_from_window(window);
  registry.append_segment(make_segment_state(window));

  window.advance(0.1, 0.2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
  registry.append_window(window);
  registry.append_segment(make_segment_state(window));

  window.advance(0.2, 0.3, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0)));
  registry.append_window(window);
  registry.append_segment(make_segment_state(window));

  ASSERT_EQ(registry.control_buffer().size(), 6U);
  ASSERT_EQ(registry.segments().size(), 3U);

  gtsam::Values aux_values;
  aux_values.insert(iap::bspline_velocity_key(1), gtsam::Vector3(1.0, 0.0, 0.0));
  aux_values.insert(iap::bspline_velocity_key(2), gtsam::Vector3(2.0, 0.0, 0.0));
  aux_values.insert(iap::bspline_velocity_key(3), gtsam::Vector3(3.0, 0.0, 0.0));
  aux_values.insert(iap::bspline_clock_key(1), gtsam::Vector2(10.0, 0.1));
  aux_values.insert(iap::bspline_clock_key(2), gtsam::Vector2(20.0, 0.2));
  aux_values.insert(iap::bspline_clock_key(3), gtsam::Vector2(30.0, 0.3));

  registry.prune_before(0.25);

  ASSERT_EQ(registry.control_buffer().size(), 5U);
  EXPECT_EQ(registry.control_buffer().states().front().index, 1U);
  ASSERT_EQ(registry.segments().size(), 1U);
  EXPECT_EQ(registry.segments().front().auxiliary_index, 3U);
  EXPECT_EQ(registry.active_auxiliary_indices(), (std::vector<std::size_t>{3U}));
  EXPECT_TRUE(registry.contains_auxiliary_index(3U));
  EXPECT_FALSE(registry.contains_auxiliary_index(2U));

  const auto marginal_states = registry.marginalization_segment_states();
  ASSERT_EQ(marginal_states.size(), 1U);
  EXPECT_EQ(marginal_states.front().auxiliary_index, 3U);
  EXPECT_EQ(marginal_states.front().control_indices[0], 2U);

  const auto filtered = registry.filter_aux_values(aux_values, true);
  EXPECT_TRUE(filtered.exists(iap::bspline_velocity_key(3)));
  EXPECT_TRUE(filtered.exists(iap::bspline_clock_key(3)));
  EXPECT_FALSE(filtered.exists(iap::bspline_velocity_key(1)));
  EXPECT_FALSE(filtered.exists(iap::bspline_velocity_key(2)));
  EXPECT_FALSE(filtered.exists(iap::bspline_clock_key(1)));
  EXPECT_FALSE(filtered.exists(iap::bspline_clock_key(2)));
}

TEST(BSplineFixedLagRegistryTest, ResetAndAppendReturnOrderedLatestSegment) {
  iap::BSplineControlWindow window;
  window.initialize(1.0, 1.1, gtsam::Pose3());

  iap::BSplineFixedLagStateRegistry registry;
  registry.reset_from_window(window);

  auto first = make_segment_state(window);
  auto& first_ref = registry.append_segment(first);
  EXPECT_EQ(first_ref.auxiliary_index, 1U);

  window.advance(1.1, 1.2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
  registry.append_window(window);
  auto second = make_segment_state(window);
  auto& second_ref = registry.append_segment(std::move(second));
  EXPECT_EQ(second_ref.auxiliary_index, 2U);

  ASSERT_EQ(registry.segments().size(), 2U);
  EXPECT_LT(registry.segments()[0].stamp, registry.segments()[1].stamp);

  registry.reset_from_window(window);
  EXPECT_TRUE(registry.segments().empty());
  EXPECT_EQ(registry.control_buffer().size(), 4U);
}
