#include <gtest/gtest.h>

#include <iap/odometry/ct_solve_domain.hpp>

TEST(CTSolveDomain, SelectsCurrentAndRecentOverlapSegments) {
  std::vector<iap::BSplineFixedLagSegmentState> segments(5);
  for (std::size_t i = 0; i < segments.size(); ++i) {
    segments[i].stamp = static_cast<double>(i);
    segments[i].scan_end = static_cast<double>(i) + 0.1;
    segments[i].auxiliary_index = 10 + i;
    segments[i].control_indices = {i, i + 1, i + 2, i + 3};
  }

  const auto domain = iap::BSplineSolveDomain::from_segments(segments, 2);
  ASSERT_FALSE(domain.empty());
  EXPECT_DOUBLE_EQ(domain.start_time(), 2.0);
  EXPECT_DOUBLE_EQ(domain.end_time(), 4.1);
  EXPECT_EQ(domain.active_segment_ordinals(), (std::vector<std::size_t>{2, 3, 4}));
  EXPECT_EQ(domain.retired_segment_ordinals(), (std::vector<std::size_t>{0, 1}));
  EXPECT_TRUE(domain.supports_time(3.5));
  EXPECT_FALSE(domain.supports_time(1.5));
}

TEST(CTSolveDomain, ComputesRetiredControlKeysWithoutActiveDuplicates) {
  std::vector<iap::BSplineFixedLagSegmentState> segments(3);
  segments[0].control_indices = {0, 1, 2, 3};
  segments[1].control_indices = {1, 2, 3, 4};
  segments[2].control_indices = {2, 3, 4, 5};
  segments[0].stamp = 0.0;
  segments[1].stamp = 1.0;
  segments[2].stamp = 2.0;
  segments[0].scan_end = 0.1;
  segments[1].scan_end = 1.1;
  segments[2].scan_end = 2.1;

  const auto domain = iap::BSplineSolveDomain::from_segments(segments, 0);
  const auto retired = domain.retired_control_keys();

  ASSERT_EQ(retired.size(), 2u);
  EXPECT_EQ(retired[0], iap::bspline_control_point_key(0));
  EXPECT_EQ(retired[1], iap::bspline_control_point_key(1));
}

