#include <gtest/gtest.h>

#include <iap/odometry/ct_incremental_solver_skeleton.hpp>

namespace {

gtsam::Pose3 translated_pose(double x) {
  return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, 0.0, 0.0));
}

}  // namespace

TEST(CTIncrementalSolverSkeleton, TracksNewAndRetiredKeysAcrossSolveDomains) {
  std::vector<iap::BSplineControlPointState> control_states = {
    {0, 0.0, translated_pose(0.0)},
    {1, 1.0, translated_pose(1.0)},
    {2, 2.0, translated_pose(2.0)},
    {3, 3.0, translated_pose(3.0)},
    {4, 4.0, translated_pose(4.0)},
  };

  std::vector<iap::BSplineFixedLagSegmentState> segments_first = {
    {1.0, 2.0, {0, 1, 2, 3}, 1},
  };
  std::vector<iap::BSplineFixedLagSegmentState> segments_second = {
    {1.0, 2.0, {0, 1, 2, 3}, 1},
    {2.0, 3.0, {1, 2, 3, 4}, 2},
  };

  gtsam::Values values;
  for (const auto& state : control_states) {
    values.insert(iap::bspline_control_point_key(state.index), state.pose);
  }
  values.insert(iap::bspline_velocity_key(1), gtsam::Vector3(1.0, 0.0, 0.0));
  values.insert(iap::bspline_velocity_key(2), gtsam::Vector3(2.0, 0.0, 0.0));
  values.insert(iap::bspline_clock_key(1), gtsam::Vector2(10.0, 0.5));
  values.insert(iap::bspline_clock_key(2), gtsam::Vector2(11.0, 0.5));
  values.insert(gtsam::symbol('j', 0), gtsam::Vector3(0.1, 0.2, 0.3));
  values.insert(gtsam::symbol('k', 0), gtsam::Vector3(0.4, 0.5, 0.6));
  values.insert(gtsam::symbol('g', 0), gtsam::Vector3(0.0, 0.0, 9.81));
  values.insert(iap::bspline_ecef_origin_key(), gtsam::Vector3(10.0, 20.0, 30.0));
  values.insert(iap::bspline_ecef_rot_key(), gtsam::Rot3::Identity());

  iap::BSplineFixedLagSharedState shared_state;
  shared_state.gyro_bias = gtsam::Vector3(0.1, 0.2, 0.3);
  shared_state.accel_bias = gtsam::Vector3(0.4, 0.5, 0.6);
  shared_state.gravity = gtsam::Vector3(0.0, 0.0, 9.81);
  shared_state.gnss_anchor_initialized = true;
  shared_state.ecef_origin = gtsam::Vector3(10.0, 20.0, 30.0);
  shared_state.ecef_rot = gtsam::Rot3::Identity();

  iap::BSplineIncrementalSolverSkeleton skeleton;
  const auto first_domain = iap::BSplineSolveDomain::from_segments(segments_first, 0);
  const auto first_delta = skeleton.prepare_update(first_domain, control_states, shared_state, values, 2.0);

  EXPECT_EQ(first_delta.active_segment_ordinals.size(), 1U);
  EXPECT_EQ(first_delta.newly_active_segment_ordinals.size(), 1U);
  ASSERT_EQ(first_delta.active_segment_ids.size(), 1U);
  ASSERT_EQ(first_delta.newly_active_segment_ids.size(), 1U);
  EXPECT_EQ(first_delta.active_segment_ids.front(), 1U);
  EXPECT_EQ(first_delta.newly_active_segment_ids.front(), 1U);
  EXPECT_EQ(first_delta.new_keys.size(), first_delta.active_keys.size());
  EXPECT_TRUE(first_delta.new_values.exists(iap::bspline_control_point_key(0)));
  EXPECT_TRUE(first_delta.new_values.exists(iap::bspline_velocity_key(1)));
  EXPECT_TRUE(first_delta.new_values.exists(iap::bspline_clock_key(1)));
  EXPECT_TRUE(first_delta.new_values.exists(iap::bspline_ecef_origin_key()));
  EXPECT_DOUBLE_EQ(first_delta.new_stamps.at(iap::bspline_ecef_origin_key()), 2.0);
  EXPECT_DOUBLE_EQ(first_delta.new_stamps.at(iap::bspline_control_point_key(3)), 3.0);
  EXPECT_DOUBLE_EQ(first_delta.new_stamps.at(iap::bspline_velocity_key(1)), 1.0);

  const auto second_domain = iap::BSplineSolveDomain::from_segments(segments_second, 0);
  const auto second_delta = skeleton.prepare_update(second_domain, control_states, shared_state, values, 3.0);

  EXPECT_EQ(second_delta.active_segment_ordinals.size(), 1U);
  EXPECT_EQ(second_delta.newly_active_segment_ordinals.size(), 1U);
  EXPECT_EQ(second_delta.retired_segment_ordinals.size(), 1U);
  ASSERT_EQ(second_delta.active_segment_ids.size(), 1U);
  ASSERT_EQ(second_delta.newly_active_segment_ids.size(), 1U);
  ASSERT_EQ(second_delta.retired_segment_ids.size(), 1U);
  EXPECT_EQ(second_delta.active_segment_ids.front(), 2U);
  EXPECT_EQ(second_delta.newly_active_segment_ids.front(), 2U);
  EXPECT_EQ(second_delta.retired_segment_ids.front(), 1U);
  EXPECT_TRUE(std::find(
                second_delta.new_keys.begin(),
                second_delta.new_keys.end(),
                iap::bspline_control_point_key(4)) != second_delta.new_keys.end());
  EXPECT_TRUE(std::find(
                second_delta.retired_keys.begin(),
                second_delta.retired_keys.end(),
                iap::bspline_control_point_key(0)) != second_delta.retired_keys.end());
  EXPECT_TRUE(second_delta.new_values.exists(iap::bspline_control_point_key(4)));
  EXPECT_FALSE(second_delta.new_values.exists(iap::bspline_control_point_key(1)));
  EXPECT_DOUBLE_EQ(second_delta.new_stamps.at(iap::bspline_ecef_origin_key()), 3.0);
}

TEST(CTIncrementalSolverSkeleton, OwnsPersistentSegmentAndPriorFactorIndices) {
  std::vector<iap::BSplineControlPointState> control_states = {
    {0, 0.0, translated_pose(0.0)},
    {1, 1.0, translated_pose(1.0)},
    {2, 2.0, translated_pose(2.0)},
    {3, 3.0, translated_pose(3.0)},
    {4, 4.0, translated_pose(4.0)},
  };
  std::vector<iap::BSplineFixedLagSegmentState> segments = {
    {1.0, 2.0, {0, 1, 2, 3}, 1},
    {2.0, 3.0, {1, 2, 3, 4}, 2},
  };

  gtsam::Values values;
  for (const auto& state : control_states) {
    values.insert(iap::bspline_control_point_key(state.index), state.pose);
  }
  values.insert(iap::bspline_velocity_key(2), gtsam::Vector3(2.0, 0.0, 0.0));

  iap::BSplineFixedLagSharedState shared_state;
  shared_state.gyro_bias = gtsam::Vector3::Zero();
  shared_state.accel_bias = gtsam::Vector3::Zero();
  shared_state.gravity = gtsam::Vector3(0.0, 0.0, 9.81);

  iap::BSplineIncrementalSolverSkeleton skeleton;
  const auto domain = iap::BSplineSolveDomain::from_segments(segments, 0);
  const auto delta = skeleton.prepare_update(domain, control_states, shared_state, values, 3.0);

  auto factors_to_remove = skeleton.begin_update(delta);
  EXPECT_TRUE(factors_to_remove.empty());
  EXPECT_FALSE(skeleton.segment_has_persistent_factors(2));

  skeleton.commit_update(
    gtsam::FactorIndices{10, 11, 12},
    std::vector<std::optional<std::size_t>>{std::size_t(2), std::size_t(2), std::nullopt});
  EXPECT_TRUE(skeleton.segment_has_persistent_factors(2));

  skeleton.release_segment_factors(2, &factors_to_remove);
  ASSERT_EQ(factors_to_remove.size(), 2U);
  EXPECT_EQ(factors_to_remove[0], 10U);
  EXPECT_EQ(factors_to_remove[1], 11U);
  EXPECT_FALSE(skeleton.segment_has_persistent_factors(2));
}
