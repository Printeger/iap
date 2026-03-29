// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Unit tests for fixed-lag B-spline survivor/removable partitioning and
// replayable carried-prior construction.

#include <gtest/gtest.h>

#include <iap/odometry/bspline_marginalization.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace {

gtsam::Pose3 translated_pose(double x) {
  return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, 0.0, 0.0));
}

gtsam::Values make_partition_values() {
  gtsam::Values values;
  for (std::size_t i = 0; i < 8; ++i) {
    values.insert(iap::bspline_control_point_key(i), translated_pose(static_cast<double>(i)));
  }

  values.insert(iap::bspline_velocity_key(1), gtsam::Vector3(1.0, 0.0, 0.0));
  values.insert(iap::bspline_velocity_key(4), gtsam::Vector3(4.0, 0.0, 0.0));
  values.insert(iap::bspline_velocity_key(6), gtsam::Vector3(6.0, 0.0, 0.0));
  values.insert(iap::bspline_clock_key(4), gtsam::Vector2(10.0, 0.1));
  values.insert(iap::bspline_clock_key(6), gtsam::Vector2(20.0, 0.2));
  values.insert(gtsam::symbol('j', 0), gtsam::Vector3(0.0, 0.0, 0.0));
  values.insert(gtsam::symbol('k', 0), gtsam::Vector3(0.0, 0.0, 0.0));
  values.insert(gtsam::symbol('g', 0), gtsam::Vector3(0.0, 0.0, 9.81));
  values.insert(iap::bspline_ecef_origin_key(), gtsam::Vector3(0.0, 0.0, 0.0));
  values.insert(iap::bspline_ecef_rot_key(), gtsam::Rot3());
  return values;
}

std::vector<iap::BSplineControlPointState> make_buffer_states() {
  std::vector<iap::BSplineControlPointState> states;
  for (std::size_t i = 0; i < 8; ++i) {
    states.push_back(iap::BSplineControlPointState{
      i,
      static_cast<double>(i),
      translated_pose(static_cast<double>(i)),
    });
  }
  return states;
}

std::vector<iap::BSplineMarginalizationSegmentState> make_segment_states() {
  return {
    iap::BSplineMarginalizationSegmentState{
      2.5,
      {0, 1, 2, 3},
      1,
    },
    iap::BSplineMarginalizationSegmentState{
      4.5,
      {3, 4, 5, 6},
      4,
    },
    iap::BSplineMarginalizationSegmentState{
      6.5,
      {4, 5, 6, 7},
      6,
    },
  };
}

}  // namespace

TEST(BSplineMarginalizationTest, PartitionKeepsSupportPointsAndPersistentStates) {
  const gtsam::Values values = make_partition_values();
  const auto partition = iap::build_bspline_marginalization_partition(
    make_buffer_states(),
    make_segment_states(),
    values,
    5.1,
    true);

  EXPECT_EQ(partition.survivor_control_indices, (std::vector<std::size_t>{3, 4, 5, 6, 7}));
  EXPECT_EQ(partition.removable_control_indices, (std::vector<std::size_t>{0, 1, 2}));
  EXPECT_EQ(partition.survivor_auxiliary_indices, (std::vector<std::size_t>{6}));
  EXPECT_EQ(partition.removable_auxiliary_indices, (std::vector<std::size_t>{1, 4}));

  EXPECT_TRUE(partition.contains_survivor(iap::bspline_control_point_key(3)));
  EXPECT_TRUE(partition.contains_survivor(iap::bspline_velocity_key(6)));
  EXPECT_TRUE(partition.contains_survivor(iap::bspline_clock_key(6)));
  EXPECT_TRUE(partition.contains_survivor(gtsam::symbol('j', 0)));
  EXPECT_TRUE(partition.contains_removed(iap::bspline_control_point_key(2)));
  EXPECT_TRUE(partition.contains_removed(iap::bspline_velocity_key(4)));
  EXPECT_TRUE(partition.contains_removed(iap::bspline_clock_key(4)));

  EXPECT_TRUE(partition.should_marginalize_factor(
    gtsam::KeyVector{iap::bspline_control_point_key(2), iap::bspline_control_point_key(3)}));
  EXPECT_FALSE(partition.should_marginalize_factor(
    gtsam::KeyVector{iap::bspline_control_point_key(5), iap::bspline_control_point_key(6)}));
  EXPECT_TRUE(partition.should_marginalize_factor(
    gtsam::KeyVector{iap::bspline_clock_key(4), iap::bspline_ecef_rot_key()}));
}

TEST(BSplineMarginalizationTest, CarriedPriorMatchesReferenceMarginalError) {
  const gtsam::Key s0 = iap::bspline_control_point_key(0);
  const gtsam::Key s1 = iap::bspline_control_point_key(1);
  const gtsam::Key s2 = iap::bspline_control_point_key(2);

  gtsam::Values values;
  values.insert(s0, translated_pose(0.0));
  values.insert(s1, translated_pose(1.0));
  values.insert(s2, translated_pose(2.0));

  const auto noise = gtsam::noiseModel::Isotropic::Precision(6, 100.0);
  gtsam::NonlinearFactorGraph graph;
  graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(s0, translated_pose(0.0), noise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(s0, s1, translated_pose(0.0).between(translated_pose(1.0)), noise);
  graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(s1, s2, translated_pose(1.0).between(translated_pose(2.0)), noise);

  std::vector<gtsam::Key> retained_keys;
  const auto carried_prior = iap::build_bspline_carried_prior(graph, values, {s1, s2}, &retained_keys);
  ASSERT_FALSE(carried_prior.empty());
  EXPECT_EQ(retained_keys, (std::vector<gtsam::Key>{s1, s2}));

  for (const auto& factor : carried_prior) {
    ASSERT_TRUE(static_cast<bool>(factor));
    for (const auto key : factor->keys()) {
      EXPECT_TRUE(key == s1 || key == s2);
    }
  }

  gtsam::Values perturbed = values;
  perturbed.update(s1, translated_pose(1.05));
  perturbed.update(s2, translated_pose(2.08));

  const auto linear_graph = graph.linearize(values);
  const auto marginal_graph = linear_graph->marginal(retained_keys);

  gtsam::Values linearization_point;
  linearization_point.insert(s1, values.at<gtsam::Pose3>(s1));
  linearization_point.insert(s2, values.at<gtsam::Pose3>(s2));

  gtsam::Values perturbed_subset;
  perturbed_subset.insert(s1, perturbed.at<gtsam::Pose3>(s1));
  perturbed_subset.insert(s2, perturbed.at<gtsam::Pose3>(s2));

  const auto delta = linearization_point.localCoordinates(perturbed_subset);
  const double expected_error = marginal_graph->error(delta);
  const double actual_error = carried_prior.error(perturbed);

  EXPECT_GT(actual_error, 0.0);
  EXPECT_NEAR(actual_error, expected_error, 1e-8);
}
