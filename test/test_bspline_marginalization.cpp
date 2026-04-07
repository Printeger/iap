// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Unit tests for fixed-lag B-spline survivor/removable partitioning and
// replayable carried-prior construction.

#include <gtest/gtest.h>

#include <algorithm>

#include <iap/odometry/bspline_fixed_lag_registry.hpp>
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
  values.insert(iap::bspline_gyro_bias_key(1), gtsam::Vector3(0.1, 0.0, 0.0));
  values.insert(iap::bspline_accel_bias_key(1), gtsam::Vector3(1.0, 0.0, 0.0));
  values.insert(iap::bspline_gyro_bias_key(4), gtsam::Vector3(0.4, 0.0, 0.0));
  values.insert(iap::bspline_accel_bias_key(4), gtsam::Vector3(4.0, 0.0, 0.0));
  values.insert(iap::bspline_gyro_bias_key(6), gtsam::Vector3(0.6, 0.0, 0.0));
  values.insert(iap::bspline_accel_bias_key(6), gtsam::Vector3(6.0, 0.0, 0.0));
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
  std::vector<iap::BSplineMarginalizationSegmentState> states(3);
  states[0].scan_end = 2.5;
  states[0].active_control_indices = {0, 1, 2, 3};
  states[0].control_indices = {0, 1, 2, 3};
  states[0].auxiliary_index = 1;

  states[1].scan_end = 4.5;
  states[1].active_control_indices = {3, 4, 5, 6};
  states[1].control_indices = {3, 4, 5, 6};
  states[1].auxiliary_index = 4;

  states[2].scan_end = 6.5;
  states[2].active_control_indices = {4, 5, 6, 7};
  states[2].control_indices = {4, 5, 6, 7};
  states[2].auxiliary_index = 6;
  return states;
}

std::vector<iap::BSplineMarginalizationSegmentState> make_multispan_segment_states() {
  std::vector<iap::BSplineMarginalizationSegmentState> states(3);
  states[0].scan_end = 2.5;
  states[0].span_begin_idx = 0;
  states[0].span_end_idx = 1;
  states[0].active_control_indices = {0, 1, 2, 3};
  states[0].control_indices = {0, 1, 2, 3};
  states[0].auxiliary_index = 1;

  states[1].scan_end = 4.6;
  states[1].span_begin_idx = 2;
  states[1].span_end_idx = 3;
  states[1].active_control_indices = {2, 3, 4, 5};
  states[1].control_indices = {2, 3, 4, 5};
  states[1].auxiliary_index = 4;

  states[2].scan_end = 5.8;
  states[2].span_begin_idx = 4;
  states[2].span_end_idx = 6;
  states[2].active_control_indices = {2, 3, 4, 5, 6, 7};
  states[2].control_indices = {4, 5, 6, 7};
  states[2].auxiliary_index = 6;
  return states;
}

std::vector<gtsam::Pose3> make_window_poses() {
  std::vector<gtsam::Pose3> poses;
  poses.reserve(8);
  for (std::size_t i = 0; i < 8; ++i) {
    poses.push_back(translated_pose(static_cast<double>(i)));
  }
  return poses;
}

std::vector<double> make_window_knots() {
  return {0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 4.0, 6.0, 6.0, 6.0, 6.0};
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

  const auto foreign = partition.classify_factor(
    gtsam::KeyVector{iap::bspline_control_point_key(6), gtsam::symbol('z', 0)});
  EXPECT_EQ(foreign.ownership, iap::BSplineMarginalizationPartition::FactorOwnership::Foreign);
  EXPECT_EQ(foreign.foreign_keys, (std::vector<gtsam::Key>{gtsam::symbol('z', 0)}));
  EXPECT_FALSE(partition.can_replay_keys({iap::bspline_control_point_key(6), gtsam::symbol('z', 0)}, values));
  EXPECT_TRUE(partition.can_replay_keys(
    {iap::bspline_control_point_key(6), iap::bspline_velocity_key(6), iap::bspline_clock_key(6)},
    values));
}

TEST(BSplineMarginalizationTest, ActiveStateSetKeepsMultiSpanReferencedControls) {
  const gtsam::Values values = make_partition_values();
  auto active_state_set = iap::build_spline_active_state_set(
    make_buffer_states(),
    make_multispan_segment_states(),
    values,
    4.4,
    true);

  std::sort(active_state_set.active_span_indices.begin(), active_state_set.active_span_indices.end());
  std::sort(active_state_set.active_control_indices.begin(), active_state_set.active_control_indices.end());
  std::sort(active_state_set.removable_control_indices.begin(), active_state_set.removable_control_indices.end());
  std::sort(active_state_set.active_auxiliary_indices.begin(), active_state_set.active_auxiliary_indices.end());
  std::sort(active_state_set.removable_auxiliary_indices.begin(), active_state_set.removable_auxiliary_indices.end());

  EXPECT_EQ(active_state_set.active_span_indices, (std::vector<int>{2, 3, 4, 5, 6}));
  EXPECT_EQ(active_state_set.active_control_indices, (std::vector<std::size_t>{2, 3, 4, 5, 6, 7}));
  EXPECT_EQ(active_state_set.removable_control_indices, (std::vector<std::size_t>{0, 1}));
  EXPECT_EQ(active_state_set.active_auxiliary_indices, (std::vector<std::size_t>{4, 6}));
  EXPECT_EQ(active_state_set.removable_auxiliary_indices, (std::vector<std::size_t>{1}));

  EXPECT_TRUE(active_state_set.contains_active_key(iap::bspline_control_point_key(2)));
  EXPECT_TRUE(active_state_set.contains_active_key(iap::bspline_velocity_key(4)));
  EXPECT_TRUE(active_state_set.contains_active_key(iap::bspline_clock_key(6)));
  EXPECT_TRUE(active_state_set.contains_removed_key(iap::bspline_control_point_key(1)));
  EXPECT_TRUE(active_state_set.contains_removed_key(iap::bspline_velocity_key(1)));
}

TEST(BSplineMarginalizationTest, ActiveStateSetCanTrackLaggedBiasInsteadOfSharedSingleton) {
  const gtsam::Values values = make_partition_values();
  auto active_state_set = iap::build_spline_active_state_set(
    make_buffer_states(),
    make_segment_states(),
    values,
    5.1,
    true,
    true,
    false,
    true,
    true);

  EXPECT_TRUE(active_state_set.contains_active_key(iap::bspline_gyro_bias_key(6)));
  EXPECT_TRUE(active_state_set.contains_active_key(iap::bspline_accel_bias_key(6)));
  EXPECT_TRUE(active_state_set.contains_removed_key(iap::bspline_gyro_bias_key(4)));
  EXPECT_TRUE(active_state_set.contains_removed_key(iap::bspline_accel_bias_key(4)));
  EXPECT_FALSE(active_state_set.contains_active_key(gtsam::symbol('j', 0)));
  EXPECT_FALSE(active_state_set.contains_active_key(gtsam::symbol('k', 0)));

  const auto partition = iap::build_bspline_marginalization_partition(active_state_set);
  EXPECT_TRUE(partition.contains_survivor(iap::bspline_gyro_bias_key(6)));
  EXPECT_TRUE(partition.contains_removed(iap::bspline_accel_bias_key(4)));
  EXPECT_FALSE(partition.contains_survivor(gtsam::symbol('j', 0)));
}

TEST(BSplineMarginalizationTest, SurvivorAnchorFilterCanExcludeLaggedBiasKeys) {
  const gtsam::Values values = make_partition_values();
  const auto active_state_set = iap::build_spline_active_state_set(
    make_buffer_states(),
    make_segment_states(),
    values,
    5.1,
    true,
    true,
    false,
    true,
    true);

  const auto anchorable = iap::filter_bspline_survivor_anchor_keys(active_state_set.active_keys(), false);
  EXPECT_FALSE(std::find(
    anchorable.begin(),
    anchorable.end(),
    iap::bspline_gyro_bias_key(6)) != anchorable.end());
  EXPECT_FALSE(std::find(
    anchorable.begin(),
    anchorable.end(),
    iap::bspline_accel_bias_key(6)) != anchorable.end());
  EXPECT_TRUE(std::find(
    anchorable.begin(),
    anchorable.end(),
    iap::bspline_control_point_key(6)) != anchorable.end());
  EXPECT_TRUE(std::find(
    anchorable.begin(),
    anchorable.end(),
    iap::bspline_velocity_key(6)) != anchorable.end());
}

TEST(BSplineMarginalizationTest, RegistryPruneFollowsActiveStateSetReferences) {
  iap::BSplineControlWindow window;
  window.seed_with_knots(make_window_knots(), make_window_poses());

  iap::BSplineFixedLagStateRegistry registry;
  registry.reset_from_window(window);
  for (const auto& segment : make_multispan_segment_states()) {
    iap::BSplineFixedLagSegmentState registry_segment;
    registry_segment.scan_end = segment.scan_end;
    registry_segment.span_begin_idx = segment.span_begin_idx;
    registry_segment.span_end_idx = segment.span_end_idx;
    registry_segment.active_control_indices = segment.active_control_indices;
    registry_segment.control_indices = segment.control_indices;
    registry_segment.auxiliary_index = segment.auxiliary_index;
    registry.append_segment(registry_segment);
  }

  registry.auxiliary_values().insert(iap::bspline_velocity_key(1), gtsam::Vector3(1.0, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_velocity_key(4), gtsam::Vector3(4.0, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_velocity_key(6), gtsam::Vector3(6.0, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_clock_key(4), gtsam::Vector2(10.0, 0.1));
  registry.auxiliary_values().insert(iap::bspline_clock_key(6), gtsam::Vector2(20.0, 0.2));

  const auto spans = registry.active_span_indices(4.4);
  ASSERT_EQ(spans, (std::vector<int>{2, 3, 4, 5, 6}));

  auto references = registry.active_control_references(4.4);
  std::sort(references.begin(), references.end(), [](const auto& lhs, const auto& rhs) {
    return lhs.control_index < rhs.control_index;
  });
  ASSERT_EQ(references.size(), 6U);
  EXPECT_EQ(references.front().control_index, 2U);
  EXPECT_EQ(references.front().reference_count, 2U);
  EXPECT_EQ(references.back().control_index, 7U);
  EXPECT_EQ(references.back().reference_count, 1U);

  const auto active_state_set = registry.active_state_set(make_partition_values(), 4.4, true);
  registry.prune_to_active_state_set(4.4, active_state_set, true);

  std::vector<std::size_t> retained_indices;
  for (const auto& state : registry.control_buffer().states()) {
    retained_indices.push_back(state.index);
  }
  EXPECT_EQ(retained_indices, (std::vector<std::size_t>{2, 3, 4, 5, 6, 7}));

  EXPECT_FALSE(registry.auxiliary_values().exists(iap::bspline_velocity_key(1)));
  EXPECT_TRUE(registry.auxiliary_values().exists(iap::bspline_velocity_key(4)));
  EXPECT_TRUE(registry.auxiliary_values().exists(iap::bspline_velocity_key(6)));
  EXPECT_TRUE(registry.auxiliary_values().exists(iap::bspline_clock_key(4)));
  EXPECT_TRUE(registry.auxiliary_values().exists(iap::bspline_clock_key(6)));
}

TEST(BSplineMarginalizationTest, RegistryPruneDropsRetiredLaggedBiasMirrors) {
  iap::BSplineControlWindow window;
  window.seed_with_knots(make_window_knots(), make_window_poses());

  iap::BSplineFixedLagStateRegistry registry;
  registry.set_bias_shared_singleton(false);
  registry.reset_from_window(window);
  for (const auto& segment : make_segment_states()) {
    iap::BSplineFixedLagSegmentState registry_segment;
    registry_segment.scan_end = segment.scan_end;
    registry_segment.span_begin_idx = segment.span_begin_idx;
    registry_segment.span_end_idx = segment.span_end_idx;
    registry_segment.active_control_indices = segment.active_control_indices;
    registry_segment.control_indices = segment.control_indices;
    registry_segment.auxiliary_index = segment.auxiliary_index;
    registry.append_segment(registry_segment);
  }

  registry.auxiliary_values().insert(iap::bspline_gyro_bias_key(1), gtsam::Vector3(0.1, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_accel_bias_key(1), gtsam::Vector3(1.0, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_gyro_bias_key(4), gtsam::Vector3(0.4, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_accel_bias_key(4), gtsam::Vector3(4.0, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_gyro_bias_key(6), gtsam::Vector3(0.6, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_accel_bias_key(6), gtsam::Vector3(6.0, 0.0, 0.0));

  const auto active_state_set = registry.active_state_set(make_partition_values(), 5.1, true);
  registry.prune_to_active_state_set(5.1, active_state_set, true);

  EXPECT_FALSE(registry.auxiliary_values().exists(iap::bspline_gyro_bias_key(1)));
  EXPECT_FALSE(registry.auxiliary_values().exists(iap::bspline_accel_bias_key(1)));
  EXPECT_FALSE(registry.auxiliary_values().exists(iap::bspline_gyro_bias_key(4)));
  EXPECT_FALSE(registry.auxiliary_values().exists(iap::bspline_accel_bias_key(4)));
  EXPECT_TRUE(registry.auxiliary_values().exists(iap::bspline_gyro_bias_key(6)));
  EXPECT_TRUE(registry.auxiliary_values().exists(iap::bspline_accel_bias_key(6)));
}

TEST(BSplineMarginalizationTest, ActiveStateSetKeepsControlsSharedAcrossBucketStyleReferences) {
  auto segment_states = make_multispan_segment_states();
  iap::BSplineMarginalizationSegmentState bucket_state_a;
  bucket_state_a.scan_end = 5.1;
  bucket_state_a.span_begin_idx = 3;
  bucket_state_a.span_end_idx = 4;
  bucket_state_a.active_control_indices = {3, 4, 5, 6};
  bucket_state_a.control_indices = {3, 4, 5, 6};
  bucket_state_a.auxiliary_index = 5;
  segment_states.push_back(bucket_state_a);

  iap::BSplineMarginalizationSegmentState bucket_state_b;
  bucket_state_b.scan_end = 5.4;
  bucket_state_b.span_begin_idx = 4;
  bucket_state_b.span_end_idx = 5;
  bucket_state_b.active_control_indices = {4, 5, 6, 7};
  bucket_state_b.control_indices = {4, 5, 6, 7};
  bucket_state_b.auxiliary_index = 6;
  segment_states.push_back(bucket_state_b);

  auto active_state_set = iap::build_spline_active_state_set(
    make_buffer_states(),
    segment_states,
    make_partition_values(),
    4.9,
    true);

  std::sort(active_state_set.active_control_indices.begin(), active_state_set.active_control_indices.end());
  std::sort(active_state_set.removable_control_indices.begin(), active_state_set.removable_control_indices.end());
  std::sort(active_state_set.active_auxiliary_indices.begin(), active_state_set.active_auxiliary_indices.end());
  std::sort(active_state_set.removable_auxiliary_indices.begin(), active_state_set.removable_auxiliary_indices.end());

  EXPECT_EQ(active_state_set.active_control_indices, (std::vector<std::size_t>{2, 3, 4, 5, 6, 7}));
  EXPECT_EQ(active_state_set.removable_control_indices, (std::vector<std::size_t>{0, 1}));
  EXPECT_EQ(active_state_set.active_auxiliary_indices, (std::vector<std::size_t>{5, 6}));
  EXPECT_EQ(active_state_set.removable_auxiliary_indices, (std::vector<std::size_t>{1, 4}));
  EXPECT_TRUE(active_state_set.contains_active_key(iap::bspline_control_point_key(2)));
  EXPECT_TRUE(active_state_set.contains_active_key(iap::bspline_control_point_key(6)));
  EXPECT_FALSE(active_state_set.contains_removed_key(iap::bspline_control_point_key(3)));

  const auto partition = iap::build_bspline_marginalization_partition(
    make_buffer_states(),
    segment_states,
    make_partition_values(),
    4.9,
    true);
  EXPECT_FALSE(partition.should_marginalize_factor(
    gtsam::KeyVector{iap::bspline_control_point_key(3), iap::bspline_control_point_key(4), iap::bspline_velocity_key(5)}));
  EXPECT_FALSE(partition.should_marginalize_factor(
    gtsam::KeyVector{iap::bspline_control_point_key(2), iap::bspline_control_point_key(3)}));
  EXPECT_TRUE(partition.should_marginalize_factor(
    gtsam::KeyVector{iap::bspline_control_point_key(1), iap::bspline_control_point_key(2)}));
}

TEST(BSplineMarginalizationTest, RegistryPruneRetainsControlsSharedAcrossBucketStyleReferences) {
  iap::BSplineControlWindow window;
  window.seed_with_knots(make_window_knots(), make_window_poses());

  iap::BSplineFixedLagStateRegistry registry;
  registry.reset_from_window(window);

  for (const auto& segment : make_multispan_segment_states()) {
    iap::BSplineFixedLagSegmentState registry_segment;
    registry_segment.scan_end = segment.scan_end;
    registry_segment.span_begin_idx = segment.span_begin_idx;
    registry_segment.span_end_idx = segment.span_end_idx;
    registry_segment.active_control_indices = segment.active_control_indices;
    registry_segment.control_indices = segment.control_indices;
    registry_segment.auxiliary_index = segment.auxiliary_index;
    registry.append_segment(registry_segment);
  }

  iap::BSplineFixedLagSegmentState bucket_segment_a;
  bucket_segment_a.scan_end = 5.1;
  bucket_segment_a.span_begin_idx = 3;
  bucket_segment_a.span_end_idx = 4;
  bucket_segment_a.active_control_indices = {3, 4, 5, 6};
  bucket_segment_a.control_indices = {3, 4, 5, 6};
  bucket_segment_a.auxiliary_index = 5;
  registry.append_segment(bucket_segment_a);

  iap::BSplineFixedLagSegmentState bucket_segment_b;
  bucket_segment_b.scan_end = 5.4;
  bucket_segment_b.span_begin_idx = 4;
  bucket_segment_b.span_end_idx = 5;
  bucket_segment_b.active_control_indices = {4, 5, 6, 7};
  bucket_segment_b.control_indices = {4, 5, 6, 7};
  bucket_segment_b.auxiliary_index = 6;
  registry.append_segment(bucket_segment_b);

  registry.auxiliary_values().insert(iap::bspline_velocity_key(1), gtsam::Vector3(1.0, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_velocity_key(4), gtsam::Vector3(4.0, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_velocity_key(5), gtsam::Vector3(5.0, 0.0, 0.0));
  registry.auxiliary_values().insert(iap::bspline_velocity_key(6), gtsam::Vector3(6.0, 0.0, 0.0));

  const auto active_state_set = registry.active_state_set(make_partition_values(), 4.9, true);
  registry.prune_to_active_state_set(4.9, active_state_set, true);

  std::vector<std::size_t> retained_indices;
  for (const auto& state : registry.control_buffer().states()) {
    retained_indices.push_back(state.index);
  }
  EXPECT_EQ(retained_indices, (std::vector<std::size_t>{2, 3, 4, 5, 6, 7}));
  EXPECT_FALSE(registry.auxiliary_values().exists(iap::bspline_velocity_key(4)));
  EXPECT_TRUE(registry.auxiliary_values().exists(iap::bspline_velocity_key(5)));
  EXPECT_TRUE(registry.auxiliary_values().exists(iap::bspline_velocity_key(6)));
  EXPECT_FALSE(registry.auxiliary_values().exists(iap::bspline_velocity_key(1)));
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

  const auto carried_prior = iap::build_bspline_carried_prior(graph, values, {s1, s2});
  ASSERT_FALSE(carried_prior.empty());
  EXPECT_EQ(carried_prior.retained_keys, (std::vector<gtsam::Key>{s1, s2}));

  const auto replayed = carried_prior.replay();

  for (const auto& factor : replayed) {
    ASSERT_TRUE(static_cast<bool>(factor));
    for (const auto key : factor->keys()) {
      EXPECT_TRUE(key == s1 || key == s2);
    }
  }

  gtsam::Values perturbed = values;
  perturbed.update(s1, translated_pose(1.05));
  perturbed.update(s2, translated_pose(2.08));

  const auto linear_graph = graph.linearize(values);
  const auto marginal_graph = linear_graph->marginal(carried_prior.retained_keys);

  gtsam::Values linearization_point;
  linearization_point.insert(s1, values.at<gtsam::Pose3>(s1));
  linearization_point.insert(s2, values.at<gtsam::Pose3>(s2));

  gtsam::Values perturbed_subset;
  perturbed_subset.insert(s1, perturbed.at<gtsam::Pose3>(s1));
  perturbed_subset.insert(s2, perturbed.at<gtsam::Pose3>(s2));

  const auto delta = linearization_point.localCoordinates(perturbed_subset);
  const double expected_error = marginal_graph->error(delta);
  const double actual_error = replayed.error(perturbed);

  EXPECT_GT(actual_error, 0.0);
  EXPECT_NEAR(actual_error, expected_error, 1e-8);
}

TEST(BSplineMarginalizationTest, CarriedPriorCanComposeWithPreviousLinearPrior) {
  const gtsam::Key s0 = iap::bspline_control_point_key(0);
  const gtsam::Key s1 = iap::bspline_control_point_key(1);
  const gtsam::Key s2 = iap::bspline_control_point_key(2);
  const gtsam::Key s3 = iap::bspline_control_point_key(3);

  gtsam::Values values;
  values.insert(s0, translated_pose(0.0));
  values.insert(s1, translated_pose(1.0));
  values.insert(s2, translated_pose(2.0));
  values.insert(s3, translated_pose(3.0));

  const auto noise = gtsam::noiseModel::Isotropic::Precision(6, 100.0);
  gtsam::NonlinearFactorGraph base_graph;
  base_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(s0, translated_pose(0.0), noise);
  base_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(s0, s1, translated_pose(0.0).between(translated_pose(1.0)), noise);
  base_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(s1, s2, translated_pose(1.0).between(translated_pose(2.0)), noise);

  const auto prior12 = iap::build_bspline_carried_prior(base_graph, values, {s1, s2});
  ASSERT_FALSE(prior12.empty());

  gtsam::NonlinearFactorGraph next_graph;
  next_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(s2, s3, translated_pose(2.0).between(translated_pose(3.0)), noise);

  const auto prior23 = iap::build_bspline_carried_prior(next_graph, values, {s2, s3}, &prior12);
  ASSERT_FALSE(prior23.empty());
  EXPECT_EQ(prior23.retained_keys, (std::vector<gtsam::Key>{s2, s3}));

  gtsam::NonlinearFactorGraph reference_graph = prior12.replay();
  for (const auto& factor : next_graph) {
    reference_graph.add(factor);
  }

  const auto replayed23 = prior23.replay();
  gtsam::Values perturbed = values;
  perturbed.update(s2, translated_pose(2.03));
  perturbed.update(s3, translated_pose(3.06));

  const auto linear_reference = reference_graph.linearize(values);
  const auto marginal_reference = linear_reference->marginal(std::vector<gtsam::Key>{s2, s3});

  gtsam::Values linearization_point;
  linearization_point.insert(s2, values.at<gtsam::Pose3>(s2));
  linearization_point.insert(s3, values.at<gtsam::Pose3>(s3));

  gtsam::Values perturbed_subset;
  perturbed_subset.insert(s2, perturbed.at<gtsam::Pose3>(s2));
  perturbed_subset.insert(s3, perturbed.at<gtsam::Pose3>(s3));

  const auto delta = linearization_point.localCoordinates(perturbed_subset);
  EXPECT_NEAR(replayed23.error(perturbed), marginal_reference->error(delta), 1e-8);
}

TEST(BSplineMarginalizationTest, CarriedPriorIgnoresPreviousPriorWhenRetainedKeysAreMissing) {
  const gtsam::Key s0 = iap::bspline_control_point_key(0);
  const gtsam::Key s1 = iap::bspline_control_point_key(1);
  const gtsam::Key e0 = iap::bspline_ecef_origin_key();

  gtsam::Values seeded_values;
  seeded_values.insert(s0, translated_pose(0.0));
  seeded_values.insert(s1, translated_pose(1.0));
  seeded_values.insert(e0, gtsam::Vector3(1.0, 2.0, 3.0));

  const auto pose_noise = gtsam::noiseModel::Isotropic::Precision(6, 100.0);
  const auto ecef_noise = gtsam::noiseModel::Isotropic::Precision(3, 10.0);

  gtsam::NonlinearFactorGraph base_graph;
  base_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(s0, translated_pose(0.0), pose_noise);
  base_graph.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
    s0,
    s1,
    translated_pose(0.0).between(translated_pose(1.0)),
    pose_noise);
  base_graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(e0, gtsam::Vector3(1.0, 2.0, 3.0), ecef_noise);

  const auto previous_prior = iap::build_bspline_carried_prior(base_graph, seeded_values, {s1, e0});
  ASSERT_FALSE(previous_prior.empty());

  gtsam::Values current_values;
  current_values.insert(s1, translated_pose(1.0));

  gtsam::NonlinearFactorGraph next_graph;
  next_graph.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(s1, translated_pose(1.0), pose_noise);

  EXPECT_NO_THROW({
    const auto carried = iap::build_bspline_carried_prior(next_graph, current_values, {s1}, &previous_prior);
    EXPECT_FALSE(carried.empty());
    EXPECT_EQ(carried.retained_keys, (std::vector<gtsam::Key>{s1}));
  });
}
