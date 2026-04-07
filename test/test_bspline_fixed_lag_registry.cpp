// IAP-RQ-020 / IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the unified fixed-lag control/segment lifecycle registry.

#include <gtest/gtest.h>

#include <iap/odometry/bspline_fixed_lag_registry.hpp>

#include <gtsam/inference/Symbol.h>

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

TEST(BSplineFixedLagRegistryTest, SharedStateSeedsAndUpdatesPersistentValues) {
  iap::BSplineFixedLagStateRegistry registry;
  registry.set_shared_imu_state(
    gtsam::Vector3(0.1, 0.2, 0.3),
    gtsam::Vector3(1.0, 2.0, 3.0),
    gtsam::Vector3(0.0, 0.0, 9.7));
  registry.set_shared_gnss_anchor(
    gtsam::Vector3(10.0, 20.0, 30.0),
    gtsam::Rot3::RzRyRx(0.1, 0.2, 0.3));

  gtsam::Values values;
  registry.seed_shared_values(values, false);
  EXPECT_TRUE(values.exists(gtsam::symbol('j', 0)));
  EXPECT_TRUE(values.exists(gtsam::symbol('k', 0)));
  EXPECT_TRUE(values.exists(gtsam::symbol('g', 0)));
  EXPECT_FALSE(values.exists(iap::bspline_ecef_origin_key()));
  EXPECT_FALSE(values.exists(iap::bspline_ecef_rot_key()));

  registry.seed_shared_values(values, true);
  EXPECT_TRUE(values.exists(iap::bspline_ecef_origin_key()));
  EXPECT_TRUE(values.exists(iap::bspline_ecef_rot_key()));
  EXPECT_EQ(
    registry.active_shared_keys(true),
    (std::vector<gtsam::Key>{
      gtsam::symbol('j', 0),
      gtsam::symbol('k', 0),
      gtsam::symbol('g', 0),
      iap::bspline_ecef_origin_key(),
      iap::bspline_ecef_rot_key(),
    }));

  values.update(gtsam::symbol('j', 0), gtsam::Vector3(0.4, 0.5, 0.6));
  values.update(gtsam::symbol('k', 0), gtsam::Vector3(4.0, 5.0, 6.0));
  values.update(gtsam::symbol('g', 0), gtsam::Vector3(0.0, 0.1, 9.81));
  values.update(iap::bspline_ecef_origin_key(), gtsam::Vector3(11.0, 21.0, 31.0));
  values.update(iap::bspline_ecef_rot_key(), gtsam::Rot3::RzRyRx(0.4, 0.5, 0.6));

  registry.update_shared_state_from_values(values);
  EXPECT_TRUE(registry.shared_state().gnss_anchor_initialized);
  EXPECT_NEAR(registry.shared_state().gyro_bias.x(), 0.4, 1e-9);
  EXPECT_NEAR(registry.shared_state().accel_bias.x(), 4.0, 1e-9);
  EXPECT_NEAR(registry.shared_state().gravity.z(), 9.81, 1e-9);
  EXPECT_NEAR(registry.shared_state().ecef_origin.x(), 11.0, 1e-9);
}

TEST(BSplineFixedLagRegistryTest, ExternalReferenceGravitySkipsGraphSeedAndWriteback) {
  iap::BSplineFixedLagStateRegistry registry;
  registry.set_shared_imu_state(
    gtsam::Vector3(0.1, 0.2, 0.3),
    gtsam::Vector3(1.0, 2.0, 3.0),
    gtsam::Vector3(0.0, 0.0, 9.7));
  registry.set_gravity_shared_optimized(false);

  gtsam::Values values;
  registry.seed_shared_values(values, false);
  EXPECT_TRUE(values.exists(gtsam::symbol('j', 0)));
  EXPECT_TRUE(values.exists(gtsam::symbol('k', 0)));
  EXPECT_FALSE(values.exists(gtsam::symbol('g', 0)));
  EXPECT_EQ(
    registry.active_shared_keys(false),
    (std::vector<gtsam::Key>{gtsam::symbol('j', 0), gtsam::symbol('k', 0)}));

  values.insert(gtsam::symbol('g', 0), gtsam::Vector3(1.0, 1.0, 1.0));
  registry.update_shared_state_from_values(values);
  EXPECT_NEAR(registry.shared_state().gravity.x(), 0.0, 1e-9);
  EXPECT_NEAR(registry.shared_state().gravity.y(), 0.0, 1e-9);
  EXPECT_NEAR(registry.shared_state().gravity.z(), 9.7, 1e-9);

  const auto telemetry = registry.telemetry();
  EXPECT_EQ(telemetry.active_shared_state_count, 2U);
}

TEST(BSplineFixedLagRegistryTest, LaggedBiasModeDropsSharedBiasSingletonSemantics) {
  iap::BSplineControlWindow window;
  window.initialize(0.0, 0.1, gtsam::Pose3());

  iap::BSplineFixedLagStateRegistry registry;
  registry.set_bias_shared_singleton(false);
  registry.reset_from_window(window);
  registry.append_segment(make_segment_state(window));

  gtsam::Values seeded_values;
  registry.seed_shared_values(seeded_values, false);
  EXPECT_FALSE(seeded_values.exists(gtsam::symbol('j', 0)));
  EXPECT_FALSE(seeded_values.exists(gtsam::symbol('k', 0)));
  EXPECT_TRUE(seeded_values.exists(gtsam::symbol('g', 0)));

  gtsam::Values aux_values;
  aux_values.insert(iap::bspline_velocity_key(1), gtsam::Vector3(1.0, 0.0, 0.0));
  aux_values.insert(iap::bspline_gyro_bias_key(1), gtsam::Vector3(0.1, 0.2, 0.3));
  aux_values.insert(iap::bspline_accel_bias_key(1), gtsam::Vector3(1.0, 2.0, 3.0));
  aux_values.insert(gtsam::symbol('j', 0), gtsam::Vector3(9.0, 9.0, 9.0));
  aux_values.insert(gtsam::symbol('k', 0), gtsam::Vector3(8.0, 8.0, 8.0));

  const auto filtered = registry.filter_aux_values(aux_values, false);
  EXPECT_TRUE(filtered.exists(iap::bspline_velocity_key(1)));
  EXPECT_TRUE(filtered.exists(iap::bspline_gyro_bias_key(1)));
  EXPECT_TRUE(filtered.exists(iap::bspline_accel_bias_key(1)));
  EXPECT_FALSE(filtered.exists(gtsam::symbol('j', 0)));
  EXPECT_FALSE(filtered.exists(gtsam::symbol('k', 0)));

  EXPECT_EQ(registry.active_shared_keys(false), (std::vector<gtsam::Key>{gtsam::symbol('g', 0)}));
  const auto telemetry = registry.telemetry();
  EXPECT_EQ(telemetry.active_shared_state_count, 1U);
}

TEST(BSplineFixedLagRegistryTest, TelemetryTracksLifecycleTransitions) {
  iap::BSplineFixedLagStateRegistry registry;

  {
    const auto telemetry = registry.telemetry();
    EXPECT_EQ(telemetry.lifecycle_state, iap::BSplineFixedLagLifecycleState::Empty);
    EXPECT_EQ(telemetry.control_point_count, 0U);
    EXPECT_EQ(telemetry.segment_count, 0U);
    EXPECT_FALSE(telemetry.has_active_segment);
    EXPECT_EQ(telemetry.active_shared_state_count, 3U);
  }

  iap::BSplineControlWindow window;
  window.initialize(2.0, 2.1, gtsam::Pose3());
  registry.reset_from_window(window);

  {
    const auto telemetry = registry.telemetry();
    EXPECT_EQ(telemetry.lifecycle_state, iap::BSplineFixedLagLifecycleState::WindowSeeded);
    EXPECT_EQ(telemetry.control_point_count, 4U);
    EXPECT_EQ(telemetry.segment_count, 0U);
    EXPECT_FALSE(telemetry.has_active_segment);
    EXPECT_DOUBLE_EQ(telemetry.lag_start_stamp, window.states().front().stamp);
    EXPECT_DOUBLE_EQ(telemetry.lag_end_stamp, window.states().back().stamp);
    EXPECT_EQ(telemetry.newest_control_index, window.states().back().index);
  }

  registry.append_segment(make_segment_state(window));

  {
    const auto telemetry = registry.telemetry();
    EXPECT_EQ(telemetry.lifecycle_state, iap::BSplineFixedLagLifecycleState::TrackingLidar);
    EXPECT_TRUE(telemetry.has_active_segment);
    EXPECT_EQ(telemetry.segment_count, 1U);
    EXPECT_EQ(telemetry.active_auxiliary_count, 1U);
    EXPECT_EQ(telemetry.active_shared_state_count, 3U);
    EXPECT_EQ(telemetry.newest_auxiliary_index, window.states()[1].index);
    EXPECT_DOUBLE_EQ(telemetry.latest_segment_stamp, window.segment_start());
    EXPECT_DOUBLE_EQ(telemetry.latest_segment_end, window.segment_end());
  }

  registry.set_shared_gnss_anchor(
    gtsam::Vector3(100.0, 200.0, 300.0),
    gtsam::Rot3::RzRyRx(0.01, 0.02, 0.03));

  {
    const auto telemetry = registry.telemetry();
    EXPECT_EQ(telemetry.lifecycle_state, iap::BSplineFixedLagLifecycleState::TrackingLidarGnss);
    EXPECT_TRUE(telemetry.gnss_anchor_initialized);
    EXPECT_EQ(telemetry.active_shared_state_count, 5U);
  }
}
