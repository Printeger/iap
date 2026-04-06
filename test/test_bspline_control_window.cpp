// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the Phase-1B B-spline control-point window.

#include <gtest/gtest.h>

#include <iap/odometry/bspline_control_window.hpp>

TEST(BSplineControlWindowTest, InitializeCreatesOrderedKeysAndTimes) {
  iap::BSplineControlWindow window;
  window.initialize(10.0, 10.1, gtsam::Pose3());

  ASSERT_TRUE(window.initialized());
  const auto states = window.states();
  EXPECT_EQ(states[0].index, 0U);
  EXPECT_EQ(states[1].index, 1U);
  EXPECT_EQ(states[2].index, 2U);
  EXPECT_EQ(states[3].index, 3U);
  EXPECT_LT(states[0].stamp, states[1].stamp);
  EXPECT_LT(states[1].stamp, states[2].stamp);
  EXPECT_LT(states[2].stamp, states[3].stamp);
}

TEST(BSplineControlWindowTest, AdvanceShiftsWindowAndAppendsFutureControlPoint) {
  iap::BSplineControlWindow window;
  window.initialize(0.0, 0.1, gtsam::Pose3());

  gtsam::Pose3 predicted_end(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
  window.advance(0.1, 0.2, predicted_end);

  const auto states = window.states();
  EXPECT_EQ(states[0].index, 1U);
  EXPECT_EQ(states[1].index, 2U);
  EXPECT_EQ(states[2].index, 3U);
  EXPECT_EQ(states[3].index, 4U);
  EXPECT_NEAR(states[1].stamp, 0.1, 1e-9);
  EXPECT_NEAR(states[2].stamp, 0.2, 1e-9);
  EXPECT_GT(states[3].stamp, states[2].stamp);
}

TEST(BSplineControlWindowTest, EvaluateReturnsInterpolatedPose) {
  iap::BSplineControlWindow window;
  window.initialize(0.0, 1.0, gtsam::Pose3());

  gtsam::Values values = window.values();
  values.update(iap::bspline_control_point_key(1), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));
  values.update(iap::bspline_control_point_key(2), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
  values.update(iap::bspline_control_point_key(3), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0)));
  window.update_from_values(values);

  const gtsam::Pose3 mid = window.evaluate(0.5);
  EXPECT_GT(mid.translation().x(), 0.0);
  EXPECT_LT(mid.translation().x(), 2.0);
}

TEST(BSplineControlWindowBufferTest, AppendWindowExtendsActiveControlSequence) {
  iap::BSplineControlWindow window;
  window.initialize(0.0, 0.1, gtsam::Pose3());

  iap::BSplineControlWindowBuffer buffer;
  buffer.reset_from_window(window);
  EXPECT_EQ(buffer.size(), 4U);

  gtsam::Pose3 predicted_end(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
  window.advance(0.1, 0.2, predicted_end);
  buffer.append_window(window);

  ASSERT_EQ(buffer.size(), 5U);
  EXPECT_EQ(buffer.states().front().index, 0U);
  EXPECT_EQ(buffer.states().back().index, 4U);
}

TEST(BSplineControlWindowBufferTest, PruneBeforeKeepsSplineSupportPoints) {
  iap::BSplineControlWindow window;
  window.initialize(0.0, 0.1, gtsam::Pose3());

  iap::BSplineControlWindowBuffer buffer;
  buffer.reset_from_window(window);

  window.advance(0.1, 0.2, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
  buffer.append_window(window);
  window.advance(0.2, 0.3, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2.0, 0.0, 0.0)));
  buffer.append_window(window);

  ASSERT_EQ(buffer.size(), 6U);
  buffer.prune_before(0.25);

  EXPECT_EQ(buffer.size(), 5U);
  EXPECT_EQ(buffer.states().front().index, 1U);
  EXPECT_LT(buffer.states().front().stamp, 0.25);
  EXPECT_GE(buffer.states()[3].stamp, 0.25);
}

TEST(BSplineControlWindowBufferTest, ValuesRoundTripUpdatesStoredControlPoses) {
  iap::BSplineControlWindow window;
  window.initialize(0.0, 0.1, gtsam::Pose3());

  iap::BSplineControlWindowBuffer buffer;
  buffer.reset_from_window(window);

  gtsam::Values values = buffer.values();
  values.update(iap::bspline_control_point_key(2), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.0, 0.0, 0.0)));
  buffer.update_from_values(values);

  ASSERT_EQ(buffer.size(), 4U);
  EXPECT_NEAR(buffer.states()[2].pose.translation().x(), 3.0, 1e-9);
}

TEST(BSplineControlWindowBufferTest, SplineControlPointsExposeVelocityStateWhenProvided) {
  iap::BSplineControlWindow window;
  window.initialize(0.0, 0.1, gtsam::Pose3());

  iap::BSplineControlWindowBuffer buffer;
  buffer.reset_from_window(window);

  gtsam::Values values;
  values.insert(iap::bspline_velocity_key(1), gtsam::Vector3(1.5, 0.0, 0.0));

  const auto control_points = buffer.spline_control_points(&values);
  ASSERT_EQ(control_points.size(), 4U);
  EXPECT_NEAR(control_points[1].vel.x(), 1.5, 1e-9);
  EXPECT_NEAR(control_points[0].vel.norm(), 0.0, 1e-9);
}
