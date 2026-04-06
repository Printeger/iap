// IAP-RQ-300 / IAP-RQ-410:
// Regression tests for spline-native explicit-knot query semantics.

#include <gtest/gtest.h>

#include <cmath>

#include <iap/odometry/spline_evaluator.hpp>

namespace {

gtsam::Pose3 translated_pose(double x) {
  return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(x, 0.0, 0.0));
}

std::shared_ptr<iap::SplineStateLayout> make_layout(
  const std::vector<double>& knots,
  const std::vector<std::size_t>& control_indices) {
  auto layout = std::make_shared<iap::SplineStateLayout>();
  layout->set_knots(knots);

  std::vector<iap::BSplineControlPointState> controls;
  controls.reserve(control_indices.size());
  for (std::size_t i = 0; i < control_indices.size(); ++i) {
    controls.push_back(iap::BSplineControlPointState{
      control_indices[i],
      static_cast<double>(i),
      translated_pose(static_cast<double>(control_indices[i])),
    });
  }
  layout->set_controls(std::move(controls));
  return layout;
}

gtsam::Values make_values(const iap::SplineStateLayout& layout) {
  gtsam::Values values;
  for (const auto& control : layout.controls()) {
    values.insert(iap::bspline_control_point_key(control.index), control.pose);
  }
  return values;
}

}  // namespace

TEST(SplineStateLayoutTest, SupportAtUsesExplicitKnotsAndSensorOffsets) {
  auto layout = make_layout(
    {0.0, 0.0, 0.0, 0.0, 1.0, 3.0, 3.0, 3.0, 3.0},
    {10, 11, 12, 13, 14});

  iap::SplineSensorModel lidar_model;
  lidar_model.id = iap::SplineSensorId::Lidar;
  lidar_model.time_offset = 0.75;
  layout->set_sensor_model(iap::SplineSensorId::Lidar, lidar_model);

  const auto support = layout->support_at(0.75, iap::SplineSensorId::Lidar);
  ASSERT_TRUE(support.has_value());
  EXPECT_EQ(support->span_idx, 4);
  EXPECT_DOUBLE_EQ(support->query_time, 1.5);
  EXPECT_DOUBLE_EQ(support->dt, 2.0);
  EXPECT_DOUBLE_EQ(support->u, 0.25);
  EXPECT_EQ(support->ctrl_indices, (std::array<std::size_t, 4>{1, 2, 3, 4}));
  EXPECT_EQ(
    support->pose_keys,
    (std::array<gtsam::Key, 4>{
      iap::bspline_control_point_key(11),
      iap::bspline_control_point_key(12),
      iap::bspline_control_point_key(13),
      iap::bspline_control_point_key(14),
    }));

  EXPECT_FALSE(layout->support_at(-1.0, iap::SplineSensorId::Lidar).has_value());
  EXPECT_FALSE(layout->support_at(3.0, iap::SplineSensorId::Lidar).has_value());
}

TEST(SplineStateLayoutTest, SupportsInRangeReturnsOneSupportPerCoveredSpan) {
  auto layout = make_layout(
    {0.0, 0.0, 0.0, 0.0, 1.0, 3.0, 3.0, 3.0, 3.0},
    {20, 21, 22, 23, 24});

  iap::SplineSensorModel gnss_model;
  gnss_model.id = iap::SplineSensorId::Gnss;
  gnss_model.time_offset = -0.25;
  layout->set_sensor_model(iap::SplineSensorId::Gnss, gnss_model);

  const auto supports = layout->supports_in_range(0.5, 2.5, iap::SplineSensorId::Gnss);
  ASSERT_EQ(supports.size(), 2U);

  EXPECT_EQ(supports[0].span_idx, 3);
  EXPECT_NEAR(supports[0].query_time, 0.625, 1e-9);
  EXPECT_DOUBLE_EQ(supports[0].dt, 1.0);
  EXPECT_EQ(supports[0].ctrl_indices, (std::array<std::size_t, 4>{0, 1, 2, 3}));

  EXPECT_EQ(supports[1].span_idx, 4);
  EXPECT_NEAR(supports[1].query_time, 1.625, 1e-9);
  EXPECT_DOUBLE_EQ(supports[1].dt, 2.0);
  EXPECT_EQ(supports[1].ctrl_indices, (std::array<std::size_t, 4>{1, 2, 3, 4}));
}

TEST(SplineEvaluatorTest, BasisAndDerivativesRespectSpanDuration) {
  auto uniform_layout = make_layout(
    {0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0},
    {0, 1, 2, 3});
  auto non_uniform_layout = make_layout(
    {0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 2.0},
    {0, 1, 2, 3});

  const auto uniform_support = uniform_layout->support_at(0.5, iap::SplineSensorId::Imu);
  const auto non_uniform_support = non_uniform_layout->support_at(1.0, iap::SplineSensorId::Imu);
  ASSERT_TRUE(uniform_support.has_value());
  ASSERT_TRUE(non_uniform_support.has_value());
  ASSERT_NEAR(uniform_support->u, non_uniform_support->u, 1e-12);

  iap::SplineEvaluator uniform_evaluator(uniform_layout);
  iap::SplineEvaluator non_uniform_evaluator(non_uniform_layout);
  const auto uniform_weights = uniform_evaluator.basis(*uniform_support);
  const auto uniform_d1 = uniform_evaluator.basis_d1(*uniform_support);
  const auto uniform_d2 = uniform_evaluator.basis_d2(*uniform_support);
  const auto non_uniform_d1 = non_uniform_evaluator.basis_d1(*non_uniform_support);
  const auto non_uniform_d2 = non_uniform_evaluator.basis_d2(*non_uniform_support);

  EXPECT_NEAR(uniform_weights[0] + uniform_weights[1] + uniform_weights[2] + uniform_weights[3], 1.0, 1e-12);
  EXPECT_NEAR(uniform_d1[0] + uniform_d1[1] + uniform_d1[2] + uniform_d1[3], 0.0, 1e-12);
  EXPECT_NEAR(uniform_d2[0] + uniform_d2[1] + uniform_d2[2] + uniform_d2[3], 0.0, 1e-12);

  for (double value : non_uniform_d1) {
    EXPECT_TRUE(std::isfinite(value));
  }
  for (double value : non_uniform_d2) {
    EXPECT_TRUE(std::isfinite(value));
  }

  const gtsam::Values uniform_values = make_values(*uniform_layout);
  const gtsam::Values non_uniform_values = make_values(*non_uniform_layout);
  const auto uniform_velocity = uniform_evaluator.eval_world_velocity(uniform_values, *uniform_support, iap::SplineSensorId::Imu);
  const auto non_uniform_velocity = non_uniform_evaluator.eval_world_velocity(non_uniform_values, *non_uniform_support, iap::SplineSensorId::Imu);
  const auto uniform_accel = uniform_evaluator.eval_world_acceleration(uniform_values, *uniform_support, iap::SplineSensorId::Imu);
  const auto non_uniform_accel = non_uniform_evaluator.eval_world_acceleration(non_uniform_values, *non_uniform_support, iap::SplineSensorId::Imu);

  EXPECT_NEAR(non_uniform_velocity.x(), 0.5 * uniform_velocity.x(), 1e-12);
  EXPECT_NEAR(uniform_accel.norm(), 0.0, 1e-12);
  EXPECT_NEAR(non_uniform_accel.norm(), 0.0, 1e-12);
}
