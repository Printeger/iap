// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the continuous-time B-spline trajectory foundation.

#include <gtest/gtest.h>

#include <iap/odometry/bspline_trajectory.hpp>

namespace {

iap::SplineControlPoint make_control_point(double stamp, const Eigen::Vector3d& pos, double yaw, double sigma) {
  iap::SplineControlPoint cp;
  cp.stamp = stamp;
  cp.pose = Eigen::Isometry3d::Identity();
  cp.pose.linear() = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  cp.pose.translation() = pos;
  cp.sigma = sigma;
  return cp;
}

}  // namespace

TEST(BSplineTrajectoryTest, UniformTrajectorySamplesStayInsideWindow) {
  iap::BSplineTrajectory::Params params;
  params.knot_mode = iap::SplineKnotMode::Uniform;

  iap::BSplineTrajectory trajectory(params);
  trajectory.set_control_points({
    make_control_point(0.0, Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, 1.0),
    make_control_point(1.0, Eigen::Vector3d(1.0, 0.0, 0.0), 0.1, 1.2),
    make_control_point(2.0, Eigen::Vector3d(2.0, 0.0, 0.0), 0.2, 1.4),
    make_control_point(3.0, Eigen::Vector3d(3.0, 0.0, 0.0), 0.3, 1.6),
  });

  ASSERT_FALSE(trajectory.empty());
  EXPECT_DOUBLE_EQ(trajectory.start_time(), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.end_time(), 3.0);

  const auto mid = trajectory.sample(1.5);
  ASSERT_TRUE(mid.has_value());
  EXPECT_GE(mid->pose.translation().x(), 0.0);
  EXPECT_LE(mid->pose.translation().x(), 3.0);
  EXPECT_GT(mid->sigma, 0.0);

  const auto latest = trajectory.latest_sample();
  ASSERT_TRUE(latest.has_value());
  EXPECT_NEAR(latest->stamp, 3.0, 1e-9);
}

TEST(BSplineTrajectoryTest, NonUniformTrajectoryExposesControlWindow) {
  iap::BSplineTrajectory::Params params;
  params.knot_mode = iap::SplineKnotMode::NonUniform;

  iap::BSplineTrajectory trajectory(params);
  trajectory.set_control_points({
    make_control_point(0.0, Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, 0.5),
    make_control_point(0.4, Eigen::Vector3d(0.5, 0.2, 0.0), 0.1, 0.6),
    make_control_point(1.2, Eigen::Vector3d(1.3, 0.4, 0.0), 0.2, 0.8),
    make_control_point(2.5, Eigen::Vector3d(2.5, 0.6, 0.0), 0.4, 1.0),
    make_control_point(4.0, Eigen::Vector3d(3.9, 0.9, 0.0), 0.5, 1.2),
  });

  const auto meta = trajectory.meta();
  EXPECT_EQ(meta.knot_mode, iap::SplineKnotMode::NonUniform);
  EXPECT_EQ(meta.order, 3);
  EXPECT_EQ(meta.control_point_count, 5U);
  EXPECT_FALSE(trajectory.knot_vector().empty());

  const auto snapshot = trajectory.clone_window();
  EXPECT_EQ(snapshot.control_points.size(), 5U);
  EXPECT_EQ(snapshot.knots.size(), trajectory.knot_vector().size());
}

TEST(BSplineTrajectoryTest, RangeSamplingIncludesEndPoint) {
  iap::BSplineTrajectory trajectory;
  trajectory.set_control_points({
    make_control_point(0.0, Eigen::Vector3d(0.0, 0.0, 0.0), 0.0, 0.2),
    make_control_point(0.5, Eigen::Vector3d(0.5, 0.0, 0.0), 0.0, 0.3),
    make_control_point(1.0, Eigen::Vector3d(1.0, 0.0, 0.0), 0.0, 0.4),
    make_control_point(1.5, Eigen::Vector3d(1.5, 0.0, 0.0), 0.0, 0.5),
  });

  const auto samples = trajectory.sample_range(0.0, 1.5, 0.4);
  ASSERT_FALSE(samples.empty());
  EXPECT_NEAR(samples.front().stamp, 0.0, 1e-9);
  EXPECT_NEAR(samples.back().stamp, 1.5, 1e-9);

  for (std::size_t i = 1; i < samples.size(); ++i) {
    EXPECT_GE(samples[i].stamp, samples[i - 1].stamp);
  }
}
