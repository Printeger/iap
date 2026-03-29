// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the continuous-time B-spline LiDAR GICP factor engineering hooks.

#include <gtest/gtest.h>

#include <iap/odometry/integrated_bspline_gicp_factor.hpp>

#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <cmath>

namespace {

gtsam_points::PointCloudCPU::Ptr make_cloud(bool with_times) {
  std::vector<Eigen::Vector4d> points = {
    Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
    Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.0, 1.0, 0.0, 1.0),
    Eigen::Vector4d(0.5, 0.2, 0.0, 1.0),
  };

  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);

  std::vector<Eigen::Matrix4d> covs(points.size(), Eigen::Matrix4d::Zero());
  for (auto& cov : covs) {
    cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-3;
  }
  cloud->add_covs(covs);

  if (with_times) {
    cloud->add_times(std::vector<double>{0.0, 0.33, 0.66, 1.0});
  }

  return cloud;
}

gtsam_points::PointCloudCPU::Ptr make_outlier_source_cloud() {
  std::vector<Eigen::Vector4d> points = {
    Eigen::Vector4d(0.0, 0.0, 0.0, 1.0),
    Eigen::Vector4d(1.0, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.0, 1.0, 0.0, 1.0),
    Eigen::Vector4d(0.5, 0.2, 0.0, 1.0),
    Eigen::Vector4d(4.0, 4.0, 0.0, 1.0),
  };

  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  std::vector<Eigen::Matrix4d> covs(points.size(), Eigen::Matrix4d::Zero());
  for (auto& cov : covs) {
    cov.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 1e-3;
  }
  cloud->add_covs(covs);
  cloud->add_times(std::vector<double>{0.0, 0.33, 0.66, 1.0, 0.5});
  return cloud;
}

gtsam::Values make_identity_control_values() {
  gtsam::Values values;
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    values.insert(iap::bspline_control_point_key(i), gtsam::Pose3());
  }
  return values;
}

iap::IntegratedBSplineGICPFactor make_factor(
  const gtsam_points::PointCloudCPU::Ptr& source_cloud = make_cloud(true)) {
  auto target_cloud = make_cloud(false);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);

  std::array<gtsam::Key, iap::kBSplineControlPointCount> keys{};
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    keys[i] = iap::bspline_control_point_key(i);
  }

  iap::IntegratedBSplineGICPFactor factor(keys, target, source_cloud);
  factor.set_max_correspondence_distance(2.0);
  return factor;
}

}  // namespace

TEST(BSplineGICPFactorTest, ErrorChangesWhenControlPosesArePerturbed) {
  auto factor = make_factor();
  auto values = make_identity_control_values();
  const double base_error = factor.error(values);
  EXPECT_TRUE(std::isfinite(base_error));

  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    values.update(
      iap::bspline_control_point_key(i),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.05, 0.0, 0.0)));
  }

  const double perturbed_error = factor.error(values);
  EXPECT_TRUE(std::isfinite(perturbed_error));
  EXPECT_GT(std::abs(perturbed_error - base_error), 1e-3);
}

TEST(BSplineGICPFactorTest, LinearizationCheckTracksNonlinearErrorNearLinearizationPoint) {
  auto factor = make_factor();
  const auto values = make_identity_control_values();

  const auto check = factor.check_linearization(values, 5e-5);
  ASSERT_TRUE(check.valid);
  EXPECT_TRUE(std::isfinite(check.predicted_error));
  EXPECT_TRUE(std::isfinite(check.actual_error));
  EXPECT_TRUE(std::isfinite(check.rel_error));
  EXPECT_LT(check.rel_error, 0.75);
}

TEST(BSplineGICPFactorTest, ProfilingReportsMatchedCorrespondences) {
  auto factor = make_factor();
  factor.set_enable_profiling(true);

  const auto values = make_identity_control_values();
  const auto linear = factor.linearize(values);
  ASSERT_TRUE(static_cast<bool>(linear));

  const auto& stats = factor.last_profiling_stats();
  EXPECT_TRUE(stats.valid);
  EXPECT_STREQ(stats.stage, "linearize");
  EXPECT_EQ(stats.source_point_count, 4U);
  EXPECT_EQ(stats.target_point_count, 4U);
  EXPECT_EQ(stats.matched_point_count, 4U);
  EXPECT_EQ(stats.inlier_point_count, 4U);
  EXPECT_NEAR(stats.match_ratio, 1.0, 1e-9);
  EXPECT_NEAR(stats.inlier_ratio, 1.0, 1e-9);
  EXPECT_GE(stats.pose_update_ms, 0.0);
  EXPECT_GE(stats.correspondence_ms, 0.0);
  EXPECT_GE(stats.accumulation_ms, 0.0);
  EXPECT_GE(stats.total_ms, 0.0);
}

TEST(BSplineGICPFactorTest, SemiAnalyticLinearizationRemainsLocallyConsistentAtPerturbedState) {
  auto semi_factor = make_factor();
  semi_factor.set_jacobian_mode(iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC);

  auto values = make_identity_control_values();
  values.update(
    iap::bspline_control_point_key(1),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.03), gtsam::Point3(0.03, -0.01, 0.0)));
  values.update(
    iap::bspline_control_point_key(2),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.02, 0.01, -0.01), gtsam::Point3(0.06, 0.02, 0.0)));

  const auto check = semi_factor.check_linearization(values, 2e-5);
  ASSERT_TRUE(check.valid);
  EXPECT_TRUE(std::isfinite(check.predicted_error));
  EXPECT_TRUE(std::isfinite(check.actual_error));
  EXPECT_LT(check.rel_error, 0.9);
}

TEST(BSplineGICPFactorTest, OutlierThresholdRejectsLargeResidualMatches) {
  auto baseline = make_factor(make_outlier_source_cloud());
  baseline.set_max_correspondence_distance(10.0);

  auto factor = make_factor(make_outlier_source_cloud());
  factor.set_enable_profiling(true);
  factor.set_max_correspondence_distance(10.0);
  factor.set_outlier_mahalanobis_threshold(10.0);

  const auto values = make_identity_control_values();
  const double baseline_error = baseline.error(values);
  const int baseline_inliers = baseline.num_inliers();
  const double error = factor.error(values);
  EXPECT_TRUE(std::isfinite(error));
  EXPECT_LT(error, baseline_error);
  EXPECT_LT(factor.num_inliers(), baseline_inliers);
  EXPECT_LT(factor.inlier_fraction(), baseline.inlier_fraction());

  const auto& stats = factor.last_profiling_stats();
  EXPECT_TRUE(stats.valid);
  EXPECT_EQ(stats.matched_point_count, 5U);
  EXPECT_GT(stats.rejected_outlier_count, 0U);
}

TEST(BSplineGICPFactorTest, RobustKernelDownweightsOutlierResiduals) {
  auto plain_factor = make_factor(make_outlier_source_cloud());
  plain_factor.set_max_correspondence_distance(10.0);

  auto robust_factor = make_factor(make_outlier_source_cloud());
  robust_factor.set_enable_profiling(true);
  robust_factor.set_max_correspondence_distance(10.0);
  robust_factor.set_robust_kernel(iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER, 1.0);

  const auto values = make_identity_control_values();
  const double plain_error = plain_factor.error(values);
  const double robust_error = robust_factor.error(values);

  EXPECT_TRUE(std::isfinite(robust_error));
  EXPECT_LT(robust_error, plain_error);
  EXPECT_LT(robust_factor.last_profiling_stats().mean_robust_weight, 1.0);
}
