// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the continuous-time B-spline LiDAR GICP factor engineering hooks.

#include <gtest/gtest.h>

#include <iap/odometry/integrated_bspline_gicp_factor.hpp>

#include <gtsam/linear/VectorValues.h>
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

gtsam_points::PointCloudCPU::Ptr make_custom_cloud(
  const std::vector<Eigen::Vector4d>& points,
  const std::vector<Eigen::Matrix3d>& cov3,
  const std::vector<double>* times = nullptr) {
  auto cloud = std::make_shared<gtsam_points::PointCloudCPU>(points);
  std::vector<Eigen::Matrix4d> covs(points.size(), Eigen::Matrix4d::Zero());
  for (std::size_t i = 0; i < covs.size(); ++i) {
    covs[i].block<3, 3>(0, 0) = cov3[i];
  }
  cloud->add_covs(covs);
  if (times) {
    cloud->add_times(*times);
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
  const std::shared_ptr<gtsam_points::iVox>& target,
  const gtsam_points::PointCloudCPU::Ptr& source_cloud);

iap::IntegratedBSplineGICPFactor make_factor(
  const gtsam_points::PointCloudCPU::Ptr& source_cloud = make_cloud(true)) {
  auto target_cloud = make_cloud(false);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);
  return make_factor(target, source_cloud);
}

iap::IntegratedBSplineGICPFactor make_factor(
  const std::shared_ptr<gtsam_points::iVox>& target,
  const gtsam_points::PointCloudCPU::Ptr& source_cloud) {
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
  EXPECT_EQ(stats.rejected_robust_count, 0U);
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

TEST(BSplineGICPFactorTest, SemiAnalyticPredictedErrorTracksNumericFullReference) {
  auto numeric_factor = make_factor();
  numeric_factor.set_jacobian_mode(iap::IntegratedBSplineGICPFactor::JacobianMode::NUMERIC_FULL);

  auto semi_factor = make_factor();
  semi_factor.set_jacobian_mode(iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC);

  auto values = make_identity_control_values();
  values.update(
    iap::bspline_control_point_key(0),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.02, -0.01, 0.01), gtsam::Point3(-0.01, 0.0, 0.0)));
  values.update(
    iap::bspline_control_point_key(1),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.01, 0.03, -0.02), gtsam::Point3(0.03, -0.02, 0.0)));
  values.update(
    iap::bspline_control_point_key(2),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, 0.01, -0.03), gtsam::Point3(0.06, 0.01, 0.0)));

  const auto numeric_linear = numeric_factor.linearize(values);
  const auto semi_linear = semi_factor.linearize(values);
  ASSERT_TRUE(static_cast<bool>(numeric_linear));
  ASSERT_TRUE(static_cast<bool>(semi_linear));

  gtsam::VectorValues delta;
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    gtsam::Vector6 d;
    d << 1.0, -0.5, 0.25, 0.4, -0.2, 0.1;
    d *= 1e-5 * (1.0 + 0.1 * static_cast<double>(i));
    delta.insert(iap::bspline_control_point_key(i), d);
  }

  const double numeric_pred = numeric_linear->error(delta);
  const double semi_pred = semi_linear->error(delta);
  const double rel_diff =
    std::abs(numeric_pred - semi_pred) / std::max(1e-9, std::max(std::abs(numeric_pred), std::abs(semi_pred)));

  EXPECT_TRUE(std::isfinite(numeric_pred));
  EXPECT_TRUE(std::isfinite(semi_pred));
  EXPECT_LT(rel_diff, 0.65);
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

TEST(BSplineGICPFactorTest, RobustWeightFloorRejectsHeavilyDownweightedMatches) {
  auto baseline_factor = make_factor(make_outlier_source_cloud());
  baseline_factor.set_enable_profiling(true);
  baseline_factor.set_max_correspondence_distance(10.0);
  baseline_factor.set_robust_kernel(iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER, 1.0);

  auto floor_factor = make_factor(make_outlier_source_cloud());
  floor_factor.set_enable_profiling(true);
  floor_factor.set_max_correspondence_distance(10.0);
  floor_factor.set_robust_kernel(iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER, 1.0);
  floor_factor.set_robust_weight_floor(0.8);

  const auto values = make_identity_control_values();
  const double baseline_error = baseline_factor.error(values);
  const double floor_error = floor_factor.error(values);

  EXPECT_TRUE(std::isfinite(floor_error));
  EXPECT_LE(floor_error, baseline_error);
  EXPECT_LT(floor_factor.num_inliers(), baseline_factor.num_inliers());
  EXPECT_GT(floor_factor.last_profiling_stats().rejected_robust_count, 0U);
}

TEST(BSplineGICPFactorTest, MahalanobisCandidateSelectionCanBeatNearestEuclideanNeighbor) {
  const std::vector<Eigen::Vector4d> target_points = {
    Eigen::Vector4d(0.15, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.30, 0.0, 0.0, 1.0),
  };
  const std::vector<Eigen::Matrix3d> target_covs = {
    Eigen::Matrix3d::Identity() * 1e-3,
    Eigen::Matrix3d::Identity() * 10.0,
  };
  auto target_cloud = make_custom_cloud(target_points, target_covs);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);

  const std::vector<Eigen::Vector4d> source_points = {
    Eigen::Vector4d(0.20, 0.0, 0.0, 1.0),
  };
  const std::vector<Eigen::Matrix3d> source_covs = {
    Eigen::Matrix3d::Identity() * 1e-3,
  };
  const std::vector<double> source_times = {0.0};
  auto source_cloud = make_custom_cloud(source_points, source_covs, &source_times);

  auto single_candidate = make_factor(target, source_cloud);
  single_candidate.set_correspondence_candidate_count(1);
  single_candidate.set_max_correspondence_distance(1.0);

  auto multi_candidate = make_factor(target, source_cloud);
  multi_candidate.set_correspondence_candidate_count(2);
  multi_candidate.set_max_correspondence_distance(1.0);

  const auto values = make_identity_control_values();
  EXPECT_LT(multi_candidate.error(values), single_candidate.error(values));
}

TEST(BSplineGICPFactorTest, AmbiguityRatioCanRejectNearlyEquivalentCandidates) {
  const std::vector<Eigen::Vector4d> target_points = {
    Eigen::Vector4d(0.15, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.25, 0.0, 0.0, 1.0),
  };
  const std::vector<Eigen::Matrix3d> target_covs = {
    Eigen::Matrix3d::Identity() * 1e-3,
    Eigen::Matrix3d::Identity() * 1e-3,
  };
  auto target_cloud = make_custom_cloud(target_points, target_covs);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);

  const std::vector<Eigen::Vector4d> source_points = {
    Eigen::Vector4d(0.20, 0.0, 0.0, 1.0),
  };
  const std::vector<Eigen::Matrix3d> source_covs = {
    Eigen::Matrix3d::Identity() * 1e-3,
  };
  const std::vector<double> source_times = {0.0};
  auto source_cloud = make_custom_cloud(source_points, source_covs, &source_times);

  auto factor = make_factor(target, source_cloud);
  factor.set_enable_profiling(true);
  factor.set_correspondence_candidate_count(2);
  factor.set_correspondence_accept_ratio(0.95);
  factor.set_max_correspondence_distance(1.0);

  const auto values = make_identity_control_values();
  const double error = factor.error(values);
  EXPECT_NEAR(error, 0.0, 1e-12);
  EXPECT_EQ(factor.num_inliers(), 0);
  EXPECT_EQ(factor.last_profiling_stats().rejected_ambiguity_count, 1U);
}

TEST(BSplineGICPFactorTest, CorrespondenceScoreGapCanRejectNearlyEquivalentCandidates) {
  const std::vector<Eigen::Vector4d> target_points = {
    Eigen::Vector4d(0.15, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.25, 0.0, 0.0, 1.0),
  };
  const std::vector<Eigen::Matrix3d> target_covs = {
    Eigen::Matrix3d::Identity() * 1e-3,
    Eigen::Matrix3d::Identity() * 1e-3,
  };
  auto target_cloud = make_custom_cloud(target_points, target_covs);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);

  const std::vector<Eigen::Vector4d> source_points = {
    Eigen::Vector4d(0.20, 0.0, 0.0, 1.0),
  };
  const std::vector<Eigen::Matrix3d> source_covs = {
    Eigen::Matrix3d::Identity() * 1e-3,
  };
  const std::vector<double> source_times = {0.0};
  auto source_cloud = make_custom_cloud(source_points, source_covs, &source_times);

  auto factor = make_factor(target, source_cloud);
  factor.set_enable_profiling(true);
  factor.set_correspondence_candidate_count(2);
  factor.set_correspondence_min_score_gap(0.1);
  factor.set_max_correspondence_distance(1.0);

  const auto values = make_identity_control_values();
  const double error = factor.error(values);
  EXPECT_NEAR(error, 0.0, 1e-12);
  EXPECT_EQ(factor.num_inliers(), 0);
  EXPECT_EQ(factor.last_profiling_stats().rejected_ambiguity_count, 1U);
}
