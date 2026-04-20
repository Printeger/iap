// IAP-RQ-300 / IAP-RQ-410:
// Unit tests for the continuous-time B-spline LiDAR GICP factor engineering hooks.

#include <gtest/gtest.h>

#include <iap/odometry/bspline_pose_jacobian.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor.hpp>
#include <gtsam_points/config.hpp>
#ifdef GTSAM_POINTS_USE_CUDA
#include <iap/odometry/integrated_bspline_gicp_factor_gpu.hpp>
#include <iap/odometry/integrated_bspline_gicp_factor_gpu_kernel.hpp>
#include <cuda_runtime_api.h>
#include <gtsam_points/cuda/stream_temp_buffer_roundrobin.hpp>
#endif

#include <gtsam/linear/VectorValues.h>
#include <gtsam_points/ann/ivox.hpp>
#include <gtsam_points/types/point_cloud_cpu.hpp>

#include <cmath>
#include <cstdio>
#include <string>

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
    // This extra point still finds a target within the search radius, but the
    // residual is large enough to exercise outlier / robust-kernel handling.
    Eigen::Vector4d(1.4, 0.4, 0.0, 1.0),
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

iap::SplineBucketContext make_full_bucket_context() {
  iap::SplineBucketContext ctx;
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    ctx.support.ctrl_indices[i] = i;
    ctx.support.pose_keys[i] = iap::bspline_control_point_key(i);
  }
  ctx.support.u = 0.5;
  ctx.point_indices = {0, 1, 2, 3};
  return ctx;
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

iap::IntegratedSplineGICPFactor make_bucket_factor(
  const std::shared_ptr<gtsam_points::iVox>& target,
  const gtsam_points::PointCloudCPU::Ptr& source_cloud) {
  iap::IntegratedSplineGICPFactor factor(make_full_bucket_context(), target, source_cloud);
  factor.set_max_correspondence_distance(2.0);
  return factor;
}

std::string read_file_contents(std::FILE* file) {
  std::string contents;
  if (!file) {
    return contents;
  }

  std::fflush(file);
  std::rewind(file);

  char buffer[1024];
  while (std::fgets(buffer, sizeof(buffer), file)) {
    contents += buffer;
  }
  return contents;
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
  EXPECT_LT(check.rel_error, 1.01);
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
  EXPECT_EQ(stats.unique_target_count, 4U);
  EXPECT_EQ(stats.max_target_reuse, 1U);
  EXPECT_EQ(stats.time_bucket_count, 4U);
  EXPECT_EQ(stats.max_time_bucket_population, 1U);
  EXPECT_GE(stats.candidate_evaluation_count, 4U);
  EXPECT_NEAR(stats.match_ratio, 1.0, 1e-9);
  EXPECT_NEAR(stats.inlier_ratio, 1.0, 1e-9);
  EXPECT_NEAR(stats.mean_time_bucket_population, 1.0, 1e-9);
  EXPECT_GE(stats.mean_candidates_per_source, 1.0);
  EXPECT_NEAR(stats.unique_target_ratio, 1.0, 1e-9);
  EXPECT_NEAR(stats.max_target_reuse_ratio, 0.25, 1e-9);
  EXPECT_NEAR(stats.mean_match_distance, 0.0, 1e-9);
  EXPECT_NEAR(stats.max_match_distance, 0.0, 1e-9);
  EXPECT_NEAR(stats.mean_match_score, 0.0, 1e-9);
  EXPECT_GE(stats.mean_score_gap, 0.0);
  EXPECT_GE(stats.mean_score_ratio, 0.0);
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

TEST(BSplineGICPFactorTest, NumericReferenceCheckSeparatesRotationAndTranslationAgreement) {
  auto factor = make_factor();
  factor.set_jacobian_mode(iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC);

  auto values = make_identity_control_values();
  values.update(
    iap::bspline_control_point_key(0),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.015, -0.01, 0.02), gtsam::Point3(-0.01, 0.0, 0.0)));
  values.update(
    iap::bspline_control_point_key(1),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.02, 0.01, -0.015), gtsam::Point3(0.02, -0.01, 0.0)));
  values.update(
    iap::bspline_control_point_key(2),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, 0.02, -0.01), gtsam::Point3(0.05, 0.02, 0.0)));

  const auto check = factor.check_against_numeric_full(values, 1e-5);
  ASSERT_TRUE(check.valid);
  EXPECT_TRUE(std::isfinite(check.numeric_rotation_predicted_error));
  EXPECT_TRUE(std::isfinite(check.semi_rotation_predicted_error));
  EXPECT_TRUE(std::isfinite(check.rotation_actual_error));
  EXPECT_TRUE(std::isfinite(check.numeric_translation_predicted_error));
  EXPECT_TRUE(std::isfinite(check.semi_translation_predicted_error));
  EXPECT_TRUE(std::isfinite(check.translation_actual_error));
  for (double axis_rel : check.axis_rotation_rel_error) {
    EXPECT_TRUE(std::isfinite(axis_rel));
  }
  EXPECT_LT(check.max_rotation_axis_rel_error, 0.9);
  EXPECT_LT(check.mean_rotation_axis_rel_error, 0.75);
  EXPECT_LT(check.worst_rotation_axis, 3U);
  EXPECT_LT(check.rotation_rel_error, 0.75);
  EXPECT_LT(check.translation_rel_error, 0.35);
}

TEST(BSplineGICPFactorTest, SharedSplinePoseSemiAnalyticJacobianTracksNumericReference) {
  std::array<gtsam::Pose3, iap::kBSplineControlPointCount> poses = {
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.03), gtsam::Point3(-0.10, 0.02, 0.01)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.03, 0.01, -0.02), gtsam::Point3(0.05, -0.03, 0.00)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(0.02, 0.04, -0.01), gtsam::Point3(0.18, 0.04, -0.02)),
    gtsam::Pose3(gtsam::Rot3::RzRyRx(-0.01, -0.02, 0.02), gtsam::Point3(0.30, 0.08, 0.03)),
  };

  double worst_rotation_rel = 0.0;
  double worst_translation_abs = 0.0;
  for (double u : {0.05, 0.35, 0.72, 0.95}) {
    const auto semi = iap::bspline_pose_jacobians_semi_analytic(poses, u);
    const auto numeric = iap::bspline_pose_jacobians_numeric(poses, u, 1e-6);

    for (std::size_t k = 0; k < iap::kBSplineControlPointCount; ++k) {
      const auto semi_rot = semi[k].block<3, 3>(0, 0);
      const auto numeric_rot = numeric[k].block<3, 3>(0, 0);
      const auto semi_trans = semi[k].block<3, 3>(3, 3);
      const auto numeric_trans = numeric[k].block<3, 3>(3, 3);

      worst_rotation_rel = std::max(
        worst_rotation_rel,
        (semi_rot - numeric_rot).norm() / std::max(1e-9, numeric_rot.norm()));
      worst_translation_abs = std::max(worst_translation_abs, (semi_trans - numeric_trans).norm());

      EXPECT_LT((semi[k].block<3, 3>(0, 3).norm()), 1e-12);
      EXPECT_LT((semi[k].block<3, 3>(3, 0).norm()), 1e-12);
    }
  }

  EXPECT_LT(worst_rotation_rel, 0.30);
  EXPECT_LT(worst_translation_abs, 1e-5);
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

TEST(BSplineGICPFactorTest, ProfilingReportsCorrespondenceDiversityAndScoreDiagnostics) {
  const std::vector<Eigen::Vector4d> target_points = {
    Eigen::Vector4d(0.05, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.10, 0.0, 0.0, 1.0),
    Eigen::Vector4d(1.00, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.0, 1.0, 0.0, 1.0),
  };
  const std::vector<Eigen::Matrix3d> target_covs(target_points.size(), Eigen::Matrix3d::Identity() * 1e-3);
  auto target_cloud = make_custom_cloud(target_points, target_covs);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);

  const std::vector<Eigen::Vector4d> source_points = {
    Eigen::Vector4d(0.08, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.11, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.0, 1.0, 0.0, 1.0),
  };
  const std::vector<Eigen::Matrix3d> source_covs(source_points.size(), Eigen::Matrix3d::Identity() * 1e-3);
  const std::vector<double> source_times = {0.0, 0.5, 1.0};
  auto source_cloud = make_custom_cloud(source_points, source_covs, &source_times);

  auto factor = make_factor(target, source_cloud);
  factor.set_enable_profiling(true);
  factor.set_correspondence_candidate_count(3);
  factor.set_max_correspondence_distance(1.0);

  const auto values = make_identity_control_values();
  const auto linear = factor.linearize(values);
  ASSERT_TRUE(static_cast<bool>(linear));

  const auto& stats = factor.last_profiling_stats();
  EXPECT_TRUE(stats.valid);
  EXPECT_EQ(stats.matched_point_count, 3U);
  EXPECT_GE(stats.unique_target_count, 2U);
  EXPECT_LT(stats.unique_target_count, stats.matched_point_count);
  EXPECT_GT(stats.max_target_reuse, 1U);
  EXPECT_GT(stats.max_target_reuse_ratio, 0.33);
  EXPECT_GT(stats.mean_match_distance, 0.0);
  EXPECT_GE(stats.max_match_distance, stats.mean_match_distance);
  EXPECT_GE(stats.mean_match_score, 0.0);
  EXPECT_GT(stats.mean_score_gap, 0.0);
  EXPECT_GT(stats.mean_score_ratio, 0.0);

  iap::IntegratedBSplineGICPFactor::DegeneracyThresholds thresholds;
  thresholds.min_unique_target_ratio = stats.unique_target_ratio + 0.1;
  thresholds.max_target_reuse_ratio = std::max(0.01, stats.max_target_reuse_ratio - 0.1);
  const auto diagnostics = factor.diagnose_degeneracy(thresholds);
  ASSERT_TRUE(diagnostics.valid);
  EXPECT_TRUE(diagnostics.low_target_diversity);
  EXPECT_TRUE(diagnostics.high_target_reuse);
  EXPECT_TRUE(diagnostics.has_warning());
}

TEST(BSplineGICPFactorTest, DegeneracyDiagnosisFlagsHighAmbiguityRejection) {
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
  EXPECT_NEAR(factor.error(values), 0.0, 1e-12);

  iap::IntegratedBSplineGICPFactor::DegeneracyThresholds thresholds;
  thresholds.max_ambiguity_rejection_ratio = 0.5;
  const auto diagnostics = factor.diagnose_degeneracy(thresholds);
  ASSERT_TRUE(diagnostics.valid);
  EXPECT_TRUE(diagnostics.high_ambiguity_rejection);
  EXPECT_TRUE(diagnostics.has_warning());
}

TEST(BSplineGICPFactorTest, AggregateFactorResultsSummarizesWindowProfiles) {
  iap::BSplineLidarFactorResult first;
  first.valid = true;
  first.backend = iap::BSplineLidarFactorBackend::CPU_GICP;
  first.profile.valid = true;
  first.profile.source_point_count = 10;
  first.profile.target_point_count = 20;
  first.profile.matched_point_count = 8;
  first.profile.inlier_point_count = 6;
  first.profile.candidate_evaluation_count = 24;
  first.profile.unique_target_ratio = 0.50;
  first.profile.max_target_reuse_ratio = 0.25;
  first.profile.mean_candidates_per_source = 2.4;
  first.profile.mean_time_bucket_population = 1.5;
  first.profile.max_time_bucket_population = 3;
  first.profile.pose_update_ms = 1.0;
  first.profile.correspondence_ms = 2.0;
  first.profile.accumulation_ms = 3.0;
  first.profile.total_ms = 6.0;
  first.numeric_audit.valid = true;
  first.numeric_audit.rotation_rel_error = 0.2;
  first.numeric_audit.translation_rel_error = 0.1;
  first.numeric_audit.max_rotation_axis_rel_error = 0.3;

  iap::BSplineLidarFactorResult second;
  second.valid = true;
  second.backend = iap::BSplineLidarFactorBackend::CPU_GICP;
  second.profile.valid = true;
  second.profile.source_point_count = 20;
  second.profile.target_point_count = 30;
  second.profile.matched_point_count = 10;
  second.profile.inlier_point_count = 8;
  second.profile.candidate_evaluation_count = 50;
  second.profile.unique_target_ratio = 0.25;
  second.profile.max_target_reuse_ratio = 0.50;
  second.profile.mean_candidates_per_source = 2.5;
  second.profile.mean_time_bucket_population = 2.5;
  second.profile.max_time_bucket_population = 5;
  second.profile.pose_update_ms = 2.0;
  second.profile.correspondence_ms = 4.0;
  second.profile.accumulation_ms = 6.0;
  second.profile.total_ms = 12.0;
  second.degeneracy.valid = true;
  second.degeneracy.warning_count = 1;
  second.degeneracy.ambiguity_rejection_ratio = 0.4;
  second.numeric_audit.valid = true;
  second.numeric_audit.rotation_rel_error = 0.6;
  second.numeric_audit.translation_rel_error = 0.5;
  second.numeric_audit.max_rotation_axis_rel_error = 0.7;

  const auto summary = iap::aggregate_bspline_lidar_factor_results({first, second});
  ASSERT_TRUE(summary.valid);
  EXPECT_EQ(summary.result_count, 2U);
  EXPECT_EQ(summary.valid_profile_count, 2U);
  EXPECT_EQ(summary.warning_result_count, 1U);
  EXPECT_EQ(summary.total_source_point_count, 30U);
  EXPECT_EQ(summary.total_target_point_count, 50U);
  EXPECT_EQ(summary.total_matched_point_count, 18U);
  EXPECT_EQ(summary.total_inlier_point_count, 14U);
  EXPECT_EQ(summary.total_candidate_evaluation_count, 74U);
  EXPECT_NEAR(summary.weighted_match_ratio, 18.0 / 30.0, 1e-9);
  EXPECT_NEAR(summary.weighted_inlier_ratio, 14.0 / 30.0, 1e-9);
  EXPECT_NEAR(summary.mean_unique_target_ratio, 0.375, 1e-9);
  EXPECT_NEAR(summary.min_unique_target_ratio, 0.25, 1e-9);
  EXPECT_NEAR(summary.max_target_reuse_ratio, 0.50, 1e-9);
  EXPECT_NEAR(summary.max_ambiguity_rejection_ratio, 0.4, 1e-9);
  EXPECT_NEAR(summary.max_numeric_rel_error, 0.6, 1e-9);
  EXPECT_NEAR(summary.max_rotation_axis_rel_error, 0.7, 1e-9);
  EXPECT_NEAR(summary.total_pose_update_ms, 3.0, 1e-9);
  EXPECT_NEAR(summary.total_correspondence_ms, 6.0, 1e-9);
  EXPECT_NEAR(summary.total_accumulation_ms, 9.0, 1e-9);
  EXPECT_NEAR(summary.total_factor_ms, 18.0, 1e-9);
  EXPECT_NEAR(summary.mean_candidates_per_source, 2.45, 1e-9);
  EXPECT_NEAR(summary.mean_time_bucket_population, 2.0, 1e-9);
  EXPECT_EQ(summary.max_time_bucket_population, 5U);
}

TEST(BSplineGICPFactorTest, MinimalGpuResultUsesUnifiedReturnSurface) {
  const auto result = iap::make_bspline_lidar_minimal_result(
    iap::BSplineLidarFactorBackend::GPU_GICP,
    12.0,
    6,
    0.3,
    20,
    40,
    "gpu_linearized");

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.backend, iap::BSplineLidarFactorBackend::GPU_GICP);
  EXPECT_TRUE(result.profile.valid);
  EXPECT_TRUE(result.profile.minimal);
  EXPECT_EQ(result.profile.backend, iap::BSplineLidarFactorBackend::GPU_GICP);
  EXPECT_EQ(result.profile.source_point_count, 20U);
  EXPECT_EQ(result.profile.target_point_count, 40U);
  EXPECT_EQ(result.profile.matched_point_count, 6U);
  EXPECT_EQ(result.profile.inlier_point_count, 6U);
  EXPECT_NEAR(result.profile.match_ratio, 0.3, 1e-9);
  EXPECT_NEAR(result.profile.inlier_ratio, 0.3, 1e-9);
  EXPECT_STREQ(result.profile.stage, "gpu_linearized");
  EXPECT_NEAR(result.rmse, std::sqrt(12.0 / 6.0), 1e-9);

  const auto summary = iap::aggregate_bspline_lidar_factor_results({result});
  ASSERT_TRUE(summary.valid);
  EXPECT_EQ(summary.valid_profile_count, 1U);
  EXPECT_EQ(summary.detailed_profile_count, 0U);
  EXPECT_EQ(summary.minimal_profile_count, 1U);
  EXPECT_NEAR(summary.weighted_match_ratio, 0.3, 1e-9);
  EXPECT_NEAR(summary.weighted_inlier_ratio, 0.3, 1e-9);
}

TEST(BSplineGICPFactorTest, BaselineExportWritesWindowAndFactorRows) {
  auto first = iap::make_bspline_lidar_minimal_result(
    iap::BSplineLidarFactorBackend::GPU_GICP,
    4.0,
    2,
    0.5,
    4,
    8,
    "gpu_bucket");
  first.degeneracy.valid = true;
  first.degeneracy.warning_count = 1;

  auto second = iap::make_bspline_lidar_minimal_result(
    iap::BSplineLidarFactorBackend::GPU_GICP,
    9.0,
    3,
    0.75,
    4,
    8,
    "gpu_bucket");

  const auto export_data =
    iap::make_bspline_lidar_baseline_export(12.5, "CT_LIDAR_GPU", {first, second}, 1);
  ASSERT_TRUE(export_data.valid);
  EXPECT_EQ(export_data.backend, iap::BSplineLidarFactorBackend::GPU_GICP);
  EXPECT_EQ(export_data.current_factor_index, 1);
  EXPECT_EQ(export_data.factor_results.size(), 2U);

  std::FILE* file = std::tmpfile();
  ASSERT_NE(file, nullptr);
  iap::write_bspline_lidar_baseline_csv_header(file);
  iap::write_bspline_lidar_baseline_csv(file, export_data);

  const std::string csv = read_file_contents(file);
  std::fclose(file);

  EXPECT_NE(csv.find("stamp,row_type,frontend_mode,backend"), std::string::npos);
  EXPECT_NE(csv.find("window_summary,CT_LIDAR_GPU,GPU_GICP,-1,0"), std::string::npos);
  EXPECT_NE(csv.find("factor_result,CT_LIDAR_GPU,GPU_GICP,0,0"), std::string::npos);
  EXPECT_NE(csv.find("factor_result,CT_LIDAR_GPU,GPU_GICP,1,1"), std::string::npos);

  int newline_count = 0;
  for (const char ch : csv) {
    if (ch == '\n') {
      newline_count++;
    }
  }
  EXPECT_EQ(newline_count, 4);
}

#ifdef GTSAM_POINTS_USE_CUDA
TEST(BSplineGICPFactorTest, GpuFactorLinearizesAndReturnsUnifiedProfile) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device is not available in the current test environment";
  }

  auto target_cloud = make_cloud(false);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);

  gtsam_points::StreamTempBufferRoundRobin stream_buffers;
  auto stream_buffer = stream_buffers.get_stream_buffer();
  iap::IntegratedBSplineGICPFactorGPU factor(
    make_full_bucket_context(),
    target,
    make_cloud(true),
    stream_buffer.first,
    stream_buffer.second);
  factor.set_jacobian_mode(iap::IntegratedBSplineGICPFactorGPU::JacobianMode::SEMI_ANALYTIC);
  factor.set_numeric_eps(1e-4);
  factor.set_max_correspondence_distance(2.0);
  factor.set_correspondence_candidate_count(2);
  factor.set_correspondence_accept_ratio(0.99);
  factor.set_correspondence_min_score_gap(1e-6);
  factor.set_outlier_mahalanobis_threshold(10.0);
  factor.set_robust_kernel(iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER, 1.0);
  factor.set_robust_weight_floor(0.0);
  factor.set_enable_profiling(true);

  const auto values = make_identity_control_values();
  const auto linear = factor.linearize(values);
  ASSERT_TRUE(static_cast<bool>(linear));

  const double factor_error = factor.error(values);
  const auto numeric_audit = factor.check_against_numeric_full(values, 1e-5);
  ASSERT_TRUE(numeric_audit.valid);

  iap::IntegratedBSplineGICPFactor::DegeneracyThresholds thresholds;
  thresholds.min_match_ratio = 0.2;
  thresholds.min_inlier_ratio = 0.2;
  thresholds.min_unique_target_ratio = 0.2;
  thresholds.max_target_reuse_ratio = 0.8;
  thresholds.max_ambiguity_rejection_ratio = 0.8;
  thresholds.min_mean_score_gap = 1e-6;
  const auto degeneracy = factor.diagnose_degeneracy(thresholds);
  ASSERT_TRUE(degeneracy.valid);

  const auto result = factor.make_result(
    factor_error,
    factor.num_inliers(),
    factor.inlier_fraction(),
    &numeric_audit,
    &degeneracy);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.backend, iap::BSplineLidarFactorBackend::GPU_GICP);
  EXPECT_TRUE(result.profile.valid);
  EXPECT_FALSE(result.profile.minimal);
  EXPECT_EQ(result.profile.source_point_count, 4U);
  EXPECT_EQ(result.profile.time_bucket_count, 1U);
  EXPECT_EQ(result.profile.max_time_bucket_population, 4U);
  EXPECT_GE(result.profile.candidate_evaluation_count, 4U);
  EXPECT_GE(result.profile.matched_point_count, 4U);
  EXPECT_GE(result.profile.unique_target_count, 1U);
  EXPECT_GE(result.profile.mean_candidates_per_source, 1.0);
  EXPECT_GE(result.profile.pose_update_ms, 0.0);
  EXPECT_GE(result.profile.correspondence_ms, 0.0);
  EXPECT_GE(result.profile.total_ms, 0.0);
  EXPECT_TRUE(result.numeric_audit.valid);
  EXPECT_TRUE(result.degeneracy.valid);
}

TEST(BSplineGICPFactorTest, GpuFactorDegeneracyDiagnosticsReportOutlierHeavyScene) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device is not available in the current test environment";
  }

  auto target_cloud = make_cloud(false);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);

  iap::SplineBucketContext ctx = make_full_bucket_context();
  ctx.point_indices = {0, 1, 2, 3, 4};

  gtsam_points::StreamTempBufferRoundRobin stream_buffers;
  auto stream_buffer = stream_buffers.get_stream_buffer();
  iap::IntegratedBSplineGICPFactorGPU factor(
    ctx,
    target,
    make_outlier_source_cloud(),
    stream_buffer.first,
    stream_buffer.second);
  factor.set_jacobian_mode(iap::IntegratedBSplineGICPFactorGPU::JacobianMode::SEMI_ANALYTIC);
  factor.set_numeric_eps(1e-4);
  factor.set_max_correspondence_distance(2.0);
  factor.set_correspondence_candidate_count(2);
  factor.set_outlier_mahalanobis_threshold(0.05);
  factor.set_robust_kernel(iap::IntegratedBSplineGICPFactor::RobustKernel::CAUCHY, 0.1);
  factor.set_robust_weight_floor(0.5);
  factor.set_enable_profiling(true);

  const auto values = make_identity_control_values();
  factor.error(values);

  iap::IntegratedBSplineGICPFactor::DegeneracyThresholds thresholds;
  thresholds.min_match_ratio = 0.9;
  thresholds.min_inlier_ratio = 0.9;
  thresholds.min_unique_target_ratio = 0.9;
  thresholds.max_target_reuse_ratio = 0.4;
  const auto degeneracy = factor.diagnose_degeneracy(thresholds);
  ASSERT_TRUE(degeneracy.valid);
  EXPECT_TRUE(degeneracy.has_warning());

  const auto profile = factor.profiling_report();
  EXPECT_FALSE(profile.minimal);
  EXPECT_GE(profile.rejected_outlier_count + profile.rejected_robust_count, 1U);
}

TEST(BSplineGICPFactorTest, GpuFactorCanRefreshTargetWithoutRebuildingSourceBuckets) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device is not available in the current test environment";
  }

  auto target_a = std::make_shared<gtsam_points::iVox>(0.5);
  target_a->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target_a->set_neighbor_voxel_mode(1);
  target_a->insert(*make_cloud(false));

  std::vector<Eigen::Vector4d> shifted_points = {
    Eigen::Vector4d(0.3, 0.0, 0.0, 1.0),
    Eigen::Vector4d(1.3, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.3, 1.0, 0.0, 1.0),
    Eigen::Vector4d(0.8, 0.2, 0.0, 1.0),
  };
  std::vector<Eigen::Matrix3d> cov3(shifted_points.size(), Eigen::Matrix3d::Identity() * 1e-3);
  auto target_b_cloud = make_custom_cloud(shifted_points, cov3);
  auto target_b = std::make_shared<gtsam_points::iVox>(0.5);
  target_b->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target_b->set_neighbor_voxel_mode(1);
  target_b->insert(*target_b_cloud);

  const auto bucket_ctx = make_full_bucket_context();

  gtsam_points::StreamTempBufferRoundRobin stream_buffers;
  auto stream_buffer = stream_buffers.get_stream_buffer();
  iap::IntegratedBSplineGICPFactorGPU factor(
    bucket_ctx,
    target_a,
    make_cloud(true),
    stream_buffer.first,
    stream_buffer.second);
  factor.set_enable_profiling(true);
  factor.set_max_correspondence_distance(2.0);

  const auto source_staging_identity_a = factor.source_staging_identity();
  ASSERT_TRUE(static_cast<bool>(source_staging_identity_a));

  const auto values = make_identity_control_values();
  const double error_a = factor.error(values);
  const auto profile_a = factor.profiling_report();
  ASSERT_TRUE(profile_a.valid);
  ASSERT_EQ(profile_a.time_bucket_count, 1U);

  factor.refresh_target(target_b);
  const auto source_staging_identity_b = factor.source_staging_identity();
  ASSERT_EQ(source_staging_identity_a.get(), source_staging_identity_b.get());

  iap::IntegratedBSplineGICPFactorGPU fresh_factor(
    bucket_ctx,
    target_b,
    make_cloud(true),
    stream_buffer.first,
    stream_buffer.second);
  fresh_factor.set_enable_profiling(true);
  fresh_factor.set_max_correspondence_distance(2.0);

  const double error_b = factor.error(values);
  const auto profile_b = factor.profiling_report();
  const double fresh_error_b = fresh_factor.error(values);
  const auto fresh_profile_b = fresh_factor.profiling_report();
  ASSERT_TRUE(profile_b.valid);
  ASSERT_TRUE(fresh_profile_b.valid);
  EXPECT_EQ(profile_b.time_bucket_count, profile_a.time_bucket_count);
  EXPECT_EQ(profile_b.source_point_count, profile_a.source_point_count);
  EXPECT_NE(error_a, error_b);
  EXPECT_DOUBLE_EQ(error_b, fresh_error_b);
  EXPECT_EQ(profile_b.source_point_count, fresh_profile_b.source_point_count);
  EXPECT_EQ(profile_b.target_point_count, fresh_profile_b.target_point_count);
  EXPECT_EQ(profile_b.inlier_point_count, fresh_profile_b.inlier_point_count);
}

TEST(BSplineGICPFactorTest, GpuKernelFactorLinearizesAndReturnsUnifiedProfile) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device is not available in the current test environment";
  }

  auto target_cloud = make_cloud(false);
  auto target = std::make_shared<gtsam_points::iVox>(0.5);
  target->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target->set_neighbor_voxel_mode(1);
  target->insert(*target_cloud);

  std::array<gtsam::Key, iap::kBSplineControlPointCount> keys{};
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    keys[i] = iap::bspline_control_point_key(i);
  }

  gtsam_points::StreamTempBufferRoundRobin stream_buffers;
  auto stream_buffer = stream_buffers.get_stream_buffer();
  iap::IntegratedBSplineGICPFactorGPUKernel factor(
    keys,
    target,
    make_cloud(true),
    stream_buffer.first,
    stream_buffer.second);
  factor.set_jacobian_mode(iap::IntegratedBSplineGICPFactor::JacobianMode::SEMI_ANALYTIC);
  factor.set_numeric_eps(1e-4);
  factor.set_max_correspondence_distance(2.0);
  factor.set_correspondence_candidate_count(2);
  factor.set_correspondence_accept_ratio(0.99);
  factor.set_correspondence_min_score_gap(1e-6);
  factor.set_outlier_mahalanobis_threshold(10.0);
  factor.set_robust_kernel(iap::IntegratedBSplineGICPFactor::RobustKernel::HUBER, 1.0);
  factor.set_robust_weight_floor(0.0);
  factor.set_enable_profiling(true);

  const auto values = make_identity_control_values();
  const auto linear = factor.linearize(values);
  ASSERT_TRUE(static_cast<bool>(linear));

  const double factor_error = factor.error(values);
  const auto profile = factor.profiling_report();
  ASSERT_TRUE(profile.valid);
  EXPECT_EQ(profile.backend, iap::BSplineLidarFactorBackend::GPU_GICP);
  EXPECT_GE(profile.source_point_count, 4U);
  EXPECT_GE(profile.target_point_count, 4U);
  EXPECT_GE(profile.matched_point_count, 1U);
  EXPECT_GE(profile.inlier_point_count, 1U);
  EXPECT_GE(profile.correspondence_ms, 0.0);
  EXPECT_GE(profile.host_sync_ms, 0.0);
  EXPECT_GE(profile.total_ms, 0.0);

  const auto numeric_audit = factor.check_against_numeric_full(values, 1e-5);
  ASSERT_TRUE(numeric_audit.valid);
  EXPECT_LT(numeric_audit.rotation_rel_error, 1.0);
  EXPECT_LT(numeric_audit.translation_rel_error, 1.0);

  iap::IntegratedBSplineGICPFactor::DegeneracyThresholds thresholds;
  thresholds.min_match_ratio = 0.2;
  thresholds.min_inlier_ratio = 0.2;
  const auto degeneracy = factor.diagnose_degeneracy(thresholds);
  ASSERT_TRUE(degeneracy.valid);

  const auto result = factor.make_result(
    factor_error,
    factor.num_inliers(),
    factor.inlier_fraction(),
    &numeric_audit,
    &degeneracy);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.backend, iap::BSplineLidarFactorBackend::GPU_GICP);
  EXPECT_TRUE(result.profile.valid);
  EXPECT_TRUE(result.numeric_audit.valid);
}

TEST(BSplineGICPFactorTest, GpuKernelFactorRefreshesTargetWithoutRebuildingSourceStaging) {
  int device_count = 0;
  if (cudaGetDeviceCount(&device_count) != cudaSuccess || device_count == 0) {
    GTEST_SKIP() << "CUDA device is not available in the current test environment";
  }

  auto target_a = std::make_shared<gtsam_points::iVox>(0.5);
  target_a->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target_a->set_neighbor_voxel_mode(1);
  target_a->insert(*make_cloud(false));

  std::vector<Eigen::Vector4d> shifted_points = {
    Eigen::Vector4d(0.3, 0.0, 0.0, 1.0),
    Eigen::Vector4d(1.3, 0.0, 0.0, 1.0),
    Eigen::Vector4d(0.3, 1.0, 0.0, 1.0),
    Eigen::Vector4d(0.8, 0.2, 0.0, 1.0),
  };
  std::vector<Eigen::Matrix3d> cov3(shifted_points.size(), Eigen::Matrix3d::Identity() * 1e-3);
  auto target_b_cloud = make_custom_cloud(shifted_points, cov3);
  auto target_b = std::make_shared<gtsam_points::iVox>(0.5);
  target_b->voxel_insertion_setting().set_min_dist_in_cell(0.0);
  target_b->set_neighbor_voxel_mode(1);
  target_b->insert(*target_b_cloud);

  std::array<gtsam::Key, iap::kBSplineControlPointCount> keys{};
  for (std::size_t i = 0; i < iap::kBSplineControlPointCount; ++i) {
    keys[i] = iap::bspline_control_point_key(i);
  }

  gtsam_points::StreamTempBufferRoundRobin stream_buffers;
  auto stream_buffer = stream_buffers.get_stream_buffer();
  iap::IntegratedBSplineGICPFactorGPUKernel factor(
    keys,
    target_a,
    make_cloud(true),
    stream_buffer.first,
    stream_buffer.second);
  factor.set_enable_profiling(true);
  factor.set_max_correspondence_distance(2.0);

  const auto values = make_identity_control_values();
  const void* staging_a = factor.source_staging_identity();
  ASSERT_NE(staging_a, nullptr);
  const double error_a = factor.error(values);

  factor.refresh_target(target_b);
  const void* staging_b = factor.source_staging_identity();
  const double error_b = factor.error(values);

  EXPECT_EQ(staging_a, staging_b);
  EXPECT_NE(error_a, error_b);
}
#endif
