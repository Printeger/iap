#include <gtest/gtest.h>

#include <cmath>

#include <iap/integrity/local_navigation_integrity.hpp>

TEST(LocalNavigationIntegrityTest, RejectsGnssContaminatedSource) {
  iap::LocalNavigationSourceEvidence source;
  source.valid = true;
  source.source_contains_gnss = true;
  source.source_identity = "mixed-fgo-marginal";
  source.stamp_s = 10.0;
  source.estimation_frame_id = 7;
  source.model_identity = "imu-model-1";
  source.state_covariance =
      Eigen::Matrix<double, 15, 15>::Identity() * 0.01;
  source.current_lidar_hpl_m = 0.2;
  source.current_lidar_vpl_m = 0.2;
  source.icp_degenerate = false;

  iap::LocalNavigationPropagationModel model;
  model.valid = true;
  model.identity = "imu-model-1";
  model.accelerometer_noise_covariance = Eigen::Matrix3d::Identity() * 0.01;
  model.gyroscope_noise_covariance = Eigen::Matrix3d::Identity() * 0.01;
  model.integration_noise_covariance = Eigen::Matrix3d::Identity() * 1.0e-6;
  model.accelerometer_bias_random_walk_covariance =
      Eigen::Matrix3d::Identity() * 1.0e-8;
  model.gyroscope_bias_random_walk_covariance =
      Eigen::Matrix3d::Identity() * 1.0e-8;
  model.maximum_horizon_s = 2.0;
  model.coverage_multiplier = 6.0;

  const auto result = iap::evaluateLocalNavigationIntegrity(
      source, model, {0.0, 0.2, 0.4});

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "local_navigation_source_contains_gnss");
}

TEST(LocalNavigationIntegrityTest, ZeroOdometryCovarianceCannotAuthorize) {
  iap::LocalNavigationSourceEvidence source;
  source.valid = true;
  source.source_contains_gnss = false;
  source.icp_degenerate = false;
  source.source_identity = "zero-odom-covariance";
  source.stamp_s = 10.0;
  source.estimation_frame_id = 7;
  source.model_identity = "imu-model";
  source.current_lidar_hpl_m = 0.1;
  source.current_lidar_vpl_m = 0.1;

  iap::LocalNavigationPropagationModel model;
  model.valid = true;
  model.identity = "imu-model";
  model.maximum_horizon_s = 1.0;
  model.coverage_multiplier = 3.0;

  const auto result = iap::evaluateLocalNavigationIntegrity(
      source, model, {0.0});

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "local_navigation_source_numerically_invalid");
}

TEST(LocalNavigationIntegrityTest,
     PropagatesConfiguredImuNoiseOverExactPlannedOffsets) {
  iap::LocalNavigationSourceEvidence source;
  source.valid = true;
  source.source_contains_gnss = false;
  source.icp_degenerate = false;
  source.source_identity = "local-only-marginal";
  source.stamp_s = 10.0;
  source.estimation_frame_id = 7;
  source.model_identity = "worked-model";
  source.state_covariance =
      Eigen::Matrix<double, 15, 15>::Identity() * 0.01;
  source.current_lidar_hpl_m = 0.01;
  source.current_lidar_vpl_m = 0.01;

  iap::LocalNavigationPropagationModel model;
  model.valid = true;
  model.identity = "worked-model";
  model.accelerometer_noise_covariance = Eigen::Matrix3d::Identity() * 0.01;
  model.integration_noise_covariance = Eigen::Matrix3d::Identity() * 0.001;
  model.maximum_horizon_s = 1.0;
  model.coverage_multiplier = 2.0;

  const auto result = iap::evaluateLocalNavigationIntegrity(
      source, model, {0.0, 1.0});

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_DOUBLE_EQ(result.stamp_s, source.stamp_s);
  EXPECT_EQ(result.estimation_frame_id, source.estimation_frame_id);
  EXPECT_FALSE(result.source_contains_gnss);
  EXPECT_FALSE(result.icp_degenerate);
  EXPECT_DOUBLE_EQ(result.current_horizontal_bound_m,
                   source.current_lidar_hpl_m);
  ASSERT_EQ(result.samples.size(), 2u);
  EXPECT_DOUBLE_EQ(result.samples[0].horizontal_bound_m, 0.2);
  // Independent worked example: P_pp(1)=.01 position + .01 velocity +
  // .0025 accelerometer-bias coupling + .01/3 accelerometer noise + .001
  // integration noise.
  EXPECT_NEAR(result.samples[1].horizontal_bound_m,
              2.0 * std::sqrt(0.02683333333333333), 1.0e-12);
  EXPECT_GT(result.samples[1].horizontal_bound_m,
            result.samples[0].horizontal_bound_m);

  auto noisier_model = model;
  noisier_model.identity = "worked-model-noisier";
  source.model_identity = noisier_model.identity;
  noisier_model.accelerometer_noise_covariance *= 4.0;
  const auto noisier = iap::evaluateLocalNavigationIntegrity(
      source, noisier_model, {0.0, 1.0});
  ASSERT_TRUE(noisier.valid) << noisier.reason;
  EXPECT_GT(noisier.samples.back().horizontal_bound_m,
            result.samples.back().horizontal_bound_m);
}

TEST(LocalNavigationIntegrityTest, DegenerateIcpFailsClosed) {
  iap::LocalNavigationSourceEvidence source;
  source.valid = true;
  source.source_contains_gnss = false;
  source.icp_degenerate = true;
  source.source_identity = "degenerate-scan-match";
  source.stamp_s = 10.0;
  source.estimation_frame_id = 7;
  source.model_identity = "imu-model";
  source.state_covariance =
      Eigen::Matrix<double, 15, 15>::Identity() * 0.01;
  source.current_lidar_hpl_m = 0.1;
  source.current_lidar_vpl_m = 0.1;
  iap::LocalNavigationPropagationModel model;
  model.valid = true;
  model.identity = "imu-model";

  const auto result = iap::evaluateLocalNavigationIntegrity(
      source, model, {0.0});

  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "local_navigation_icp_degenerate");
}

TEST(LocalNavigationIntegrityTest, SyntheticTruthIsOnlyATestOracle) {
  iap::LocalNavigationSourceEvidence source;
  source.valid = true;
  source.source_contains_gnss = false;
  source.icp_degenerate = false;
  source.source_identity = "synthetic-local-source";
  source.stamp_s = 10.0;
  source.estimation_frame_id = 7;
  source.model_identity = "synthetic-model";
  source.state_covariance =
      Eigen::Matrix<double, 15, 15>::Identity() * 0.01;
  source.current_lidar_hpl_m = 0.1;
  source.current_lidar_vpl_m = 0.1;
  iap::LocalNavigationPropagationModel model;
  model.valid = true;
  model.identity = source.model_identity;
  model.maximum_horizon_s = 1.0;
  model.coverage_multiplier = 3.0;

  const auto result = iap::evaluateLocalNavigationIntegrity(
      source, model, {0.0});
  const Eigen::Vector3d synthetic_truth_error(0.12, 0.08, 0.20);

  ASSERT_TRUE(result.valid) << result.reason;
  EXPECT_LT(synthetic_truth_error.head<2>().norm(),
            result.samples.front().horizontal_bound_m);
  EXPECT_LT(std::abs(synthetic_truth_error.z()),
            result.samples.front().vertical_bound_m);
}
