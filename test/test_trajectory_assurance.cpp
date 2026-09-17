#include <gtest/gtest.h>

#include <iap/planner/trajectory_assurance.hpp>

namespace {

iap::LocalMotionEvidence clearCurrentFrameEvidence() {
  iap::LocalMotionEvidence evidence;
  evidence.complete = true;
  evidence.support_fresh = true;
  evidence.registration_health_valid = true;
  evidence.icp_degenerate = false;
  evidence.icp_rmse_m = 0.01;
  evidence.icp_gamma = 1.0;
  evidence.certified_empty_clearance_m = 5.0;
  evidence.identity = "local-evidence-1";
  return evidence;
}

std::vector<iap::LocalMotionCurve> shortCurve() {
  iap::LocalMotionCurve nominal;
  nominal.curve_id = "nominal";
  nominal.samples = {
      {0.0, Eigen::Vector3d(0.0, 0.0, 1.0), 0.05},
      {0.2, Eigen::Vector3d(0.2, 0.0, 1.0), 0.05},
      {0.4, Eigen::Vector3d(0.4, 0.0, 1.0), 0.05},
  };
  iap::LocalMotionCurve brake = nominal;
  brake.curve_id = "brake-0";
  brake.braking_curve = true;
  return {nominal, brake};
}

std::vector<iap::LocalMotionCurve> longHealthyRegisteredCurve() {
  iap::LocalMotionCurve nominal;
  nominal.curve_id = "nominal-long";
  for (int index = 0; index <= 43; ++index) {
    nominal.samples.push_back({
        0.2 * static_cast<double>(index),
        Eigen::Vector3d(0.0, 0.0, 1.0), 0.05});
  }
  iap::LocalMotionCurve brake;
  brake.curve_id = "brake-now";
  brake.braking_curve = true;
  brake.samples = {
      {0.0, Eigen::Vector3d(0.0, 0.0, 1.0), 0.05},
      {0.2, Eigen::Vector3d(0.0, 0.0, 1.0), 0.05},
  };
  return {nominal, brake};
}

std::vector<iap::GlobalNavigationExposureSample> slightVplExceedance() {
  return {
      {0.0, 10.0, 39.0, 20.0, 40.0, true},
      {0.2, 10.0, 40.74, 20.0, 40.0, true},
      {0.4, 10.0, 40.74, 20.0, 40.0, true},
      {0.6, 10.0, 38.0, 20.0, 40.0, true},
  };
}

TEST(TrajectoryAssuranceTest, BriefSlightVplExceedanceIsControlledDegraded) {
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = slightVplExceedance();
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance().evaluate(request);

  EXPECT_TRUE(result.authorized());
  EXPECT_EQ(result.mode,
            iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION);
  EXPECT_NEAR(result.global.peak_ratio, 1.0185, 1.0e-12);
  EXPECT_LT(result.global.exceedance_duration_s, 1.0);
  EXPECT_LT(result.global.exceedance_integral_ratio_s, 0.025);
  EXPECT_EQ(result.local.status, iap::LocalMotionAssuranceStatus::SAFE);
}

TEST(GlobalNavigationExposureTest, UsesGnssChannelInsteadOfFusedOrLidar) {
  iap::ForwardRiskPointResult point;
  point.ranking_state = iap::ForwardRiskRankingState::COMPARABLE;
  point.failure_reason = iap::ForwardRiskFailureReason::SAFETY_LIMIT_EXCEEDED;
  point.gnss_supported = true;
  point.prediction.gnss.valid = true;
  point.prediction.gnss.hpl = 10.0;
  point.prediction.gnss.vpl = 40.74;
  point.prediction.fused.hpl = 100.0;
  point.prediction.fused.vpl = 100.0;
  point.prediction.lidar.valid = true;

  const auto samples = iap::globalNavigationSamplesFromForwardRisk(
      {point}, {0.0}, 20.0, 40.0);

  ASSERT_EQ(samples.size(), 1u);
  EXPECT_TRUE(samples.front().complete);
  EXPECT_DOUBLE_EQ(samples.front().hpl_m, 10.0);
  EXPECT_DOUBLE_EQ(samples.front().vpl_m, 40.74);
  EXPECT_DOUBLE_EQ(samples.front().hal_m, 20.0);
  EXPECT_DOUBLE_EQ(samples.front().val_m, 40.0);
}

TEST(TrajectoryAssuranceTest, PeakAboveFivePercentRequiresRecovery) {
  auto global = slightVplExceedance();
  global[1].vpl_m = 42.01;
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = global;
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance().evaluate(request);

  EXPECT_FALSE(result.authorized());
  EXPECT_EQ(result.mode,
            iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
  EXPECT_EQ(result.reason, "global_navigation_budget_exceeded");
  EXPECT_TRUE(result.global.peak_ratio_exceeded);
  EXPECT_FALSE(result.global.continuous_exceedance_exceeded);
  EXPECT_FALSE(result.global.exceedance_integral_exceeded);
  EXPECT_FALSE(result.global.prior_episode_budget_exhausted);
  EXPECT_EQ(result.global.budget_failure_causes, "PEAK_RATIO");
}

TEST(GlobalNavigationExposureTest, HardGlobalRejectsAnyTaskLimitExceedance) {
  iap::GlobalNavigationExposurePolicy policy;
  policy.hard_global = true;
  const auto result =
      iap::GlobalNavigationExposureEvaluator(policy).evaluate(
          slightVplExceedance());
  EXPECT_TRUE(result.complete);
  EXPECT_FALSE(result.within_budget);
  EXPECT_EQ(result.reason, "global_navigation_budget_exceeded");
  EXPECT_TRUE(result.hard_global_exceedance);
  EXPECT_FALSE(result.peak_ratio_exceeded);
  EXPECT_EQ(result.budget_failure_causes, "HARD_GLOBAL_LIMIT");
}

TEST(GlobalNavigationExposureTest, DurationAndIntegralBudgetsAreIndependent) {
  auto sustained = slightVplExceedance();
  sustained = {
      {0.0, 10.0, 40.8, 20.0, 40.0, true},
      {0.6, 10.0, 40.8, 20.0, 40.0, true},
      {1.1, 10.0, 40.8, 20.0, 40.0, true},
  };
  const auto duration =
      iap::GlobalNavigationExposureEvaluator().evaluate(sustained);
  EXPECT_FALSE(duration.within_budget);
  EXPECT_GT(duration.maximum_continuous_exceedance_s, 1.0);
  EXPECT_FALSE(duration.peak_ratio_exceeded);
  EXPECT_TRUE(duration.continuous_exceedance_exceeded);
  EXPECT_FALSE(duration.exceedance_integral_exceeded);
  EXPECT_EQ(duration.budget_failure_causes, "CONTINUOUS_DURATION");

  const std::vector<iap::GlobalNavigationExposureSample> integral = {
      {0.0, 10.0, 42.0, 20.0, 40.0, true},
      {0.6, 10.0, 42.0, 20.0, 40.0, true},
  };
  const auto integrated =
      iap::GlobalNavigationExposureEvaluator().evaluate(integral);
  EXPECT_FALSE(integrated.within_budget);
  EXPECT_GT(integrated.exceedance_integral_ratio_s, 0.025);
  EXPECT_FALSE(integrated.peak_ratio_exceeded);
  EXPECT_FALSE(integrated.continuous_exceedance_exceeded);
  EXPECT_TRUE(integrated.exceedance_integral_exceeded);
  EXPECT_EQ(integrated.budget_failure_causes, "EXCESS_INTEGRAL");
}

TEST(TrajectoryAssuranceTest,
     PriorEpisodeExhaustionIsReportedSeparatelyFromCurrentCurve) {
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = {
      {0.0, 10.0, 39.0, 20.0, 40.0, true},
      {0.2, 10.0, 39.0, 20.0, 40.0, true},
  };
  request.has_prior_global_episode = true;
  request.prior_global_episode.active = true;
  request.prior_global_episode.budget_exhausted = true;
  request.prior_global_episode.peak_ratio = 1.02;
  request.prior_global_episode.continuous_exceedance_s = 0.4;
  request.prior_global_episode.exceedance_integral_ratio_s = 0.01;
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance().evaluate(request);

  EXPECT_FALSE(result.authorized());
  EXPECT_TRUE(result.global.prior_episode_budget_exhausted);
  EXPECT_FALSE(result.global.peak_ratio_exceeded);
  EXPECT_FALSE(result.global.continuous_exceedance_exceeded);
  EXPECT_FALSE(result.global.exceedance_integral_exceeded);
  EXPECT_EQ(result.global.budget_failure_causes,
            "PRIOR_EPISODE_EXHAUSTED");
}

TEST(GlobalNavigationExposureTest,
     NewlyExhaustedEpisodeIsNotMislabeledAsPreviouslyExhausted) {
  iap::GlobalNavigationExposureResult result;
  result.peak_ratio = 1.01;
  result.maximum_continuous_exceedance_s = 1.1;
  result.exceedance_integral_ratio_s = 0.011;

  iap::annotateGlobalNavigationBudgetFailures(&result, {}, false);

  EXPECT_TRUE(result.continuous_exceedance_exceeded);
  EXPECT_FALSE(result.prior_episode_budget_exhausted);
  EXPECT_EQ(result.budget_failure_causes, "CONTINUOUS_DURATION");

  iap::annotateGlobalNavigationBudgetFailures(&result, {}, true);
  EXPECT_TRUE(result.prior_episode_budget_exhausted);
  EXPECT_EQ(result.budget_failure_causes,
            "CONTINUOUS_DURATION|PRIOR_EPISODE_EXHAUSTED");
}

TEST(GlobalNavigationExposureTest,
     DownCrossingCountsContinuousExposureBeforeReset) {
  const std::vector<iap::GlobalNavigationExposureSample> samples = {
      {0.0, 20.2, 10.0, 20.0, 40.0, true},
      {0.8, 20.2, 10.0, 20.0, 40.0, true},
      {1.2, 19.8, 10.0, 20.0, 40.0, true},
  };
  const auto result =
      iap::GlobalNavigationExposureEvaluator().evaluate(samples);
  EXPECT_NEAR(result.maximum_continuous_exceedance_s, 1.0, 1.0e-12);
}

TEST(GlobalNavigationExposureTest,
     PredictedRecoveryRequiresSustainedNinetyFivePercentMargin) {
  const std::vector<iap::GlobalNavigationExposureSample> shallow = {
      {0.0, 20.2, 10.0, 20.0, 40.0, true},
      {0.2, 19.9, 10.0, 20.0, 40.0, true},
      {0.8, 19.8, 10.0, 20.0, 40.0, true},
  };
  EXPECT_FALSE(iap::GlobalNavigationExposureEvaluator()
                   .evaluate(shallow).recovery_predicted);
  const std::vector<iap::GlobalNavigationExposureSample> recovered = {
      {0.0, 20.2, 10.0, 20.0, 40.0, true},
      {0.2, 18.9, 10.0, 20.0, 40.0, true},
      {0.7, 18.8, 10.0, 20.0, 40.0, true},
  };
  EXPECT_TRUE(iap::GlobalNavigationExposureEvaluator()
                  .evaluate(recovered).recovery_predicted);
}

TEST(LocalMotionAssuranceTest, CurrentFrameCommonTransformDoesNotAddGlobalPl) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 1.5, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  evidence.obstacles.push_back(obstacle);

  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, shortCurve());

  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::SAFE);
  EXPECT_GT(result.minimum_margin_m, 0.0);
  EXPECT_LT(result.maximum_required_envelope_m, 1.0);
}

TEST(LocalMotionAssuranceTest,
     HealthyRegisteredObstacleDoesNotChargeAbsolutePlOrSourceIcpAgain) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  // The closest surface is 3.957 m away, matching the live regression that
  // was rejected by a fabricated 5.146 m current+source absolute-PL term.
  obstacle.center_map = Eigen::Vector3d(0.0, 4.007, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance =
      iap::LocalObstacleProvenance::ACTIVE_WINDOW_CERTIFIED;
  obstacle.source_frame_id = 7;
  obstacle.source_identity = "frame-7";
  evidence.obstacles.push_back(obstacle);

  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, shortCurve());

  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::SAFE);
  EXPECT_GT(result.minimum_margin_m, 3.0);
  EXPECT_LT(result.maximum_required_envelope_m, 1.0);
}

TEST(LocalMotionAssuranceTest,
     HealthyRegisteredSlamDoesNotInventTimeLinearDrift) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  // Surface clearance is 0.90 m. The measured/local envelope is 0.622 m;
  // extending the same healthy registered geometry to 8.6 s must not add an
  // uncalibrated 0.86 m error that the SLAM producer never reported.
  obstacle.center_map = Eigen::Vector3d(0.0, 0.95, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance =
      iap::LocalObstacleProvenance::ACTIVE_WINDOW_CERTIFIED;
  obstacle.source_frame_id = 9;
  obstacle.source_identity = "frame-9";
  evidence.obstacles.push_back(obstacle);

  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, longHealthyRegisteredCurve());

  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::SAFE)
      << result.reason << " curve=" << result.first_failure.curve_id
      << " sample=" << result.first_failure.sample_index
      << " clearance=" << result.first_failure.obstacle_clearance_m
      << " envelope=" << result.first_failure.required_envelope_m
      << " drift=" << result.first_failure.drift_error_m;
  EXPECT_NEAR(result.minimum_margin_m, 0.278, 1.0e-12);
  EXPECT_NEAR(result.maximum_required_envelope_m, 0.622, 1.0e-12);
}

TEST(LocalMotionAssuranceTest, RegistrationAndSupportFailuresRemainFailClosed) {
  auto registration_failure = clearCurrentFrameEvidence();
  registration_failure.icp_degenerate = true;
  const auto invalid_registration = iap::LocalMotionAssurance().evaluate(
      registration_failure, shortCurve());
  EXPECT_EQ(invalid_registration.status,
            iap::LocalMotionAssuranceStatus::UNKNOWN);
  EXPECT_EQ(invalid_registration.reason, "slam_registration_health_invalid");

  auto stale_support = clearCurrentFrameEvidence();
  stale_support.support_fresh = false;
  const auto invalid_support = iap::LocalMotionAssurance().evaluate(
      stale_support, shortCurve());
  EXPECT_EQ(invalid_support.status, iap::LocalMotionAssuranceStatus::UNKNOWN);
  EXPECT_EQ(invalid_support.reason, "local_motion_evidence_incomplete");
}

TEST(LocalMotionAssuranceTest, UnsafeCertifiedBrakeStillFailsClosed) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 0.0, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  evidence.obstacles.push_back(obstacle);

  auto curves = shortCurve();
  for (auto& sample : curves.front().samples) {
    sample.position_map.y() = 2.0;
  }
  const auto result = iap::LocalMotionAssurance().evaluate(evidence, curves);

  ASSERT_EQ(result.status, iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_EQ(result.first_failure.curve_id, "brake-0");
  EXPECT_EQ(result.reason, "hard_collision");
}

TEST(LocalMotionAssuranceTest, IcpResidualDoesNotReuseGnssAraimMultiplier) {
  auto evidence = clearCurrentFrameEvidence();
  evidence.icp_rmse_m = 0.25;
  evidence.icp_gamma = 1.0;
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 1.9, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  evidence.obstacles.push_back(obstacle);

  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, shortCurve());

  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::SAFE);
  EXPECT_NEAR(result.first_failure.scan_error_m, 0.0, 1.0e-12);
  EXPECT_LT(result.maximum_required_envelope_m, 1.1);
}

TEST(LocalMotionAssuranceTest,
     CertifiedTrackingBoundIsDistinctFromLossOfControlThreshold) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 1.3, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  evidence.obstacles.push_back(obstacle);

  auto bounded = shortCurve();
  for (auto& curve : bounded) {
    for (auto& sample : curve.samples) sample.tracking_error_m = 0.15;
  }
  const auto safe = iap::LocalMotionAssurance().evaluate(evidence, bounded);
  EXPECT_EQ(safe.status, iap::LocalMotionAssuranceStatus::SAFE);

  auto loss_limit_as_bound = bounded;
  for (auto& curve : loss_limit_as_bound) {
    for (auto& sample : curve.samples) sample.tracking_error_m = 0.75;
  }
  const auto unnecessarily_blocked =
      iap::LocalMotionAssurance().evaluate(evidence, loss_limit_as_bound);
  EXPECT_EQ(unnecessarily_blocked.status,
            iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_EQ(unnecessarily_blocked.reason,
            "local_clearance_margin_not_positive");
}

TEST(LocalMotionAssuranceTest, UncertifiedOldMapObstacleFailsClosed) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 1.5, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance =
      iap::LocalObstacleProvenance::ACTIVE_WINDOW_UNCERTIFIED;
  evidence.obstacles.push_back(obstacle);

  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, shortCurve());

  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::UNKNOWN);
  EXPECT_EQ(result.reason, "active_window_source_health_missing");
}

TEST(LocalMotionAssuranceTest, CertifiedOldMapObstacleUsesCommonEnvelope) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 1.5, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance =
      iap::LocalObstacleProvenance::ACTIVE_WINDOW_CERTIFIED;
  obstacle.source_frame_id = 7;
  obstacle.source_identity = "frame-7";
  evidence.obstacles.push_back(obstacle);

  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, shortCurve());

  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::SAFE);
  EXPECT_DOUBLE_EQ(result.first_failure.relative_map_error_m, 0.0);
  EXPECT_LT(result.maximum_required_envelope_m, 1.0);
}

TEST(LocalMotionAssuranceTest,
     NearestGeometryControlsMarginAcrossCertifiedSources) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence current;
  current.center_map = Eigen::Vector3d(0.0, 0.65, 1.0);
  current.half_extent_m = Eigen::Vector3d::Constant(0.05);
  current.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  evidence.obstacles.push_back(current);
  iap::LocalObstacleEvidence old;
  old.center_map = Eigen::Vector3d(0.0, 2.0, 1.0);
  old.half_extent_m = Eigen::Vector3d::Constant(0.05);
  old.provenance = iap::LocalObstacleProvenance::ACTIVE_WINDOW_CERTIFIED;
  old.source_frame_id = 8;
  old.source_identity = "frame-8";
  evidence.obstacles.push_back(old);

  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, shortCurve());

  ASSERT_EQ(result.status, iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_EQ(result.first_failure.provenance,
            iap::LocalObstacleProvenance::CURRENT_FRAME);
  EXPECT_DOUBLE_EQ(result.first_failure.relative_map_error_m, 0.0);
}

TEST(LocalMotionAssuranceTest, InvalidObstacleGeometryFailsClosed) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  evidence.obstacles.push_back(obstacle);
  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, shortCurve());
  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::UNKNOWN);
  EXPECT_EQ(result.reason, "local_obstacle_geometry_invalid");
}

TEST(LocalMotionAssuranceTest, CertifiedSourceRequiresIdentity) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 1.5, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance = iap::LocalObstacleProvenance::ACTIVE_WINDOW_CERTIFIED;
  evidence.obstacles.push_back(obstacle);
  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, shortCurve());
  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::UNKNOWN);
  EXPECT_EQ(result.reason, "active_window_source_identity_missing");
}

TEST(GlobalNavigationExposureLedgerTest, ReplanningDoesNotResetEpisode) {
  iap::GlobalNavigationExposureLedger ledger;
  EXPECT_TRUE(ledger.update(0.0, 1.02, "epoch-a"));
  EXPECT_TRUE(ledger.update(0.4, 1.02, "epoch-b"));
  EXPECT_FALSE(ledger.update(0.4, 1.02, "epoch-b"));
  EXPECT_TRUE(ledger.update(0.8, 1.02, "epoch-c"));

  const auto before_replan = ledger.state();
  EXPECT_TRUE(before_replan.active);
  EXPECT_NEAR(before_replan.continuous_exceedance_s, 0.8, 1.0e-12);

  ledger.noteTrajectoryReplacement(99);
  EXPECT_TRUE(ledger.update(1.1, 1.02, "epoch-d"));
  EXPECT_TRUE(ledger.state().budget_exhausted);
  EXPECT_GT(ledger.state().continuous_exceedance_s, 1.0);
}

TEST(TrajectoryAssuranceTest, ReplanningConsumesRemainingEpisodeBudget) {
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = slightVplExceedance();
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;
  request.has_prior_global_episode = true;
  request.prior_global_episode.active = true;
  request.prior_global_episode.peak_ratio = 1.02;
  request.prior_global_episode.continuous_exceedance_s = 0.4;
  request.prior_global_episode.exceedance_integral_ratio_s = 0.021;

  const auto result = iap::TrajectoryAssurance().evaluate(request);
  EXPECT_FALSE(result.authorized());
  EXPECT_GT(result.global.exceedance_integral_ratio_s, 0.025);
  EXPECT_EQ(result.global.reason,
            "global_navigation_episode_budget_exceeded");
}

TEST(GlobalNavigationExposureLedgerTest,
     EpisodeEndsOnlyAfterSustainedRecoveryHysteresis) {
  iap::GlobalNavigationExposureLedger ledger;
  ASSERT_TRUE(ledger.update(0.0, 1.01, "unsafe-a"));
  ASSERT_TRUE(ledger.update(0.2, 0.94, "safe-a"));
  EXPECT_TRUE(ledger.state().active);
  ASSERT_TRUE(ledger.update(0.55, 0.94, "safe-b"));
  EXPECT_TRUE(ledger.state().active);
  ASSERT_TRUE(ledger.update(0.75, 0.94, "safe-c"));
  EXPECT_FALSE(ledger.state().active);
  EXPECT_FALSE(ledger.state().budget_exhausted);
}

TEST(GlobalNavigationExposureLedgerTest,
     DescendingThresholdCrossingConsumesExposureBudget) {
  iap::GlobalNavigationExposureLedger ledger;
  ASSERT_TRUE(ledger.update(0.0, 1.02, "unsafe"));
  ASSERT_TRUE(ledger.update(1.0, 0.98, "below-al"));
  EXPECT_TRUE(ledger.state().active);
  EXPECT_NEAR(ledger.state().continuous_exceedance_s, 0.5, 1.0e-12);
  EXPECT_NEAR(ledger.state().exceedance_integral_ratio_s, 0.005, 1.0e-12);
}

TEST(TrajectoryAssuranceTest,
     PredictedRecoveryCannotClearAnExhaustedRuntimeEpisode) {
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = {
      {0.0, 10.0, 38.0, 20.0, 40.0, true},
      {0.5, 10.0, 38.0, 20.0, 40.0, true},
      {1.0, 10.0, 38.0, 20.0, 40.0, true}};
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;
  request.has_prior_global_episode = true;
  request.prior_global_episode.active = true;
  request.prior_global_episode.budget_exhausted = true;
  request.prior_global_episode.peak_ratio = 1.06;
  request.prior_global_episode.continuous_exceedance_s = 1.1;
  request.prior_global_episode.exceedance_integral_ratio_s = 0.03;

  const auto result = iap::TrajectoryAssurance().evaluate(request);
  EXPECT_FALSE(result.authorized());
  EXPECT_FALSE(result.global.within_budget);
  EXPECT_EQ(result.global.reason,
            "global_navigation_episode_budget_exceeded");
}

TEST(TrajectoryAssuranceTest, WiderControlledRouteCanBeatNarrowNormalRoute) {
  iap::TrajectoryAssuranceResult narrow;
  narrow.mode = iap::TrajectoryExecutionMode::NORMAL_EXECUTION;
  narrow.local.status = iap::LocalMotionAssuranceStatus::SAFE;
  narrow.local.maximum_clearance_utilization = 0.92;
  narrow.worst_budget_utilization = 0.92;
  narrow.mission_progress_m = 4.0;

  iap::TrajectoryAssuranceResult wide;
  wide.mode =
      iap::TrajectoryExecutionMode::CONTROLLED_DEGRADED_EXECUTION;
  wide.local.status = iap::LocalMotionAssuranceStatus::SAFE;
  wide.global.peak_budget_utilization = 0.37;
  wide.local.maximum_clearance_utilization = 0.40;
  wide.worst_budget_utilization = 0.40;
  wide.mission_progress_m = 4.0;

  EXPECT_TRUE(iap::TrajectoryAssurance::prefer(wide, narrow));
  EXPECT_FALSE(iap::TrajectoryAssurance::prefer(narrow, wide));
}

}  // namespace
