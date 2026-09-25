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

TEST(TrajectoryAssuranceTest, PeakAboveFivePercentExhaustsTheMissionBudget) {
  auto global = slightVplExceedance();
  global[1].vpl_m = 42.01;
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = global;
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance().evaluate(request);

  EXPECT_FALSE(result.authorized());
  EXPECT_EQ(result.mode, iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
  EXPECT_EQ(result.reason, "global_navigation_budget_exceeded");
  EXPECT_TRUE(result.global.peak_ratio_exceeded);
  EXPECT_FALSE(result.global.continuous_exceedance_exceeded);
  EXPECT_FALSE(result.global.exceedance_integral_exceeded);
  EXPECT_FALSE(result.global.prior_episode_budget_exhausted);
  EXPECT_EQ(result.global.budget_failure_causes, "PEAK_RATIO");
}

TEST(TrajectoryAssuranceTest,
     BestEffortRejectsLocallySafeCurveWhenExposureBudgetIsExhausted) {
  auto global = slightVplExceedance();
  global[1].vpl_m = 48.0;
  iap::GlobalNavigationExposurePolicy policy;
  policy.task_mode = iap::GlobalNavigationTaskMode::MISSION_BEST_EFFORT;
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = global;
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance(policy).evaluate(request);

  EXPECT_FALSE(result.authorized());
  EXPECT_FALSE(result.global.within_budget);
  EXPECT_EQ(result.mode, iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
  EXPECT_EQ(result.reason, "global_navigation_budget_exceeded");
}

TEST(TrajectoryAssuranceTest,
     IncompleteGnssUsesWholeBoundedSegmentConservativeCharge) {
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = {
      {0.0, NAN, NAN, 20.0, 40.0, false},
      {0.4, NAN, NAN, 20.0, 40.0, false}};
  request.conservative_incomplete_global_navigation = true;
  request.committed_duration_s = 0.4;
  request.global_evidence_identity = "incomplete-curve-a";
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance().evaluate(request);

  EXPECT_TRUE(result.authorized()) << result.reason;
  EXPECT_EQ(result.mode,
            iap::TrajectoryExecutionMode::MISSION_DEGRADED_EXECUTION);
  EXPECT_TRUE(result.conservative_global_charge_applied);
  EXPECT_DOUBLE_EQ(result.conservative_global_charge_ratio, 1.05);
  EXPECT_DOUBLE_EQ(result.conservative_global_charge_duration_s, 0.4);
  EXPECT_NEAR(result.global.maximum_continuous_exceedance_s, 0.4, 1.0e-12);
  EXPECT_NEAR(result.global.exceedance_integral_ratio_s, 0.02, 1.0e-12);
  EXPECT_TRUE(result.global.within_budget);
}

TEST(TrajectoryAssuranceTest,
     IncompleteGnssRejectsWhenWholeSegmentChargeExceedsBudget) {
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = {
      {0.0, NAN, NAN, 20.0, 40.0, false},
      {0.6, NAN, NAN, 20.0, 40.0, false}};
  request.conservative_incomplete_global_navigation = true;
  request.committed_duration_s = 0.6;
  request.global_evidence_identity = "incomplete-curve-b";
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance().evaluate(request);

  EXPECT_FALSE(result.authorized());
  EXPECT_EQ(result.mode, iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
  EXPECT_EQ(result.reason, "global_navigation_exposure_budget_exhausted");
  EXPECT_TRUE(result.global.exceedance_integral_exceeded);
}

TEST(TrajectoryAssuranceTest,
     RepeatedIncompleteEvidenceIdentityIsNotChargedTwice) {
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = {
      {0.0, NAN, NAN, 20.0, 40.0, false},
      {0.4, NAN, NAN, 20.0, 40.0, false}};
  request.conservative_incomplete_global_navigation = true;
  request.committed_duration_s = 0.4;
  request.global_evidence_identity = "same-incomplete-evidence";
  request.has_prior_global_episode = true;
  request.prior_global_episode.active = true;
  request.prior_global_episode.peak_ratio = 1.05;
  request.prior_global_episode.current_continuous_exceedance_s = 0.2;
  request.prior_global_episode.continuous_exceedance_s = 0.2;
  request.prior_global_episode.exceedance_integral_ratio_s = 0.01;
  request.prior_global_episode.last_evidence_identity =
      request.global_evidence_identity;
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance().evaluate(request);

  EXPECT_TRUE(result.authorized()) << result.reason;
  EXPECT_FALSE(result.conservative_global_charge_applied);
  EXPECT_NEAR(result.global.maximum_continuous_exceedance_s, 0.2, 1.0e-12);
  EXPECT_NEAR(result.global.exceedance_integral_ratio_s, 0.01, 1.0e-12);
}

TEST(TrajectoryAssuranceTest,
     StrictGlobalStillRejectsAnyGlobalLimitExceedance) {
  auto global = slightVplExceedance();
  iap::GlobalNavigationExposurePolicy policy;
  policy.task_mode = iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
  iap::TrajectoryAssuranceRequest request;
  request.global_samples = global;
  request.local_evidence = clearCurrentFrameEvidence();
  request.local_curves = shortCurve();
  request.certified_braking_available = true;

  const auto result = iap::TrajectoryAssurance(policy).evaluate(request);

  EXPECT_FALSE(result.authorized());
  EXPECT_EQ(result.mode, iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
  EXPECT_TRUE(result.global.hard_global_exceedance);
}

TEST(GlobalNavigationExposureTest, HardGlobalRejectsAnyTaskLimitExceedance) {
  iap::GlobalNavigationExposurePolicy policy;
  policy.task_mode = iap::GlobalNavigationTaskMode::STRICT_GLOBAL;
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
  EXPECT_EQ(result.mode, iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
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

TEST(LocalMotionAssuranceTest, IcpResidualIsHealthOnlyNotEnvelopeDistance) {
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
  EXPECT_NEAR(result.maximum_required_envelope_m, 0.622, 1.0e-12);
  EXPECT_GT(result.minimum_margin_m, 1.0);
}

TEST(LocalClearanceEvaluatorTest,
     HealthyIcpResidualDoesNotChangeCalibratedSurfaceEnvelope) {
  auto low_residual = clearCurrentFrameEvidence();
  low_residual.icp_rmse_m = 0.02;
  auto high_residual = low_residual;
  high_residual.icp_rmse_m = 0.30;

  iap::LocalMotionAssurancePolicy policy;
  policy.surface_error_bound_m = 0.04;
  policy.surface_error_calibration_id = "forest-held-out-v1";
  const auto low = iap::LocalMotionAssurance(policy).evaluate(
      low_residual, shortCurve());
  const auto high = iap::LocalMotionAssurance(policy).evaluate(
      high_residual, shortCurve());

  ASSERT_EQ(low.status, iap::LocalMotionAssuranceStatus::SAFE);
  ASSERT_EQ(high.status, iap::LocalMotionAssuranceStatus::SAFE);
  EXPECT_DOUBLE_EQ(low.maximum_required_envelope_m,
                   high.maximum_required_envelope_m);
  EXPECT_NEAR(low.maximum_required_envelope_m, 0.642, 1.0e-12);
  EXPECT_DOUBLE_EQ(low.surface_error_bound_m, 0.04);
  EXPECT_DOUBLE_EQ(high.surface_error_bound_m, 0.04);
  EXPECT_EQ(high.surface_error_calibration_id, "forest-held-out-v1");
  EXPECT_DOUBLE_EQ(high.raw_icp_rmse_m, 0.30);
  EXPECT_NE(low.certificate_hash, high.certificate_hash)
      << "raw ICP remains diagnostic certificate evidence even though it "
         "does not inflate clearance";
}

TEST(LocalClearanceEvaluatorTest,
     ReturnsSharedEnvelopeMarginAndEscapeDirection) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 1.0, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  obstacle.source_frame_id = 12;
  obstacle.source_identity = "current-12";
  evidence.obstacles.push_back(obstacle);

  iap::LocalMotionAssurancePolicy policy;
  policy.surface_error_bound_m = 0.04;
  policy.surface_error_calibration_id = "forest-held-out-v1";
  const iap::LocalClearanceEvaluator evaluator(evidence, policy);
  const auto result = evaluator.query(
      Eigen::Vector3d(0.0, 0.0, 1.0), 0.15, 0.05);

  ASSERT_EQ(result.status, iap::LocalClearanceStatus::VALID);
  EXPECT_NEAR(result.obstacle_clearance_m, 0.95, 1.0e-12);
  EXPECT_NEAR(result.required_envelope_m, 0.742, 1.0e-12);
  EXPECT_NEAR(result.planning_required_envelope_m, 0.792, 1.0e-12);
  EXPECT_NEAR(result.signed_margin_m, 0.158, 1.0e-12);
  EXPECT_TRUE(result.nearest_obstacle_position_map.isApprox(
      Eigen::Vector3d(0.0, 1.0, 1.0)));
  EXPECT_TRUE(result.escape_direction_map.isApprox(
      Eigen::Vector3d(0.0, -1.0, 0.0)));
  EXPECT_EQ(result.nearest_obstacle_identity, "current-12");
  EXPECT_DOUBLE_EQ(result.raw_icp_rmse_m, 0.01);
  EXPECT_DOUBLE_EQ(result.surface_error_bound_m, 0.04);
}

TEST(LocalClearanceEvaluatorTest,
     FarObstacleReturnsExplicitCertifiedClearanceCap) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 3.0, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Constant(0.05);
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  obstacle.source_frame_id = 21;
  obstacle.source_identity = "surface-at-three-metres";
  evidence.obstacles.push_back(obstacle);

  const iap::LocalClearanceEvaluator evaluator(evidence);
  const auto result = evaluator.query(
      Eigen::Vector3d(0.0, 0.0, 1.0), 0.05, 0.05);

  ASSERT_EQ(result.status, iap::LocalClearanceStatus::VALID);
  EXPECT_DOUBLE_EQ(result.obstacle_clearance_m, 5.0);
  EXPECT_TRUE(result.obstacle_clearance_capped);
  EXPECT_TRUE(result.nearest_obstacle_identity.empty());
}

TEST(LocalMotionAssuranceTest,
     GenerationBufferRejectsSplineThatOnlyPassesHardBoundary) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 0.65, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Zero();
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  obstacle.source_frame_id = 12;
  obstacle.source_identity = "current-12";
  evidence.obstacles.push_back(obstacle);

  iap::LocalMotionAssurancePolicy policy;
  policy.surface_error_bound_m = 0.02;
  policy.surface_error_calibration_id = "test-bound-v1";
  const iap::LocalMotionAssurance assurance(policy);

  const auto runtime = assurance.evaluate(
      evidence, longHealthyRegisteredCurve(), 0.0);
  const auto generation = assurance.evaluate(
      evidence, longHealthyRegisteredCurve(), 0.05);

  ASSERT_EQ(runtime.status, iap::LocalMotionAssuranceStatus::SAFE);
  EXPECT_NEAR(runtime.minimum_margin_m, 0.028, 1.0e-12);
  ASSERT_EQ(generation.status, iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_EQ(generation.reason, "local_clearance_margin_not_positive");
  EXPECT_NEAR(generation.first_failure.margin_m, -0.022, 1.0e-12);
  EXPECT_DOUBLE_EQ(generation.planning_buffer_m, 0.05);
}

TEST(LocalMotionAssuranceTest,
     InitialClearanceRecoveryRegainsPlanningBufferWithinBound) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 0.65, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Zero();
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  obstacle.source_frame_id = 12;
  obstacle.source_identity = "recovery-tree";
  evidence.obstacles.push_back(obstacle);

  iap::LocalMotionCurve nominal;
  nominal.curve_id = "recovery-nominal";
  nominal.samples = {
      {0.0, Eigen::Vector3d(0.0, 0.0, 1.0), 0.05},
      {0.2, Eigen::Vector3d(0.0, -0.03, 1.0), 0.05},
      {0.4, Eigen::Vector3d(0.0, -0.06, 1.0), 0.05},
  };
  iap::LocalMotionCurve early_brake;
  early_brake.curve_id = "brake-early";
  early_brake.braking_curve = true;
  early_brake.samples = {
      {0.0, Eigen::Vector3d(0.0, 0.0, 1.0), 0.05},
      {0.2, Eigen::Vector3d(0.0, -0.01, 1.0), 0.05},
  };
  iap::LocalMotionCurve late_brake;
  late_brake.curve_id = "brake-late";
  late_brake.braking_curve = true;
  late_brake.samples = {
      {0.4, Eigen::Vector3d(0.0, -0.06, 1.0), 0.05},
      {0.6, Eigen::Vector3d(0.0, -0.06, 1.0), 0.05},
  };
  iap::LocalMotionInitialClearanceRecovery recovery;
  recovery.enabled = true;
  recovery.maximum_transition_duration_s = 0.4;

  const auto result = iap::LocalMotionAssurance().evaluate(
      evidence, {nominal, early_brake, late_brake}, 0.05, recovery);

  EXPECT_EQ(result.status, iap::LocalMotionAssuranceStatus::SAFE)
      << result.reason;
  EXPECT_TRUE(result.initial_clearance_recovery_complete);
  EXPECT_NEAR(result.initial_clearance_recovery_time_s, 0.2, 1.0e-12);
  EXPECT_LT(result.minimum_margin_m, 0.0);
  EXPECT_GT(result.minimum_hard_margin_m, 0.0);
  EXPECT_LT(result.minimum_hard_margin_m, result.minimum_margin_m + 0.051);
}

TEST(LocalMotionAssuranceTest,
     InitialClearanceRecoveryRejectsHardCollisionAndNonImprovingExit) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 0.65, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Zero();
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  evidence.obstacles.push_back(obstacle);
  auto curves = shortCurve();
  for (auto& curve : curves) {
    curve.samples = {
        {0.0, Eigen::Vector3d(0.0, 0.0, 1.0), 0.05},
        {0.2, Eigen::Vector3d(0.0, 0.02, 1.0), 0.05},
        {0.4, Eigen::Vector3d(0.0, 0.03, 1.0), 0.05},
    };
  }
  iap::LocalMotionInitialClearanceRecovery recovery;
  recovery.enabled = true;
  recovery.maximum_transition_duration_s = 0.4;

  const auto non_improving = iap::LocalMotionAssurance().evaluate(
      evidence, curves, 0.05, recovery);
  EXPECT_EQ(non_improving.status, iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_EQ(non_improving.reason,
            "initial_clearance_recovery_not_improving");

  obstacle.center_map.y() = 0.60;
  evidence.obstacles = {obstacle};
  const auto hard_collision = iap::LocalMotionAssurance().evaluate(
      evidence, curves, 0.05, recovery);
  EXPECT_EQ(hard_collision.status, iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_EQ(hard_collision.reason, "initial_clearance_recovery_hard_unsafe");
}

TEST(LocalMotionAssuranceTest,
     InitialClearanceRecoveryRejectsLateOrIncompleteTransition) {
  auto evidence = clearCurrentFrameEvidence();
  iap::LocalObstacleEvidence obstacle;
  obstacle.center_map = Eigen::Vector3d(0.0, 0.65, 1.0);
  obstacle.half_extent_m = Eigen::Vector3d::Zero();
  obstacle.provenance = iap::LocalObstacleProvenance::CURRENT_FRAME;
  evidence.obstacles.push_back(obstacle);
  auto curves = shortCurve();
  for (auto& curve : curves) {
    curve.samples = {
        {0.0, Eigen::Vector3d(0.0, 0.0, 1.0), 0.05},
        {0.2, Eigen::Vector3d(0.0, -0.01, 1.0), 0.05},
        {0.4, Eigen::Vector3d(0.0, -0.03, 1.0), 0.05},
    };
  }
  iap::LocalMotionInitialClearanceRecovery recovery;
  recovery.enabled = true;
  recovery.maximum_transition_duration_s = 0.1;

  const auto late = iap::LocalMotionAssurance().evaluate(
      evidence, curves, 0.05, recovery);
  EXPECT_EQ(late.status, iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_EQ(late.reason, "initial_clearance_recovery_deadline_exceeded");

  recovery.maximum_transition_duration_s = 0.5;
  for (auto& curve : curves) {
    curve.samples.back().position_map.y() = -0.015;
  }
  const auto incomplete = iap::LocalMotionAssurance().evaluate(
      evidence, curves, 0.05, recovery);
  EXPECT_EQ(incomplete.status, iap::LocalMotionAssuranceStatus::UNSAFE);
  EXPECT_EQ(incomplete.reason, "initial_clearance_recovery_incomplete");
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
  EXPECT_EQ(result.mode, iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
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
  EXPECT_EQ(result.mode, iap::TrajectoryExecutionMode::RECOVERY_OR_EXIT);
  EXPECT_FALSE(result.global.within_budget);
  EXPECT_EQ(result.global.reason,
            "global_navigation_episode_budget_exceeded");
}

TEST(TrajectoryAssuranceTest, FormalRoutePrecedesDegradedRoute) {
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

  EXPECT_FALSE(iap::TrajectoryAssurance::prefer(wide, narrow));
  EXPECT_TRUE(iap::TrajectoryAssurance::prefer(narrow, wide));
}

}  // namespace
