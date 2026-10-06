#include <gtest/gtest.h>

#include <plan_env/registered_lidar_window.h>

#include <cstdlib>
#include <unistd.h>

namespace {

RegisteredLidarFrameData frame(
    const std::int64_t id, const Eigen::Vector3d& origin,
    std::initializer_list<Eigen::Vector3d> hits) {
  RegisteredLidarFrameData value;
  value.frame_id = id;
  value.stamp_s = static_cast<double>(id);
  value.scan_end_stamp_s = value.stamp_s + 0.1;
  value.sensor_receipt_steady_ns = 1U;
  value.T_map_lidar = Eigen::Isometry3d::Identity();
  value.T_map_lidar.translation() = origin;
  value.hits_lidar.assign(hits.begin(), hits.end());
  value.frame_contract_id = "contract-a";
  return value;
}

RegisteredLidarFrameData beamFrame(
    const std::int64_t id, const Eigen::Vector3d& origin,
    std::initializer_list<RegisteredLidarBeamData> beams) {
  auto value = frame(id, origin, {});
  value.sensor_model_id = "first_hit_spherical_v1";
  value.horizontal_samples = static_cast<std::uint32_t>(beams.size());
  value.vertical_samples = 1U;
  value.horizontal_fov_rad = 0.0;
  value.vertical_min_rad = 0.0;
  value.vertical_max_rad = 0.0;
  value.min_range_m = 0.1;
  value.max_range_m = 6.0;
  value.beam_evidence_complete = true;
  value.beam_content_hash = "fixture-beams-v1";
  value.beams.assign(beams.begin(), beams.end());
  return value;
}

RegisteredLidarWindow makeWindow() {
  RegisteredLidarWindow::Geometry geometry;
  geometry.origin = Eigen::Vector3d::Zero();
  geometry.dimensions = Eigen::Vector3i(8, 4, 4);
  geometry.resolution_m = 1.0;
  geometry.frame_contract_id = "contract-a";
  return RegisteredLidarWindow(geometry);
}

TEST(RegisteredLidarWindow, CurrentOverlayReplacementPreservesActiveEvidence) {
  auto window = makeWindow();

  ActiveLidarWindowDeltaData base;
  base.base_generation = 0;
  base.generation = 1;
  base.complete = true;
  base.frame_contract_id = "contract-a";
  base.added.push_back(frame(10, Eigen::Vector3d(0.5, 0.5, 0.5),
                             {Eigen::Vector3d(3.0, 0.0, 0.0)}));
  ASSERT_TRUE(window.applyActiveDelta(base).accepted);

  ASSERT_TRUE(window.applyCurrentFrame(
      frame(20, Eigen::Vector3d(0.5, 1.5, 0.5),
            {Eigen::Vector3d(2.0, 0.0, 0.0)})).accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 0, 0)),
            RegisteredVoxelState::OCCUPIED);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(2, 1, 0)),
            RegisteredVoxelState::OCCUPIED);

  ASSERT_TRUE(window.applyCurrentFrame(
      frame(21, Eigen::Vector3d(0.5, 2.5, 0.5), {})).accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 0, 0)),
            RegisteredVoxelState::OCCUPIED);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(2, 1, 0)),
            RegisteredVoxelState::UNKNOWN);
}

TEST(RegisteredLidarWindow, SuccessfulHitRayMarksFreeAndHitWins) {
  auto window = makeWindow();
  ASSERT_TRUE(window.applyCurrentFrame(
      frame(1, Eigen::Vector3d(0.5, 0.5, 0.5),
            {Eigen::Vector3d(3.0, 0.0, 0.0)})).accepted);

  EXPECT_EQ(window.stateAt(Eigen::Vector3i(0, 0, 0)),
            RegisteredVoxelState::OBSERVED_FREE);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(1, 0, 0)),
            RegisteredVoxelState::OBSERVED_FREE);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(2, 0, 0)),
            RegisteredVoxelState::OBSERVED_FREE);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 0, 0)),
            RegisteredVoxelState::OCCUPIED);

  ActiveLidarWindowDeltaData delta;
  delta.base_generation = 0;
  delta.generation = 1;
  delta.complete = true;
  delta.frame_contract_id = "contract-a";
  delta.added.push_back(frame(2, Eigen::Vector3d(3.5, 1.5, 0.5),
                              {Eigen::Vector3d(0.0, -1.0, 0.0)}));
  ASSERT_TRUE(window.applyActiveDelta(delta).accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 0, 0)),
            RegisteredVoxelState::OCCUPIED);
}

TEST(RegisteredLidarWindow, UnthinnedDiagnosticSeparatesRayGapFromEndpointDeduplication) {
  auto window = makeWindow();
  // Both returns land in voxel (3,2,0), but their rays differ in voxel (2,1,0).
  auto source = frame(1, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(3.4, 1.6, 0), Eigen::Vector3d(2.6, 2.4, 0)});
  ASSERT_TRUE(window.applyCurrentFrame(source).accepted);
  const auto thinned = window.observationSourceFlags();
  const auto raw = window.unthinnedObservationMask(source);
  bool found_gap = false;
  for (size_t i = 0; i < raw.size(); ++i)
    found_gap |= raw[i] && !(thinned[i] & 3);
  EXPECT_TRUE(found_gap);
  // Replaying all rays cannot grant any additional online observation.
  EXPECT_EQ(window.observationSourceFlags(), thinned);
  EXPECT_FALSE(raw[(6 * 4 + 3) * 4]);
  EXPECT_EQ(window.currentFrameSource()->hits_lidar.size(), 2u);
}

TEST(RegisteredLidarWindow,
     NearBoundaryDiagonalRayCompletesWithoutOvershootingItsEndpoint) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  ASSERT_EXIT(
      {
        alarm(1U);
        auto window = makeWindow();
        const auto update = window.applyCurrentFrame(frame(
            3, Eigen::Vector3d(0.5, 0.5, 0.5),
            {Eigen::Vector3d(2.5 - 1.0e-13, 2.5, 0.0)}));
        alarm(0U);
        const bool correct = update.accepted &&
            window.stateAt(Eigen::Vector3i(2, 2, 0)) ==
                RegisteredVoxelState::OBSERVED_FREE &&
            window.stateAt(Eigen::Vector3i(2, 3, 0)) ==
                RegisteredVoxelState::OCCUPIED;
        std::_Exit(correct ? EXIT_SUCCESS : EXIT_FAILURE);
      },
      ::testing::ExitedWithCode(EXIT_SUCCESS), "");
}

TEST(RegisteredLidarWindow,
     ExplicitBeamOutcomesFreezeImmutableFreshTriStateEvidence) {
  auto window = makeWindow();
  RegisteredLidarBeamData hit;
  hit.direction_lidar = Eigen::Vector3d::UnitX();
  hit.outcome = RegisteredLidarBeamOutcome::HIT;
  hit.range_m = 3.0;
  RegisteredLidarBeamData no_return;
  no_return.direction_lidar = Eigen::Vector3d::UnitY();
  no_return.outcome = RegisteredLidarBeamOutcome::NO_RETURN;
  no_return.range_m = 6.0;
  RegisteredLidarBeamData invalid;
  invalid.direction_lidar = Eigen::Vector3d::UnitZ();
  invalid.outcome = RegisteredLidarBeamOutcome::INVALID;

  ASSERT_TRUE(window.applyCurrentFrame(beamFrame(
      10, Eigen::Vector3d(0.5, 0.5, 0.5),
      {hit, no_return, invalid})).accepted);
  const auto snapshot = window.captureLocalEvidenceSnapshot(41U);
  ASSERT_NE(snapshot, nullptr);
  EXPECT_EQ(snapshot->identity().occupancy_generation, 41U);
  EXPECT_EQ(snapshot->identity().active_window_generation, 0U);
  EXPECT_EQ(snapshot->identity().coordinate_contract, "contract-a");
  EXPECT_EQ(snapshot->identity().sensor_model_identity,
            "first_hit_spherical_v1");

  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(1.5, 0.5, 0.5), 10.5).state,
            EvidenceVoxelState::OBSERVED_FREE);
  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(3.5, 0.5, 0.5), 10.5).state,
            EvidenceVoxelState::RAW_OCCUPIED);
  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(4.5, 0.5, 0.5), 10.5).state,
            EvidenceVoxelState::UNKNOWN);
  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(0.5, 3.5, 0.5), 10.5).state,
            EvidenceVoxelState::OBSERVED_FREE);
  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(0.5, 0.5, 1.5), 10.5).state,
            EvidenceVoxelState::UNKNOWN);

  const auto stale_free =
      snapshot->queryVoxel(Eigen::Vector3d(1.5, 0.5, 0.5), 11.100001);
  EXPECT_EQ(stale_free.state, EvidenceVoxelState::UNKNOWN);
  EXPECT_EQ(stale_free.reason, LocalEvidenceReason::STALE_OBSERVATION);
  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(3.5, 0.5, 0.5), 11.100001)
                .state,
            EvidenceVoxelState::RAW_OCCUPIED);

  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(8.5, 0.5, 0.5), 10.5)
                .reason,
            LocalEvidenceReason::OUT_OF_RANGE);
  EXPECT_FALSE(snapshot->matches(41U, 0U, "other-contract",
                                 "first_hit_spherical_v1"));

  ASSERT_TRUE(window.applyCurrentFrame(
      beamFrame(11, Eigen::Vector3d(0.5, 0.5, 0.5), {invalid})).accepted);
  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(3.5, 0.5, 0.5), 10.5).state,
            EvidenceVoxelState::RAW_OCCUPIED)
      << "the frozen snapshot must not follow later window mutations";
}

TEST(RegisteredLidarWindow,
     FreshHistoricalFrameSuppliesEvidenceOutsideCurrentScan) {
  auto window = makeWindow();
  RegisteredLidarBeamData historical;
  historical.direction_lidar = Eigen::Vector3d::UnitX();
  historical.outcome = RegisteredLidarBeamOutcome::NO_RETURN;
  historical.range_m = 6.0;
  auto old_frame = beamFrame(
      20, Eigen::Vector3d(0.5, 0.5, 0.5), {historical});
  old_frame.stamp_s = 100.0;
  old_frame.scan_end_stamp_s = 100.1;
  ActiveLidarWindowDeltaData active;
  active.base_generation = 0;
  active.generation = 1;
  active.complete = true;
  active.frame_contract_id = "contract-a";
  active.added.push_back(old_frame);
  ASSERT_TRUE(window.applyActiveDelta(active).accepted);

  RegisteredLidarBeamData current;
  current.direction_lidar = Eigen::Vector3d::UnitY();
  current.outcome = RegisteredLidarBeamOutcome::NO_RETURN;
  current.range_m = 6.0;
  auto current_frame = beamFrame(
      21, Eigen::Vector3d(0.5, 0.5, 0.5), {current});
  current_frame.stamp_s = 100.2;
  current_frame.scan_end_stamp_s = 100.3;
  ASSERT_TRUE(window.applyCurrentFrame(current_frame).accepted);

  const auto snapshot = window.captureLocalEvidenceSnapshot(52U);
  ASSERT_NE(snapshot, nullptr);
  const auto historical_query =
      snapshot->queryVoxel(Eigen::Vector3d(4.5, 0.5, 0.5), 100.9);
  EXPECT_EQ(historical_query.state, EvidenceVoxelState::OBSERVED_FREE);
  EXPECT_EQ(historical_query.source_frame_id, 20);
  EXPECT_EQ(snapshot->queryVoxel(Eigen::Vector3d(4.5, 0.5, 0.5), 101.100001)
                .state,
            EvidenceVoxelState::UNKNOWN);
}

TEST(RegisteredLidarWindow,
     ActiveObstacleSourcesPreserveFrameIdentityAndImmutableCenters) {
  auto window = makeWindow();
  ActiveLidarWindowDeltaData delta;
  delta.base_generation = 0;
  delta.generation = 1;
  delta.complete = true;
  delta.frame_contract_id = "contract-a";
  auto source_frame = frame(
      12, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(2.0, 0.0, 0.0)});
  source_frame.source_is_map_reference = true;
  source_frame.source_health_valid = true;
  source_frame.source_health_stamp_s = source_frame.stamp_s;
  source_frame.source_icp_degenerate = false;
  source_frame.source_icp_rmse = 0.02;
  source_frame.source_icp_condition = 11.0;
  source_frame.source_icp_gamma_lidar = 1.2;
  source_frame.source_lidar_pl_enu_m = Eigen::Vector3d(0.3, 0.4, 0.5);
  delta.added.push_back(source_frame);
  ASSERT_TRUE(window.applyActiveDelta(delta).accepted);

  const auto sources = window.activeObstacleSources();
  ASSERT_NE(sources, nullptr);
  ASSERT_EQ(sources->size(), 1u);
  EXPECT_EQ(sources->front().metadata.frame_id, 12);
  EXPECT_DOUBLE_EQ(sources->front().metadata.stamp_s, 12.0);
  EXPECT_TRUE(sources->front().metadata.source_is_map_reference);
  EXPECT_TRUE(sources->front().metadata.source_health_valid);
  EXPECT_FALSE(sources->front().metadata.source_icp_degenerate);
  EXPECT_DOUBLE_EQ(sources->front().metadata.source_icp_rmse, 0.02);
  EXPECT_TRUE(sources->front().metadata.source_lidar_pl_enu_m.isApprox(
      Eigen::Vector3d(0.3, 0.4, 0.5)));
  EXPECT_FALSE(sources->front().metadata.content_hash.empty());
  EXPECT_EQ(window.activeObstacleSources()->front().metadata.content_hash,
            sources->front().metadata.content_hash);
  ASSERT_NE(sources->front().occupied_voxel_centers, nullptr);
  ASSERT_EQ(sources->front().occupied_voxel_centers->size(), 1u);
  EXPECT_TRUE(sources->front().occupied_voxel_centers->front().isApprox(
      Eigen::Vector3d(2.5, 0.5, 0.5)));

  ActiveLidarWindowDeltaData remove;
  remove.base_generation = 1;
  remove.generation = 2;
  remove.complete = true;
  remove.frame_contract_id = "contract-a";
  remove.removed_frame_ids.push_back(12);
  ASSERT_TRUE(window.applyActiveDelta(remove).accepted);
  EXPECT_EQ(window.activeObstacleSources()->size(), 0u);
  EXPECT_EQ(sources->size(), 1u);
}

TEST(RegisteredLidarWindow,
     AtomicFrameReplacementUpgradesSourceHealthWithoutLosingOccupancy) {
  auto window = makeWindow();
  ActiveLidarWindowDeltaData add;
  add.base_generation = 0;
  add.generation = 1;
  add.complete = true;
  add.frame_contract_id = "contract-a";
  const auto initial = frame(
      12, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(2.0, 0.0, 0.0)});
  add.added.push_back(initial);
  ASSERT_TRUE(window.applyActiveDelta(add).accepted);
  ASSERT_EQ(window.activeObstacleSources()->size(), 1u);
  EXPECT_FALSE(window.activeObstacleSources()->front().metadata.
                   source_health_valid);

  auto certified = initial;
  certified.source_health_valid = true;
  certified.source_health_stamp_s = certified.stamp_s;
  certified.source_icp_degenerate = false;
  certified.source_icp_rmse = 0.02;
  certified.source_icp_condition = 11.0;
  certified.source_icp_gamma_lidar = 1.2;
  certified.source_lidar_pl_enu_m = Eigen::Vector3d(0.3, 0.4, 0.5);
  ActiveLidarWindowDeltaData replace;
  replace.base_generation = 1;
  replace.generation = 2;
  replace.complete = true;
  replace.frame_contract_id = "contract-a";
  replace.removed_frame_ids.push_back(12);
  replace.added.push_back(certified);
  const auto update = window.applyActiveDelta(replace);
  ASSERT_TRUE(update.accepted) << update.reason;
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(2, 0, 0)),
            RegisteredVoxelState::OCCUPIED);
  const auto sources = window.activeObstacleSources();
  ASSERT_EQ(sources->size(), 1u);
  EXPECT_TRUE(sources->front().metadata.source_health_valid);
  EXPECT_DOUBLE_EQ(sources->front().metadata.source_icp_rmse, 0.02);
}

TEST(RegisteredLidarWindow,
     AbsoluteLidarPlDiagnosticDoesNotChangeRegisteredContentIdentity) {
  auto first_window = makeWindow();
  auto second_window = makeWindow();
  auto first = frame(12, Eigen::Vector3d(0.5, 0.5, 0.5),
                     {Eigen::Vector3d(2.0, 0.0, 0.0)});
  first.source_health_valid = true;
  first.source_health_stamp_s = first.stamp_s;
  first.source_icp_degenerate = false;
  first.source_icp_rmse = 0.02;
  first.source_icp_condition = 11.0;
  first.source_icp_gamma_lidar = 1.2;
  first.source_lidar_pl_enu_m = Eigen::Vector3d(1.0, 2.0, 3.0);
  auto second = first;
  second.source_lidar_pl_enu_m = Eigen::Vector3d(100.0, 200.0, 300.0);

  ActiveLidarWindowDeltaData first_delta;
  first_delta.base_generation = 0;
  first_delta.generation = 1;
  first_delta.complete = true;
  first_delta.frame_contract_id = "contract-a";
  first_delta.added.push_back(first);
  ActiveLidarWindowDeltaData second_delta = first_delta;
  second_delta.added.front() = second;
  ASSERT_TRUE(first_window.applyActiveDelta(first_delta).accepted);
  ASSERT_TRUE(second_window.applyActiveDelta(second_delta).accepted);

  const auto first_sources = first_window.activeObstacleSources();
  const auto second_sources = second_window.activeObstacleSources();
  ASSERT_EQ(first_sources->size(), 1u);
  ASSERT_EQ(second_sources->size(), 1u);
  EXPECT_EQ(first_sources->front().metadata.content_hash,
            second_sources->front().metadata.content_hash);
  EXPECT_FALSE(first_sources->front().metadata.source_lidar_pl_enu_m.isApprox(
      second_sources->front().metadata.source_lidar_pl_enu_m));
}

TEST(RegisteredLidarWindow, OutOfBoundsHitRetainsObservedFreeMapPrefix) {
  auto window = makeWindow();
  const auto update = window.applyCurrentFrame(frame(
      1, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(10.0, 0.0, 0.0)}));
  ASSERT_TRUE(update.accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(0, 0, 0)),
            RegisteredVoxelState::OBSERVED_FREE);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(7, 0, 0)),
            RegisteredVoxelState::OBSERVED_FREE);
  const auto environment = window.environmentOccupiedVoxelCenters();
  ASSERT_NE(environment, nullptr);
  ASSERT_EQ(environment->size(), 1U);
  EXPECT_GT(environment->front().x(), 8.0);
}

TEST(RegisteredLidarWindow,
     EnvironmentSnapshotIsImmutableAndCurrentMetadataIsLightweight) {
  auto window = makeWindow();
  ASSERT_TRUE(window.applyCurrentFrame(frame(
      1, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(10.0, 0.0, 0.0)})).accepted);
  const auto first = window.environmentOccupiedVoxelCenters();
  ASSERT_NE(first, nullptr);
  ASSERT_EQ(first->size(), 1U);
  const Eigen::Vector3d first_center = first->front();

  ASSERT_TRUE(window.applyCurrentFrame(frame(
      2, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(12.0, 0.0, 0.0)})).accepted);
  const auto second = window.environmentOccupiedVoxelCenters();
  ASSERT_NE(second, nullptr);
  ASSERT_EQ(second->size(), 1U);
  EXPECT_NE(first.get(), second.get());
  EXPECT_TRUE(first->front().isApprox(first_center));
  EXPECT_FALSE(second->front().isApprox(first_center));

  const auto metadata = window.currentFrameMetadata();
  ASSERT_TRUE(metadata.has_value());
  EXPECT_EQ(metadata->frame_id, 2);
  EXPECT_DOUBLE_EQ(metadata->scan_end_stamp_s, 2.1);
  EXPECT_TRUE(metadata->T_map_lidar.translation().isApprox(
      Eigen::Vector3d(0.5, 0.5, 0.5)));
}

TEST(RegisteredLidarWindow,
     EnvironmentReferenceCountsPreserveActiveHitAfterCurrentReplacement) {
  auto window = makeWindow();
  ActiveLidarWindowDeltaData active;
  active.base_generation = 0;
  active.generation = 1;
  active.complete = true;
  active.frame_contract_id = "contract-a";
  active.added.push_back(frame(
      10, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(10.0, 0.0, 0.0)}));
  ASSERT_TRUE(window.applyActiveDelta(active).accepted);
  ASSERT_TRUE(window.applyCurrentFrame(frame(
      20, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(10.0, 0.0, 0.0)})).accepted);
  ASSERT_EQ(window.environmentOccupiedVoxelCenters()->size(), 1U);

  ASSERT_TRUE(window.applyCurrentFrame(frame(
      21, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(12.0, 0.0, 0.0)})).accepted);
  const auto environment = window.environmentOccupiedVoxelCenters();
  ASSERT_EQ(environment->size(), 2U);
  EXPECT_LT(environment->front().x(), environment->back().x());
}

TEST(RegisteredLidarWindow, DeltaGapAndContractChangeRequireRecovery) {
  auto window = makeWindow();
  ActiveLidarWindowDeltaData delta;
  delta.base_generation = 1;
  delta.generation = 2;
  delta.complete = true;
  delta.frame_contract_id = "contract-a";
  const auto gap = window.applyActiveDelta(delta);
  EXPECT_FALSE(gap.accepted);
  EXPECT_TRUE(gap.recovery_required);
  EXPECT_EQ(gap.reason, "generation_gap");

  delta.base_generation = 0;
  delta.frame_contract_id = "contract-b";
  const auto contract = window.applyActiveDelta(delta);
  EXPECT_FALSE(contract.accepted);
  EXPECT_TRUE(contract.recovery_required);
  EXPECT_EQ(contract.reason, "frame_contract_mismatch");

  delta.frame_contract_id = "contract-a";
  delta.base_generation = 0;
  delta.generation = 2;
  const auto skipped_generation = window.applyActiveDelta(delta);
  EXPECT_FALSE(skipped_generation.accepted);
  EXPECT_TRUE(skipped_generation.recovery_required);
  EXPECT_EQ(skipped_generation.reason, "generation_gap");
}

TEST(RegisteredLidarWindow, RecoveryCannotRollBackActiveGeneration) {
  auto window = makeWindow();
  ActiveLidarWindowDeltaData delta;
  delta.base_generation = 0;
  delta.generation = 1;
  delta.complete = true;
  delta.frame_contract_id = "contract-a";
  ASSERT_TRUE(window.applyActiveDelta(delta).accepted);

  const auto stale = window.replaceActiveWindow(0, "contract-a", {});
  EXPECT_FALSE(stale.accepted);
  EXPECT_TRUE(stale.recovery_required);
  EXPECT_EQ(stale.reason, "recovery_generation_regression");
  EXPECT_EQ(window.activeGeneration(), 1U);
}

TEST(RegisteredLidarWindow, PoseCorrectionRetractsOldContribution) {
  auto window = makeWindow();
  ActiveLidarWindowDeltaData add;
  add.base_generation = 0;
  add.generation = 1;
  add.complete = true;
  add.frame_contract_id = "contract-a";
  add.added.push_back(frame(7, Eigen::Vector3d(0.5, 0.5, 0.5),
                            {Eigen::Vector3d(2.0, 0.0, 0.0)}));
  ASSERT_TRUE(window.applyActiveDelta(add).accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(2, 0, 0)),
            RegisteredVoxelState::OCCUPIED);

  ActiveLidarWindowDeltaData update;
  update.base_generation = 1;
  update.generation = 2;
  update.complete = true;
  update.frame_contract_id = "contract-a";
  update.pose_updates.emplace_back(7, Eigen::Translation3d(0.0, 1.0, 0.0) *
                                          Eigen::Isometry3d::Identity());
  ASSERT_TRUE(window.applyActiveDelta(update).accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(2, 0, 0)),
            RegisteredVoxelState::UNKNOWN);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(2, 1, 0)),
            RegisteredVoxelState::OCCUPIED);
}

TEST(RegisteredLidarWindow, CurrentFramePromotionDoesNotLoseItsEvidence) {
  auto window = makeWindow();
  const auto promoted = frame(
      9, Eigen::Vector3d(0.5, 0.5, 0.5),
      {Eigen::Vector3d(3.0, 0.0, 0.0)});
  ASSERT_TRUE(window.applyCurrentFrame(promoted).accepted);

  ActiveLidarWindowDeltaData add;
  add.base_generation = 0;
  add.generation = 1;
  add.complete = true;
  add.frame_contract_id = "contract-a";
  add.added.push_back(promoted);
  ASSERT_TRUE(window.applyActiveDelta(add).accepted);

  ASSERT_TRUE(window.applyCurrentFrame(
      frame(10, Eigen::Vector3d(0.5, 2.5, 0.5), {})).accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 0, 0)),
            RegisteredVoxelState::OCCUPIED);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(1, 0, 0)),
            RegisteredVoxelState::OBSERVED_FREE);
}

TEST(RegisteredLidarWindow, RemovingActiveFrameRetractsOnlyItsContribution) {
  auto window = makeWindow();
  ActiveLidarWindowDeltaData add;
  add.base_generation = 0;
  add.generation = 1;
  add.complete = true;
  add.frame_contract_id = "contract-a";
  add.added.push_back(frame(1, Eigen::Vector3d(0.5, 0.5, 0.5),
                            {Eigen::Vector3d(3.0, 0.0, 0.0)}));
  add.added.push_back(frame(2, Eigen::Vector3d(0.5, 1.5, 0.5),
                            {Eigen::Vector3d(3.0, 0.0, 0.0)}));
  ASSERT_TRUE(window.applyActiveDelta(add).accepted);

  ActiveLidarWindowDeltaData remove;
  remove.base_generation = 1;
  remove.generation = 2;
  remove.complete = true;
  remove.frame_contract_id = "contract-a";
  remove.removed_frame_ids.push_back(1);
  const auto result = window.applyActiveDelta(remove);
  ASSERT_TRUE(result.accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 0, 0)),
            RegisteredVoxelState::UNKNOWN);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 1, 0)),
            RegisteredVoxelState::OCCUPIED);
}

TEST(RegisteredLidarWindow, InvalidDeltaDoesNotPartiallyMutateWindow) {
  auto window = makeWindow();
  ActiveLidarWindowDeltaData add;
  add.base_generation = 0;
  add.generation = 1;
  add.complete = true;
  add.frame_contract_id = "contract-a";
  add.added.push_back(frame(1, Eigen::Vector3d(0.5, 0.5, 0.5),
                            {Eigen::Vector3d(3.0, 0.0, 0.0)}));
  ASSERT_TRUE(window.applyActiveDelta(add).accepted);

  ActiveLidarWindowDeltaData invalid;
  invalid.base_generation = 1;
  invalid.generation = 2;
  invalid.complete = true;
  invalid.frame_contract_id = "contract-a";
  invalid.removed_frame_ids.push_back(1);
  invalid.removed_frame_ids.push_back(999);
  const auto result = window.applyActiveDelta(invalid);
  EXPECT_FALSE(result.accepted);
  EXPECT_TRUE(result.recovery_required);
  EXPECT_EQ(window.activeGeneration(), 1U);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 0, 0)),
            RegisteredVoxelState::OCCUPIED);
}

TEST(RegisteredLidarWindow, DiagonalRayDoesNotClaimCornerAdjacentVoxels) {
  auto window = makeWindow();
  ASSERT_TRUE(window.applyCurrentFrame(
      frame(1, Eigen::Vector3d(0.5, 0.5, 0.5),
            {Eigen::Vector3d(3.0, 3.0, 0.0)})).accepted);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(1, 1, 0)),
            RegisteredVoxelState::OBSERVED_FREE);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(2, 2, 0)),
            RegisteredVoxelState::OBSERVED_FREE);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(1, 0, 0)),
            RegisteredVoxelState::UNKNOWN);
  EXPECT_EQ(window.stateAt(Eigen::Vector3i(3, 3, 0)),
            RegisteredVoxelState::OCCUPIED);
}

}  // namespace
