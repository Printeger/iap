#include <gtest/gtest.h>

#include <iap/local_map/active_window_delta_policy.hpp>

TEST(ActiveWindowDeltaPolicy, UnchangedCompleteWindowIsNotAStateChange) {
  iap::msg::ActiveLidarWindowDelta delta;
  delta.complete = true;

  EXPECT_FALSE(iap::local_map::activeWindowDeltaChangesState(delta, true));
}

TEST(ActiveWindowDeltaPolicy, EveryAuthoritativeMutationIsAStateChange) {
  iap::msg::ActiveLidarWindowDelta delta;
  delta.complete = true;

  delta.added.emplace_back();
  EXPECT_TRUE(iap::local_map::activeWindowDeltaChangesState(delta, true));
  delta.added.clear();

  delta.removed_frame_ids.push_back(1);
  EXPECT_TRUE(iap::local_map::activeWindowDeltaChangesState(delta, true));
  delta.removed_frame_ids.clear();

  delta.pose_updated_frame_ids.push_back(1);
  EXPECT_TRUE(iap::local_map::activeWindowDeltaChangesState(delta, true));
  delta.pose_updated_frame_ids.clear();

  EXPECT_TRUE(iap::local_map::activeWindowDeltaChangesState(delta, false));
  delta.complete = false;
  EXPECT_TRUE(iap::local_map::activeWindowDeltaChangesState(delta, true));
}

TEST(ActiveWindowDeltaPolicy,
     CertifiedSourceHealthRequiresAtomicFrameReplacement) {
  iap::msg::RegisteredLidarFrame uncertified;
  iap::msg::RegisteredLidarFrame certified;
  certified.source_health_valid = true;
  certified.source_health_stamp_s = 10.0;
  certified.source_icp_degenerate = false;
  certified.source_icp_rmse = 0.1;
  certified.source_icp_condition = 2.0;
  certified.source_icp_gamma_lidar = 1.0;

  ASSERT_FALSE(iap::local_map::hasCertifiedSourceHealth(uncertified));
  ASSERT_TRUE(iap::local_map::hasCertifiedSourceHealth(certified));
  ASSERT_TRUE(iap::local_map::sourceEvidenceReplacementRequired(
      uncertified, certified));

  iap::msg::ActiveLidarWindowDelta replacement;
  replacement.complete = true;
  replacement.removed_frame_ids.push_back(7);
  replacement.added.push_back(certified);
  EXPECT_TRUE(iap::local_map::activeWindowDeltaChangesState(
      replacement, true));
}

TEST(ActiveWindowDeltaPolicy,
     LateBeamEvidenceUpgradesAnAlreadyHealthyFrame) {
  iap::msg::RegisteredLidarFrame current;
  current.frame_id = 89;
  current.header.stamp.sec = 1791257733;
  current.header.stamp.nanosec = 470096800;
  current.scan_end_stamp_s = 1791257733.4700968;
  current.frame_contract_id = "contract-a";
  current.source_health_valid = true;
  current.source_health_stamp_s = current.scan_end_stamp_s;
  current.source_icp_rmse = 0.1;
  current.source_icp_condition = 2.0;
  current.source_icp_gamma_lidar = 1.0;
  auto refreshed = current;
  refreshed.beam_evidence_complete = true;
  refreshed.beam_content_hash = "validated-at-receipt";
  EXPECT_TRUE(iap::local_map::sourceEvidenceReplacementRequired(
      current, refreshed));
  EXPECT_FALSE(iap::local_map::sourceEvidenceReplacementRequired(
      refreshed, current));
  auto neighbor = refreshed;
  ++neighbor.frame_id;
  EXPECT_FALSE(iap::local_map::sourceEvidenceReplacementRequired(current, neighbor));
  neighbor = refreshed;
  ++neighbor.header.stamp.nanosec;
  EXPECT_FALSE(iap::local_map::sourceEvidenceReplacementRequired(current, neighbor));
  neighbor = refreshed;
  neighbor.source_health_valid = false;
  EXPECT_FALSE(iap::local_map::sourceEvidenceReplacementRequired(current, neighbor));
}

TEST(ActiveWindowDeltaPolicy,
     CompleteCommittedSnapshotSurvivesNewerPendingProposal) {
  const iap::local_map::ActiveWindowRecoveryState state{
      11U, 10U, 1U, 2U, true, true};

  EXPECT_TRUE(iap::local_map::committedActiveWindowRecoverable(state));
}

TEST(ActiveWindowDeltaPolicy,
     IncompleteProducerStateStillRejectsCommittedRecovery) {
  const iap::local_map::ActiveWindowRecoveryState state{
      11U, 10U, 1U, 2U, false, true};

  EXPECT_FALSE(iap::local_map::committedActiveWindowRecoverable(state));
}

TEST(ActiveWindowDeltaPolicy,
     RecoveryRejectsEmptyOrGenerationZeroCommittedState) {
  EXPECT_FALSE(iap::local_map::committedActiveWindowRecoverable(
      {11U, 10U, 0U, 2U, true, true}));
  EXPECT_FALSE(iap::local_map::committedActiveWindowRecoverable(
      {11U, 10U, 1U, 0U, true, true}));
}
