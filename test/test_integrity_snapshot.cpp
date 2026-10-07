#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <vector>

#include <iap/planner/integrity_snapshot.hpp>
#include <iap/gnss/gnss_epoch_wire.hpp>
#include <iap/msg/integrity_report.hpp>

namespace {

iap::CurrentIntegrityState make_current() {
  iap::CurrentIntegrityState current;
  current.stamp = 12.0;
  current.valid = true;
  current.integrity_state = 0;
  current.hpl = 4.0;
  current.vpl = 6.0;
  current.pl_e = 3.0;
  current.pl_n = 4.0;
  current.pl_u = 6.0;
  current.hal = 30.0;
  current.val = 20.0;
  current.im = 14.0;
  current.n_sv_used = 8;
  current.pdop = 2.5;
  current.n_hypotheses = 8;
  current.n_detected = 1;
  current.excluded_prns = {7};
  return current;
}

iap::GnssEpoch make_epoch() {
  iap::GnssEpoch epoch;
  epoch.stamp = 12.0;
  epoch.gps_sec = 1000.0;
  for (int i = 0; i < 5; ++i) {
    iap::SatObs sat;
    sat.sat_id = 10 + i;
    epoch.sats.push_back(sat);
  }
  return epoch;
}

}  // namespace

TEST(IntegritySnapshotBuilderTest, RejectedGnssTransportPreservesOptimizedObservations) {
  auto epoch=make_epoch(); epoch.source_identity=123;
  epoch.sats[0].pr_residual=2.75; epoch.sats[0].excluded=true;
  epoch.sats[0].sat_pos=Eigen::Vector3d(2e7,3e7,4e7);
  epoch.sats[1].pr_residual=std::numeric_limits<double>::quiet_NaN();
  epoch.sats[0].pr_sigma=7.5; epoch.sats[0].nis_pr=1.25;
  epoch.sats[0].sat_vel=Eigen::Vector3d(1,2,3);
  epoch.iono_params={1,2,3,4,5,6,7,8};
  iap::msg::IntegrityReport msg; msg.gnss_valid=false;
  iap::write_advisory_epoch(&epoch,msg);
  const auto decoded=iap::read_advisory_epoch(msg);
  ASSERT_TRUE(decoded); EXPECT_FALSE(msg.gnss_valid);
  EXPECT_EQ(decoded->source_identity,123u);
  EXPECT_EQ(decoded->sats.size(),epoch.sats.size());
  EXPECT_EQ(decoded->iono_params,epoch.iono_params);
  EXPECT_DOUBLE_EQ(decoded->sats[0].pr_residual,2.75);
  EXPECT_TRUE(decoded->sats[0].excluded);
  EXPECT_TRUE(decoded->sats[0].sat_pos.isApprox(epoch.sats[0].sat_pos));
  EXPECT_TRUE(std::isnan(decoded->sats[1].pr_residual));
  EXPECT_DOUBLE_EQ(decoded->sats[0].pr_sigma,7.5);
  EXPECT_DOUBLE_EQ(decoded->sats[0].nis_pr,1.25);
  EXPECT_TRUE(decoded->sats[0].sat_vel.isApprox(epoch.sats[0].sat_vel));
  iap::write_advisory_epoch(nullptr,msg);
  EXPECT_FALSE(iap::read_advisory_epoch(msg));
  EXPECT_TRUE(msg.advisory_gnss_observations.empty());
}

TEST(IntegritySnapshotBuilderTest, FullInputProducesValidSnapshot) {
  iap::IntegritySnapshotBuilder builder;
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = 12.5;
  input.has_pose = true;
  input.p_wb = Eigen::Vector3d(1.0, 2.0, 3.0);
  input.q_wb = Eigen::Quaterniond::Identity();
  input.current = make_current();
  const iap::GnssEpoch epoch = make_epoch();
  input.gnss_epoch = &epoch;
  const Eigen::Matrix3d lambda = Eigen::Matrix3d::Identity() * 3.0;
  input.lambda_base_pos = &lambda;

  iap::LidarAraimSnapshot lidar_snapshot;
  lidar_snapshot.valid = true;
  lidar_snapshot.current_icp_quality.gamma_lidar = 1.2;
  lidar_snapshot.blocks.emplace_back();
  input.lidar_snapshot = &lidar_snapshot;

  iap::LidarAraimResult lidar_result;
  lidar_result.valid = true;
  lidar_result.n_hypotheses = 2;
  lidar_result.n_detected = 1;
  input.lidar_araim_result = &lidar_result;

  const auto snapshot = builder.build_from_latest(input);

  EXPECT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.has_pose);
  EXPECT_TRUE(snapshot.has_epoch);
  EXPECT_EQ(snapshot.gnss_epoch.sats.size(), 5u);
  EXPECT_TRUE(snapshot.has_lambda_base);
  EXPECT_TRUE(snapshot.has_lidar_snapshot);
  EXPECT_TRUE(snapshot.lidar_snapshot_valid);
  EXPECT_EQ(snapshot.lidar_block_count, 1);
  EXPECT_DOUBLE_EQ(snapshot.lidar_alpha, 1.2);
  EXPECT_TRUE(snapshot.has_lidar_araim_result);
  EXPECT_EQ(snapshot.lidar_araim_n_hypotheses, 2);
  EXPECT_EQ(snapshot.current.excluded_prns, std::vector<int>({7}));
  EXPECT_DOUBLE_EQ(snapshot.current.pl, 6.0);
}

TEST(IntegritySnapshotBuilderTest, MissingGnssEpochIsExplicit) {
  iap::IntegritySnapshotBuilder builder;
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = 10.0;
  input.has_pose = true;
  input.p_wb = Eigen::Vector3d::Zero();
  input.q_wb = Eigen::Quaterniond::Identity();
  input.current = make_current();

  const auto snapshot = builder.build_from_latest(input);

  EXPECT_TRUE(snapshot.valid);
  EXPECT_FALSE(snapshot.has_epoch);
  EXPECT_TRUE(snapshot.gnss_epoch.sats.empty());
}

TEST(IntegritySnapshotBuilderTest, MissingLidarDoesNotInvalidateGnssOnlySnapshot) {
  iap::IntegritySnapshotBuilder builder;
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = 10.0;
  input.has_pose = true;
  input.p_wb = Eigen::Vector3d::Zero();
  input.q_wb = Eigen::Quaterniond::Identity();
  input.current = make_current();
  const iap::GnssEpoch epoch = make_epoch();
  input.gnss_epoch = &epoch;

  const auto snapshot = builder.build_from_latest(input);

  EXPECT_TRUE(snapshot.valid);
  EXPECT_TRUE(snapshot.has_epoch);
  EXPECT_FALSE(snapshot.has_lidar_snapshot);
  EXPECT_FALSE(snapshot.has_lidar_araim_result);
}

TEST(IntegritySnapshotBuilderTest, CurrentFieldsAreCopied) {
  iap::IntegritySnapshotBuilder builder;
  iap::IntegritySnapshotBuilderInput input;
  input.stamp = 10.0;
  input.has_pose = true;
  input.p_wb = Eigen::Vector3d::Zero();
  input.q_wb = Eigen::Quaterniond::Identity();
  input.current = make_current();
  input.current.pl = std::numeric_limits<double>::quiet_NaN();

  const auto snapshot = builder.build_from_latest(input);

  EXPECT_TRUE(snapshot.valid);
  EXPECT_DOUBLE_EQ(snapshot.current.hpl, 4.0);
  EXPECT_DOUBLE_EQ(snapshot.current.vpl, 6.0);
  EXPECT_DOUBLE_EQ(snapshot.current.pl, 6.0);
  EXPECT_DOUBLE_EQ(snapshot.current.hal, 30.0);
  EXPECT_DOUBLE_EQ(snapshot.current.val, 20.0);
  EXPECT_DOUBLE_EQ(snapshot.current.im, 14.0);
  EXPECT_EQ(snapshot.current.n_sv_used, 8);
  EXPECT_DOUBLE_EQ(snapshot.current.pdop, 2.5);
}
