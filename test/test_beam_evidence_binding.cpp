#include <gtest/gtest.h>

#include <iap/local_map/beam_evidence_binding.hpp>

namespace {

iap::msg::LidarBeamEvidence makeEvidence() {
  iap::msg::LidarBeamEvidence evidence;
  evidence.header.stamp.sec = 42;
  evidence.header.stamp.nanosec = 250000000u;
  evidence.scan_end_stamp_s = 42.35;
  evidence.sensor_model_id = "first-hit-spherical-v1";
  evidence.horizontal_samples = 2u;
  evidence.vertical_samples = 1u;
  evidence.horizontal_fov_rad = 1.0;
  evidence.vertical_min_rad = -0.2;
  evidence.vertical_max_rad = 0.2;
  evidence.min_range_m = 0.1;
  evidence.max_range_m = 8.0;
  evidence.complete = true;
  evidence.outcomes = {1u, 2u};
  evidence.direction_x = {1.0, 0.0};
  evidence.direction_y = {0.0, 1.0};
  evidence.direction_z = {0.0, 0.0};
  evidence.ranges_m = {3.0, 8.0};
  evidence.content_hash =
      iap::local_map::beamEvidenceContentHash(evidence);
  return evidence;
}

TEST(BeamEvidenceBinding, RequiresExactScanStartAndEndCorrelation) {
  const auto evidence = makeEvidence();
  EXPECT_TRUE(iap::local_map::validBeamEvidenceMessage(evidence));
  EXPECT_TRUE(iap::local_map::beamEvidenceMatchesRegisteredScan(
      evidence, 42.25, 42.35));
  EXPECT_FALSE(iap::local_map::beamEvidenceMatchesRegisteredScan(
      evidence, 42.250002, 42.35));
  EXPECT_FALSE(iap::local_map::beamEvidenceMatchesRegisteredScan(
      evidence, 42.25, 42.350002));
}

TEST(BeamEvidenceBinding, RejectsIncompleteOrContentMismatchedEvidence) {
  auto evidence = makeEvidence();
  evidence.ranges_m.front() = 3.5;
  EXPECT_FALSE(iap::local_map::validBeamEvidenceMessage(evidence));
  EXPECT_FALSE(iap::local_map::beamEvidenceMatchesRegisteredScan(
      evidence, 42.25, 42.35));

  evidence = makeEvidence();
  evidence.complete = false;
  EXPECT_FALSE(iap::local_map::validBeamEvidenceMessage(evidence));
}

TEST(BeamEvidenceBinding, RejectsMissingBeamRowsAndInvalidSensorGeometry) {
  auto evidence = makeEvidence();
  evidence.direction_z.pop_back();
  EXPECT_FALSE(iap::local_map::validBeamEvidenceMessage(evidence));

  evidence = makeEvidence();
  evidence.max_range_m = evidence.min_range_m;
  evidence.content_hash =
      iap::local_map::beamEvidenceContentHash(evidence);
  EXPECT_FALSE(iap::local_map::validBeamEvidenceMessage(evidence));
}

}  // namespace
