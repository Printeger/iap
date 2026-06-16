// IAP Step 4: Unit tests for IntegrityFusionPolicy

#include <gtest/gtest.h>
#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/integrity_source_result.hpp>
#include <iap/integrity/integrity_fusion_policy.hpp>
#include <algorithm>
#include <cmath>
#include <string>

using namespace iap;

namespace {

IntegritySourceResult source(const std::string& name,
                             double pl_e,
                             double pl_n,
                             double pl_u) {
  return IntegritySourceResult::make_valid(
      name, std::max(pl_e, pl_n), pl_u, pl_e, pl_n, pl_u);
}

IntegritySourceResult gnss_source() {
  return source("GNSS", 3.0, 2.0, 10.0);
}

IntegritySourceResult lidar_source() {
  return source("LIDAR", 8.0, 4.0, 4.0);
}

IntegritySourceResult fallback_source() {
  return source("FALLBACK", 5.0, 5.0, 5.0);
}

void expect_finite_conservative(const IntegrityFusionResult& r,
                                double expected_hpl = 999.0,
                                double expected_vpl = 999.0) {
  EXPECT_TRUE(std::isfinite(r.HPL));
  EXPECT_TRUE(std::isfinite(r.VPL));
  EXPECT_TRUE(std::isfinite(r.PL_E));
  EXPECT_TRUE(std::isfinite(r.PL_N));
  EXPECT_TRUE(std::isfinite(r.PL_U));
  EXPECT_DOUBLE_EQ(r.HPL, expected_hpl);
  EXPECT_DOUBLE_EQ(r.VPL, expected_vpl);
  EXPECT_DOUBLE_EQ(r.PL_E, expected_hpl);
  EXPECT_DOUBLE_EQ(r.PL_N, expected_hpl);
  EXPECT_DOUBLE_EQ(r.PL_U, expected_vpl);
  EXPECT_EQ(r.final_HPL_source, "CONSERVATIVE");
  EXPECT_EQ(r.final_VPL_source, "CONSERVATIVE");
  EXPECT_EQ(r.final_PL_source, "CONSERVATIVE");
  EXPECT_FALSE(r.any_source_valid);
  EXPECT_FALSE(r.failure_reason.empty());
}

void expect_same_fusion_result(const IntegrityFusionResult& a,
                               const IntegrityFusionResult& b) {
  EXPECT_DOUBLE_EQ(a.HPL, b.HPL);
  EXPECT_DOUBLE_EQ(a.VPL, b.VPL);
  EXPECT_DOUBLE_EQ(a.PL_E, b.PL_E);
  EXPECT_DOUBLE_EQ(a.PL_N, b.PL_N);
  EXPECT_DOUBLE_EQ(a.PL_U, b.PL_U);
  EXPECT_EQ(a.final_HPL_source, b.final_HPL_source);
  EXPECT_EQ(a.final_VPL_source, b.final_VPL_source);
  EXPECT_EQ(a.final_PL_source, b.final_PL_source);
  EXPECT_EQ(a.failure_reason, b.failure_reason);
  EXPECT_EQ(a.any_source_valid, b.any_source_valid);
}

}  // namespace

// ============================================================================
// T4.1: to_string / from_string
// ============================================================================
TEST(IntegrityFusionModeTest, ToStringRoundTrip) {
  EXPECT_STREQ(to_string(IntegrityFusionMode::GNSS_ONLY), "gnss_only");
  EXPECT_STREQ(to_string(IntegrityFusionMode::LIDAR_ONLY), "lidar_only");
  EXPECT_STREQ(to_string(IntegrityFusionMode::FALLBACK_ONLY), "fallback_only");
  EXPECT_STREQ(to_string(IntegrityFusionMode::MAX_PL), "max_pl");
  EXPECT_STREQ(to_string(IntegrityFusionMode::WEIGHTED_DEBUG_ONLY), "weighted_debug_only");
}

TEST(IntegrityFusionModeTest, FromStringValid) {
  EXPECT_EQ(fusion_mode_from_string("gnss_only"), IntegrityFusionMode::GNSS_ONLY);
  EXPECT_EQ(fusion_mode_from_string("lidar_only"), IntegrityFusionMode::LIDAR_ONLY);
  EXPECT_EQ(fusion_mode_from_string("fallback_only"), IntegrityFusionMode::FALLBACK_ONLY);
  EXPECT_EQ(fusion_mode_from_string("max_pl"), IntegrityFusionMode::MAX_PL);
  EXPECT_EQ(fusion_mode_from_string("weighted_debug_only"),
            IntegrityFusionMode::WEIGHTED_DEBUG_ONLY);
}

TEST(IntegrityFusionModeTest, FromStringUnknownFallsBackToMaxPl) {
  EXPECT_EQ(fusion_mode_from_string("bogus"), IntegrityFusionMode::MAX_PL);
  EXPECT_EQ(fusion_mode_from_string(""), IntegrityFusionMode::MAX_PL);
}

// ============================================================================
// T4.2: GNSS-only mode
// ============================================================================
TEST(IntegrityFusionPolicyTest, GnssOnlyUsesGnss) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::GNSS_ONLY;
  IntegrityFusionPolicy policy(p);

  const auto gnss = gnss_source();
  const auto r = policy.fuse(fallback_source(), gnss, lidar_source());

  EXPECT_DOUBLE_EQ(r.HPL, gnss.HPL);
  EXPECT_DOUBLE_EQ(r.VPL, gnss.VPL);
  EXPECT_DOUBLE_EQ(r.PL_E, gnss.PL_E);
  EXPECT_DOUBLE_EQ(r.PL_N, gnss.PL_N);
  EXPECT_DOUBLE_EQ(r.PL_U, gnss.PL_U);
  EXPECT_EQ(r.final_HPL_source, "GNSS");
  EXPECT_EQ(r.final_VPL_source, "GNSS");
  EXPECT_EQ(r.final_PL_source, "GNSS");
  EXPECT_TRUE(r.any_source_valid);
  EXPECT_TRUE(r.failure_reason.empty());
}

TEST(IntegrityFusionPolicyTest, GnssOnlyConservativeWhenGnssInvalid) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::GNSS_ONLY;
  p.conservative_hpl_m = 999.0;
  p.conservative_vpl_m = 888.0;
  IntegrityFusionPolicy policy(p);

  const auto r = policy.fuse(
      fallback_source(),
      IntegritySourceResult::make_invalid("GNSS", "ARAIM failed"),
      lidar_source());

  expect_finite_conservative(r, 999.0, 888.0);
  EXPECT_EQ(r.failure_reason, "GNSS source unavailable or invalid (gnss_only mode)");
}

// ============================================================================
// T4.3: LiDAR-only mode
// ============================================================================
TEST(IntegrityFusionPolicyTest, LidarOnlyUsesLidar) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::LIDAR_ONLY;
  IntegrityFusionPolicy policy(p);

  const auto lidar = lidar_source();
  const auto r = policy.fuse(fallback_source(), gnss_source(), lidar);

  EXPECT_DOUBLE_EQ(r.HPL, lidar.HPL);
  EXPECT_DOUBLE_EQ(r.VPL, lidar.VPL);
  EXPECT_DOUBLE_EQ(r.PL_E, lidar.PL_E);
  EXPECT_DOUBLE_EQ(r.PL_N, lidar.PL_N);
  EXPECT_DOUBLE_EQ(r.PL_U, lidar.PL_U);
  EXPECT_EQ(r.final_HPL_source, "LIDAR");
  EXPECT_EQ(r.final_VPL_source, "LIDAR");
  EXPECT_EQ(r.final_PL_source, "LIDAR");
  EXPECT_TRUE(r.any_source_valid);
  EXPECT_TRUE(r.failure_reason.empty());
}

// ============================================================================
// T4.4: Fallback-only mode
// ============================================================================
TEST(IntegrityFusionPolicyTest, FallbackOnlyUsesFallback) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::FALLBACK_ONLY;
  IntegrityFusionPolicy policy(p);

  const auto fb = fallback_source();
  const auto r = policy.fuse(fb, gnss_source(), lidar_source());

  EXPECT_DOUBLE_EQ(r.HPL, fb.HPL);
  EXPECT_DOUBLE_EQ(r.VPL, fb.VPL);
  EXPECT_DOUBLE_EQ(r.PL_E, fb.PL_E);
  EXPECT_DOUBLE_EQ(r.PL_N, fb.PL_N);
  EXPECT_DOUBLE_EQ(r.PL_U, fb.PL_U);
  EXPECT_EQ(r.final_HPL_source, "FALLBACK");
  EXPECT_EQ(r.final_VPL_source, "FALLBACK");
  EXPECT_EQ(r.final_PL_source, "FALLBACK");
  EXPECT_TRUE(r.any_source_valid);
  EXPECT_TRUE(r.failure_reason.empty());
}

// ============================================================================
// T4.5: max_pl per-axis max
// ============================================================================
TEST(IntegrityFusionPolicyTest, MaxPlPerAxisMax) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::MAX_PL;
  IntegrityFusionPolicy policy(p);

  const auto r = policy.fuse(fallback_source(), gnss_source(), lidar_source());

  EXPECT_DOUBLE_EQ(r.PL_E, 8.0);
  EXPECT_DOUBLE_EQ(r.PL_N, 5.0);
  EXPECT_DOUBLE_EQ(r.PL_U, 10.0);
  EXPECT_DOUBLE_EQ(r.HPL, 8.0);
  EXPECT_DOUBLE_EQ(r.VPL, 10.0);
  EXPECT_EQ(r.final_HPL_source, "LIDAR");
  EXPECT_EQ(r.final_VPL_source, "GNSS");
  EXPECT_EQ(r.final_PL_source, "GNSS");
  EXPECT_TRUE(r.any_source_valid);
  EXPECT_TRUE(r.failure_reason.empty());
}

TEST(IntegrityFusionPolicyTest, MaxPlWithOnlyFallback) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::MAX_PL;
  IntegrityFusionPolicy policy(p);

  const auto fb = fallback_source();
  const auto r = policy.fuse(
      fb,
      IntegritySourceResult::make_invalid("GNSS", "no data"),
      IntegritySourceResult::make_invalid("LIDAR", "no data"));

  EXPECT_DOUBLE_EQ(r.HPL, fb.HPL);
  EXPECT_DOUBLE_EQ(r.VPL, fb.VPL);
  EXPECT_DOUBLE_EQ(r.PL_E, fb.PL_E);
  EXPECT_DOUBLE_EQ(r.PL_N, fb.PL_N);
  EXPECT_DOUBLE_EQ(r.PL_U, fb.PL_U);
  EXPECT_EQ(r.final_HPL_source, "FALLBACK");
  EXPECT_EQ(r.final_VPL_source, "FALLBACK");
  EXPECT_EQ(r.final_PL_source, "FALLBACK");
  EXPECT_TRUE(r.any_source_valid);
  EXPECT_TRUE(r.failure_reason.empty());
}

// ============================================================================
// T4.6: Disabled source ignored, not marked as failure
// ============================================================================
TEST(IntegrityFusionPolicyTest, DisabledSourceIgnored) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::MAX_PL;
  IntegrityFusionPolicy policy(p);

  auto disabled_gnss = IntegritySourceResult::make_disabled("GNSS");
  disabled_gnss.HPL = 1e12;
  disabled_gnss.VPL = 1e12;
  disabled_gnss.PL_E = 1e12;
  disabled_gnss.PL_N = 1e12;
  disabled_gnss.PL_U = 1e12;

  const auto r = policy.fuse(fallback_source(), disabled_gnss, lidar_source());

  EXPECT_DOUBLE_EQ(r.PL_E, 8.0);
  EXPECT_DOUBLE_EQ(r.PL_N, 5.0);
  EXPECT_DOUBLE_EQ(r.PL_U, 5.0);
  EXPECT_DOUBLE_EQ(r.HPL, 8.0);
  EXPECT_DOUBLE_EQ(r.VPL, 5.0);
  EXPECT_EQ(r.final_HPL_source, "LIDAR");
  EXPECT_EQ(r.final_VPL_source, "FALLBACK");
  EXPECT_TRUE(r.any_source_valid);
  EXPECT_TRUE(r.failure_reason.empty());
}

// ============================================================================
// T4.7: Invalid source ignored but diagnostics remain observable
// ============================================================================
TEST(IntegrityFusionPolicyTest, InvalidSourceIgnoredButFailureObservable) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::MAX_PL;
  IntegrityFusionPolicy policy(p);

  const auto invalid_lidar =
      IntegritySourceResult::make_invalid("LIDAR", "LiDAR ARAIM result invalid");
  const auto r = policy.fuse(fallback_source(), gnss_source(), invalid_lidar);

  EXPECT_FALSE(invalid_lidar.valid);
  EXPECT_TRUE(invalid_lidar.enabled);
  EXPECT_EQ(invalid_lidar.failure_reason, "LiDAR ARAIM result invalid");
  EXPECT_DOUBLE_EQ(r.PL_E, 5.0);
  EXPECT_DOUBLE_EQ(r.PL_N, 5.0);
  EXPECT_DOUBLE_EQ(r.PL_U, 10.0);
  EXPECT_DOUBLE_EQ(r.HPL, 5.0);
  EXPECT_DOUBLE_EQ(r.VPL, 10.0);
  EXPECT_EQ(r.final_HPL_source, "FALLBACK");
  EXPECT_EQ(r.final_VPL_source, "GNSS");
  EXPECT_TRUE(r.any_source_valid);
  EXPECT_TRUE(r.failure_reason.empty());
}

// ============================================================================
// T4.8: Required sources
// ============================================================================
TEST(IntegrityFusionPolicyTest, RequiredGnssMissingGivesConservative) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::MAX_PL;
  p.require_valid_gnss = true;
  IntegrityFusionPolicy policy(p);

  const auto r = policy.fuse(
      fallback_source(),
      IntegritySourceResult::make_invalid("GNSS", "no data"),
      lidar_source());

  expect_finite_conservative(r);
  EXPECT_EQ(r.failure_reason, "required GNSS source missing or invalid");
}

TEST(IntegrityFusionPolicyTest, RequiredLidarMissingGivesConservative) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::MAX_PL;
  p.require_valid_lidar = true;
  IntegrityFusionPolicy policy(p);

  const auto r = policy.fuse(
      fallback_source(),
      gnss_source(),
      IntegritySourceResult::make_invalid("LIDAR", "no snapshot"));

  expect_finite_conservative(r);
  EXPECT_EQ(r.failure_reason, "required LiDAR source missing or invalid");
}

// ============================================================================
// T4.9: No valid sources
// ============================================================================
TEST(IntegrityFusionPolicyTest, NoValidSourcesGivesConservative) {
  IntegrityFusionPolicyParams p;
  p.mode = IntegrityFusionMode::MAX_PL;
  IntegrityFusionPolicy policy(p);

  const auto r = policy.fuse(
      IntegritySourceResult::make_invalid("FALLBACK", "bad cov"),
      IntegritySourceResult::make_invalid("GNSS", "no epoch"),
      IntegritySourceResult::make_invalid("LIDAR", "no snapshot"));

  expect_finite_conservative(r);
  EXPECT_EQ(r.failure_reason, "no valid integrity source available");
}

// ============================================================================
// T4.10: weighted_debug_only deterministic
// ============================================================================
TEST(IntegrityFusionPolicyTest, WeightedDebugOnlyFallsBackToMaxPl) {
  IntegrityFusionPolicyParams max_p;
  max_p.mode = IntegrityFusionMode::MAX_PL;
  IntegrityFusionPolicy max_policy(max_p);

  IntegrityFusionPolicyParams weighted_p;
  weighted_p.mode = IntegrityFusionMode::WEIGHTED_DEBUG_ONLY;
  IntegrityFusionPolicy weighted_policy(weighted_p);

  const auto expected =
      max_policy.fuse(fallback_source(), gnss_source(), lidar_source());
  const auto weighted =
      weighted_policy.fuse(fallback_source(), gnss_source(), lidar_source());

  expect_same_fusion_result(weighted, expected);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
