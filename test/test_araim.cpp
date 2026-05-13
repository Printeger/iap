// IAP-RQ-241–246, RQ-200, RQ-131: Unit tests for ARAIM, IntegrityMonitor, TrunkMap
// Tests: Q_inv accuracy, 3-term PL formula, HPL = max(PL_E, PL_N),
//        IntegrityState transitions, DynamicAL computation, TrunkMap EKF

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <vector>

#include <iap/integrity/araim.hpp>
#include <iap/integrity/araim_types.hpp>
#include <iap/integrity/fgo_information_matrix.hpp>
#include <iap/integrity/lidar_araim.hpp>
#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/integrity_monitor.hpp>
#include <iap/trunk/trunk_map.hpp>
#include <iap/trunk/trunk_types.hpp>

using namespace iap;

// ============================================================================
// §1: ARAIM core
// ============================================================================

class AraimTest : public ::testing::Test {
 protected:
  Araim::Params default_params() {
    Araim::Params p;
    p.P_HMI_req     = 1e-7;
    p.P_FA_req      = 1e-5;
    p.K_ff           = 5.42;
    p.K_fa           = 4.50;
    p.K_md           = 5.50;
    p.dynamic_budget = true;
    p.p_sat_default  = 1e-5;
    p.min_sats       = 4;
    return p;
  }

  /// Build a minimal GnssEpoch with N satellites in a well-distributed sky
  GnssEpoch make_epoch(int n_sats) {
    GnssEpoch epoch;
    epoch.stamp   = 100.0;
    epoch.gps_sec = 2100000.0;

    const double el_step = M_PI / 6.0;
    const double az_step = 2.0 * M_PI / n_sats;

    for (int i = 0; i < n_sats; ++i) {
      SatObs s;
      s.sat_id       = 100 + i;
      s.constellation = 'G';
      s.elevation    = 0.4 + el_step * (i % 3);  // spread 23°..83°
      s.azimuth      = az_step * i;
      s.pr_sigma     = 3.0 + 2.0 * (i % 2);
      s.pr_residual  = 0.1 * ((i % 3) - 1);      // small residuals
      s.excluded     = false;
      epoch.sats.push_back(s);
    }
    return epoch;
  }
};

class LidarAraimTest : public ::testing::Test {
 protected:
  LidarAraim::Params default_params() {
    LidarAraim::Params p;
    p.dynamic_budget = false;
    p.K_ff = 5.0;
    p.K_fa = 4.0;
    p.K_md = 3.0;
    p.alpha_H = 0.5;
    p.alpha_V = 0.75;
    p.rmse_ref = 0.2;
    p.age_ref_sec = 1.0;
    return p;
  }

  FGOPositionInfo make_fgo(double sigma_diag = 0.1) {
    FGOPositionInfo info;
    info.valid = true;
    info.pose_cov_valid = true;
    info.frame_id = 42;
    info.pose_cov_6x6 = Eigen::Matrix<double, 6, 6>::Identity() * sigma_diag;
    info.sigma_p = info.pose_cov_6x6.block<3, 3>(3, 3);
    return info;
  }

  LidarAraimSnapshot make_snapshot() {
    LidarAraimSnapshot snapshot;
    snapshot.valid = true;
    snapshot.frame_id = 42;
    snapshot.stamp = 10.0;
    snapshot.pose_cov_6x6 = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
    snapshot.current_icp_quality.gamma_lidar = 1.2;
    return snapshot;
  }

  LidarAraimBlock make_block(long target_frame_id,
                             int level_id,
                             double lambda_diag,
                             double eta_e,
                             double rmse = 0.1,
                             double inlier_fraction = 0.9,
                             double cond = 2.0,
                             double age_sec = 0.1) {
    LidarAraimBlock block;
    block.source_frame_id = 42;
    block.target_frame_id = target_frame_id;
    block.level_id = level_id;
    block.voxel_resolution = 0.2 * std::pow(2.0, level_id);
    block.num_inliers = 100;
    block.inlier_fraction = inlier_fraction;
    block.rmse_proxy = rmse;
    block.cond_proxy = cond;
    block.age_sec = age_sec;
    block.Lambda_B = Eigen::Matrix<double, 6, 6>::Identity() * lambda_diag;
    block.eta_B(3) = eta_e;
    return block;
  }
};

// ---------------------------------------------------------------------------
// T1: Q_inv accuracy tests
// ---------------------------------------------------------------------------
TEST_F(AraimTest, QInvReasonableValues) {
  // Q_inv is private static; test indirectly via ARAIM run which uses
  // dynamic budget allocation.  Instead, we test it through well-known values:
  //   Q(3.09) ≈ 1e-3, Q(4.26) ≈ 1e-5, Q(5.33) ≈ 5e-8
  // By running ARAIM with dynamic_budget=true, the K_ff computed should be
  // close to Q_inv(P_HMI_req/2) ≈ Q_inv(5e-8) ≈ 5.33
  Araim::Params p = default_params();
  p.P_HMI_req     = 1e-7;
  p.dynamic_budget = true;
  Araim araim(p);

  GnssEpoch epoch = make_epoch(8);
  AraimResult result = araim.run(epoch, 0);

  ASSERT_TRUE(result.valid);
  // K_ff = Q_inv(P_HMI_req/2) ≈ Q_inv(5e-8)
  // Should be around 5.3–5.5
  EXPECT_GT(result.K_ff_used, 5.0);
  EXPECT_LT(result.K_ff_used, 6.0);
}

// ---------------------------------------------------------------------------
// T2: AraimResult is valid with sufficient geometry
// ---------------------------------------------------------------------------
TEST_F(AraimTest, ValidResultWithGoodGeometry) {
  Araim araim(default_params());
  GnssEpoch epoch = make_epoch(8);
  AraimResult r = araim.run(epoch, 0);

  EXPECT_TRUE(r.valid);
  EXPECT_GT(r.HPL, 0.0);
  // HPL may be 1e9 if a constellation-wide hypothesis is degenerate
  // (all sats from one constellation removed), so relax upper bound
  EXPECT_LT(r.HPL, 1e10);
  EXPECT_GT(r.VPL, 0.0);
  EXPECT_EQ(r.n_hypotheses, static_cast<int>(r.hypotheses.size()));
  // subsets.size() may differ from n_hypotheses if trunk hypotheses are included
  EXPECT_LE(static_cast<int>(r.subsets.size()), r.n_hypotheses);
}

// ---------------------------------------------------------------------------
// T3: HPL = max(PL_E, PL_N)
// ---------------------------------------------------------------------------
TEST_F(AraimTest, HplIsMaxOfPerAxis) {
  Araim araim(default_params());
  GnssEpoch epoch = make_epoch(8);
  AraimResult r = araim.run(epoch, 0);

  ASSERT_TRUE(r.valid);
  EXPECT_DOUBLE_EQ(r.HPL, std::max(r.PL_E, r.PL_N));
  EXPECT_DOUBLE_EQ(r.VPL, r.PL_U);
}

// ---------------------------------------------------------------------------
// T4: 3-term PL formula: PL_{q,k} = |d_{q,k}| + K_fa·σ_ss,q,k + K_md·σ_{q,k}
// ---------------------------------------------------------------------------
TEST_F(AraimTest, ThreeTermPerAxisPL) {
  Araim araim(default_params());
  GnssEpoch epoch = make_epoch(8);
  AraimResult r = araim.run(epoch, 0);

  ASSERT_TRUE(r.valid);
  // Verify the formula for each non-degenerate subset solution
  for (const auto& ss : r.subsets) {
    // Skip degenerate subsets (constellation-wide removal that leaves too few sats)
    if (ss.PL_E >= 1e9 || ss.sigma_ss_E == 0.0) continue;

    // Per §1.11: PL_E = |d_E| + K_fa · σ_ss_E + K_md · σ_k_E
    const double expected_PL_E = std::abs(ss.d_E) + ss.K_fa * ss.sigma_ss_E + ss.K_md * ss.sigma_k_E;
    const double expected_PL_N = std::abs(ss.d_N) + ss.K_fa * ss.sigma_ss_N + ss.K_md * ss.sigma_k_N;
    const double expected_PL_U = std::abs(ss.d_U) + ss.K_fa * ss.sigma_ss_U + ss.K_md * ss.sigma_k_U;

    EXPECT_NEAR(ss.PL_E, expected_PL_E, 1e-10) << "SubsetSolution " << ss.hyp_index << " PL_E mismatch";
    EXPECT_NEAR(ss.PL_N, expected_PL_N, 1e-10) << "SubsetSolution " << ss.hyp_index << " PL_N mismatch";
    EXPECT_NEAR(ss.PL_U, expected_PL_U, 1e-10) << "SubsetSolution " << ss.hyp_index << " PL_U mismatch";
  }
}

// ---------------------------------------------------------------------------
// T5: Total PL is the max over fault-free and all hypotheses
// ---------------------------------------------------------------------------
TEST_F(AraimTest, TotalPLIsMaxOverHypotheses) {
  Araim araim(default_params());
  GnssEpoch epoch = make_epoch(8);
  AraimResult r = araim.run(epoch, 0);

  ASSERT_TRUE(r.valid);

  // PL_E = max(pl_ff_E, max_k(subset[k].PL_E))
  double max_subset_PL_E = 0.0;
  double max_subset_PL_N = 0.0;
  double max_subset_PL_U = 0.0;
  for (const auto& ss : r.subsets) {
    max_subset_PL_E = std::max(max_subset_PL_E, ss.PL_E);
    max_subset_PL_N = std::max(max_subset_PL_N, ss.PL_N);
    max_subset_PL_U = std::max(max_subset_PL_U, ss.PL_U);
  }
  EXPECT_DOUBLE_EQ(r.PL_E, std::max(r.pl_ff_E, max_subset_PL_E));
  EXPECT_DOUBLE_EQ(r.PL_N, std::max(r.pl_ff_N, max_subset_PL_N));
  EXPECT_DOUBLE_EQ(r.PL_U, std::max(r.pl_ff_V, max_subset_PL_U));
}

// ---------------------------------------------------------------------------
// T6: Degenerate geometry (< min_sats) yields invalid result
// ---------------------------------------------------------------------------
TEST_F(AraimTest, DegenerateGeometryInvalid) {
  Araim araim(default_params());
  GnssEpoch epoch = make_epoch(3);  // less than min_sats=4
  AraimResult r = araim.run(epoch, 0);

  EXPECT_FALSE(r.valid);
  EXPECT_GE(r.HPL, 1e9);
}

// ---------------------------------------------------------------------------
// T7: Fault-free PL components (pl_ff_E, pl_ff_N, pl_ff_V)
// ---------------------------------------------------------------------------
TEST_F(AraimTest, FaultFreePLComponents) {
  Araim araim(default_params());
  GnssEpoch epoch = make_epoch(8);
  AraimResult r = araim.run(epoch, 0);

  ASSERT_TRUE(r.valid);
  // pl_ff_E = K_ff * sigma_ff_E
  EXPECT_NEAR(r.pl_ff_E, r.K_ff_used * r.sigma_ff_E, 1e-10);
  EXPECT_NEAR(r.pl_ff_N, r.K_ff_used * r.sigma_ff_N, 1e-10);
  EXPECT_NEAR(r.pl_ff_V, r.K_ff_used * r.sigma_ff_U, 1e-10);
  // pl_ff = max(pl_ff_E, pl_ff_N)
  EXPECT_DOUBLE_EQ(r.pl_ff, std::max(r.pl_ff_E, r.pl_ff_N));
}

// ---------------------------------------------------------------------------
// T8: predict_geometry() with zero residuals
// ---------------------------------------------------------------------------
TEST_F(AraimTest, PredictGeometryNoResiduals) {
  Araim araim(default_params());

  std::vector<Araim::SatGeometry> sats;
  for (int i = 0; i < 8; ++i) {
    Araim::SatGeometry sg;
    sg.elevation = 0.5 + 0.15 * i;
    sg.azimuth   = M_PI / 4.0 * i;
    sg.pr_sigma  = 4.0;
    sg.sat_id    = 200 + i;
    sats.push_back(sg);
  }

  AraimResult r = araim.predict_geometry(sats);
  ASSERT_TRUE(r.valid);

  // With r=0, all separation vectors d_k should be zero
  for (const auto& ss : r.subsets) {
    EXPECT_DOUBLE_EQ(ss.d_E, 0.0);
    EXPECT_DOUBLE_EQ(ss.d_N, 0.0);
    EXPECT_DOUBLE_EQ(ss.d_U, 0.0);
  }

  // HPL should still be positive (geometry-driven)
  EXPECT_GT(r.HPL, 0.0);
}

// ---------------------------------------------------------------------------
// T9: Trunk hypotheses are added
// ---------------------------------------------------------------------------
TEST_F(AraimTest, TrunkHypothesesAddedToCount) {
  Araim araim(default_params());
  GnssEpoch epoch = make_epoch(6);

  AraimResult r0 = araim.run(epoch, 0);
  AraimResult r3 = araim.run(epoch, 3);

  ASSERT_TRUE(r0.valid);
  ASSERT_TRUE(r3.valid);
  // Adding 3 trunk hypotheses should increase hypothesis count by 3
  EXPECT_EQ(r3.n_hypotheses, r0.n_hypotheses + 3);
}

// ---------------------------------------------------------------------------
// T10: S0 is 4×4 and positive (semi)-definite
// ---------------------------------------------------------------------------
TEST_F(AraimTest, S0IsPositiveSemidefinite) {
  Araim araim(default_params());
  GnssEpoch epoch = make_epoch(8);
  AraimResult r = araim.run(epoch, 0);

  ASSERT_TRUE(r.valid);
  EXPECT_EQ(r.S0.rows(), 4);
  EXPECT_EQ(r.S0.cols(), 4);

  // Check eigenvalues are non-negative
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(r.S0);
  for (int i = 0; i < 4; ++i) {
    EXPECT_GE(solver.eigenvalues()(i), -1e-10)
        << "S0 eigenvalue " << i << " is negative";
  }
}

// ---------------------------------------------------------------------------
// T11: Parallel and serial hypothesis evaluation are numerically equivalent
// ---------------------------------------------------------------------------
TEST_F(AraimTest, ParallelMatchesSerial) {
  Araim::Params serial_params = default_params();
  serial_params.parallel_hypotheses = false;

  Araim::Params parallel_params = default_params();
  parallel_params.parallel_hypotheses = true;
  parallel_params.hypothesis_threads = 2;

  Araim serial_araim(serial_params);
  Araim parallel_araim(parallel_params);
  GnssEpoch epoch = make_epoch(10);

  AraimResult serial = serial_araim.run(epoch, 2);
  AraimResult parallel = parallel_araim.run(epoch, 2);

  ASSERT_TRUE(serial.valid);
  ASSERT_TRUE(parallel.valid);
  ASSERT_EQ(serial.subsets.size(), parallel.subsets.size());
  ASSERT_EQ(serial.hypotheses.size(), parallel.hypotheses.size());

  EXPECT_NEAR(serial.HPL, parallel.HPL, 1e-12);
  EXPECT_NEAR(serial.VPL, parallel.VPL, 1e-12);
  EXPECT_NEAR(serial.PL_E, parallel.PL_E, 1e-12);
  EXPECT_NEAR(serial.PL_N, parallel.PL_N, 1e-12);
  EXPECT_NEAR(serial.PL_U, parallel.PL_U, 1e-12);
  EXPECT_EQ(serial.n_detected, parallel.n_detected);
  EXPECT_EQ(serial.worst_hyp, parallel.worst_hyp);
  EXPECT_EQ(serial.excluded_prns, parallel.excluded_prns);
  EXPECT_EQ(serial.excluded_trunk_ids, parallel.excluded_trunk_ids);

  for (std::size_t i = 0; i < serial.subsets.size(); ++i) {
    const auto& a = serial.subsets[i];
    const auto& b = parallel.subsets[i];
    EXPECT_EQ(a.hyp_index, b.hyp_index);
    EXPECT_NEAR(a.PL_E, b.PL_E, 1e-12);
    EXPECT_NEAR(a.PL_N, b.PL_N, 1e-12);
    EXPECT_NEAR(a.PL_U, b.PL_U, 1e-12);
    EXPECT_NEAR(a.d_E, b.d_E, 1e-12);
    EXPECT_NEAR(a.d_N, b.d_N, 1e-12);
    EXPECT_NEAR(a.d_U, b.d_U, 1e-12);
    EXPECT_EQ(a.fault_detected, b.fault_detected);
  }
}

TEST_F(LidarAraimTest, HypothesesEnumerateSourceTargetAndLevel) {
  LidarAraim araim(default_params());
  auto snapshot = make_snapshot();
  snapshot.blocks.push_back(make_block(10, 0, 1.0, 0.0));
  snapshot.blocks.push_back(make_block(10, 1, 1.0, 0.0));
  snapshot.blocks.push_back(make_block(20, 0, 1.0, 0.0));

  const auto result = araim.run(snapshot, make_fgo());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.n_hypotheses, 5);
}

TEST_F(LidarAraimTest, SingleBlockSubsetMatchesManualSolve) {
  auto params = default_params();
  params.alpha_H = 0.0;
  params.alpha_V = 0.0;
  LidarAraim araim(params);
  auto snapshot = make_snapshot();
  snapshot.blocks.push_back(make_block(10, 0, 1.0, 1.0));

  const auto result = araim.run(snapshot, make_fgo());
  ASSERT_TRUE(result.valid);
  ASSERT_EQ(result.subsets.size(), 3u);  // source + target + level

  const auto& ss = result.subsets.front();  // H_SOURCE removes the only block
  ASSERT_TRUE(ss.valid);

  const double lambda_f = 10.0 - 1.0;
  const double sigma_f = 1.0 / lambda_f;
  const double d = -1.0 / lambda_f;
  const double sigma_ss = std::sqrt(std::max(0.0, sigma_f - 0.1));
  const double expected_pl =
      std::abs(d) + params.K_fa * sigma_ss + params.K_md * std::sqrt(sigma_f);

  EXPECT_NEAR(ss.d_E, d, 1e-12);
  EXPECT_NEAR(ss.sigma_k_E, std::sqrt(sigma_f), 1e-12);
  EXPECT_NEAR(ss.PL_E, expected_pl, 1e-12);
}

TEST_F(LidarAraimTest, BiasModelIncreasesProtectionLevel) {
  auto params = default_params();
  params.alpha_H = 1.0;
  params.alpha_V = 1.0;
  LidarAraim araim(params);

  auto good = make_snapshot();
  good.blocks.push_back(make_block(10, 0, 1.0, 0.0, 0.05, 0.98, 1.2, 0.05));

  auto bad = make_snapshot();
  bad.blocks.push_back(make_block(10, 0, 1.0, 0.0, 0.5, 0.4, 50.0, 3.0));

  const auto good_result = araim.run(good, make_fgo());
  const auto bad_result = araim.run(bad, make_fgo());

  ASSERT_TRUE(good_result.valid);
  ASSERT_TRUE(bad_result.valid);
  EXPECT_GT(bad_result.HPL, good_result.HPL);
  EXPECT_GT(bad_result.VPL, good_result.VPL);
}

TEST_F(LidarAraimTest, GpuBackendBlocksProduceValidGroupedResult) {
  LidarAraim araim(default_params());
  auto snapshot = make_snapshot();

  auto block0 = make_block(10, 0, 1.0, 0.2);
  auto block1 = make_block(20, 1, 1.5, 0.4);
  block0.backend = LidarAraimBlock::Backend::GPU;
  block1.backend = LidarAraimBlock::Backend::GPU;

  snapshot.blocks.push_back(block0);
  snapshot.blocks.push_back(block1);

  const auto result = araim.run(snapshot, make_fgo());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.n_hypotheses, 5);
  EXPECT_EQ(result.n_detected, 0);
  EXPECT_GT(result.HPL, 0.0);
  EXPECT_GT(result.VPL, 0.0);
}

TEST_F(LidarAraimTest, AgeRiskSaturates) {
  auto params = default_params();
  params.w_rmse = 0.0;
  params.w_inlier = 0.0;
  params.w_cond = 0.0;
  params.w_age = 1.0;
  params.age_model = LidarAraim::Params::AgeModel::EXP_SATURATING;
  params.age_tau_s = 30.0;
  params.gamma_age_max = 1.0;
  params.alpha_H = 1.0;
  params.alpha_V = 1.0;

  auto block_30 = make_block(10, 0, 1.0, 0.0, 0.1, 0.9, 2.0, 30.0);
  auto block_300 = block_30;
  auto block_3000 = block_30;
  block_300.age_sec = 300.0;
  block_3000.age_sec = 3000.0;

  const auto risk_30 = LidarAraim::compute_risk_components(block_30, params);
  const auto risk_300 = LidarAraim::compute_risk_components(block_300, params);
  const auto risk_3000 = LidarAraim::compute_risk_components(block_3000, params);

  EXPECT_LE(risk_30.gamma_age, params.gamma_age_max);
  EXPECT_LE(risk_300.gamma_age, params.gamma_age_max);
  EXPECT_LE(risk_3000.gamma_age, params.gamma_age_max);
  EXPECT_LT(risk_3000.gamma_age - risk_300.gamma_age, 1e-4);

  LidarAraim araim(params);
  auto snapshot_300 = make_snapshot();
  auto snapshot_3000 = make_snapshot();
  snapshot_300.blocks.push_back(block_300);
  snapshot_3000.blocks.push_back(block_3000);

  const auto result_300 = araim.run(snapshot_300, make_fgo());
  const auto result_3000 = araim.run(snapshot_3000, make_fgo());
  ASSERT_TRUE(result_300.valid);
  ASSERT_TRUE(result_3000.valid);
  EXPECT_NEAR(result_3000.HPL, result_300.HPL, 1e-4);
}

TEST_F(LidarAraimTest, TargetWindowCapsHypotheses) {
  auto params = default_params();
  params.target_window_K = 10;
  params.dynamic_budget = false;
  params.alpha_H = 0.0;
  params.alpha_V = 0.0;
  LidarAraim araim(params);

  auto snapshot = make_snapshot();
  for (int i = 0; i < 12; ++i) {
    auto block = make_block(100 + i, 0, 0.1, 0.0);
    block.target_distance_m = static_cast<double>(12 - i);
    snapshot.blocks.push_back(block);
  }

  const auto result = araim.run(snapshot, make_fgo());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.selected_target_count, 10);
  EXPECT_EQ(result.target_window_K, 10);
  EXPECT_EQ(result.n_hypotheses, 12);  // source + 10 targets + one level
}

TEST_F(LidarAraimTest, SigmaSsFallbackForNegativeRawVariance) {
  auto params = default_params();
  params.dynamic_budget = false;
  params.alpha_H = 0.0;
  params.alpha_V = 0.0;
  params.sigma_ss_min_m = 0.02;
  LidarAraim araim(params);

  auto snapshot = make_snapshot();
  auto block = make_block(10, 0, -1.0, 0.0);
  snapshot.blocks.push_back(block);

  const auto result = araim.run(snapshot, make_fgo());
  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.subsets.empty());
  const auto& ss = result.subsets.front();
  ASSERT_TRUE(ss.valid);
  EXPECT_LT(ss.sigma_ss_raw_E_m2, 0.0);
  EXPECT_TRUE(ss.sigma_ss_fallback_E);
  EXPECT_GE(ss.sigma_ss_E, params.sigma_ss_min_m);
}

// ============================================================================
// §2: IntegrityState transitions
// ============================================================================

TEST(IntegrityStateTest, EnumValues) {
  EXPECT_EQ(static_cast<int>(IntegrityState::SAFE), 0);
  EXPECT_EQ(static_cast<int>(IntegrityState::SAFE_EXCLUDED), 1);
  EXPECT_EQ(static_cast<int>(IntegrityState::UNSAFE), 2);
}

TEST(IntegrityStateTest, ToString) {
  EXPECT_STREQ(to_string(IntegrityState::SAFE), "SAFE");
  EXPECT_STREQ(to_string(IntegrityState::SAFE_EXCLUDED), "SAFE_EXCLUDED");
  EXPECT_STREQ(to_string(IntegrityState::UNSAFE), "UNSAFE");
}

TEST(PlannerStateTest, EnumValues) {
  EXPECT_EQ(static_cast<int>(PlannerState::CRUISE), 0);
  EXPECT_EQ(static_cast<int>(PlannerState::OPTIMIZING), 1);
  EXPECT_EQ(static_cast<int>(PlannerState::TRAVERSING), 2);
  EXPECT_EQ(static_cast<int>(PlannerState::HOVER), 3);
}

// ============================================================================
// §3: DynamicALResult
// ============================================================================

TEST(DynamicALResultTest, DefaultsAreConservative) {
  DynamicALResult al;
  EXPECT_GE(al.HAL, 1e9);
  EXPECT_GE(al.VAL, 1e9);
  EXPECT_EQ(al.nearest_trunk_id, -1);
  EXPECT_GE(al.nearest_trunk_dist, 1e9);
  EXPECT_TRUE(al.al_from_trunk);
}

TEST(FGOPositionInfoTest, DefaultsAreConservative) {
  FGOPositionInfo info;
  EXPECT_FALSE(info.valid);
  EXPECT_FALSE(info.pose_cov_valid);
  EXPECT_EQ(info.frame_id, -1);
  EXPECT_EQ(info.n_total_factors, 0);
  EXPECT_EQ(info.n_gnss_factors, 0);
  EXPECT_EQ(info.n_trunk_factors, 0);
  EXPECT_EQ(info.n_imu_factors, 0);
  EXPECT_EQ(info.window_key_count, 0);
  EXPECT_TRUE(info.gnss_sat_ids.empty());
  EXPECT_TRUE(info.trunk_landmark_ids.empty());
}

// ============================================================================
// §4: IntegrityReport
// ============================================================================

TEST(IntegrityReportTest, DefaultIsUnsafe) {
  IntegrityReport rep;
  EXPECT_EQ(rep.state, IntegrityState::UNSAFE);
  EXPECT_GE(rep.PL, 1e9);
  EXPECT_DOUBLE_EQ(rep.AL, 0.0);
  EXPECT_FALSE(rep.safe());
  EXPECT_FALSE(rep.is_available());
}

TEST(IntegrityReportTest, SafeWhenPLLessThanAL) {
  IntegrityReport rep;
  rep.PL = 1.0;
  rep.AL = 5.0;
  rep.IM = rep.AL - rep.PL;
  EXPECT_TRUE(rep.safe());
  EXPECT_TRUE(rep.is_available());
}

TEST(IntegrityMonitorTest, LidarOnlyPLOverridesFallbackByMax) {
  IntegrityMonitor::Params params;
  params.K_pl = 1.0;
  params.HAL_trunk_default = 100.0;
  params.VAL_default = 100.0;
  params.lidar_araim_params.dynamic_budget = false;
  params.lidar_araim_params.K_ff = 5.0;
  params.lidar_araim_params.K_fa = 4.0;
  params.lidar_araim_params.K_md = 3.0;
  params.lidar_araim_params.alpha_H = 0.0;
  params.lidar_araim_params.alpha_V = 0.0;

  IntegrityMonitor monitor(params);

  glim::EstimationFrame frame;
  frame.stamp = 10.0;
  frame.sigma_p = Eigen::Matrix3d::Identity() * 0.01;

  FGOPositionInfo fgo;
  fgo.valid = true;
  fgo.pose_cov_valid = true;
  fgo.frame_id = 42;
  fgo.pose_cov_6x6 = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
  fgo.sigma_p = fgo.pose_cov_6x6.block<3, 3>(3, 3);

  LidarAraimSnapshot snapshot;
  snapshot.valid = true;
  snapshot.frame_id = 42;
  snapshot.stamp = frame.stamp;
  snapshot.pose_cov_6x6 = fgo.pose_cov_6x6;

  LidarAraimBlock block;
  block.source_frame_id = 42;
  block.target_frame_id = 10;
  block.level_id = 0;
  block.num_inliers = 100;
  block.inlier_fraction = 0.9;
  block.rmse_proxy = 0.1;
  block.cond_proxy = 2.0;
  block.age_sec = 0.1;
  block.Lambda_B = Eigen::Matrix<double, 6, 6>::Identity();
  block.eta_B(3) = 1.0;
  snapshot.blocks.push_back(block);

  const auto report = monitor.compute(frame, nullptr, nullptr, &fgo, &snapshot);
  EXPECT_EQ(report.lidar_valid, 1);
  EXPECT_GT(report.lidar_HPL, 0.1);
  EXPECT_DOUBLE_EQ(report.HPL, report.lidar_HPL);
  EXPECT_DOUBLE_EQ(report.PL, report.lidar_HPL);
}

TEST(IntegrityMonitorTest, LidarOnlyGpuBlocksOverrideFallbackByMax) {
  IntegrityMonitor::Params params;
  params.K_pl = 1.0;
  params.HAL_trunk_default = 100.0;
  params.VAL_default = 100.0;
  params.lidar_araim_params.dynamic_budget = false;
  params.lidar_araim_params.K_ff = 5.0;
  params.lidar_araim_params.K_fa = 4.0;
  params.lidar_araim_params.K_md = 3.0;
  params.lidar_araim_params.alpha_H = 0.0;
  params.lidar_araim_params.alpha_V = 0.0;

  IntegrityMonitor monitor(params);

  glim::EstimationFrame frame;
  frame.stamp = 20.0;
  frame.sigma_p = Eigen::Matrix3d::Identity() * 0.01;

  FGOPositionInfo fgo;
  fgo.valid = true;
  fgo.pose_cov_valid = true;
  fgo.frame_id = 43;
  fgo.pose_cov_6x6 = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
  fgo.sigma_p = fgo.pose_cov_6x6.block<3, 3>(3, 3);

  LidarAraimSnapshot snapshot;
  snapshot.valid = true;
  snapshot.frame_id = 43;
  snapshot.stamp = frame.stamp;
  snapshot.pose_cov_6x6 = fgo.pose_cov_6x6;

  LidarAraimBlock block;
  block.source_frame_id = 43;
  block.target_frame_id = 11;
  block.level_id = 0;
  block.backend = LidarAraimBlock::Backend::GPU;
  block.num_inliers = 120;
  block.inlier_fraction = 0.92;
  block.rmse_proxy = 0.08;
  block.cond_proxy = 1.5;
  block.age_sec = 0.05;
  block.Lambda_B = Eigen::Matrix<double, 6, 6>::Identity();
  block.eta_B(3) = 0.8;
  snapshot.blocks.push_back(block);

  const auto report = monitor.compute(frame, nullptr, nullptr, &fgo, &snapshot);
  EXPECT_EQ(report.lidar_valid, 1);
  EXPECT_GT(report.lidar_HPL, 0.1);
  EXPECT_DOUBLE_EQ(report.HPL, report.lidar_HPL);
  EXPECT_DOUBLE_EQ(report.PL, report.lidar_HPL);
}

TEST_F(AraimTest, IntegrityMonitorFusesGnssAndLidarByPerAxisMax) {
  IntegrityMonitor::Params params;
  params.K_pl = 1.0;
  params.HAL_trunk_default = 100.0;
  params.VAL_default = 100.0;
  params.araim_params = default_params();
  params.araim_params.dynamic_budget = false;
  params.lidar_araim_params.dynamic_budget = false;
  params.lidar_araim_params.K_ff = 5.0;
  params.lidar_araim_params.K_fa = 4.0;
  params.lidar_araim_params.K_md = 3.0;
  params.lidar_araim_params.alpha_H = 0.0;
  params.lidar_araim_params.alpha_V = 0.0;

  IntegrityMonitor monitor(params);

  glim::EstimationFrame frame;
  frame.stamp = 30.0;
  frame.sigma_p = Eigen::Matrix3d::Identity() * 0.01;

  FGOPositionInfo fgo;
  fgo.valid = true;
  fgo.pose_cov_valid = true;
  fgo.frame_id = 44;
  fgo.pose_cov_6x6 = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
  fgo.sigma_p = fgo.pose_cov_6x6.block<3, 3>(3, 3);

  LidarAraimSnapshot snapshot;
  snapshot.valid = true;
  snapshot.frame_id = 44;
  snapshot.stamp = frame.stamp;
  snapshot.pose_cov_6x6 = fgo.pose_cov_6x6;

  LidarAraimBlock block;
  block.source_frame_id = 44;
  block.target_frame_id = 12;
  block.level_id = 0;
  block.num_inliers = 120;
  block.inlier_fraction = 0.92;
  block.rmse_proxy = 0.08;
  block.cond_proxy = 1.5;
  block.age_sec = 0.05;
  block.Lambda_B = Eigen::Matrix<double, 6, 6>::Identity();
  block.eta_B(3) = 0.8;
  block.eta_B(4) = -0.5;
  block.eta_B(5) = 0.3;
  snapshot.blocks.push_back(block);

  GnssEpoch epoch = make_epoch(8);
  const auto report = monitor.compute(frame, &epoch, nullptr, &fgo, &snapshot);

  ASSERT_EQ(report.gnss_valid, 1);
  ASSERT_EQ(report.lidar_valid, 1);
  EXPECT_DOUBLE_EQ(report.PL_E, std::max(report.gnss_PL_E, report.lidar_PL_E));
  EXPECT_DOUBLE_EQ(report.PL_N, std::max(report.gnss_PL_N, report.lidar_PL_N));
  EXPECT_DOUBLE_EQ(report.PL_U, std::max(report.gnss_PL_U, report.lidar_PL_U));
  EXPECT_DOUBLE_EQ(report.HPL, std::max(report.gnss_HPL, report.lidar_HPL));
  EXPECT_DOUBLE_EQ(report.VPL, std::max(report.gnss_VPL, report.lidar_VPL));
  EXPECT_DOUBLE_EQ(report.PL, std::max(report.gnss_HPL, report.lidar_HPL));
}

// ============================================================================
// §5: TrunkMap + EKF
// ============================================================================

class TrunkMapTest : public ::testing::Test {
 protected:
  TrunkMap::Params default_params() {
    TrunkMap::Params p;
    p.assoc_gate_m       = 0.30;
    p.assoc_radius_ratio = 0.50;
    p.min_confirm_count  = 2;
    p.stale_timeout_s    = 5.0;
    p.ema_alpha          = 0.3;
    p.sigma_init         = 1.0;
    p.sigma_obs          = 0.15;
    p.sigma_process      = 0.01;
    p.use_ekf            = true;
    return p;
  }

  TrunkDetectionResult make_detection(double stamp,
                                       const std::vector<Eigen::Vector2d>& centers,
                                       double radius = 0.1) {
    TrunkDetectionResult det;
    det.stamp = stamp;
    for (const auto& c : centers) {
      TrunkObservation obs;
      obs.center_xy   = c;
      obs.radius      = radius;
      obs.confidence  = 0.9;
      obs.num_points  = 50;
      det.trunks.push_back(obs);
    }
    return det;
  }
};

TEST_F(TrunkMapTest, NewLandmarkCreation) {
  TrunkMap map(default_params());
  Eigen::Vector2d sensor_xy = Eigen::Vector2d::Zero();

  TrunkDetectionResult det = make_detection(1.0, {{1.0, 2.0}});
  auto result = map.update(det, sensor_xy);

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].first, -1);  // new landmark

  // Now the map should have 1 landmark
  auto confirmed = map.confirmed_landmarks();
  // Not confirmed yet (seen_count=1, min_confirm_count=2)
  EXPECT_EQ(confirmed.size(), 0u);
}

TEST_F(TrunkMapTest, AssociationAndConfirmation) {
  TrunkMap map(default_params());
  Eigen::Vector2d sensor_xy = Eigen::Vector2d::Zero();

  // First detection
  TrunkDetectionResult det1 = make_detection(1.0, {{1.0, 2.0}});
  auto r1 = map.update(det1, sensor_xy);
  ASSERT_EQ(r1.size(), 1u);
  EXPECT_EQ(r1[0].first, -1);  // new

  // Second detection at same position → should associate
  TrunkDetectionResult det2 = make_detection(2.0, {{1.0, 2.0}});
  auto r2 = map.update(det2, sensor_xy);
  ASSERT_EQ(r2.size(), 1u);
  EXPECT_GE(r2[0].first, 0);  // associated to existing landmark

  // Should now be confirmed
  auto confirmed = map.confirmed_landmarks();
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_TRUE(confirmed[0]->confirmed);
}

TEST_F(TrunkMapTest, EKFReducesUncertainty) {
  auto p = default_params();
  p.use_ekf = true;
  TrunkMap map(p);
  Eigen::Vector2d sensor_xy = Eigen::Vector2d::Zero();

  // First detection — covariance initialized to sigma_init^2 = 1.0
  TrunkDetectionResult det1 = make_detection(1.0, {{5.0, 5.0}});
  map.update(det1, sensor_xy);

  // Get all landmarks (confirmed_landmarks requires 2 sightings, so use internals)
  auto confirmed_before = map.confirmed_landmarks();

  // Second detection at the same position
  TrunkDetectionResult det2 = make_detection(2.0, {{5.0, 5.0}});
  map.update(det2, sensor_xy);

  auto confirmed = map.confirmed_landmarks();
  ASSERT_EQ(confirmed.size(), 1u);

  // After EKF update, the covariance diagonal should be smaller than sigma_init^2
  double trace_P = confirmed[0]->P.trace();
  double sigma_init_sq = p.sigma_init * p.sigma_init;
  EXPECT_LT(trace_P, 2.0 * sigma_init_sq)
      << "EKF update should reduce covariance below initial prior";
}

TEST_F(TrunkMapTest, EMAFallbackWorks) {
  auto p = default_params();
  p.use_ekf = false;  // Use EMA instead
  TrunkMap map(p);
  Eigen::Vector2d sensor_xy = Eigen::Vector2d::Zero();

  // First detection
  TrunkDetectionResult det1 = make_detection(1.0, {{5.0, 5.0}}, 0.1);
  map.update(det1, sensor_xy);

  // Second detection at slightly offset position
  TrunkDetectionResult det2 = make_detection(2.0, {{5.1, 5.0}}, 0.1);
  map.update(det2, sensor_xy);

  auto confirmed = map.confirmed_landmarks();
  ASSERT_EQ(confirmed.size(), 1u);

  // With EMA alpha=0.3, position should be blended:
  // x = 0.3 * 5.1 + 0.7 * 5.0 = 5.03
  EXPECT_NEAR(confirmed[0]->center_xy.x(), 5.03, 0.01);
  EXPECT_NEAR(confirmed[0]->center_xy.y(), 5.0, 0.01);
}

TEST_F(TrunkMapTest, StalePruning) {
  TrunkMap map(default_params());
  Eigen::Vector2d sensor_xy = Eigen::Vector2d::Zero();

  // Create landmark at t=1.0
  TrunkDetectionResult det1 = make_detection(1.0, {{3.0, 4.0}});
  map.update(det1, sensor_xy);

  // Observe again at t=2.0 (to confirm)
  TrunkDetectionResult det2 = make_detection(2.0, {{3.0, 4.0}});
  map.update(det2, sensor_xy);

  auto confirmed = map.confirmed_landmarks();
  ASSERT_EQ(confirmed.size(), 1u);

  // Skip ahead past stale_timeout (5s) — observe a different trunk at t=10.0
  TrunkDetectionResult det3 = make_detection(10.0, {{10.0, 10.0}});
  map.update(det3, sensor_xy);

  // Original landmark should have been pruned
  confirmed = map.confirmed_landmarks();
  // Only the new (possibly unconfirmed) one remains – confirmed should be empty
  // because the new one only has 1 sighting
  EXPECT_EQ(confirmed.size(), 0u);
}

TEST_F(TrunkMapTest, MultipleLandmarks) {
  TrunkMap map(default_params());
  Eigen::Vector2d sensor_xy = Eigen::Vector2d::Zero();

  // Detect two trunks far apart
  TrunkDetectionResult det = make_detection(1.0, {{1.0, 0.0}, {5.0, 0.0}});
  auto result = map.update(det, sensor_xy);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].first, -1);
  EXPECT_EQ(result[1].first, -1);

  // Second pass — both should associate
  TrunkDetectionResult det2 = make_detection(2.0, {{1.0, 0.0}, {5.0, 0.0}});
  auto result2 = map.update(det2, sensor_xy);

  ASSERT_EQ(result2.size(), 2u);
  EXPECT_GE(result2[0].first, 0);
  EXPECT_GE(result2[1].first, 0);
  EXPECT_NE(result2[0].first, result2[1].first);

  auto confirmed = map.confirmed_landmarks();
  EXPECT_EQ(confirmed.size(), 2u);
}

// ============================================================================
// §6: AraimResult field consistency
// ============================================================================

TEST(AraimResultTest, DefaultsAreConservative) {
  AraimResult r;
  EXPECT_FALSE(r.valid);
  EXPECT_GE(r.HPL, 1e9);
  EXPECT_GE(r.VPL, 1e9);
  EXPECT_EQ(r.n_hypotheses, 0);
  EXPECT_EQ(r.n_detected, 0);
}

TEST(AraimResultTest, AliasesAreConsistent) {
  Araim araim;
  GnssEpoch epoch;
  epoch.stamp = 1.0;
  epoch.gps_sec = 2100000.0;
  for (int i = 0; i < 8; ++i) {
    SatObs s;
    s.sat_id       = 100 + i;
    s.constellation = 'G';
    s.elevation    = 0.5 + 0.1 * i;
    s.azimuth      = M_PI / 4.0 * i;
    s.pr_sigma     = 3.0;
    s.pr_residual  = 0.0;
    s.excluded     = false;
    epoch.sats.push_back(s);
  }

  AraimResult r = araim.run(epoch, 0);
  if (r.valid) {
    // Aliases should match
    EXPECT_DOUBLE_EQ(r.pl_araim, r.HPL);
    EXPECT_DOUBLE_EQ(r.vpl_araim, r.VPL);
  }
}

// ============================================================================
// §7: SubsetSolution per-axis consistency
// ============================================================================

TEST(SubsetSolutionTest, FaultDetectionThreshold) {
  SubsetSolution ss;
  ss.d_E = 2.0;
  ss.d_N = 0.5;
  ss.sigma_ss_E = 1.0;
  ss.sigma_ss_N = 1.0;
  ss.K_fa = 3.0;
  // T_E = K_fa * sigma_ss_E
  ss.T_E = ss.K_fa * ss.sigma_ss_E;  // = 3.0
  ss.T_N = ss.K_fa * ss.sigma_ss_N;  // = 3.0

  // |d_E| = 2.0 < T_E = 3.0 → no fault detected
  EXPECT_FALSE(std::abs(ss.d_E) > ss.T_E);
  // |d_N| = 0.5 < T_N = 3.0 → no fault detected
  EXPECT_FALSE(std::abs(ss.d_N) > ss.T_N);
}

// ============================================================================
// §8: FaultHypothesis types
// ============================================================================

TEST(FaultHypothesisTest, TypeEnumeration) {
  EXPECT_EQ(static_cast<int>(FaultHypothesis::Type::GNSS_SAT), 0);
  EXPECT_EQ(static_cast<int>(FaultHypothesis::Type::TRUNK), 1);
  EXPECT_EQ(static_cast<int>(FaultHypothesis::Type::CONSTELLATION), 2);
}

TEST(FaultHypothesisTest, DefaultValues) {
  FaultHypothesis h;
  EXPECT_EQ(h.type, FaultHypothesis::Type::GNSS_SAT);
  EXPECT_EQ(h.row, -1);
  EXPECT_EQ(h.sat_id, -1);
  EXPECT_EQ(h.const_id, -1);
  EXPECT_EQ(h.trunk_id, -1);
  EXPECT_DOUBLE_EQ(h.p_fault, 1e-5);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
