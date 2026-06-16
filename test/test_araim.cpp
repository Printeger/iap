// IAP-RQ-241–246, RQ-200, RQ-131: Unit tests for ARAIM, IntegrityMonitor, TrunkMap
// Tests: Q_inv accuracy, 3-term PL formula, HPL = max(PL_E, PL_N),
//        IntegrityState transitions, DynamicAL computation, TrunkMap EKF

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <iap/integrity/araim.hpp>
#include <iap/integrity/araim_types.hpp>
#include <iap/integrity/fgo_information_matrix.hpp>
#include <iap/integrity/lidar_araim.hpp>
#include <iap/integrity/integrity_types.hpp>
#include <iap/integrity/integrity_monitor.hpp>
#include <iap/integrity/integrity_report_mapping.hpp>
#include <iap/integrity/numerical_guard.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <iap/predictor/lidar_observability_fim.hpp>
#include <iap/trunk/trunk_map.hpp>
#include <iap/trunk/trunk_types.hpp>

using namespace iap;

// ============================================================================
// §1: ARAIM core
// ============================================================================
class GnssAraimEvaluatorTest : public ::testing::Test {
 protected:
  GnssAraimParams default_params() {
    GnssAraimParams p;
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

namespace {

struct SyntheticLidarFimEvaluation {
  LidarAraimResult result;
  double condition_number = 1.0e12;
  std::string failure_reason;
};

struct LidarBlockFaultRun {
  LidarAraimResult result;
  LidarAraimSnapshot snapshot;
};

struct RuntimeBlockBridgeRun {
  std::string name;
  LidarAraimResult result;
  LidarAraimSnapshot snapshot;
  double condition_number = 1.0e12;
};

struct SingleBlockHypothesisObservation {
  int block_id = -1;
  int block_index = -1;
  int hyp_index = -1;
  bool fault_detected = false;
  double d_E = 0.0;
  double T_E = 0.0;
  double PL_E = -std::numeric_limits<double>::infinity();
  double HPL = -std::numeric_limits<double>::infinity();
  std::string mode;
};

struct PointCloudToLidarPlEvaluation {
  std::string name;
  std::vector<Eigen::Vector3d> points;
  std::shared_ptr<std::vector<LidarFimPrimitive>> primitives;
  LidarFimPrimitiveGenerationDiagnostics primitive_diag;
  LidarAdvisoryFimResult fim;
  SyntheticLidarFimEvaluation pl;
  std::string failure_reason;
};

std::string araim_validation_results_path(const std::string& filename) {
  return std::string(IAP_SOURCE_ROOT) + "/results/araim_validation/" + filename;
}

std::string csv_escape(const std::string& value) {
  bool needs_quotes = false;
  for (const char c : value) {
    if (c == ',' || c == '"' || c == '\n' || c == '\r') {
      needs_quotes = true;
      break;
    }
  }
  if (!needs_quotes) {
    return value;
  }
  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') {
      escaped += "\"\"";
    } else {
      escaped += c;
    }
  }
  escaped += "\"";
  return escaped;
}

bool is_finite_lidar_pl(const LidarAraimResult& result) {
  return std::isfinite(result.HPL) &&
         std::isfinite(result.VPL) &&
         std::isfinite(result.PL_E) &&
         std::isfinite(result.PL_N) &&
         std::isfinite(result.PL_U);
}

FGOPositionInfo make_fgo_from_position_fim(const Eigen::Matrix3d& fim,
                                           const long frame_id = 420) {
  FGOPositionInfo info;
  info.valid = true;
  info.pose_cov_valid = true;
  info.frame_id = frame_id;
  info.pose_cov_6x6 = Eigen::Matrix<double, 6, 6>::Identity() * 0.01;
  info.pose_cov_6x6.block<3, 3>(3, 3) = fim.inverse();
  info.sigma_p = info.pose_cov_6x6.block<3, 3>(3, 3);
  return info;
}

LidarAraimSnapshot make_snapshot_with_zero_lidar_block(
    const FGOPositionInfo& fgo,
    const double stamp = 40.0) {
  LidarAraimSnapshot snapshot;
  snapshot.valid = true;
  snapshot.frame_id = fgo.frame_id;
  snapshot.stamp = stamp;
  snapshot.pose_cov_6x6 = fgo.pose_cov_6x6;
  snapshot.current_icp_quality.gamma_lidar = 1.0;

  LidarAraimBlock block;
  block.source_frame_id = fgo.frame_id;
  block.target_frame_id = fgo.frame_id - 1;
  block.level_id = 0;
  block.num_inliers = 100;
  block.inlier_fraction = 1.0;
  block.rmse_proxy = 0.0;
  block.cond_proxy = 1.0;
  block.gamma_lidar = 1.0;
  block.age_sec = 0.0;
  block.target_distance_m = 1.0;
  block.Lambda_B.setZero();
  block.eta_B.setZero();
  snapshot.blocks.push_back(block);

  return snapshot;
}

SyntheticLidarFimEvaluation evaluate_synthetic_lidar_fim(
    const Eigen::Matrix3d& fim) {
  SyntheticLidarFimEvaluation evaluation;
  if (!fim.allFinite()) {
    evaluation.failure_reason = "invalid_lidar_fim";
    return evaluation;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(fim);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) {
    evaluation.failure_reason = "invalid_lidar_fim_eigendecomposition";
    return evaluation;
  }

  const double min_eig = eig.eigenvalues().minCoeff();
  const double max_eig = eig.eigenvalues().maxCoeff();
  evaluation.condition_number =
      min_eig > 0.0 ? max_eig / min_eig : 1.0e12;
  if (min_eig <= 0.0 || max_eig <= 0.0) {
    evaluation.failure_reason = "singular_lidar_fim";
    return evaluation;
  }

  LidarAraim::Params params;
  params.dynamic_budget = false;
  params.K_ff = 5.0;
  params.K_fa = 4.0;
  params.K_md = 3.0;
  params.alpha_H = 0.0;
  params.alpha_V = 0.0;
  LidarAraim evaluator(params);

  const FGOPositionInfo fgo = make_fgo_from_position_fim(fim);
  const LidarAraimSnapshot snapshot = make_snapshot_with_zero_lidar_block(fgo);
  evaluation.result = evaluator.run(snapshot, fgo);
  if (!evaluation.result.valid) {
    evaluation.failure_reason = "LiDAR integrity evaluator returned invalid";
  }
  return evaluation;
}

FGOPositionInfo make_lidar_block_fault_fgo(const long frame_id = 430) {
  FGOPositionInfo info;
  info.valid = true;
  info.pose_cov_valid = true;
  info.frame_id = frame_id;
  info.pose_cov_6x6 = Eigen::Matrix<double, 6, 6>::Identity() * 0.1;
  info.sigma_p = info.pose_cov_6x6.block<3, 3>(3, 3);
  return info;
}

LidarAraimBlock make_lidar_fault_injection_block(const int block_id,
                                                 const double residual_e) {
  LidarAraimBlock block;
  block.source_frame_id = 430;
  block.target_frame_id = block_id;
  block.level_id = block_id;
  block.voxel_resolution = 0.2;
  block.num_inliers = 100;
  block.inlier_fraction = 0.95;
  block.rmse_proxy = std::abs(residual_e);
  block.cond_proxy = 2.0;
  block.gamma_lidar = 1.0;
  block.age_sec = 0.1;
  block.target_distance_m = static_cast<double>(block_id);
  block.Lambda_B = Eigen::Matrix<double, 6, 6>::Identity();
  block.eta_B(3) = residual_e;
  return block;
}

LidarAraimSnapshot make_lidar_block_fault_snapshot(
    const FGOPositionInfo& fgo,
    const std::vector<double>& residuals_e) {
  LidarAraimSnapshot snapshot;
  snapshot.valid = true;
  snapshot.frame_id = fgo.frame_id;
  snapshot.stamp = 50.0;
  snapshot.pose_cov_6x6 = fgo.pose_cov_6x6;
  snapshot.current_icp_quality.gamma_lidar = 1.0;
  for (int i = 0; i < static_cast<int>(residuals_e.size()); ++i) {
    snapshot.blocks.push_back(
        make_lidar_fault_injection_block(i + 1, residuals_e[i]));
  }
  return snapshot;
}

LidarBlockFaultRun run_lidar_block_fault_injection(
    const std::vector<double>& residuals_e) {
  LidarAraim::Params params;
  params.dynamic_budget = false;
  params.K_ff = 5.0;
  params.K_fa = 4.0;
  params.K_md = 3.0;
  params.alpha_H = 0.0;
  params.alpha_V = 0.0;
  LidarAraim evaluator(params);

  LidarBlockFaultRun run;
  const FGOPositionInfo fgo = make_lidar_block_fault_fgo();
  run.snapshot = make_lidar_block_fault_snapshot(fgo, residuals_e);
  run.result = evaluator.run(run.snapshot, fgo);
  return run;
}

LidarAraimBlock make_runtime_bridge_block(
    const long target_frame_id,
    const int level_id,
    const Eigen::Vector3d& lambda_position_diag,
    const double cond_proxy) {
  LidarAraimBlock block;
  block.source_frame_id = 440;
  block.target_frame_id = target_frame_id;
  block.level_id = level_id;
  block.voxel_resolution = 0.2 * std::pow(2.0, level_id);
  block.num_inliers = 100;
  block.inlier_fraction = 0.95;
  block.rmse_proxy = 0.05;
  block.cond_proxy = cond_proxy;
  block.gamma_lidar = 1.0;
  block.age_sec = 0.1;
  block.target_distance_m = static_cast<double>(target_frame_id);
  block.Lambda_B.setZero();
  block.Lambda_B.block<3, 3>(3, 3) = lambda_position_diag.asDiagonal();
  block.eta_B.setZero();
  return block;
}

RuntimeBlockBridgeRun run_runtime_block_bridge_case(
    const std::string& name,
    const Eigen::Vector3d& position_fim_diag) {
  RuntimeBlockBridgeRun run;
  run.name = name;
  const double min_fim = position_fim_diag.minCoeff();
  const double max_fim = position_fim_diag.maxCoeff();
  run.condition_number =
      (min_fim > 0.0 && std::isfinite(min_fim) && std::isfinite(max_fim))
          ? max_fim / min_fim
          : 1.0e12;

  const Eigen::Matrix3d fim = position_fim_diag.asDiagonal();
  FGOPositionInfo fgo = make_fgo_from_position_fim(fim, 440);

  run.snapshot.valid = true;
  run.snapshot.frame_id = fgo.frame_id;
  run.snapshot.stamp = 60.0;
  run.snapshot.pose_cov_6x6 = fgo.pose_cov_6x6;
  run.snapshot.current_icp_quality.gamma_lidar = 1.0;

  // Keep every subset positive definite: the three blocks together contribute
  // 80% of the current-frame position information.
  const Eigen::Vector3d per_block = (0.8 / 3.0) * position_fim_diag;
  for (int i = 0; i < 3; ++i) {
    run.snapshot.blocks.push_back(make_runtime_bridge_block(
        i + 1, i, per_block, run.condition_number));
  }

  LidarAraim::Params params;
  params.dynamic_budget = false;
  params.K_ff = 5.0;
  params.K_fa = 4.0;
  params.K_md = 3.0;
  params.alpha_H = 0.0;
  params.alpha_V = 0.0;
  LidarAraim evaluator(params);
  run.result = evaluator.run(run.snapshot, fgo);
  return run;
}

std::string lidar_hypothesis_mode(const LidarHypothesis& hyp) {
  switch (hyp.type) {
    case LidarHypothesis::Type::H_SOURCE:
      return "H_SOURCE";
    case LidarHypothesis::Type::H_TARGET:
      return "H_TARGET(" + std::to_string(hyp.target_frame_id) + ")";
    case LidarHypothesis::Type::H_LEVEL:
      return "H_LEVEL(" + std::to_string(hyp.level_id) + ")";
  }
  return "UNKNOWN";
}

SingleBlockHypothesisObservation find_worst_single_block_hypothesis(
    const LidarAraimResult& result,
    const LidarAraimSnapshot& snapshot) {
  SingleBlockHypothesisObservation worst;
  const std::size_t n =
      std::min(result.hypotheses.size(), result.subsets.size());
  for (std::size_t i = 0; i < n; ++i) {
    const auto& hyp = result.hypotheses[i];
    if (hyp.block_indices.size() != 1) {
      continue;
    }
    const int block_index = hyp.block_indices.front();
    if (block_index < 0 ||
        block_index >= static_cast<int>(snapshot.blocks.size())) {
      continue;
    }
    const auto& ss = result.subsets[i];
    if (ss.PL_E > worst.PL_E) {
      worst.block_index = block_index;
      worst.block_id =
          static_cast<int>(snapshot.blocks[block_index].target_frame_id);
      worst.hyp_index = static_cast<int>(i);
      worst.fault_detected = ss.fault_detected;
      worst.d_E = ss.d_E;
      worst.T_E = ss.T_E;
      worst.PL_E = ss.PL_E;
      worst.HPL = ss.HPL;
      worst.mode = lidar_hypothesis_mode(hyp);
    }
  }
  return worst;
}

SingleBlockHypothesisObservation find_single_block_hypothesis_for_block(
    const LidarAraimResult& result,
    const LidarAraimSnapshot& snapshot,
    const int block_id) {
  SingleBlockHypothesisObservation best;
  const std::size_t n =
      std::min(result.hypotheses.size(), result.subsets.size());
  for (std::size_t i = 0; i < n; ++i) {
    const auto& hyp = result.hypotheses[i];
    if (hyp.block_indices.size() != 1) {
      continue;
    }
    const int block_index = hyp.block_indices.front();
    if (block_index < 0 ||
        block_index >= static_cast<int>(snapshot.blocks.size()) ||
        snapshot.blocks[block_index].target_frame_id != block_id) {
      continue;
    }
    const auto& ss = result.subsets[i];
    if (ss.PL_E > best.PL_E) {
      best.block_index = block_index;
      best.block_id = block_id;
      best.hyp_index = static_cast<int>(i);
      best.fault_detected = ss.fault_detected;
      best.d_E = ss.d_E;
      best.T_E = ss.T_E;
      best.PL_E = ss.PL_E;
      best.HPL = ss.HPL;
      best.mode = lidar_hypothesis_mode(hyp);
    }
  }
  return best;
}

std::vector<Eigen::Vector3d> read_lidar_cloud_csv(const std::string& path) {
  std::vector<Eigen::Vector3d> points;
  std::ifstream file(path);
  if (!file.is_open()) {
    return points;
  }

  std::string line;
  std::getline(file, line);  // header
  while (std::getline(file, line)) {
    if (line.empty()) {
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream iss(line);
    double x = 0.0, y = 0.0, z = 0.0;
    if (iss >> x >> y >> z) {
      points.emplace_back(x, y, z);
    }
  }
  return points;
}

CurrentIntegrityState make_lidar_cloud_current_state() {
  CurrentIntegrityState current;
  current.valid = true;
  current.hpl = 5.0;
  current.vpl = 5.0;
  current.hal = 30.0;
  current.val = 20.0;
  current.im = 15.0;
  current.tdop = 2.0;
  current.n_trunks_observed = 4;
  return current;
}

PointCloudToLidarPlEvaluation evaluate_lidar_cloud_csv(
    const std::string& name,
    const std::string& filename) {
  PointCloudToLidarPlEvaluation evaluation;
  evaluation.name = name;
  const std::string path = araim_validation_results_path(filename);
  evaluation.points = read_lidar_cloud_csv(path);
  if (evaluation.points.empty()) {
    evaluation.failure_reason = "missing_or_empty_cloud_csv";
    return evaluation;
  }

  LidarFimPrimitiveGenerationParams primitive_params;
  primitive_params.pca_radius_m = 1.2;
  primitive_params.pca_min_support = 8;
  primitive_params.pca_voxel_sample_m = 0.35;
  primitive_params.pca_max_points = 3500;
  primitive_params.pca_max_primitives = 2500;
  evaluation.primitives = make_lidar_fim_primitives(
      evaluation.points, nullptr, primitive_params, &evaluation.primitive_diag);
  if (!evaluation.primitives || evaluation.primitives->empty()) {
    evaluation.failure_reason = evaluation.primitive_diag.fallback_reason;
    return evaluation;
  }

  LidarObservabilityFim::Params fim_params;
  fim_params.fim_radius_m = 12.0;
  fim_params.fim_min_voxels = 6;
  fim_params.fim_range_sigma_base = 1.0;
  fim_params.fim_condition_max = 1.0e12;
  fim_params.fim_weight_scale = 1.0;
  LidarObservabilityFim fim_estimator(fim_params);
  evaluation.fim = fim_estimator.evaluate_advisory_fim(
      Eigen::Vector3d::Zero(), evaluation.primitives.get(),
      make_lidar_cloud_current_state());
  if (!evaluation.fim.valid) {
    evaluation.failure_reason = evaluation.fim.fallback_reason;
    return evaluation;
  }

  Eigen::Matrix3d fim_for_pl = evaluation.fim.lambda;
  if (evaluation.fim.min_eig <= 0.0) {
    evaluation.failure_reason = "non_positive_lidar_fim";
    return evaluation;
  }
  evaluation.pl = evaluate_synthetic_lidar_fim(fim_for_pl);
  evaluation.failure_reason = evaluation.pl.failure_reason;
  return evaluation;
}

void record_lidar_cloud_eval(const std::string& prefix,
                             const PointCloudToLidarPlEvaluation& eval) {
  ::testing::Test::RecordProperty((prefix + "_point_count").c_str(),
                 static_cast<int>(eval.points.size()));
  ::testing::Test::RecordProperty((prefix + "_primitive_count").c_str(),
                 eval.primitives ? static_cast<int>(eval.primitives->size()) : 0);
  ::testing::Test::RecordProperty((prefix + "_valid_normals").c_str(),
                 eval.fim.n_valid_normals);
  ::testing::Test::RecordProperty((prefix + "_fim_valid").c_str(), eval.fim.valid ? 1 : 0);
  ::testing::Test::RecordProperty((prefix + "_fim_trace").c_str(), eval.fim.trace);
  ::testing::Test::RecordProperty((prefix + "_fim_min_eig").c_str(), eval.fim.min_eig);
  ::testing::Test::RecordProperty((prefix + "_fim_max_eig").c_str(), eval.fim.max_eig);
  ::testing::Test::RecordProperty((prefix + "_fim_condition").c_str(), eval.fim.condition);
  ::testing::Test::RecordProperty((prefix + "_lidar_valid").c_str(),
                 eval.pl.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty((prefix + "_lidar_hpl").c_str(), eval.pl.result.HPL);
  ::testing::Test::RecordProperty((prefix + "_lidar_vpl").c_str(), eval.pl.result.VPL);
  ::testing::Test::RecordProperty((prefix + "_lidar_pl_e").c_str(), eval.pl.result.PL_E);
  ::testing::Test::RecordProperty((prefix + "_lidar_pl_n").c_str(), eval.pl.result.PL_N);
  ::testing::Test::RecordProperty((prefix + "_lidar_pl_u").c_str(), eval.pl.result.PL_U);
  ::testing::Test::RecordProperty((prefix + "_lidar_worst_mode").c_str(),
                 eval.pl.result.worst_mode.c_str());
  ::testing::Test::RecordProperty((prefix + "_fallback_reason").c_str(),
                 eval.failure_reason.empty() ? "none" : eval.failure_reason.c_str());
}

void write_lidar_pointcloud_metrics_csv(
    const std::vector<PointCloudToLidarPlEvaluation>& evaluations) {
  std::ofstream out(araim_validation_results_path(
      "lidar_pointcloud_fim_pl_metrics.csv"));
  out << "scenario,point_count,primitive_count,valid_normals,fim_valid,"
      << "fim_00,fim_01,fim_02,fim_10,fim_11,fim_12,fim_20,fim_21,fim_22,"
      << "lambda_min,lambda_mid,lambda_max,fim_trace,fim_condition,"
      << "lidar_valid,hpl,vpl,pl_e,pl_n,pl_u,worst_mode,failure_reason\n";

  for (const auto& eval : evaluations) {
    Eigen::Vector3d eigenvalues =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(eval.fim.lambda);
    if (eig.info() == Eigen::Success && eig.eigenvalues().allFinite()) {
      eigenvalues = eig.eigenvalues();
    }

    out << csv_escape(eval.name) << ','
        << eval.points.size() << ','
        << (eval.primitives ? eval.primitives->size() : 0) << ','
        << eval.fim.n_valid_normals << ','
        << (eval.fim.valid ? 1 : 0) << ','
        << eval.fim.lambda(0, 0) << ',' << eval.fim.lambda(0, 1) << ','
        << eval.fim.lambda(0, 2) << ',' << eval.fim.lambda(1, 0) << ','
        << eval.fim.lambda(1, 1) << ',' << eval.fim.lambda(1, 2) << ','
        << eval.fim.lambda(2, 0) << ',' << eval.fim.lambda(2, 1) << ','
        << eval.fim.lambda(2, 2) << ','
        << eigenvalues(0) << ',' << eigenvalues(1) << ','
        << eigenvalues(2) << ','
        << eval.fim.trace << ',' << eval.fim.condition << ','
        << (eval.pl.result.valid ? 1 : 0) << ','
        << eval.pl.result.HPL << ',' << eval.pl.result.VPL << ','
        << eval.pl.result.PL_E << ',' << eval.pl.result.PL_N << ','
        << eval.pl.result.PL_U << ','
        << csv_escape(eval.pl.result.worst_mode) << ','
        << csv_escape(eval.failure_reason.empty() ? "none"
                                                  : eval.failure_reason)
        << '\n';
  }
}

void write_lidar_block_fault_metrics_csv(
    const LidarBlockFaultRun& clean,
    const LidarBlockFaultRun& injected,
    const SingleBlockHypothesisObservation& worst,
    const SingleBlockHypothesisObservation& bad_block) {
  std::ofstream out(araim_validation_results_path(
      "lidar_block_fault_metrics.csv"));
  out << "case,clean_hpl,clean_vpl,bad_hpl,bad_vpl,bad_pl_e,bad_pl_n,"
      << "bad_pl_u,bad_block_id,abs_d_e,t_e,fault_detected,worst_block_id,"
      << "worst_mode,n_hypotheses,n_detected\n";
  out << "block_fault_injection,"
      << clean.result.HPL << ','
      << clean.result.VPL << ','
      << injected.result.HPL << ','
      << injected.result.VPL << ','
      << injected.result.PL_E << ','
      << injected.result.PL_N << ','
      << injected.result.PL_U << ','
      << bad_block.block_id << ','
      << std::abs(bad_block.d_E) << ','
      << bad_block.T_E << ','
      << (bad_block.fault_detected ? 1 : 0) << ','
      << worst.block_id << ','
      << csv_escape(injected.result.worst_mode) << ','
      << injected.result.n_hypotheses << ','
      << injected.result.n_detected << '\n';
}

}  // namespace

// ---------------------------------------------------------------------------
// T1: Q_inv accuracy tests
// ---------------------------------------------------------------------------
TEST_F(GnssAraimEvaluatorTest, QInvReasonableValues) {
  // Q_inv is private static; test indirectly via ARAIM run which uses
  // dynamic budget allocation.  Instead, we test it through well-known values:
  //   Q(3.09) ≈ 1e-3, Q(4.26) ≈ 1e-5, Q(5.33) ≈ 5e-8
  // By running ARAIM with dynamic_budget=true, the K_ff computed should be
  // close to Q_inv(P_HMI_req/2) ≈ Q_inv(5e-8) ≈ 5.33
  GnssAraimParams p = default_params();
  p.P_HMI_req     = 1e-7;
  p.dynamic_budget = true;
  GnssAraimEvaluator evaluator(p);

  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult result = evaluator.run(epoch, 0);

  ASSERT_TRUE(result.valid);
  // K_ff = Q_inv(P_HMI_req/2) ≈ Q_inv(5e-8)
  // Should be around 5.3–5.5
  EXPECT_GT(result.K_ff_used, 5.0);
  EXPECT_LT(result.K_ff_used, 6.0);
}

// ---------------------------------------------------------------------------
// T2: GnssAraimResult is valid with sufficient geometry
// ---------------------------------------------------------------------------
TEST_F(GnssAraimEvaluatorTest, ValidResultWithGoodGeometry) {
  GnssAraimEvaluator evaluator(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = evaluator.run(epoch, 0);

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
TEST_F(GnssAraimEvaluatorTest, HplIsMaxOfPerAxis) {
  GnssAraimEvaluator evaluator(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = evaluator.run(epoch, 0);

  ASSERT_TRUE(r.valid);
  EXPECT_DOUBLE_EQ(r.HPL, std::max(r.PL_E, r.PL_N));
  EXPECT_DOUBLE_EQ(r.VPL, r.PL_U);
}

// ---------------------------------------------------------------------------
// T4: 3-term PL formula: PL_{q,k} = |d_{q,k}| + K_fa·σ_ss,q,k + K_md·σ_{q,k}
// ---------------------------------------------------------------------------
TEST_F(GnssAraimEvaluatorTest, ThreeTermPerAxisPL) {
  GnssAraimEvaluator evaluator(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = evaluator.run(epoch, 0);

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
TEST_F(GnssAraimEvaluatorTest, TotalPLIsMaxOverHypotheses) {
  GnssAraimEvaluator evaluator(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = evaluator.run(epoch, 0);

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
TEST_F(GnssAraimEvaluatorTest, DegenerateGeometryInvalid) {
  GnssAraimEvaluator evaluator(default_params());
  GnssEpoch epoch = make_epoch(3);  // less than min_sats=4
  GnssAraimResult r = evaluator.run(epoch, 0);

  EXPECT_FALSE(r.valid);
  EXPECT_GE(r.HPL, 1e9);
}

// ---------------------------------------------------------------------------
// T7: Fault-free PL components (pl_ff_E, pl_ff_N, pl_ff_V)
// ---------------------------------------------------------------------------
TEST_F(GnssAraimEvaluatorTest, FaultFreePLComponents) {
  GnssAraimEvaluator evaluator(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = evaluator.run(epoch, 0);

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
TEST_F(GnssAraimEvaluatorTest, PredictGeometryNoResiduals) {
  GnssAraimEvaluator evaluator(default_params());

  std::vector<GnssAraimEvaluator::GnssAraimSatGeometry> sats;
  for (int i = 0; i < 8; ++i) {
    GnssAraimEvaluator::GnssAraimSatGeometry sg;
    sg.elevation = 0.5 + 0.15 * i;
    sg.azimuth   = M_PI / 4.0 * i;
    sg.pr_sigma  = 4.0;
    sg.sat_id    = 200 + i;
    sats.push_back(sg);
  }

  GnssAraimResult r = evaluator.predict_geometry(sats);
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
// T9: Trunk hypotheses disabled by default (Step 5)
// ---------------------------------------------------------------------------
TEST_F(GnssAraimEvaluatorTest, TrunkHypothesesDisabledByDefault) {
  auto p = default_params();
  // default: enable_trunk_hypotheses = false
  GnssAraimEvaluator evaluator(p);
  GnssEpoch epoch = make_epoch(6);

  GnssAraimResult r0 = evaluator.run(epoch, 0);
  GnssAraimResult r3 = evaluator.run(epoch, 3);

  ASSERT_TRUE(r0.valid);
  ASSERT_TRUE(r3.valid);
  EXPECT_EQ(r3.n_hypotheses, r0.n_hypotheses);
  EXPECT_EQ(r3.n_trunk_placeholders, 0);
  EXPECT_FALSE(r3.trunk_hypotheses_enabled);
  EXPECT_TRUE(r3.excluded_trunk_ids.empty());
}

// T10: Trunk hypotheses explicit placeholder (only when enabled)
TEST_F(GnssAraimEvaluatorTest, TrunkHypothesesExplicitPlaceholderWhenEnabled) {
  auto p = default_params();
  p.enable_trunk_hypotheses = true;
  GnssAraimEvaluator evaluator(p);
  GnssEpoch epoch = make_epoch(6);

  GnssAraimResult r0 = evaluator.run(epoch, 0);
  GnssAraimResult r3 = evaluator.run(epoch, 3);

  ASSERT_TRUE(r0.valid);
  ASSERT_TRUE(r3.valid);
  EXPECT_EQ(r3.n_hypotheses, r0.n_hypotheses + 3);
  EXPECT_TRUE(r3.trunk_hypotheses_enabled);
  EXPECT_EQ(r3.n_trunk_placeholders, 3);
  EXPECT_EQ(r3.n_detected, r0.n_detected);
  EXPECT_TRUE(r3.excluded_trunk_ids.empty());
}

// T11: Constellation faults can be disabled
TEST_F(GnssAraimEvaluatorTest, ConstellationFaultsCanBeDisabled) {
  auto p = default_params();
  p.enable_constellation_faults = false;
  GnssAraimEvaluator evaluator(p);
  GnssEpoch epoch = make_epoch(6);
  GnssAraimResult r = evaluator.run(epoch, 0);

  ASSERT_TRUE(r.valid);
  EXPECT_FALSE(r.constellation_faults_enabled);
  for (const auto& h : r.hypotheses) {
    EXPECT_NE(h.type, FaultHypothesis::Type::CONSTELLATION);
  }
  int active = 0;
  for (const auto& s : epoch.sats) if (!s.excluded) ++active;
  EXPECT_EQ(r.n_hypotheses, active);
}

// T12: Constellation faults enabled by default
TEST_F(GnssAraimEvaluatorTest, ConstellationFaultsEnabledByDefault) {
  auto p = default_params();
  GnssAraimEvaluator evaluator(p);
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = evaluator.run(epoch, 0);

  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.constellation_faults_enabled);
  bool found = false;
  for (const auto& h : r.hypotheses) {
    if (h.type == FaultHypothesis::Type::CONSTELLATION) { found = true; break; }
  }
  EXPECT_TRUE(found);
}

// T13: Degenerate constellation is explicit
TEST_F(GnssAraimEvaluatorTest, DegenerateConstellationIsExplicit) {
  auto p = default_params();
  p.degrade_on_degenerate_hypothesis = true;
  GnssAraimEvaluator evaluator(p);
  GnssEpoch epoch = make_epoch(6);
  GnssAraimResult r = evaluator.run(epoch, 0);

  ASSERT_TRUE(r.valid) << "Result should use conservative degraded PL";
  EXPECT_TRUE(r.has_degenerate_hypothesis);
  EXPECT_GT(r.n_degenerate_hypotheses, 0);
  bool found = false;
  for (const auto& ss : r.subsets) {
    if (ss.degenerate) {
      found = true;
      EXPECT_FALSE(ss.valid);
      EXPECT_FALSE(ss.failure_reason.empty());
      if (p.degrade_on_degenerate_hypothesis) {
        EXPECT_GE(ss.PL_E, 1e9 * 0.99);
      }
      break;
    }
  }
  EXPECT_TRUE(found);
  EXPECT_GE(r.HPL, 1e8);
  EXPECT_LT(r.HPL, 1e10);
}

// T14: No false trunk detection
TEST_F(GnssAraimEvaluatorTest, NoFalseTrunkDetection) {
  auto p = default_params();
  p.enable_trunk_hypotheses = true;
  GnssAraimEvaluator evaluator(p);
  GnssEpoch epoch = make_epoch(6);
  GnssAraimResult r = evaluator.run(epoch, 3);

  ASSERT_TRUE(r.valid);
  EXPECT_TRUE(r.excluded_trunk_ids.empty());
  EXPECT_EQ(r.n_detected, 0);
}

// ---------------------------------------------------------------------------
// T15: S0 is 4×4 and positive (semi)-definite
// ---------------------------------------------------------------------------
TEST_F(GnssAraimEvaluatorTest, S0IsPositiveSemidefinite) {
  GnssAraimEvaluator evaluator(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = evaluator.run(epoch, 0);

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
TEST_F(GnssAraimEvaluatorTest, ParallelMatchesSerial) {
  GnssAraimParams serial_params = default_params();
  serial_params.parallel_hypotheses = false;

  GnssAraimParams parallel_params = default_params();
  parallel_params.parallel_hypotheses = true;
  parallel_params.hypothesis_threads = 2;

  Araim serial_araim(serial_params);
  Araim parallel_araim(parallel_params);
  GnssEpoch epoch = make_epoch(10);

  GnssAraimResult serial = serial_araim.run(epoch, 2);
  GnssAraimResult parallel = parallel_araim.run(epoch, 2);

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
    EXPECT_EQ(a.n_removed_by_hyp, b.n_removed_by_hyp);
    EXPECT_EQ(a.n_remaining_after_hyp, b.n_remaining_after_hyp);
    EXPECT_EQ(a.removed_prn_list, b.removed_prn_list);
    EXPECT_EQ(a.remaining_prn_list, b.remaining_prn_list);
    EXPECT_NEAR(a.HDOP_subset, b.HDOP_subset, 1e-12);
    EXPECT_NEAR(a.VDOP_subset, b.VDOP_subset, 1e-12);
    EXPECT_EQ(a.fault_detected, b.fault_detected);
  }
}

TEST_F(GnssAraimEvaluatorTest, ConstellationHypothesisDiagnosticsPopulated) {
  GnssAraimParams p = default_params();
  p.parallel_hypotheses = false;
  GnssAraimEvaluator eval(p);

  GnssEpoch epoch = make_epoch(8);
  const std::vector<int> prns = {1, 2, 57, 58, 101, 102, 33, 34};
  for (std::size_t i = 0; i < epoch.sats.size(); ++i) {
    epoch.sats[i].sat_id = prns[i];
  }

  const GnssAraimResult result = eval.run(epoch, 0);
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.n_used_total, 8);
  EXPECT_EQ(result.used_prns, prns);
  EXPECT_NEAR(result.HDOP_full,
              std::sqrt(result.sigma_ff_E * result.sigma_ff_E +
                        result.sigma_ff_N * result.sigma_ff_N),
              1e-12);
  EXPECT_NEAR(result.VDOP_full, result.sigma_ff_U, 1e-12);

  int bds_hyp_index = -1;
  for (int i = 0; i < static_cast<int>(result.hypotheses.size()); ++i) {
    const auto& hyp = result.hypotheses[static_cast<std::size_t>(i)];
    if (hyp.type == FaultHypothesis::Type::CONSTELLATION && hyp.const_id == 2) {
      bds_hyp_index = i;
      break;
    }
  }
  ASSERT_GE(bds_hyp_index, 0);
  ASSERT_LT(bds_hyp_index, static_cast<int>(result.subsets.size()));

  const auto& subset = result.subsets[static_cast<std::size_t>(bds_hyp_index)];
  EXPECT_EQ(subset.n_removed_by_hyp, 2);
  EXPECT_EQ(subset.n_remaining_after_hyp, 6);
  EXPECT_EQ(subset.removed_prn_list, std::vector<int>({101, 102}));
  EXPECT_EQ(subset.remaining_prn_list, std::vector<int>({1, 2, 57, 58, 33, 34}));
  EXPECT_NEAR(subset.HDOP_full, result.HDOP_full, 1e-12);
  EXPECT_NEAR(subset.VDOP_full, result.VDOP_full, 1e-12);
  EXPECT_NEAR(subset.PDOP_full, result.PDOP_full, 1e-12);
  EXPECT_NEAR(subset.HDOP_subset,
              std::sqrt(subset.sigma_k_E * subset.sigma_k_E +
                        subset.sigma_k_N * subset.sigma_k_N),
              1e-12);
  EXPECT_NEAR(subset.VDOP_subset, subset.sigma_k_U, 1e-12);
  EXPECT_NEAR(subset.PDOP_subset,
              std::sqrt(subset.sigma_k_E * subset.sigma_k_E +
                        subset.sigma_k_N * subset.sigma_k_N +
                        subset.sigma_k_U * subset.sigma_k_U),
              1e-12);
}

// ============================================================================
// §1.9: Step 8 linearized input seam
// ============================================================================

TEST_F(GnssAraimEvaluatorTest, LinearizedMatchesEpochRoundTrip) {
  // A: epoch → linearized → runLinearized  ≡  run(epoch)
  GnssAraimEvaluator eval(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult direct = eval.run(epoch, 0);
  ASSERT_TRUE(direct.valid);

  auto input = GnssAraimEvaluator::buildLinearizedInputFromGnssEpoch(epoch);
  GnssAraimResult linear = eval.runLinearized(input, 0);
  ASSERT_TRUE(linear.valid);

  EXPECT_NEAR(direct.HPL, linear.HPL, 1e-12);
  EXPECT_NEAR(direct.VPL, linear.VPL, 1e-12);
  EXPECT_NEAR(direct.PL_E, linear.PL_E, 1e-12);
  EXPECT_NEAR(direct.PL_N, linear.PL_N, 1e-12);
  EXPECT_NEAR(direct.PL_U, linear.PL_U, 1e-12);
  EXPECT_EQ(direct.n_hypotheses, linear.n_hypotheses);
  EXPECT_EQ(direct.n_detected, linear.n_detected);
}

TEST_F(GnssAraimEvaluatorTest, LinearizedMatchesEpochWithTrunk) {
  // B: epoch → linearized → runLinearized(n_trunk)  ≡  run(epoch, n_trunk)
  GnssAraimParams p = default_params();
  p.enable_trunk_hypotheses = true;
  GnssAraimEvaluator eval(p);
  GnssEpoch epoch = make_epoch(10);
  GnssAraimResult direct = eval.run(epoch, 2);
  ASSERT_TRUE(direct.valid);

  auto input = GnssAraimEvaluator::buildLinearizedInputFromGnssEpoch(epoch);
  GnssAraimResult linear = eval.runLinearized(input, 2);
  ASSERT_TRUE(linear.valid);

  EXPECT_NEAR(direct.HPL, linear.HPL, 1e-12);
  EXPECT_NEAR(direct.VPL, linear.VPL, 1e-12);
  EXPECT_EQ(direct.n_hypotheses, linear.n_hypotheses);
  EXPECT_EQ(direct.hypotheses.size(), linear.hypotheses.size());
}

TEST_F(GnssAraimEvaluatorTest, LinearizedRejectsTooFewRows) {
  // C: N < min_sats → invalid
  GnssAraimEvaluator eval(default_params());
  GnssAraimLinearizedInput input;
  input.G = Eigen::MatrixXd::Zero(3, 4);   // 3 sat, min_sats=4
  input.W = Eigen::VectorXd::Ones(3);
  input.r = Eigen::VectorXd::Zero(3);
  input.prns = {1, 2, 3};
  input.constellation_ids = {0, 0, 0};
  GnssAraimResult r = eval.runLinearized(input, 0);
  EXPECT_FALSE(r.valid);
}

TEST_F(GnssAraimEvaluatorTest, LinearizedRejectsBadDimensions) {
  // D: G.cols != 4 or size mismatches → invalid
  GnssAraimEvaluator eval(default_params());
  GnssAraimLinearizedInput input;
  input.G = Eigen::MatrixXd::Zero(5, 6);  // cols != 4
  input.W = Eigen::VectorXd::Ones(5);
  input.r = Eigen::VectorXd::Zero(5);
  input.prns = {1, 2, 3, 4, 5};
  input.constellation_ids = {0, 0, 0, 0, 0};
  GnssAraimResult r = eval.runLinearized(input, 0);
  EXPECT_FALSE(r.valid);
}

TEST_F(GnssAraimEvaluatorTest, LinearizedRejectsNaN) {
  // E: NaN in G/W/r → invalid
  GnssAraimEvaluator eval(default_params());
  GnssAraimLinearizedInput input;
  input.G = Eigen::MatrixXd::Zero(5, 4);
  input.G(2, 1) = std::numeric_limits<double>::quiet_NaN();
  input.W = Eigen::VectorXd::Ones(5);
  input.r = Eigen::VectorXd::Zero(5);
  input.prns = {1, 2, 3, 4, 5};
  input.constellation_ids = {0, 0, 0, 0, 0};
  GnssAraimResult r = eval.runLinearized(input, 0);
  EXPECT_FALSE(r.valid);
}

TEST_F(GnssAraimEvaluatorTest, LinearizedRejectsNonPositiveWeight) {
  // F: W(i) <= 0 → invalid
  GnssAraimEvaluator eval(default_params());
  GnssAraimLinearizedInput input;
  input.G = Eigen::MatrixXd::Zero(5, 4);
  input.W = Eigen::VectorXd::Ones(5);
  input.W(3) = 0.0;  // non-positive
  input.r = Eigen::VectorXd::Zero(5);
  input.prns = {1, 2, 3, 4, 5};
  input.constellation_ids = {0, 0, 0, 0, 0};
  GnssAraimResult r = eval.runLinearized(input, 0);
  EXPECT_FALSE(r.valid);
}

TEST_F(GnssAraimEvaluatorTest, LinearizedConstellationDisabledMatchesEpoch) {
  // G: constellation faults disabled → same result via both paths
  GnssAraimParams p = default_params();
  p.enable_constellation_faults = false;
  GnssAraimEvaluator eval(p);
  GnssEpoch epoch = make_epoch(6);
  GnssAraimResult direct = eval.run(epoch, 0);
  ASSERT_TRUE(direct.valid);

  auto input = GnssAraimEvaluator::buildLinearizedInputFromGnssEpoch(epoch);
  GnssAraimResult linear = eval.runLinearized(input, 0);
  ASSERT_TRUE(linear.valid);

  EXPECT_NEAR(direct.HPL, linear.HPL, 1e-12);
  EXPECT_NEAR(direct.VPL, linear.VPL, 1e-12);
  EXPECT_EQ(direct.n_hypotheses, linear.n_hypotheses);
}

TEST(AraimGoldenTest, FaultFreeSyntheticSixSatelliteGeometry) {
  GnssAraimLinearizedInput input;
  input.G.resize(6, 4);
  input.G << -0.0, -0.5, -0.866025403784, 1.0,
             -0.556670399226, -0.321393804843, -0.766044443119, 1.0,
             -0.496731764892, 0.286788218176, -0.819152044289, 1.0,
             0.241844762648, 0.664463024389, -0.707106781187, 1.0,
             0.633022221559, -0.111618897049, -0.766044443119, 1.0,
             0.211309130870, -0.365998150771, -0.906307787037, 1.0;

  constexpr double kSigma = 1.5;
  constexpr double kWeight = 1.0 / (kSigma * kSigma);
  input.W = Eigen::VectorXd::Constant(6, kWeight);
  input.r = Eigen::VectorXd::Zero(6);
  input.prns = {1, 2, 3, 4, 5, 6};
  input.constellation_ids = {0, 0, 0, 0, 0, 0};
  input.sigmas_m = {kSigma, kSigma, kSigma, kSigma, kSigma, kSigma};
  input.elevations_rad = {
      60.0 * M_PI / 180.0,
      50.0 * M_PI / 180.0,
      55.0 * M_PI / 180.0,
      45.0 * M_PI / 180.0,
      50.0 * M_PI / 180.0,
      65.0 * M_PI / 180.0};
  input.stamp = 100.0;

  GnssAraimParams params;
  params.P_HMI_req = 1e-7;
  params.P_FA_req = 1e-5;
  params.p_sat_default = 1e-5;
  params.p_const_GPS = 1e-4;
  params.dynamic_budget = false;
  params.K_ff = 5.451310438136472;
  params.K_fa = 0.0;
  params.K_md = 0.0;
  params.enable_constellation_faults = false;
  params.enable_trunk_hypotheses = false;
  params.parallel_hypotheses = false;

  GnssAraimEvaluator solver(params);
  const GnssAraimResult result = solver.runLinearized(input, 0);

  constexpr double kTol = 1e-6;
  constexpr double kExpectedKff = 5.451310438136472;
  constexpr double kExpectedSigmaE = 1.460597699646529;
  constexpr double kExpectedSigmaN = 2.044854590757374;
  constexpr double kExpectedSigmaU = 12.528175678508841;
  constexpr double kExpectedPlE = 7.962171486001244;
  constexpr double kExpectedPlN = 11.147137175066957;
  constexpr double kExpectedPlU = 68.29497484706273;
  constexpr double kExpectedHpl = 11.147137175066957;
  constexpr double kExpectedVpl = 68.29497484706273;

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.n_hypotheses, 6);
  EXPECT_EQ(result.n_detected, 0);
  EXPECT_FALSE(result.has_degenerate_hypothesis);

  EXPECT_NEAR(result.K_ff_used, kExpectedKff, kTol);
  EXPECT_NEAR(result.sigma_ff_E, kExpectedSigmaE, kTol);
  EXPECT_NEAR(result.sigma_ff_N, kExpectedSigmaN, kTol);
  EXPECT_NEAR(result.sigma_ff_U, kExpectedSigmaU, kTol);
  EXPECT_NEAR(result.pl_ff_E, kExpectedPlE, kTol);
  EXPECT_NEAR(result.pl_ff_N, kExpectedPlN, kTol);
  EXPECT_NEAR(result.pl_ff_V, kExpectedPlU, kTol);
  EXPECT_NEAR(result.PL_E, kExpectedPlE, kTol);
  EXPECT_NEAR(result.PL_N, kExpectedPlN, kTol);
  EXPECT_NEAR(result.PL_U, kExpectedPlU, kTol);
  EXPECT_NEAR(result.HPL, kExpectedHpl, kTol);
  EXPECT_NEAR(result.VPL, kExpectedVpl, kTol);
}

namespace {

void ExpectFiniteConservativeInvalid(const GnssAraimResult& result) {
  EXPECT_FALSE(result.valid);
  EXPECT_TRUE(std::isfinite(result.HPL));
  EXPECT_TRUE(std::isfinite(result.VPL));
  EXPECT_TRUE(std::isfinite(result.pl_araim));
  EXPECT_TRUE(std::isfinite(result.vpl_araim));
  EXPECT_FALSE(std::isnan(result.HPL));
  EXPECT_FALSE(std::isnan(result.VPL));
  EXPECT_FALSE(std::isinf(result.HPL));
  EXPECT_FALSE(std::isinf(result.VPL));
  EXPECT_GE(result.HPL, 1e9 * 0.99);
  EXPECT_GE(result.VPL, 1e9 * 0.99);
}

GnssAraimLinearizedInput make_linearized_from_az_el(
    const std::vector<std::pair<double, double>>& az_el_deg,
    double sigma_m = 1.5) {
  GnssAraimLinearizedInput input;
  const int n = static_cast<int>(az_el_deg.size());
  input.G.resize(n, 4);
  input.W = Eigen::VectorXd::Constant(n, 1.0 / (sigma_m * sigma_m));
  input.r = Eigen::VectorXd::Zero(n);
  input.prns.resize(static_cast<std::size_t>(n));
  input.constellation_ids.assign(static_cast<std::size_t>(n), 0);
  input.elevations_rad.resize(static_cast<std::size_t>(n));
  input.sigmas_m.assign(static_cast<std::size_t>(n), sigma_m);
  input.stamp = 100.0;

  for (int i = 0; i < n; ++i) {
    const double az = az_el_deg[static_cast<std::size_t>(i)].first * M_PI / 180.0;
    const double el = az_el_deg[static_cast<std::size_t>(i)].second * M_PI / 180.0;
    const double e_east = std::cos(el) * std::sin(az);
    const double e_north = std::cos(el) * std::cos(az);
    const double e_up = std::sin(el);
    input.G(i, 0) = -e_east;
    input.G(i, 1) = -e_north;
    input.G(i, 2) = -e_up;
    input.G(i, 3) = 1.0;
    input.prns[static_cast<std::size_t>(i)] = i + 1;
    input.elevations_rad[static_cast<std::size_t>(i)] = el;
  }

  return input;
}

const SubsetSolution* find_satellite_subset(const GnssAraimResult& result,
                                            int prn) {
  for (std::size_t i = 0; i < result.hypotheses.size(); ++i) {
    const auto& hyp = result.hypotheses[i];
    if (hyp.type == FaultHypothesis::Type::GNSS_SAT && hyp.sat_id == prn) {
      for (const auto& subset : result.subsets) {
        if (subset.hyp_index == static_cast<int>(i)) {
          return &subset;
        }
      }
    }
  }
  return nullptr;
}

bool contains_int(const std::vector<int>& values, int needle) {
  return std::find(values.begin(), values.end(), needle) != values.end();
}

}  // namespace

TEST(AraimDegenerateTest, TooFewSatellitesDoesNotProduceNan) {
  const GnssAraimLinearizedInput input = make_linearized_from_az_el({
      {0.0, 60.0},
      {60.0, 50.0},
      {120.0, 55.0},
  });

  GnssAraimEvaluator solver(GnssAraimParams{});
  const GnssAraimResult result = solver.runLinearized(input, 0);

  ExpectFiniteConservativeInvalid(result);
  EXPECT_EQ(result.n_hypotheses, 0);
}

TEST(AraimDegenerateTest, NearlySingularGeometryDoesNotProduceNan) {
  const GnssAraimLinearizedInput input = make_linearized_from_az_el({
      {0.0, 30.0},
      {5.0, 31.0},
      {10.0, 32.0},
      {15.0, 33.0},
      {20.0, 34.0},
  });

  GnssAraimParams params;
  params.eps_degen = 1e-3;
  params.enable_constellation_faults = false;
  params.parallel_hypotheses = false;
  GnssAraimEvaluator solver(params);
  const GnssAraimResult result = solver.runLinearized(input, 0);

  ExpectFiniteConservativeInvalid(result);
}

TEST(AraimDegenerateTest, InvalidWeightsAreRejected) {
  const GnssAraimLinearizedInput valid_input = make_linearized_from_az_el({
      {0.0, 60.0},
      {60.0, 50.0},
      {120.0, 55.0},
      {200.0, 45.0},
      {280.0, 50.0},
  });

  struct WeightCase {
    const char* name;
    double value;
  };
  const std::vector<WeightCase> cases = {
      {"nan_weight", std::numeric_limits<double>::quiet_NaN()},
      {"inf_weight", std::numeric_limits<double>::infinity()},
      {"zero_weight", 0.0},
      {"negative_weight", -1.0},
  };

  GnssAraimEvaluator solver(GnssAraimParams{});
  for (const auto& c : cases) {
    // runLinearized consumes W=1/sigma^2 directly, so invalid sigma/variance
    // is represented here by invalid or non-positive measurement weight.
    GnssAraimLinearizedInput input = valid_input;
    input.W(2) = c.value;
    const GnssAraimResult result = solver.runLinearized(input, 0);
    SCOPED_TRACE(c.name);
    ExpectFiniteConservativeInvalid(result);
    EXPECT_EQ(result.n_hypotheses, 0);
  }
}

TEST(AraimFaultInjectionTest, Prn3ResidualBiasIncreasesSeparationAndDetectsLargeFault) {
  const GnssAraimLinearizedInput base_input = make_linearized_from_az_el({
      {0.0, 60.0},
      {60.0, 50.0},
      {120.0, 55.0},
      {200.0, 45.0},
      {280.0, 50.0},
      {330.0, 65.0},
  });

  GnssAraimParams params;
  params.dynamic_budget = false;
  params.K_ff = 5.451310438136472;
  params.K_fa = 4.5;
  params.K_md = 0.0;
  params.enable_constellation_faults = false;
  params.enable_trunk_hypotheses = false;
  params.parallel_hypotheses = false;

  GnssAraimEvaluator solver(params);

  double previous_d_horiz = -1.0;
  double previous_d_vert = -1.0;
  double zero_bias_hpl = 0.0;
  double zero_bias_vpl = 0.0;
  double zero_bias_d_horiz = 0.0;
  double zero_bias_d_vert = 0.0;

  struct BiasCase {
    const char* label;
    double bias_m;
  };
  const std::vector<BiasCase> bias_cases = {
      {"bias_0m", 0.0},
      {"bias_1m", 1.0},
      {"bias_3m", 3.0},
      {"bias_5m", 5.0},
      {"bias_10m", 10.0},
      {"bias_20m", 20.0},
  };

  for (const auto& c : bias_cases) {
    const double bias_m = c.bias_m;
    GnssAraimLinearizedInput input = base_input;
    input.r(2) = bias_m;

    const GnssAraimResult result = solver.runLinearized(input, 0);
    const SubsetSolution* prn3_subset = find_satellite_subset(result, 3);

    SCOPED_TRACE(::testing::Message()
                 << "bias_m=" << bias_m
                 << " HPL=" << result.HPL
                 << " VPL=" << result.VPL
                 << " n_detected=" << result.n_detected
                 << " worst_hyp=" << result.worst_hyp);

    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(std::isfinite(result.HPL));
    EXPECT_TRUE(std::isfinite(result.VPL));
    EXPECT_TRUE(std::isfinite(result.PL_E));
    EXPECT_TRUE(std::isfinite(result.PL_N));
    EXPECT_TRUE(std::isfinite(result.PL_U));
    EXPECT_FALSE(std::isnan(result.HPL));
    EXPECT_FALSE(std::isnan(result.VPL));
    EXPECT_FALSE(std::isinf(result.HPL));
    EXPECT_FALSE(std::isinf(result.VPL));

    ASSERT_NE(prn3_subset, nullptr);
    EXPECT_TRUE(prn3_subset->valid);
    EXPECT_FALSE(prn3_subset->degenerate);
    EXPECT_TRUE(std::isfinite(prn3_subset->d_horiz));
    EXPECT_TRUE(std::isfinite(prn3_subset->d_vert));

    const std::string prefix = c.label;
    ::testing::Test::RecordProperty((prefix + "_HPL").c_str(), result.HPL);
    ::testing::Test::RecordProperty((prefix + "_VPL").c_str(), result.VPL);
    ::testing::Test::RecordProperty((prefix + "_prn3_d_horiz").c_str(), prn3_subset->d_horiz);
    ::testing::Test::RecordProperty((prefix + "_prn3_d_vert").c_str(), prn3_subset->d_vert);
    ::testing::Test::RecordProperty((prefix + "_n_detected").c_str(), result.n_detected);
    ::testing::Test::RecordProperty((prefix + "_n_hypotheses").c_str(), result.n_hypotheses);
    ::testing::Test::RecordProperty((prefix + "_worst_hyp").c_str(), result.worst_hyp);
    ::testing::Test::RecordProperty((prefix + "_excluded_prn3").c_str(),
                   contains_int(result.excluded_prns, 3) ? 1 : 0);
    ::testing::Test::RecordProperty((prefix + "_valid").c_str(), result.valid ? 1 : 0);
    ::testing::Test::RecordProperty((prefix + "_failure_reason").c_str(),
                   prn3_subset->failure_reason.empty()
                       ? "none"
                       : prn3_subset->failure_reason.c_str());

    EXPECT_GE(prn3_subset->d_horiz + 1e-12, previous_d_horiz);
    EXPECT_GE(prn3_subset->d_vert + 1e-12, previous_d_vert);
    previous_d_horiz = prn3_subset->d_horiz;
    previous_d_vert = prn3_subset->d_vert;

    if (bias_m == 0.0) {
      zero_bias_hpl = result.HPL;
      zero_bias_vpl = result.VPL;
      zero_bias_d_horiz = prn3_subset->d_horiz;
      zero_bias_d_vert = prn3_subset->d_vert;
      EXPECT_EQ(result.n_detected, 0);
      EXPECT_TRUE(result.excluded_prns.empty());
      EXPECT_NEAR(prn3_subset->d_horiz, 0.0, 1e-12);
      EXPECT_NEAR(prn3_subset->d_vert, 0.0, 1e-12);
    }

    if (result.n_detected > 0) {
      EXPECT_TRUE(contains_int(result.excluded_prns, 3));
    }

    if (bias_m == 20.0) {
      EXPECT_GT(result.n_detected, 0);
      EXPECT_TRUE(contains_int(result.excluded_prns, 3));
      EXPECT_GT(prn3_subset->d_horiz, zero_bias_d_horiz + 1.0);
      EXPECT_GT(prn3_subset->d_vert, zero_bias_d_vert + 1.0);
      EXPECT_TRUE(std::abs(result.HPL - zero_bias_hpl) > 1e-9 ||
                  std::abs(result.VPL - zero_bias_vpl) > 1e-9);
    }
  }
}

// ============================================================================
// §1.9: Step 9 regression — compute_core decomposition
// ============================================================================

// A1: Comprehensive round-trip for multiple satellite counts
TEST_F(GnssAraimEvaluatorTest, ComputeCoreDecompositionRegression) {
  GnssAraimEvaluator eval(default_params());
  for (int n_sats : {4, 6, 8, 10}) {
    GnssEpoch epoch = make_epoch(n_sats);
    GnssAraimResult direct = eval.run(epoch, 0);
    ASSERT_TRUE(direct.valid) << "n_sats=" << n_sats;

    auto input = GnssAraimEvaluator::buildLinearizedInputFromGnssEpoch(epoch);
    GnssAraimResult linear = eval.runLinearized(input, 0);
    ASSERT_TRUE(linear.valid) << "n_sats=" << n_sats;

    EXPECT_NEAR(direct.HPL, linear.HPL, 1e-12) << "n_sats=" << n_sats;
    EXPECT_NEAR(direct.VPL, linear.VPL, 1e-12) << "n_sats=" << n_sats;
    EXPECT_NEAR(direct.PL_E, linear.PL_E, 1e-12) << "n_sats=" << n_sats;
    EXPECT_NEAR(direct.PL_N, linear.PL_N, 1e-12) << "n_sats=" << n_sats;
    EXPECT_NEAR(direct.PL_U, linear.PL_U, 1e-12) << "n_sats=" << n_sats;
    EXPECT_NEAR(direct.pl_ff, linear.pl_ff, 1e-12) << "n_sats=" << n_sats;
    EXPECT_EQ(direct.n_hypotheses, linear.n_hypotheses) << "n_sats=" << n_sats;
    EXPECT_EQ(direct.n_detected, linear.n_detected) << "n_sats=" << n_sats;
    EXPECT_EQ(direct.worst_hyp, linear.worst_hyp) << "n_sats=" << n_sats;
    EXPECT_EQ(direct.n_degenerate_hypotheses, linear.n_degenerate_hypotheses)
        << "n_sats=" << n_sats;
    EXPECT_EQ(direct.has_degenerate_hypothesis, linear.has_degenerate_hypothesis)
        << "n_sats=" << n_sats;
    EXPECT_EQ(direct.n_trunk_placeholders, linear.n_trunk_placeholders)
        << "n_sats=" << n_sats;
    EXPECT_EQ(direct.subsets.size(), linear.subsets.size()) << "n_sats=" << n_sats;
  }
}

// A2: Budget fields are populated and positive
TEST_F(GnssAraimEvaluatorTest, ComputeCoreBudgetFieldsPopulated) {
  GnssAraimEvaluator eval(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = eval.run(epoch, 0);
  ASSERT_TRUE(r.valid);
  EXPECT_GT(r.K_ff_used, 0.0);
  EXPECT_GT(r.K_fa_used, 0.0);
  EXPECT_TRUE(std::isfinite(r.K_ff_used));
  EXPECT_TRUE(std::isfinite(r.K_fa_used));
}

// A3: Fault-free PL is max of horizontal components
TEST_F(GnssAraimEvaluatorTest, FaultFreePLMatchesDirect) {
  GnssAraimEvaluator eval(default_params());
  GnssEpoch epoch = make_epoch(8);
  GnssAraimResult r = eval.run(epoch, 0);
  ASSERT_TRUE(r.valid);
  EXPECT_DOUBLE_EQ(r.pl_ff, std::max(r.pl_ff_E, r.pl_ff_N));
}

TEST_F(LidarAraimTest, HypothesesEnumerateSourceTargetAndLevel) {
  LidarAraim lidar_araim(default_params());
  auto snapshot = make_snapshot();
  snapshot.blocks.push_back(make_block(10, 0, 1.0, 0.0));
  snapshot.blocks.push_back(make_block(10, 1, 1.0, 0.0));
  snapshot.blocks.push_back(make_block(20, 0, 1.0, 0.0));

  const auto result = lidar_araim.run(snapshot, make_fgo());
  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.n_hypotheses, 5);
}

TEST_F(LidarAraimTest, SingleBlockSubsetMatchesManualSolve) {
  auto params = default_params();
  params.alpha_H = 0.0;
  params.alpha_V = 0.0;
  LidarAraim lidar_araim(params);
  auto snapshot = make_snapshot();
  snapshot.blocks.push_back(make_block(10, 0, 1.0, 1.0));

  const auto result = lidar_araim.run(snapshot, make_fgo());
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
  LidarAraim lidar_araim(params);

  auto good = make_snapshot();
  good.blocks.push_back(make_block(10, 0, 1.0, 0.0, 0.05, 0.98, 1.2, 0.05));

  auto bad = make_snapshot();
  bad.blocks.push_back(make_block(10, 0, 1.0, 0.0, 0.5, 0.4, 50.0, 3.0));

  const auto good_result = lidar_araim.run(good, make_fgo());
  const auto bad_result = lidar_araim.run(bad, make_fgo());

  ASSERT_TRUE(good_result.valid);
  ASSERT_TRUE(bad_result.valid);
  EXPECT_GT(bad_result.HPL, good_result.HPL);
  EXPECT_GT(bad_result.VPL, good_result.VPL);
}

TEST_F(LidarAraimTest, GpuBackendBlocksProduceValidGroupedResult) {
  LidarAraim lidar_araim(default_params());
  auto snapshot = make_snapshot();

  auto block0 = make_block(10, 0, 1.0, 0.2);
  auto block1 = make_block(20, 1, 1.5, 0.4);
  block0.backend = LidarAraimBlock::Backend::GPU;
  block1.backend = LidarAraimBlock::Backend::GPU;

  snapshot.blocks.push_back(block0);
  snapshot.blocks.push_back(block1);

  const auto result = lidar_araim.run(snapshot, make_fgo());
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

  LidarAraim lidar_araim(params);
  auto snapshot_300 = make_snapshot();
  auto snapshot_3000 = make_snapshot();
  snapshot_300.blocks.push_back(block_300);
  snapshot_3000.blocks.push_back(block_3000);

  const auto result_300 = lidar_araim.run(snapshot_300, make_fgo());
  const auto result_3000 = lidar_araim.run(snapshot_3000, make_fgo());
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
  LidarAraim lidar_araim(params);

  auto snapshot = make_snapshot();
  for (int i = 0; i < 12; ++i) {
    auto block = make_block(100 + i, 0, 0.1, 0.0);
    block.target_distance_m = static_cast<double>(12 - i);
    snapshot.blocks.push_back(block);
  }

  const auto result = lidar_araim.run(snapshot, make_fgo());
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
  LidarAraim lidar_araim(params);

  auto snapshot = make_snapshot();
  auto block = make_block(10, 0, -1.0, 0.0);
  snapshot.blocks.push_back(block);

  const auto result = lidar_araim.run(snapshot, make_fgo());
  ASSERT_TRUE(result.valid);
  ASSERT_FALSE(result.subsets.empty());
  const auto& ss = result.subsets.front();
  ASSERT_TRUE(ss.valid);
  EXPECT_LT(ss.sigma_ss_raw_E_m2, 0.0);
  EXPECT_TRUE(ss.sigma_ss_fallback_E);
  EXPECT_GE(ss.sigma_ss_E, params.sigma_ss_min_m);
}

// ============================================================================
// LiDAR Integrity Evaluator Validation: Synthetic FIM monotonicity
// ============================================================================
TEST(LidarIntegrityValidationTest, FimMonotonicity) {
  const Eigen::Matrix3d fim_strong =
      Eigen::Vector3d(100.0, 100.0, 100.0).asDiagonal();
  const Eigen::Matrix3d fim_weak =
      Eigen::Vector3d(25.0, 25.0, 25.0).asDiagonal();

  const auto strong = evaluate_synthetic_lidar_fim(fim_strong);
  const auto weak = evaluate_synthetic_lidar_fim(fim_weak);

  ::testing::Test::RecordProperty("strong_lidar_valid", strong.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty("strong_lidar_hpl", strong.result.HPL);
  ::testing::Test::RecordProperty("strong_lidar_vpl", strong.result.VPL);
  ::testing::Test::RecordProperty("strong_lidar_pl_e", strong.result.PL_E);
  ::testing::Test::RecordProperty("strong_lidar_pl_n", strong.result.PL_N);
  ::testing::Test::RecordProperty("strong_lidar_pl_u", strong.result.PL_U);
  ::testing::Test::RecordProperty("strong_condition_number", strong.condition_number);
  ::testing::Test::RecordProperty("weak_lidar_valid", weak.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty("weak_lidar_hpl", weak.result.HPL);
  ::testing::Test::RecordProperty("weak_lidar_vpl", weak.result.VPL);
  ::testing::Test::RecordProperty("weak_lidar_pl_e", weak.result.PL_E);
  ::testing::Test::RecordProperty("weak_lidar_pl_n", weak.result.PL_N);
  ::testing::Test::RecordProperty("weak_lidar_pl_u", weak.result.PL_U);
  ::testing::Test::RecordProperty("weak_condition_number", weak.condition_number);

  ASSERT_TRUE(strong.result.valid);
  ASSERT_TRUE(weak.result.valid);
  EXPECT_TRUE(is_finite_lidar_pl(strong.result));
  EXPECT_TRUE(is_finite_lidar_pl(weak.result));

  EXPECT_LE(strong.result.PL_E, weak.result.PL_E);
  EXPECT_LE(strong.result.PL_N, weak.result.PL_N);
  EXPECT_LE(strong.result.PL_U, weak.result.PL_U);
  EXPECT_LE(strong.result.HPL, weak.result.HPL);
  EXPECT_LE(strong.result.VPL, weak.result.VPL);
}

TEST(LidarIntegrityValidationTest, DegenerateFimProducesLargeDirectionalPlOrInvalid) {
  const Eigen::Matrix3d fim_degenerate =
      Eigen::Vector3d(100.0, 1.0e-6, 100.0).asDiagonal();

  const auto degenerate = evaluate_synthetic_lidar_fim(fim_degenerate);

  ::testing::Test::RecordProperty("degenerate_lidar_valid",
                 degenerate.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty("degenerate_lidar_hpl", degenerate.result.HPL);
  ::testing::Test::RecordProperty("degenerate_lidar_vpl", degenerate.result.VPL);
  ::testing::Test::RecordProperty("degenerate_lidar_pl_e", degenerate.result.PL_E);
  ::testing::Test::RecordProperty("degenerate_lidar_pl_n", degenerate.result.PL_N);
  ::testing::Test::RecordProperty("degenerate_lidar_pl_u", degenerate.result.PL_U);
  ::testing::Test::RecordProperty("degenerate_condition_number",
                 degenerate.condition_number);
  ::testing::Test::RecordProperty("degenerate_lidar_worst_mode",
                 degenerate.result.worst_mode.c_str());
  ::testing::Test::RecordProperty("degenerate_failure_reason",
                 degenerate.failure_reason.empty()
                     ? "none"
                     : degenerate.failure_reason.c_str());

  EXPECT_TRUE(is_finite_lidar_pl(degenerate.result));
  EXPECT_TRUE(!degenerate.result.valid ||
              degenerate.condition_number > 1.0e6);

  if (degenerate.result.valid) {
    EXPECT_GT(degenerate.result.PL_N, 100.0 * degenerate.result.PL_E);
    EXPECT_GT(degenerate.result.PL_N, 100.0 * degenerate.result.PL_U);
  }
}

TEST(LidarIntegrityValidationTest, SingularFimIsInvalidOrConservativeFinite) {
  const Eigen::Matrix3d fim_singular =
      Eigen::Vector3d(100.0, 0.0, 100.0).asDiagonal();

  const auto singular = evaluate_synthetic_lidar_fim(fim_singular);

  ::testing::Test::RecordProperty("singular_lidar_valid", singular.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty("singular_lidar_hpl", singular.result.HPL);
  ::testing::Test::RecordProperty("singular_lidar_vpl", singular.result.VPL);
  ::testing::Test::RecordProperty("singular_lidar_pl_e", singular.result.PL_E);
  ::testing::Test::RecordProperty("singular_lidar_pl_n", singular.result.PL_N);
  ::testing::Test::RecordProperty("singular_lidar_pl_u", singular.result.PL_U);
  ::testing::Test::RecordProperty("singular_condition_number",
                 singular.condition_number);
  ::testing::Test::RecordProperty("singular_lidar_worst_mode",
                 singular.result.worst_mode.c_str());
  ::testing::Test::RecordProperty("singular_failure_reason",
                 singular.failure_reason.c_str());

  EXPECT_TRUE(is_finite_lidar_pl(singular.result));
  EXPECT_FALSE(singular.result.valid);
  EXPECT_GE(singular.result.HPL, 1.0e9 * 0.99);
  EXPECT_GE(singular.result.VPL, 1.0e9 * 0.99);
  EXPECT_FALSE(singular.failure_reason.empty());
}

TEST(LidarIntegrityValidationTest, GeometryObservabilityTrend) {
  const Eigen::Matrix3d fim_feature =
      Eigen::Vector3d(100.0, 100.0, 80.0).asDiagonal();
  const Eigen::Matrix3d fim_corridor =
      Eigen::Vector3d(5.0, 100.0, 60.0).asDiagonal();
  const Eigen::Matrix3d fim_sparse =
      Eigen::Vector3d(1.0, 1.0, 1.0).asDiagonal();

  const auto feature = evaluate_synthetic_lidar_fim(fim_feature);
  const auto corridor = evaluate_synthetic_lidar_fim(fim_corridor);
  const auto sparse = evaluate_synthetic_lidar_fim(fim_sparse);

  ::testing::Test::RecordProperty("feature_lidar_valid", feature.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty("feature_lidar_hpl", feature.result.HPL);
  ::testing::Test::RecordProperty("feature_lidar_vpl", feature.result.VPL);
  ::testing::Test::RecordProperty("feature_lidar_pl_e", feature.result.PL_E);
  ::testing::Test::RecordProperty("feature_lidar_pl_n", feature.result.PL_N);
  ::testing::Test::RecordProperty("feature_lidar_pl_u", feature.result.PL_U);
  ::testing::Test::RecordProperty("feature_condition_number", feature.condition_number);
  ::testing::Test::RecordProperty("feature_lidar_worst_mode",
                 feature.result.worst_mode.c_str());
  ::testing::Test::RecordProperty("feature_failure_reason",
                 feature.failure_reason.empty()
                     ? "none"
                     : feature.failure_reason.c_str());

  ::testing::Test::RecordProperty("corridor_lidar_valid", corridor.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty("corridor_lidar_hpl", corridor.result.HPL);
  ::testing::Test::RecordProperty("corridor_lidar_vpl", corridor.result.VPL);
  ::testing::Test::RecordProperty("corridor_lidar_pl_e", corridor.result.PL_E);
  ::testing::Test::RecordProperty("corridor_lidar_pl_n", corridor.result.PL_N);
  ::testing::Test::RecordProperty("corridor_lidar_pl_u", corridor.result.PL_U);
  ::testing::Test::RecordProperty("corridor_condition_number", corridor.condition_number);
  ::testing::Test::RecordProperty("corridor_lidar_worst_mode",
                 corridor.result.worst_mode.c_str());
  ::testing::Test::RecordProperty("corridor_failure_reason",
                 corridor.failure_reason.empty()
                     ? "none"
                     : corridor.failure_reason.c_str());

  ::testing::Test::RecordProperty("sparse_lidar_valid", sparse.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty("sparse_lidar_hpl", sparse.result.HPL);
  ::testing::Test::RecordProperty("sparse_lidar_vpl", sparse.result.VPL);
  ::testing::Test::RecordProperty("sparse_lidar_pl_e", sparse.result.PL_E);
  ::testing::Test::RecordProperty("sparse_lidar_pl_n", sparse.result.PL_N);
  ::testing::Test::RecordProperty("sparse_lidar_pl_u", sparse.result.PL_U);
  ::testing::Test::RecordProperty("sparse_condition_number", sparse.condition_number);
  ::testing::Test::RecordProperty("sparse_lidar_worst_mode",
                 sparse.result.worst_mode.c_str());
  ::testing::Test::RecordProperty("sparse_failure_reason",
                 sparse.failure_reason.empty()
                     ? "none"
                     : sparse.failure_reason.c_str());

  ASSERT_TRUE(feature.result.valid);
  ASSERT_TRUE(corridor.result.valid);
  EXPECT_TRUE(is_finite_lidar_pl(feature.result));
  EXPECT_TRUE(is_finite_lidar_pl(corridor.result));
  EXPECT_TRUE(is_finite_lidar_pl(sparse.result));

  EXPECT_LT(feature.result.HPL, corridor.result.HPL);
  if (sparse.result.valid) {
    EXPECT_LT(corridor.result.HPL, sparse.result.HPL);
  } else {
    EXPECT_FALSE(sparse.failure_reason.empty());
  }

  EXPECT_GT(corridor.result.PL_E, feature.result.PL_E);
  EXPECT_GT(corridor.condition_number, feature.condition_number);
  EXPECT_FALSE(corridor.result.worst_mode.empty());
}

TEST(LidarIntegrityValidationTest, RuntimeBlockBridgeGeometryTrend) {
  const auto feature = run_runtime_block_bridge_case(
      "feature", Eigen::Vector3d(100.0, 100.0, 80.0));
  const auto corridor = run_runtime_block_bridge_case(
      "corridor", Eigen::Vector3d(1.0, 100.0, 60.0));
  const auto sparse = run_runtime_block_bridge_case(
      "sparse", Eigen::Vector3d(0.001, 1.0, 0.001));

  auto record = [](const char* prefix, const RuntimeBlockBridgeRun& run) {
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_lidar_valid").c_str(),
        run.result.valid ? 1 : 0);
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_lidar_hpl").c_str(), run.result.HPL);
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_lidar_vpl").c_str(), run.result.VPL);
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_lidar_pl_e").c_str(), run.result.PL_E);
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_lidar_pl_n").c_str(), run.result.PL_N);
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_lidar_pl_u").c_str(), run.result.PL_U);
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_condition_number").c_str(),
        run.condition_number);
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_condition_number_full").c_str(),
        run.result.condition_number_full);
    ::testing::Test::RecordProperty(
        (std::string(prefix) + "_worst_mode").c_str(),
        run.result.worst_mode.c_str());
    if (run.result.worst_hyp >= 0 &&
        run.result.worst_hyp < static_cast<int>(run.result.subsets.size())) {
      const auto& ss =
          run.result.subsets[static_cast<std::size_t>(run.result.worst_hyp)];
      ::testing::Test::RecordProperty(
          (std::string(prefix) + "_worst_subset_condition").c_str(),
          ss.condition_number_subset);
      ::testing::Test::RecordProperty(
          (std::string(prefix) + "_worst_ss_fallback_e").c_str(),
          ss.sigma_ss_fallback_E ? 1 : 0);
      ::testing::Test::RecordProperty(
          (std::string(prefix) + "_worst_ss_fallback_n").c_str(),
          ss.sigma_ss_fallback_N ? 1 : 0);
      ::testing::Test::RecordProperty(
          (std::string(prefix) + "_worst_ss_fallback_u").c_str(),
          ss.sigma_ss_fallback_U ? 1 : 0);
    }
  };
  record("bridge_feature", feature);
  record("bridge_corridor", corridor);
  record("bridge_sparse", sparse);

  ASSERT_TRUE(feature.result.valid);
  ASSERT_TRUE(corridor.result.valid);
  ASSERT_TRUE(sparse.result.valid);
  EXPECT_TRUE(is_finite_lidar_pl(feature.result));
  EXPECT_TRUE(is_finite_lidar_pl(corridor.result));
  EXPECT_TRUE(is_finite_lidar_pl(sparse.result));
  EXPECT_FALSE(feature.result.worst_mode.empty());
  EXPECT_FALSE(corridor.result.worst_mode.empty());
  EXPECT_FALSE(sparse.result.worst_mode.empty());
  EXPECT_GT(corridor.condition_number, feature.condition_number);
  EXPECT_GT(sparse.condition_number, corridor.condition_number);

  const bool sensitivity_pass =
      corridor.result.HPL > feature.result.HPL &&
      sparse.result.HPL > corridor.result.HPL &&
      corridor.result.PL_E > feature.result.PL_E;
  const bool sensitivity_suppressed =
      corridor.condition_number > feature.condition_number &&
      corridor.result.HPL <= 1.2 * feature.result.HPL;
  ::testing::Test::RecordProperty(
      "runtime_block_bridge_sensitivity",
      sensitivity_pass ? "pass" : "suppressed_or_weak");

  EXPECT_TRUE(sensitivity_pass || sensitivity_suppressed);
}

TEST(LidarIntegrityValidationTest, BlockFaultInjectionBadBlockDetected) {
  const auto clean = run_lidar_block_fault_injection(
      {0.1, 0.1, 0.1, 0.1, 0.1});
  const auto injected = run_lidar_block_fault_injection(
      {0.1, 0.1, 10.0, 0.1, 0.1});

  const auto worst =
      find_worst_single_block_hypothesis(injected.result, injected.snapshot);
  const auto bad_block = find_single_block_hypothesis_for_block(
      injected.result, injected.snapshot, 3);

  write_lidar_block_fault_metrics_csv(clean, injected, worst, bad_block);

  ::testing::Test::RecordProperty("block_fault_lidar_valid",
                 injected.result.valid ? 1 : 0);
  ::testing::Test::RecordProperty("block_fault_lidar_hpl", injected.result.HPL);
  ::testing::Test::RecordProperty("block_fault_lidar_vpl", injected.result.VPL);
  ::testing::Test::RecordProperty("block_fault_lidar_pl_e", injected.result.PL_E);
  ::testing::Test::RecordProperty("block_fault_lidar_pl_n", injected.result.PL_N);
  ::testing::Test::RecordProperty("block_fault_lidar_pl_u", injected.result.PL_U);
  ::testing::Test::RecordProperty("block_fault_n_hypotheses",
                 injected.result.n_hypotheses);
  ::testing::Test::RecordProperty("block_fault_n_detected", injected.result.n_detected);
  ::testing::Test::RecordProperty("block_fault_lidar_worst_mode",
                 injected.result.worst_mode.c_str());
  ::testing::Test::RecordProperty("block_fault_worst_block_id", worst.block_id);
  ::testing::Test::RecordProperty("block_fault_worst_block_mode", worst.mode.c_str());
  ::testing::Test::RecordProperty("block_fault_bad_block_d_e", bad_block.d_E);
  ::testing::Test::RecordProperty("block_fault_bad_block_t_e", bad_block.T_E);
  ::testing::Test::RecordProperty("block_fault_bad_block_pl_e", bad_block.PL_E);
  ::testing::Test::RecordProperty("block_fault_bad_block_detected",
                 bad_block.fault_detected ? 1 : 0);
  ::testing::Test::RecordProperty("block_fault_clean_hpl", clean.result.HPL);
  ::testing::Test::RecordProperty("block_fault_clean_vpl", clean.result.VPL);

  EXPECT_TRUE(is_finite_lidar_pl(injected.result));
  if (!injected.result.valid) {
    EXPECT_GE(injected.result.HPL, 1.0e9 * 0.99);
    EXPECT_GE(injected.result.VPL, 1.0e9 * 0.99);
    return;
  }

  ASSERT_TRUE(clean.result.valid);
  ASSERT_TRUE(is_finite_lidar_pl(clean.result));
  EXPECT_GT(injected.result.n_detected, 0);
  EXPECT_FALSE(injected.result.worst_mode.empty());
  EXPECT_EQ(bad_block.block_id, 3);
  EXPECT_TRUE(bad_block.fault_detected);
  EXPECT_GT(std::abs(bad_block.d_E), bad_block.T_E);
  EXPECT_EQ(worst.block_id, 3);
  EXPECT_GT(injected.result.HPL, clean.result.HPL);
}

TEST(LidarIntegrityValidationTest, PointCloudToFimToPlTrend) {
  const auto feature = evaluate_lidar_cloud_csv(
      "feature_rich", "feature_rich_cloud.csv");
  const auto corridor = evaluate_lidar_cloud_csv(
      "corridor", "corridor_cloud.csv");
  const auto sparse = evaluate_lidar_cloud_csv(
      "sparse", "sparse_cloud.csv");

  write_lidar_pointcloud_metrics_csv({feature, corridor, sparse});

  record_lidar_cloud_eval("feature", feature);
  record_lidar_cloud_eval("corridor", corridor);
  record_lidar_cloud_eval("sparse", sparse);

  ASSERT_FALSE(feature.points.empty());
  ASSERT_FALSE(corridor.points.empty());
  ASSERT_FALSE(sparse.points.empty());

  ASSERT_TRUE(feature.fim.valid) << feature.failure_reason;
  ASSERT_TRUE(corridor.fim.valid) << corridor.failure_reason;
  ASSERT_TRUE(feature.pl.result.valid) << feature.failure_reason;
  ASSERT_TRUE(corridor.pl.result.valid) << corridor.failure_reason;
  EXPECT_TRUE(is_finite_lidar_pl(feature.pl.result));
  EXPECT_TRUE(is_finite_lidar_pl(corridor.pl.result));
  EXPECT_TRUE(is_finite_lidar_pl(sparse.pl.result));

  EXPECT_GT(feature.fim.n_valid_normals, corridor.fim.n_valid_normals / 2);
  EXPECT_GT(corridor.fim.condition, feature.fim.condition);
  EXPECT_LT(feature.pl.result.HPL, corridor.pl.result.HPL);

  if (sparse.fim.valid && sparse.pl.result.valid) {
    EXPECT_LT(corridor.pl.result.HPL, sparse.pl.result.HPL);
  } else {
    EXPECT_FALSE(sparse.failure_reason.empty());
  }
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
  rep.HPL = 1.0;
  rep.VPL = 0.5;
  rep.HAL = 5.0;
  rep.VAL = 5.0;
  rep.AL  = 5.0;
  rep.im_h = rep.HAL - rep.HPL;  // 4.0
  rep.im_v = rep.VAL - rep.VPL;  // 4.5
  rep.IM   = std::min(rep.im_h, rep.im_v);  // 4.0
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

TEST_F(GnssAraimEvaluatorTest, IntegrityMonitorFusesGnssAndLidarByPerAxisMax) {
  IntegrityMonitor::Params params;
  params.K_pl = 1.0;
  params.HAL_trunk_default = 100.0;
  params.VAL_default = 100.0;
  params.gnss_araim_params = default_params();
  params.gnss_araim_params.dynamic_budget = false;
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
// §6: GnssAraimResult field consistency
// ============================================================================
TEST(AraimResultTest, DefaultsAreConservative) {
  GnssAraimResult r;
  EXPECT_FALSE(r.valid);
  EXPECT_GE(r.HPL, 1e9);
  EXPECT_GE(r.VPL, 1e9);
  EXPECT_EQ(r.n_hypotheses, 0);
  EXPECT_EQ(r.n_detected, 0);
}

TEST(AraimResultTest, AliasesAreConsistent) {
  GnssAraimEvaluator evaluator;
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

  GnssAraimResult r = evaluator.run(epoch, 0);
  if (r.valid) {
    // Aliases should match
    EXPECT_DOUBLE_EQ(r.pl_araim, r.HPL);
    EXPECT_DOUBLE_EQ(r.vpl_araim, r.VPL);
  }
}

// ============================================================================
// §0: Baseline integrity monitor tests (pre-refactor golden outputs)
// ============================================================================
class IntegrityMonitorBaselineTest : public ::testing::Test {
 protected:
  IntegrityMonitor::Params default_monitor_params() {
    IntegrityMonitor::Params p;
    p.K_pl = 3.0;
    p.HAL_trunk_default = 10.0;
    p.VAL_default = 20.0;
    p.al_min = 0.5;
    p.gamma_H = 0.5;
    p.gamma_V = 0.8;
    p.h_min = 2.0;
    p.canopy_height_default = 5.0;
    p.VAL_max = 50.0;
    p.r_drone = 0.35;
    p.recovery_count = 5;
    p.nominal_fraction = 0.6;
    return p;
  }

  glim::EstimationFrame make_frame(double sigma_diag = 0.01) {
    glim::EstimationFrame f;
    f.stamp = 100.0;
    f.sigma_p = Eigen::Matrix3d::Identity() * sigma_diag;
    f.T_world_imu = Eigen::Isometry3d::Identity();
    f.T_world_imu.translation() = Eigen::Vector3d(0, 0, 10);
    return f;
  }
};

namespace iap {

class IntegrityMonitorTestAccess {
 public:
  static void compute_margins(const IntegrityMonitor& monitor,
                              IntegrityReport& report) {
    monitor.computeIntegrityMargins(report);
  }

  static void update_state_and_mode(IntegrityMonitor& monitor,
                                    IntegrityReport& report) {
    monitor.updateStateAndPlannerMode(report);
  }
};

}  // namespace iap

namespace {

IntegrityMonitor::Params hv_semantics_params() {
  IntegrityMonitor::Params p;
  p.recovery_count = 1;
  p.nominal_fraction = 1.0;
  return p;
}

IntegrityReport evaluate_hv_semantics(double hpl,
                                      double hal,
                                      double vpl,
                                      double val) {
  IntegrityMonitor monitor(hv_semantics_params());
  IntegrityReport report;
  report.HPL = hpl;
  report.VPL = vpl;
  report.PL = hpl;
  report.PL_E = hpl;
  report.PL_N = hpl;
  report.PL_U = vpl;
  report.HAL = hal;
  report.VAL = val;
  report.AL = std::min(hal, val);
  report.araim_n_det = 0;

  IntegrityMonitorTestAccess::compute_margins(monitor, report);
  IntegrityMonitorTestAccess::update_state_and_mode(monitor, report);
  return report;
}

}  // namespace

TEST(IntegrityMonitorHvSafetyTest, FourCaseSafetyTable) {
  struct Case {
    const char* name;
    double hpl;
    double hal;
    double vpl;
    double val;
    IntegrityState expected_state;
    bool expected_safe;
  };

  const std::vector<Case> cases = {
      {"both_safe", 5.0, 10.0, 6.0, 10.0, IntegrityState::SAFE, true},
      {"horizontal_unsafe", 12.0, 10.0, 6.0, 10.0, IntegrityState::UNSAFE, false},
      {"vertical_unsafe", 5.0, 10.0, 12.0, 10.0, IntegrityState::UNSAFE, false},
      {"both_unsafe", 12.0, 10.0, 12.0, 10.0, IntegrityState::UNSAFE, false},
  };

  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    const IntegrityReport report =
        evaluate_hv_semantics(c.hpl, c.hal, c.vpl, c.val);

    const double expected_im_h = c.hal - c.hpl;
    const double expected_im_v = c.val - c.vpl;
    const double expected_im = std::min(expected_im_h, expected_im_v);

    EXPECT_DOUBLE_EQ(report.im_h, expected_im_h);
    EXPECT_DOUBLE_EQ(report.im_v, expected_im_v);
    EXPECT_DOUBLE_EQ(report.IM, expected_im);
    EXPECT_EQ(report.safe(), c.expected_safe);
    EXPECT_EQ(report.is_available(), c.expected_safe);
    EXPECT_EQ(report.state, c.expected_state);
    EXPECT_EQ(report.planner_state,
              c.expected_state == IntegrityState::UNSAFE
                  ? PlannerState::HOVER
                  : PlannerState::CRUISE);
  }
}

TEST(IntegrityMonitorHvSafetyTest, VerticalViolationDominatesState) {
  const IntegrityReport report =
      evaluate_hv_semantics(5.0, 10.0, 10.0, 10.0);

  EXPECT_EQ(report.state, IntegrityState::UNSAFE);
  EXPECT_GT(report.im_h, 0.0);
  EXPECT_LE(report.im_v, 0.0);
  EXPECT_DOUBLE_EQ(report.IM, report.im_v);
  EXPECT_FALSE(report.safe());
  EXPECT_FALSE(report.is_available());
}

TEST(IntegrityMonitorHvSafetyTest, EqualityAtAlertLimitIsUnsafe) {
  {
    SCOPED_TRACE("horizontal equality");
    const IntegrityReport report =
        evaluate_hv_semantics(10.0, 10.0, 6.0, 10.0);
    EXPECT_DOUBLE_EQ(report.im_h, 0.0);
    EXPECT_GT(report.im_v, 0.0);
    EXPECT_DOUBLE_EQ(report.IM, 0.0);
    EXPECT_EQ(report.state, IntegrityState::UNSAFE);
    EXPECT_FALSE(report.safe());
    EXPECT_FALSE(report.is_available());
  }

  {
    SCOPED_TRACE("vertical equality");
    const IntegrityReport report =
        evaluate_hv_semantics(5.0, 10.0, 10.0, 10.0);
    EXPECT_GT(report.im_h, 0.0);
    EXPECT_DOUBLE_EQ(report.im_v, 0.0);
    EXPECT_DOUBLE_EQ(report.IM, 0.0);
    EXPECT_EQ(report.state, IntegrityState::UNSAFE);
    EXPECT_FALSE(report.safe());
    EXPECT_FALSE(report.is_available());
  }
}

TEST(IntegrityReportMappingTest, HvMarginsMapToRosMessage) {
  IntegrityReport report;
  report.state = IntegrityState::UNSAFE;
  report.HPL = 12.0;
  report.VPL = 6.0;
  report.PL_E = 12.0;
  report.PL_N = 4.0;
  report.PL_U = 6.0;
  report.HAL = 10.0;
  report.VAL = 10.0;
  report.im_h = report.HAL - report.HPL;
  report.im_v = report.VAL - report.VPL;
  report.IM = std::min(report.im_h, report.im_v);

  iap::msg::IntegrityReport msg;
  fill_integrity_report_msg(report, msg);

  EXPECT_DOUBLE_EQ(msg.im_h, report.im_h);
  EXPECT_DOUBLE_EQ(msg.im_v, report.im_v);
  EXPECT_DOUBLE_EQ(msg.im_min, report.IM);
  EXPECT_DOUBLE_EQ(msg.im, msg.im_min);
}

TEST(IntegrityReportMappingTest, SourceFusionAndFailureFieldsMapToRosMessage) {
  IntegrityReport report;
  report.gnss_valid = 1;
  report.gnss_HPL = 3.0;
  report.gnss_VPL = 10.0;
  report.gnss_PL_E = 3.0;
  report.gnss_PL_N = 2.0;
  report.gnss_PL_U = 10.0;

  report.lidar_valid = 0;
  report.lidar_HPL = 8.0;
  report.lidar_VPL = 4.0;
  report.lidar_PL_E = 8.0;
  report.lidar_PL_N = 4.0;
  report.lidar_PL_U = 4.0;

  report.fallback_HPL = 5.0;
  report.fallback_VPL = 5.0;

  report.fusion_mode_str = "weighted_debug_only";
  report.final_HPL_source = "LIDAR";
  report.final_VPL_source = "GNSS";
  report.final_PL_source = "GNSS";

  report.numerical_failure.fallback_pl_invalid = true;
  report.numerical_failure.gnss_araim_invalid = true;
  report.numerical_failure.lidar_integrity_invalid = true;
  report.numerical_failure.hal_invalid = true;
  report.numerical_failure.val_invalid = true;
  report.numerical_failure.im_invalid = true;
  report.numerical_failure.any_nan_rejected = true;
  report.numerical_failure.any_inf_rejected = true;
  report.numerical_failure.negative_variance_rejected = true;
  report.numerical_failure.degenerate_geometry = true;
  report.numerical_failure.failure_reason =
      "required GNSS source missing or invalid";

  iap::msg::IntegrityReport msg;
  fill_integrity_report_msg(report, msg);

  EXPECT_TRUE(msg.gnss_valid);
  EXPECT_FALSE(msg.lidar_valid);
  EXPECT_FALSE(msg.fallback_valid);
  EXPECT_DOUBLE_EQ(msg.gnss_hpl, report.gnss_HPL);
  EXPECT_DOUBLE_EQ(msg.gnss_vpl, report.gnss_VPL);
  EXPECT_DOUBLE_EQ(msg.lidar_hpl, report.lidar_HPL);
  EXPECT_DOUBLE_EQ(msg.lidar_vpl, report.lidar_VPL);
  EXPECT_DOUBLE_EQ(msg.fallback_hpl, report.fallback_HPL);
  EXPECT_DOUBLE_EQ(msg.fallback_vpl, report.fallback_VPL);
  EXPECT_DOUBLE_EQ(msg.gnss_pl_e, report.gnss_PL_E);
  EXPECT_DOUBLE_EQ(msg.gnss_pl_n, report.gnss_PL_N);
  EXPECT_DOUBLE_EQ(msg.gnss_pl_u, report.gnss_PL_U);
  EXPECT_DOUBLE_EQ(msg.lidar_pl_e, report.lidar_PL_E);
  EXPECT_DOUBLE_EQ(msg.lidar_pl_n, report.lidar_PL_N);
  EXPECT_DOUBLE_EQ(msg.lidar_pl_u, report.lidar_PL_U);
  EXPECT_EQ(msg.fusion_mode, report.fusion_mode_str);
  EXPECT_EQ(msg.final_hpl_source, report.final_HPL_source);
  EXPECT_EQ(msg.final_vpl_source, report.final_VPL_source);
  EXPECT_EQ(msg.final_pl_source, report.final_PL_source);
  EXPECT_TRUE(msg.fallback_pl_invalid);
  EXPECT_TRUE(msg.gnss_araim_invalid);
  EXPECT_TRUE(msg.lidar_integrity_invalid);
  EXPECT_TRUE(msg.hal_invalid);
  EXPECT_TRUE(msg.val_invalid);
  EXPECT_TRUE(msg.im_invalid);
  EXPECT_TRUE(msg.any_nan_rejected);
  EXPECT_TRUE(msg.any_inf_rejected);
  EXPECT_TRUE(msg.negative_variance_rejected);
  EXPECT_TRUE(msg.degenerate_geometry);
  EXPECT_EQ(msg.failure_reason, report.numerical_failure.failure_reason);
}

// T0.1 — Fallback-only PL and IM (after Step 1: H/V aware)
TEST_F(IntegrityMonitorBaselineTest, FallbackOnlyGivesCorrectIM) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(0.01);
  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  // Fallback PL = K_pl * sqrt(lambda_max) = 3.0 * 0.1 = 0.3
  EXPECT_NEAR(report.PL, 0.3, 1e-6);
  EXPECT_DOUBLE_EQ(report.HPL, report.PL);
  EXPECT_DOUBLE_EQ(report.VPL, report.PL);
  // Default VAL=20, HAL=10, AL=min(10,20)=10
  // im_h = HAL - HPL = 10 - 0.3 = 9.7
  // im_v = VAL - VPL = 20 - 0.3 = 19.7
  // IM = min(9.7, 19.7) = 9.7
  EXPECT_NEAR(report.im_h, report.HAL - report.HPL, 1e-6);
  EXPECT_NEAR(report.im_v, report.VAL - report.VPL, 1e-6);
  EXPECT_DOUBLE_EQ(report.IM, std::min(report.im_h, report.im_v));
  EXPECT_TRUE(report.safe());
}

// T0.2 — Document state transitions from scalar PL vs AL
TEST_F(IntegrityMonitorBaselineTest, StateUnsafeWhenPLExceedsAL) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(100.0);  // PL = 3.0 * 10 = 30.0, AL = 10.0
  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  EXPECT_GE(report.PL, report.AL);
  EXPECT_EQ(report.state, IntegrityState::UNSAFE);
}

// T0.3 — Vertical violation now correctly detected as UNSAFE (Step 1 fix)
TEST_F(IntegrityMonitorBaselineTest, VerticalViolationIsDetectedAsUnsafe) {
  IntegrityMonitor::Params p = default_monitor_params();
  p.VAL_default = 1.0;        // Very tight vertical limit
  p.HAL_trunk_default = 100.0; // Very loose horizontal limit
  auto monitor = IntegrityMonitor(p);

  auto frame = make_frame(0.01);
  frame.sigma_p(2, 2) = 100.0;  // Large vertical variance
  frame.sigma_p(0, 0) = 0.0001; // Small horizontal variance
  frame.sigma_p(1, 1) = 0.0001;

  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  // After Step 1: VPL > VAL → UNSAFE regardless of horizontal margin
  EXPECT_EQ(report.state, IntegrityState::UNSAFE);
  EXPECT_GT(report.im_h, 0.0);  // Horizontal still safe
  EXPECT_LT(report.im_v, 0.0);  // Vertical violated
  EXPECT_FALSE(report.safe());
}

// T0.4 — IM field consistency (after Step 1: IM = min(im_h, im_v))
TEST_F(IntegrityMonitorBaselineTest, IMIsMinOfHorizontalAndVerticalMargins) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(0.04);
  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);
  EXPECT_DOUBLE_EQ(report.im_h, report.HAL - report.HPL);
  EXPECT_DOUBLE_EQ(report.im_v, report.VAL - report.VPL);
  EXPECT_DOUBLE_EQ(report.IM, std::min(report.im_h, report.im_v));
}

// ============================================================================
// §2: Numerical guard tests (Step 2 refactor)
// ============================================================================
// T2.1 — NaN in covariance produces invalid fallback
TEST_F(IntegrityMonitorBaselineTest, NaNCovarianceGivesInvalidFallback) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(0.01);
  frame.sigma_p(0, 0) = std::numeric_limits<double>::quiet_NaN();

  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  EXPECT_TRUE(report.numerical_failure.fallback_pl_invalid);
  EXPECT_GE(report.PL, 999.0 * 0.99);
  EXPECT_GE(report.HPL, 999.0 * 0.99);
}

// T2.2 — Inf in covariance produces invalid fallback
TEST_F(IntegrityMonitorBaselineTest, InfCovarianceGivesInvalidFallback) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(0.01);
  frame.sigma_p(0, 0) = std::numeric_limits<double>::infinity();

  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  EXPECT_TRUE(report.numerical_failure.fallback_pl_invalid);
  EXPECT_GE(report.PL, 999.0 * 0.99);
}

// T2.3 — Negative variance handled
TEST_F(IntegrityMonitorBaselineTest, NegativeVarianceGivesInvalidFallback) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(0.01);
  frame.sigma_p(0, 0) = -1.0;

  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  EXPECT_TRUE(report.numerical_failure.fallback_pl_invalid);
}

// T2.4 — No NaN/Inf published in ROS fields
TEST_F(IntegrityMonitorBaselineTest, NoNaNInfInPublishedFields) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(0.01);
  frame.sigma_p.setConstant(std::numeric_limits<double>::quiet_NaN());

  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  EXPECT_TRUE(std::isfinite(report.PL) || report.PL >= 999.0 * 0.99);
  EXPECT_TRUE(std::isfinite(report.HPL) || report.HPL >= 999.0 * 0.99);
  EXPECT_TRUE(std::isfinite(report.VPL) || report.VPL >= 999.0 * 0.99);
  EXPECT_TRUE(std::isfinite(report.HAL));
  EXPECT_TRUE(std::isfinite(report.VAL));
  EXPECT_TRUE(std::isfinite(report.AL));
  EXPECT_TRUE(std::isfinite(report.IM) || report.IM >= 999.0 * 0.99);
  EXPECT_FALSE(std::isnan(report.PL));
  EXPECT_FALSE(std::isinf(report.PL));
  EXPECT_FALSE(std::isnan(report.HPL));
  EXPECT_FALSE(std::isinf(report.HPL));
}

// T2.5 — NaN in altitude produces valid default VAL
TEST_F(IntegrityMonitorBaselineTest, NaNAaltitudeUsesDefaultVAL) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  monitor.set_altitude(std::numeric_limits<double>::quiet_NaN());

  auto frame = make_frame(0.01);
  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  EXPECT_TRUE(std::isfinite(report.VAL));
  EXPECT_GT(report.VAL, 0.0);
}

// T2.6 — All sources invalid → conservative PL with flags
TEST_F(IntegrityMonitorBaselineTest, AllSourcesInvalidGivesConservativePL) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(0.01);
  frame.sigma_p.setConstant(std::numeric_limits<double>::quiet_NaN());

  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  EXPECT_TRUE(report.numerical_failure.fallback_pl_invalid);
  EXPECT_TRUE(report.has_numerical_failure());
  EXPECT_GE(report.PL, 999.0 * 0.99);
  EXPECT_EQ(report.state, IntegrityState::UNSAFE);
}

// ============================================================================
// Step 9 regression — IntegrityMonitor decomposition
// ============================================================================

// C1: GNSS source report fields populated when GNSS available
TEST_F(IntegrityMonitorBaselineTest, GnssSourceFieldsPopulated) {
  IntegrityMonitor::Params p = default_monitor_params();
  p.enable_gnss_integrity = true;
  p.enable_gnss_araim = true;
  auto monitor = IntegrityMonitor(p);
  auto frame = make_frame(0.01);

  // Build a minimal valid GnssEpoch
  GnssEpoch epoch;
  epoch.stamp = 100.0;
  epoch.gps_sec = 2100000.0;
  for (int i = 0; i < 6; ++i) {
    SatObs sat;
    sat.sat_id = i + 1;
    sat.constellation = 'G';
    sat.elevation = 0.5 + i * 0.1;
    sat.azimuth = i * 1.0;
    sat.pr_sigma = 2.0;
    sat.pr_residual = 0.1 * (i - 3);
    sat.nis_pr = 1.0;
    sat.excluded = false;
    epoch.sats.push_back(sat);
  }

  auto report = monitor.compute(frame, &epoch, nullptr, nullptr, nullptr);

  // GNSS source fields should be populated
  EXPECT_EQ(report.gnss_valid, 1);
  EXPECT_GT(report.gnss_n_hyp, 0);
  EXPECT_GT(report.gnss_HPL, 0.0);
  EXPECT_GT(report.gnss_VPL, 0.0);
  EXPECT_TRUE(std::isfinite(report.gnss_HPL));
  EXPECT_TRUE(std::isfinite(report.gnss_VPL));
}

// C2: Final source fields reflect fusion policy output
TEST_F(IntegrityMonitorBaselineTest, FinalSourceFieldsReflectFusion) {
  IntegrityMonitor::Params p = default_monitor_params();
  p.enable_gnss_integrity = true;
  p.enable_gnss_araim = true;
  p.fusion_mode = IntegrityFusionMode::MAX_PL;
  auto monitor = IntegrityMonitor(p);
  auto frame = make_frame(0.01);

  GnssEpoch epoch;
  epoch.stamp = 100.0;
  epoch.gps_sec = 2100000.0;
  for (int i = 0; i < 6; ++i) {
    SatObs sat;
    sat.sat_id = i + 1;
    sat.constellation = 'G';
    sat.elevation = 0.5 + i * 0.1;
    sat.azimuth = i * 1.0;
    sat.pr_sigma = 2.0;
    sat.pr_residual = 0.1 * (i - 3);
    sat.nis_pr = 1.0;
    sat.excluded = false;
    epoch.sats.push_back(sat);
  }

  auto report = monitor.compute(frame, &epoch, nullptr, nullptr, nullptr);

  EXPECT_FALSE(report.final_HPL_source.empty());
  EXPECT_FALSE(report.final_VPL_source.empty());
  EXPECT_FALSE(report.final_PL_source.empty());
  EXPECT_FALSE(report.fusion_mode_str.empty());
}

// C3: Fallback-only output unchanged (enhanced T0.1)
TEST_F(IntegrityMonitorBaselineTest, FallbackOnlyOutputUnchanged) {
  auto monitor = IntegrityMonitor(default_monitor_params());
  auto frame = make_frame(0.01);
  auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

  EXPECT_NEAR(report.PL, 0.3, 1e-6);
  EXPECT_DOUBLE_EQ(report.HPL, report.PL);
  EXPECT_DOUBLE_EQ(report.VPL, report.PL);
  EXPECT_NEAR(report.im_h, report.HAL - report.HPL, 1e-6);
  EXPECT_NEAR(report.im_v, report.VAL - report.VPL, 1e-6);
  EXPECT_DOUBLE_EQ(report.IM, std::min(report.im_h, report.im_v));
  EXPECT_TRUE(report.safe());

  // Verify fallback-specific fields
  EXPECT_GT(report.fallback_HPL, 0.0);
  EXPECT_GT(report.fallback_VPL, 0.0);
  EXPECT_FALSE(report.numerical_failure.fallback_pl_invalid);
  EXPECT_FALSE(report.numerical_failure.im_invalid);
}

// C4: IM formula: IM = min(HAL - HPL, VAL - VPL) for various PL values
TEST_F(IntegrityMonitorBaselineTest, IMFormulaPreserved) {
  IntegrityMonitor::Params p = default_monitor_params();
  p.HAL_trunk_default = 10.0;
  p.VAL_default = 20.0;

  // Test several frame covariance values to exercise different PL levels
  for (double sigma : {0.01, 0.25, 1.0, 4.0}) {
    auto monitor = IntegrityMonitor(p);
    auto frame = make_frame(sigma);
    auto report = monitor.compute(frame, nullptr, nullptr, nullptr, nullptr);

    const double expected_im_h = report.HAL - report.HPL;
    const double expected_im_v = report.VAL - report.VPL;
    const double expected_IM = std::min(expected_im_h, expected_im_v);

    EXPECT_NEAR(report.im_h, expected_im_h, 1e-9) << "sigma=" << sigma;
    EXPECT_NEAR(report.im_v, expected_im_v, 1e-9) << "sigma=" << sigma;
    EXPECT_DOUBLE_EQ(report.IM, std::min(report.im_h, report.im_v)) << "sigma=" << sigma;
  }
}

// C5: H/V state semantics preserved (step 1/2 behavior)
TEST_F(IntegrityMonitorBaselineTest, StateTransitionsPreserved) {
  IntegrityMonitor::Params p = default_monitor_params();
  p.recovery_count = 2;
  p.nominal_fraction = 0.6;
  auto monitor = IntegrityMonitor(p);

  // Small covariance, dimensions safe → state reflects H/V semantics
  auto frame_safe = make_frame(0.01);
  auto r1 = monitor.compute(frame_safe, nullptr, nullptr, nullptr, nullptr);
  // After Step 1 fix: im_h and im_v are positive when dimensions safe
  EXPECT_GT(r1.im_h, 0.0);
  EXPECT_GT(r1.im_v, 0.0);
  EXPECT_GT(r1.IM, 0.0);

  // Large covariance → UNSAFE
  auto frame_unsafe = make_frame(100.0);
  auto r2 = monitor.compute(frame_unsafe, nullptr, nullptr, nullptr, nullptr);
  EXPECT_EQ(r2.state, IntegrityState::UNSAFE);
  EXPECT_LT(r2.im_h, 0.0);  // im_h negative
  EXPECT_LT(r2.IM, 0.0);    // IM negative
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
// §3: IntegrityReport → ROS msg mapping (Step 3)
//
// ROS msg mapping is verified by compilation of integrity_extension.cpp
// which calls fill_integrity_report_msg(). The internal IntegrityReport
// fields are validated by the Baseline and H/V tests above.
// ============================================================================

// Step 7: Deprecated Araim alias still compiles
TEST(GnssAraimCompatibilityTest, DeprecatedAraimAliasStillCompiles) {
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  Araim::Params p;
  Araim araim(p);
  GnssEpoch epoch;
  epoch.stamp = 1.0;
  epoch.gps_sec = 2100000.0;
  for (int i = 0; i < 8; ++i) {
    SatObs s;
    s.sat_id = 100 + i;
    s.constellation = 'G';
    s.elevation = 0.5 + 0.1 * i;
    s.azimuth = M_PI / 4.0 * i;
    s.pr_sigma = 3.0;
    s.pr_residual = 0.0;
    s.excluded = false;
    epoch.sats.push_back(s);
  }
  AraimResult r = araim.run(epoch, 0);
  EXPECT_TRUE(r.valid || !r.valid);
  #pragma GCC diagnostic pop
}

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
