// LiDAR ARAIM for current-frame VGICP block hypotheses.

#include <iap/integrity/lidar_araim.hpp>

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace iap {

namespace {

using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

Matrix6d covariance_from_ldlt(const Eigen::LDLT<Matrix6d>& ldlt) {
  return ldlt.solve(Matrix6d::Identity());
}

std::string hypothesis_label(const LidarHypothesis& hyp) {
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

int unique_target_count(const LidarAraimSnapshot& snapshot) {
  std::set<long> targets;
  for (const auto& block : snapshot.blocks) {
    targets.insert(block.target_frame_id);
  }
  return static_cast<int>(targets.size());
}

LidarAraimSnapshot filter_target_window(const LidarAraimSnapshot& snapshot,
                                        const LidarAraim::Params& params) {
  const int target_window_K = std::max(1, params.target_window_K);
  if (unique_target_count(snapshot) <= target_window_K) {
    return snapshot;
  }

  struct TargetCandidate {
    long id = -1;
    double distance_m = std::numeric_limits<double>::infinity();
    double age_sec = std::numeric_limits<double>::infinity();
    bool has_distance = false;
  };

  std::map<long, TargetCandidate> by_target;
  for (const auto& block : snapshot.blocks) {
    auto& candidate = by_target[block.target_frame_id];
    candidate.id = block.target_frame_id;
    if (std::isfinite(block.target_distance_m) &&
        block.target_distance_m >= 0.0) {
      candidate.has_distance = true;
      candidate.distance_m = std::min(candidate.distance_m,
                                      block.target_distance_m);
    }
    if (std::isfinite(block.age_sec) && block.age_sec >= 0.0) {
      candidate.age_sec = std::min(candidate.age_sec, block.age_sec);
    }
  }

  std::vector<TargetCandidate> candidates;
  candidates.reserve(by_target.size());
  for (const auto& kv : by_target) {
    candidates.push_back(kv.second);
  }

  std::sort(candidates.begin(), candidates.end(),
            [](const TargetCandidate& a, const TargetCandidate& b) {
              if (a.has_distance != b.has_distance) {
                return a.has_distance;
              }
              if (a.has_distance && a.distance_m != b.distance_m) {
                return a.distance_m < b.distance_m;
              }
              if (a.age_sec != b.age_sec) {
                return a.age_sec < b.age_sec;
              }
              return a.id > b.id;
            });

  std::set<long> selected;
  for (int i = 0; i < target_window_K &&
                  i < static_cast<int>(candidates.size()); ++i) {
    selected.insert(candidates[static_cast<std::size_t>(i)].id);
  }

  LidarAraimSnapshot filtered = snapshot;
  filtered.blocks.clear();
  filtered.blocks.reserve(snapshot.blocks.size());
  for (const auto& block : snapshot.blocks) {
    if (selected.count(block.target_frame_id)) {
      filtered.blocks.push_back(block);
    }
  }
  filtered.valid = snapshot.valid && !filtered.blocks.empty();
  return filtered;
}

void update_hypothesis_risk(LidarHypothesis* hyp,
                            int block_index,
                            const LidarAraimBlock& block,
                            const LidarAraim::Params& params) {
  const auto risk = LidarAraim::compute_risk_components(block, params);
  if (risk.gamma_total >= hyp->gamma_mode) {
    hyp->gamma_mode = risk.gamma_total;
    hyp->selected_block_index = block_index;
    hyp->selected_risk = risk;
  }
}

struct SigmaSsAxis {
  double raw_m2 = 0.0;
  double used_m = 0.0;
  bool fallback = false;
};

SigmaSsAxis compute_sigma_ss_axis(const Matrix6d& Sigma0,
                                  const Matrix6d& Sigma_f,
                                  int axis,
                                  double sigma_ss_min_m) {
  SigmaSsAxis out;
  const double floor_m2 = sigma_ss_min_m * sigma_ss_min_m;
  out.raw_m2 = Sigma_f(axis, axis) - Sigma0(axis, axis);
  if (std::isfinite(out.raw_m2) && out.raw_m2 > floor_m2) {
    out.used_m = std::sqrt(std::max(out.raw_m2, floor_m2));
    return out;
  }

  const double fallback_m2 = Sigma_f(axis, axis);
  out.fallback = true;
  out.used_m = std::sqrt(std::max(
      std::isfinite(fallback_m2) ? fallback_m2 : floor_m2,
      floor_m2));
  return out;
}

}  // namespace

LidarAraim::LidarAraim() : params_{} {}

LidarAraim::LidarAraim(const Params& params) : params_{params} {}

const char* LidarAraim::to_string(const LidarHypothesis::Type type) {
  switch (type) {
    case LidarHypothesis::Type::H_SOURCE:
      return "H_SOURCE";
    case LidarHypothesis::Type::H_TARGET:
      return "H_TARGET";
    case LidarHypothesis::Type::H_LEVEL:
      return "H_LEVEL";
  }
  return "UNKNOWN";
}

const char* LidarAraim::to_string(const Params::AgeModel model) {
  switch (model) {
    case Params::AgeModel::EXP_SATURATING:
      return "exp_saturating";
    case Params::AgeModel::LINEAR_CAPPED:
      return "linear_capped";
  }
  return "unknown";
}

double LidarAraim::Q_inv(double p) {
  if (p <= 0.0) return 1e9;
  if (p >= 0.5) return 0.0;

  const double t = std::sqrt(-2.0 * std::log(p));
  constexpr double c0 = 2.515517, c1 = 0.802853, c2 = 0.010328;
  constexpr double d1 = 1.432788, d2 = 0.189269, d3 = 0.001308;
  double z = t - (c0 + c1 * t + c2 * t * t) /
                     (1.0 + d1 * t + d2 * t * t + d3 * t * t * t);

  const double Q_z = 0.5 * std::erfc(z / std::sqrt(2.0));
  const double err = Q_z - p;
  const double dQdz = -(1.0 / std::sqrt(2.0 * M_PI)) * std::exp(-0.5 * z * z);
  if (std::abs(dQdz) > 1e-30) {
    z -= err / dQdz;
  }
  return z;
}

bool LidarAraim::factorize_information(
    const Matrix6d& Lambda,
    const double eps_degen,
    Eigen::LDLT<Matrix6d>* ldlt) {
  ldlt->compute(Lambda);
  if (ldlt->info() != Eigen::Success) {
    return false;
  }

  const Eigen::Matrix<double, 6, 1> d = ldlt->vectorD();
  if (!d.allFinite()) {
    return false;
  }

  return d.minCoeff() >= eps_degen;
}

LidarRiskComponents LidarAraim::compute_risk_components(
    const LidarAraimBlock& block,
    const Params& params) {
  LidarRiskComponents risk;
  risk.gamma_rmse = std::clamp(
      block.rmse_proxy / std::max(params.rmse_ref, 1e-6),
      0.0,
      std::max(0.0, params.gamma_rmse_max));
  risk.gamma_inlier = 1.0 - std::clamp(block.inlier_fraction, 0.0, 1.0);

  const double log_ref = std::log(std::max(params.condition_ref, 1.0 + 1e-6));
  const double log_cond = std::log(std::max(block.cond_proxy, 1.0));
  risk.gamma_condition = std::clamp(
      log_ref > 1e-12 ? log_cond / log_ref : 0.0,
      0.0,
      std::max(0.0, params.gamma_condition_max));

  const double age_sec = std::max(0.0, block.age_sec);
  if (params.age_model == Params::AgeModel::LINEAR_CAPPED) {
    risk.gamma_age = std::min(
        age_sec / std::max(params.age_ref_sec, 1e-6),
        std::max(0.0, params.gamma_age_max));
  } else {
    risk.gamma_age = std::min(
        1.0 - std::exp(-age_sec / std::max(params.age_tau_s, 1e-6)),
        std::max(0.0, params.gamma_age_max));
  }

  risk.gamma_total = std::max(
      0.0,
      params.w_rmse * risk.gamma_rmse +
      params.w_inlier * risk.gamma_inlier +
      params.w_cond * risk.gamma_condition +
      params.w_age * risk.gamma_age);
  return risk;
}

std::vector<LidarHypothesis> LidarAraim::enumerate_hypotheses(
    const LidarAraimSnapshot& snapshot,
    const Params& params) {
  std::vector<LidarHypothesis> hyps;
  if (snapshot.blocks.empty()) {
    return hyps;
  }

  LidarHypothesis source_hyp;
  source_hyp.type = LidarHypothesis::Type::H_SOURCE;
  source_hyp.p_fault = params.p_source;
  source_hyp.block_indices.reserve(snapshot.blocks.size());
  for (int i = 0; i < static_cast<int>(snapshot.blocks.size()); ++i) {
    source_hyp.block_indices.push_back(i);
    update_hypothesis_risk(
        &source_hyp, i, snapshot.blocks[static_cast<std::size_t>(i)], params);
  }
  hyps.push_back(source_hyp);

  std::map<long, LidarHypothesis> target_hyps;
  std::map<int, LidarHypothesis> level_hyps;

  for (int i = 0; i < static_cast<int>(snapshot.blocks.size()); ++i) {
    const auto& block = snapshot.blocks[static_cast<std::size_t>(i)];
    auto& target_hyp = target_hyps[block.target_frame_id];
    target_hyp.type = LidarHypothesis::Type::H_TARGET;
    target_hyp.target_frame_id = block.target_frame_id;
    target_hyp.p_fault = params.p_target;
    target_hyp.block_indices.push_back(i);
    update_hypothesis_risk(&target_hyp, i, block, params);

    auto& level_hyp = level_hyps[block.level_id];
    level_hyp.type = LidarHypothesis::Type::H_LEVEL;
    level_hyp.level_id = block.level_id;
    level_hyp.p_fault = params.p_level;
    level_hyp.block_indices.push_back(i);
    update_hypothesis_risk(&level_hyp, i, block, params);
  }

  for (const auto& kv : target_hyps) {
    hyps.push_back(kv.second);
  }
  for (const auto& kv : level_hyps) {
    hyps.push_back(kv.second);
  }

  return hyps;
}

LidarAraimResult LidarAraim::run(const LidarAraimSnapshot& snapshot,
                                 const FGOPositionInfo& fgo_info) const {
  LidarAraimResult result;
  result.target_window_K = std::max(1, params_.target_window_K);
  if (!snapshot.valid || snapshot.blocks.empty()) {
    return result;
  }

  const LidarAraimSnapshot filtered_snapshot =
      filter_target_window(snapshot, params_);
  result.selected_target_count = unique_target_count(filtered_snapshot);
  if (!filtered_snapshot.valid || filtered_snapshot.blocks.empty()) {
    return result;
  }

  Matrix6d Sigma0 = Matrix6d::Zero();
  bool have_covariance = false;
  if (fgo_info.pose_cov_valid) {
    Sigma0 = fgo_info.pose_cov_6x6;
    have_covariance = true;
  } else if (filtered_snapshot.pose_cov_6x6.diagonal().minCoeff() > 0.0) {
    Sigma0 = filtered_snapshot.pose_cov_6x6;
    have_covariance = true;
  }

  if (!have_covariance) {
    return result;
  }

  Eigen::LDLT<Matrix6d> sigma_ldlt;
  if (!factorize_information(Sigma0, params_.eps_degen, &sigma_ldlt)) {
    return result;
  }

  const Matrix6d Lambda0 = sigma_ldlt.solve(Matrix6d::Identity());
  if (!Lambda0.allFinite()) {
    return result;
  }

  result.valid = true;
  result.Sigma0 = Sigma0;
  result.sigma_ff_E = std::sqrt(std::max(0.0, Sigma0(3, 3)));
  result.sigma_ff_N = std::sqrt(std::max(0.0, Sigma0(4, 4)));
  result.sigma_ff_U = std::sqrt(std::max(0.0, Sigma0(5, 5)));

  double K_ff_eff = params_.K_ff;
  double K_fa_eff = params_.K_fa;

  auto hyps = enumerate_hypotheses(filtered_snapshot, params_);
  result.hypotheses = hyps;
  result.n_hypotheses = static_cast<int>(hyps.size());

  if (params_.dynamic_budget && !hyps.empty()) {
    const double P_HMI_0 = params_.P_HMI_req / 2.0;
    K_ff_eff = Q_inv(P_HMI_0 / 2.0);

    const double P_FA_per = params_.P_FA_req / static_cast<double>(hyps.size());
    K_fa_eff = Q_inv(P_FA_per / 2.0);
  }

  result.K_ff_used = K_ff_eff;
  result.K_fa_used = K_fa_eff;

  result.pl_ff_E = K_ff_eff * result.sigma_ff_E;
  result.pl_ff_N = K_ff_eff * result.sigma_ff_N;
  result.pl_ff_U = K_ff_eff * result.sigma_ff_U;

  result.PL_E = result.pl_ff_E;
  result.PL_N = result.pl_ff_N;
  result.PL_U = result.pl_ff_U;
  result.HPL = std::max(result.PL_E, result.PL_N);
  result.VPL = result.PL_U;

  result.subsets.resize(hyps.size());

  for (int hi = 0; hi < static_cast<int>(hyps.size()); ++hi) {
    const auto& hyp = hyps[static_cast<std::size_t>(hi)];
    auto& ss = result.subsets[static_cast<std::size_t>(hi)];
    ss.hyp_index = hi;

    Matrix6d Lambda_f = Lambda0;
    Vector6d eta_f = Vector6d::Zero();
    for (const int bi : hyp.block_indices) {
      const auto& block =
          filtered_snapshot.blocks[static_cast<std::size_t>(bi)];
      Lambda_f -= block.Lambda_B;
      eta_f -= block.eta_B;
    }

    Eigen::LDLT<Matrix6d> ldlt_f;
    if (!factorize_information(Lambda_f, params_.eps_degen, &ldlt_f)) {
      ss.valid = false;
      ss.PL_E = ss.PL_N = ss.PL_U = 1e9;
      ss.HPL = ss.VPL = 1e9;
      continue;
    }

    const Matrix6d Sigma_f = covariance_from_ldlt(ldlt_f);
    if (!Sigma_f.allFinite()) {
      ss.valid = false;
      ss.PL_E = ss.PL_N = ss.PL_U = 1e9;
      ss.HPL = ss.VPL = 1e9;
      continue;
    }
    const Vector6d delta_f = ldlt_f.solve(eta_f);
    const auto d = ldlt_f.vectorD();
    const double d_min = d.minCoeff();
    const double d_max = d.maxCoeff();

    ss.valid = true;
    ss.d_E = delta_f(3);
    ss.d_N = delta_f(4);
    ss.d_U = delta_f(5);
    ss.lambda_min_subset = d_min;
    ss.condition_number_subset =
        (d_min > 0.0 && std::isfinite(d_min) && std::isfinite(d_max))
            ? d_max / d_min
            : 1e9;

    const SigmaSsAxis ss_E =
        compute_sigma_ss_axis(Sigma0, Sigma_f, 3, params_.sigma_ss_min_m);
    const SigmaSsAxis ss_N =
        compute_sigma_ss_axis(Sigma0, Sigma_f, 4, params_.sigma_ss_min_m);
    const SigmaSsAxis ss_U =
        compute_sigma_ss_axis(Sigma0, Sigma_f, 5, params_.sigma_ss_min_m);
    ss.sigma_ss_E = ss_E.used_m;
    ss.sigma_ss_N = ss_N.used_m;
    ss.sigma_ss_U = ss_U.used_m;
    ss.sigma_ss_raw_E_m2 = ss_E.raw_m2;
    ss.sigma_ss_raw_N_m2 = ss_N.raw_m2;
    ss.sigma_ss_raw_U_m2 = ss_U.raw_m2;
    ss.sigma_ss_fallback_E = ss_E.fallback;
    ss.sigma_ss_fallback_N = ss_N.fallback;
    ss.sigma_ss_fallback_U = ss_U.fallback;

    ss.sigma_k_E = std::sqrt(std::max(0.0, Sigma_f(3, 3)));
    ss.sigma_k_N = std::sqrt(std::max(0.0, Sigma_f(4, 4)));
    ss.sigma_k_U = std::sqrt(std::max(0.0, Sigma_f(5, 5)));

    double K_md_i = params_.K_md;
    if (params_.dynamic_budget && !hyps.empty() && hyp.p_fault > 0.0) {
      const double P_HMI_i = params_.P_HMI_req /
          (2.0 * static_cast<double>(hyps.size()));
      const double ratio = P_HMI_i / hyp.p_fault;
      K_md_i = (ratio < 0.5) ? Q_inv(ratio) : 0.0;
    }

    ss.K_fa = K_fa_eff;
    ss.K_md = K_md_i;
    ss.T_E = ss.K_fa * ss.sigma_ss_E;
    ss.T_N = ss.K_fa * ss.sigma_ss_N;
    ss.T_U = ss.K_fa * ss.sigma_ss_U;
    ss.fault_detected =
        std::abs(ss.d_E) > ss.T_E ||
        std::abs(ss.d_N) > ss.T_N ||
        std::abs(ss.d_U) > ss.T_U;

    ss.bias_H = params_.alpha_H * hyp.gamma_mode;
    ss.bias_V = params_.alpha_V * hyp.gamma_mode;

    ss.PL_E = std::abs(ss.d_E) + ss.T_E + ss.K_md * ss.sigma_k_E + ss.bias_H;
    ss.PL_N = std::abs(ss.d_N) + ss.T_N + ss.K_md * ss.sigma_k_N + ss.bias_H;
    ss.PL_U = std::abs(ss.d_U) + ss.T_U + ss.K_md * ss.sigma_k_U + ss.bias_V;
    ss.HPL = std::max(ss.PL_E, ss.PL_N);
    ss.VPL = ss.PL_U;

    if (ss.PL_E > result.PL_E) result.PL_E = ss.PL_E;
    if (ss.PL_N > result.PL_N) result.PL_N = ss.PL_N;
    if (ss.PL_U > result.PL_U) result.PL_U = ss.PL_U;
    if (ss.HPL > result.HPL) {
      result.worst_hyp = hi;
      result.worst_mode = hypothesis_label(hyp);
    }
    if (ss.fault_detected) {
      ++result.n_detected;
    }
  }

  result.HPL = std::max(result.PL_E, result.PL_N);
  result.VPL = result.PL_U;
  if (result.worst_hyp < 0 && !hyps.empty()) {
    result.worst_mode = "FAULT_FREE";
  }

  return result;
}

}  // namespace iap
