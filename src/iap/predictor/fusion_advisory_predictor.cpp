#include <iap/predictor/fusion_advisory_predictor.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace iap {
namespace {

std::string join_reasons(const std::vector<std::string>& reasons) {
  std::ostringstream oss;
  for (const auto& reason : reasons) {
    if (reason.empty()) {
      continue;
    }
    if (oss.tellp() > 0) {
      oss << ";";
    }
    oss << reason;
  }
  return oss.str();
}

bool valid_position_information(const Eigen::Matrix3d& lambda,
                                Eigen::Matrix3d* symmetric_lambda) {
  if (!lambda.allFinite()) {
    return false;
  }
  const Eigen::Matrix3d sym = 0.5 * (lambda + lambda.transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(sym);
  if (eig.info() != Eigen::Success || !eig.eigenvalues().allFinite()) {
    return false;
  }
  const double max_eig = eig.eigenvalues().maxCoeff();
  const double min_eig = eig.eigenvalues().minCoeff();
  const double psd_tol = std::max(1.0e-9, 1.0e-10 * std::abs(max_eig));
  if (max_eig <= 0.0 || min_eig < -psd_tol) {
    return false;
  }
  if (symmetric_lambda) {
    *symmetric_lambda = sym;
  }
  return true;
}

}  // namespace

FusionAdvisoryPredictor::FusionAdvisoryPredictor()
    : FusionAdvisoryPredictor(FusionAdvisoryPredictorParams{}) {}

FusionAdvisoryPredictor::FusionAdvisoryPredictor(
    const FusionAdvisoryPredictorParams& params)
    : params_(params) {}

void FusionAdvisoryPredictor::set_params(
    const FusionAdvisoryPredictorParams& params) {
  params_ = params;
}

FusionAdvisoryResult FusionAdvisoryPredictor::query(
    const IntegritySnapshot& snapshot,
    const GnssAdvisoryResult& gnss,
    const LidarAdvisoryResult& lidar) const {
  FusionAdvisoryResult out;
  std::vector<std::string> reasons;

  if (snapshot.has_lambda_base &&
      valid_position_information(snapshot.lambda_base_pos,
                                 &out.lambda_prior)) {
    out.prior_valid = true;
  } else if (snapshot.has_lambda_base) {
    reasons.push_back("invalid_prior_position_information");
  } else {
    reasons.push_back("missing_prior");
  }

  if (gnss.valid && gnss.fim_valid &&
      gnss.information_state == PredictorInformationState::Position3MapEnu &&
      valid_position_information(gnss.lambda_gnss, &out.lambda_gnss)) {
    out.gnss_used = true;
  } else if (gnss.valid && gnss.fim_valid) {
    reasons.push_back("gnss:invalid_gnss_position_information");
  } else if (!gnss.fallback_reason.empty()) {
    reasons.push_back("gnss:" + gnss.fallback_reason);
  } else {
    reasons.push_back("gnss_unavailable");
  }

  if (lidar.valid &&
      lidar.information_state == PredictorInformationState::Position3MapEnu &&
      valid_position_information(lidar.lambda_lidar, &out.lambda_lidar)) {
    out.lidar_used = true;
  } else if (lidar.valid) {
    reasons.push_back("lidar:invalid_lidar_position_information");
  } else if (!lidar.fallback_reason.empty()) {
    reasons.push_back("lidar:" + lidar.fallback_reason);
  } else {
    reasons.push_back("lidar_unavailable");
  }

  if (!out.gnss_used && !out.lidar_used) {
    out.available = false;
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = join_reasons(reasons);
    if (out.fallback_reason.empty()) {
      out.fallback_reason = "missing_advisory_information";
    }
    return out;
  }

  out.lambda_pred = out.lambda_prior + out.lambda_gnss + out.lambda_lidar;
  out.lambda_pred = 0.5 * (out.lambda_pred + out.lambda_pred.transpose());
  out.lambda_prior_trace = out.lambda_prior.trace();
  out.lambda_gnss_trace = out.lambda_gnss.trace();
  out.lambda_lidar_trace = out.lambda_lidar.trace();

  FimDiagnostic diag;
  diag.lambda = out.lambda_pred;
  fill_fim_diagnostics(diag);
  out.degeneracy_regularized =
      !std::isfinite(diag.min_eig) || diag.min_eig <= 0.0;

  const double eps =
      std::isfinite(params_.fim_epsilon) && params_.fim_epsilon > 0.0
          ? params_.fim_epsilon
          : 1.0e-6;
  out.epsilon_applied = eps > 0.0;
  const Eigen::Matrix3d regularized_lambda =
      out.lambda_pred + eps * Eigen::Matrix3d::Identity();
  Eigen::LDLT<Eigen::Matrix3d> ldlt(regularized_lambda);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    out.available = false;
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "singular_advisory_fim";
    return out;
  }

  out.sigma_pos = ldlt.solve(Eigen::Matrix3d::Identity());
  if (!out.sigma_pos.allFinite()) {
    out.available = false;
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "invalid_advisory_covariance";
    return out;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig_h(
      out.sigma_pos.block<2, 2>(0, 0), Eigen::EigenvaluesOnly);
  if (eig_h.info() != Eigen::Success || out.sigma_pos(2, 2) < 0.0) {
    out.available = false;
    out.valid = false;
    out.fallback = true;
    out.fallback_reason = "invalid_advisory_covariance";
    return out;
  }

  const double k_h =
      std::isfinite(params_.K_H_adv) && params_.K_H_adv > 0.0
          ? params_.K_H_adv
          : 5.0;
  const double k_v =
      std::isfinite(params_.K_V_adv) && params_.K_V_adv > 0.0
          ? params_.K_V_adv
          : 5.0;
  out.sigma_h = std::sqrt(std::max(0.0, eig_h.eigenvalues().maxCoeff()));
  out.sigma_v = std::sqrt(std::max(0.0, out.sigma_pos(2, 2)));
  out.hpl = k_h * out.sigma_h + params_.b_H_pred + params_.s_H_pred;
  out.vpl = k_v * out.sigma_v + params_.b_V_pred + params_.s_V_pred;

  if (params_.conservative_max_with_gnss && gnss.valid) {
    out.conservative_max_applied = true;
    out.hpl = std::max(out.hpl, gnss.hpl);
    out.vpl = std::max(out.vpl, gnss.vpl);
  }
  out.pl_scalar = std::max(out.hpl, out.vpl);

  FimDiagnostic regularized_diag;
  regularized_diag.lambda = regularized_lambda;
  regularized_diag.valid = true;
  regularized_diag.regularized = out.degeneracy_regularized;
  fill_fim_diagnostics(regularized_diag);
  out.lambda_pred_trace = regularized_diag.trace;
  out.lambda_pred_min_eig = regularized_diag.min_eig;
  out.lambda_pred_max_eig = regularized_diag.max_eig;
  out.lambda_pred_condition = regularized_diag.condition;

  out.available = std::isfinite(out.hpl) && std::isfinite(out.vpl);
  out.valid = out.available;
  out.fallback = !out.valid;
  out.fallback_reason =
      out.valid ? join_reasons(reasons) : "invalid_advisory_pl";
  return out;
}

}  // namespace iap
