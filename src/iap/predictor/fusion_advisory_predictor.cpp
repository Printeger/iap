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
  if ((lambda-lambda.transpose()).cwiseAbs().maxCoeff() > psd_tol || max_eig <= 0.0 || min_eig < -psd_tol) {
    return false;
  }
  if (symmetric_lambda) {
    *symmetric_lambda = sym;
  }
  return true;
}

bool information_to_pl(const Eigen::Matrix3d& lambda,
                       const FusionAdvisoryPredictorParams& params,
                       double* hpl,
                       double* vpl) {
  Eigen::Matrix3d symmetric;
  if (hpl == nullptr || vpl == nullptr ||
      !valid_position_information(lambda, &symmetric)) {
    return false;
  }
  const double eps =
      std::isfinite(params.fim_epsilon) && params.fim_epsilon > 0.0
          ? params.fim_epsilon : 1.0e-6;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(symmetric);
  const double minimum = eig.eigenvalues().minCoeff();
  if (minimum <= 0 || eps/(minimum+eps) > params.max_regularization_fraction) return false;
  Eigen::LDLT<Eigen::Matrix3d> ldlt(
      symmetric + eps * Eigen::Matrix3d::Identity());
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) {
    return false;
  }
  const Eigen::Matrix3d covariance =
      ldlt.solve(Eigen::Matrix3d::Identity());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> horizontal(
      covariance.block<2, 2>(0, 0), Eigen::EigenvaluesOnly);
  if (!covariance.allFinite() || horizontal.info() != Eigen::Success ||
      covariance(2, 2) < 0.0) {
    return false;
  }
  const double k_h = std::isfinite(params.K_H_adv) && params.K_H_adv > 0.0
      ? params.K_H_adv : 5.0;
  const double k_v = std::isfinite(params.K_V_adv) && params.K_V_adv > 0.0
      ? params.K_V_adv : 5.0;
  *hpl = k_h * std::sqrt(std::max(
      0.0, horizontal.eigenvalues().maxCoeff())) +
      params.b_H_pred + params.s_H_pred;
  *vpl = k_v * std::sqrt(std::max(0.0, covariance(2, 2))) +
      params.b_V_pred + params.s_V_pred;
  return std::isfinite(*hpl) && std::isfinite(*vpl);
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

  if (gnss.fim_valid &&
      gnss.information_state == PredictorInformationState::Position3MapEnu &&
      valid_position_information(gnss.lambda_gnss, &out.lambda_gnss)) {
    out.gnss_used = true;
  } else if (gnss.fim_valid) {
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
  if (out.gnss_used) {
    information_to_pl(out.lambda_gnss, params_, &out.gnss_information_hpl, &out.gnss_information_vpl);
  }
  if (out.prior_valid) {
    information_to_pl(out.lambda_prior, params_, &out.prior_only_hpl,
                      &out.prior_only_vpl);
  }
  if (out.lidar_used) {
    information_to_pl(out.lambda_lidar, params_, &out.lidar_only_hpl,
                      &out.lidar_only_vpl);
  }

  FimDiagnostic diag;
  diag.lambda = out.lambda_pred;
  fill_fim_diagnostics(diag);
  out.lambda_pred_trace = diag.trace;
  out.lambda_pred_min_eig = diag.min_eig;
  out.lambda_pred_max_eig = diag.max_eig;
  out.lambda_pred_condition = diag.condition;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> joint(out.lambda_pred);
  if (joint.info() != Eigen::Success || !out.lambda_pred.allFinite() ||
      diag.min_eig < -std::max(1e-9,1e-10*std::abs(diag.max_eig))) {
    out.numerical_status = AdvisoryNumericalStatus::INVALID_INFORMATION;
    out.fallback_reason = "invalid_joint_information";
    return out;
  }
  out.weak_direction = joint.eigenvectors().col(0);
  const double eps = params_.fim_epsilon;
  if (!std::isfinite(eps) || eps <= 0 ||
      !std::isfinite(params_.max_regularization_fraction) ||
      params_.max_regularization_fraction <= 0 || params_.max_regularization_fraction > 0.01) {
    out.numerical_status = AdvisoryNumericalStatus::INVALID_INFORMATION;
    out.fallback_reason = "invalid_numerical_quality_parameter";
    return out;
  }
  out.regularization_fraction = eps / (std::max(0.,diag.min_eig)+eps);
  const double rank_tolerance = std::max(0.,diag.max_eig) * 32 * std::numeric_limits<double>::epsilon();
  out.numerical_status = diag.min_eig <= rank_tolerance
      ? AdvisoryNumericalStatus::RANK_DEFICIENT
      : out.regularization_fraction > params_.max_regularization_fraction
        ? AdvisoryNumericalStatus::REGULARIZATION_DOMINATED
        : AdvisoryNumericalStatus::OBSERVATION_SUPPORTED;
  out.degeneracy_regularized = out.numerical_status != AdvisoryNumericalStatus::OBSERVATION_SUPPORTED;
  out.epsilon_applied = true;
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
  out.regularized_diagnostic_hpl = k_h * out.sigma_h + params_.b_H_pred + params_.s_H_pred;
  out.regularized_diagnostic_vpl = k_v * out.sigma_v + params_.b_V_pred + params_.s_V_pred;
  if (out.degeneracy_regularized) {
    out.fallback_reason = out.numerical_status == AdvisoryNumericalStatus::RANK_DEFICIENT
        ? "rank_deficient_joint_information" : "regularization_dominated_joint_information";
    return out;
  }
  out.hpl = out.regularized_diagnostic_hpl;
  out.vpl = out.regularized_diagnostic_vpl;
  out.pre_conservative_hpl = out.hpl;
  out.pre_conservative_vpl = out.vpl;

  if (params_.conservative_max_with_gnss && gnss.valid) {
    out.conservative_max_applied = true;
    out.hpl = std::max(out.hpl, gnss.hpl);
    out.vpl = std::max(out.vpl, gnss.vpl);
    out.floor_increment_h = out.hpl - out.pre_conservative_hpl;
    out.floor_increment_v = out.vpl - out.pre_conservative_vpl;
    if (out.floor_increment_h > 0.0) out.floor_source_h = "gnss";
    if (out.floor_increment_v > 0.0) out.floor_source_v = "gnss";
  }
  out.pl_scalar = std::max(out.hpl, out.vpl);

  out.available = std::isfinite(out.hpl) && std::isfinite(out.vpl);
  out.valid = out.available;
  out.fallback = !out.valid;
  out.fallback_reason =
      out.valid ? join_reasons(reasons) : "invalid_advisory_pl";
  return out;
}

}  // namespace iap
