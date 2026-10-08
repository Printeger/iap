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

using PoseInformation=Eigen::Matrix<double,6,6>;

bool source_pose_information(PredictorInformationState state,
                             const Eigen::Matrix3d& position,
                             const PoseInformation& pose,
                             PoseInformation* output) {
  output->setZero();
  if(state==PredictorInformationState::Position3MapEnu) {
    output->topLeftCorner<3,3>()=position;
    return true;
  }
  if(state!=PredictorInformationState::Pose6Map || !pose.allFinite()) return false;
  const double tolerance=1e-9+1e-10*pose.cwiseAbs().maxCoeff();
  if((pose-pose.transpose()).cwiseAbs().maxCoeff()>tolerance ||
     (pose.topLeftCorner<3,3>()-position).cwiseAbs().maxCoeff()>tolerance) return false;
  Eigen::SelfAdjointEigenSolver<PoseInformation> eigen(pose,Eigen::EigenvaluesOnly);
  if(eigen.info()!=Eigen::Success || eigen.eigenvalues().minCoeff() < -tolerance) return false;
  *output=0.5*(pose+pose.transpose());
  return true;
}

bool marginal_position_information(const PoseInformation& joint,
                                   Eigen::Matrix3d* position) {
  const Eigen::Matrix3d nuisance=joint.bottomRightCorner<3,3>();
  const Eigen::Matrix3d cross=joint.topRightCorner<3,3>();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(nuisance);
  if(eigen.info()!=Eigen::Success) return false;
  const double tolerance=64*std::numeric_limits<double>::epsilon()*
      std::max(0.,eigen.eigenvalues().maxCoeff());
  Eigen::Vector3d inverse=Eigen::Vector3d::Zero();
  for(int i=0;i<3;++i) {
    if(eigen.eigenvalues()[i]>tolerance) inverse[i]=1./eigen.eigenvalues()[i];
    else if((cross*eigen.eigenvectors().col(i)).norm()>
            1e-8*std::max(1.,joint.cwiseAbs().maxCoeff())) return false;
  }
  // An unobserved nuisance has no prior, including no epsilon prior. Its null
  // direction is harmless only when uncoupled from position (PSD range test).
  const Eigen::Matrix3d pseudo=eigen.eigenvectors()*inverse.asDiagonal()*eigen.eigenvectors().transpose();
  *position=joint.topLeftCorner<3,3>()-cross*pseudo*cross.transpose();
  *position=0.5*(*position+position->transpose()).eval();
  return position->allFinite();
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
      symmetric);
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
  PoseInformation gnss_pose=PoseInformation::Zero(),lidar_pose=PoseInformation::Zero();

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
      valid_position_information(gnss.lambda_gnss, &out.lambda_gnss) &&
      source_pose_information(gnss.information_state,out.lambda_gnss,gnss.joint_pose_information,&gnss_pose)) {
    out.gnss_used = true;
  } else if (gnss.fim_valid) {
    reasons.push_back("gnss:invalid_gnss_position_information");
  } else if (!gnss.fallback_reason.empty()) {
    reasons.push_back("gnss:" + gnss.fallback_reason);
  } else {
    reasons.push_back("gnss_unavailable");
  }

  if (lidar.valid &&
      valid_position_information(lidar.lambda_lidar, &out.lambda_lidar) &&
      source_pose_information(lidar.information_state,out.lambda_lidar,lidar.joint_pose_information,&lidar_pose)) {
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
  if(!out.gnss_used) out.lambda_gnss.setZero();
  if(!out.lidar_used) out.lambda_lidar.setZero();

  const bool shared_pose=gnss.information_state==PredictorInformationState::Pose6Map ||
      lidar.information_state==PredictorInformationState::Pose6Map;
  out.cross_source_noise_inflation=shared_pose && out.gnss_used && out.lidar_used ? 2. : 1.;
  // Cauchy-Schwarz gives R <= 2 diag(R_g,R_l) for arbitrary cross-source
  // covariance with the declared marginal noises. This is a covariance bound,
  // not trace normalization or equal source weighting. Unknown map/bias errors
  // and within-source noise miscalibration remain explicitly unqualified.
  out.joint_pose_information=(gnss_pose+lidar_pose)/out.cross_source_noise_inflation;
  out.joint_pose_information.topLeftCorner<3,3>()+=out.lambda_prior;
  if(!marginal_position_information(out.joint_pose_information,&out.lambda_pred)) {
    out.numerical_status=AdvisoryNumericalStatus::INVALID_INFORMATION;
    out.fallback_reason="invalid_shared_pose_information";
    return out;
  }
  out.lambda_pred = 0.5 * (out.lambda_pred + out.lambda_pred.transpose());
  out.lambda_prior_trace = out.lambda_prior.trace();
  out.lambda_gnss_trace = out.lambda_gnss.trace();
  out.lambda_lidar_trace = out.lambda_lidar.trace();
  if (out.gnss_used) {
    Eigen::Matrix3d marginal;
    if(marginal_position_information(gnss_pose,&marginal))
      information_to_pl(marginal, params_, &out.gnss_information_hpl, &out.gnss_information_vpl);
  }
  if (out.prior_valid) {
    information_to_pl(out.lambda_prior, params_, &out.prior_only_hpl,
                      &out.prior_only_vpl);
  }
  if (out.lidar_used) {
    Eigen::Matrix3d marginal;
    if(marginal_position_information(lidar_pose,&marginal))
      information_to_pl(marginal, params_, &out.lidar_only_hpl,&out.lidar_only_vpl);
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
  // Epsilon establishes a diagnostic inverse and an admission ceiling only.
  // An admitted covariance uses the actual observable information unchanged.
  ldlt.compute(out.lambda_pred);
  if(ldlt.info()!=Eigen::Success || !ldlt.isPositive()) {
    out.numerical_status=AdvisoryNumericalStatus::INVALID_INFORMATION;
    out.fallback_reason="invalid_observed_position_solve";
    return out;
  }
  out.sigma_pos=ldlt.solve(Eigen::Matrix3d::Identity());
  eig_h.compute(out.sigma_pos.topLeftCorner<2,2>(),Eigen::EigenvaluesOnly);
  if(!out.sigma_pos.allFinite() || eig_h.info()!=Eigen::Success || out.sigma_pos(2,2)<0.) {
    out.fallback_reason="invalid_observed_position_covariance";
    return out;
  }
  out.sigma_h=std::sqrt(std::max(0.,eig_h.eigenvalues().maxCoeff()));
  out.sigma_v=std::sqrt(std::max(0.,out.sigma_pos(2,2)));
  out.hpl=k_h*out.sigma_h+params_.b_H_pred+params_.s_H_pred;
  out.vpl=k_v*out.sigma_v+params_.b_V_pred+params_.s_V_pred;
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
