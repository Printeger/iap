// IAP-RQ-132: TrunkFactor — trunk cylinder observation factor for FGO
#include <iap/trunk/trunk_factor.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <stdexcept>

namespace iap {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

TrunkFactor::TrunkFactor(gtsam::Key key,
                         const Eigen::Vector3d& landmark_world,
                         const Eigen::Vector3d& meas_sensor,
                         const gtsam::SharedNoiseModel& noise_model)
    : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, key),
      landmark_world_(landmark_world),
      meas_sensor_(meas_sensor) {}

// ---------------------------------------------------------------------------
// Static noise factory
// ---------------------------------------------------------------------------

gtsam::SharedNoiseModel TrunkFactor::make_noise(double confidence,
                                                const NoiseParams& p) {
  const double c = std::max(confidence, p.min_conf);
  const double s = 1.0 / std::sqrt(c);
  gtsam::Vector3 sigmas;
  sigmas << p.sigma_xy * s, p.sigma_xy * s, p.sigma_z * s;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

// ---------------------------------------------------------------------------
// Error function + Jacobian
// ---------------------------------------------------------------------------
// Observation model:
//   h(T) = R^T * (c_k - p)     [predicted meas in sensor frame]
// Residual:
//   r = z_k - h(T) = z_k - R^T*(c_k - p)
//
// Jacobian w.r.t. Pose3 (tangent = [ω, ρ]):
//   ∂r/∂ω = +[R^T*(c_k - p)]×   (skew of predicted meas)
//   ∂r/∂ρ = -R^T
//
// packed as H = [∂r/∂ω | ∂r/∂ρ]  (GTSAM convention: [-1 * pure-Jacobian])
// ---------------------------------------------------------------------------

gtsam::Vector TrunkFactor::evaluateError(
    const gtsam::Pose3& pose,
    gtsam::OptionalMatrixType H) const {

  const gtsam::Rot3 R = pose.rotation();
  const gtsam::Point3 t(pose.translation());

  // Predicted measurement in sensor frame: h = R^T * (c - p)
  const Eigen::Vector3d c_minus_p = landmark_world_ - t;
  const Eigen::Vector3d h = R.matrix().transpose() * c_minus_p;

  // Residual
  const gtsam::Vector3 residual = meas_sensor_ - h;

  if (H) {
    // ∂r/∂ω = [h]×   (skew-symmetric of predicted meas)
    Eigen::Matrix3d skew_h;
    skew_h <<     0, -h(2),  h(1),
               h(2),     0, -h(0),
              -h(1),  h(0),     0;

    // ∂r/∂ρ = R^T (i.e., -(-R^T) in GTSAM's convention)
    const Eigen::Matrix3d Rt = R.matrix().transpose();

    // GTSAM Pose3 tangent order is [ω (3), ρ (3)] = rotation then translation
    Eigen::Matrix<double, 3, 6> J;
    J.leftCols<3>()  = skew_h;   // ∂r/∂ω
    J.rightCols<3>() = Rt;       // ∂r/∂ρ   (note: sign = -(- R^T) = +R^T)

    *H = J;
  }

  return residual;
}

}  // namespace iap
