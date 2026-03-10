// IAP-RQ-132: TrunkFactor — trunk cylinder observation factor for FGO
// Factor2<Pose3, Point2>: 2D residual in sensor XY frame.
#include <iap/trunk/trunk_factor.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <stdexcept>

namespace iap {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

TrunkFactor::TrunkFactor(gtsam::Key pose_key,
                         gtsam::Key landmark_key,
                         const Eigen::Vector2d& meas_sensor,
                         const gtsam::SharedNoiseModel& noise_model)
    : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point2>(noise_model, pose_key, landmark_key),
      meas_sensor_(meas_sensor) {}

// ---------------------------------------------------------------------------
// Static noise factory
// ---------------------------------------------------------------------------

gtsam::SharedNoiseModel TrunkFactor::make_noise(double confidence,
                                                const NoiseParams& p) {
  const double c = std::max(confidence, p.min_conf);
  const double s = 1.0 / std::sqrt(c);
  gtsam::Vector2 sigmas;
  sigmas << p.sigma_xy * s, p.sigma_xy * s;
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

// ---------------------------------------------------------------------------
// Error function + Jacobians
// ---------------------------------------------------------------------------
// Observation model (2D XY):
//   h(x_t, c_k) = R_{2x2}^T * (c_k - p^{xy})   [predicted meas in sensor XY]
// Residual:
//   r = z_k - h(x_t, c_k) = z_k - R_{2x2}^T * (c_k - p^{xy})
//
// R_{2x2} = top-left 2×2 block of Pose3::rotation().matrix()
//
// Jacobian w.r.t. Pose3 (tangent = [ω₁ ω₂ ω₃ ρ₁ ρ₂ ρ₃]):
//   Let h_2d = R_{2x2}^T * (c_k - p^{xy})  — the predicted 2D sensor measurement.
//   Let h_3d = R^T * (c_k_3d - p)  where c_k_3d = (c_k.x, c_k.y, p_z) to
//     stay in same Z plane.  Then h_2d = h_3d.head<2>().
//
//   ∂r/∂ω (2×3):  rows 0-1 of skew(h_3d)  — since r = z - h, and
//                  ∂h_3d/∂ω = -[h_3d]× , giving ∂r/∂ω = +[h_3d]× rows 0-1
//   ∂r/∂ρ (2×3):  rows 0-1 of R^T         — only first 2 rows
//     (∂r/∂ρ = + R^T  because r = z - R^T*(c-p), ∂r/∂p = +R^T, ∂r/∂ρ = +R^T)
//
// Jacobian w.r.t. Point2 landmark:
//   ∂r/∂c_k (2×2) = -R_{2x2}^T
// ---------------------------------------------------------------------------

gtsam::Vector TrunkFactor::evaluateError(
    const gtsam::Pose3& pose,
    const gtsam::Point2& landmark,
    gtsam::OptionalMatrixType H1,
    gtsam::OptionalMatrixType H2) const {

  const Eigen::Matrix3d R = pose.rotation().matrix();
  const Eigen::Vector3d t = pose.translation();

  // Build 3D vectors for Jacobian derivation (project everything into same Z)
  const Eigen::Vector3d c3(landmark.x(), landmark.y(), t.z());
  const Eigen::Vector3d c_minus_p = c3 - t;  // only XY non-zero

  // Predicted 3D and extract XY
  const Eigen::Vector3d h3 = R.transpose() * c_minus_p;
  const Eigen::Vector2d h = h3.head<2>();

  // 2D residual in sensor XY frame
  const gtsam::Vector2 residual = meas_sensor_ - h;

  if (H1) {
    // H1 = ∂r/∂pose (2×6)  tangent = [ω, ρ]
    // ∂r/∂ω = +[h3]×  (first 2 rows of 3×3 skew)
    // [h3]× = | 0      -h3(2)  h3(1)|
    //         | h3(2)   0     -h3(0)|
    //         |-h3(1)   h3(0)  0    |
    Eigen::Matrix<double, 2, 3> dr_dw;
    dr_dw(0, 0) =  0.0;      dr_dw(0, 1) = -h3(2);   dr_dw(0, 2) =  h3(1);
    dr_dw(1, 0) =  h3(2);    dr_dw(1, 1) =  0.0;     dr_dw(1, 2) = -h3(0);

    // ∂r/∂ρ = +R^T (first 2 rows)
    Eigen::Matrix<double, 2, 3> dr_dp = R.transpose().topRows<2>();

    Eigen::Matrix<double, 2, 6> J;
    J.leftCols<3>()  = dr_dw;
    J.rightCols<3>() = dr_dp;
    *H1 = J;
  }

  if (H2) {
    // H2 = ∂r/∂landmark (2×2) = -R_{2x2}^T
    const Eigen::Matrix2d R2 = R.topLeftCorner<2, 2>();
    *H2 = -R2.transpose();
  }

  return residual;
}

}  // namespace iap
