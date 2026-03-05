// IAP-RQ-020: PseudorangeFactor implementation

#include <iap/gnss/pseudorange_factor.hpp>
#include <gtsam/base/Matrix.h>
#include <cmath>

namespace iap {

PseudorangeFactor::PseudorangeFactor(gtsam::Key                    pose_key,
                                     gtsam::Key                    clk_key,
                                     double                        pr_meas,
                                     const Eigen::Vector3d&        sat_pos,
                                     const gtsam::SharedNoiseModel& noise)
    : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector2>(noise, pose_key, clk_key),
      pr_meas_(pr_meas),
      sat_pos_(sat_pos) {}

gtsam::Vector PseudorangeFactor::evaluateError(const gtsam::Pose3&   pose,
                                                const gtsam::Vector2& clk,
                                                gtsam::OptionalMatrixType H_pose,
                                                gtsam::OptionalMatrixType H_clk) const {
  // Receiver position in world frame
  const Eigen::Vector3d p_r = pose.translation();

  // Satellite–receiver difference and range
  const Eigen::Vector3d dp   = p_r - sat_pos_;
  const double          rho  = dp.norm();
  constexpr double      kEps = 1e-6;  // guard against degenerate geometry

  // Unit line-of-sight vector (receiver → satellite direction: from sat perspective)
  Eigen::Vector3d e;
  if (rho > kEps) { e = dp / rho; } else { e = Eigen::Vector3d::UnitX(); }

  // Predicted pseudorange = geometric range + clock bias
  const double pr_pred = rho + clk(0);

  // Residual: z_pr - pr_pred  (scalar)
  const double residual = pr_meas_ - pr_pred;

  // --- Jacobians ---
  // GTSAM Pose3 tangent order: [rot(0:2), trans(3:5)]
  if (H_pose) {
    // d(residual)/d(pose) — 1×6
    // d(rho)/d(p_r) = eᵀ  →  d(pr_pred)/d(p_r) = eᵀ  →  d(residual)/d(p_r) = -eᵀ
    // Rotation columns: 0 (range doesn't depend on orientation)
    *H_pose = gtsam::Matrix::Zero(1, 6);
    (*H_pose)(0, 3) = -e(0);
    (*H_pose)(0, 4) = -e(1);
    (*H_pose)(0, 5) = -e(2);
  }
  if (H_clk) {
    // d(residual)/d([δt, δṫ]) — 1×2
    // Only clock bias (δt) enters pseudorange: d(pr_pred)/d(δt) = 1
    *H_clk = gtsam::Matrix::Zero(1, 2);
    (*H_clk)(0, 0) = -1.0;
  }

  return (gtsam::Vector(1) << residual).finished();
}

}  // namespace iap
