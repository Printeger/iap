#pragma once
// Advisory/future FIM diagnostics for planner-side prediction only.

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <limits>
#include <string>

namespace iap {

struct FimDiagnostic {
  Eigen::Matrix3d lambda = Eigen::Matrix3d::Zero();
  bool valid = false;
  bool regularized = false;
  double trace = 0.0;
  double min_eig = 0.0;
  double max_eig = 0.0;
  double condition = 1.0e12;
  std::string fallback_reason = "not_evaluated";
};

struct GnssAdvisoryFimResult : FimDiagnostic {
  Eigen::Matrix4d h_full = Eigen::Matrix4d::Zero();
  int n_visible = 0;
  int n_used = 0;
};

struct LidarAdvisoryFimResult : FimDiagnostic {
  int n_primitives = 0;
  int n_valid_normals = 0;
};

struct FusedAdvisoryFimResult : FimDiagnostic {
  FimDiagnostic prior;
  GnssAdvisoryFimResult gnss;
  LidarAdvisoryFimResult lidar;
  Eigen::Matrix3d sigma_pos = Eigen::Matrix3d::Identity();
  double hpl_adv = 1e9;
  double vpl_adv = 1e9;
  std::string fusion_mode = "legacy";
};

inline void fill_fim_diagnostics(FimDiagnostic& out) {
  out.trace = out.lambda.trace();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(out.lambda);
  if (eig.info() != Eigen::Success) {
    out.valid = false;
    out.fallback_reason = "invalid_fim_eigendecomposition";
    out.min_eig = -std::numeric_limits<double>::infinity();
    out.max_eig = std::numeric_limits<double>::infinity();
    out.condition = 1.0e12;
    return;
  }
  out.min_eig = eig.eigenvalues().minCoeff();
  out.max_eig = eig.eigenvalues().maxCoeff();
  out.condition =
      out.min_eig > 0.0 ? out.max_eig / out.min_eig : 1.0e12;
}

}  // namespace iap
