#pragma once
// LiDAR ARAIM for current-frame VGICP block hypotheses.

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iap/integrity/fgo_information_matrix.hpp>
#include <iap/odometry/estimation_frame.hpp>
#include <string>
#include <vector>

namespace iap {

struct LidarAraimBlock {
  enum class Backend {
    CPU = 0,
    GPU = 1,
  };

  long source_frame_id = -1;
  long target_frame_id = -1;
  bool target_is_fixed = false;
  int level_id = -1;
  double voxel_resolution = 0.0;
  Backend backend = Backend::CPU;

  int num_inliers = 0;
  double inlier_fraction = 0.0;
  double rmse_proxy = 0.0;
  double cond_proxy = 1.0;
  double gamma_lidar = 1.0;
  double age_sec = 0.0;

  Eigen::Matrix<double, 6, 6> Lambda_B =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> eta_B =
      Eigen::Matrix<double, 6, 1>::Zero();
};

struct LidarAraimSnapshot {
  double stamp = 0.0;
  long frame_id = -1;
  bool valid = false;

  Eigen::Isometry3d T_world_imu = Eigen::Isometry3d::Identity();
  Eigen::Matrix<double, 6, 6> pose_cov_6x6 =
      Eigen::Matrix<double, 6, 6>::Zero();
  glim::EstimationFrame::IcpQuality current_icp_quality;

  std::vector<LidarAraimBlock> blocks;
};

struct LidarHypothesis {
  enum class Type {
    H_SOURCE = 0,
    H_TARGET = 1,
    H_LEVEL = 2,
  };

  Type type = Type::H_SOURCE;
  long target_frame_id = -1;
  int level_id = -1;
  std::vector<int> block_indices;
  double gamma_mode = 0.0;
  double p_fault = 1e-4;
};

struct LidarSubsetSolution {
  int hyp_index = -1;
  double d_E = 0.0;
  double d_N = 0.0;
  double d_U = 0.0;

  double sigma_ss_E = 0.0;
  double sigma_ss_N = 0.0;
  double sigma_ss_U = 0.0;

  double sigma_k_E = 0.0;
  double sigma_k_N = 0.0;
  double sigma_k_U = 0.0;

  double T_E = 0.0;
  double T_N = 0.0;
  double T_U = 0.0;

  double K_fa = 0.0;
  double K_md = 0.0;
  double bias_H = 0.0;
  double bias_V = 0.0;

  double PL_E = 0.0;
  double PL_N = 0.0;
  double PL_U = 0.0;
  double HPL = 0.0;
  double VPL = 0.0;

  bool valid = false;
  bool fault_detected = false;
};

struct LidarAraimResult {
  bool valid = false;
  double sigma_ff_E = 0.0;
  double sigma_ff_N = 0.0;
  double sigma_ff_U = 0.0;
  double pl_ff_E = 0.0;
  double pl_ff_N = 0.0;
  double pl_ff_U = 0.0;

  double PL_E = 1e9;
  double PL_N = 1e9;
  double PL_U = 1e9;
  double HPL = 1e9;
  double VPL = 1e9;

  double K_ff_used = 0.0;
  double K_fa_used = 0.0;

  int n_hypotheses = 0;
  int n_detected = 0;
  int worst_hyp = -1;
  std::string worst_mode = "NONE";

  Eigen::Matrix<double, 6, 6> Sigma0 =
      Eigen::Matrix<double, 6, 6>::Zero();
  std::vector<LidarHypothesis> hypotheses;
  std::vector<LidarSubsetSolution> subsets;
};

class LidarAraim {
 public:
  struct Params {
    double P_HMI_req = 1e-6;
    double P_FA_req = 1e-4;
    bool dynamic_budget = true;

    double K_fa = 4.5;
    double K_md = 5.5;
    double K_ff = 5.42;
    double eps_degen = 1e-10;

    double p_source = 1e-3;
    double p_target = 1e-4;
    double p_level = 1e-4;

    double rmse_ref = 0.2;
    double age_ref_sec = 1.0;
    double w_rmse = 1.0;
    double w_inlier = 1.0;
    double w_cond = 1.0;
    double w_age = 1.0;
    double alpha_H = 1.0;
    double alpha_V = 1.0;
  };

  LidarAraim();
  explicit LidarAraim(const Params& params);

  LidarAraimResult run(const LidarAraimSnapshot& snapshot,
                       const FGOPositionInfo& fgo_info) const;

  const Params& params() const { return params_; }

  static const char* to_string(LidarHypothesis::Type type);

 private:
  static double Q_inv(double p);
  static bool factorize_information(
      const Eigen::Matrix<double, 6, 6>& Lambda,
      double eps_degen,
      Eigen::LDLT<Eigen::Matrix<double, 6, 6>>* ldlt);
  static std::vector<LidarHypothesis> enumerate_hypotheses(
      const LidarAraimSnapshot& snapshot,
      const Params& params);
  static double block_risk_score(const LidarAraimBlock& block,
                                 const Params& params);

  Params params_;
};

}  // namespace iap
