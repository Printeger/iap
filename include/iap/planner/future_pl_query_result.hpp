#pragma once
// Phase D: common advisory PL field query result for direct and grid prediction.

#include <Eigen/Core>

#include <limits>
#include <string>

#include <iap/planner/predicted_araim.hpp>

namespace iap {

struct FuturePLQueryResult {
  bool valid = false;
  bool fallback = true;
  std::string fallback_reason = "not_evaluated";
  std::string query_source = "direct";

  // Advisory predicted outputs for planner use. These are not certified
  // monitor PL values.
  double hpl = 1e9;
  double vpl = 1e9;
  double pl_scalar = 1e9;
  double pl_e = 1e9;
  double pl_n = 1e9;
  double pl_u = 1e9;
  double pl_ff_h = 1e9;
  double pl_ff_v = 1e9;
  double sigma_h = 1e9;
  double sigma_v = 1e9;
  double pdop = 1e9;
  int n_vis = 0;
  int n_hypotheses = 0;

  double gnss_hpl = 1e9;   ///< gnss_advisory_hpl_proxy [m]
  double gnss_vpl = 1e9;   ///< gnss_advisory_vpl_proxy [m]
  double fused_hpl = 1e9;  ///< advisory_predicted_fused_hpl [m]
  double fused_vpl = 1e9;  ///< advisory_predicted_fused_vpl [m]
  bool lidar_valid = false;
  double lidar_alpha = 0.0;
  double lidar_tdop = 20.0;
  double lidar_condition = 1.0e6;
  int lidar_n_primitives = 0;
  double lidar_bias_h = 0.0;
  double lidar_bias_v = 0.0;
  std::string lidar_fallback_reason = "lidar_disabled";

  double lambda_prior_trace = 0.0;
  double lambda_gnss_trace = 0.0;
  double lambda_lidar_trace = 0.0;
  double lambda_adv_trace = 0.0;
  double lambda_adv_min_eig = 0.0;
  double lambda_adv_condition = 1.0e12;
  double hpl_adv = 1e9;
  double vpl_adv = 1e9;
  bool lidar_fim_valid = false;
  bool gnss_fim_valid = false;
  bool fim_regularized = false;
  std::string advisory_fusion_mode = "legacy";

  Eigen::Vector3d grad_hpl = Eigen::Vector3d::Zero();
  Eigen::Vector3d grad_vpl = Eigen::Vector3d::Zero();
  Eigen::Vector3d grad_pl_scalar = Eigen::Vector3d::Zero();

  int grid_generation = -1;
  double grid_age_s = std::numeric_limits<double>::quiet_NaN();
  double grid_build_time_ms = std::numeric_limits<double>::quiet_NaN();

  // Non-breaking Stage 1 semantic aliases. Public storage field names stay
  // unchanged until downstream planner/log consumers migrate.
  double advisory_predicted_hpl() const { return hpl; }
  double advisory_predicted_vpl() const { return vpl; }
  double advisory_predicted_pl() const { return pl_scalar; }
  double gnss_advisory_hpl_proxy() const { return gnss_hpl; }
  double gnss_advisory_vpl_proxy() const { return gnss_vpl; }
  double advisory_predicted_fused_hpl() const { return fused_hpl; }
  double advisory_predicted_fused_vpl() const { return fused_vpl; }
};

FuturePLQueryResult make_future_pl_query_result(
    const PredictedAraimResult& pred,
    const std::string& query_source);

}  // namespace iap
