#pragma once
// Phase D: common PL field query result for direct and grid prediction.

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

  double gnss_hpl = 1e9;
  double gnss_vpl = 1e9;
  double fused_hpl = 1e9;
  double fused_vpl = 1e9;
  bool lidar_valid = false;
  double lidar_alpha = 0.0;
  double lidar_tdop = 20.0;
  double lidar_condition = 1.0e6;
  int lidar_n_primitives = 0;
  double lidar_bias_h = 0.0;
  double lidar_bias_v = 0.0;
  std::string lidar_fallback_reason = "lidar_disabled";

  Eigen::Vector3d grad_hpl = Eigen::Vector3d::Zero();
  Eigen::Vector3d grad_vpl = Eigen::Vector3d::Zero();
  Eigen::Vector3d grad_pl_scalar = Eigen::Vector3d::Zero();

  int grid_generation = -1;
  double grid_age_s = std::numeric_limits<double>::quiet_NaN();
  double grid_build_time_ms = std::numeric_limits<double>::quiet_NaN();
};

FuturePLQueryResult make_future_pl_query_result(
    const PredictedAraimResult& pred,
    const std::string& query_source);

}  // namespace iap
