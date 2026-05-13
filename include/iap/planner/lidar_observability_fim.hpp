#pragma once
// Phase F: LiDAR advisory observability proxy for planner-side PL queries.
//
// This is a future/advisory LOI-style proxy. It is not the certified current
// LiDAR ARAIM monitor and must not be reported as certified LiDAR PL.

#include <Eigen/Core>

#include <limits>
#include <string>
#include <vector>

#include <iap/planner/integrity_snapshot.hpp>

namespace iap {

struct LidarObservabilityResult {
  bool valid = false;
  Eigen::Matrix3d delta_lambda = Eigen::Matrix3d::Zero();  ///< advisory LOI
  double tdop_proxy = 20.0;
  double lidar_alpha = 0.0;
  double condition = 1.0e6;
  int n_primitives = 0;
  double bias_h = 0.0;
  double bias_v = 0.0;
  std::string fallback_reason = "not_evaluated";
};

class LidarObservabilityFim {
 public:
  struct Params {
    double search_radius_m = 8.0;
    int min_points = 12;
    int good_points = 80;
    double sigma_lidar_m = 0.5;
    double alpha_min = 0.02;
    double alpha_max = 1.0;
    double condition_ref = 30.0;
    double condition_max = 1.0e6;
    double tdop_ref = 2.0;
    double tdop_max = 20.0;
    double bias_h_m = 0.0;
    double bias_v_m = 0.0;
  };

  LidarObservabilityFim();
  explicit LidarObservabilityFim(const Params& params);

  LidarObservabilityResult evaluate(
      const Eigen::Vector3d& p_w,
      const std::vector<Eigen::Vector3d>* map_points,
      const CurrentIntegrityState& current) const;

  const Params& params() const { return params_; }

 private:
  Params params_;
};

}  // namespace iap
