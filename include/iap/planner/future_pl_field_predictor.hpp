#pragma once
// Phase D: direct/grid advisory future PL field predictor.

#include <Eigen/Core>

#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <iap/map/local_occupancy.hpp>
#include <iap/planner/integrity_snapshot.hpp>
#include <iap/planner/lidar_observability_fim.hpp>
#include <iap/planner/pl_grid.hpp>
#include <iap/planner/predicted_araim.hpp>

namespace iap {

class FuturePLFieldPredictor {
 public:
  struct Params {
    PredictedAraimComputer::Params araim_params;

    bool use_grid = false;
    double grid_resolution_m = 1.0;
    double grid_size_x_m = 30.0;
    double grid_size_y_m = 30.0;
    double grid_size_z_m = 8.0;
    double grid_max_age_s = 2.0;

    bool use_fused_fim_grid = false;
    bool use_lidar_observability = false;
    bool use_advisory_fim_add = false;
    bool use_lidar_advisory_fim = false;
    double fim_epsilon = 1.0e-6;
    double lidar_fim_radius_m = 8.0;
    int lidar_fim_min_voxels = 6;
    double lidar_fim_range_sigma_base = 0.5;
    double lidar_fim_condition_max = 1.0e6;
    double lidar_fim_weight_scale = 1.0;
    double K_H_adv = 5.0;
    double K_V_adv = 5.0;
    double b_H_pred = 0.0;
    double b_V_pred = 0.0;
    double s_H_pred = 0.0;
    double s_V_pred = 0.0;
    double lidar_search_radius_m = 8.0;
    int lidar_min_points = 12;
    int lidar_good_points = 80;
    double lidar_sigma_m = 0.5;
    double lidar_info_scale = 1.0;
    double lidar_alpha_min = 0.02;
    double lidar_alpha_max = 1.0;
    double lidar_condition_ref = 30.0;
    double lidar_condition_max = 1.0e6;
    double lidar_tdop_ref = 2.0;
    double lidar_tdop_max = 20.0;
    double lidar_bias_h_m = 0.0;
    double lidar_bias_v_m = 0.0;
  };

  struct GridStats {
    bool enabled = false;
    bool active = false;
    double resolution_m = 1.0;
    double size_x_m = 30.0;
    double size_y_m = 30.0;
    double size_z_m = 8.0;
    int generation = -1;
    int update_count = 0;
    int skip_count = 0;
    int query_grid_count = 0;
    int query_direct_count = 0;
    int query_fallback_count = 0;
    double last_build_time_ms = std::numeric_limits<double>::quiet_NaN();
    double mean_build_time_ms = std::numeric_limits<double>::quiet_NaN();
    double max_build_time_ms = std::numeric_limits<double>::quiet_NaN();
    double last_grid_age_s = std::numeric_limits<double>::quiet_NaN();
    double last_self_check_pl_ratio =
        std::numeric_limits<double>::quiet_NaN();

    bool lidar_enabled = false;
    int lidar_query_count = 0;
    int lidar_valid_count = 0;
    int lidar_fallback_count = 0;
    int lidar_conservative_check_count = 0;
    int lidar_conservative_violation_count = 0;
    int lidar_nonfinite_debug_count = 0;
    double mean_lidar_alpha = std::numeric_limits<double>::quiet_NaN();
    double max_lidar_alpha = std::numeric_limits<double>::quiet_NaN();
    double mean_lidar_tdop = std::numeric_limits<double>::quiet_NaN();
    double mean_lidar_condition = std::numeric_limits<double>::quiet_NaN();
    int fim_query_count = 0;
    int fim_regularized_count = 0;
    int gnss_fim_valid_count = 0;
    int lidar_fim_valid_count = 0;
    std::map<std::string, int> lidar_fallback_reason_histogram;
  };

  FuturePLFieldPredictor();
  explicit FuturePLFieldPredictor(const Params& params);

  void set_occupancy(const LocalOccupancyGrid* grid);
  void set_lidar_map_points(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> points);
  void set_lidar_fim_primitives(
      std::shared_ptr<const std::vector<LidarFimPrimitive>> primitives);
  void update_snapshot(const IntegritySnapshot& snapshot);
  void set_params(const Params& params);

  FuturePLQueryResult evaluate_point_direct(const Eigen::Vector3d& p_w) const;
  FuturePLQueryResult query(const Eigen::Vector3d& p_w, double now_s) const;
  bool rebuild_grid(double now_s);

  const Params& params() const { return params_; }
  std::shared_ptr<const PLGrid> active_grid() const;
  GridStats stats() const;

 private:
  FuturePLQueryResult query_grid(const Eigen::Vector3d& p_w,
                                 double now_s) const;
  FuturePLQueryResult evaluate_point(
      const Eigen::Vector3d& p_w,
      const IntegritySnapshot& snapshot,
      const std::shared_ptr<const std::vector<Eigen::Vector3d>>& points,
      const std::shared_ptr<const std::vector<LidarFimPrimitive>>& primitives,
      const std::string& query_source) const;
  void record_query(const FuturePLQueryResult& result) const;
  void refresh_stats_params();

  Params params_;
  const LocalOccupancyGrid* occupancy_ = nullptr;

  mutable std::mutex snapshot_mutex_;
  IntegritySnapshot snapshot_;

  mutable std::mutex grid_mutex_;
  std::shared_ptr<PLGrid> active_grid_;

  mutable std::mutex lidar_map_mutex_;
  std::shared_ptr<const std::vector<Eigen::Vector3d>> lidar_map_points_;
  std::shared_ptr<const std::vector<LidarFimPrimitive>> lidar_fim_primitives_;

  mutable std::mutex stats_mutex_;
  mutable GridStats stats_;
  mutable double build_time_sum_ms_ = 0.0;
  mutable double lidar_alpha_sum_ = 0.0;
  mutable double lidar_tdop_sum_ = 0.0;
  mutable double lidar_condition_sum_ = 0.0;
  int next_generation_ = 0;
};

}  // namespace iap
