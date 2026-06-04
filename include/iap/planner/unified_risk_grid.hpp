#pragma once
// Stage 4: unified advisory risk grid for planner-compatible PI queries.

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <vector>

namespace iap {

enum UnifiedRiskFlags : uint32_t {
  VALID_ESDF = 1u << 0,
  VALID_OCCUPANCY = 1u << 1,
  VALID_AL = 1u << 2,
  VALID_ADVISORY_PL = 1u << 3,
  VALID_PI = 1u << 4,
  STALE_PL = 1u << 5,
  UNKNOWN_RISK = 1u << 6,
  OCCUPIED = 1u << 7,
  OUT_OF_RANGE = 1u << 8,
  FIM_ADD_USED = 1u << 9,
  LIDAR_FIM_VALID = 1u << 10,
  GNSS_FIM_VALID = 1u << 11,
  PI_INPUT_VALID = 1u << 12,
};

struct UnifiedRiskVoxel {
  float esdf_m = std::numeric_limits<float>::quiet_NaN();
  float occ_prob = std::numeric_limits<float>::quiet_NaN();
  float al_h_m = std::numeric_limits<float>::quiet_NaN();
  float al_v_m = std::numeric_limits<float>::quiet_NaN();
  float hal_m = std::numeric_limits<float>::quiet_NaN();
  float val_m = std::numeric_limits<float>::quiet_NaN();

  float hpl_adv_m = std::numeric_limits<float>::quiet_NaN();
  float vpl_adv_m = std::numeric_limits<float>::quiet_NaN();
  float pl_adv_m = std::numeric_limits<float>::quiet_NaN();

  float im_h_adv_m = std::numeric_limits<float>::quiet_NaN();
  float im_v_adv_m = std::numeric_limits<float>::quiet_NaN();
  float im_min_adv_m = std::numeric_limits<float>::quiet_NaN();

  float pi_cost = std::numeric_limits<float>::quiet_NaN();
  float pi_grad_x = std::numeric_limits<float>::quiet_NaN();
  float pi_grad_y = std::numeric_limits<float>::quiet_NaN();
  float pi_grad_z = std::numeric_limits<float>::quiet_NaN();

  double updated_time_s = std::numeric_limits<double>::quiet_NaN();
  float age_s = std::numeric_limits<float>::quiet_NaN();
  uint32_t flags = 0;
};

struct UnifiedRiskQueryResult {
  bool valid = false;
  bool grid_hit = false;
  bool grid_miss = false;
  bool direct_query_used = false;
  Eigen::Vector3d position = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  UnifiedRiskVoxel voxel;
  uint32_t flags = 0;
  int grid_generation = -1;
  float unknown_penalty = 0.0f;
  const char* query_source = "miss";
};

struct UnifiedRiskQueryOptions {
  double now_s = std::numeric_limits<double>::quiet_NaN();
  double fresh_timeout_s = 1.0;
  double stale_timeout_s = 5.0;
  double unknown_penalty = 3000.0;
  double unknown_tau_s = 2.0;
  bool direct_query_on_miss = true;
  std::function<bool(const Eigen::Vector3d&, UnifiedRiskVoxel*)> direct_query;
};

class UnifiedRiskGrid {
 public:
  struct UpdateTiming {
    double pl_query_ms = std::numeric_limits<double>::quiet_NaN();
    double al_esdf_ms = std::numeric_limits<double>::quiet_NaN();
    double pi_ms = std::numeric_limits<double>::quiet_NaN();
    double gradient_ms = std::numeric_limits<double>::quiet_NaN();
    double csv_ms = std::numeric_limits<double>::quiet_NaN();
    double total_ms = std::numeric_limits<double>::quiet_NaN();
  };

  struct Stats {
    bool enabled = false;
    bool active = false;
    int generation = -1;
    int update_count = 0;
    int query_count = 0;
    int direct_query_count = 0;
    int grid_hit_count = 0;
    int grid_miss_count = 0;
    int stale_count = 0;
    int unknown_count = 0;
    int valid_pi_count = 0;
    int unknown_penalty_count = 0;
    double max_age_s = std::numeric_limits<double>::quiet_NaN();
    double mean_update_ms = std::numeric_limits<double>::quiet_NaN();
    double p95_update_ms = std::numeric_limits<double>::quiet_NaN();
    UpdateTiming last_timing;
    int front_field_points = 0;
    int backend_field_points = 0;
    std::map<uint32_t, int> flags_histogram;
  };

  bool reset(const Eigen::Vector3d& center,
             double half_extent_x_m,
             double half_extent_y_m,
             int z_slices,
             double resolution_m);

  bool valid() const { return valid_; }
  bool contains(const Eigen::Vector3d& p) const;
  bool cell_index(const Eigen::Vector3d& p, int* ix, int* iy, int* iz) const;

  UnifiedRiskVoxel& at(int ix, int iy, int iz);
  const UnifiedRiskVoxel& at(int ix, int iy, int iz) const;
  Eigen::Vector3d position(int ix, int iy, int iz) const;

  UnifiedRiskQueryResult interpolate(const Eigen::Vector3d& p) const;
  UnifiedRiskQueryResult queryRisk(const Eigen::Vector3d& p,
                                   const UnifiedRiskQueryOptions& options);
  void compute_gradients();
  void zero_gradients();

  const Eigen::Vector3d& center() const { return center_; }
  const Eigen::Vector3d& min_corner() const { return min_corner_; }
  double resolution() const { return resolution_; }
  int nx() const { return nx_; }
  int ny() const { return ny_; }
  int nz() const { return nz_; }
  int generation() const { return generation_; }
  void set_generation(int generation) { generation_ = generation; }
  double stamp_s() const { return stamp_s_; }
  void set_stamp_s(double value) { stamp_s_ = value; }
  double build_time_ms() const { return build_time_ms_; }
  void set_build_time_ms(double value) { build_time_ms_ = value; }
  void note_update(double build_time_ms, const UpdateTiming& timing);
  void set_enabled(bool enabled) { stats_.enabled = enabled; }
  void set_front_field_points(int value) { stats_.front_field_points = value; }
  void set_backend_field_points(int value) { stats_.backend_field_points = value; }
  const Stats& stats() const { return stats_; }

 private:
  std::size_t flat_index(int ix, int iy, int iz) const;
  bool in_bounds(int ix, int iy, int iz) const;

  bool valid_ = false;
  Eigen::Vector3d center_ = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d min_corner_ = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double resolution_ = 1.0;
  int nx_ = 0;
  int ny_ = 0;
  int nz_ = 0;
  int generation_ = -1;
  double stamp_s_ = std::numeric_limits<double>::quiet_NaN();
  double build_time_ms_ = std::numeric_limits<double>::quiet_NaN();
  std::vector<UnifiedRiskVoxel> cells_;
  Stats stats_;
  std::vector<double> update_times_ms_;
};

double unified_risk_unknown_penalty(double age_s,
                                    double unknown_penalty,
                                    double unknown_tau_s);
void apply_unified_risk_stale_policy(UnifiedRiskQueryResult* result,
                                     const UnifiedRiskQueryOptions& options,
                                     double grid_stamp_s);

}  // namespace iap
