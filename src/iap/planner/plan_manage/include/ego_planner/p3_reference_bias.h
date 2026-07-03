#ifndef EGO_PLANNER_P3_REFERENCE_BIAS_H_
#define EGO_PLANNER_P3_REFERENCE_BIAS_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>

namespace iap
{
  class RiskGridSnapshot;
}

namespace ego_planner
{

struct P3ReferenceBiasConfig
{
  bool enable_local_reference_bias = false;
  bool enable_global_reference_bias = false;
  double local_bias_radius_m = 1.5;
  double min_improvement_ratio = 0.05;
  double w_risk = 1.0;
  double w_detour = 0.25;
  double w_unknown = 5.0;
  double min_corridor_valid_ratio = 0.8;
  double station_spacing_m = 2.0;
  double lateral_sample_step_m = 1.0;
  int lateral_sample_count_each_side = 3;
  int beam_width = 5;
  double max_detour_ratio = 1.5;
  bool debug_csv_enable = false;
  std::string debug_csv_path;
};

struct P3LocalBiasInput
{
  Eigen::Vector3d start_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d nominal_target = Eigen::Vector3d::Zero();
  double max_vel = 1.0;
};

struct P3LocalBiasResult
{
  bool used_bias = false;
  Eigen::Vector3d start_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_pt = Eigen::Vector3d::Zero();
  Eigen::Vector3d nominal_target = Eigen::Vector3d::Zero();
  Eigen::Vector3d target = Eigen::Vector3d::Zero();
  double nominal_score = 0.0;
  double biased_score = 0.0;
  double improvement_ratio = 0.0;
  uint64_t snapshot_generation_id = 0;
  int candidate_count = 0;
  int valid_count = 0;
  int unknown_count = 0;
  int occupied_count = 0;
  int out_of_map_count = 0;
  std::string reason = "not_evaluated";
};

struct P3GlobalBiasInput
{
  Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_pos = Eigen::Vector3d::Zero();
  double max_vel = 1.0;
};

struct P3GlobalBiasResult
{
  bool used_bias = false;
  Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d end_pos = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> biased_waypoints;
  double nominal_score = 0.0;
  double biased_score = 0.0;
  double improvement_ratio = 0.0;
  double corridor_valid_ratio = 0.0;
  double detour_ratio = 1.0;
  uint64_t snapshot_generation_id = 0;
  int station_count = 0;
  int candidate_count = 0;
  int valid_count = 0;
  int unknown_count = 0;
  int occupied_count = 0;
  std::string reason = "not_evaluated";
};

using P3PositionSafetyFn = std::function<bool(const Eigen::Vector3d&)>;

P3LocalBiasResult applyP3LocalReferenceBias(
    const P3LocalBiasInput& input,
    const P3ReferenceBiasConfig& config,
    std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
    const P3PositionSafetyFn& is_position_safe,
    double stamp_s,
    uint64_t batch_id);

P3GlobalBiasResult computeP3GlobalReferenceBias(
    const P3GlobalBiasInput& input,
    const P3ReferenceBiasConfig& config,
    std::shared_ptr<const iap::RiskGridSnapshot> snapshot,
    const P3PositionSafetyFn& is_position_safe,
    double stamp_s,
    uint64_t batch_id);

}  // namespace ego_planner

#endif  // EGO_PLANNER_P3_REFERENCE_BIAS_H_
