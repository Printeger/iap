#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <iap/planner/risk_grid_map.hpp>

namespace iap {

struct P1AcceptedContextSample {
  Eigen::Vector3d position_w = Eigen::Vector3d::Zero();
  double trajectory_time_s = std::numeric_limits<double>::quiet_NaN();
  bool query_hit = false;
  bool query_valid = false;
  bool query_stale = true;
  std::string query_reason = "not_evaluated";
};

struct P1AcceptedContextValidationInput {
  std::shared_ptr<const RiskGridSnapshot> snapshot;
  std::string snapshot_frame_id;
  std::string trajectory_frame_id;
  uint64_t expected_generation_id = 0;
  double query_base_time_s = std::numeric_limits<double>::quiet_NaN();
  double accepted_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t minimum_covered_samples = 20;
  double minimum_coverage_ratio = 0.5;
  std::vector<P1AcceptedContextSample> samples;
};

struct P1AcceptedContextValidation {
  bool valid = false;
  bool snapshot_available = false;
  bool spatial_in_bounds = false;
  bool temporal_in_horizon = false;
  bool frame_match = false;
  bool generation_match = false;
  bool query_time_match = false;
  bool fresh = false;
  bool coverage_ok = false;
  Eigen::Vector3d interpolation_min = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d interpolation_max = Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
  double horizon_min_s = std::numeric_limits<double>::quiet_NaN();
  double horizon_max_s = std::numeric_limits<double>::quiet_NaN();
  double context_age_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t expected_sample_count = 0;
  std::size_t covered_sample_count = 0;
  double coverage_ratio = 0.0;
  std::size_t spatial_miss_count = 0;
  std::size_t temporal_miss_count = 0;
  std::size_t occupied_miss_count = 0;
  std::size_t stale_miss_count = 0;
  std::size_t invalid_miss_count = 0;
  std::map<std::string, std::size_t> reason_counts;
  std::vector<std::string> failure_reasons;
};

P1AcceptedContextValidation validateP1AcceptedContext(
    const P1AcceptedContextValidationInput& input);

}  // namespace iap
