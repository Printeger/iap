#include <iap/planner/p1_accepted_context_validation.hpp>

#include <algorithm>
#include <cmath>

namespace iap {

P1AcceptedContextValidation validateP1AcceptedContext(
    const P1AcceptedContextValidationInput& input) {
  P1AcceptedContextValidation result;
  result.expected_sample_count = input.samples.size();
  result.snapshot_available = static_cast<bool>(input.snapshot);
  if (!input.snapshot) {
    result.failure_reasons.push_back("snapshot_unavailable");
    return result;
  }

  const auto& snapshot = *input.snapshot;
  const auto& params = snapshot.params();
  const Eigen::Vector3i dimensions = snapshot.voxelNum();
  const double resolution = params.resolution_m;
  result.interpolation_min =
      snapshot.origin() + Eigen::Vector3d::Constant(0.5 * resolution);
  result.interpolation_max = snapshot.origin() +
      (dimensions.cast<double>() - Eigen::Vector3d::Constant(0.5)) *
          resolution;

  const std::string snapshot_frame = input.snapshot_frame_id.empty()
      ? params.frame_id
      : input.snapshot_frame_id;
  result.frame_match = !snapshot_frame.empty() &&
                       !input.trajectory_frame_id.empty() &&
                       snapshot_frame == input.trajectory_frame_id;
  result.generation_match = input.expected_generation_id != 0 &&
      input.expected_generation_id == snapshot.generation_id();
  result.query_time_match = std::isfinite(input.query_base_time_s) &&
      std::isfinite(snapshot.stamp_s()) &&
      std::abs(input.query_base_time_s - snapshot.stamp_s()) <= 1.0e-9;

  if (!params.horizons_s.empty()) {
    const auto horizon_range = std::minmax_element(
        params.horizons_s.begin(), params.horizons_s.end());
    result.horizon_min_s = *horizon_range.first;
    result.horizon_max_s = *horizon_range.second;
  }
  result.context_age_s = input.accepted_stamp_s - snapshot.stamp_s();
  result.fresh = std::isfinite(result.context_age_s) &&
      result.context_age_s >= 0.0 &&
      (params.stale_timeout_s < 0.0 ||
       result.context_age_s <= params.stale_timeout_s);

  result.spatial_in_bounds = !input.samples.empty() &&
      result.interpolation_min.allFinite() &&
      result.interpolation_max.allFinite();
  result.temporal_in_horizon = !input.samples.empty() &&
      std::isfinite(result.horizon_min_s) &&
      std::isfinite(result.horizon_max_s) &&
      std::isfinite(input.query_base_time_s);

  constexpr double kBoundTolerance = 1.0e-12;
  for (const auto& sample : input.samples) {
    bool sample_spatial = sample.position_w.allFinite();
    for (int axis = 0; sample_spatial && axis < 3; ++axis) {
      sample_spatial =
          sample.position_w(axis) >= result.interpolation_min(axis) -
                                         kBoundTolerance &&
          sample.position_w(axis) < result.interpolation_max(axis) -
                                        kBoundTolerance;
    }
    const double query_time_s =
        input.query_base_time_s + sample.trajectory_time_s;
    const bool sample_temporal =
        std::isfinite(sample.trajectory_time_s) &&
        std::isfinite(query_time_s) &&
        query_time_s >= input.query_base_time_s + result.horizon_min_s -
                            kBoundTolerance &&
        query_time_s <= input.query_base_time_s + result.horizon_max_s +
                            kBoundTolerance;
    result.spatial_in_bounds = result.spatial_in_bounds && sample_spatial;
    result.temporal_in_horizon =
        result.temporal_in_horizon && sample_temporal;

    std::string category;
    if (!sample_spatial) {
      ++result.spatial_miss_count;
      category = "spatial_out_of_interpolation_bounds";
    } else if (!sample_temporal) {
      ++result.temporal_miss_count;
      category = "time_out_of_horizon";
    } else if (sample.query_stale ||
               sample.query_reason.find("stale") != std::string::npos) {
      ++result.stale_miss_count;
      category = "stale";
    } else if (sample.query_reason == "occupied" ||
               sample.query_reason.find("occupied") != std::string::npos) {
      ++result.occupied_miss_count;
      category = "occupied";
    } else if (sample.query_hit && sample.query_valid) {
      ++result.covered_sample_count;
      category = "matched";
    } else {
      ++result.invalid_miss_count;
      category = sample.query_reason.empty() ? "query_miss"
                                             : sample.query_reason;
    }
    ++result.reason_counts[category];
  }

  result.coverage_ratio = result.expected_sample_count == 0
      ? 0.0
      : static_cast<double>(result.covered_sample_count) /
            static_cast<double>(result.expected_sample_count);
  result.coverage_ok =
      result.covered_sample_count >= input.minimum_covered_samples &&
      result.coverage_ratio >= input.minimum_coverage_ratio;

  const auto record_failure = [&result](const bool condition,
                                        const char* reason) {
    if (!condition) {
      result.failure_reasons.emplace_back(reason);
    }
  };
  record_failure(result.frame_match, "frame_mismatch");
  record_failure(result.generation_match, "generation_mismatch");
  record_failure(result.query_time_match, "query_base_time_mismatch");
  record_failure(result.fresh, "stale_context");
  record_failure(result.spatial_in_bounds,
                 "spatial_out_of_interpolation_bounds");
  record_failure(result.temporal_in_horizon, "temporal_out_of_horizon");
  record_failure(result.coverage_ok, "coverage_insufficient");
  result.valid = result.failure_reasons.empty();
  return result;
}

}  // namespace iap
