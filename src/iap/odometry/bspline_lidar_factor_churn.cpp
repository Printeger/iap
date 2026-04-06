#include <iap/odometry/bspline_lidar_factor_churn.hpp>

#include <iap/odometry/bspline_factor_family.hpp>

#include <algorithm>
#include <unordered_set>

namespace iap {

namespace {

std::vector<std::size_t> sort_unique_indices(std::vector<std::size_t> indices) {
  std::sort(indices.begin(), indices.end());
  indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
  return indices;
}

std::unordered_set<gtsam::Key> to_key_set(const gtsam::KeyVector& keys) {
  return std::unordered_set<gtsam::Key>(keys.begin(), keys.end());
}

bool factor_keys_active(
  const gtsam::NonlinearFactor::shared_ptr& factor,
  const std::unordered_set<gtsam::Key>& active_key_set) {
  if (!factor) {
    return false;
  }
  for (const auto key : factor->keys()) {
    if (!active_key_set.count(key)) {
      return false;
    }
  }
  return true;
}

bool support_sets_overlap(
  const std::vector<std::size_t>& lhs,
  const std::vector<std::size_t>& rhs) {
  if (lhs.empty() || rhs.empty()) {
    return false;
  }

  const auto* smaller = &lhs;
  const auto* larger = &rhs;
  if (lhs.size() > rhs.size()) {
    smaller = &rhs;
    larger = &lhs;
  }

  const std::unordered_set<std::size_t> larger_set(larger->begin(), larger->end());
  for (const auto control_index : *smaller) {
    if (larger_set.count(control_index)) {
      return true;
    }
  }
  return false;
}

void accumulate_lidar_churn_counts(
  BSplineLidarFactorChurnCounts& counts,
  const BSplineLidarFactorMetadata& metadata,
  std::size_t current_source_frame_index,
  const std::vector<std::size_t>& current_support_control_indices) {
  if (metadata.source_frame_index == current_source_frame_index) {
    ++counts.current_segment_factor_count;
  } else {
    ++counts.old_segment_factor_count;
  }

  if (current_support_control_indices.empty()) {
    return;
  }

  if (support_sets_overlap(metadata.support_control_indices, current_support_control_indices)) {
    ++counts.same_support_factor_count;
  } else {
    ++counts.cross_support_factor_count;
  }
}

}  // namespace

void register_lidar_factor_metadata(
  BSplineLidarFactorMetadataMap& metadata_by_factor_index,
  const gtsam::FactorIndices& new_factor_indices,
  const gtsam::NonlinearFactorGraph& new_factors,
  const std::vector<BSplineLidarFactorTelemetryMetadata>& lidar_metadata) {
  for (const auto& entry : lidar_metadata) {
    if (entry.delta_factor_index >= new_factor_indices.size() || entry.delta_factor_index >= new_factors.size()) {
      continue;
    }

    const auto& factor = new_factors[entry.delta_factor_index];
    if (!factor || classify_bspline_factor_family(*factor) != BSplineFactorFamily::LIDAR) {
      continue;
    }

    metadata_by_factor_index[new_factor_indices[entry.delta_factor_index]] = BSplineLidarFactorMetadata{
      entry.source_frame_index,
      sort_unique_indices(entry.support_control_indices),
    };
  }
}

BSplineLidarFactorChurnCounts count_lidar_factor_indices(
  const gtsam::NonlinearFactorGraph& graph,
  gtsam::FactorIndices factor_indices,
  const BSplineLidarFactorMetadataMap& metadata_by_factor_index,
  std::size_t current_source_frame_index,
  const std::vector<std::size_t>& current_support_control_indices) {
  std::sort(factor_indices.begin(), factor_indices.end());
  factor_indices.erase(std::unique(factor_indices.begin(), factor_indices.end()), factor_indices.end());

  BSplineLidarFactorChurnCounts counts;
  for (const auto idx : factor_indices) {
    if (idx >= graph.size()) {
      continue;
    }
    const auto& factor = graph[idx];
    if (!factor || classify_bspline_factor_family(*factor) != BSplineFactorFamily::LIDAR) {
      continue;
    }
    const auto metadata_it = metadata_by_factor_index.find(idx);
    if (metadata_it == metadata_by_factor_index.end()) {
      continue;
    }
    accumulate_lidar_churn_counts(
      counts,
      metadata_it->second,
      current_source_frame_index,
      current_support_control_indices);
  }
  return counts;
}

BSplineLidarFactorChurnCounts count_active_window_lidar_factors(
  const gtsam::NonlinearFactorGraph& graph,
  const gtsam::KeyVector& active_keys,
  const BSplineLidarFactorMetadataMap& metadata_by_factor_index,
  std::size_t current_source_frame_index,
  const std::vector<std::size_t>& current_support_control_indices) {
  const auto active_key_set = to_key_set(active_keys);

  BSplineLidarFactorChurnCounts counts;
  for (std::size_t idx = 0; idx < graph.size(); ++idx) {
    const auto& factor = graph[idx];
    if (!factor_keys_active(factor, active_key_set)) {
      continue;
    }
    if (classify_bspline_factor_family(*factor) != BSplineFactorFamily::LIDAR) {
      continue;
    }
    const auto metadata_it = metadata_by_factor_index.find(idx);
    if (metadata_it == metadata_by_factor_index.end()) {
      continue;
    }
    accumulate_lidar_churn_counts(
      counts,
      metadata_it->second,
      current_source_frame_index,
      current_support_control_indices);
  }

  return counts;
}

}  // namespace iap
