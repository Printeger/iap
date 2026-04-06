#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace iap {

struct BSplineLidarFactorTelemetryMetadata {
  std::size_t delta_factor_index{0};
  std::size_t source_frame_index{0};
  std::vector<std::size_t> support_control_indices;
};

struct BSplineLidarFactorMetadata {
  std::size_t source_frame_index{0};
  std::vector<std::size_t> support_control_indices;
};

using BSplineLidarFactorMetadataMap = std::unordered_map<std::size_t, BSplineLidarFactorMetadata>;

struct BSplineLidarFactorChurnCounts {
  std::size_t current_segment_factor_count{0};
  std::size_t old_segment_factor_count{0};
  std::size_t same_support_factor_count{0};
  std::size_t cross_support_factor_count{0};
};

void register_lidar_factor_metadata(
  BSplineLidarFactorMetadataMap& metadata_by_factor_index,
  const gtsam::FactorIndices& new_factor_indices,
  const gtsam::NonlinearFactorGraph& new_factors,
  const std::vector<BSplineLidarFactorTelemetryMetadata>& lidar_metadata);

BSplineLidarFactorChurnCounts count_lidar_factor_indices(
  const gtsam::NonlinearFactorGraph& graph,
  gtsam::FactorIndices factor_indices,
  const BSplineLidarFactorMetadataMap& metadata_by_factor_index,
  std::size_t current_source_frame_index,
  const std::vector<std::size_t>& current_support_control_indices);

BSplineLidarFactorChurnCounts count_active_window_lidar_factors(
  const gtsam::NonlinearFactorGraph& graph,
  const gtsam::KeyVector& active_keys,
  const BSplineLidarFactorMetadataMap& metadata_by_factor_index,
  std::size_t current_source_frame_index,
  const std::vector<std::size_t>& current_support_control_indices);

}  // namespace iap
