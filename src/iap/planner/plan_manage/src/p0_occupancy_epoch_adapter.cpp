#include <ego_planner/p0_occupancy_epoch_adapter.h>

#include <climits>
#include <cmath>
#include <utility>

namespace ego_planner {

std::optional<P0OccupancyEpoch> P0OccupancyEpochAdapter::adaptFields(
    std::shared_ptr<const std::vector<Eigen::Vector3d>> occupied_centers,
    const Eigen::Vector3d& lattice_origin,
    const double resolution_m,
    std::string frame_id,
    const double cloud_stamp_s,
    const uint64_t generation,
    iap::RiskGridMap::OccupancyDiagnosticQuery diagnostic_query,
    P0OccupancyEpoch::SourceOwner source_owner,
    P0OccupancyEpoch::LiveSourceOwner live_source_owner,
    P0OccupancyEpoch::LiveGeneration live_generation) {
  if (!diagnostic_query || !occupied_centers || !source_owner ||
      !live_source_owner || !live_generation ||
      generation == 0u || !std::isfinite(cloud_stamp_s) ||
      frame_id.empty() || !std::isfinite(resolution_m) ||
      resolution_m <= 0.0 || !lattice_origin.allFinite()) {
    return std::nullopt;
  }

  const std::size_t captured_count = occupied_centers->size();
  if (captured_count > static_cast<std::size_t>(INT_MAX)) {
    return std::nullopt;
  }
  for (const auto& center : *occupied_centers) {
    if (!center.allFinite()) {
      return std::nullopt;
    }
  }

  iap::LocalOccupancyGrid::Params params;
  params.voxel_size = resolution_m;
  params.lattice_origin = lattice_origin;
  params.max_voxels = captured_count == 0u
      ? 1
      : static_cast<int>(captured_count);
  params.enable_eviction = false;
  auto los_owner = std::make_shared<iap::LocalOccupancyGrid>(params);
  los_owner->insert_points(*occupied_centers);
  const auto diagnostics = los_owner->diagnostics();
  if (diagnostics.rejected_count != 0u ||
      diagnostics.inserted_count != captured_count ||
      diagnostics.voxel_count != captured_count ||
      los_owner->size() != captured_count) {
    return std::nullopt;
  }

  P0OccupancyEpoch adapted;
  adapted.diagnostic_query = std::move(diagnostic_query);
  adapted.los_owner = std::move(los_owner);
  adapted.source_owner = std::move(source_owner);
  adapted.live_source_owner = std::move(live_source_owner);
  adapted.live_generation = std::move(live_generation);
  adapted.generation = generation;
  adapted.cloud_stamp_s = cloud_stamp_s;
  adapted.frame_id = std::move(frame_id);
  return adapted;
}

}  // namespace ego_planner
