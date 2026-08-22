#ifndef _P0_OCCUPANCY_EPOCH_ADAPTER_H_
#define _P0_OCCUPANCY_EPOCH_ADAPTER_H_

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <iap/map/local_occupancy.hpp>
#include <iap/planner/risk_grid_map.hpp>

namespace ego_planner {

struct P0OccupancyEpoch {
  using SourceOwner = std::shared_ptr<const void>;
  using LiveSourceOwner = std::function<SourceOwner()>;
  using LiveGeneration = std::function<uint64_t()>;

  iap::RiskGridMap::OccupancyDiagnosticQuery diagnostic_query;
  std::shared_ptr<const iap::LocalOccupancyGrid> los_owner;
  SourceOwner source_owner;
  LiveSourceOwner live_source_owner;
  LiveGeneration live_generation;
  uint64_t generation = 0;
  double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::string frame_id;
};

enum class P0OccupancyEpochCaptureStatus {
  VALID = 0,
  SNAPSHOT_UNAVAILABLE,
  ADAPTER_INVALID,
};

struct P0OccupancyEpochCapture {
  P0OccupancyEpochCaptureStatus status =
      P0OccupancyEpochCaptureStatus::SNAPSHOT_UNAVAILABLE;
  std::optional<P0OccupancyEpoch> epoch;
};

class P0OccupancyEpochAdapter {
 public:
  template <typename CapturedEpoch>
  static std::optional<P0OccupancyEpoch> adapt(
      const CapturedEpoch& epoch,
      P0OccupancyEpoch::SourceOwner source_owner,
      P0OccupancyEpoch::LiveSourceOwner live_source_owner,
      P0OccupancyEpoch::LiveGeneration live_generation) {
    iap::RiskGridMap::OccupancyDiagnosticQuery diagnostic_query;
    if (epoch.diagnostic_query) {
      const auto neutral_query = epoch.diagnostic_query;
      diagnostic_query =
          [neutral_query](const Eigen::Vector3d& position) {
            const auto source = neutral_query(position);
            iap::RiskOccupancyDiagnostic out;
            out.available = source.available;
            out.raw_occupied = source.raw_occupied;
            out.inflated_occupied = source.inflated_occupied;
            out.voxel_index = source.voxel_index;
            out.voxel_center = source.voxel_center;
            out.resolution_m = source.resolution_m;
            out.inflation_m = source.inflation_m;
            out.frame_id = source.frame_id;
            out.cloud_stamp_s = source.cloud_stamp_s;
            out.occupancy_generation = source.generation;
            out.source = source.source;
            return out;
          };
    }
    return adaptFields(epoch.raw_occupied_voxel_centers,
                       epoch.lattice_origin, epoch.resolution_m,
                       epoch.frame_id, epoch.cloud_stamp_s,
                       epoch.generation, std::move(diagnostic_query),
                       std::move(source_owner),
                       std::move(live_source_owner),
                       std::move(live_generation));
  }

 private:
  static std::optional<P0OccupancyEpoch> adaptFields(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> occupied_centers,
      const Eigen::Vector3d& lattice_origin,
      double resolution_m,
      std::string frame_id,
      double cloud_stamp_s,
      uint64_t generation,
      iap::RiskGridMap::OccupancyDiagnosticQuery diagnostic_query,
      P0OccupancyEpoch::SourceOwner source_owner,
      P0OccupancyEpoch::LiveSourceOwner live_source_owner,
      P0OccupancyEpoch::LiveGeneration live_generation);
};

}  // namespace ego_planner

#endif
