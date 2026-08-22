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

struct P0RawOccupancyChangedBounds {
  iap::VoxelKey minimum;
  iap::VoxelKey maximum;
};

// Immutable normalized identity for one complete raw-occupancy capture. Keys
// are unique and lexicographically sorted on the captured fixed lattice.
class P0RawOccupancyIdentity {
 public:
  const std::vector<iap::VoxelKey>& keys() const { return keys_; }
  const Eigen::Vector3d& latticeOrigin() const { return lattice_origin_; }
  double resolutionM() const { return resolution_m_; }
  const std::string& frameId() const { return frame_id_; }

 private:
  friend class P0OccupancyEpochAdapter;
  P0RawOccupancyIdentity(std::vector<iap::VoxelKey> keys,
                         Eigen::Vector3d lattice_origin,
                         double resolution_m,
                         std::string frame_id)
      : keys_(std::move(keys)),
        lattice_origin_(std::move(lattice_origin)),
        resolution_m_(resolution_m),
        frame_id_(std::move(frame_id)) {}

  std::vector<iap::VoxelKey> keys_;
  Eigen::Vector3d lattice_origin_;
  double resolution_m_;
  std::string frame_id_;
};

// Complete net set difference between two coherent immutable captures.
// Absence of this object means the comparison could not be proven safely.
class P0RawOccupancyDelta {
 public:
  uint64_t baseGeneration() const { return base_generation_; }
  uint64_t targetGeneration() const { return target_generation_; }
  const std::vector<iap::VoxelKey>& addedKeys() const { return added_keys_; }
  const std::vector<iap::VoxelKey>& removedKeys() const {
    return removed_keys_;
  }
  const std::optional<P0RawOccupancyChangedBounds>& changedBounds() const {
    return changed_bounds_;
  }
  bool empty() const {
    return added_keys_.empty() && removed_keys_.empty();
  }

 private:
  friend class P0OccupancyEpochAdapter;
  uint64_t base_generation_ = 0;
  uint64_t target_generation_ = 0;
  std::vector<iap::VoxelKey> added_keys_;
  std::vector<iap::VoxelKey> removed_keys_;
  std::optional<P0RawOccupancyChangedBounds> changed_bounds_;
};

struct P0OccupancyEpoch {
  using SourceOwner = std::shared_ptr<const void>;
  using LiveSourceOwner = std::function<SourceOwner()>;
  using LiveGeneration = std::function<uint64_t()>;

  iap::RiskGridMap::OccupancyDiagnosticQuery diagnostic_query;
  std::shared_ptr<const iap::LocalOccupancyGrid> los_owner;
  std::shared_ptr<const P0RawOccupancyIdentity> raw_identity;
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

  static bool sameVersion(const P0OccupancyEpoch& base,
                          const P0OccupancyEpoch& target);
  static std::optional<P0RawOccupancyDelta> completeDelta(
      const P0OccupancyEpoch& base, const P0OccupancyEpoch& target);

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
