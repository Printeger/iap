#ifndef _P0_OCCUPANCY_EPOCH_ADAPTER_H_
#define _P0_OCCUPANCY_EPOCH_ADAPTER_H_

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include <iap/map/local_occupancy.hpp>
#include <iap/map/trusted_local_map_support.hpp>
#include <iap/planner/risk_grid_map.hpp>

struct FrozenOccupancyEpoch;

namespace ego_planner {

namespace p0_occupancy_detail {
template <typename Epoch>
auto currentFrameCenters(const Epoch& epoch, int)
    -> decltype(epoch.current_frame_occupied_voxel_centers) {
  return epoch.current_frame_occupied_voxel_centers;
}

template <typename Epoch>
std::shared_ptr<const std::vector<Eigen::Vector3d>> currentFrameCenters(
    const Epoch&, long) {
  return nullptr;
}
}  // namespace p0_occupancy_detail

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
  // Exact immutable raw-hit centers captured by the GridMap epoch. Consumers
  // reuse this vector instead of rebuilding it on the planner callback path.
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      raw_occupied_voxel_centers;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      current_frame_occupied_voxel_centers;
  std::shared_ptr<const std::vector<Eigen::Vector3d>>
      environment_occupied_voxel_centers;
  std::shared_ptr<const iap::TrustedLocalMapSupport> trusted_local_map_support;
  // The exact producer-owned epoch used to build this P0 snapshot. P4 uses
  // it as the immutable base for route-local commit validation; it must never
  // be replaced with a newly captured live map.
  std::shared_ptr<const FrozenOccupancyEpoch> frozen_grid_map_epoch;
  iap::PlanningLatticeGeometry geometry;
  SourceOwner source_owner;
  LiveSourceOwner live_source_owner;
  LiveGeneration live_generation;
  uint64_t generation = 0;
  double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
  std::string frame_id;
  std::string frame_contract_id;
};

// Resolve one point against the two independent local-map authorities. A
// ray-derived OBSERVED_FREE voxel remains an observation (and keeps
// STRICT_OBSERVATION authority); a trusted envelope remains model support.
// Either may complete support, but neither is rewritten as the other.
inline iap::LocalMapSupportQuery queryP0LocalMapSupport(
    const P0OccupancyEpoch& epoch, const Eigen::Vector3d& position,
    const double evaluation_time_s, const double query_time_s) {
  iap::LocalMapSupportQuery model;
  if (epoch.trusted_local_map_support) {
    model = epoch.trusted_local_map_support->query(
        position, evaluation_time_s, query_time_s);
    if (model.complete()) {
      return model;
    }
  }
  if (!epoch.diagnostic_query || !position.allFinite() ||
      !std::isfinite(evaluation_time_s) || !std::isfinite(query_time_s)) {
    return model;
  }
  const auto observed = epoch.diagnostic_query(position);
  if (!observed.available || !observed.observed ||
      observed.state != iap::RiskOccupancyState::OBSERVED_FREE) {
    return model;
  }
  const double hard_lifetime_s = epoch.trusted_local_map_support
      ? epoch.trusted_local_map_support->valid_until_s -
            epoch.trusted_local_map_support->stamp_s
      : -1.0;
  const double age_s = evaluation_time_s - epoch.cloud_stamp_s;
  iap::LocalMapSupportQuery strict;
  strict.authority = iap::LocalMapSupportAuthority::STRICT_OBSERVATION;
  strict.observation_stamp_s = epoch.cloud_stamp_s;
  strict.observation_age_s = age_s;
  strict.status = std::isfinite(epoch.cloud_stamp_s) && age_s >= -1.0e-6 &&
          (hard_lifetime_s < 0.0 || age_s <= hard_lifetime_s)
      ? iap::LocalMapSupportStatus::MODEL_COMPLETE
      : iap::LocalMapSupportStatus::EXPIRED;
  return strict;
}

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

// A reusable LOS grid is valid only for the exact immutable environment-point
// owner from which it was built.  Keeping both owners together prevents a
// same-size but different obstacle cloud from being reused accidentally.
struct P0ReusableLosOccupancy {
  std::shared_ptr<const std::vector<Eigen::Vector3d>> environment_centers;
  std::shared_ptr<const iap::LocalOccupancyGrid> los_owner;
};

class P0OccupancyEpochAdapter {
 public:
  template <typename CapturedEpoch>
  static std::optional<P0OccupancyEpoch> adapt(
      const CapturedEpoch& epoch,
      P0OccupancyEpoch::SourceOwner source_owner,
      P0OccupancyEpoch::LiveSourceOwner live_source_owner,
      P0OccupancyEpoch::LiveGeneration live_generation,
      double clearance_transition_m = 0.0,
      std::optional<P0ReusableLosOccupancy> reusable_los = std::nullopt) {
    iap::RiskGridMap::OccupancyDiagnosticQuery diagnostic_query;
    if (epoch.diagnostic_query) {
      const auto neutral_query = epoch.diagnostic_query;
      diagnostic_query =
          [neutral_query](const Eigen::Vector3d& position) {
            const auto source = neutral_query(position);
            iap::RiskOccupancyDiagnostic out;
            out.available = source.available;
            out.observed = source.observed;
            out.raw_occupied = source.raw_occupied;
            out.inflated_occupied = source.inflated_occupied;
            out.state = (source.raw_occupied || source.inflated_occupied)
                ? iap::RiskOccupancyState::OCCUPIED
                : source.observed
                    ? iap::RiskOccupancyState::OBSERVED_FREE
                    : iap::RiskOccupancyState::UNKNOWN;
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
                       epoch.environment_occupied_voxel_centers,
                       epoch.trusted_local_map_support,
                       epoch.lattice_origin, epoch.extent_m,
                       epoch.voxel_dimensions, epoch.resolution_m,
                       epoch.frame_id, epoch.cloud_stamp_s,
                       epoch.geometry_id, epoch.frame_contract_id,
                       epoch.generation,
                       std::move(diagnostic_query),
                       std::move(source_owner),
                       std::move(live_source_owner),
                       std::move(live_generation), clearance_transition_m,
                       std::move(reusable_los),
                       p0_occupancy_detail::currentFrameCenters(epoch, 0));
  }

  static bool sameVersion(const P0OccupancyEpoch& base,
                          const P0OccupancyEpoch& target);
  static std::optional<P0RawOccupancyDelta> completeDelta(
      const P0OccupancyEpoch& base, const P0OccupancyEpoch& target);

 private:
  static std::optional<P0OccupancyEpoch> adaptFields(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> occupied_centers,
      std::shared_ptr<const std::vector<Eigen::Vector3d>> environment_centers,
      std::shared_ptr<const iap::TrustedLocalMapSupport> trusted_support,
      const Eigen::Vector3d& lattice_origin,
      const Eigen::Vector3d& extent_m,
      const Eigen::Vector3i& voxel_dimensions,
      double resolution_m,
      std::string frame_id,
      double cloud_stamp_s,
      std::string geometry_id,
      std::string frame_contract_id,
      uint64_t generation,
      iap::RiskGridMap::OccupancyDiagnosticQuery diagnostic_query,
      P0OccupancyEpoch::SourceOwner source_owner,
      P0OccupancyEpoch::LiveSourceOwner live_source_owner,
      P0OccupancyEpoch::LiveGeneration live_generation,
      double clearance_transition_m,
      std::optional<P0ReusableLosOccupancy> reusable_los,
      std::shared_ptr<const std::vector<Eigen::Vector3d>>
          current_frame_centers);
};

}  // namespace ego_planner

#endif
