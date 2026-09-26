#ifndef _P0_OCCUPANCY_EPOCH_ADAPTER_H_
#define _P0_OCCUPANCY_EPOCH_ADAPTER_H_

#include <algorithm>
#include <cmath>
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
#include <plan_env/local_evidence_snapshot.h>

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

template <typename Epoch>
auto rawVoxelKeys(const Epoch& epoch, int)
    -> decltype(epoch.raw_occupied_voxel_keys) {
  return epoch.raw_occupied_voxel_keys;
}

template <typename Epoch>
std::shared_ptr<const std::vector<iap::VoxelKey>> rawVoxelKeys(
    const Epoch&, long) {
  return nullptr;
}

template <typename Epoch>
auto localEvidenceSnapshot(const Epoch& epoch, int)
    -> decltype(epoch.local_evidence_snapshot) {
  return epoch.local_evidence_snapshot;
}

template <typename Epoch>
std::shared_ptr<const LocalEvidenceSnapshot> localEvidenceSnapshot(
    const Epoch&, long) {
  return nullptr;
}

template <typename Epoch>
auto currentVehiclePosition(const Epoch& epoch, int)
    -> decltype(epoch.current_vehicle_position) {
  return epoch.current_vehicle_position;
}

template <typename Epoch>
Eigen::Vector3d currentVehiclePosition(const Epoch&, long) {
  return Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN());
}

template <typename Epoch>
auto currentVehicleClearanceRadius(const Epoch& epoch, int)
    -> decltype(epoch.current_vehicle_clearance_radius_m) {
  return epoch.current_vehicle_clearance_radius_m;
}

template <typename Epoch>
double currentVehicleClearanceRadius(const Epoch&, long) {
  return std::numeric_limits<double>::quiet_NaN();
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
  std::shared_ptr<const LocalEvidenceSnapshot> local_evidence_snapshot;
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
  Eigen::Vector3d current_vehicle_position =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double current_vehicle_clearance_radius_m =
      std::numeric_limits<double>::quiet_NaN();
};

// Resolve one point against the two independent local-map authorities. A
// ray-derived OBSERVED_FREE voxel remains an observation (and keeps
// STRICT_OBSERVATION authority); a trusted envelope remains model support.
// Either may complete support, but neither is rewritten as the other.
inline iap::LocalMapSupportQuery queryP0LocalMapSupport(
    const P0OccupancyEpoch& epoch, const Eigen::Vector3d& position,
    const double evaluation_time_s, const double query_time_s) {
  const auto strict_observed_free =
      [&epoch, evaluation_time_s](
          const iap::RiskOccupancyDiagnostic& observed,
          const double hard_lifetime_s) {
        iap::LocalMapSupportQuery strict;
        strict.authority =
            iap::LocalMapSupportAuthority::STRICT_OBSERVATION;
        strict.observation_stamp_s = observed.cloud_stamp_s;
        strict.observation_age_s =
            evaluation_time_s - observed.cloud_stamp_s;
        if (!observed.available || !observed.observed ||
            observed.state != iap::RiskOccupancyState::OBSERVED_FREE ||
            observed.frame_id != epoch.frame_id ||
            observed.occupancy_generation != epoch.generation ||
            !std::isfinite(observed.cloud_stamp_s) ||
            !std::isfinite(epoch.cloud_stamp_s) ||
            std::abs(observed.cloud_stamp_s - epoch.cloud_stamp_s) > 1.0e-6 ||
            std::isnan(hard_lifetime_s) || hard_lifetime_s < 0.0) {
          strict.status = iap::LocalMapSupportStatus::FRAME_INVALID;
          return strict;
        }
        strict.status = strict.observation_age_s >= -1.0e-6 &&
                strict.observation_age_s <= hard_lifetime_s
            ? iap::LocalMapSupportStatus::MODEL_COMPLETE
            : iap::LocalMapSupportStatus::EXPIRED;
        return strict;
      };
  const auto strict_observation_lifetime = [&epoch]() {
    if (epoch.trusted_local_map_support &&
        epoch.trusted_local_map_support->valid()) {
      return epoch.trusted_local_map_support->valid_until_s -
          epoch.trusted_local_map_support->stamp_s;
    }
    // Preserve the legacy diagnostic-only contract when no trusted support
    // owns a configured lifetime. The typed vehicle-footprint exception
    // below always requires valid trusted support and cannot take this path.
    return std::numeric_limits<double>::infinity();
  };
  iap::LocalMapSupportQuery model;
  if (epoch.trusted_local_map_support) {
    model = epoch.trusted_local_map_support->query(
        position, evaluation_time_s, query_time_s);
  }
  if (epoch.local_evidence_snapshot) {
    const auto evidence = epoch.local_evidence_snapshot->queryVoxel(
        position, evaluation_time_s, epoch.generation,
        epoch.local_evidence_snapshot->identity().active_window_generation,
        epoch.frame_contract_id,
        epoch.local_evidence_snapshot->identity().sensor_model_identity);
    iap::LocalMapSupportQuery strict;
    strict.authority = iap::LocalMapSupportAuthority::STRICT_OBSERVATION;
    strict.observation_stamp_s = evidence.observation_timestamp_s;
    strict.observation_age_s = evidence.age_s;
    if (evidence.state != EvidenceVoxelState::UNKNOWN) {
      strict.status = iap::LocalMapSupportStatus::MODEL_COMPLETE;
      return strict;
    }
    if (evidence.reason == LocalEvidenceReason::STALE_OBSERVATION) {
      strict.status = iap::LocalMapSupportStatus::EXPIRED;
    } else if (evidence.reason == LocalEvidenceReason::OUT_OF_RANGE) {
      strict.status = iap::LocalMapSupportStatus::OUTSIDE_ENVELOPE;
    } else if (evidence.reason ==
                   LocalEvidenceReason::COORDINATE_CONTRACT_MISMATCH ||
               evidence.reason == LocalEvidenceReason::SENSOR_MODEL_MISMATCH ||
               evidence.reason == LocalEvidenceReason::GENERATION_MISMATCH) {
      strict.status = iap::LocalMapSupportStatus::FRAME_INVALID;
      return strict;
    } else {
      strict.status = iap::LocalMapSupportStatus::OBSERVATION_INCOMPLETE;
    }
    // A fresh trusted hit-only envelope is independent model support. It is
    // neither strict voxel evidence nor an OBSERVED_FREE claim, so preserve
    // its authority when the strict voxel is merely unknown. Identity
    // mismatches above remain fail-closed and can never fall back.
    if (model.complete())
      return model;
    // The sparse execution epoch deliberately records the vehicle's current
    // physical footprint as observed free before committing a new pose.  That
    // point can be inside the LiDAR minimum range, where neither the strict
    // voxel window nor the hit-only model can provide support.  Preserve only
    // this producer-owned typed footprint; the diagnostic source label is
    // not authority, and ordinary free space must continue to use the
    // immutable local-evidence authority above.
    if (epoch.diagnostic_query && epoch.trusted_local_map_support &&
        epoch.trusted_local_map_support->valid() && position.allFinite() &&
        std::isfinite(evaluation_time_s) && std::isfinite(query_time_s)) {
      const auto observed = epoch.diagnostic_query(position);
      const auto& trusted = *epoch.trusted_local_map_support;
      const Eigen::Vector3d point_sensor = trusted.T_map_sensor.linear()
          .transpose() * (position - trusted.T_map_sensor.translation());
      const bool inside_retained_envelope =
          (position.array() >= trusted.retained_min_map.array()).all() &&
          (position.array() <= trusted.retained_max_map.array()).all();
      const bool inside_minimum_range =
          point_sensor.allFinite() &&
          point_sensor.norm() < trusted.min_range_m;
      const bool inside_current_vehicle_footprint =
          epoch.current_vehicle_position.allFinite() &&
          std::isfinite(epoch.current_vehicle_clearance_radius_m) &&
          epoch.current_vehicle_clearance_radius_m >= 0.0 &&
          (position - epoch.current_vehicle_position).norm() <=
              epoch.current_vehicle_clearance_radius_m;
      const bool bound_to_current_observation =
          trusted.frame_id == epoch.frame_id &&
          std::abs(trusted.stamp_s - epoch.cloud_stamp_s) <= 1.0e-6;
      if (inside_current_vehicle_footprint && inside_retained_envelope &&
          inside_minimum_range && bound_to_current_observation) {
        return strict_observed_free(
            observed, strict_observation_lifetime());
      }
    }
    return strict;
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
  return strict_observed_free(observed, strict_observation_lifetime());
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
      std::optional<P0ReusableLosOccupancy> reusable_los = std::nullopt,
      std::string* failure_reason = nullptr) {
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
    auto adapted = adaptFields(epoch.raw_occupied_voxel_centers,
                       p0_occupancy_detail::rawVoxelKeys(epoch, 0),
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
                       p0_occupancy_detail::currentFrameCenters(epoch, 0),
                       failure_reason);
    if (adapted) {
      adapted->local_evidence_snapshot =
          p0_occupancy_detail::localEvidenceSnapshot(epoch, 0);
      adapted->current_vehicle_position =
          p0_occupancy_detail::currentVehiclePosition(epoch, 0);
      adapted->current_vehicle_clearance_radius_m =
          p0_occupancy_detail::currentVehicleClearanceRadius(epoch, 0);
    }
    return adapted;
  }

  static bool sameVersion(const P0OccupancyEpoch& base,
                          const P0OccupancyEpoch& target);
  static std::optional<P0RawOccupancyDelta> completeDelta(
      const P0OccupancyEpoch& base, const P0OccupancyEpoch& target);

 private:
  static std::optional<P0OccupancyEpoch> adaptFields(
      std::shared_ptr<const std::vector<Eigen::Vector3d>> occupied_centers,
      std::shared_ptr<const std::vector<iap::VoxelKey>> occupied_keys,
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
          current_frame_centers,
      std::string* failure_reason);
};

}  // namespace ego_planner

#endif
