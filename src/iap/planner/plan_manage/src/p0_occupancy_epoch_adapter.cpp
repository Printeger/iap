#include <ego_planner/p0_occupancy_epoch_adapter.h>

#include <algorithm>
#include <climits>
#include <cmath>
#include <cstring>
#include <iterator>
#include <limits>
#include <utility>

namespace ego_planner {
namespace {

bool keyLess(const iap::VoxelKey& lhs, const iap::VoxelKey& rhs) {
  if (lhs.x != rhs.x) return lhs.x < rhs.x;
  if (lhs.y != rhs.y) return lhs.y < rhs.y;
  return lhs.z < rhs.z;
}

bool exactDouble(const double lhs, const double rhs) {
  std::uint64_t lhs_bits = 0;
  std::uint64_t rhs_bits = 0;
  std::memcpy(&lhs_bits, &lhs, sizeof(lhs));
  std::memcpy(&rhs_bits, &rhs, sizeof(rhs));
  return lhs_bits == rhs_bits;
}

bool exactVector(const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
  return exactDouble(lhs.x(), rhs.x()) && exactDouble(lhs.y(), rhs.y()) &&
         exactDouble(lhs.z(), rhs.z());
}

bool sameOwner(const P0OccupancyEpoch::SourceOwner& lhs,
               const P0OccupancyEpoch::SourceOwner& rhs) {
  return lhs && rhs && !lhs.owner_before(rhs) && !rhs.owner_before(lhs);
}

bool sameIdentity(const P0RawOccupancyIdentity& lhs,
                  const P0RawOccupancyIdentity& rhs) {
  return lhs.frameId() == rhs.frameId() &&
         exactVector(lhs.latticeOrigin(), rhs.latticeOrigin()) &&
         exactDouble(lhs.resolutionM(), rhs.resolutionM()) &&
         lhs.keys() == rhs.keys();
}

bool coherentGeometry(const P0RawOccupancyIdentity& lhs,
                      const P0RawOccupancyIdentity& rhs) {
  return lhs.frameId() == rhs.frameId() &&
         exactVector(lhs.latticeOrigin(), rhs.latticeOrigin()) &&
         exactDouble(lhs.resolutionM(), rhs.resolutionM());
}

void extendBounds(const iap::VoxelKey& key,
                  P0RawOccupancyChangedBounds* bounds) {
  bounds->minimum.x = std::min(bounds->minimum.x, key.x);
  bounds->minimum.y = std::min(bounds->minimum.y, key.y);
  bounds->minimum.z = std::min(bounds->minimum.z, key.z);
  bounds->maximum.x = std::max(bounds->maximum.x, key.x);
  bounds->maximum.y = std::max(bounds->maximum.y, key.y);
  bounds->maximum.z = std::max(bounds->maximum.z, key.z);
}

}  // namespace

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
  std::vector<iap::VoxelKey> normalized_keys;
  normalized_keys.reserve(captured_count);
  for (const auto& center : *occupied_centers) {
    if (!center.allFinite()) {
      return std::nullopt;
    }
    iap::VoxelKey key{};
    for (int axis = 0; axis < 3; ++axis) {
      const double scaled =
          (center(axis) - lattice_origin(axis)) / resolution_m;
      const double floored = std::floor(scaled);
      if (!std::isfinite(floored) ||
          floored < static_cast<double>(std::numeric_limits<int>::min()) ||
          floored > static_cast<double>(std::numeric_limits<int>::max())) {
        return std::nullopt;
      }
      const double expected = lattice_origin(axis) +
          (floored + 0.5) * resolution_m;
      const double scale = std::max(
          {1.0, std::abs(center(axis)), std::abs(lattice_origin(axis)),
           std::abs(resolution_m)});
      const double tolerance =
          64.0 * std::numeric_limits<double>::epsilon() * scale;
      if (std::abs(center(axis) - expected) > tolerance) {
        return std::nullopt;
      }
      if (axis == 0) key.x = static_cast<int>(floored);
      if (axis == 1) key.y = static_cast<int>(floored);
      if (axis == 2) key.z = static_cast<int>(floored);
    }
    normalized_keys.push_back(key);
  }
  std::sort(normalized_keys.begin(), normalized_keys.end(), keyLess);
  if (std::adjacent_find(normalized_keys.begin(), normalized_keys.end()) !=
      normalized_keys.end()) {
    return std::nullopt;
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
  adapted.raw_identity = std::shared_ptr<const P0RawOccupancyIdentity>(
      new P0RawOccupancyIdentity(std::move(normalized_keys), lattice_origin,
                                 resolution_m, frame_id));
  adapted.source_owner = std::move(source_owner);
  adapted.live_source_owner = std::move(live_source_owner);
  adapted.live_generation = std::move(live_generation);
  adapted.generation = generation;
  adapted.cloud_stamp_s = cloud_stamp_s;
  adapted.frame_id = std::move(frame_id);
  return adapted;
}

bool P0OccupancyEpochAdapter::sameVersion(
    const P0OccupancyEpoch& base, const P0OccupancyEpoch& target) {
  return base.generation != 0u && base.generation == target.generation &&
         std::isfinite(base.cloud_stamp_s) &&
         exactDouble(base.cloud_stamp_s, target.cloud_stamp_s) &&
         sameOwner(base.source_owner, target.source_owner) &&
         base.raw_identity && target.raw_identity &&
         sameIdentity(*base.raw_identity, *target.raw_identity);
}

std::optional<P0RawOccupancyDelta>
P0OccupancyEpochAdapter::completeDelta(
    const P0OccupancyEpoch& base, const P0OccupancyEpoch& target) {
  if (base.generation == 0u || target.generation <= base.generation ||
      !std::isfinite(base.cloud_stamp_s) ||
      !std::isfinite(target.cloud_stamp_s) ||
      !sameOwner(base.source_owner, target.source_owner) ||
      !base.raw_identity || !target.raw_identity ||
      !coherentGeometry(*base.raw_identity, *target.raw_identity)) {
    return std::nullopt;
  }

  P0RawOccupancyDelta delta;
  delta.base_generation_ = base.generation;
  delta.target_generation_ = target.generation;
  const auto& base_keys = base.raw_identity->keys();
  const auto& target_keys = target.raw_identity->keys();
  std::set_difference(target_keys.begin(), target_keys.end(),
                      base_keys.begin(), base_keys.end(),
                      std::back_inserter(delta.added_keys_), keyLess);
  std::set_difference(base_keys.begin(), base_keys.end(),
                      target_keys.begin(), target_keys.end(),
                      std::back_inserter(delta.removed_keys_), keyLess);
  const auto add_changed_key = [&delta](const iap::VoxelKey& key) {
    if (!delta.changed_bounds_) {
      delta.changed_bounds_ = P0RawOccupancyChangedBounds{key, key};
    } else {
      extendBounds(key, &*delta.changed_bounds_);
    }
  };
  for (const auto& key : delta.added_keys_) add_changed_key(key);
  for (const auto& key : delta.removed_keys_) add_changed_key(key);
  return delta;
}

}  // namespace ego_planner
