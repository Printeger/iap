// IAP-RQ-311: LocalOccupancyGrid implementation
// Amanatides & Woo DDA for ray_occluded(); uniform sample for occupancy_ratio().

#include <iap/map/local_occupancy.hpp>
#include <algorithm>

namespace iap {

// ---------------------------------------------------------------------------
LocalOccupancyGrid::LocalOccupancyGrid() : params_(Params{}) {}
LocalOccupancyGrid::LocalOccupancyGrid(const Params& p) : params_(p) {}

// ---------------------------------------------------------------------------
VoxelKey LocalOccupancyGrid::to_key(const Eigen::Vector3d& p) const {
  const double inv = 1.0 / params_.voxel_size;
  return {
      static_cast<int>(std::floor(p.x() * inv)),
      static_cast<int>(std::floor(p.y() * inv)),
      static_cast<int>(std::floor(p.z() * inv))
  };
}

Eigen::Vector3d LocalOccupancyGrid::key_center(const VoxelKey& k) const {
  const double vs = params_.voxel_size;
  return Eigen::Vector3d((static_cast<double>(k.x) + 0.5) * vs,
                         (static_cast<double>(k.y) + 0.5) * vs,
                         (static_cast<double>(k.z) + 0.5) * vs);
}

bool LocalOccupancyGrid::is_occupied(const VoxelKey& k) const {
  return voxels_.count(k) != 0;
}

LocalOccupancyGrid::EvictionPolicy
LocalOccupancyGrid::eviction_policy_from_string(const std::string& policy) {
  if (policy == "distance") {
    return EvictionPolicy::DISTANCE;
  }
  if (policy == "age") {
    return EvictionPolicy::AGE;
  }
  return EvictionPolicy::DISTANCE_THEN_AGE;
}

std::string LocalOccupancyGrid::eviction_policy_to_string(
    EvictionPolicy policy) {
  switch (policy) {
    case EvictionPolicy::DISTANCE:
      return "distance";
    case EvictionPolicy::AGE:
      return "age";
    case EvictionPolicy::DISTANCE_THEN_AGE:
      return "distance_then_age";
  }
  return "distance_then_age";
}

// ---------------------------------------------------------------------------
void LocalOccupancyGrid::insert(const gtsam_points::PointCloud& cloud,
                                const Eigen::Isometry3d& T_world_sensor) {
  insert(cloud, T_world_sensor,
         Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()),
         std::numeric_limits<double>::quiet_NaN());
}

void LocalOccupancyGrid::insert(const gtsam_points::PointCloud& cloud,
                                const Eigen::Isometry3d& T_world_sensor,
                                const Eigen::Vector3d& center_world,
                                double stamp_s) {
  if (!cloud.points) return;
  for (int i = 0; i < cloud.size(); ++i) {
    const Eigen::Vector3d pw = T_world_sensor *
        cloud.points[i].head<3>();  // points are Eigen::Vector4d
    if (!insert_voxel(pw, center_world, stamp_s) &&
        !params_.enable_eviction) {
      break;
    }
  }
}

void LocalOccupancyGrid::insert_points(
    const std::vector<Eigen::Vector3d>& points_world) {
  insert_points(points_world,
                Eigen::Vector3d::Constant(
                    std::numeric_limits<double>::quiet_NaN()),
                std::numeric_limits<double>::quiet_NaN());
}

void LocalOccupancyGrid::insert_points(
    const std::vector<Eigen::Vector3d>& points_world,
    const Eigen::Vector3d& center_world,
    double stamp_s) {
  if (params_.enable_eviction) {
    evict_around(center_world, stamp_s);
  }
  for (const auto& pw : points_world) {
    if (!insert_voxel(pw, center_world, stamp_s) &&
        !params_.enable_eviction) {
      break;
    }
  }
}

bool LocalOccupancyGrid::insert_voxel(const Eigen::Vector3d& p_world,
                                      const Eigen::Vector3d& center_world,
                                      double stamp_s) {
  if (!p_world.allFinite()) {
    return true;
  }
  const VoxelKey key = to_key(p_world);
  auto existing = voxels_.find(key);
  if (existing != voxels_.end()) {
    existing->second.occupied = 1u;
    if (std::isfinite(stamp_s)) {
      existing->second.stamp_s = stamp_s;
    }
    return true;
  }

  if (params_.max_voxels <= 0) {
    ++diagnostics_.rejected_count;
    return false;
  }

  const bool has_center = center_world.allFinite();
  if (params_.enable_eviction && has_center &&
      std::isfinite(params_.local_radius_m) && params_.local_radius_m > 0.0 &&
      (p_world - center_world).norm() > params_.local_radius_m) {
    ++diagnostics_.rejected_count;
    return false;
  }

  if (params_.enable_eviction &&
      static_cast<int>(voxels_.size()) >= params_.max_voxels) {
    evict_around(center_world, stamp_s);
    if (static_cast<int>(voxels_.size()) >= params_.max_voxels) {
      diagnostics_.evicted_count +=
          evict_to_capacity(center_world,
                            static_cast<std::size_t>(params_.max_voxels - 1));
    }
  }

  if (static_cast<int>(voxels_.size()) >= params_.max_voxels) {
    ++diagnostics_.rejected_count;
    return false;
  }

  VoxelRecord record;
  record.occupied = 1u;
  record.stamp_s = stamp_s;
  record.sequence = next_sequence_++;
  voxels_.emplace(key, record);
  ++diagnostics_.inserted_count;
  return true;
}

std::size_t LocalOccupancyGrid::evict_around(
    const Eigen::Vector3d& center_world,
    double now_s) {
  if (!params_.enable_eviction) {
    return 0;
  }

  std::size_t evicted = 0;
  const bool has_center = center_world.allFinite();
  const bool use_radius = has_center && std::isfinite(params_.local_radius_m) &&
                          params_.local_radius_m > 0.0;
  const double radius2 = params_.local_radius_m * params_.local_radius_m;
  const bool use_age = std::isfinite(now_s) && std::isfinite(params_.max_age_s) &&
                       params_.max_age_s > 0.0;

  for (auto it = voxels_.begin(); it != voxels_.end();) {
    bool erase = false;
    if (use_radius) {
      const Eigen::Vector3d pc = key_center(it->first);
      erase = (pc - center_world).squaredNorm() > radius2;
    }
    if (!erase && use_age && std::isfinite(it->second.stamp_s)) {
      erase = (now_s - it->second.stamp_s) > params_.max_age_s;
    }
    if (erase) {
      it = voxels_.erase(it);
      ++evicted;
    } else {
      ++it;
    }
  }

  if (static_cast<int>(voxels_.size()) > params_.max_voxels) {
    evicted += evict_to_capacity(
        center_world, static_cast<std::size_t>(params_.max_voxels));
  }
  diagnostics_.evicted_count += evicted;
  return evicted;
}

std::size_t LocalOccupancyGrid::evict_to_capacity(
    const Eigen::Vector3d& center_world,
    std::size_t target_size) {
  if (params_.max_voxels <= 0) {
    const std::size_t removed = voxels_.size();
    voxels_.clear();
    return removed;
  }
  if (voxels_.size() <= target_size) {
    return 0;
  }

  const bool has_center = center_world.allFinite();
  auto score_distance2 = [&](const VoxelKey& key) {
    if (!has_center) {
      return -std::numeric_limits<double>::infinity();
    }
    return (key_center(key) - center_world).squaredNorm();
  };

  auto worse = [&](const auto& a, const auto& b) {
    const double da = score_distance2(a.first);
    const double db = score_distance2(b.first);
    const std::uint64_t sa = a.second.sequence;
    const std::uint64_t sb = b.second.sequence;
    switch (params_.eviction_policy) {
      case EvictionPolicy::DISTANCE:
        if (da != db) return da < db;
        return sa > sb;
      case EvictionPolicy::AGE:
        return sa > sb;
      case EvictionPolicy::DISTANCE_THEN_AGE:
        if (da != db) return da < db;
        return sa > sb;
    }
    return sa > sb;
  };

  std::size_t evicted = 0;
  while (voxels_.size() > target_size && !voxels_.empty()) {
    auto victim = std::max_element(voxels_.begin(), voxels_.end(), worse);
    if (victim == voxels_.end()) {
      break;
    }
    voxels_.erase(victim);
    ++evicted;
  }
  return evicted;
}

// ---------------------------------------------------------------------------
void LocalOccupancyGrid::reset() {
  voxels_.clear();
}

// ---------------------------------------------------------------------------
// Amanatides & Woo DDA ray traversal.
bool LocalOccupancyGrid::ray_occluded(const Eigen::Vector3d& origin,
                                      const Eigen::Vector3d& dir_unit,
                                      double max_range) const {
  if (voxels_.empty()) return false;

  const double vs = params_.voxel_size;
  const double inv = 1.0 / vs;

  // Current voxel index
  int cx = static_cast<int>(std::floor(origin.x() * inv));
  int cy = static_cast<int>(std::floor(origin.y() * inv));
  int cz = static_cast<int>(std::floor(origin.z() * inv));

  // Step direction
  const int sx = (dir_unit.x() >= 0.0) ? 1 : -1;
  const int sy = (dir_unit.y() >= 0.0) ? 1 : -1;
  const int sz = (dir_unit.z() >= 0.0) ? 1 : -1;

  // t at which we cross the next voxel boundary in each axis
  auto t_boundary = [&](double o, double d, int c) -> double {
    if (std::abs(d) < 1e-12) return 1e30;
    const double boundary = (d > 0) ? (c + 1) * vs : c * vs;
    return (boundary - o) / d;
  };

  double tx = t_boundary(origin.x(), dir_unit.x(), cx);
  double ty = t_boundary(origin.y(), dir_unit.y(), cy);
  double tz = t_boundary(origin.z(), dir_unit.z(), cz);

  // Delta t to cross one voxel in each axis
  const double dtx = (std::abs(dir_unit.x()) < 1e-12) ? 1e30 : vs / std::abs(dir_unit.x());
  const double dty = (std::abs(dir_unit.y()) < 1e-12) ? 1e30 : vs / std::abs(dir_unit.y());
  const double dtz = (std::abs(dir_unit.z()) < 1e-12) ? 1e30 : vs / std::abs(dir_unit.z());

  double t = 0.0;
  while (t < max_range) {
    if (is_occupied({cx, cy, cz})) return true;

    // Advance to next voxel boundary
    if (tx <= ty && tx <= tz) {
      t = tx; tx += dtx; cx += sx;
    } else if (ty <= tz) {
      t = ty; ty += dty; cy += sy;
    } else {
      t = tz; tz += dtz; cz += sz;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
double LocalOccupancyGrid::occupancy_ratio(const Eigen::Vector3d& origin,
                                           const Eigen::Vector3d& dir_unit,
                                           double L) const {
  if (voxels_.empty() || params_.n_kappa_steps <= 0) return 0.0;

  const double dt = L / static_cast<double>(params_.n_kappa_steps);
  int occupied = 0;
  for (int i = 0; i < params_.n_kappa_steps; ++i) {
    const double t = (i + 0.5) * dt;
    const Eigen::Vector3d p = origin + t * dir_unit;
    if (is_occupied(to_key(p))) ++occupied;
  }
  return static_cast<double>(occupied) / static_cast<double>(params_.n_kappa_steps);
}

// ---------------------------------------------------------------------------
bool LocalOccupancyGrid::occupied_at(const Eigen::Vector3d& p_world) const {
  if (!p_world.allFinite()) {
    return false;
  }
  return is_occupied(to_key(p_world));
}

// ---------------------------------------------------------------------------
double LocalOccupancyGrid::occupancy_probability(
    const Eigen::Vector3d& p_world) const {
  return occupied_at(p_world) ? 1.0 : 0.0;
}

LocalOccupancyGrid::Diagnostics LocalOccupancyGrid::diagnostics() const {
  Diagnostics out = diagnostics_;
  out.voxel_count = voxels_.size();
  return out;
}

}  // namespace iap
