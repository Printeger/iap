// IAP-RQ-311: LocalOccupancyGrid implementation
// Amanatides & Woo DDA for ray_occluded(); uniform sample for occupancy_ratio().

#include <iap/map/local_occupancy.hpp>
#include <algorithm>
#include <stdexcept>

namespace iap {

namespace {

// Exact squared Euclidean distance transform for a one-dimensional line.
// This is the lower-envelope algorithm from Felzenszwalb and Huttenlocher.
// Calling it along x/y/z makes clearance-field construction O(number of
// lattice cells), independent of transition radius and occupied-voxel count.
void squared_distance_transform_1d(const std::vector<float>& input,
                                   std::vector<float>* output) {
  const int n = static_cast<int>(input.size());
  output->assign(input.size(), std::numeric_limits<float>::infinity());
  if (n == 0) return;

  std::vector<int> finite_indices;
  finite_indices.reserve(input.size());
  for (int index = 0; index < n; ++index) {
    if (std::isfinite(input[static_cast<std::size_t>(index)])) {
      finite_indices.push_back(index);
    }
  }
  if (finite_indices.empty()) return;

  std::vector<int> sites(finite_indices.size());
  std::vector<double> boundaries(finite_indices.size() + 1u);
  int envelope_size = 0;
  sites[0] = finite_indices[0];
  boundaries[0] = -std::numeric_limits<double>::infinity();
  boundaries[1] = std::numeric_limits<double>::infinity();
  for (std::size_t candidate_index = 1;
       candidate_index < finite_indices.size(); ++candidate_index) {
    const int candidate = finite_indices[candidate_index];
    double crossing = 0.0;
    while (true) {
      const int active = sites[static_cast<std::size_t>(envelope_size)];
      crossing =
          (static_cast<double>(input[static_cast<std::size_t>(candidate)]) +
               static_cast<double>(candidate) * candidate -
           static_cast<double>(input[static_cast<std::size_t>(active)]) -
               static_cast<double>(active) * active) /
          (2.0 * static_cast<double>(candidate - active));
      if (crossing > boundaries[static_cast<std::size_t>(envelope_size)] ||
          envelope_size == 0) {
        break;
      }
      --envelope_size;
    }
    ++envelope_size;
    sites[static_cast<std::size_t>(envelope_size)] = candidate;
    boundaries[static_cast<std::size_t>(envelope_size)] = crossing;
    boundaries[static_cast<std::size_t>(envelope_size + 1)] =
        std::numeric_limits<double>::infinity();
  }

  int active_index = 0;
  for (int query = 0; query < n; ++query) {
    while (active_index < envelope_size &&
           boundaries[static_cast<std::size_t>(active_index + 1)] < query) {
      ++active_index;
    }
    const int active = sites[static_cast<std::size_t>(active_index)];
    const double delta = static_cast<double>(query - active);
    (*output)[static_cast<std::size_t>(query)] = static_cast<float>(
        delta * delta + input[static_cast<std::size_t>(active)]);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
LocalOccupancyGrid::LocalOccupancyGrid() : params_(Params{}) {}
LocalOccupancyGrid::LocalOccupancyGrid(const Params& p) : params_(p) {
  if (!std::isfinite(params_.voxel_size) || params_.voxel_size <= 0.0 ||
      !std::isfinite(params_.clearance_transition_m) ||
      params_.clearance_transition_m < 0.0 ||
      params_.clearance_transition_m > kMaxClearanceTransitionM ||
      params_.clearance_transition_m / params_.voxel_size >
          kMaxClearanceTransitionRadiusVoxels) {
    throw std::invalid_argument("invalid local occupancy clearance geometry");
  }
}

// ---------------------------------------------------------------------------
VoxelKey LocalOccupancyGrid::to_key(const Eigen::Vector3d& p) const {
  const double inv = 1.0 / params_.voxel_size;
  const Eigen::Vector3d local = p - params_.lattice_origin;
  return {
      static_cast<int>(std::floor(local.x() * inv)),
      static_cast<int>(std::floor(local.y() * inv)),
      static_cast<int>(std::floor(local.z() * inv))
  };
}

Eigen::Vector3d LocalOccupancyGrid::key_center(const VoxelKey& k) const {
  const double vs = params_.voxel_size;
  return params_.lattice_origin +
      Eigen::Vector3d((static_cast<double>(k.x) + 0.5) * vs,
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
  rebuild_clearance_field();
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
    evict_around_impl(center_world, stamp_s);
  }
  for (const auto& pw : points_world) {
    if (!insert_voxel(pw, center_world, stamp_s) &&
        !params_.enable_eviction) {
      break;
    }
  }
  rebuild_clearance_field();
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
    evict_around_impl(center_world, stamp_s);
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
  const std::size_t evicted = evict_around_impl(center_world, now_s);
  if (evicted > 0u) rebuild_clearance_field();
  return evicted;
}

std::size_t LocalOccupancyGrid::evict_around_impl(
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
  clearance_field_dense_.clear();
  clearance_field_dims_.setZero();
}

void LocalOccupancyGrid::rebuild_clearance_field() {
  clearance_field_dense_.clear();
  clearance_field_dims_.setZero();
  const double width = params_.clearance_transition_m;
  const double vs = params_.voxel_size;
  if (!(std::isfinite(width) && width > 0.0 && std::isfinite(vs) &&
        vs > 0.0) || voxels_.empty()) {
    return;
  }
  const int radius = std::max(1, static_cast<int>(std::ceil(width / vs)) + 1);
  VoxelKey occupied_min = voxels_.begin()->first;
  VoxelKey occupied_max = occupied_min;
  for (const auto& occupied : voxels_) {
    occupied_min.x = std::min(occupied_min.x, occupied.first.x);
    occupied_min.y = std::min(occupied_min.y, occupied.first.y);
    occupied_min.z = std::min(occupied_min.z, occupied.first.z);
    occupied_max.x = std::max(occupied_max.x, occupied.first.x);
    occupied_max.y = std::max(occupied_max.y, occupied.first.y);
    occupied_max.z = std::max(occupied_max.z, occupied.first.z);
  }
  clearance_field_min_ = {occupied_min.x - radius,
                          occupied_min.y - radius,
                          occupied_min.z - radius};
  const Eigen::Array3<int64_t> dimensions64(
      static_cast<int64_t>(occupied_max.x) - occupied_min.x + 2 + 2 * radius,
      static_cast<int64_t>(occupied_max.y) - occupied_min.y + 2 + 2 * radius,
      static_cast<int64_t>(occupied_max.z) - occupied_min.z + 2 + 2 * radius);
  constexpr std::size_t kMaxDenseClearanceCells = 16u * 1024u * 1024u;
  std::size_t cell_count = 0u;
  if (dimensions64.minCoeff() > 0 &&
      dimensions64.maxCoeff() <= std::numeric_limits<int>::max()) {
    const std::size_t xy = static_cast<std::size_t>(dimensions64.x()) *
        static_cast<std::size_t>(dimensions64.y());
    if (xy <= kMaxDenseClearanceCells &&
        static_cast<std::size_t>(dimensions64.z()) <=
            kMaxDenseClearanceCells / xy) {
      cell_count = xy * static_cast<std::size_t>(dimensions64.z());
    }
  }
  const bool use_dense = cell_count > 0u &&
      cell_count <= kMaxDenseClearanceCells &&
      dimensions64.maxCoeff() <= kMaxClearanceDenseAxisCells;
  if (use_dense) {
    const Eigen::Vector3i dimensions = dimensions64.cast<int>();
    clearance_field_dims_ = dimensions;
    clearance_field_dense_.assign(
        cell_count, std::numeric_limits<float>::infinity());
  } else {
    // Avoid O(N*radius^3) construction and unbounded sparse memory. Queries
    // fail closed in constant time in this rare representation.
    return;
  }
  // Seed all eight lattice vertices of each occupied AABB. A lattice vertex's
  // nearest point on an axis-aligned voxel is itself a lattice vertex, so the
  // separable EDT below is exact at every stored sample.
  for (const auto& occupied : voxels_) {
    for (int dx = 0; dx <= 1; ++dx) {
      for (int dy = 0; dy <= 1; ++dy) {
        for (int dz = 0; dz <= 1; ++dz) {
          const VoxelKey key{occupied.first.x + dx,
                             occupied.first.y + dy,
                             occupied.first.z + dz};
          const int x = key.x - clearance_field_min_.x;
          const int y = key.y - clearance_field_min_.y;
          const int z = key.z - clearance_field_min_.z;
          const std::size_t index =
              (static_cast<std::size_t>(z) *
                   static_cast<std::size_t>(clearance_field_dims_.y()) +
               static_cast<std::size_t>(y)) *
                  static_cast<std::size_t>(clearance_field_dims_.x()) +
              static_cast<std::size_t>(x);
          clearance_field_dense_[index] = 0.0f;
        }
      }
    }
  }

  const int nx = clearance_field_dims_.x();
  const int ny = clearance_field_dims_.y();
  const int nz = clearance_field_dims_.z();
  std::vector<float> line;
  std::vector<float> transformed;
  const auto index_of = [nx, ny](const int x, const int y, const int z) {
      return (static_cast<std::size_t>(z) * static_cast<std::size_t>(ny) +
              static_cast<std::size_t>(y)) * static_cast<std::size_t>(nx) +
          static_cast<std::size_t>(x);
    };
  const auto transform_line = [&](const int length, const auto read,
                                  const auto write) {
      line.resize(static_cast<std::size_t>(length));
      for (int i = 0; i < length; ++i) {
        line[static_cast<std::size_t>(i)] = read(i);
      }
      squared_distance_transform_1d(line, &transformed);
      for (int i = 0; i < length; ++i) {
        write(i, transformed[static_cast<std::size_t>(i)]);
      }
    };
  for (int z = 0; z < nz; ++z)
    for (int y = 0; y < ny; ++y)
      transform_line(nx,
          [&](const int x) {
            return clearance_field_dense_[index_of(x, y, z)];
          },
          [&](const int x, const float value) {
            clearance_field_dense_[index_of(x, y, z)] = value;
          });
  for (int z = 0; z < nz; ++z)
    for (int x = 0; x < nx; ++x)
      transform_line(ny,
          [&](const int y) {
            return clearance_field_dense_[index_of(x, y, z)];
          },
          [&](const int y, const float value) {
            clearance_field_dense_[index_of(x, y, z)] = value;
          });
  for (int y = 0; y < ny; ++y)
    for (int x = 0; x < nx; ++x)
      transform_line(nz,
          [&](const int z) {
            return clearance_field_dense_[index_of(x, y, z)];
          },
          [&](const int z, const float value) {
            clearance_field_dense_[index_of(x, y, z)] = static_cast<float>(
                std::min(width, vs * std::sqrt(static_cast<double>(value))));
          });
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
  const Eigen::Vector3d local_origin = origin - params_.lattice_origin;
  int cx = static_cast<int>(std::floor(local_origin.x() * inv));
  int cy = static_cast<int>(std::floor(local_origin.y() * inv));
  int cz = static_cast<int>(std::floor(local_origin.z() * inv));

  // Step direction
  const int sx = (dir_unit.x() >= 0.0) ? 1 : -1;
  const int sy = (dir_unit.y() >= 0.0) ? 1 : -1;
  const int sz = (dir_unit.z() >= 0.0) ? 1 : -1;

  // t at which we cross the next voxel boundary in each axis
  auto t_boundary = [&](double o, double d, int c,
                        double lattice_origin) -> double {
    if (std::abs(d) < 1e-12) return 1e30;
    const double boundary = lattice_origin +
        ((d > 0) ? (c + 1) * vs : c * vs);
    return (boundary - o) / d;
  };

  double tx = t_boundary(origin.x(), dir_unit.x(), cx,
                         params_.lattice_origin.x());
  double ty = t_boundary(origin.y(), dir_unit.y(), cy,
                         params_.lattice_origin.y());
  double tz = t_boundary(origin.z(), dir_unit.z(), cz,
                         params_.lattice_origin.z());

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

double LocalOccupancyGrid::clearance_to_occupied(
    const Eigen::Vector3d& p_world) const {
  const double width = params_.clearance_transition_m;
  const double vs = params_.voxel_size;
  if (!p_world.allFinite() || !(std::isfinite(width) && width > 0.0) ||
      !(std::isfinite(vs) && vs > 0.0) ||
      voxels_.empty()) {
    return std::isfinite(width) && width > 0.0 ? width :
        std::numeric_limits<double>::infinity();
  }
  const Eigen::Vector3d cell =
      (p_world - params_.lattice_origin) / vs;
  const Eigen::Vector3i base = cell.array().floor().cast<int>();
  if (clearance_field_dense_.empty()) {
    // A pathological coordinate extent cannot safely allocate the bounded
    // dense field. Fail closed in constant time instead of performing an
    // unbounded radius^3 lookup in every LOS sample.
    return 0.0;
  }
  const Eigen::Vector3d frac = cell - base.cast<double>();
  double clearance = 0.0;
  for (int dx = 0; dx <= 1; ++dx) {
    for (int dy = 0; dy <= 1; ++dy) {
      for (int dz = 0; dz <= 1; ++dz) {
        const VoxelKey key{base.x() + dx, base.y() + dy, base.z() + dz};
        double value = width;
        const int x = key.x - clearance_field_min_.x;
        const int y = key.y - clearance_field_min_.y;
        const int z = key.z - clearance_field_min_.z;
        if (x >= 0 && y >= 0 && z >= 0 &&
            x < clearance_field_dims_.x() &&
            y < clearance_field_dims_.y() &&
            z < clearance_field_dims_.z()) {
          const std::size_t index =
              (static_cast<std::size_t>(z) *
                   static_cast<std::size_t>(clearance_field_dims_.y()) +
               static_cast<std::size_t>(y)) *
                  static_cast<std::size_t>(clearance_field_dims_.x()) +
              static_cast<std::size_t>(x);
          value = clearance_field_dense_[index];
        }
        const double weight = (dx ? frac.x() : 1.0 - frac.x()) *
                              (dy ? frac.y() : 1.0 - frac.y()) *
                              (dz ? frac.z() : 1.0 - frac.z());
        clearance += weight * value;
      }
    }
  }
  return std::clamp(clearance, 0.0, width);
}

double LocalOccupancyGrid::clearance_proximity_ratio(
    const Eigen::Vector3d& origin, const Eigen::Vector3d& dir_unit,
    const double L) const {
  const double width = params_.clearance_transition_m;
  if (!(std::isfinite(width) && width > 0.0) || voxels_.empty() ||
      params_.n_kappa_steps <= 0 || !origin.allFinite() ||
      !dir_unit.allFinite() || !(std::isfinite(L) && L > 0.0)) {
    return 0.0;
  }
  const double dt = L / static_cast<double>(params_.n_kappa_steps);
  double proximity_sum = 0.0;
  for (int i = 0; i < params_.n_kappa_steps; ++i) {
    const Eigen::Vector3d p =
        origin + (static_cast<double>(i) + 0.5) * dt * dir_unit;
    const double x = std::clamp(clearance_to_occupied(p) / width, 0.0, 1.0);
    const double smoothstep = x * x * (3.0 - 2.0 * x);
    proximity_sum += 1.0 - smoothstep;
  }
  return std::clamp(
      proximity_sum / static_cast<double>(params_.n_kappa_steps), 0.0, 1.0);
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
