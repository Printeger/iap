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

bool LocalOccupancyGrid::is_occupied(const VoxelKey& k) const {
  return voxels_.count(k) != 0;
}

// ---------------------------------------------------------------------------
void LocalOccupancyGrid::insert(const gtsam_points::PointCloud& cloud,
                                const Eigen::Isometry3d& T_world_sensor) {
  if (!cloud.points) return;
  for (int i = 0; i < cloud.size(); ++i) {
    const Eigen::Vector3d pw = T_world_sensor *
        cloud.points[i].head<3>();  // points are Eigen::Vector4d
    // Evict oldest (simple guard: just stop if full — sufficient for rolling window)
    if (static_cast<int>(voxels_.size()) >= params_.max_voxels) break;
    voxels_[to_key(pw)] = 1u;
  }
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

}  // namespace iap
