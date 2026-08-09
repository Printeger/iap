#pragma once

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace iap::planner {

struct P1FixturePoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

struct P1FixtureConfig {
  std::string name;
  bool mirror_y = false;
  bool central_obstacle_enabled = true;
  double central_x_min_m = -8.0;
  double central_x_max_m = -3.0;
  double central_y_half_width_m = 0.65;
  double central_z_max_m = 2.8;
  double lane_center_m = 2.0;
  double lane_half_width_m = 0.75;
  double safe_tree_density_per_m2 = 0.25;
  double risky_tree_density_per_m2 = 0.75;
  double safe_canopy_probability = 0.05;
  double risky_canopy_probability = 0.85;
  double trunk_radius_m = 0.14;
  double resolution_m = 0.10;
  double canopy_resolution_m = 0.15;
};

inline void append_p1_box(std::vector<P1FixturePoint>& points,
                          double x_min, double x_max, double y_min, double y_max,
                          double z_min, double z_max, double resolution) {
  const double res = std::max(0.05, resolution);
  for (double x = std::min(x_min, x_max); x <= std::max(x_min, x_max) + 1e-9; x += res)
    for (double y = std::min(y_min, y_max); y <= std::max(y_min, y_max) + 1e-9; y += res)
      for (double z = std::min(z_min, z_max); z <= std::max(z_min, z_max) + 1e-9; z += res)
        points.push_back({x, y, z});
}

inline void append_p1_cylinder(std::vector<P1FixturePoint>& points, double cx,
                               double cy, double radius, double z_max,
                               double resolution) {
  const double res = std::max(0.05, resolution);
  for (double x = cx - radius; x <= cx + radius + 1e-9; x += res)
    for (double y = cy - radius; y <= cy + radius + 1e-9; y += res) {
      const double dx = x - cx, dy = y - cy;
      if (dx * dx + dy * dy > radius * radius) continue;
      for (double z = 0.0; z <= z_max + 1e-9; z += res) points.push_back({x, y, z});
    }
}

inline void append_p1_canopy(std::vector<P1FixturePoint>& points, double cx,
                             double cy, double resolution) {
  const double res = std::max(0.05, resolution);
  for (double x = cx - 0.42; x <= cx + 0.42 + 1e-9; x += res)
    for (double y = cy - 0.42; y <= cy + 0.42 + 1e-9; y += res)
      for (double z = 2.83; z <= 3.67 + 1e-9; z += res) {
        const double sx = x - cx, sy = y - cy, sz = z - 3.25;
        const double hz = z - 2.85;
        if (sx * sx + sy * sy + sz * sz <= 0.42 * 0.42 && hz >= 0.0 &&
            sx * sx + sy * sy + hz * hz <= 1.10 * 1.10)
          points.push_back({x, y, z});
      }
}

inline void append_p1_observability_landmarks(
    std::vector<P1FixturePoint>& points, double resolution) {
  // Symmetric survey pylons outside both flight lanes add longitudinal and
  // vertical LiDAR structure without changing tree density/canopy parameters
  // or obstructing either fork homotopy.
  for (const double x : {-12.0, -10.0, -8.0, -6.0, -4.0, -2.0, 0.0}) {
    const std::size_t first = points.size();
    append_p1_box(points, x - 0.25, x + 0.25, -4.75, -4.25,
                  0.0, 3.0, resolution);
    const std::size_t last = points.size();
    for (std::size_t index = first; index < last; ++index)
      points.push_back({points[index].x, -points[index].y, points[index].z});
  }
}

inline void append_p1_startup_localization_beacons(
    std::vector<P1FixturePoint>& points, double resolution) {
  // A short, symmetric pair of staggered structures is visible in the
  // forward LiDAR field while the vehicle is stationary at x=-12 m.  The
  // structures remain outside both formal lanes and are behind the vehicle by
  // the risk checkpoint, so they improve initialization without changing the
  // registered fork comparison.
  for (const double x : {-11.25, -10.75}) {
    const std::size_t first = points.size();
    append_p1_box(points, x - 0.25, x + 0.25, -4.75, -4.25,
                  0.0, 3.0, resolution);
    const std::size_t last = points.size();
    for (std::size_t index = first; index < last; ++index)
      points.push_back({points[index].x, -points[index].y, points[index].z});
  }
}

inline void append_p1_overhead_observability_rafters(
    std::vector<P1FixturePoint>& points, double center_y, double resolution) {
  // Compact structured returns above the flight layer strengthen the real
  // LiDAR geometry without occupying either homotopy or encoding a risk
  // value. Counts and dimensions of the preregistered trees/canopies are not
  // changed.
  for (const double x : {-10.0, -8.0, -6.0, -4.0, -2.0})
    append_p1_box(points, x - 0.30, x + 0.30,
                  center_y - 0.30, center_y + 0.30,
                  2.85, 3.35, resolution);
}

inline void append_p1_overhead_gnss_mask(
    std::vector<P1FixturePoint>& points, double center_y, double resolution) {
  // The GNSS simulator consumes the global map while the simulated forward
  // LiDAR rejects points whose vertical displacement exceeds
  // sensing_horizon*tan(30 deg).  At the formal 1.5 m flight altitude and
  // 10 m horizon, z >= 7.30 m is therefore a real GNSS-only part of the same
  // physical scene.  Keeping the continuous mask there prevents its dense
  // returns from improving LiDAR observability and inverting fused risk.
  append_p1_box(points, -11.5, 2.5, center_y - 1.25, center_y + 1.25,
                7.30, 7.55, resolution);
}

inline std::vector<P1FixturePoint> make_p1_fixture_points(const P1FixtureConfig& config) {
  std::vector<P1FixturePoint> base;
  const auto add_lane = [&](double center_y, double density,
                            double canopy_probability, bool short_features,
                            double canopy_center_y) {
    const int count = std::max(2, static_cast<int>(std::llround(32.0 * density)));
    const int canopy_count = static_cast<int>(std::llround(count * canopy_probability));
    for (int index = 0; index < count; ++index) {
      const bool external = index % 2 == 0;
      const int inner_count = count / 2;
      const double fraction = (static_cast<double>(index) + 0.5) / count;
      double x = -7.8 + 16.0 * fraction;
      const double external_sign = center_y < 0.0 ? -1.0 : 1.0;
      double y = center_y +
          external_sign * (config.lane_half_width_m + 1.25);
      if (!external) {
        // The denser primary boundary must remain observable to the ordinary
        // (metrics-only) planner, but inner trunks must never obstruct the
        // split or merge. Keep them alongside the already occupied central
        // box and put their nearest surface exactly 1.70 m from either formal
        // lane centre. Primary density can then break the base-path tie while
        // mirror/null retain the same geometric construction.
        const int inner_index = index / 2;
        const double inner_fraction =
            (static_cast<double>(inner_index) + 0.5) / inner_count;
        x = config.central_x_min_m + config.trunk_radius_m +
            (config.central_x_max_m - config.central_x_min_m -
             2.0 * config.trunk_radius_m) * inner_fraction;
        const double inner_surface_abs = config.central_y_half_width_m +
            0.20 * std::max(0.0, config.lane_center_m - 2.0);
        y = std::copysign(
            inner_surface_abs - config.trunk_radius_m, center_y);
      }
      append_p1_cylinder(base, x, y, config.trunk_radius_m,
                         short_features ? 0.55 : 2.85, config.resolution_m);
      if (index < canopy_count) {
        // Keep the preregistered tree/canopy counts and dimensions, but place
        // crowns at the caller-declared LOS location so real ray-casting sees
        // the intended GNSS asymmetry. The crown remains wholly above the
        // flight layer; trunks retain their conservative lateral clearance.
        const double canopy_y = short_features ? y : canopy_center_y;
        append_p1_canopy(base, x, canopy_y, config.canopy_resolution_m);
      }
    }
  };
  if (config.central_obstacle_enabled)
    append_p1_box(base, config.central_x_min_m, config.central_x_max_m,
                  -config.central_y_half_width_m, config.central_y_half_width_m,
                  0.0, config.central_z_max_m, config.resolution_m);
  if (config.name == "p1_fork_symmetric_null_v1") {
    add_lane(-config.lane_center_m, config.safe_tree_density_per_m2,
             config.safe_canopy_probability, true, -config.lane_center_m);
    const auto lower = base;
    for (const auto& point : lower) base.push_back({point.x, -point.y, point.z});
  } else if (config.name == "p1_soft_risk_island_v1") {
    const int count = std::max(4, static_cast<int>(std::llround(
        24.0 * config.risky_tree_density_per_m2)));
    for (int index = 0; index < count; ++index) {
      const double fraction = (static_cast<double>(index) + 0.5) / count;
      append_p1_canopy(base, -6.0 + 8.0 * fraction,
                       -2.0,
                       config.canopy_resolution_m);
    }
    append_p1_overhead_observability_rafters(
        base, -config.lane_center_m, config.resolution_m);
  } else {
    // Dense trunks retain LiDAR structure on the preferred lower route. The
    // unchanged risky-crown count is placed over the canonical reference arm
    // so physical GNSS LOS obstruction supplies the spatial contrast; exact
    // scene mirror reflection swaps both roles.
    add_lane(-config.lane_center_m, config.risky_tree_density_per_m2,
             config.risky_canopy_probability, false, config.lane_center_m);
    add_lane(config.lane_center_m, config.safe_tree_density_per_m2,
             config.safe_canopy_probability, true, config.lane_center_m);
    append_p1_overhead_gnss_mask(
        base, config.lane_center_m, config.resolution_m);
  }
  append_p1_observability_landmarks(base, config.resolution_m);
  append_p1_startup_localization_beacons(base, config.resolution_m);
  if (config.mirror_y)
    for (auto& point : base) point.y = -point.y;
  return base;
}

}  // namespace iap::planner
