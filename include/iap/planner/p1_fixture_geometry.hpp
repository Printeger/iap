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
  double central_x_min_m = -7.0;
  double central_x_max_m = -2.0;
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

inline std::vector<P1FixturePoint> make_p1_fixture_points(const P1FixtureConfig& config) {
  std::vector<P1FixturePoint> base;
  const auto add_lane = [&](double center_y, double density,
                            double canopy_probability, bool short_features) {
    const int count = std::max(2, static_cast<int>(std::llround(32.0 * density)));
    const int canopy_count = static_cast<int>(std::llround(count * canopy_probability));
    for (int index = 0; index < count; ++index) {
      const double fraction = (static_cast<double>(index) + 0.5) / count;
      const double x = -7.8 + 16.0 * fraction;
      const double side = index % 2 == 0 ? -1.0 : 1.0;
      const double y = center_y + side * (config.lane_half_width_m + 0.35);
      append_p1_cylinder(base, x, y, config.trunk_radius_m,
                         short_features ? 1.05 : 2.85, config.resolution_m);
      if (index < canopy_count) append_p1_canopy(base, x, y, config.canopy_resolution_m);
    }
  };
  if (config.central_obstacle_enabled)
    append_p1_box(base, config.central_x_min_m, config.central_x_max_m,
                  -config.central_y_half_width_m, config.central_y_half_width_m,
                  0.0, config.central_z_max_m, config.resolution_m);
  if (config.name == "p1_fork_symmetric_null_v1") {
    add_lane(-config.lane_center_m, config.safe_tree_density_per_m2,
             config.safe_canopy_probability, true);
    const auto lower = base;
    for (const auto& point : lower) base.push_back({point.x, -point.y, point.z});
  } else if (config.name == "p1_soft_risk_island_v1") {
    const int count = std::max(4, static_cast<int>(std::llround(
        24.0 * config.risky_tree_density_per_m2)));
    for (int index = 0; index < count; ++index) {
      const double fraction = (static_cast<double>(index) + 0.5) / count;
      append_p1_canopy(base, -6.0 + 8.0 * fraction,
                       0.9 + (index % 2 == 0 ? 1.15 : -1.15),
                       config.canopy_resolution_m);
    }
  } else {
    add_lane(-config.lane_center_m, config.safe_tree_density_per_m2,
             config.safe_canopy_probability, true);
    add_lane(config.lane_center_m, config.risky_tree_density_per_m2,
             config.risky_canopy_probability, false);
  }
  if (config.mirror_y)
    for (auto& point : base) point.y = -point.y;
  return base;
}

}  // namespace iap::planner
