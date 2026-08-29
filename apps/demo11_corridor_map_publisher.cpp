#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <iap/planner/p1_fixture_geometry.hpp>
#include <iap/sim/demo11_publication_stamp_authority.hpp>
#include <iap/sim/forked_forest_geometry.hpp>

namespace {

struct Point {
  float x;
  float y;
  float z;
};

struct ForestGroups {
  std::vector<Point> all;
  std::vector<Point> trunks;
  std::vector<Point> canopy;
  std::vector<Point> terminal_wall;
  std::vector<Point> p0_6_fixture;
  std::vector<Point> p1_fixture;
};

struct TrunkInstance {
  double x;
  double y;
  double height;
  int region_index;
};

void append_point(std::vector<Point>& points,
                  const double x,
                  const double y,
                  const double z) {
  points.push_back(
      Point{static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)});
}

void add_cylinder(std::vector<Point>& group,
                  std::vector<Point>& all,
                  const double cx,
                  const double cy,
                  const double radius,
                  const double z_min,
                  const double z_max,
                  const double resolution) {
  const double res = std::max(0.05, resolution);
  for (double x = cx - radius; x <= cx + radius + 1.0e-9; x += res) {
    for (double y = cy - radius; y <= cy + radius + 1.0e-9; y += res) {
      const double dx = x - cx;
      const double dy = y - cy;
      if (dx * dx + dy * dy > radius * radius) {
        continue;
      }
      for (double z = z_min; z <= z_max + 1.0e-9; z += res) {
        append_point(group, x, y, z);
        append_point(all, x, y, z);
      }
    }
  }
}

void add_sphere_clipped_to_hemisphere(std::vector<Point>& group,
                                      std::vector<Point>& all,
                                      const double cx,
                                      const double cy,
                                      const double cz,
                                      const double sphere_radius,
                                      const double hemisphere_cx,
                                      const double hemisphere_cy,
                                      const double hemisphere_base_z,
                                      const double hemisphere_radius,
                                      const double resolution) {
  const double res = std::max(0.05, resolution);
  for (double x = cx - sphere_radius; x <= cx + sphere_radius + 1.0e-9; x += res) {
    for (double y = cy - sphere_radius; y <= cy + sphere_radius + 1.0e-9; y += res) {
      for (double z = cz - sphere_radius; z <= cz + sphere_radius + 1.0e-9; z += res) {
        const double sx = x - cx;
        const double sy = y - cy;
        const double sz = z - cz;
        if (sx * sx + sy * sy + sz * sz > sphere_radius * sphere_radius) {
          continue;
        }

        const double hz = z - hemisphere_base_z;
        if (hz < 0.0) {
          continue;
        }
        const double hx = x - hemisphere_cx;
        const double hy = y - hemisphere_cy;
        if (hx * hx + hy * hy + hz * hz >
            hemisphere_radius * hemisphere_radius) {
          continue;
        }

        append_point(group, x, y, z);
        append_point(all, x, y, z);
      }
    }
  }
}

void add_box(std::vector<Point>& group,
             std::vector<Point>& all,
             const double x_min,
             const double x_max,
             const double y_min,
             const double y_max,
             const double z_min,
             const double z_max,
             const double resolution) {
  const double res = std::max(0.05, resolution);
  const double xmin = std::min(x_min, x_max);
  const double xmax = std::max(x_min, x_max);
  const double ymin = std::min(y_min, y_max);
  const double ymax = std::max(y_min, y_max);
  const double zmin = std::min(z_min, z_max);
  const double zmax = std::max(z_min, z_max);
  for (double x = xmin; x <= xmax + 1.0e-9; x += res) {
    for (double y = ymin; y <= ymax + 1.0e-9; y += res) {
      for (double z = zmin; z <= zmax + 1.0e-9; z += res) {
        append_point(group, x, y, z);
        append_point(all, x, y, z);
      }
    }
  }
}

void add_terminal_wall(std::vector<Point>& group,
                       std::vector<Point>& all,
                       const double center_x,
                       const double center_y,
                       const double width_y,
                       const double z_min,
                       const double z_max,
                       const double thickness_x,
                       const double resolution,
                       const double feature_depth_x,
                       const int feature_count,
                       const int feature_seed) {
  const double res = std::max(0.05, resolution);
  const double half_width = 0.5 * std::max(res, width_y);
  const double half_thickness = 0.5 * std::max(res, thickness_x);
  const double wall_height = std::max(res, z_max - z_min);
  const double max_depth = std::max(0.0, feature_depth_x);
  const double front_x = center_x - half_thickness;
  const double back_x = center_x + half_thickness;
  const double y_min = center_y - half_width;
  const double y_max = center_y + half_width;

  add_box(group, all, front_x, back_x, y_min, y_max, z_min, z_max, res);

  if (max_depth <= 1.0e-9) {
    return;
  }

  std::mt19937 rng(static_cast<std::uint32_t>(feature_seed));
  std::uniform_real_distribution<double> unit(0.0, 1.0);

  const auto add_feature = [&](const double y_center,
                               const double z_center,
                               const double size_y,
                               const double size_z,
                               const double depth,
                               const bool on_front) {
    const double sy = std::clamp(size_y, res, 0.95 * width_y);
    const double sz = std::clamp(size_z, res, 0.95 * wall_height);
    const double y0 = std::clamp(y_center - 0.5 * sy, y_min, y_max);
    const double y1 = std::clamp(y_center + 0.5 * sy, y_min, y_max);
    const double z0 = std::clamp(z_center - 0.5 * sz, z_min, z_max);
    const double z1 = std::clamp(z_center + 0.5 * sz, z_min, z_max);
    const double d = std::max(res, std::min(max_depth, depth));
    if (on_front) {
      add_box(group, all, front_x - d, front_x, y0, y1, z0, z1, res);
    } else {
      add_box(group, all, back_x, back_x + d, y0, y1, z0, z1, res);
    }
  };

  const double y_span = std::max(res, width_y);
  const double z_span = wall_height;

  for (int i = 0; i < 4; ++i) {
    const bool on_front = (i % 2) == 0;
    const double y = y_min + (0.18 + 0.21 * static_cast<double>(i)) * y_span;
    const double z = z_min + 0.5 * z_span;
    const double depth = max_depth * (0.40 + 0.12 * static_cast<double>(i));
    add_feature(y, z, 0.18 + 0.04 * static_cast<double>(i), z_span, depth,
                on_front);
  }

  for (int i = 0; i < 3; ++i) {
    const bool on_front = (i % 2) != 0;
    const double y = center_y;
    const double z = z_min + (0.28 + 0.22 * static_cast<double>(i)) * z_span;
    const double depth = max_depth * (0.55 + 0.10 * static_cast<double>(i));
    add_feature(y, z, y_span, 0.16 + 0.05 * static_cast<double>(i), depth,
                on_front);
  }

  for (int i = 0; i < feature_count; ++i) {
    const bool on_front = unit(rng) < 0.55;
    const double y = y_min + unit(rng) * y_span;
    const double z = z_min + unit(rng) * z_span;
    const double size_y = 0.35 + unit(rng) * 1.10;
    const double size_z = 0.25 + unit(rng) * 0.85;
    const double depth = max_depth * (0.25 + unit(rng) * 0.75);
    add_feature(y, z, size_y, size_z, depth, on_front);
  }
}

}  // namespace

class Demo11CorridorMapPublisher : public rclcpp::Node {
 public:
  Demo11CorridorMapPublisher()
      : rclcpp::Node("demo11_corridor_map_publisher") {
    resolution_ = declare_parameter<double>("resolution_m", 0.10);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    stamp_authority_topic_ = declare_parameter<std::string>(
        "stamp_authority_topic", "/sim/drone_0/truth_odom");
    forest_size_x_m_ = declare_parameter<double>("forest_size_x_m", 20.0);
    forest_size_y_m_ = declare_parameter<double>("forest_size_y_m", 20.0);
    tree_density_lower_left_per_m2_ =
        declare_parameter<double>("tree_density_lower_left_per_m2", 0.25);
    tree_density_lower_right_per_m2_ =
        declare_parameter<double>("tree_density_lower_right_per_m2", 0.25);
    tree_density_upper_left_per_m2_ =
        declare_parameter<double>("tree_density_upper_left_per_m2", 0.25);
    tree_density_upper_right_per_m2_ =
        declare_parameter<double>("tree_density_upper_right_per_m2", 0.25);
    stratified_cell_size_m_ =
        declare_parameter<double>("stratified_cell_size_m", 2.0);
    clear_corridor_enabled_ =
        declare_parameter<bool>("clear_corridor_enabled", false);
    clear_corridor_center_y_m_ =
        declare_parameter<double>("clear_corridor_center_y_m", 0.0);
    clear_corridor_half_width_y_m_ =
        declare_parameter<double>("clear_corridor_half_width_y_m", 0.0);
    clear_corridor_x_min_m_ =
        declare_parameter<double>("clear_corridor_x_min_m", -1.0e9);
    clear_corridor_x_max_m_ =
        declare_parameter<double>("clear_corridor_x_max_m", 1.0e9);
    forest_layout_mode_ =
        declare_parameter<std::string>("forest_layout_mode", "random_forest");
    forked_forest_fork_count_ =
        declare_parameter<int>("forked_forest.fork_count", 4);
    forked_forest_config_.fork_x_min_m =
        declare_parameter<double>("forked_forest.fork_x_min_m", -16.0);
    forked_forest_config_.fork_length_m =
        declare_parameter<double>("forked_forest.fork_length_m", 8.0);
    forked_forest_config_.low_risk_amplitude_m =
        declare_parameter<double>("forked_forest.low_risk_amplitude_m", 4.0);
    forked_forest_config_.high_risk_amplitude_m =
        declare_parameter<double>("forked_forest.high_risk_amplitude_m", 2.8);
    forked_forest_config_.corridor_half_width_m = 0.5 *
        declare_parameter<double>("forked_forest.corridor_width_m", 2.4);
    forked_forest_config_.junction_clearance_radius_m = declare_parameter<double>(
        "forked_forest.junction_clearance_radius_m", 2.0);
    forked_forest_config_.start_x_m =
        declare_parameter<double>("forked_forest.start_x_m", -18.0);
    forked_forest_config_.goal_x_m =
        declare_parameter<double>("forked_forest.goal_x_m", 18.0);
    forked_forest_config_.fork_risk_seed = static_cast<std::uint32_t>(
        declare_parameter<int>("forked_forest.risk_seed", 21));
    forked_forest_flight_clearance_z_m_ = declare_parameter<double>(
        "forked_forest.flight_clearance_z_m", 2.8);
    forked_forest_edge_tree_spacing_m_ = declare_parameter<double>(
        "forked_forest.edge_tree_spacing_m", 1.0);
    forked_forest_edge_tree_height_m_ = declare_parameter<double>(
        "forked_forest.edge_tree_height_m", 3.2);
    forked_forest_edge_canopy_radius_m_ = declare_parameter<double>(
        "forked_forest.edge_canopy_radius_m", 1.5);
    forked_forest_side_boundary_spacing_m_ = declare_parameter<double>(
        "forked_forest.side_boundary_tree_spacing_m", 0.0);
    canopy_density_lower_left_ =
        declare_parameter<double>("canopy_density_lower_left", 0.5);
    canopy_density_lower_right_ =
        declare_parameter<double>("canopy_density_lower_right", 0.5);
    canopy_density_upper_left_ =
        declare_parameter<double>("canopy_density_upper_left", 0.5);
    canopy_density_upper_right_ =
        declare_parameter<double>("canopy_density_upper_right", 0.5);
    canopy_hemisphere_radius_min_m_ =
        declare_parameter<double>("canopy_hemisphere_radius_min_m", 0.50);
    canopy_hemisphere_radius_max_m_ =
        declare_parameter<double>("canopy_hemisphere_radius_max_m", 2.00);
    canopy_leaf_ball_radius_m_ =
        declare_parameter<double>("canopy_leaf_ball_radius_m", 0.22);
    canopy_ball_spacing_ratio_ =
        declare_parameter<double>("canopy_ball_spacing_ratio", 1.70);
    canopy_resolution_m_ = declare_parameter<double>("canopy_resolution_m", 0.15);
    random_seed_ = declare_parameter<int>("random_seed", 11);
    trunk_radius_m_ = declare_parameter<double>("trunk_radius_m", 0.14);
    trunk_min_height_m_ = declare_parameter<double>("trunk_min_height_m", 1.5);
    trunk_max_height_m_ = declare_parameter<double>("trunk_max_height_m", 3.0);
    terminal_wall_enabled_ =
        declare_parameter<bool>("terminal_wall_enabled", true);
    terminal_wall_x_m_ = declare_parameter<double>("terminal_wall_x_m", 13.5);
    terminal_wall_y_m_ = declare_parameter<double>("terminal_wall_y_m", 0.0);
    terminal_wall_width_y_m_ =
        declare_parameter<double>("terminal_wall_width_y_m", 10.0);
    terminal_wall_z_min_m_ =
        declare_parameter<double>("terminal_wall_z_min_m", 0.0);
    terminal_wall_z_max_m_ =
        declare_parameter<double>("terminal_wall_z_max_m", 3.2);
    terminal_wall_thickness_x_m_ =
        declare_parameter<double>("terminal_wall_thickness_x_m", 0.20);
    terminal_wall_resolution_m_ =
        declare_parameter<double>("terminal_wall_resolution_m", 0.10);
    terminal_wall_feature_depth_x_m_ =
        declare_parameter<double>("terminal_wall_feature_depth_x_m", 0.65);
    terminal_wall_feature_count_ =
        declare_parameter<int>("terminal_wall_feature_count", 48);
    terminal_wall_feature_seed_ =
        declare_parameter<int>("terminal_wall_feature_seed", random_seed_ + 11011);
    corridor_walls_enabled_ =
        declare_parameter<bool>("corridor_walls_enabled", false);
    corridor_floor_enabled_ =
        declare_parameter<bool>("corridor_floor_enabled", false);
    corridor_x_min_m_ = declare_parameter<double>("corridor_x_min_m", -14.0);
    corridor_x_max_m_ = declare_parameter<double>("corridor_x_max_m", 14.0);
    corridor_half_width_y_m_ =
        declare_parameter<double>("corridor_half_width_y_m", 2.0);
    corridor_wall_z_min_m_ =
        declare_parameter<double>("corridor_wall_z_min_m", 0.0);
    corridor_wall_z_max_m_ =
        declare_parameter<double>("corridor_wall_z_max_m", 3.0);
    corridor_wall_thickness_y_m_ =
        declare_parameter<double>("corridor_wall_thickness_y_m", 0.10);
    corridor_floor_thickness_z_m_ =
        declare_parameter<double>("corridor_floor_thickness_z_m", 0.05);
    corridor_surface_resolution_m_ =
        declare_parameter<double>("corridor_surface_resolution_m", 0.10);
    p0_6_fixture_enabled_ =
        declare_parameter<bool>("p0_6.fixture.enabled", false);
    p0_6_fixture_name_ =
        declare_parameter<std::string>("p0_6.fixture.name", "");
    p0_6_fixture_x_min_m_ =
        declare_parameter<double>("p0_6.fixture.x_min", -1.5);
    p0_6_fixture_x_max_m_ =
        declare_parameter<double>("p0_6.fixture.x_max", 1.5);
    p0_6_fixture_y_min_m_ =
        declare_parameter<double>("p0_6.fixture.y_min", -0.75);
    p0_6_fixture_y_max_m_ =
        declare_parameter<double>("p0_6.fixture.y_max", 0.75);
    p0_6_fixture_z_min_m_ =
        declare_parameter<double>("p0_6.fixture.z_min", 1.0);
    p0_6_fixture_z_max_m_ =
        declare_parameter<double>("p0_6.fixture.z_max", 2.0);
    p1_map_fixture_ = declare_parameter<std::string>("p1_map_fixture", "");
    p1_fixture_mirror_y_ = declare_parameter<bool>("p1_fixture_mirror_y", false);
    p1_fixture_central_obstacle_enabled_ =
        declare_parameter<bool>("p1_fixture_central_obstacle_enabled", false);
    p1_fixture_central_x_min_m_ =
        declare_parameter<double>("p1_fixture_central_x_min_m", -7.0);
    p1_fixture_central_x_max_m_ =
        declare_parameter<double>("p1_fixture_central_x_max_m", -2.0);
    p1_fixture_central_y_half_width_m_ =
        declare_parameter<double>("p1_fixture_central_y_half_width_m", 0.65);
    p1_fixture_central_z_max_m_ =
        declare_parameter<double>("p1_fixture_central_z_max_m", 2.8);
    p1_fixture_lane_center_m_ =
        declare_parameter<double>("p1_fixture_lane_center_m", 2.0);
    p1_fixture_lane_half_width_m_ =
        declare_parameter<double>("p1_fixture_lane_half_width_m", 0.75);
    p1_fixture_safe_tree_density_per_m2_ =
        declare_parameter<double>("p1_fixture_safe_tree_density_per_m2", 0.25);
    p1_fixture_risky_tree_density_per_m2_ =
        declare_parameter<double>("p1_fixture_risky_tree_density_per_m2", 0.75);
    p1_fixture_safe_canopy_probability_ =
        declare_parameter<double>("p1_fixture_safe_canopy_probability", 0.05);
    p1_fixture_risky_canopy_probability_ =
        declare_parameter<double>("p1_fixture_risky_canopy_probability", 0.85);

    build_map();
    global_cloud_ = make_cloud(groups_.all);
    trunk_cloud_ = make_cloud(groups_.trunks);
    canopy_cloud_ = make_cloud(groups_.canopy);
    terminal_wall_cloud_ = make_cloud(groups_.terminal_wall);
    p0_6_fixture_cloud_ = make_cloud(groups_.p0_6_fixture);
    p1_fixture_cloud_ = make_cloud(groups_.p1_fixture);

    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    global_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", qos);
    local_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/local_cloud", qos);
    trunk_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo11/trunk_cloud", qos);
    canopy_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo11/canopy_cloud", qos);
    terminal_wall_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo11/terminal_wall_cloud", qos);
    p0_6_fixture_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo11/p0_6_fixture_cloud", qos);
    p1_fixture_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo11/p1_fixture_cloud", qos);
    stamp_authority_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        stamp_authority_topic_, rclcpp::QoS(10).best_effort(),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          const auto result = stamp_authority_.update(msg->header.stamp);
          if (result == iap::sim::StampUpdateResult::kMalformed ||
              result == iap::sim::StampUpdateResult::kNonPositive) {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring invalid Demo11 publication authority stamp %d.%09u",
                msg->header.stamp.sec, msg->header.stamp.nanosec);
          } else if (result == iap::sim::StampUpdateResult::kRegressed) {
            RCLCPP_WARN(
                get_logger(),
                "Ignoring regressed Demo11 publication authority stamp %d.%09u",
                msg->header.stamp.sec, msg->header.stamp.nanosec);
          }
        });

    const auto period = std::chrono::duration<double>(
        1.0 / std::max(0.1, publish_rate_hz_));
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        [this]() { publish_map(); });
    RCLCPP_INFO(
        get_logger(),
        "Demo11 forest built: area %.1fm x %.1fm split into four "
        "%.1fm x %.1fm regions, %.1fm stratified cells, trees "
        "LL/LR/UL/UR=%d/%d/%d/%d, density LL/LR/UL/UR=%.2f/%.2f/%.2f/%.2f "
        "trees/m^2, canopy LL/LR/UL/UR=%d/%d/%d/%d, canopy radius %.2f-%.2fm, canopy density "
        "LL/LR/UL/UR=%.2f/%.2f/%.2f/%.2f, height range %.2f-%.2fm, %zu total "
        "points, terminal wall %s at x=%.2fm y=%.2fm width_y=%.2fm z=%.2f-%.2fm "
        "thickness_x=%.2fm feature_depth_x=%.2fm feature_count=%d feature_seed=%d "
        "wall_points=%zu, corridor walls=%s floor=%s x=%.2f..%.2fm half_width_y=%.2fm "
        "z=%.2f..%.2fm corridor_points=%zu, p0_6_fixture=%s name=%s "
        "x=%.2f..%.2f y=%.2f..%.2f z=%.2f..%.2f fixture_points=%zu, "
        "resolution %.2fm, seed %d",
        forest_size_x_m_, forest_size_y_m_, 0.5 * forest_size_x_m_,
        0.5 * forest_size_y_m_, stratified_cell_size_m_, region_tree_counts_[0],
        region_tree_counts_[1], region_tree_counts_[2],
        region_tree_counts_[3], tree_density_lower_left_per_m2_,
        tree_density_lower_right_per_m2_, tree_density_upper_left_per_m2_,
        tree_density_upper_right_per_m2_, region_canopy_counts_[0],
        region_canopy_counts_[1], region_canopy_counts_[2],
        region_canopy_counts_[3], canopy_hemisphere_radius_min_m_,
        canopy_hemisphere_radius_max_m_, canopy_density_lower_left_,
        canopy_density_lower_right_, canopy_density_upper_left_,
        canopy_density_upper_right_, trunk_min_height_m_,
        trunk_max_height_m_, groups_.all.size(),
        terminal_wall_enabled_ ? "enabled" : "disabled",
        terminal_wall_x_m_, terminal_wall_y_m_, terminal_wall_width_y_m_,
        terminal_wall_z_min_m_, terminal_wall_z_max_m_,
        terminal_wall_thickness_x_m_, terminal_wall_feature_depth_x_m_,
        terminal_wall_feature_count_, terminal_wall_feature_seed_,
        groups_.terminal_wall.size(),
        corridor_walls_enabled_ ? "enabled" : "disabled",
        corridor_floor_enabled_ ? "enabled" : "disabled",
        corridor_x_min_m_, corridor_x_max_m_, corridor_half_width_y_m_,
        corridor_wall_z_min_m_, corridor_wall_z_max_m_,
        corridor_structure_point_count_,
        p0_6_fixture_enabled_ ? "enabled" : "disabled",
        p0_6_fixture_name_.c_str(),
        p0_6_fixture_x_min_m_, p0_6_fixture_x_max_m_,
        p0_6_fixture_y_min_m_, p0_6_fixture_y_max_m_,
        p0_6_fixture_z_min_m_, p0_6_fixture_z_max_m_,
        groups_.p0_6_fixture.size(),
        resolution_, random_seed_);
    if (forked_forest_enabled()) {
      const auto signs = iap::sim::forkLowRiskSigns(forked_forest_config_);
      RCLCPP_INFO(
          get_logger(),
          "Forked forest layout=%s forks=%d x_min=%.2f length=%.2f "
          "corridor_width=%.2f low/high_amplitude=%.2f/%.2f "
          "low_sides(y_sign)=%d,%d,%d,%d risk_seed=%u "
          "flight_clearance_z=%.2f edge_canopy_base/radius=%.2f/%.2f "
          "canopy_column_hit_rate_low/high=%.3f/%.3f",
          forest_layout_mode_.c_str(), iap::sim::kForkedForestForkCount,
          forked_forest_config_.fork_x_min_m,
          forked_forest_config_.fork_length_m,
          2.0 * forked_forest_config_.corridor_half_width_m,
          forked_forest_config_.low_risk_amplitude_m,
          forked_forest_config_.high_risk_amplitude_m,
          signs[0], signs[1], signs[2], signs[3],
          forked_forest_config_.fork_risk_seed,
          forked_forest_flight_clearance_z_m_,
          forked_forest_edge_tree_height_m_,
          forked_forest_edge_canopy_radius_m_,
          fork_route_canopy_hit_rate(iap::sim::ForkArm::kLowRisk),
          fork_route_canopy_hit_rate(iap::sim::ForkArm::kHighRisk));
    }
    if (!p1_map_fixture_.empty()) {
      RCLCPP_INFO(
          get_logger(),
          "P1 scenario fixture=%s mirror_y=%s points=%zu central=%s "
          "x=%.2f..%.2f y_half=%.2f z=0..%.2f lane_center=+/-%.2f "
          "lane_half_width=%.2f safe/risky_density=%.2f/%.2f "
          "safe/risky_canopy=%.2f/%.2f seed=%d",
          p1_map_fixture_.c_str(), p1_fixture_mirror_y_ ? "true" : "false",
          groups_.p1_fixture.size(),
          p1_fixture_central_obstacle_enabled_ ? "true" : "false",
          p1_fixture_central_x_min_m_, p1_fixture_central_x_max_m_,
          p1_fixture_central_y_half_width_m_, p1_fixture_central_z_max_m_,
          p1_fixture_lane_center_m_, p1_fixture_lane_half_width_m_,
          p1_fixture_safe_tree_density_per_m2_,
          p1_fixture_risky_tree_density_per_m2_,
          p1_fixture_safe_canopy_probability_,
          p1_fixture_risky_canopy_probability_, random_seed_);
    }
  }

 private:
  void add_trunk(const double x,
                 const double y,
                 const double height,
                 const int region_index) {
    add_cylinder(groups_.trunks, groups_.all, x, y, trunk_radius_m_, 0.0, height,
                 resolution_);
    trunks_.push_back(TrunkInstance{x, y, height, region_index});
  }

  double canopy_density_for_region(const int region_index) const {
    switch (region_index) {
      case 0:
        return canopy_density_lower_left_;
      case 1:
        return canopy_density_lower_right_;
      case 2:
        return canopy_density_upper_left_;
      case 3:
        return canopy_density_upper_right_;
      default:
        return 0.0;
    }
  }

  void add_canopy_with_radius(const TrunkInstance& trunk,
                              const double canopy_radius,
                              const bool count_region = true) {
    const double ball_spacing = std::max(
        canopy_resolution_m_, canopy_leaf_ball_radius_m_ * canopy_ball_spacing_ratio_);
    const int layer_count =
        std::max(1, static_cast<int>(std::ceil(canopy_radius / ball_spacing)));

    for (int iz = 0; iz <= layer_count; ++iz) {
      const double local_z = std::min(
          canopy_radius, static_cast<double>(iz) * ball_spacing);
      const double layer_radius = std::sqrt(
          std::max(0.0, canopy_radius * canopy_radius - local_z * local_z));
      const int layer_steps =
          std::max(0, static_cast<int>(std::ceil(layer_radius / ball_spacing)));

      for (int ix = -layer_steps; ix <= layer_steps; ++ix) {
        for (int iy = -layer_steps; iy <= layer_steps; ++iy) {
          const double local_x = static_cast<double>(ix) * ball_spacing;
          const double local_y = static_cast<double>(iy) * ball_spacing;
          if (local_x * local_x + local_y * local_y + local_z * local_z >
              canopy_radius * canopy_radius) {
            continue;
          }
          add_sphere_clipped_to_hemisphere(
              groups_.canopy, groups_.all,
              trunk.x + local_x, trunk.y + local_y, trunk.height + local_z,
              canopy_leaf_ball_radius_m_, trunk.x, trunk.y, trunk.height,
              canopy_radius, canopy_resolution_m_);
        }
      }
    }
    if (count_region && trunk.region_index >= 0 && trunk.region_index < 4) {
      ++region_canopy_counts_[trunk.region_index];
    }
  }

  void add_canopy(std::mt19937& rng, const TrunkInstance& trunk) {
    std::uniform_real_distribution<double> canopy_radius_dist(
        canopy_hemisphere_radius_min_m_, canopy_hemisphere_radius_max_m_);
    add_canopy_with_radius(trunk, canopy_radius_dist(rng));
  }

  void add_region_trees(std::mt19937& rng,
                        const int region_index,
                        const double x_min,
                        const double x_max,
                        const double y_min,
                        const double y_max,
                        const double density,
                        std::uniform_real_distribution<double>& height_dist) {
    const double forest_x_min = -0.5 * forest_size_x_m_;
    const double forest_x_max = 0.5 * forest_size_x_m_;
    const double forest_y_min = -0.5 * forest_size_y_m_;
    const double forest_y_max = 0.5 * forest_size_y_m_;
    const double eps = 1.0e-9;

    const double width = x_max - x_min;
    const double height = y_max - y_min;
    const int cell_count_x = std::max(
        1, static_cast<int>(std::ceil(width / stratified_cell_size_m_)));
    const int cell_count_y = std::max(
        1, static_cast<int>(std::ceil(height / stratified_cell_size_m_)));
    const double cell_width = width / static_cast<double>(cell_count_x);
    const double cell_height = height / static_cast<double>(cell_count_y);
    const double cell_area = cell_width * cell_height;
    const int trees_per_cell =
        density <= 0.0
            ? 0
            : std::max(1, static_cast<int>(std::llround(density * cell_area)));

    int tree_count = 0;
    for (int ix = 0; ix < cell_count_x; ++ix) {
      for (int iy = 0; iy < cell_count_y; ++iy) {
        const double cell_x_min = x_min + static_cast<double>(ix) * cell_width;
        const double cell_x_max = cell_x_min + cell_width;
        const double cell_y_min = y_min + static_cast<double>(iy) * cell_height;
        const double cell_y_max = cell_y_min + cell_height;

        const double sample_x_min =
            cell_x_min +
            (cell_x_min <= forest_x_min + eps ? trunk_radius_m_ : 0.0);
        const double sample_x_max =
            cell_x_max -
            (cell_x_max >= forest_x_max - eps ? trunk_radius_m_ : 0.0);
        const double sample_y_min =
            cell_y_min +
            (cell_y_min <= forest_y_min + eps ? trunk_radius_m_ : 0.0);
        const double sample_y_max =
            cell_y_max -
            (cell_y_max >= forest_y_max - eps ? trunk_radius_m_ : 0.0);

        if (sample_x_min > sample_x_max || sample_y_min > sample_y_max) {
          continue;
        }

        std::uniform_real_distribution<double> x_dist(sample_x_min, sample_x_max);
        std::uniform_real_distribution<double> y_dist(sample_y_min, sample_y_max);
        for (int i = 0; i < trees_per_cell; ++i) {
          bool added = false;
          for (int attempt = 0; attempt < 16; ++attempt) {
            const double x = x_dist(rng);
            const double y = y_dist(rng);
            if (inside_clear_corridor(x, y)) {
              continue;
            }
            add_trunk(x, y, height_dist(rng), region_index);
            ++tree_count;
            added = true;
            break;
          }
          if (!added && !clear_corridor_enabled_ && !forked_forest_enabled()) {
            add_trunk(x_dist(rng), y_dist(rng), height_dist(rng), region_index);
            ++tree_count;
          }
        }
      }
    }
    region_tree_counts_[region_index] = tree_count;
  }

  bool inside_clear_corridor(const double x, const double y) const {
    if (forked_forest_enabled()) {
      auto padded = forked_forest_config_;
      padded.corridor_half_width_m += trunk_radius_m_;
      return iap::sim::classifyForkCorridor(padded, x, y).insideAny();
    }
    if (!clear_corridor_enabled_) {
      return false;
    }
    if (x < clear_corridor_x_min_m_ || x > clear_corridor_x_max_m_) {
      return false;
    }
    return std::abs(y - clear_corridor_center_y_m_) <=
           clear_corridor_half_width_y_m_ + trunk_radius_m_;
  }

  bool forked_forest_enabled() const {
    return forest_layout_mode_ == "forked_s_forest_v1";
  }

  void add_forked_forest_edge_canopies() {
    if (!forked_forest_enabled()) {
      return;
    }
    const double offset = forked_forest_config_.corridor_half_width_m +
                          trunk_radius_m_ + 0.05;
    const int samples = std::max(
        2, static_cast<int>(std::floor(
               forked_forest_config_.fork_length_m /
               forked_forest_edge_tree_spacing_m_)));
    for (int fork = 0; fork < iap::sim::kForkedForestForkCount; ++fork) {
      const double x0 = forked_forest_config_.fork_x_min_m +
                        static_cast<double>(fork) *
                            forked_forest_config_.fork_length_m;
      for (int sample = 1; sample < samples; ++sample) {
        const double t = static_cast<double>(sample) /
                         static_cast<double>(samples);
        const double x = x0 + t * forked_forest_config_.fork_length_m;
        for (const auto arm : {iap::sim::ForkArm::kLowRisk,
                               iap::sim::ForkArm::kHighRisk}) {
          const double y = iap::sim::forkArmCenterY(
              forked_forest_config_, fork, arm, x);
          const double slope = iap::sim::forkArmSlope(
              forked_forest_config_, fork, arm, x);
          const double normal_scale = 1.0 / std::hypot(1.0, slope);
          const double nx = -slope * normal_scale;
          const double ny = normal_scale;
          for (const double side : {-1.0, 1.0}) {
            const double tree_x = x + side * offset * nx;
            const double tree_y = y + side * offset * ny;
            auto padded = forked_forest_config_;
            padded.corridor_half_width_m += trunk_radius_m_;
            if (iap::sim::classifyForkCorridor(
                    padded, tree_x, tree_y).insideAny()) {
              continue;
            }
            const int region = (tree_x < 0.0 ? 0 : 1) +
                               (tree_y < 0.0 ? 0 : 2);
            const TrunkInstance trunk{
                tree_x, tree_y, forked_forest_edge_tree_height_m_, region};
            add_trunk(trunk.x, trunk.y, trunk.height, trunk.region_index);
            if (arm == iap::sim::ForkArm::kHighRisk) {
              add_canopy_with_radius(
                  trunk, forked_forest_edge_canopy_radius_m_, false);
            }
          }
        }
      }
    }
  }

  void add_forked_forest_side_boundaries() {
    if (!forked_forest_enabled() ||
        forked_forest_side_boundary_spacing_m_ <= 0.0) {
      return;
    }
    const double x_min = -0.5 * forest_size_x_m_ + trunk_radius_m_;
    const double x_max = 0.5 * forest_size_x_m_ - trunk_radius_m_;
    const double y_abs = 0.5 * forest_size_y_m_ - trunk_radius_m_;
    const double spacing = std::max(
        2.0 * trunk_radius_m_, forked_forest_side_boundary_spacing_m_);
    const int intervals = std::max(
        1, static_cast<int>(std::ceil((x_max - x_min) / spacing)));
    for (int index = 0; index <= intervals; ++index) {
      const double t = static_cast<double>(index) /
                       static_cast<double>(intervals);
      const double x = x_min + t * (x_max - x_min);
      add_trunk(x, -y_abs, trunk_max_height_m_, x < 0.0 ? 0 : 1);
      add_trunk(x, y_abs, trunk_max_height_m_, x < 0.0 ? 2 : 3);
    }
  }

  template <typename Predicate>
  static void erase_points(std::vector<Point>& points,
                           Predicate should_erase) {
    points.erase(
        std::remove_if(points.begin(), points.end(), should_erase),
        points.end());
  }

  void apply_forked_forest_clearance() {
    if (!forked_forest_enabled()) {
      return;
    }
    const double x_limit = 0.5 * forest_size_x_m_ + 1.0e-6;
    const double y_limit = 0.5 * forest_size_y_m_ + 1.0e-6;
    const auto outside_scene = [x_limit, y_limit](const Point& point) {
      return std::abs(point.x) > x_limit || std::abs(point.y) > y_limit;
    };
    erase_points(groups_.trunks, [&](const Point& point) {
      return outside_scene(point) ||
          iap::sim::classifyForkCorridorEnvelope(
              forked_forest_config_, point.x, point.y).insideAny();
    });
    erase_points(groups_.canopy, [&](const Point& point) {
      if (outside_scene(point)) {
        return true;
      }
      const auto corridor = iap::sim::classifyForkCorridorEnvelope(
          forked_forest_config_, point.x, point.y);
      if (corridor.inside_low || corridor.inside_buffer) {
        return true;
      }
      return corridor.inside_high &&
             point.z <= forked_forest_flight_clearance_z_m_;
    });
  }

  void rebuild_global_cloud_points() {
    groups_.all.clear();
    const std::array<const std::vector<Point>*, 5> groups = {
        &groups_.trunks, &groups_.canopy, &groups_.terminal_wall,
        &groups_.p0_6_fixture, &groups_.p1_fixture};
    std::size_t total = 0;
    for (const auto* group : groups) {
      total += group->size();
    }
    groups_.all.reserve(total);
    for (const auto* group : groups) {
      groups_.all.insert(groups_.all.end(), group->begin(), group->end());
    }
  }

  double fork_route_canopy_hit_rate(const iap::sim::ForkArm arm) const {
    constexpr int kSamplesPerFork = 21;
    // P0 voxel support is 0.5 m in this profile; count a vertical canopy
    // column when the centerline's containing support column is occupied.
    constexpr double kColumnRadiusM = 0.50;
    int hits = 0;
    int samples = 0;
    for (int fork = 0; fork < iap::sim::kForkedForestForkCount; ++fork) {
      const double x0 = forked_forest_config_.fork_x_min_m +
                        static_cast<double>(fork) *
                            forked_forest_config_.fork_length_m;
      for (int index = 0; index < kSamplesPerFork; ++index) {
        const double t = 0.1 + 0.8 * static_cast<double>(index) /
                                   static_cast<double>(kSamplesPerFork - 1);
        const double x = x0 + t * forked_forest_config_.fork_length_m;
        const double y = iap::sim::forkArmCenterY(
            forked_forest_config_, fork, arm, x);
        const bool hit = std::any_of(
            groups_.canopy.begin(), groups_.canopy.end(),
            [x, y, kColumnRadiusM](const Point& point) {
              return point.z >= 3.2f && point.z <= 6.5f &&
                     std::hypot(static_cast<double>(point.x) - x,
                                static_cast<double>(point.y) - y) <=
                         kColumnRadiusM;
            });
        hits += hit ? 1 : 0;
        ++samples;
      }
    }
    return samples > 0 ? static_cast<double>(hits) /
                             static_cast<double>(samples)
                       : 0.0;
  }

  void add_corridor_degenerate_geometry() {
    corridor_structure_point_count_ = 0;
    const auto before = groups_.terminal_wall.size();
    const double half_thickness = 0.5 * corridor_wall_thickness_y_m_;

    if (corridor_walls_enabled_) {
      add_box(groups_.terminal_wall, groups_.all,
              corridor_x_min_m_, corridor_x_max_m_,
              -corridor_half_width_y_m_ - half_thickness,
              -corridor_half_width_y_m_ + half_thickness,
              corridor_wall_z_min_m_, corridor_wall_z_max_m_,
              corridor_surface_resolution_m_);
      add_box(groups_.terminal_wall, groups_.all,
              corridor_x_min_m_, corridor_x_max_m_,
              corridor_half_width_y_m_ - half_thickness,
              corridor_half_width_y_m_ + half_thickness,
              corridor_wall_z_min_m_, corridor_wall_z_max_m_,
              corridor_surface_resolution_m_);
    }

    if (corridor_floor_enabled_) {
      add_box(groups_.terminal_wall, groups_.all,
              corridor_x_min_m_, corridor_x_max_m_,
              -corridor_half_width_y_m_, corridor_half_width_y_m_,
              0.0, corridor_floor_thickness_z_m_,
              corridor_surface_resolution_m_);
    }

    corridor_structure_point_count_ = groups_.terminal_wall.size() - before;
  }

  void add_p0_6_fixture_geometry() {
    if (!p0_6_fixture_enabled_ ||
        p0_6_fixture_name_ != "occupied_overlap_box_v1") {
      return;
    }
    add_box(groups_.p0_6_fixture, groups_.all,
            p0_6_fixture_x_min_m_, p0_6_fixture_x_max_m_,
            p0_6_fixture_y_min_m_, p0_6_fixture_y_max_m_,
            p0_6_fixture_z_min_m_, p0_6_fixture_z_max_m_,
            resolution_);
  }

  void add_p1_fixture_geometry() {
    if (p1_map_fixture_.empty()) {
      return;
    }
    iap::planner::P1FixtureConfig config;
    config.name = p1_map_fixture_;
    config.mirror_y = p1_fixture_mirror_y_;
    config.central_obstacle_enabled = p1_fixture_central_obstacle_enabled_;
    config.central_x_min_m = p1_fixture_central_x_min_m_;
    config.central_x_max_m = p1_fixture_central_x_max_m_;
    config.central_y_half_width_m = p1_fixture_central_y_half_width_m_;
    config.central_z_max_m = p1_fixture_central_z_max_m_;
    config.lane_center_m = p1_fixture_lane_center_m_;
    config.lane_half_width_m = p1_fixture_lane_half_width_m_;
    config.safe_tree_density_per_m2 = p1_fixture_safe_tree_density_per_m2_;
    config.risky_tree_density_per_m2 = p1_fixture_risky_tree_density_per_m2_;
    config.safe_canopy_probability = p1_fixture_safe_canopy_probability_;
    config.risky_canopy_probability = p1_fixture_risky_canopy_probability_;
    config.trunk_radius_m = trunk_radius_m_;
    config.resolution_m = resolution_;
    config.canopy_resolution_m = canopy_resolution_m_;
    for (const auto& point : iap::planner::make_p1_fixture_points(config)) {
      append_point(groups_.p1_fixture, point.x, point.y, point.z);
      append_point(groups_.all, point.x, point.y, point.z);
    }
  }

  void build_map() {
    groups_ = ForestGroups{};
    trunks_.clear();
    region_tree_counts_.fill(0);
    region_canopy_counts_.fill(0);

    if (forest_layout_mode_ != "random_forest" &&
        forest_layout_mode_ != "forked_s_forest_v1") {
      throw std::runtime_error(
          "forest_layout_mode must be random_forest or forked_s_forest_v1");
    }
    if (forked_forest_enabled() &&
        forked_forest_fork_count_ != iap::sim::kForkedForestForkCount) {
      throw std::runtime_error(
          "forked_s_forest_v1 requires exactly four forks");
    }
    forked_forest_config_.fork_length_m =
        std::max(0.1, forked_forest_config_.fork_length_m);
    forked_forest_config_.low_risk_amplitude_m =
        std::max(0.0, forked_forest_config_.low_risk_amplitude_m);
    forked_forest_config_.high_risk_amplitude_m =
        std::max(0.0, forked_forest_config_.high_risk_amplitude_m);
    forked_forest_config_.corridor_half_width_m =
        std::max(0.1, forked_forest_config_.corridor_half_width_m);
    forked_forest_config_.junction_clearance_radius_m =
        std::max(forked_forest_config_.corridor_half_width_m,
                 forked_forest_config_.junction_clearance_radius_m);
    forked_forest_flight_clearance_z_m_ =
        std::max(0.1, forked_forest_flight_clearance_z_m_);
    forked_forest_edge_tree_spacing_m_ =
        std::max(0.25, forked_forest_edge_tree_spacing_m_);
    forked_forest_edge_tree_height_m_ = std::max(
        forked_forest_flight_clearance_z_m_ + 0.1,
        forked_forest_edge_tree_height_m_);
    forked_forest_edge_canopy_radius_m_ =
        std::max(0.25, forked_forest_edge_canopy_radius_m_);
    if (forked_forest_side_boundary_spacing_m_ > 0.0) {
      forked_forest_side_boundary_spacing_m_ = std::max(
          2.0 * trunk_radius_m_, forked_forest_side_boundary_spacing_m_);
    }

    forest_size_x_m_ = std::max(0.1, forest_size_x_m_);
    forest_size_y_m_ = std::max(0.1, forest_size_y_m_);
    trunk_radius_m_ = std::max(0.02, trunk_radius_m_);
    stratified_cell_size_m_ = std::max(0.1, stratified_cell_size_m_);
    clear_corridor_half_width_y_m_ =
        std::max(0.0, clear_corridor_half_width_y_m_);
    if (clear_corridor_x_min_m_ > clear_corridor_x_max_m_) {
      std::swap(clear_corridor_x_min_m_, clear_corridor_x_max_m_);
    }
    tree_density_lower_left_per_m2_ =
        std::max(0.0, tree_density_lower_left_per_m2_);
    tree_density_lower_right_per_m2_ =
        std::max(0.0, tree_density_lower_right_per_m2_);
    tree_density_upper_left_per_m2_ =
        std::max(0.0, tree_density_upper_left_per_m2_);
    tree_density_upper_right_per_m2_ =
        std::max(0.0, tree_density_upper_right_per_m2_);
    canopy_density_lower_left_ =
        std::clamp(canopy_density_lower_left_, 0.0, 1.0);
    canopy_density_lower_right_ =
        std::clamp(canopy_density_lower_right_, 0.0, 1.0);
    canopy_density_upper_left_ =
        std::clamp(canopy_density_upper_left_, 0.0, 1.0);
    canopy_density_upper_right_ =
        std::clamp(canopy_density_upper_right_, 0.0, 1.0);
    canopy_hemisphere_radius_min_m_ =
        std::max(0.05, canopy_hemisphere_radius_min_m_);
    canopy_hemisphere_radius_max_m_ =
        std::max(0.05, canopy_hemisphere_radius_max_m_);
    if (canopy_hemisphere_radius_min_m_ > canopy_hemisphere_radius_max_m_) {
      std::swap(canopy_hemisphere_radius_min_m_, canopy_hemisphere_radius_max_m_);
    }
    canopy_leaf_ball_radius_m_ = std::max(0.03, canopy_leaf_ball_radius_m_);
    canopy_ball_spacing_ratio_ = std::clamp(canopy_ball_spacing_ratio_, 0.5, 2.0);
    canopy_resolution_m_ = std::max(0.05, canopy_resolution_m_);
    terminal_wall_width_y_m_ = std::max(0.1, terminal_wall_width_y_m_);
    terminal_wall_thickness_x_m_ = std::max(0.05, terminal_wall_thickness_x_m_);
    terminal_wall_resolution_m_ = std::max(0.05, terminal_wall_resolution_m_);
    terminal_wall_feature_depth_x_m_ =
        std::max(0.0, terminal_wall_feature_depth_x_m_);
    terminal_wall_feature_count_ = std::max(0, terminal_wall_feature_count_);
    if (terminal_wall_z_min_m_ > terminal_wall_z_max_m_) {
      std::swap(terminal_wall_z_min_m_, terminal_wall_z_max_m_);
    }
    if (terminal_wall_z_max_m_ - terminal_wall_z_min_m_ < terminal_wall_resolution_m_) {
      terminal_wall_z_max_m_ = terminal_wall_z_min_m_ + terminal_wall_resolution_m_;
    }
    if (corridor_x_min_m_ > corridor_x_max_m_) {
      std::swap(corridor_x_min_m_, corridor_x_max_m_);
    }
    corridor_half_width_y_m_ = std::max(0.1, corridor_half_width_y_m_);
    corridor_wall_thickness_y_m_ = std::max(0.05, corridor_wall_thickness_y_m_);
    corridor_floor_thickness_z_m_ = std::max(0.0, corridor_floor_thickness_z_m_);
    corridor_surface_resolution_m_ = std::max(0.05, corridor_surface_resolution_m_);
    if (corridor_wall_z_min_m_ > corridor_wall_z_max_m_) {
      std::swap(corridor_wall_z_min_m_, corridor_wall_z_max_m_);
    }
    if (corridor_wall_z_max_m_ - corridor_wall_z_min_m_ < corridor_surface_resolution_m_) {
      corridor_wall_z_max_m_ = corridor_wall_z_min_m_ + corridor_surface_resolution_m_;
    }
    if (p0_6_fixture_x_min_m_ > p0_6_fixture_x_max_m_) {
      std::swap(p0_6_fixture_x_min_m_, p0_6_fixture_x_max_m_);
    }
    if (p0_6_fixture_y_min_m_ > p0_6_fixture_y_max_m_) {
      std::swap(p0_6_fixture_y_min_m_, p0_6_fixture_y_max_m_);
    }
    if (p0_6_fixture_z_min_m_ > p0_6_fixture_z_max_m_) {
      std::swap(p0_6_fixture_z_min_m_, p0_6_fixture_z_max_m_);
    }
    if (trunk_min_height_m_ > trunk_max_height_m_) {
      std::swap(trunk_min_height_m_, trunk_max_height_m_);
    }
    trunk_min_height_m_ = std::max(0.1, trunk_min_height_m_);
    trunk_max_height_m_ = std::max(trunk_min_height_m_, trunk_max_height_m_);

    const double x_min = -0.5 * forest_size_x_m_;
    const double x_mid = 0.0;
    const double x_max = 0.5 * forest_size_x_m_;
    const double y_min = -0.5 * forest_size_y_m_;
    const double y_mid = 0.0;
    const double y_max = 0.5 * forest_size_y_m_;

    std::mt19937 rng(static_cast<std::uint32_t>(random_seed_));
    std::uniform_real_distribution<double> height_dist(
        trunk_min_height_m_, trunk_max_height_m_);

    add_region_trees(rng, 0, x_min, x_mid, y_min, y_mid,
                     tree_density_lower_left_per_m2_, height_dist);
    add_region_trees(rng, 1, x_mid, x_max, y_min, y_mid,
                     tree_density_lower_right_per_m2_, height_dist);
    add_region_trees(rng, 2, x_min, x_mid, y_mid, y_max,
                     tree_density_upper_left_per_m2_, height_dist);
    add_region_trees(rng, 3, x_mid, x_max, y_mid, y_max,
                     tree_density_upper_right_per_m2_, height_dist);

    std::uniform_real_distribution<double> probability_dist(0.0, 1.0);
    for (const auto& trunk : trunks_) {
      if (probability_dist(rng) <= canopy_density_for_region(trunk.region_index)) {
        add_canopy(rng, trunk);
      }
    }
    add_forked_forest_side_boundaries();
    add_forked_forest_edge_canopies();
    apply_forked_forest_clearance();

    if (terminal_wall_enabled_) {
      add_terminal_wall(groups_.terminal_wall, groups_.all,
                        terminal_wall_x_m_, terminal_wall_y_m_,
                        terminal_wall_width_y_m_, terminal_wall_z_min_m_,
                        terminal_wall_z_max_m_, terminal_wall_thickness_x_m_,
                        terminal_wall_resolution_m_,
                        terminal_wall_feature_depth_x_m_,
                        terminal_wall_feature_count_,
                        terminal_wall_feature_seed_);
    }

    add_corridor_degenerate_geometry();
    add_p0_6_fixture_geometry();
    add_p1_fixture_geometry();
    rebuild_global_cloud_points();
  }

  sensor_msgs::msg::PointCloud2 make_cloud(const std::vector<Point>& points) const {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = frame_id_;
    cloud.height = 1;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto& point : points) {
      *x = point.x;
      *y = point.y;
      *z = point.z;
      ++x;
      ++y;
      ++z;
    }
    cloud.width = static_cast<uint32_t>(points.size());
    cloud.is_dense = true;
    return cloud;
  }

  void publish_map() {
    if (!iap::sim::stamp_demo11_publication(
            stamp_authority_, global_cloud_, trunk_cloud_, canopy_cloud_,
            terminal_wall_cloud_, p0_6_fixture_cloud_, p1_fixture_cloud_)) {
      return;
    }
    global_pub_->publish(global_cloud_);
    local_pub_->publish(global_cloud_);
    trunk_pub_->publish(trunk_cloud_);
    canopy_pub_->publish(canopy_cloud_);
    terminal_wall_pub_->publish(terminal_wall_cloud_);
    p0_6_fixture_pub_->publish(p0_6_fixture_cloud_);
    p1_fixture_pub_->publish(p1_fixture_cloud_);
  }

  double resolution_ = 0.10;
  double publish_rate_hz_ = 2.0;
  std::string frame_id_ = "map";
  std::string stamp_authority_topic_ = "/sim/drone_0/truth_odom";
  double forest_size_x_m_ = 20.0;
  double forest_size_y_m_ = 20.0;
  double tree_density_lower_left_per_m2_ = 0.25;
  double tree_density_lower_right_per_m2_ = 0.25;
  double tree_density_upper_left_per_m2_ = 0.25;
  double tree_density_upper_right_per_m2_ = 0.25;
  double stratified_cell_size_m_ = 2.0;
  bool clear_corridor_enabled_ = false;
  double clear_corridor_center_y_m_ = 0.0;
  double clear_corridor_half_width_y_m_ = 0.0;
  double clear_corridor_x_min_m_ = -1.0e9;
  double clear_corridor_x_max_m_ = 1.0e9;
  std::string forest_layout_mode_ = "random_forest";
  iap::sim::ForkedForestConfig forked_forest_config_;
  double forked_forest_flight_clearance_z_m_ = 2.8;
  int forked_forest_fork_count_ = iap::sim::kForkedForestForkCount;
  double forked_forest_edge_tree_spacing_m_ = 1.0;
  double forked_forest_edge_tree_height_m_ = 3.2;
  double forked_forest_edge_canopy_radius_m_ = 1.5;
  double forked_forest_side_boundary_spacing_m_ = 0.0;
  double canopy_density_lower_left_ = 0.5;
  double canopy_density_lower_right_ = 0.5;
  double canopy_density_upper_left_ = 0.5;
  double canopy_density_upper_right_ = 0.5;
  double canopy_hemisphere_radius_min_m_ = 0.50;
  double canopy_hemisphere_radius_max_m_ = 2.00;
  double canopy_leaf_ball_radius_m_ = 0.22;
  double canopy_ball_spacing_ratio_ = 1.70;
  double canopy_resolution_m_ = 0.15;
  int random_seed_ = 11;
  double trunk_radius_m_ = 0.14;
  double trunk_min_height_m_ = 1.5;
  double trunk_max_height_m_ = 3.0;
  bool terminal_wall_enabled_ = true;
  double terminal_wall_x_m_ = 13.5;
  double terminal_wall_y_m_ = 0.0;
  double terminal_wall_width_y_m_ = 10.0;
  double terminal_wall_z_min_m_ = 0.0;
  double terminal_wall_z_max_m_ = 3.2;
  double terminal_wall_thickness_x_m_ = 0.20;
  double terminal_wall_resolution_m_ = 0.10;
  double terminal_wall_feature_depth_x_m_ = 0.65;
  int terminal_wall_feature_count_ = 48;
  int terminal_wall_feature_seed_ = 11022;
  bool corridor_walls_enabled_ = false;
  bool corridor_floor_enabled_ = false;
  double corridor_x_min_m_ = -14.0;
  double corridor_x_max_m_ = 14.0;
  double corridor_half_width_y_m_ = 2.0;
  double corridor_wall_z_min_m_ = 0.0;
  double corridor_wall_z_max_m_ = 3.0;
  double corridor_wall_thickness_y_m_ = 0.10;
  double corridor_floor_thickness_z_m_ = 0.05;
  double corridor_surface_resolution_m_ = 0.10;
  std::size_t corridor_structure_point_count_ = 0;
  bool p0_6_fixture_enabled_ = false;
  std::string p0_6_fixture_name_;
  double p0_6_fixture_x_min_m_ = -1.5;
  double p0_6_fixture_x_max_m_ = 1.5;
  double p0_6_fixture_y_min_m_ = -0.75;
  double p0_6_fixture_y_max_m_ = 0.75;
  double p0_6_fixture_z_min_m_ = 1.0;
  double p0_6_fixture_z_max_m_ = 2.0;
  std::string p1_map_fixture_;
  bool p1_fixture_mirror_y_ = false;
  bool p1_fixture_central_obstacle_enabled_ = false;
  double p1_fixture_central_x_min_m_ = -7.0;
  double p1_fixture_central_x_max_m_ = -2.0;
  double p1_fixture_central_y_half_width_m_ = 0.65;
  double p1_fixture_central_z_max_m_ = 2.8;
  double p1_fixture_lane_center_m_ = 2.0;
  double p1_fixture_lane_half_width_m_ = 0.75;
  double p1_fixture_safe_tree_density_per_m2_ = 0.25;
  double p1_fixture_risky_tree_density_per_m2_ = 0.75;
  double p1_fixture_safe_canopy_probability_ = 0.05;
  double p1_fixture_risky_canopy_probability_ = 0.85;
  std::array<int, 4> region_tree_counts_{};
  std::array<int, 4> region_canopy_counts_{};
  std::vector<TrunkInstance> trunks_;
  ForestGroups groups_;
  sensor_msgs::msg::PointCloud2 global_cloud_;
  sensor_msgs::msg::PointCloud2 trunk_cloud_;
  sensor_msgs::msg::PointCloud2 canopy_cloud_;
  sensor_msgs::msg::PointCloud2 terminal_wall_cloud_;
  sensor_msgs::msg::PointCloud2 p0_6_fixture_cloud_;
  sensor_msgs::msg::PointCloud2 p1_fixture_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trunk_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr canopy_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terminal_wall_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr p0_6_fixture_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr p1_fixture_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr stamp_authority_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  iap::sim::Demo11PublicationStampAuthority stamp_authority_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Demo11CorridorMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
