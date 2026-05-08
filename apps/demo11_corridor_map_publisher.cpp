#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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

}  // namespace

class Demo11CorridorMapPublisher : public rclcpp::Node {
 public:
  Demo11CorridorMapPublisher()
      : rclcpp::Node("demo11_corridor_map_publisher") {
    resolution_ = declare_parameter<double>("resolution_m", 0.10);
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 2.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
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

    build_map();
    global_cloud_ = make_cloud(groups_.all);
    trunk_cloud_ = make_cloud(groups_.trunks);
    canopy_cloud_ = make_cloud(groups_.canopy);

    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    global_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/global_cloud", qos);
    local_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/map_generator/local_cloud", qos);
    trunk_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo11/trunk_cloud", qos);
    canopy_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo11/canopy_cloud", qos);

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
        "points, resolution %.2fm, seed %d",
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
        trunk_max_height_m_, groups_.all.size(), resolution_, random_seed_);
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

  void add_canopy(std::mt19937& rng, const TrunkInstance& trunk) {
    std::uniform_real_distribution<double> canopy_radius_dist(
        canopy_hemisphere_radius_min_m_, canopy_hemisphere_radius_max_m_);
    const double canopy_radius = canopy_radius_dist(rng);
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
    ++region_canopy_counts_[trunk.region_index];
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
          add_trunk(x_dist(rng), y_dist(rng), height_dist(rng), region_index);
          ++tree_count;
        }
      }
    }
    region_tree_counts_[region_index] = tree_count;
  }

  void build_map() {
    groups_ = ForestGroups{};
    trunks_.clear();
    region_tree_counts_.fill(0);
    region_canopy_counts_.fill(0);

    forest_size_x_m_ = std::max(0.1, forest_size_x_m_);
    forest_size_y_m_ = std::max(0.1, forest_size_y_m_);
    trunk_radius_m_ = std::max(0.02, trunk_radius_m_);
    stratified_cell_size_m_ = std::max(0.1, stratified_cell_size_m_);
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
    const auto stamp = now();
    global_cloud_.header.stamp = stamp;
    trunk_cloud_.header.stamp = stamp;
    canopy_cloud_.header.stamp = stamp;
    global_pub_->publish(global_cloud_);
    local_pub_->publish(global_cloud_);
    trunk_pub_->publish(trunk_cloud_);
    canopy_pub_->publish(canopy_cloud_);
  }

  double resolution_ = 0.10;
  double publish_rate_hz_ = 2.0;
  std::string frame_id_ = "map";
  double forest_size_x_m_ = 20.0;
  double forest_size_y_m_ = 20.0;
  double tree_density_lower_left_per_m2_ = 0.25;
  double tree_density_lower_right_per_m2_ = 0.25;
  double tree_density_upper_left_per_m2_ = 0.25;
  double tree_density_upper_right_per_m2_ = 0.25;
  double stratified_cell_size_m_ = 2.0;
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
  std::array<int, 4> region_tree_counts_{};
  std::array<int, 4> region_canopy_counts_{};
  std::vector<TrunkInstance> trunks_;
  ForestGroups groups_;
  sensor_msgs::msg::PointCloud2 global_cloud_;
  sensor_msgs::msg::PointCloud2 trunk_cloud_;
  sensor_msgs::msg::PointCloud2 canopy_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr local_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr trunk_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr canopy_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Demo11CorridorMapPublisher>());
  rclcpp::shutdown();
  return 0;
}
