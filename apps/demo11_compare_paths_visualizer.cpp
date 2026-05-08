#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace {

using Row = std::unordered_map<std::string, std::string>;

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> out;
  std::string item;
  std::stringstream ss(line);
  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }
  if (!line.empty() && line.back() == ',') {
    out.emplace_back();
  }
  return out;
}

std::vector<Row> read_csv(const std::string& path) {
  std::ifstream in(path);
  if (!in) {
    return {};
  }
  std::string line;
  if (!std::getline(in, line)) {
    return {};
  }
  const auto header = split_csv_line(line);
  std::vector<Row> rows;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    const auto values = split_csv_line(line);
    Row row;
    for (std::size_t i = 0; i < header.size() && i < values.size(); ++i) {
      row.emplace(header[i], values[i]);
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

double as_double(const Row& row, const std::string& key) {
  const auto it = row.find(key);
  if (it == row.end() || it->second.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  try {
    const double value = std::stod(it->second);
    return std::isfinite(value) ? value : std::numeric_limits<double>::quiet_NaN();
  } catch (...) {
    return std::numeric_limits<double>::quiet_NaN();
  }
}

nav_msgs::msg::Path make_path(const std::vector<Row>& rows,
                              const std::string& frame_id,
                              const std::string& x_key,
                              const std::string& y_key,
                              const std::string& z_key) {
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  for (const auto& row : rows) {
    const double x = as_double(row, x_key);
    const double y = as_double(row, y_key);
    const double z = as_double(row, z_key);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      continue;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.w = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

sensor_msgs::msg::PointCloud2 make_cost_cloud(const std::vector<Row>& rows,
                                              const std::string& frame_id) {
  struct Sample {
    float x;
    float y;
    float z;
    float cost;
    float risk;
  };
  std::vector<Sample> samples;
  samples.reserve(rows.size());
  for (const auto& row : rows) {
    const double x = as_double(row, "x");
    const double y = as_double(row, "y");
    const double z = as_double(row, "z");
    const double cost = as_double(row, "pi_cost_total");
    const double risk = as_double(row, "pi_risk_band_code");
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(cost)) {
      continue;
    }
    samples.push_back(Sample{static_cast<float>(x),
                             static_cast<float>(y),
                             static_cast<float>(z),
                             static_cast<float>(cost),
                             static_cast<float>(std::isfinite(risk) ? risk : 0.0)});
  }

  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.height = 1;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "cost", 1, sensor_msgs::msg::PointField::FLOAT32,
      "risk_band_code", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(samples.size());

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> cost(cloud, "cost");
  sensor_msgs::PointCloud2Iterator<float> risk(cloud, "risk_band_code");
  for (const auto& sample : samples) {
    *x = sample.x;
    *y = sample.y;
    *z = sample.z;
    *cost = sample.cost;
    *risk = sample.risk;
    ++x;
    ++y;
    ++z;
    ++cost;
    ++risk;
  }
  cloud.width = static_cast<uint32_t>(samples.size());
  cloud.is_dense = true;
  return cloud;
}

}  // namespace

class Demo11ComparePathsVisualizer : public rclcpp::Node {
 public:
  Demo11ComparePathsVisualizer()
      : rclcpp::Node("demo11_compare_paths_visualizer") {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    const auto off_run_dir = declare_parameter<std::string>("off_run_dir", "");
    const auto on_run_dir = declare_parameter<std::string>("on_run_dir", "");

    off_path_ = make_path(
        read_csv(off_run_dir + "/export/desired_vs_truth.csv"),
        frame_id_, "truth_x", "truth_y", "truth_z");
    on_path_ = make_path(
        read_csv(on_run_dir + "/export/desired_vs_truth.csv"),
        frame_id_, "truth_x", "truth_y", "truth_z");
    on_cost_cloud_ = make_cost_cloud(
        read_csv(on_run_dir + "/export/integrity_along_planner_traj.csv"),
        frame_id_);

    const auto qos = rclcpp::QoS(1).transient_local().reliable();
    off_path_pub_ = create_publisher<nav_msgs::msg::Path>("/demo11/off/path", qos);
    on_path_pub_ = create_publisher<nav_msgs::msg::Path>("/demo11/on/path", qos);
    on_cost_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/demo11/on/integrity_cost_field", qos);
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
                               [this]() { publish_outputs(); });
    RCLCPP_INFO(get_logger(),
                "Demo11 compare visualizer loaded off=%zu poses, on=%zu poses, cost points=%u",
                off_path_.poses.size(), on_path_.poses.size(), on_cost_cloud_.width);
  }

 private:
  void publish_outputs() {
    const auto stamp = now();
    off_path_.header.stamp = stamp;
    on_path_.header.stamp = stamp;
    for (auto& pose : off_path_.poses) {
      pose.header.stamp = stamp;
    }
    for (auto& pose : on_path_.poses) {
      pose.header.stamp = stamp;
    }
    on_cost_cloud_.header.stamp = stamp;
    off_path_pub_->publish(off_path_);
    on_path_pub_->publish(on_path_);
    on_cost_pub_->publish(on_cost_cloud_);
  }

  std::string frame_id_;
  nav_msgs::msg::Path off_path_;
  nav_msgs::msg::Path on_path_;
  sensor_msgs::msg::PointCloud2 on_cost_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr off_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr on_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr on_cost_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Demo11ComparePathsVisualizer>());
  rclcpp::shutdown();
  return 0;
}
