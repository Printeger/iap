#include <ego_planner/planner_manager.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace ego_planner {
namespace {

uint32_t riskColor(double value, double maximum) {
  const double t = std::clamp(value / maximum, 0.0, 1.0);
  uint8_t red = 0, green = 0, blue = 0;
  if (t <= 0.5) {
    const double u = t * 2.0;
    red = static_cast<uint8_t>(255 * u);
    green = static_cast<uint8_t>(255 * u);
    blue = static_cast<uint8_t>(255 * (1.0 - u));
  } else {
    const double u = (t - 0.5) * 2.0;
    red = 255;
    green = static_cast<uint8_t>(255 * (1.0 - u));
  }
  return (static_cast<uint32_t>(red) << 16) |
         (static_cast<uint32_t>(green) << 8) | blue;
}

struct SlicePoint {
  Eigen::Vector3d center;
  GridRiskVoxel risk;
};

sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<SlicePoint>& samples, const std::string& frame,
    const rclcpp::Time& stamp, const std::string& metric,
    double hpl_max, double vpl_max) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(7,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl", 1, sensor_msgs::msg::PointField::FLOAT32,
      "status", 1, sensor_msgs::msg::PointField::UINT8);
  modifier.resize(samples.size());
  cloud.is_dense = false; // Invalid PL fields are NaN, while XYZ is finite.
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> rgb(cloud, "rgb"), hpl(cloud, "hpl"), vpl(cloud, "vpl");
  sensor_msgs::PointCloud2Iterator<uint8_t> status(cloud, "status");
  for (const auto& sample : samples) {
    *x = static_cast<float>(sample.center.x());
    *y = static_cast<float>(sample.center.y());
    *z = static_cast<float>(sample.center.z());
    *hpl = static_cast<float>(sample.risk.hpl);
    *vpl = static_cast<float>(sample.risk.vpl);
    *status = static_cast<uint8_t>(sample.risk.status);
    const uint32_t packed = sample.risk.status == GridRiskStatus::VALID
        ? riskColor(metric == "hpl" ? sample.risk.hpl : sample.risk.vpl,
                    metric == "hpl" ? hpl_max : vpl_max)
        : 0x9900ccu;
    std::memcpy(&*rgb, &packed, sizeof(packed));
    ++x; ++y; ++z; ++rgb; ++hpl; ++vpl; ++status;
  }
  return cloud;
}

} // namespace

void EGOPlannerManager::initRiskVisualization(const rclcpp::Node::SharedPtr& node) {
  risk_viz_enabled_ = node->declare_parameter("risk_viz/enabled", false);
  risk_viz_metric_ = node->declare_parameter<std::string>("risk_viz/metric", "hpl");
  risk_viz_z_mode_ = node->declare_parameter<std::string>("risk_viz/z_mode", "follow");
  risk_viz_fixed_z_m_ = node->declare_parameter("risk_viz/fixed_z_m", 1.5);
  risk_viz_hpl_max_m_ = node->declare_parameter("risk_viz/hpl_max_m", 10.0);
  risk_viz_vpl_max_m_ = node->declare_parameter("risk_viz/vpl_max_m", 20.0);
  if ((risk_viz_metric_ != "hpl" && risk_viz_metric_ != "vpl") ||
      (risk_viz_z_mode_ != "follow" && risk_viz_z_mode_ != "fixed") ||
      !std::isfinite(risk_viz_fixed_z_m_) ||
      !std::isfinite(risk_viz_hpl_max_m_) || risk_viz_hpl_max_m_ <= 0 ||
      !std::isfinite(risk_viz_vpl_max_m_) || risk_viz_vpl_max_m_ <= 0)
    throw std::invalid_argument("risk_viz parameters require hpl/vpl, follow/fixed and finite ranges");
  if (!risk_viz_enabled_) return;

  risk_slice_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/risk_slice", 1);
  risk_status_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("grid_map/risk_status", 1);
  risk_viz_param_callback_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
          if (param.get_name() != "risk_viz/metric") continue;
          if (param.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
              (param.as_string() != "hpl" && param.as_string() != "vpl")) {
            result.successful = false;
            result.reason = "risk_viz/metric must be hpl or vpl";
            return result;
          }
        }
        for (const auto& param : params)
          if (param.get_name() == "risk_viz/metric") risk_viz_metric_ = param.as_string();
        return result;
      });
  risk_viz_timer_ = node->create_wall_timer(std::chrono::seconds(1),
      [this]() { publishRiskSlice(); });
}

void EGOPlannerManager::publishRiskSlice() {
  if (!risk_viz_enabled_) return;
  const auto started = std::chrono::steady_clock::now();
  const auto stamp = node_->now();
  const double now = stamp.seconds();
  const std::string frame = grid_map_->getFrameId();
  std::vector<SlicePoint> samples;
  samples.reserve(100);
  std::string reason;
  bool truncated = false;
  uint64_t version = 0;
  size_t valid = 0, invalid = 0;
  double slice_z = risk_viz_fixed_z_m_;

  if (!risk_odom_) {
    reason = "waiting for GLIO odometry";
  } else if (risk_odom_->header.frame_id != frame) {
    reason = "GLIO/GridMap frame mismatch";
  } else {
    const auto& p = risk_odom_->pose.pose.position;
    Eigen::Vector3d vehicle(p.x, p.y, p.z);
    if (!vehicle.allFinite()) reason = "invalid GLIO position";
    else {
      if (risk_viz_z_mode_ == "follow") slice_z = vehicle.z();
      Eigen::Vector3i z_index;
      Eigen::Vector3d z_point(vehicle.x(), vehicle.y(), slice_z);
      if (!grid_map_->isInMap(z_point)) reason = "slice height outside GridMap";
      else {
        grid_map_->posToIndex(z_point, z_index);
        Eigen::Vector3d z_center;
        grid_map_->indexToPos(z_index, z_center);
        slice_z = z_center.z();
        version = beginRiskQuery();
        for (int ix = 0; ix < 10 && reason.empty() && !truncated; ++ix) {
          for (int iy = 0; iy < 10; ++iy) {
            if (std::chrono::steady_clock::now() - started >= std::chrono::milliseconds(20)) {
              truncated = true;
              break;
            }
            Eigen::Vector3d query(vehicle.x() + ix - 4.5,
                                  vehicle.y() + iy - 4.5, slice_z);
            if (!grid_map_->isInMap(query)) continue;
            Eigen::Vector3i index;
            grid_map_->posToIndex(query, index);
            Eigen::Vector3d center;
            grid_map_->indexToPos(index, center);
            if (grid_map_->getOccupancy(center) != 0) continue;
            const auto risk = grid_map_->queryRisk(center, version, node_->now().seconds());
            if (risk.status == GridRiskStatus::VERSION_CHANGED ||
                risk.status == GridRiskStatus::STALE ||
                risk.status == GridRiskStatus::FRAME_MISMATCH ||
                risk.status == GridRiskStatus::UNCOMPUTED) {
              reason = "prediction context unavailable (status=" +
                  std::to_string(static_cast<int>(risk.status)) + ")";
              samples.clear();
              valid = invalid = 0;
              break;
            }
            if (risk.status == GridRiskStatus::VALID) ++valid;
            else ++invalid;
            samples.push_back({center, risk});
          }
        }
      }
    }
  }
  risk_slice_pub_->publish(makeCloud(samples, frame, stamp, risk_viz_metric_,
                                     risk_viz_hpl_max_m_, risk_viz_vpl_max_m_));
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.ns = "grid_map_risk";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  if (risk_odom_) {
    marker.pose.position = risk_odom_->pose.pose.position;
    marker.pose.position.z += 1.5;
  }
  marker.scale.z = 0.32;
  marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);
  std::ostringstream label;
  label << "Advisory spatial PL (frozen) " << risk_viz_metric_
        << " [m] v=" << version << " t=" << std::fixed << std::setprecision(2)
        << now << " z=" << slice_z << " valid=" << valid << "/"
        << (valid + invalid) << " incomplete=" << (truncated ? "yes" : "no")
        << " cost=" << std::setprecision(1) << elapsed_ms << "ms";
  if (!reason.empty()) label << " " << reason;
  marker.text = label.str();
  risk_status_pub_->publish(marker);
}

} // namespace ego_planner
