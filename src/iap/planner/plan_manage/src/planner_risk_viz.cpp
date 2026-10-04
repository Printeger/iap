#include <ego_planner/planner_manager.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace ego_planner {
namespace {

uint32_t riskColor(double value, double minimum, double maximum) {
  const double t = std::clamp((value - minimum) / (maximum - minimum), 0.0, 1.0);
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

std_msgs::msg::ColorRGBA markerColor(uint32_t packed, float alpha) {
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>((packed >> 16) & 0xffu) / 255.0f;
  color.g = static_cast<float>((packed >> 8) & 0xffu) / 255.0f;
  color.b = static_cast<float>(packed & 0xffu) / 255.0f;
  color.a = alpha;
  return color;
}

visualization_msgs::msg::Marker markerBase(
    const std::string& frame, const rclcpp::Time& stamp,
    const std::string& ns, int id, int type, double lifetime_s) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame;
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0;
  marker.lifetime = rclcpp::Duration::from_seconds(lifetime_s);
  return marker;
}

geometry_msgs::msg::Point markerPoint(const Eigen::Vector3d& point, double z_offset) {
  geometry_msgs::msg::Point out;
  out.x = point.x();
  out.y = point.y();
  out.z = point.z() + z_offset;
  return out;
}

struct SlicePoint {
  Eigen::Vector3d center;
  GridRiskVoxel risk;
};

sensor_msgs::msg::PointCloud2 makeCloud(
    const std::vector<SlicePoint>& samples, const std::string& frame,
    const rclcpp::Time& stamp, const std::string& metric,
    double hpl_min, double hpl_max, double vpl_min, double vpl_max) {
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
                    metric == "hpl" ? hpl_min : vpl_min,
                    metric == "hpl" ? hpl_max : vpl_max)
        : 0x9900ccu;
    std::memcpy(&*rgb, &packed, sizeof(packed));
    ++x; ++y; ++z; ++rgb; ++hpl; ++vpl; ++status;
  }
  return cloud;
}

void appendSurfaceTriangle(visualization_msgs::msg::Marker& marker,
                           const SlicePoint& a, const SlicePoint& b,
                           const SlicePoint& c, const std::string& metric,
                           double minimum, double maximum) {
  for (const SlicePoint* sample : {&a, &b, &c}) {
    marker.points.push_back(markerPoint(sample->center, -0.025));
    const double value = metric == "hpl" ? sample->risk.hpl : sample->risk.vpl;
    marker.colors.push_back(markerColor(riskColor(value, minimum, maximum), 0.72f));
  }
}

bool freeSurfaceCell(GridMap& map, const SlicePoint& a, const SlicePoint& b,
                     const SlicePoint& c, const SlicePoint& d) {
  const std::array<Eigen::Vector3d, 9> checks = {
      a.center, b.center, c.center, d.center,
      (a.center + b.center) * 0.5, (a.center + c.center) * 0.5,
      (b.center + d.center) * 0.5, (c.center + d.center) * 0.5,
      (a.center + d.center) * 0.5};
  return std::all_of(checks.begin(), checks.end(),
                     [&map](const Eigen::Vector3d& p) {
                       return map.getInflateOccupancy(p) == 0;
                     });
}

visualization_msgs::msg::MarkerArray makeSurface(
    const std::array<const SlicePoint*, 100>& lattice, GridMap& map,
    const std::string& frame, const rclcpp::Time& stamp,
    const std::string& metric, double minimum, double maximum) {
  auto surface = markerBase(frame, stamp, "risk_current_surface", 0,
                            visualization_msgs::msg::Marker::TRIANGLE_LIST, 1.5);
  surface.scale.x = surface.scale.y = surface.scale.z = 1.0;
  surface.color.a = 0.72;
  auto lines = markerBase(frame, stamp, "risk_sample_grid", 1,
                          visualization_msgs::msg::Marker::LINE_LIST, 1.5);
  lines.scale.x = 0.012;
  lines.color = markerColor(0xffffffu, 0.18f);
  for (int ix = 0; ix < 9; ++ix) {
    for (int iy = 0; iy < 9; ++iy) {
      const auto* a = lattice[ix * 10 + iy];
      const auto* b = lattice[(ix + 1) * 10 + iy];
      const auto* c = lattice[ix * 10 + iy + 1];
      const auto* d = lattice[(ix + 1) * 10 + iy + 1];
      if (!a || !b || !c || !d ||
          a->risk.status != GridRiskStatus::VALID ||
          b->risk.status != GridRiskStatus::VALID ||
          c->risk.status != GridRiskStatus::VALID ||
          d->risk.status != GridRiskStatus::VALID ||
          !freeSurfaceCell(map, *a, *b, *c, *d)) continue;
      appendSurfaceTriangle(surface, *a, *b, *d, metric, minimum, maximum);
      appendSurfaceTriangle(surface, *a, *d, *c, metric, minimum, maximum);
      for (const auto& edge : std::array<std::pair<const SlicePoint*, const SlicePoint*>, 4>{
               {{a, b}, {b, d}, {d, c}, {c, a}}}) {
        lines.points.push_back(markerPoint(edge.first->center, 0.01));
        lines.points.push_back(markerPoint(edge.second->center, 0.01));
      }
    }
  }
  if (surface.points.empty()) {
    surface.action = visualization_msgs::msg::Marker::DELETE;
    lines.action = visualization_msgs::msg::Marker::DELETE;
  }
  visualization_msgs::msg::MarkerArray array;
  array.markers = {std::move(surface), std::move(lines)};
  return array;
}

void appendBarVertex(visualization_msgs::msg::Marker& marker,
                     double x, double y, double z, uint32_t rgb) {
  geometry_msgs::msg::Point point;
  point.x = x; point.y = y; point.z = z;
  marker.points.push_back(point);
  marker.colors.push_back(markerColor(rgb, 0.95f));
}

visualization_msgs::msg::MarkerArray makeLegend(
    const std::string& frame, const rclcpp::Time& stamp,
    const Eigen::Vector3d& corner, const std::string& metric,
    double minimum, double maximum, double history_s) {
  visualization_msgs::msg::MarkerArray array;
  auto bar = markerBase(frame, stamp, "risk_legend", 0,
                        visualization_msgs::msg::Marker::TRIANGLE_LIST, 1.5);
  bar.scale.x = bar.scale.y = bar.scale.z = 1.0;
  for (int i = 0; i < 12; ++i) {
    const double x0 = corner.x() + 0.25 * i;
    const double x1 = x0 + 0.25;
    const double y0 = corner.y(), y1 = y0 + 0.20, z = corner.z();
    const uint32_t left = riskColor(minimum + (maximum - minimum) * i / 12.0,
                                    minimum, maximum);
    const uint32_t right = riskColor(minimum + (maximum - minimum) * (i + 1) / 12.0,
                                     minimum, maximum);
    appendBarVertex(bar, x0, y0, z, left);
    appendBarVertex(bar, x1, y0, z, right);
    appendBarVertex(bar, x1, y1, z, right);
    appendBarVertex(bar, x0, y0, z, left);
    appendBarVertex(bar, x1, y1, z, right);
    appendBarVertex(bar, x0, y1, z, left);
  }
  array.markers.push_back(std::move(bar));
  for (int i = 0; i < 3; ++i) {
    auto label = markerBase(frame, stamp, "risk_legend", i + 1,
                            visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.5);
    label.pose.position.x = corner.x() + 1.5 * i;
    label.pose.position.y = corner.y() - 0.26;
    label.pose.position.z = corner.z() + 0.05;
    label.scale.z = 0.24;
    label.color = markerColor(0xffffffu, 1.0f);
    std::ostringstream text;
    text << std::fixed << std::setprecision(2)
         << minimum + (maximum - minimum) * i / 2.0 << " m";
    label.text = text.str();
    array.markers.push_back(std::move(label));
  }
  auto title = markerBase(frame, stamp, "risk_legend", 4,
                          visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.5);
  title.pose.position.x = corner.x() + 1.5;
  title.pose.position.y = corner.y() - 0.55;
  title.pose.position.z = corner.z() + 0.05;
  title.scale.z = 0.18;
  title.color = markerColor(0xffffffu, 1.0f);
  std::ostringstream text;
  text << (metric == "hpl" ? "HPL" : "VPL")
       << ": dots predicted, surface interpolated, faded past "
       << std::fixed << std::setprecision(0) << history_s << " s";
  title.text = text.str();
  array.markers.push_back(std::move(title));
  return array;
}

} // namespace

void EGOPlannerManager::initRiskVisualization(const rclcpp::Node::SharedPtr& node) {
  risk_viz_enabled_ = node->declare_parameter("risk_viz/enabled", false);
  risk_viz_metric_ = node->declare_parameter<std::string>("risk_viz/metric", "hpl");
  risk_viz_z_mode_ = node->declare_parameter<std::string>("risk_viz/z_mode", "follow");
  risk_viz_fixed_z_m_ = node->declare_parameter("risk_viz/fixed_z_m", 1.5);
  risk_viz_hpl_min_m_ = node->declare_parameter("risk_viz/hpl_min_m", 0.25);
  risk_viz_hpl_max_m_ = node->declare_parameter("risk_viz/hpl_max_m", 0.65);
  risk_viz_vpl_min_m_ = node->declare_parameter("risk_viz/vpl_min_m", 0.4);
  risk_viz_vpl_max_m_ = node->declare_parameter("risk_viz/vpl_max_m", 1.2);
  risk_viz_history_lifetime_s_ = node->declare_parameter("risk_viz/history_lifetime_s", 60.0);
  risk_viz_history_step_m_ = node->declare_parameter("risk_viz/history_step_m", 4.0);
  if ((risk_viz_metric_ != "hpl" && risk_viz_metric_ != "vpl") ||
      (risk_viz_z_mode_ != "follow" && risk_viz_z_mode_ != "fixed") ||
      !std::isfinite(risk_viz_fixed_z_m_) ||
      !std::isfinite(risk_viz_hpl_min_m_) ||
      !std::isfinite(risk_viz_hpl_max_m_) ||
      risk_viz_hpl_max_m_ <= risk_viz_hpl_min_m_ ||
      !std::isfinite(risk_viz_vpl_min_m_) ||
      !std::isfinite(risk_viz_vpl_max_m_) ||
      risk_viz_vpl_max_m_ <= risk_viz_vpl_min_m_ ||
      !std::isfinite(risk_viz_history_lifetime_s_) || risk_viz_history_lifetime_s_ <= 0.0 ||
      !std::isfinite(risk_viz_history_step_m_) || risk_viz_history_step_m_ <= 0.0)
    throw std::invalid_argument("risk_viz parameters require hpl/vpl, follow/fixed and finite display/history ranges");
  if (!risk_viz_enabled_) return;

  risk_slice_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/risk_slice", 1);
  risk_status_pub_ = node->create_publisher<visualization_msgs::msg::Marker>("grid_map/risk_status", 1);
  risk_surface_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("grid_map/risk_surface", 1);
  risk_history_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("grid_map/risk_history", 1);
  risk_legend_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("grid_map/risk_legend", 1);
  glio_path_pub_ = node->create_publisher<nav_msgs::msg::Path>("grid_map/glio_path", 1);
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
          if (param.get_name() == "risk_viz/metric" && risk_viz_metric_ != param.as_string()) {
            risk_viz_metric_ = param.as_string();
            risk_history_clear_pending_ = true;
          }
        return result;
      });
  risk_viz_timer_ = node->create_wall_timer(std::chrono::seconds(1),
      [this]() { publishRiskSlice(); });
}

void EGOPlannerManager::updateGlioPath(const nav_msgs::msg::Odometry& odom) {
  if (!risk_viz_enabled_ || !glio_path_pub_ ||
      odom.header.frame_id != grid_map_->getFrameId()) return;
  const auto& position = odom.pose.pose.position;
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z)) return;
  if (!glio_path_.header.frame_id.empty() &&
      glio_path_.header.frame_id != odom.header.frame_id) glio_path_.poses.clear();
  glio_path_.header = odom.header;
  bool append = glio_path_.poses.empty();
  if (!append) {
    const auto& previous = glio_path_.poses.back().pose.position;
    append = std::hypot(std::hypot(position.x - previous.x,
                                   position.y - previous.y),
                        position.z - previous.z) >= 0.05;
  }
  if (append) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = odom.header;
    pose.pose = odom.pose.pose;
    glio_path_.poses.push_back(pose);
    if (glio_path_.poses.size() > 500) glio_path_.poses.erase(glio_path_.poses.begin());
  }
  if (++glio_path_publish_count_ % 5 == 0) glio_path_pub_->publish(glio_path_);
}

void EGOPlannerManager::publishRiskSlice() {
  if (!risk_viz_enabled_) return;
  const auto started = std::chrono::steady_clock::now();
  const auto stamp = node_->now();
  const double now = stamp.seconds();
  const std::string frame = grid_map_->getFrameId();
  std::vector<SlicePoint> samples;
  samples.reserve(100);
  std::array<const SlicePoint*, 100> lattice{};
  std::string reason;
  bool truncated = false;
  uint64_t version = 0;
  size_t valid = 0, invalid = 0;
  double slice_z = risk_viz_fixed_z_m_;
  double binding_ms = 0.0;

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
        const auto sample_started = std::chrono::steady_clock::now();
        binding_ms = std::chrono::duration<double, std::milli>(sample_started - started).count();
        for (int ix = 0; ix < 10 && reason.empty() && !truncated; ++ix) {
          for (int iy = 0; iy < 10; ++iy) {
            if (std::chrono::steady_clock::now() - sample_started >= std::chrono::milliseconds(20)) {
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
            if (grid_map_->queryOccupancyDiagnostic(center).raw_occupied) continue;
            const auto risk = grid_map_->queryRisk(center, version, node_->now().seconds());
            if (risk.status == GridRiskStatus::VERSION_CHANGED ||
                risk.status == GridRiskStatus::STALE ||
                risk.status == GridRiskStatus::FRAME_MISMATCH ||
                risk.status == GridRiskStatus::UNCOMPUTED) {
              reason = "prediction context unavailable (status=" +
                  std::to_string(static_cast<int>(risk.status)) + ")";
              samples.clear();
              lattice.fill(nullptr);
              valid = invalid = 0;
              break;
            }
            if (risk.status == GridRiskStatus::VALID) ++valid;
            else ++invalid;
            samples.push_back({center, risk});
            lattice[ix * 10 + iy] = &samples.back();
          }
        }
      }
    }
  }
  risk_slice_pub_->publish(makeCloud(samples, frame, stamp, risk_viz_metric_,
                                     risk_viz_hpl_min_m_, risk_viz_hpl_max_m_,
                                     risk_viz_vpl_min_m_, risk_viz_vpl_max_m_));
  const double minimum = risk_viz_metric_ == "hpl" ? risk_viz_hpl_min_m_ : risk_viz_vpl_min_m_;
  const double maximum = risk_viz_metric_ == "hpl" ? risk_viz_hpl_max_m_ : risk_viz_vpl_max_m_;
  auto surface = makeSurface(lattice, *grid_map_, frame, stamp,
                             risk_viz_metric_, minimum, maximum);
  const size_t tiles = surface.markers.front().points.size() / 6;
  risk_surface_pub_->publish(surface);

  if (!reason.empty() || valid == 0 || risk_history_clear_pending_) {
    visualization_msgs::msg::MarkerArray clear;
    auto marker = markerBase(frame, stamp, "risk_history", 0,
                             visualization_msgs::msg::Marker::TRIANGLE_LIST, 0.0);
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    clear.markers.push_back(marker);
    risk_history_pub_->publish(clear);
    last_history_position_ = Eigen::Vector3d::Constant(
        std::numeric_limits<double>::quiet_NaN());
    risk_history_clear_pending_ = false;
  }
  if (tiles > 0 && risk_odom_ &&
      risk_odom_->header.frame_id == frame &&
      std::isfinite(risk_odom_->pose.pose.position.x) &&
      std::isfinite(risk_odom_->pose.pose.position.y)) {
    const auto& position = risk_odom_->pose.pose.position;
    Eigen::Vector3d current(position.x, position.y, slice_z);
    if (!last_history_position_.allFinite() ||
        (current.head<2>() - last_history_position_.head<2>()).norm() >=
            risk_viz_history_step_m_) {
      auto past = surface.markers.front();
      past.ns = "risk_history";
      past.id = ++risk_history_id_;
      past.lifetime = rclcpp::Duration::from_seconds(risk_viz_history_lifetime_s_);
      past.color.a = 0.18;
      for (auto& color : past.colors) color.a = 0.18f;
      visualization_msgs::msg::MarkerArray history;
      history.markers.push_back(std::move(past));
      risk_history_pub_->publish(history);
      last_history_position_ = current;
    }
  }
  if (risk_odom_ && std::isfinite(risk_odom_->pose.pose.position.x) &&
      std::isfinite(risk_odom_->pose.pose.position.y)) {
    const auto& position = risk_odom_->pose.pose.position;
    const Eigen::Vector3d corner(position.x - 4.7, position.y - 5.2, slice_z + 0.08);
    risk_legend_pub_->publish(makeLegend(frame, stamp, corner, risk_viz_metric_,
                                          minimum, maximum, risk_viz_history_lifetime_s_));
  }
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
  if (current_integrity_.valid && current_integrity_.integrity_state == 2) {
    marker.color.g = marker.color.b = 0.2;
  }
  marker.lifetime = rclcpp::Duration::from_seconds(1.5);
  std::ostringstream label;
  label << "Advisory spatial PL (frozen) " << risk_viz_metric_
        << " [m] v=" << version << " t=" << std::fixed << std::setprecision(2)
        << now << " z=" << slice_z << " valid=" << valid << "/"
        << (valid + invalid) << " incomplete=" << (truncated ? "yes" : "no")
        << " tiles=" << tiles << " display=" << minimum << ".." << maximum << "m"
        << " cost=" << std::setprecision(1) << elapsed_ms << "ms"
        << " bind=" << binding_ms << "ms";
  if (current_integrity_.valid)
    label << " monitor_state=" << static_cast<int>(current_integrity_.integrity_state)
          << " monitor_HPL/VPL=" << current_integrity_.hpl << "/"
          << current_integrity_.vpl;
  if (!reason.empty()) label << " " << reason;
  marker.text = label.str();
  risk_status_pub_->publish(marker);
}

} // namespace ego_planner
