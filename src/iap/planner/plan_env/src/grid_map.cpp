#include "plan_env/grid_map.h"

#include <algorithm>
#include <exception>
#include <iomanip>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sstream>
#include <stdexcept>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

namespace
{

constexpr auto kRegisteredRecoveryControlPeriod =
    std::chrono::milliseconds(250);

int64_t steadyNowNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool sourceStampSeconds(const builtin_interfaces::msg::Time &stamp,
                        double *stamp_s)
{
  if (stamp.sec < 0 || stamp.nanosec >= 1000000000U || stamp_s == nullptr)
    return false;
  *stamp_s = static_cast<double>(stamp.sec) +
             static_cast<double>(stamp.nanosec) * 1e-9;
  return std::isfinite(*stamp_s) && *stamp_s > 0.0;
}

std::string geometryIdentity(const std::string &frame_id,
                             const Eigen::Vector3d &origin,
                             const Eigen::Vector3i &dimensions,
                             const double resolution,
                             const Eigen::Vector3d* extent = nullptr)
{
  std::ostringstream canonical;
  canonical << std::setprecision(17) << frame_id << '|'
            << origin.x() << ',' << origin.y() << ',' << origin.z() << '|'
            << dimensions.x() << ',' << dimensions.y() << ','
            << dimensions.z() << '|' << resolution;
  if (extent) canonical << '|' << extent->x() << ',' << extent->y() << ',' << extent->z();
  // Stable FNV-1a is sufficient for an identity token; the canonical fields
  // remain present alongside it and are always validated independently.
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : canonical.str())
  {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  std::ostringstream out;
  out << "planning_lattice_v1:" << std::hex << std::setw(16)
      << std::setfill('0') << hash;
  return out.str();
}

Eigen::Isometry3d poseFromMessage(const geometry_msgs::msg::Pose &message)
{
  Eigen::Quaterniond quaternion(
      message.orientation.w, message.orientation.x,
      message.orientation.y, message.orientation.z);
  if (!quaternion.coeffs().allFinite() || quaternion.norm() < 1.0e-9)
    return Eigen::Isometry3d(Eigen::Matrix4d::Constant(
        std::numeric_limits<double>::quiet_NaN()));
  quaternion.normalize();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(
      message.position.x, message.position.y, message.position.z);
  pose.linear() = quaternion.toRotationMatrix();
  return pose;
}

bool registeredFrameFromMessage(
    const iap::msg::RegisteredLidarFrame &message,
    const std::string &expected_planner_frame,
    const std::string &expected_lidar_frame,
    RegisteredLidarFrameData *frame)
{
  if (frame == nullptr)
    return false;
  double stamp_s = std::numeric_limits<double>::quiet_NaN();
  double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
  if (message.header.frame_id != expected_planner_frame ||
      message.deskewed_hits_lidar.header.frame_id != expected_lidar_frame ||
      !sourceStampSeconds(message.header.stamp, &stamp_s) ||
      !sourceStampSeconds(message.deskewed_hits_lidar.header.stamp,
                          &cloud_stamp_s) ||
      std::abs(stamp_s - cloud_stamp_s) > 1.0e-6 ||
      !std::isfinite(message.scan_end_stamp_s) ||
      message.scan_end_stamp_s + 1.0e-9 < stamp_s ||
      message.sensor_receipt_steady_ns == 0U)
    return false;
  const auto has_xyz_field = [&message](const std::string &name)
  {
    return std::any_of(
        message.deskewed_hits_lidar.fields.begin(),
        message.deskewed_hits_lidar.fields.end(),
        [&name](const sensor_msgs::msg::PointField &field)
        {
          return field.name == name && field.count == 1U &&
              field.datatype == sensor_msgs::msg::PointField::FLOAT32;
        });
  };
  if (!has_xyz_field("x") || !has_xyz_field("y") ||
      !has_xyz_field("z"))
    return false;
  frame->frame_id = message.frame_id;
  frame->stamp_s = stamp_s;
  frame->scan_end_stamp_s = message.scan_end_stamp_s;
  frame->sensor_receipt_steady_ns = message.sensor_receipt_steady_ns;
  frame->T_map_lidar = poseFromMessage(message.t_map_lidar);
  frame->frame_contract_id = message.frame_contract_id;
  frame->sensor_model_id = message.sensor_model_id;
  frame->horizontal_samples = message.horizontal_samples;
  frame->vertical_samples = message.vertical_samples;
  frame->horizontal_fov_rad = message.horizontal_fov_rad;
  frame->vertical_min_rad = message.vertical_min_rad;
  frame->vertical_max_rad = message.vertical_max_rad;
  frame->min_range_m = message.min_range_m;
  frame->max_range_m = message.max_range_m;
  frame->beam_evidence_complete = message.beam_evidence_complete;
  frame->beam_content_hash = message.beam_content_hash;
  frame->beam_binding_reason = message.beam_binding_reason;
  frame->beam_received_count = message.beam_received_count;
  frame->beam_invalid_count = message.beam_invalid_count;
  frame->beam_evicted_count = message.beam_evicted_count;
  frame->beam_history_oldest_stamp_s = message.beam_history_oldest_stamp_s;
  frame->beam_history_newest_stamp_s = message.beam_history_newest_stamp_s;
  frame->beam_same_start_end_stamp_s = message.beam_same_start_end_stamp_s;
  if (message.beam_evidence_complete)
  {
    const std::size_t count = message.beam_outcomes.size();
    const std::size_t expected =
        static_cast<std::size_t>(message.horizontal_samples) *
        static_cast<std::size_t>(message.vertical_samples);
    if (count != expected || message.beam_direction_x.size() != count ||
        message.beam_direction_y.size() != count ||
        message.beam_direction_z.size() != count ||
        message.beam_ranges_m.size() != count ||
        message.sensor_model_id.empty() || message.beam_content_hash.empty())
      return false;
    frame->beams.reserve(count);
    for (std::size_t index = 0; index < count; ++index)
    {
      RegisteredLidarBeamData beam;
      beam.direction_lidar = Eigen::Vector3d(
          message.beam_direction_x[index], message.beam_direction_y[index],
          message.beam_direction_z[index]);
      beam.outcome = static_cast<RegisteredLidarBeamOutcome>(
          message.beam_outcomes[index]);
      beam.range_m = message.beam_ranges_m[index];
      frame->beams.push_back(std::move(beam));
    }
  }
  frame->source_is_map_reference = message.source_is_map_reference;
  frame->source_health_valid = message.source_health_valid;
  frame->source_health_stamp_s = message.source_health_stamp_s;
  frame->source_icp_degenerate = message.source_icp_degenerate;
  frame->source_icp_rmse = message.source_icp_rmse;
  frame->source_icp_condition = message.source_icp_condition;
  frame->source_icp_gamma_lidar = message.source_icp_gamma_lidar;
  frame->source_lidar_pl_enu_m = Eigen::Vector3d(
      message.source_lidar_pl_e, message.source_lidar_pl_n,
      message.source_lidar_pl_u);
  if (frame->source_health_valid &&
      (!std::isfinite(frame->source_health_stamp_s) ||
       std::abs(frame->source_health_stamp_s - stamp_s) > 0.20 ||
       frame->source_icp_degenerate ||
       !std::isfinite(frame->source_icp_rmse) ||
       !std::isfinite(frame->source_icp_condition) ||
       !std::isfinite(frame->source_icp_gamma_lidar) ||
       frame->source_icp_rmse < 0.0 || frame->source_icp_condition < 0.0 ||
       frame->source_icp_gamma_lidar < 1.0))
    return false;
  try
  {
    const std::size_t point_count =
        static_cast<std::size_t>(message.deskewed_hits_lidar.width) *
        static_cast<std::size_t>(message.deskewed_hits_lidar.height);
    frame->hits_lidar.clear();
    frame->hits_lidar.reserve(point_count);
    sensor_msgs::PointCloud2ConstIterator<float> x(
        message.deskewed_hits_lidar, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(
        message.deskewed_hits_lidar, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(
        message.deskewed_hits_lidar, "z");
    for (std::size_t index = 0; index < point_count;
         ++index, ++x, ++y, ++z)
    {
      const Eigen::Vector3d hit(*x, *y, *z);
      if (!hit.allFinite())
        return false;
      frame->hits_lidar.push_back(hit);
    }
  }
  catch (const std::exception &)
  {
    return false;
  }
  return frame->frame_id >= 0 &&
      frame->T_map_lidar.matrix().allFinite();
}

}  // namespace

void GridMap::initMap(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  /* get parameter */
  double x_size, y_size, z_size;
  node_->declare_parameter("grid_map/resolution", -1.0);
  node_->declare_parameter("grid_map/map_size_x", -1.0);
  node_->declare_parameter("grid_map/map_size_y", -1.0);
  node_->declare_parameter("grid_map/map_size_z", -1.0);
  node_->declare_parameter("grid_map/local_update_range_x", -1.0);
  node_->declare_parameter("grid_map/local_update_range_y", -1.0);
  node_->declare_parameter("grid_map/local_update_range_z", -1.0);
  node_->declare_parameter("grid_map/obstacles_inflation", -1.0);
  node_->declare_parameter("grid_map/fx", -1.0);
  node_->declare_parameter("grid_map/fy", -1.0);
  node_->declare_parameter("grid_map/cx", -1.0);
  node_->declare_parameter("grid_map/cy", -1.0);
  node_->declare_parameter("grid_map/use_depth_filter", true);
  node_->declare_parameter("grid_map/depth_filter_tolerance", -1.0);
  node_->declare_parameter("grid_map/depth_filter_maxdist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_mindist", -1.0);
  node_->declare_parameter("grid_map/depth_filter_margin", -1);
  node_->declare_parameter("grid_map/k_depth_scaling_factor", -1.0);
  node_->declare_parameter("grid_map/skip_pixel", -1);
  node_->declare_parameter("grid_map/p_hit", 0.70);
  node_->declare_parameter("grid_map/p_miss", 0.35);
  node_->declare_parameter("grid_map/p_min", 0.12);
  node_->declare_parameter("grid_map/p_max", 0.97);
  node_->declare_parameter("grid_map/p_occ", 0.80);
  node_->declare_parameter("grid_map/min_ray_length", -0.1);
  node_->declare_parameter("grid_map/max_ray_length", -0.1);
  node_->declare_parameter("grid_map/visualization_truncate_height", -0.1);
  node_->declare_parameter("grid_map/visualization_period_s", 0.11);
  node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yp", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yn", -0.1);
  node_->declare_parameter("grid_map/show_occ_time", false);
  node_->declare_parameter("grid_map/pose_type", 1);
  node_->declare_parameter("grid_map/frame_id", "world");
  node_->declare_parameter("grid_map/local_map_margin", 1);
  node_->declare_parameter("grid_map/ground_height", 1.0);
  node_->declare_parameter("grid_map/origin_x",
                           std::numeric_limits<double>::quiet_NaN());
  node_->declare_parameter("grid_map/origin_y",
                           std::numeric_limits<double>::quiet_NaN());
  node_->declare_parameter("grid_map/origin_z",
                           std::numeric_limits<double>::quiet_NaN());
  node_->declare_parameter("grid_map/unknown_as_occupied", false);
  node_->declare_parameter("grid_map/odom_depth_timeout", 1.0);
  node_->declare_parameter("grid_map/independent_cloud_min_interval_s", 0.0);
  node_->declare_parameter("grid_map/independent_cloud_clock_guard_s", 0.0);
  node_->declare_parameter("grid_map/registered_lidar_window_enabled", false);
  node_->declare_parameter(
      "grid_map/registered_frame_contract_id", std::string(""));
  node_->declare_parameter(
      "grid_map/registered_current_topic",
      std::string("/iap/local_map/current_frame"));
  node_->declare_parameter(
      "grid_map/registered_delta_topic",
      std::string("/iap/local_map/window_delta"));
  node_->declare_parameter(
      "grid_map/registered_recovery_service",
      std::string("/iap/local_map/get_active_window"));
  node_->declare_parameter(
      "grid_map/registered_lidar_reference_frame_id",
      std::string("iap_lidar_reference"));
  node_->declare_parameter("grid_map/trusted_local_map_support_enabled", true);
  node_->declare_parameter("grid_map/trusted_support_min_range_m", 0.1);
  node_->declare_parameter("grid_map/trusted_support_max_range_m", 10.0);
  node_->declare_parameter("grid_map/trusted_support_horizontal_fov_deg", 360.0);
  node_->declare_parameter("grid_map/trusted_support_vertical_min_deg", -7.0);
  node_->declare_parameter("grid_map/trusted_support_vertical_max_deg", 52.0);
  node_->declare_parameter("grid_map/trusted_support_validity_s", 1.0);
  node_->declare_parameter(
      "grid_map/trusted_support_model_version",
      std::string("trusted_local_map_v1"));

  node_->get_parameter("grid_map/resolution", mp_.resolution_);
  node_->get_parameter("grid_map/map_size_x", x_size);
  node_->get_parameter("grid_map/map_size_y", y_size);
  node_->get_parameter("grid_map/map_size_z", z_size);
  node_->get_parameter("grid_map/local_update_range_x", mp_.local_update_range_(0));
  node_->get_parameter("grid_map/local_update_range_y", mp_.local_update_range_(1));
  node_->get_parameter("grid_map/local_update_range_z", mp_.local_update_range_(2));
  node_->get_parameter("grid_map/obstacles_inflation", mp_.obstacles_inflation_);
  node_->get_parameter("grid_map/fx", mp_.fx_);
  node_->get_parameter("grid_map/fy", mp_.fy_);
  node_->get_parameter("grid_map/cx", mp_.cx_);
  node_->get_parameter("grid_map/cy", mp_.cy_);
  node_->get_parameter("grid_map/use_depth_filter", mp_.use_depth_filter_);
  node_->get_parameter("grid_map/depth_filter_tolerance", mp_.depth_filter_tolerance_);
  node_->get_parameter("grid_map/depth_filter_maxdist", mp_.depth_filter_maxdist_);
  node_->get_parameter("grid_map/depth_filter_mindist", mp_.depth_filter_mindist_);
  node_->get_parameter("grid_map/depth_filter_margin", mp_.depth_filter_margin_);
  node_->get_parameter("grid_map/k_depth_scaling_factor", mp_.k_depth_scaling_factor_);
  node_->get_parameter("grid_map/skip_pixel", mp_.skip_pixel_);
  node_->get_parameter("grid_map/p_hit", mp_.p_hit_);
  node_->get_parameter("grid_map/p_miss", mp_.p_miss_);
  node_->get_parameter("grid_map/p_min", mp_.p_min_);
  node_->get_parameter("grid_map/p_max", mp_.p_max_);
  node_->get_parameter("grid_map/p_occ", mp_.p_occ_);
  node_->get_parameter("grid_map/min_ray_length", mp_.min_ray_length_);
  node_->get_parameter("grid_map/max_ray_length", mp_.max_ray_length_);
  node_->get_parameter("grid_map/visualization_truncate_height", mp_.visualization_truncate_height_);
  double visualization_period_s = 0.11;
  node_->get_parameter("grid_map/visualization_period_s", visualization_period_s);
  if (!std::isfinite(visualization_period_s) || visualization_period_s <= 0.0)
    throw std::invalid_argument("grid_map/visualization_period_s must be positive and finite");
  node_->get_parameter("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_);
  node_->get_parameter("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_);
  node_->get_parameter("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_);
  node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
  node_->get_parameter("grid_map/pose_type", mp_.pose_type_);
  node_->get_parameter("grid_map/frame_id", mp_.frame_id_);
  node_->get_parameter("grid_map/local_map_margin", mp_.local_map_margin_);
  node_->get_parameter("grid_map/ground_height", mp_.ground_height_);
  double origin_x = std::numeric_limits<double>::quiet_NaN();
  double origin_y = std::numeric_limits<double>::quiet_NaN();
  double origin_z = std::numeric_limits<double>::quiet_NaN();
  node_->get_parameter("grid_map/origin_x", origin_x);
  node_->get_parameter("grid_map/origin_y", origin_y);
  node_->get_parameter("grid_map/origin_z", origin_z);
  node_->get_parameter("grid_map/unknown_as_occupied",
                       mp_.unknown_as_occupied_);
  node_->get_parameter("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_);
  node_->get_parameter("grid_map/independent_cloud_min_interval_s",
                       mp_.independent_cloud_min_interval_s_);
  node_->get_parameter("grid_map/independent_cloud_clock_guard_s",
                       mp_.independent_cloud_clock_guard_s_);
  node_->get_parameter("grid_map/registered_lidar_window_enabled",
                       registered_lidar_window_enabled_);
  node_->get_parameter("grid_map/registered_frame_contract_id",
                       registered_frame_contract_id_);
  node_->get_parameter("grid_map/registered_current_topic",
                       registered_current_topic_);
  node_->get_parameter("grid_map/registered_delta_topic",
                       registered_delta_topic_);
  node_->get_parameter("grid_map/registered_recovery_service",
                       registered_recovery_service_);
  node_->get_parameter("grid_map/registered_lidar_reference_frame_id",
                       registered_lidar_reference_frame_id_);
  node_->get_parameter("grid_map/trusted_local_map_support_enabled",
                       trusted_local_map_support_enabled_);
  node_->get_parameter("grid_map/trusted_support_min_range_m",
                       trusted_support_min_range_m_);
  node_->get_parameter("grid_map/trusted_support_max_range_m",
                       trusted_support_max_range_m_);
  node_->get_parameter("grid_map/trusted_support_horizontal_fov_deg",
                       trusted_support_horizontal_fov_deg_);
  node_->get_parameter("grid_map/trusted_support_vertical_min_deg",
                       trusted_support_vertical_min_deg_);
  node_->get_parameter("grid_map/trusted_support_vertical_max_deg",
                       trusted_support_vertical_max_deg_);
  node_->get_parameter("grid_map/trusted_support_validity_s",
                       trusted_support_validity_s_);
  node_->get_parameter("grid_map/trusted_support_model_version",
                       trusted_support_model_version_);
  mp_.independent_cloud_min_interval_s_ =
      std::max(0.0, mp_.independent_cloud_min_interval_s_);
  mp_.independent_cloud_clock_guard_s_ =
      std::max(0.0, mp_.independent_cloud_clock_guard_s_);
  if (registered_lidar_window_enabled_ && trusted_local_map_support_enabled_ &&
      (!std::isfinite(trusted_support_min_range_m_) ||
       !std::isfinite(trusted_support_max_range_m_) ||
       trusted_support_min_range_m_ < 0.0 ||
       trusted_support_max_range_m_ <= trusted_support_min_range_m_ ||
       !std::isfinite(trusted_support_horizontal_fov_deg_) ||
       trusted_support_horizontal_fov_deg_ <= 0.0 ||
       trusted_support_horizontal_fov_deg_ > 360.0 ||
       !std::isfinite(trusted_support_vertical_min_deg_) ||
       !std::isfinite(trusted_support_vertical_max_deg_) ||
       trusted_support_vertical_min_deg_ >= trusted_support_vertical_max_deg_ ||
       !std::isfinite(trusted_support_validity_s_) ||
       trusted_support_validity_s_ < 0.0 ||
       trusted_support_model_version_.empty()))
    throw std::runtime_error("invalid trusted local map support parameters");
  RCLCPP_INFO(node_->get_logger(),
              "[grid_map] independent cloud interval=%.3f s "
              "clock_guard=%.3f s",
              mp_.independent_cloud_min_interval_s_,
              mp_.independent_cloud_clock_guard_s_);

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  const bool explicit_origin = std::isfinite(origin_x) &&
      std::isfinite(origin_y) && std::isfinite(origin_z);
  mp_.map_origin_ = explicit_origin
      ? Eigen::Vector3d(origin_x, origin_y, origin_z)
      : Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0,
                        mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.risk_buffer_.resize(buffer_size);
  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_raw_cloud_ = vector<char>(buffer_size, 0);
  md_.observed_buffer_ = vector<char>(buffer_size, 0);

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  if (registered_lidar_window_enabled_)
  {
    if (registered_frame_contract_id_.empty())
      throw std::runtime_error(
          "grid_map/registered_frame_contract_id must not be empty");
    RegisteredLidarWindow::Geometry geometry;
    geometry.origin = mp_.map_origin_;
    geometry.dimensions = mp_.map_voxel_num_;
    geometry.resolution_m = mp_.resolution_;
    geometry.frame_contract_id = registered_frame_contract_id_;
    registered_lidar_window_ =
        std::make_unique<RegisteredLidarWindow>(std::move(geometry));
    registered_inflation_dirty_bits_.assign(
        (md_.occupancy_buffer_raw_cloud_.size() + 63U) / 64U, 0U);
    registered_raw_inflation_count_.assign(
        md_.occupancy_buffer_raw_cloud_.size(), 0U);
    if (mp_.virtual_ceil_height_ > -0.5)
    {
      const int ceil_id = static_cast<int>(std::floor(
          (mp_.virtual_ceil_height_ - mp_.map_origin_(2)) *
          mp_.resolution_inv_)) - 1;
      if (ceil_id >= 0 && ceil_id < mp_.map_voxel_num_(2))
        for (int x = 0; x < mp_.map_voxel_num_(0); ++x)
          for (int y = 0; y < mp_.map_voxel_num_(1); ++y)
          {
            int z = ceil_id;
            md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 1;
          }
    }
  }

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  // The registered-window seam is the sole occupancy/evidence producer in
  // GLIM mode.  Keeping the legacy depth synchronizer alive here would mix a
  // second pose authority into the same FrozenOccupancyEpoch.
  if (!registered_lidar_window_enabled_)
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
        node_, "grid_map/depth", rclcpp::QoS(50).get_rmw_qos_profile());

    extrinsic_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/vins_estimator/extrinsic", 10,
        std::bind(&GridMap::extrinsicCallback, this, std::placeholders::_1));

    if (mp_.pose_type_ == POSE_STAMPED)
    {
      pose_sub_ = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(
          node_, "grid_map/pose", rclcpp::QoS(25).get_rmw_qos_profile());

      sync_image_pose_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImagePose>>(
          SyncPolicyImagePose(100), *depth_sub_, *pose_sub_);
      sync_image_pose_->registerCallback(
          std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else if (mp_.pose_type_ == ODOMETRY)
    {
      odom_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(
          node_, "grid_map/odom", rclcpp::QoS(100).get_rmw_qos_profile());

      sync_image_odom_ = std::make_shared<message_filters::Synchronizer<SyncPolicyImageOdom>>(
          SyncPolicyImageOdom(100), *depth_sub_, *odom_sub_);
      sync_image_odom_->registerCallback(
          std::bind(&GridMap::depthOdomCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

  // Keep occupancy production independent from the planner FSM callback
  // group. P4 replanning can otherwise delay the authoritative 2 Hz cloud
  // long enough for P0 to correctly reject it as stale.
  independent_cloud_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  independent_cloud_input_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  independent_odom_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  registered_current_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  registered_control_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions independent_cloud_options;
  independent_cloud_options.callback_group =
      independent_cloud_input_callback_group_;
  rclcpp::SubscriptionOptions independent_odom_options;
  independent_odom_options.callback_group = independent_odom_callback_group_;
  rclcpp::SubscriptionOptions registered_current_options;
  registered_current_options.callback_group =
      registered_current_callback_group_;
  rclcpp::SubscriptionOptions registered_control_options;
  registered_control_options.callback_group =
      registered_control_callback_group_;

  // 使用独立的里程计和点云订阅
  if (!registered_lidar_window_enabled_)
  {
    indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        "grid_map/cloud", rclcpp::SensorDataQoS().keep_last(1),
        [this](const sensor_msgs::msg::PointCloud2::ConstPtr &message)
        {
          if (mp_.independent_cloud_min_interval_s_ > 0.0)
            independentCloudInputCallback(message);
          else
            cloudCallback(message);
        },
        independent_cloud_options);
  }
  else
  {
    registered_current_sub_ =
        node_->create_subscription<iap::msg::RegisteredLidarFrame>(
            registered_current_topic_,
            rclcpp::SensorDataQoS().keep_last(1),
            std::bind(&GridMap::registeredCurrentFrameCallback, this,
                      std::placeholders::_1),
            registered_current_options);
    registered_delta_sub_ =
        node_->create_subscription<iap::msg::ActiveLidarWindowDelta>(
            registered_delta_topic_, rclcpp::QoS(128).reliable(),
            std::bind(&GridMap::registeredWindowDeltaCallback, this,
                      std::placeholders::_1),
            registered_control_options);
    registered_recovery_client_ =
        node_->create_client<iap::srv::GetActiveLidarWindow>(
            registered_recovery_service_, rclcpp::ServicesQoS(),
            registered_control_callback_group_);
    const std::weak_ptr<GridMap> recovery_owner = weak_from_this();
    registered_recovery_client_->set_on_new_response_callback(
        [recovery_owner](const std::size_t response_count)
        {
          if (response_count == 0U)
            return;
          const auto self = recovery_owner.lock();
          if (!self || !self->registered_recovery_in_flight_.load(
                           std::memory_order_acquire))
            return;
          int64_t not_received = 0;
          self->registered_recovery_response_ready_ns_.compare_exchange_strong(
              not_received, steadyNowNanoseconds(),
              std::memory_order_acq_rel, std::memory_order_acquire);
        });
    registered_recovery_pending_.store(true, std::memory_order_release);
    registered_recovery_timer_ = node_->create_wall_timer(
        kRegisteredRecoveryControlPeriod,
        std::bind(&GridMap::maintainRegisteredWindowRecovery, this),
        registered_control_callback_group_);
    RCLCPP_INFO(
        node_->get_logger(),
        "[grid_map] registered LiDAR window enabled current=%s delta=%s "
        "recovery=%s contract=%s",
        registered_current_topic_.c_str(), registered_delta_topic_.c_str(),
        registered_recovery_service_.c_str(),
        registered_frame_contract_id_.c_str());
  }

  if (!registered_lidar_window_enabled_ &&
      mp_.independent_cloud_min_interval_s_ > 0.0)
  {
    independent_cloud_timer_ = node_->create_wall_timer(
        std::chrono::duration<double>(
            mp_.independent_cloud_min_interval_s_),
        std::bind(&GridMap::processLatestIndependentCloud, this),
        independent_cloud_callback_group_);
  }

  indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "grid_map/odom", 10,
      std::bind(&GridMap::odomCallback, this, std::placeholders::_1),
      independent_odom_options);

  // 定时器
  occ_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.05),
      std::bind(&GridMap::updateOccupancyCallback, this));

  vis_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(visualization_period_s),
      std::bind(&GridMap::visCallback, this));

  // 发布者
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  md_.pending_depth_source_stamp_s_ =
      std::numeric_limits<double>::quiet_NaN();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;

  md_.flag_depth_odom_timeout_ = false;
  md_.flag_use_depth_fusion = false;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::resetBuffer()
{
  Eigen::Vector3d min_pos = mp_.map_min_boundary_;
  Eigen::Vector3d max_pos = mp_.map_max_boundary_;

  resetBuffer(min_pos, max_pos);

  md_.local_bound_min_ = Eigen::Vector3i::Zero();
  md_.local_bound_max_ = mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  std::unique_lock<std::mutex> lock(occupancy_epoch_mutex_);
  beginOccupancyWriteTransaction();
  resetBufferUnlocked(min_pos, max_pos);
  const auto notification = commitOccupancyWriteTransaction(
      occupancy_cloud_stamp_s_.load(std::memory_order_acquire));
  lock.unlock();
  notifyOccupancyCommitted(notification);
}

void GridMap::resetBufferUnlocked(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  invalidateRiskContext();

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        const int address = toAddress(x, y, z);
        if (md_.occupancy_buffer_raw_cloud_[address] != 0)
          recordCollisionStateBeforeMutation(address);
        md_.occupancy_buffer_inflate_[address] = 0;
        md_.occupancy_buffer_raw_cloud_[address] = 0;
        // A return-only PointCloud2 frame carries no evidence about voxels
        // that were not hit or explicitly ray-traversed in this frame.  Drop
        // the old observation bit together with the old raw hit so a missing
        // return becomes UNKNOWN, never OBSERVED_FREE by inheritance.
        md_.observed_buffer_[address] = 0;
      }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

  if (idx_ctns >= 0 &&
      idx_ctns < static_cast<int>(md_.observed_buffer_.size()))
    md_.observed_buffer_[static_cast<std::size_t>(idx_ctns)] = 1;

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;
  int skip_pix = mp_.skip_pixel_;

  double depth;

  Eigen::Matrix3d camera_r = md_.camera_r_m_;

  if (!mp_.use_depth_filter_)
  {
    for (int v = 0; v < rows; v += skip_pix)
    {
      row_ptr = md_.depth_image_.ptr<uint16_t>(v);

      for (int u = 0; u < cols; u += skip_pix)
      {

        Eigen::Vector3d proj_pt;
        depth = (*row_ptr++) / mp_.k_depth_scaling_factor_;
        proj_pt(0) = (u - mp_.cx_) * depth / mp_.fx_;
        proj_pt(1) = (v - mp_.cy_) * depth / mp_.fy_;
        proj_pt(2) = depth;

        proj_pt = camera_r * proj_pt + md_.camera_pos_;

        if (u == 320 && v == 240)
          std::cout << "depth: " << depth << std::endl;
        md_.proj_points_[md_.proj_points_cnt++] = proj_pt;
      }
    }
  }
  /* use depth filter */
  else
  {

    if (!md_.has_first_depth_)
      md_.has_first_depth_ = true;
    else
    {
      Eigen::Vector3d pt_cur, pt_world, pt_reproj;

      Eigen::Matrix3d last_camera_r_inv;
      last_camera_r_inv = md_.last_camera_r_m_.inverse();
      const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

      for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
      {
        row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

        for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_;
             u += mp_.skip_pixel_)
        {

          depth = (*row_ptr) * inv_factor;
          row_ptr = row_ptr + mp_.skip_pixel_;

          // filter depth
          // depth += rand_noise_(eng_);
          // if (depth > 0.01) depth += rand_noise2_(eng_);

          if (*row_ptr == 0)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }
          else if (depth < mp_.depth_filter_mindist_)
          {
            continue;
          }
          else if (depth > mp_.depth_filter_maxdist_)
          {
            depth = mp_.max_ray_length_ + 0.1;
          }

          // project to world frame
          pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
          pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
          pt_cur(2) = depth;

          pt_world = camera_r * pt_cur + md_.camera_pos_;
          // if (!isInMap(pt_world)) {
          //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
          // }

          md_.proj_points_[md_.proj_points_cnt++] = pt_world;

          // check consistency with last image, disabled...
          if (false)
          {
            pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos_);
            double uu = pt_reproj.x() * mp_.fx_ / pt_reproj.z() + mp_.cx_;
            double vv = pt_reproj.y() * mp_.fy_ / pt_reproj.z() + mp_.cy_;

            if (uu >= 0 && uu < cols && vv >= 0 && vv < rows)
            {
              if (fabs(md_.last_depth_image_.at<uint16_t>((int)vv, (int)uu) * inv_factor -
                       pt_reproj.z()) < mp_.depth_filter_tolerance_)
              {
                md_.proj_points_[md_.proj_points_cnt++] = pt_world;
              }
            }
            else
            {
              md_.proj_points_[md_.proj_points_cnt++] = pt_world;
            }
          }
        }
      }
    }
  }

  /* maintain camera pose for consistency check */

  md_.last_camera_pos_ = md_.camera_pos_;
  md_.last_camera_r_m_ = md_.camera_r_m_;
  md_.last_depth_image_ = md_.depth_image_;
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  rclcpp::Time t1, t2;

  md_.raycast_num_ += 1;

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.camera_pos_);

      length = (pt_w - md_.camera_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
      }
      vox_idx = setCacheOccupancy(pt_w, 0);
    }
    else
    {
      length = (pt_w - md_.camera_pos_).norm();

      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.camera_pos_) / length * mp_.max_ray_length_ + md_.camera_pos_;
        vox_idx = setCacheOccupancy(pt_w, 0);
      }
      else
      {
        vox_idx = setCacheOccupancy(pt_w, 1);
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between camera center and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    raycaster.setInput(pt_w / mp_.resolution_, md_.camera_pos_ / mp_.resolution_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.camera_pos_).norm();

      // if (length < mp_.min_ray_length_) break;

      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          break;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  md_.local_updated_ = true;

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.camera_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.camera_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    double log_odds_update =
        md_.count_hit_[idx_ctns] >= md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns] ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    recordCollisionStateBeforeMutation(idx_ctns);

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      md_.occupancy_buffer_[idx_ctns] = mp_.clamp_min_log_;
    }

    md_.occupancy_buffer_[idx_ctns] =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
{
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - camera_pt;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return camera_pt + (min_t - 1e-3) * diff;
}

void GridMap::clearAndInflateLocalMap()
{
  /*clear outside local*/
  const int vec_margin = 5;
  // Eigen::Vector3i min_vec_margin = min_vec - Eigen::Vector3i(vec_margin,
  // vec_margin, vec_margin); Eigen::Vector3i max_vec_margin = max_vec +
  // Eigen::Vector3i(vec_margin, vec_margin, vec_margin);

  Eigen::Vector3i min_cut = md_.local_bound_min_ -
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  Eigen::Vector3i max_cut = md_.local_bound_max_ +
                            Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
  boundIndex(min_cut);
  boundIndex(max_cut);

  Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
  boundIndex(min_cut_m);
  boundIndex(max_cut_m);

  // clear data outside the local range

  for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    {

      for (int z = min_cut_m(2); z < min_cut(2); ++z)
      {
        int idx = toAddress(x, y, z);
        if (collisionOccupiedAtAddress(idx))
          recordCollisionStateBeforeMutation(idx);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        if (collisionOccupiedAtAddress(idx))
          recordCollisionStateBeforeMutation(idx);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        if (collisionOccupiedAtAddress(idx))
          recordCollisionStateBeforeMutation(idx);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        if (collisionOccupiedAtAddress(idx))
          recordCollisionStateBeforeMutation(idx);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        if (collisionOccupiedAtAddress(idx))
          recordCollisionStateBeforeMutation(idx);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        if (collisionOccupiedAtAddress(idx))
          recordCollisionStateBeforeMutation(idx);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  // inflate occupied voxels to compensate robot size

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  // int inf_step_z = 1;
  vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));
  // inf_pts.resize(4 * inf_step + 3);
  Eigen::Vector3i inf_pt;

  // clear outdated data
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }

  // inflate obstacles
  for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] > mp_.min_occupancy_log_)
        {
          inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);

          for (int k = 0; k < (int)inf_pts.size(); ++k)
          {
            inf_pt = inf_pts[k];
            int idx_inf = toAddress(inf_pt);
            if (idx_inf < 0 ||
                idx_inf >= mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2))
            {
              continue;
            }
            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
        }
      }

  // add virtual ceiling to limit flight height
  if (mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
}

void GridMap::visCallback()
{
  publishMapInflate(true);
  publishMap();
}

void GridMap::updateOccupancyCallback()
{
  const rclcpp::Time receipt_time = node_->now();
  bool watchdog_timed_out = false;
  double last_receipt_time_s = std::numeric_limits<double>::quiet_NaN();
  updateOccupancyFromPendingDepth(
      receipt_time, &watchdog_timed_out, &last_receipt_time_s);
  if (watchdog_timed_out)
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "odom or depth lost! now=%f, last_occ_update_time=%f, odom_depth_timeout=%f",
                 receipt_time.seconds(),
                 last_receipt_time_s,
                 mp_.odom_depth_timeout_);
  }
}

bool GridMap::updateOccupancyFromPendingDepth(
    const rclcpp::Time &receipt_time, bool *watchdog_timed_out,
    double *last_receipt_time_s)
{
  std::unique_lock<std::mutex> occupancy_lock(occupancy_epoch_mutex_);
  if (watchdog_timed_out)
    *watchdog_timed_out = false;
  if (md_.last_occ_update_time_.seconds() < 1.0)
    md_.last_occ_update_time_ = receipt_time;

  if (!md_.occ_need_update_)
  {
    if (md_.flag_use_depth_fusion &&
        (receipt_time - md_.last_occ_update_time_).seconds() >
            mp_.odom_depth_timeout_)
    {
      md_.flag_depth_odom_timeout_ = true;
      if (watchdog_timed_out)
        *watchdog_timed_out = true;
      if (last_receipt_time_s)
        *last_receipt_time_s = md_.last_occ_update_time_.seconds();
    }
    return false;
  }
  const double source_stamp_s = md_.pending_depth_source_stamp_s_;
  if (!std::isfinite(source_stamp_s) || source_stamp_s <= 0.0)
  {
    md_.occ_need_update_ = false;
    md_.local_updated_ = false;
    return false;
  }
  md_.last_occ_update_time_ = receipt_time;
  beginOccupancyWriteTransaction();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

  occupancy_cloud_stamp_s_.store(
      source_stamp_s, std::memory_order_release);
  const auto notification = commitOccupancyWriteTransaction(source_stamp_s);

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.pending_depth_source_stamp_s_ =
      std::numeric_limits<double>::quiet_NaN();
  occupancy_lock.unlock();
  notifyOccupancyCommitted(notification);
  return true;
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const geometry_msgs::msg::PoseStamped::ConstPtr &pose)
{
  double source_stamp_s = std::numeric_limits<double>::quiet_NaN();
  if (!sourceStampSeconds(img->header.stamp, &source_stamp_s))
    return;

  std::lock_guard<std::mutex> occupancy_lock(occupancy_epoch_mutex_);
  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  md_.camera_pos_(0) = pose->pose.position.x;
  md_.camera_pos_(1) = pose->pose.position.y;
  md_.camera_pos_(2) = pose->pose.position.z;
  md_.camera_r_m_ = Eigen::Quaterniond(pose->pose.orientation.w, pose->pose.orientation.x,
                                       pose->pose.orientation.y, pose->pose.orientation.z)
                        .toRotationMatrix();
  if (isInMap(md_.camera_pos_))
  {
    md_.has_odom_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
    md_.pending_depth_source_stamp_s_ = source_stamp_s;
  }
  else
  {
    md_.occ_need_update_ = false;
    md_.pending_depth_source_stamp_s_ =
        std::numeric_limits<double>::quiet_NaN();
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  double odom_stamp_s = std::numeric_limits<double>::quiet_NaN();
  if (odom && sourceStampSeconds(odom->header.stamp, &odom_stamp_s))
    independent_odom_stamp_s_.store(odom_stamp_s, std::memory_order_release);

  std::lock_guard<std::mutex> occupancy_lock(occupancy_epoch_mutex_);
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

void GridMap::independentCloudInputCallback(
    const sensor_msgs::msg::PointCloud2::ConstPtr &img)
{
  if (!img)
    return;
  std::lock_guard<std::mutex> lock(independent_cloud_input_mutex_);
  pending_independent_clouds_.push_back(img);
  constexpr std::size_t kMaxPendingClouds = 32u;
  while (pending_independent_clouds_.size() > kMaxPendingClouds)
    pending_independent_clouds_.pop_front();
}

sensor_msgs::msg::PointCloud2::ConstPtr
GridMap::takeLatestIndependentCloudAtOrBefore(const double clock_stamp_s)
{
  if (!std::isfinite(clock_stamp_s) || clock_stamp_s <= 0.0)
    return {};

  std::lock_guard<std::mutex> lock(independent_cloud_input_mutex_);
  for (auto reverse_it = pending_independent_clouds_.rbegin();
       reverse_it != pending_independent_clouds_.rend(); ++reverse_it)
  {
    double source_stamp_s = std::numeric_limits<double>::quiet_NaN();
    if (!*reverse_it ||
        !sourceStampSeconds((*reverse_it)->header.stamp, &source_stamp_s) ||
        source_stamp_s > clock_stamp_s)
      continue;

    const auto selected_it = std::prev(reverse_it.base());
    const auto selected = *selected_it;
    pending_independent_clouds_.erase(
        pending_independent_clouds_.begin(), std::next(selected_it));
    return selected;
  }
  return {};
}

void GridMap::processLatestIndependentCloud()
{
  const double odom_stamp_s =
      independent_odom_stamp_s_.load(std::memory_order_acquire);
  const double clock_stamp_s =
      std::isfinite(odom_stamp_s)
          ? odom_stamp_s - mp_.independent_cloud_clock_guard_s_
          : std::numeric_limits<double>::quiet_NaN();
  const auto cloud = takeLatestIndependentCloudAtOrBefore(clock_stamp_s);
  if (cloud)
    cloudCallback(cloud);
}

GridMap::OccupancyCommitNotification GridMap::applyRegisteredLidarUpdate(
    const RegisteredLidarWindowUpdate &update)
{
  if (!update.accepted || !registered_lidar_window_)
    return {};

  beginOccupancyWriteTransaction();
  const int inflation_xy =
      static_cast<int>(std::ceil(mp_.obstacles_inflation_ / mp_.resolution_));
  constexpr int inflation_z = 1;
  std::vector<int> affected;
  affected.reserve(update.changes.size());
  std::vector<std::pair<Eigen::Vector3i, bool>> collision_changes;
  collision_changes.reserve(update.changes.size());
  const auto mark_affected = [this, &affected](const int address)
  {
    const auto unsigned_address = static_cast<std::size_t>(address);
    auto &word = registered_inflation_dirty_bits_[unsigned_address >> 6U];
    const uint64_t mask = uint64_t{1} << (unsigned_address & 63U);
    if ((word & mask) == 0U)
    {
      word |= mask;
      affected.push_back(address);
    }
  };
  for (const auto &change : update.changes)
  {
    if (!isInMap(change.index))
      continue;
    const int changed_address = toAddress(change.index);
    const bool was_occupied =
        md_.occupancy_buffer_raw_cloud_[changed_address] != 0;
    const bool becomes_occupied =
        change.state == RegisteredVoxelState::OCCUPIED;
    // Observed-free/unknown transitions are risk evidence only. They must not
    // trigger the considerably more expensive collision-inflation update.
    if (was_occupied == becomes_occupied)
      continue;
    collision_changes.emplace_back(change.index, becomes_occupied);
    // Keep the sparse execution snapshot tied to actual LiDAR returns.  The
    // collision journal below intentionally observes the inflated footprint,
    // which can be tens of times larger and must not be mistaken for raw LOS
    // obstacles or copied on every execution-snapshot generation.
    if (becomes_occupied)
      registered_raw_occupied_addresses_.insert(changed_address);
    else
      registered_raw_occupied_addresses_.erase(changed_address);
    for (int x = -inflation_xy; x <= inflation_xy; ++x)
      for (int y = -inflation_xy; y <= inflation_xy; ++y)
        for (int z = -inflation_z; z <= inflation_z; ++z)
        {
          const Eigen::Vector3i index =
              change.index + Eigen::Vector3i(x, y, z);
          if (isInMap(index))
            mark_affected(toAddress(index));
        }
  }
  for (const int address : affected)
    recordCollisionStateBeforeMutation(address);

  Eigen::Vector3i minimum = Eigen::Vector3i::Zero();
  Eigen::Vector3i maximum = Eigen::Vector3i::Zero();
  bool have_changed_bounds = false;
  for (const auto &change : update.changes)
  {
    if (!isInMap(change.index))
      continue;
    if (!have_changed_bounds)
    {
      minimum = change.index;
      maximum = change.index;
      have_changed_bounds = true;
    }
    else
    {
      minimum = minimum.cwiseMin(change.index);
      maximum = maximum.cwiseMax(change.index);
    }
    const int address = toAddress(change.index);
    if (failure_evidence_capture_) {
      if (change.state == RegisteredVoxelState::UNKNOWN &&
          md_.observed_buffer_[address] != 0)
        observation_loss_producer_[address] =
            static_cast<uint8_t>(update.operation);
      else if (change.state != RegisteredVoxelState::UNKNOWN)
        observation_loss_producer_[address] = 0;
    }
    md_.occupancy_buffer_raw_cloud_[address] =
        change.state == RegisteredVoxelState::OCCUPIED ? 1 : 0;
    md_.observed_buffer_[address] =
        change.state == RegisteredVoxelState::UNKNOWN ? 0 : 1;
  }

  // Registered mode has a single hit-map producer, so maintain exact raw-hit
  // inflation reference counts instead of rescanning every affected
  // neighborhood. Fused depth occupancy is intentionally disabled in this
  // mode by the input-seam contract.
  for (const auto &[changed_index, becomes_occupied] : collision_changes)
  {
    for (int x = -inflation_xy; x <= inflation_xy; ++x)
      for (int y = -inflation_xy; y <= inflation_xy; ++y)
        for (int z = -inflation_z; z <= inflation_z; ++z)
        {
          const Eigen::Vector3i inflated_index =
              changed_index + Eigen::Vector3i(x, y, z);
          if (!isInMap(inflated_index))
            continue;
          auto &count = registered_raw_inflation_count_[
              static_cast<std::size_t>(toAddress(inflated_index))];
          if (becomes_occupied)
          {
            if (count < std::numeric_limits<uint16_t>::max())
              ++count;
          }
          else if (count > 0U)
            --count;
        }
  }
  const int ceil_id = mp_.virtual_ceil_height_ > -0.5
      ? static_cast<int>(std::floor(
            (mp_.virtual_ceil_height_ - mp_.map_origin_(2)) *
            mp_.resolution_inv_)) - 1
      : -1;
  const int yz = mp_.map_voxel_num_.y() * mp_.map_voxel_num_.z();
  for (const int linear_address : affected)
  {
    const int z = yz > 0 ? linear_address % mp_.map_voxel_num_.z() : -1;
    md_.occupancy_buffer_inflate_[linear_address] =
        (z == ceil_id || registered_raw_inflation_count_[
            static_cast<std::size_t>(linear_address)] > 0U) ? 1 : 0;
  }
  for (const int address : affected)
  {
    const auto unsigned_address = static_cast<std::size_t>(address);
    registered_inflation_dirty_bits_[unsigned_address >> 6U] &=
        ~(uint64_t{1} << (unsigned_address & 63U));
  }

  if (have_changed_bounds)
  {
    md_.local_bound_min_ =
        (minimum - Eigen::Vector3i::Constant(inflation_xy)).cwiseMax(
            Eigen::Vector3i::Zero());
    md_.local_bound_max_ =
        (maximum + Eigen::Vector3i::Constant(inflation_xy)).cwiseMin(
            mp_.map_voxel_num_ - Eigen::Vector3i::Ones());
  }
  md_.has_cloud_ = true;
  // Registered-map mode owns the same physical current-vehicle clearance
  // contract as the legacy point-cloud path. Record it before committing the
  // occupancy transaction so execution snapshots never see a new LiDAR pose
  // paired with an UNKNOWN voxel at the vehicle itself. Occupied/inflated
  // state remains dominant in diagnostic queries.
  markCurrentVehicleFootprintObserved();
  if (std::isfinite(update.stamp_s) && update.stamp_s > 0.0)
    occupancy_cloud_stamp_s_.store(
        update.stamp_s, std::memory_order_release);
  return commitOccupancyWriteTransaction(update.stamp_s);
}

void GridMap::registeredCurrentFrameCallback(
    const iap::msg::RegisteredLidarFrame::ConstSharedPtr &message)
{
  const auto started = std::chrono::steady_clock::now();
  if (!message || !registered_lidar_window_)
    return;
  RegisteredLidarFrameData frame;
  if (!registeredFrameFromMessage(
          *message, mp_.frame_id_, registered_lidar_reference_frame_id_,
          &frame))
  {
    {
      std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
      registered_current_frame_healthy_ = false;
    }
    RCLCPP_WARN(node_->get_logger(),
                "[grid_map] rejected invalid registered current frame");
    return;
  }
  const auto parsed = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(occupancy_epoch_mutex_);
  const bool first_healthy_current = !registered_current_frame_healthy_;
  md_.camera_pos_ = frame.T_map_lidar.translation();
  md_.camera_r_m_ = frame.T_map_lidar.linear();
  md_.has_odom_ = true;
  const auto update = registered_lidar_window_->applyCurrentFrame(frame);
  registered_current_frame_healthy_ = update.accepted;
  if (first_healthy_current && update.accepted)
    RCLCPP_INFO(node_->get_logger(),
                "[grid_map] first registered current frame applied id=%ld",
                frame.frame_id);
  if (update.accepted)
  {
    RegisteredLidarFrameMetadata metadata;
    metadata.frame_id = frame.frame_id;
    metadata.stamp_s = frame.stamp_s;
    metadata.scan_end_stamp_s = frame.scan_end_stamp_s;
    metadata.sensor_receipt_steady_ns = frame.sensor_receipt_steady_ns;
    metadata.T_map_lidar = frame.T_map_lidar;
    metadata.frame_contract_id = frame.frame_contract_id;
    metadata.source_is_map_reference = frame.source_is_map_reference;
    registered_support_history_.push_back(std::move(metadata));
    const double oldest_allowed_stamp =
        frame.scan_end_stamp_s - trusted_support_validity_s_;
    while (!registered_support_history_.empty() &&
           registered_support_history_.front().scan_end_stamp_s <
               oldest_allowed_stamp)
      registered_support_history_.pop_front();
    while (registered_support_history_.size() > 64U)
      registered_support_history_.pop_front();
  }
  const auto accumulated = std::chrono::steady_clock::now();
  const auto notification = applyRegisteredLidarUpdate(update);
  const auto applied = std::chrono::steady_clock::now();
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      applied - started).count();
  registered_current_apply_latency_ms_.push_back(elapsed_ms);
  const auto applied_steady_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          applied.time_since_epoch()).count();
  const double sensor_to_occupancy_ms =
      static_cast<double>(
          applied_steady_ns -
          static_cast<int64_t>(frame.sensor_receipt_steady_ns)) * 1.0e-6;
  if (std::isfinite(sensor_to_occupancy_ms) &&
      sensor_to_occupancy_ms >= 0.0 && sensor_to_occupancy_ms < 10000.0)
    registered_sensor_to_occupancy_latency_ms_.push_back(
        sensor_to_occupancy_ms);
  if (registered_current_apply_latency_ms_.size() >= 100U)
  {
    std::sort(registered_current_apply_latency_ms_.begin(),
              registered_current_apply_latency_ms_.end());
    const double p95 = registered_current_apply_latency_ms_[94];
    const double maximum = registered_current_apply_latency_ms_.back();
    RCLCPP_INFO(node_->get_logger(),
                "[grid_map] registered current frame latency count=100 "
                "p95_ms=%.3f max_ms=%.3f budget_ms=10.000",
                p95, maximum);
    registered_current_apply_latency_ms_.clear();
  }
  if (registered_sensor_to_occupancy_latency_ms_.size() >= 100U)
  {
    std::sort(registered_sensor_to_occupancy_latency_ms_.begin(),
              registered_sensor_to_occupancy_latency_ms_.end());
    RCLCPP_INFO(node_->get_logger(),
                "[grid_map] sensor to occupancy latency count=100 "
                "p95_ms=%.3f max_ms=%.3f budget_ms=80.000",
                registered_sensor_to_occupancy_latency_ms_[94],
                registered_sensor_to_occupancy_latency_ms_.back());
    registered_sensor_to_occupancy_latency_ms_.clear();
  }
  if (elapsed_ms > 10.0)
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[grid_map] registered current frame apply %.3f ms exceeds 10 ms budget "
        "(decode=%.3f accumulate=%.3f grid=%.3f changes=%zu)",
        elapsed_ms,
        std::chrono::duration<double, std::milli>(parsed - started).count(),
        std::chrono::duration<double, std::milli>(accumulated - parsed).count(),
        std::chrono::duration<double, std::milli>(applied - accumulated).count(),
        update.changes.size());
  lock.unlock();
  notifyOccupancyCommitted(notification);
}

void GridMap::registeredWindowDeltaCallback(
    const iap::msg::ActiveLidarWindowDelta::ConstSharedPtr &message)
{
  const auto started = std::chrono::steady_clock::now();
  if (!message || !registered_lidar_window_)
    return;
  if (message->header.frame_id != mp_.frame_id_ ||
      message->frame_contract_id != registered_frame_contract_id_ ||
      message->generation != message->base_generation + 1U)
  {
    {
      std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
      registered_active_window_healthy_ = false;
    }
    requestRegisteredWindowRecovery("invalid_delta_envelope");
    return;
  }
  uint64_t observed = registered_observed_generation_.load(
      std::memory_order_acquire);
  while (observed < message->generation &&
         !registered_observed_generation_.compare_exchange_weak(
             observed, message->generation, std::memory_order_acq_rel,
             std::memory_order_acquire))
  {
  }
  if (observed == 0U)
    RCLCPP_INFO(node_->get_logger(),
                "[grid_map] first active-window delta base=%lu generation=%lu",
                message->base_generation, message->generation);
  if (!message->complete)
  {
    {
      std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
      registered_active_window_healthy_ = false;
    }
    requestRegisteredWindowRecovery("producer_active_window_incomplete");
    return;
  }
  ActiveLidarWindowDeltaData delta;
  delta.frame_contract_id = message->frame_contract_id;
  delta.base_generation = message->base_generation;
  delta.generation = message->generation;
  delta.complete = message->complete;
  delta.removed_frame_ids = message->removed_frame_ids;
  if (message->pose_updated_frame_ids.size() !=
      message->updated_t_map_lidar.size())
  {
    requestRegisteredWindowRecovery("pose_update_size_mismatch");
    return;
  }
  for (const auto &added : message->added)
  {
    RegisteredLidarFrameData frame;
    if (!registeredFrameFromMessage(
            added, mp_.frame_id_, registered_lidar_reference_frame_id_,
            &frame))
    {
      requestRegisteredWindowRecovery("invalid_added_frame");
      return;
    }
    delta.added.push_back(std::move(frame));
  }
  for (std::size_t i = 0; i < message->pose_updated_frame_ids.size(); ++i)
    delta.pose_updates.emplace_back(
        message->pose_updated_frame_ids[i],
        poseFromMessage(message->updated_t_map_lidar[i]));

  RegisteredLidarWindowUpdate update;
  {
    std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
    update = registered_lidar_window_->applyActiveDelta(delta);
    if (update.accepted)
    {
      registered_active_window_healthy_ =
          update.active_generation >=
          registered_observed_generation_.load(
              std::memory_order_acquire);
      registered_recovery_pending_.store(
          !registered_active_window_healthy_, std::memory_order_release);
      applyRegisteredLidarUpdate(update);
    }
    else if (update.recovery_required)
    {
      registered_active_window_healthy_ = false;
      registered_recovery_pending_.store(true, std::memory_order_release);
    }
  }
  if (!update.accepted && update.recovery_required)
    requestRegisteredWindowRecovery(update.reason);
  const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  registered_delta_apply_latency_ms_.push_back(elapsed_ms);
  if (registered_delta_apply_latency_ms_.size() >= 20U)
  {
    std::sort(registered_delta_apply_latency_ms_.begin(),
              registered_delta_apply_latency_ms_.end());
    const double p95 = registered_delta_apply_latency_ms_[18];
    const double maximum = registered_delta_apply_latency_ms_.back();
    RCLCPP_INFO(node_->get_logger(),
                "[grid_map] registered keyframe delta latency count=20 "
                "p95_ms=%.3f max_ms=%.3f budget_ms=40.000",
                p95, maximum);
    registered_delta_apply_latency_ms_.clear();
  }
  if (elapsed_ms > 40.0)
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[grid_map] registered keyframe delta apply %.3f ms exceeds 40 ms budget",
        elapsed_ms);
}

void GridMap::requestRegisteredWindowRecovery(const std::string &reason)
{
  {
    std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
    registered_active_window_healthy_ = false;
  }
  registered_recovery_pending_.store(true, std::memory_order_release);
  if (!registered_recovery_client_ ||
      registered_recovery_in_flight_.exchange(true,
                                               std::memory_order_acq_rel))
    return;
  if (!registered_recovery_client_->service_is_ready())
  {
    registered_recovery_in_flight_.store(false, std::memory_order_release);
    RCLCPP_WARN(node_->get_logger(),
                "[grid_map] active-window recovery request request_serial=%lu "
                "request_base_generation=0 observed_generation=%lu "
                "committed_generation=0 service_ready=0 "
                "request_sent_steady=0 reject_reason=service_unavailable "
                "trigger=%s",
                registered_recovery_serial_.load(std::memory_order_acquire),
                registered_observed_generation_.load(
                    std::memory_order_acquire), reason.c_str());
    return;
  }
  uint64_t request_base_generation = 0;
  {
    std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
    request_base_generation = registered_lidar_window_
        ? registered_lidar_window_->activeGeneration() : 0U;
  }
  const uint64_t request_serial =
      registered_recovery_serial_.fetch_add(
          1U, std::memory_order_acq_rel) + 1U;
  const int64_t request_sent_ns = steadyNowNanoseconds();
  const int64_t deadline_ns = request_sent_ns +
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::seconds(1)).count();
  registered_recovery_request_sent_ns_.store(
      request_sent_ns, std::memory_order_release);
  registered_recovery_response_ready_ns_.store(
      0, std::memory_order_release);
  registered_recovery_request_base_generation_.store(
      request_base_generation, std::memory_order_release);
  registered_recovery_deadline_ns_.store(
      deadline_ns, std::memory_order_release);
  RCLCPP_INFO(node_->get_logger(),
              "[grid_map] active-window recovery request "
              "request_serial=%lu request_base_generation=%lu "
              "observed_generation=%lu committed_generation=%lu "
              "service_ready=1 request_sent_steady=%ld trigger=%s",
              request_serial, request_base_generation,
              registered_observed_generation_.load(
                  std::memory_order_acquire),
              request_base_generation, request_sent_ns, reason.c_str());
  auto request =
      std::make_shared<iap::srv::GetActiveLidarWindow::Request>();
  request->expected_frame_contract_id = registered_frame_contract_id_;
  const std::weak_ptr<GridMap> weak_self = weak_from_this();
  if (weak_self.expired())
  {
    registered_recovery_in_flight_.store(false, std::memory_order_release);
    RCLCPP_ERROR(node_->get_logger(),
                 "[grid_map] recovery requires shared GridMap ownership");
    return;
  }
  try
  {
    registered_recovery_future_ =
      registered_recovery_client_->async_send_request(
      request,
      [weak_self, reason, request_serial, request_base_generation,
       request_sent_ns, deadline_ns](
          rclcpp::Client<iap::srv::GetActiveLidarWindow>::SharedFuture future)
      {
        const auto self = weak_self.lock();
        if (!self)
          return;
        const int64_t callback_started_ns = steadyNowNanoseconds();
        if (request_serial != self->registered_recovery_serial_.load(
                                  std::memory_order_acquire))
        {
          RCLCPP_WARN(
              self->node_->get_logger(),
              "[grid_map] active-window recovery response "
              "request_serial=%lu request_base_generation=%lu "
              "observed_generation=%lu committed_generation=%lu "
              "service_ready=1 request_sent_steady=%ld "
              "response_received_steady=%ld callback_started_steady=%ld "
              "timeout_steady=%ld response_complete=unknown "
              "response_generation=unknown commit_result=rejected "
              "reject_reason=stale_request_serial",
              request_serial, request_base_generation,
              self->registered_observed_generation_.load(
                  std::memory_order_acquire),
              self->registered_lidar_window_->activeGeneration(),
              request_sent_ns, callback_started_ns, callback_started_ns,
              deadline_ns);
          return;
        }
        int64_t response_received_ns =
            self->registered_recovery_response_ready_ns_.load(
                std::memory_order_acquire);
        if (response_received_ns == 0)
        {
          response_received_ns = callback_started_ns;
          self->registered_recovery_response_ready_ns_.store(
              response_received_ns, std::memory_order_release);
        }
        iap::srv::GetActiveLidarWindow::Response::SharedPtr response;
        try
        {
          response = future.get();
        }
        catch (const std::exception &error)
        {
          self->registered_recovery_in_flight_.store(
              false, std::memory_order_release);
          self->registered_recovery_pending_.store(
              true, std::memory_order_release);
          self->registered_recovery_future_.reset();
          self->registered_recovery_deadline_ns_.store(
              0, std::memory_order_release);
          RCLCPP_WARN(self->node_->get_logger(),
                      "[grid_map] active-window recovery response "
                      "request_serial=%lu request_base_generation=%lu "
                      "observed_generation=%lu committed_generation=%lu "
                      "service_ready=1 request_sent_steady=%ld "
                      "response_received_steady=%ld "
                      "callback_started_steady=%ld timeout_steady=%ld "
                      "response_complete=unknown "
                      "response_generation=unknown commit_result=rejected "
                      "reject_reason=future_exception detail=%s trigger=%s",
                      request_serial, request_base_generation,
                      self->registered_observed_generation_.load(
                          std::memory_order_acquire),
                      self->registered_lidar_window_->activeGeneration(),
                      request_sent_ns,
                      response_received_ns, callback_started_ns,
                      deadline_ns,
                      error.what(), reason.c_str());
          return;
        }
        RegisteredLidarWindowUpdate update;
        bool committed_request_state = false;
        std::string reject_reason = "none";
        if (response && response->complete &&
            response->header.frame_id == self->mp_.frame_id_ &&
            response->frame_contract_id ==
                self->registered_frame_contract_id_)
        {
          std::vector<RegisteredLidarFrameData> frames;
          bool valid = true;
          frames.reserve(response->frames.size());
          for (const auto &message : response->frames)
          {
            RegisteredLidarFrameData frame;
            if (!registeredFrameFromMessage(
                    message, self->mp_.frame_id_,
                    self->registered_lidar_reference_frame_id_, &frame))
            {
              valid = false;
              break;
            }
            frames.push_back(std::move(frame));
          }
          if (valid)
          {
            std::lock_guard<std::mutex> lock(self->occupancy_epoch_mutex_);
            if (request_serial != self->registered_recovery_serial_.load(
                                      std::memory_order_acquire))
              return;
            const uint64_t local_generation =
                self->registered_lidar_window_->activeGeneration();
            const uint64_t required_generation = std::max(
                request_base_generation, local_generation);
            if (response->generation >= required_generation)
            {
              update = self->registered_lidar_window_->replaceActiveWindow(
                  response->generation, response->frame_contract_id, frames);
              if (update.accepted)
              {
                const uint64_t observed_generation =
                    self->registered_observed_generation_.load(
                        std::memory_order_acquire);
                self->registered_active_window_healthy_ =
                    response->generation >= observed_generation;
                self->applyRegisteredLidarUpdate(update);
                // Commit health, pending and in-flight under the same state
                // lock used by the delta callback. A fault arriving after
                // this point will therefore set pending=true after us rather
                // than being overwritten by a late success completion.
                self->registered_recovery_pending_.store(
                    !self->registered_active_window_healthy_,
                    std::memory_order_release);
                self->registered_recovery_in_flight_.store(
                    false, std::memory_order_release);
                committed_request_state = true;
              }
            }
            else
              update.reason = "stale_recovery_generation";
          }
          else
            update.reason = "invalid_recovery_frame";
        }
        else if (!response)
          update.reason = "null_recovery_response";
        else if (!response->complete)
          update.reason = response->reason.empty()
              ? "incomplete_recovery_response" : response->reason;
        else if (response->header.frame_id != self->mp_.frame_id_)
          update.reason = "recovery_frame_mismatch";
        else
          update.reason = "recovery_contract_mismatch";
        if (!committed_request_state &&
            request_serial == self->registered_recovery_serial_.load(
                                  std::memory_order_acquire))
        {
          self->registered_recovery_in_flight_.store(
              false, std::memory_order_release);
          self->registered_recovery_pending_.store(
              true, std::memory_order_release);
        }
        if (!update.accepted)
          reject_reason = update.reason.empty()
              ? "replace_active_window_failed" : update.reason;
        if (request_serial == self->registered_recovery_serial_.load(
                                  std::memory_order_acquire))
        {
          self->registered_recovery_future_.reset();
          self->registered_recovery_deadline_ns_.store(
              0, std::memory_order_release);
        }
        const uint64_t response_generation = response
            ? response->generation : 0U;
        const int response_complete = response && response->complete ? 1 : 0;
        RCLCPP_INFO(
            self->node_->get_logger(),
            "[grid_map] active-window recovery response "
            "request_serial=%lu request_base_generation=%lu "
            "observed_generation=%lu committed_generation=%lu "
            "service_ready=1 request_sent_steady=%ld "
            "response_received_steady=%ld callback_started_steady=%ld "
            "timeout_steady=%ld response_complete=%d "
            "response_generation=%lu commit_result=%s reject_reason=%s "
            "pending=%d trigger=%s",
            request_serial, request_base_generation,
            self->registered_observed_generation_.load(
                std::memory_order_acquire),
            self->registered_lidar_window_->activeGeneration(),
            request_sent_ns,
            response_received_ns, callback_started_ns,
            deadline_ns,
            response_complete, response_generation,
            committed_request_state ? "committed" : "rejected",
            committed_request_state ? "none" : reject_reason.c_str(),
            self->registered_recovery_pending_.load(
                std::memory_order_acquire) ? 1 : 0,
            reason.c_str());
      });
  }
  catch (const std::exception &error)
  {
    if (request_serial == registered_recovery_serial_.load(
                              std::memory_order_acquire))
      registered_recovery_in_flight_.store(false,
                                            std::memory_order_release);
    registered_recovery_pending_.store(true, std::memory_order_release);
    registered_recovery_future_.reset();
    registered_recovery_deadline_ns_.store(0, std::memory_order_release);
    RCLCPP_WARN(node_->get_logger(),
                "[grid_map] active-window recovery request failed after %s: %s",
                reason.c_str(), error.what());
  }
}

void GridMap::maintainRegisteredWindowRecovery()
{
  if (!registered_lidar_window_enabled_)
    return;
  const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  if (registered_recovery_in_flight_.load(std::memory_order_acquire))
  {
    const auto deadline_ns = registered_recovery_deadline_ns_.load(
        std::memory_order_acquire);
    if (deadline_ns > 0 && now_ns < deadline_ns)
      return;
    const int64_t response_ready_ns =
        registered_recovery_response_ready_ns_.load(
            std::memory_order_acquire);
    const int64_t control_dispatch_period_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            kRegisteredRecoveryControlPeriod).count();
    if (response_ready_ns > 0 &&
        now_ns <= response_ready_ns + control_dispatch_period_ns)
      return;
    const uint64_t timed_out_serial = registered_recovery_serial_.load(
        std::memory_order_acquire);
    const uint64_t request_base_generation =
        registered_recovery_request_base_generation_.load(
            std::memory_order_acquire);
    if (registered_recovery_future_.has_value())
      registered_recovery_client_->remove_pending_request(
          registered_recovery_future_->request_id);
    registered_recovery_future_.reset();
    registered_recovery_serial_.fetch_add(1U, std::memory_order_acq_rel);
    registered_recovery_in_flight_.store(false, std::memory_order_release);
    registered_recovery_pending_.store(true, std::memory_order_release);
    RCLCPP_WARN(
        node_->get_logger(),
        "[grid_map] active-window recovery timeout "
        "request_serial=%lu request_base_generation=%lu "
        "observed_generation=%lu committed_generation=%lu "
        "service_ready=%d request_sent_steady=%ld "
        "response_received_steady=%ld callback_started_steady=0 "
        "timeout_steady=%ld response_complete=unknown "
        "response_generation=unknown commit_result=timed_out "
        "reject_reason=%s",
        timed_out_serial, request_base_generation,
        registered_observed_generation_.load(std::memory_order_acquire),
        registered_lidar_window_ ?
            registered_lidar_window_->activeGeneration() : 0U,
        registered_recovery_client_->service_is_ready() ? 1 : 0,
        registered_recovery_request_sent_ns_.load(
            std::memory_order_acquire), response_ready_ns, now_ns,
        response_ready_ns > 0
            ? "uncorrelated_or_undispatched_response"
            : "no_response_before_deadline");
  }
  if (registered_recovery_pending_.load(std::memory_order_acquire))
    requestRegisteredWindowRecovery("startup_or_retry");
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &img)
{
  double source_stamp_s = std::numeric_limits<double>::quiet_NaN();
  const bool valid_source_stamp =
      img && sourceStampSeconds(img->header.stamp, &source_stamp_s);
  if (!independent_cloud_timer_ && valid_source_stamp &&
      std::isfinite(last_independent_cloud_stamp_s_) &&
      source_stamp_s >= last_independent_cloud_stamp_s_ &&
      source_stamp_s - last_independent_cloud_stamp_s_ + 1e-9 <
          mp_.independent_cloud_min_interval_s_)
  {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  // A hit-only LiDAR legitimately publishes an empty PointCloud2 when no
  // beam returns. Avoid asking PCL to decode an empty data buffer, while still
  // processing the frame below so the local rolling buffer is refreshed.
  if (!img->data.empty())
    pcl::fromROSMsg(*img, latest_cloud);

  std::unique_lock<std::mutex> occupancy_lock(occupancy_epoch_mutex_);
  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  beginOccupancyWriteTransaction();

  this->resetBufferUnlocked(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);

  markCurrentVehicleFootprintObserved();

  pcl::PointXYZ pt;
  Eigen::Vector3d p3d, p3d_inf;

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = 1;

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    pt = latest_cloud.points[i];
    p3d(0) = pt.x, p3d(1) = pt.y, p3d(2) = pt.z;

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;
    Eigen::Vector3i inf_pt;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {

      // PointCloud2 supplies returns, not an implicit global free-space map.
      // Only the explicit sensor-to-return traversal is marked observed free.
      RayCaster observation_ray;
      Eigen::Vector3d ray_voxel;
      observation_ray.setInput(p3d / mp_.resolution_,
                               md_.camera_pos_ / mp_.resolution_);
      while (observation_ray.step(ray_voxel))
      {
        const Eigen::Vector3d ray_position =
            (ray_voxel + Eigen::Vector3d::Constant(0.5)) * mp_.resolution_;
        Eigen::Vector3i ray_id;
        posToIndex(ray_position, ray_id);
        if (isInMap(ray_id))
          md_.observed_buffer_[toAddress(ray_id)] = 1;
      }

      /* inflate the point */
      // 点云膨胀
      Eigen::Vector3i raw_id;
      posToIndex(p3d, raw_id);
      if (isInMap(raw_id))
      {
        const int raw_address = toAddress(raw_id);
        recordCollisionStateBeforeMutation(raw_address);
        md_.occupancy_buffer_raw_cloud_[raw_address] = 1;
        md_.observed_buffer_[raw_address] = 1;
      }
      for (int x = -inf_step; x <= inf_step; ++x)
        for (int y = -inf_step; y <= inf_step; ++y)
          for (int z = -inf_step_z; z <= inf_step_z; ++z)
          {

            p3d_inf(0) = pt.x + x * mp_.resolution_;
            p3d_inf(1) = pt.y + y * mp_.resolution_;
            p3d_inf(2) = pt.z + z * mp_.resolution_;

            max_x = max(max_x, p3d_inf(0));
            max_y = max(max_y, p3d_inf(1));
            max_z = max(max_z, p3d_inf(2));

            min_x = min(min_x, p3d_inf(0));
            min_y = min(min_y, p3d_inf(1));
            min_z = min(min_z, p3d_inf(2));

            posToIndex(p3d_inf, inf_pt);

            if (!isInMap(inf_pt))
              continue;

            int idx_inf = toAddress(inf_pt);

            md_.occupancy_buffer_inflate_[idx_inf] = 1;
          }
    }
  }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  // 更新局部地图边界
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // add virtual ceiling to limit flight height
  // 添加虚拟天花板控制飞行高度
  if (mp_.virtual_ceil_height_ > -0.5) {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y) {
        md_.occupancy_buffer_inflate_[toAddress(x, y, ceil_id)] = 1;
      }
  }
  occupancy_cloud_stamp_s_.store(
      rclcpp::Time(img->header.stamp).seconds(), std::memory_order_release);
  if (valid_source_stamp)
    last_independent_cloud_stamp_s_ = source_stamp_s;
  const auto notification = commitOccupancyWriteTransaction(
      rclcpp::Time(img->header.stamp).seconds());
  occupancy_lock.unlock();
  notifyOccupancyCommitted(notification);
}

void GridMap::markCurrentVehicleFootprintObserved()
{
  const double radius = current_vehicle_clearance_radius_m_;
  if (!md_.camera_pos_.allFinite() || !std::isfinite(mp_.resolution_) ||
      mp_.resolution_ <= 0.0)
    return;

  if (radius <= 0.0)
  {
    Eigen::Vector3i sensor_id;
    posToIndex(md_.camera_pos_, sensor_id);
    if (isInMap(sensor_id))
      md_.observed_buffer_[toAddress(sensor_id)] = 1;
    return;
  }

  // Match P4's swept-volume voxelization: every voxel whose AABB intersects
  // the vehicle sphere is part of the volume currently occupied by the
  // vehicle and is therefore directly known, not extrapolated UNKNOWN space.
  const Eigen::Vector3i minimum = ((md_.camera_pos_.array() - radius -
      mp_.map_origin_.array()) * mp_.resolution_inv_).floor().cast<int>();
  const Eigen::Vector3i maximum = ((md_.camera_pos_.array() + radius -
      mp_.map_origin_.array()) * mp_.resolution_inv_).floor().cast<int>();
  for (int x = minimum.x(); x <= maximum.x(); ++x)
    for (int y = minimum.y(); y <= maximum.y(); ++y)
      for (int z = minimum.z(); z <= maximum.z(); ++z)
      {
        const Eigen::Vector3i index(x, y, z);
        if (!isInMap(index))
          continue;
        const Eigen::Vector3d cell_min = mp_.map_origin_ + mp_.resolution_ *
            index.cast<double>();
        const Eigen::Vector3d cell_max =
            cell_min + Eigen::Vector3d::Constant(mp_.resolution_);
        const Eigen::Vector3d closest =
            md_.camera_pos_.cwiseMax(cell_min).cwiseMin(cell_max);
        if ((closest - md_.camera_pos_).squaredNorm() <= radius * radius)
          md_.observed_buffer_[toAddress(index)] = 1;
      }
}

void GridMap::setCurrentVehicleClearanceRadius(const double radius_m)
{
  std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
  current_vehicle_clearance_radius_m_ =
      std::isfinite(radius_m) && radius_m >= 0.0 ? radius_m : 0.0;
}

void GridMap::setFailureEvidenceCapture(const bool enabled)
{
  std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
  failure_evidence_capture_ = enabled;
  observation_loss_producer_.clear();
  if (enabled)
    observation_loss_producer_.resize(md_.observed_buffer_.size(), 0);
}

void GridMap::publishMap()
{

  if (map_pub_->get_subscription_count() <= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  int lmm = mp_.local_map_margin_ / 2;
  min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
  max_cut += Eigen::Vector3i(lmm, lmm, lmm);

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        const int address = toAddress(x, y, z);
        const bool occupied = registered_lidar_window_enabled_
            ? md_.occupancy_buffer_raw_cloud_[address] != 0
            : md_.occupancy_buffer_[address] >= mp_.min_occupancy_log_;
        if (!occupied)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_pub_->publish(cloud_msg);
}

void GridMap::publishMapInflate(bool all_info)
{

  if (map_inf_pub_->get_subscription_count()<= 0)
    return;

  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  if (all_info)
  {
    int lmm = mp_.local_map_margin_;
    min_cut -= Eigen::Vector3i(lmm, lmm, lmm);
    max_cut += Eigen::Vector3i(lmm, lmm, lmm);
  }

  boundIndex(min_cut);
  boundIndex(max_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {
        if (md_.occupancy_buffer_inflate_[toAddress(x, y, z)] == 0)
          continue;

        Eigen::Vector3d pos;
        indexToPos(Eigen::Vector3i(x, y, z), pos);
        if (pos(2) > mp_.visualization_truncate_height_)
          continue;

        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        cloud.push_back(pt);
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  map_inf_pub_->publish(cloud_msg);

  // RCLCPP_INFO(rclcpp::get_logger("publishMapInflate"), "pub map");
}

bool GridMap::odomValid() { return md_.has_odom_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

uint64_t GridMap::occupancyGeneration() const
{
  const uint64_t sequence = occupancy_update_sequence_.load(
      std::memory_order_acquire);
  // During an in-progress write, the previous committed immutable epoch is
  // still authoritative. Returning zero here falsely revoked a P0 refresh
  // that had already captured that epoch.
  return sequence / 2u;
}

void GridMap::setOccupancyCommitObserver(OccupancyCommitObserver observer)
{
  std::lock_guard<std::mutex> lock(occupancy_commit_observer_mutex_);
  occupancy_commit_observer_ = std::move(observer);
}

bool GridMap::collisionOccupiedAtAddress(const int address) const
{
  if (address < 0 ||
      address >= static_cast<int>(md_.occupancy_buffer_raw_cloud_.size()) ||
      address >= static_cast<int>(md_.occupancy_buffer_.size()))
    return false;
  return md_.occupancy_buffer_raw_cloud_[static_cast<std::size_t>(address)] !=
             0 ||
         md_.occupancy_buffer_[static_cast<std::size_t>(address)] >
             mp_.min_occupancy_log_;
}

void GridMap::beginOccupancyWriteTransaction()
{
  collision_state_before_transaction_.clear();
  collision_transaction_active_ = true;
  // Serialize the transition to an odd (writer-active) sequence with journal
  // readers. A reader must never observe the previous committed generation as
  // complete while a point-cloud callback is already mutating its successor.
  std::lock_guard<std::mutex> lock(collision_delta_mutex_);
  occupancy_update_sequence_.fetch_add(1, std::memory_order_acq_rel);
}

void GridMap::recordCollisionStateBeforeMutation(const int address)
{
  if (!collision_transaction_active_ || address < 0)
    return;
  collision_state_before_transaction_.try_emplace(
      address, collisionOccupiedAtAddress(address));
}

GridMap::OccupancyCommitNotification GridMap::commitOccupancyWriteTransaction(
    const double stamp_s)
{
  const uint64_t odd_sequence = occupancy_update_sequence_.load(
      std::memory_order_acquire);
  const uint64_t from_generation = odd_sequence / 2u;
  auto delta = std::make_shared<OccupancyCollisionDelta>();
  delta->from_generation = from_generation;
  delta->to_generation = from_generation + 1u;
  delta->stamp_s = stamp_s;
  delta->geometry_id = geometryIdentity(
      mp_.frame_id_, mp_.map_origin_, mp_.map_voxel_num_, mp_.resolution_);
  delta->complete = collision_transaction_active_ &&
      (odd_sequence & 1u) != 0u && !delta->geometry_id.empty();

  std::vector<std::pair<int, bool>> semantic_changes;
  semantic_changes.reserve(collision_state_before_transaction_.size());
  for (const auto &[address, before_occupied] :
       collision_state_before_transaction_)
  {
    const bool after_occupied = collisionOccupiedAtAddress(address);
    if (after_occupied != before_occupied)
      semantic_changes.emplace_back(address, after_occupied);
  }
  std::sort(semantic_changes.begin(), semantic_changes.end(),
            [](const auto &lhs, const auto &rhs) {
              return lhs.first < rhs.first;
            });
  const int yz = mp_.map_voxel_num_.y() * mp_.map_voxel_num_.z();
  for (const auto &[address, occupied] : semantic_changes)
  {
    const int x = yz > 0 ? address / yz : -1;
    const int remainder = yz > 0 ? address % yz : -1;
    const int y = mp_.map_voxel_num_.z() > 0
        ? remainder / mp_.map_voxel_num_.z()
        : -1;
    const int z = mp_.map_voxel_num_.z() > 0
        ? remainder % mp_.map_voxel_num_.z()
        : -1;
    delta->changes.push_back({Eigen::Vector3i(x, y, z), occupied});
  }

  {
    std::lock_guard<std::mutex> lock(collision_delta_mutex_);
    collision_delta_history_.push_back(std::move(delta));
    while (collision_delta_history_.size() >
           kCollisionDeltaHistoryCapacity)
      collision_delta_history_.pop_front();
    // Publish the journal entry and its even committed generation under one
    // synchronization boundary.
    occupancy_update_sequence_.fetch_add(1, std::memory_order_release);
  }
  collision_state_before_transaction_.clear();
  collision_transaction_active_ = false;
  return {from_generation + 1u, stamp_s};
}

void GridMap::notifyOccupancyCommitted(
    const OccupancyCommitNotification &notification)
{
  if (!notification)
    return;
  OccupancyCommitObserver observer;
  {
    std::lock_guard<std::mutex> lock(occupancy_commit_observer_mutex_);
    observer = occupancy_commit_observer_;
  }
  // Invoke user code after the occupancy transaction lock is released. This
  // permits a notification consumer to freeze the just-committed generation
  // without blocking or re-entering the map writer.
  if (observer)
    observer(notification.generation, notification.source_stamp_s);
}

OccupancyCollisionDeltaHistory GridMap::collisionDeltasSince(
    const uint64_t base_generation) const
{
  OccupancyCollisionDeltaHistory out;
  out.base_generation = base_generation;
  std::lock_guard<std::mutex> lock(collision_delta_mutex_);
  const uint64_t sequence = occupancy_update_sequence_.load(
      std::memory_order_acquire);
  out.latest_generation = sequence / 2u;
  if ((sequence & 1u) != 0u)
  {
    out.update_in_progress = true;
    return out;
  }
  if (base_generation == 0u || base_generation > out.latest_generation)
    return out;
  if (base_generation == out.latest_generation)
  {
    out.complete = true;
    if (!collision_delta_history_.empty())
      out.geometry_id = collision_delta_history_.back()->geometry_id;
    return out;
  }

  uint64_t expected = base_generation;
  for (const auto &delta : collision_delta_history_)
  {
    if (!delta || delta->to_generation <= base_generation ||
        delta->from_generation >= out.latest_generation)
      continue;
    if (!delta->complete || delta->from_generation != expected ||
        delta->to_generation != expected + 1u)
      return out;
    if (out.geometry_id.empty())
      out.geometry_id = delta->geometry_id;
    else if (out.geometry_id != delta->geometry_id)
      return out;
    out.deltas.push_back(delta);
    expected = delta->to_generation;
    if (expected == out.latest_generation)
      break;
  }
  out.complete = expected == out.latest_generation;
  if (!out.complete)
    out.deltas.clear();
  return out;
}

GridMap::OccupancyDiagnostic GridMap::queryOccupancyDiagnostic(
    const Eigen::Vector3d &pos, const bool include_details) const
{
  std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
  OccupancyDiagnostic out;
  out.resolution_m = mp_.resolution_;
  if (include_details) {
    out.inflation_m = mp_.obstacles_inflation_;
    out.frame_id = mp_.frame_id_;
  }
  out.cloud_stamp_s = occupancy_cloud_stamp_s_.load(
      std::memory_order_acquire);
  if (!pos.allFinite())
    return out;

  const uint64_t before = occupancy_update_sequence_.load(
      std::memory_order_acquire);
  if ((before & 1u) != 0u)
  {
    if (include_details) out.source = "occupancy_update_in_progress";
    return out;
  }
  for (int axis = 0; axis < 3; ++axis)
    out.voxel_index(axis) = static_cast<int>(std::floor(
        (pos(axis) - mp_.map_origin_(axis)) * mp_.resolution_inv_));
  for (int axis = 0; axis < 3; ++axis)
  {
    if (out.voxel_index(axis) < 0 ||
        out.voxel_index(axis) >= mp_.map_voxel_num_(axis))
    {
      if (include_details) out.source = "position_out_of_map";
      return out;
    }
  }
  const int address = out.voxel_index(0) * mp_.map_voxel_num_(1) *
          mp_.map_voxel_num_(2) +
      out.voxel_index(1) * mp_.map_voxel_num_(2) + out.voxel_index(2);
  if (include_details) out.voxel_center =
      (out.voxel_index.cast<double>() + Eigen::Vector3d::Constant(0.5)) *
          mp_.resolution_ + mp_.map_origin_;
  const bool raw_cloud =
      address >= 0 && address < static_cast<int>(md_.occupancy_buffer_raw_cloud_.size()) &&
      md_.occupancy_buffer_raw_cloud_[static_cast<std::size_t>(address)] != 0;
  const bool raw_fused =
      address >= 0 && address < static_cast<int>(md_.occupancy_buffer_.size()) &&
      md_.occupancy_buffer_[static_cast<std::size_t>(address)] >
          mp_.min_occupancy_log_;
  out.raw_occupied = raw_cloud || raw_fused;
  out.inflated_occupied =
      address >= 0 && address < static_cast<int>(md_.occupancy_buffer_inflate_.size()) &&
      md_.occupancy_buffer_inflate_[static_cast<std::size_t>(address)] != 0;
  out.observed = out.raw_occupied || out.inflated_occupied ||
      (address >= 0 &&
       address < static_cast<int>(md_.observed_buffer_.size()) &&
       md_.observed_buffer_[static_cast<std::size_t>(address)] != 0);
  if (include_details) out.state = (out.raw_occupied || out.inflated_occupied)
      ? GridMapObservationState::OCCUPIED
      : out.observed ? GridMapObservationState::OBSERVED_FREE
                     : GridMapObservationState::UNKNOWN;
  const uint64_t after = occupancy_update_sequence_.load(
      std::memory_order_acquire);
  if (before != after || (after & 1u) != 0u)
  {
    if (include_details) out.source = "occupancy_generation_changed";
    return out;
  }
  out.available = true;
  out.generation = after / 2u;
  if (include_details) out.source = raw_cloud ? "raw_cloud" : raw_fused ? "fused_depth" :
      out.inflated_occupied ? "inflated_neighbor" :
      out.observed ? "observed_free" : "unknown";
  return out;
}

GridMap::OccupancyDiagnosticQuery
GridMap::captureOccupancyDiagnosticQuery() const
{
  const auto epoch = captureFrozenOccupancyEpoch();
  return epoch ? epoch->diagnostic_query : OccupancyDiagnosticQuery{};
}

std::shared_ptr<const FrozenOccupancyEpoch>
GridMap::captureFrozenExecutionOccupancyEpoch() const
{
  if (!registered_lidar_window_enabled_ ||
      !trusted_local_map_support_enabled_)
    return captureFrozenOccupancyEpoch();

  struct SparseFrozenState
  {
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3i dimensions = Eigen::Vector3i::Zero();
    double resolution = std::numeric_limits<double>::quiet_NaN();
    double resolution_inv = std::numeric_limits<double>::quiet_NaN();
    double inflation = 0.0;
    double virtual_ceiling = -1.0;
    std::string frame_id;
    std::string frame_contract_id;
    double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
    uint64_t generation = 0;
    uint64_t active_window_generation = 0;
    int64_t current_frame_id = -1;
    Eigen::Vector3d current_vehicle_position =
        Eigen::Vector3d::Constant(
            std::numeric_limits<double>::quiet_NaN());
    double current_vehicle_clearance_radius_m = 0.0;
    std::vector<int> raw_addresses;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> environment_hits;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> current_hits;
    std::shared_ptr<const std::vector<RegisteredLidarObstacleSource>>
        active_obstacle_sources;
    std::optional<RegisteredLidarFrameMetadata> current_frame;
    std::shared_ptr<const LocalEvidenceSnapshot> local_evidence_snapshot;
    std::vector<RegisteredLidarFrameMetadata> support_history;
  };

  auto state = std::make_shared<SparseFrozenState>();
  {
    std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
    const uint64_t sequence = occupancy_update_sequence_.load(
        std::memory_order_acquire);
    const double cloud_stamp_s = occupancy_cloud_stamp_s_.load(
        std::memory_order_acquire);
    if ((sequence & 1u) != 0u || sequence == 0u ||
        !registered_lidar_window_ || !registered_active_window_healthy_ ||
        !registered_current_frame_healthy_ ||
        !std::isfinite(cloud_stamp_s) || !mp_.map_origin_.allFinite() ||
        (mp_.map_voxel_num_.array() <= 0).any() ||
        !std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0.0 ||
        !std::isfinite(mp_.resolution_inv_) ||
        mp_.resolution_inv_ <= 0.0 || mp_.frame_id_.empty())
      return nullptr;
    state->origin = mp_.map_origin_;
    state->dimensions = mp_.map_voxel_num_;
    state->resolution = mp_.resolution_;
    state->resolution_inv = mp_.resolution_inv_;
    state->inflation = mp_.obstacles_inflation_;
    state->virtual_ceiling = mp_.virtual_ceil_height_;
    state->frame_id = mp_.frame_id_;
    state->frame_contract_id = registered_frame_contract_id_;
    state->cloud_stamp_s = cloud_stamp_s;
    state->generation = sequence / 2u;
    state->active_window_generation =
        registered_lidar_window_->activeGeneration();
    state->current_frame_id = registered_lidar_window_->currentFrameId();
    state->current_vehicle_position = md_.camera_pos_;
    state->current_vehicle_clearance_radius_m =
        current_vehicle_clearance_radius_m_;
    state->environment_hits =
        registered_lidar_window_->environmentOccupiedVoxelCenters();
    state->current_hits =
        registered_lidar_window_->currentOccupiedVoxelCenters();
    state->active_obstacle_sources =
        registered_lidar_window_->activeObstacleSources();
    state->current_frame =
        registered_lidar_window_->currentFrameMetadata();
    state->local_evidence_snapshot =
        registered_lidar_window_->captureLocalEvidenceSnapshot(
            state->generation);
    state->support_history.assign(
        registered_support_history_.begin(),
        registered_support_history_.end());
    state->raw_addresses.assign(
        registered_raw_occupied_addresses_.begin(),
        registered_raw_occupied_addresses_.end());
  }
  if (!state->environment_hits || !state->current_frame ||
      state->frame_contract_id.empty())
    return nullptr;

  auto raw_centers = std::make_shared<std::vector<Eigen::Vector3d>>();
  auto raw_keys = std::make_shared<std::vector<iap::VoxelKey>>();
  raw_centers->reserve(state->raw_addresses.size());
  raw_keys->reserve(state->raw_addresses.size());
  const int yz = state->dimensions.y() * state->dimensions.z();
  for (const int address : state->raw_addresses)
  {
    const int x = address / yz;
    const int remainder = address % yz;
    const int y = remainder / state->dimensions.z();
    const int z = remainder % state->dimensions.z();
    raw_keys->push_back(iap::VoxelKey{x, y, z});
    raw_centers->push_back(
        (Eigen::Vector3i(x, y, z).cast<double>() +
         Eigen::Vector3d::Constant(0.5)) * state->resolution +
        state->origin);
  }

  const std::shared_ptr<const SparseFrozenState> frozen = state;
  OccupancyDiagnosticQuery diagnostic_query =
      [frozen](const Eigen::Vector3d &pos)
      {
        OccupancyDiagnostic out;
        out.resolution_m = frozen->resolution;
        out.inflation_m = frozen->inflation;
        out.frame_id = frozen->frame_id;
        out.cloud_stamp_s = frozen->cloud_stamp_s;
        out.generation = frozen->generation;
        if (!pos.allFinite())
          return out;
        for (int axis = 0; axis < 3; ++axis)
          out.voxel_index(axis) = static_cast<int>(std::floor(
              (pos(axis) - frozen->origin(axis)) *
              frozen->resolution_inv));
        for (int axis = 0; axis < 3; ++axis)
          if (out.voxel_index(axis) < 0 ||
              out.voxel_index(axis) >= frozen->dimensions(axis))
          {
            out.source = "position_out_of_map";
            return out;
          }
        const int yz_count = frozen->dimensions.y() *
            frozen->dimensions.z();
        const auto address_of = [&frozen, yz_count](
            const Eigen::Vector3i &index)
        {
          return index.x() * yz_count +
              index.y() * frozen->dimensions.z() + index.z();
        };
        const int address = address_of(out.voxel_index);
        const auto occupied = [&frozen](const int candidate)
        {
          return std::binary_search(
              frozen->raw_addresses.begin(),
              frozen->raw_addresses.end(), candidate);
        };
        out.raw_occupied = occupied(address);
        out.inflated_occupied = out.raw_occupied;
        const int ceiling_index = frozen->virtual_ceiling > -0.5
            ? static_cast<int>(std::floor(
                  (frozen->virtual_ceiling - frozen->origin.z()) *
                  frozen->resolution_inv)) - 1
            : -1;
        if (out.voxel_index.z() == ceiling_index)
          out.inflated_occupied = true;
        if (!out.inflated_occupied)
        {
          const int inflation_xy = static_cast<int>(std::ceil(
              frozen->inflation / frozen->resolution));
          for (int dx = -inflation_xy;
               dx <= inflation_xy && !out.inflated_occupied; ++dx)
            for (int dy = -inflation_xy;
                 dy <= inflation_xy && !out.inflated_occupied; ++dy)
              for (int dz = -1; dz <= 1; ++dz)
              {
                const Eigen::Vector3i source = out.voxel_index +
                    Eigen::Vector3i(dx, dy, dz);
                if ((source.array() < 0).any() ||
                    (source.array() >= frozen->dimensions.array()).any())
                  continue;
                if (occupied(address_of(source)))
                {
                  out.inflated_occupied = true;
                  break;
                }
              }
        }
        out.voxel_center =
            (out.voxel_index.cast<double>() +
             Eigen::Vector3d::Constant(0.5)) * frozen->resolution +
            frozen->origin;
        // Do not turn model-envelope support into ray-observed free space.
        // Preserve only the physically occupied vehicle footprint that the
        // registered-map transaction marks as observed. This avoids copying
        // the full dense observed buffer into every lightweight execution
        // snapshot while retaining the exact current-position safety fact.
        const Eigen::Vector3d cell_min = frozen->origin +
            frozen->resolution * out.voxel_index.cast<double>();
        const Eigen::Vector3d cell_max =
            cell_min + Eigen::Vector3d::Constant(frozen->resolution);
        const Eigen::Vector3d closest =
            frozen->current_vehicle_position.cwiseMax(cell_min).cwiseMin(
                cell_max);
        const bool footprint_observed =
            frozen->current_vehicle_position.allFinite() &&
            std::isfinite(frozen->current_vehicle_clearance_radius_m) &&
            frozen->current_vehicle_clearance_radius_m >= 0.0 &&
            (closest - frozen->current_vehicle_position).squaredNorm() <=
                frozen->current_vehicle_clearance_radius_m *
                frozen->current_vehicle_clearance_radius_m;
        out.observed = out.raw_occupied || out.inflated_occupied ||
            footprint_observed;
        out.state = out.inflated_occupied
            ? GridMapObservationState::OCCUPIED
            : footprint_observed ? GridMapObservationState::OBSERVED_FREE
                                 : GridMapObservationState::UNKNOWN;
        out.available = true;
        out.source = out.raw_occupied ? "registered_sparse_raw" :
            out.inflated_occupied ? "registered_sparse_inflated" :
            footprint_observed ? "current_vehicle_footprint" :
            "registered_sparse_unobserved";
        return out;
      };

  auto epoch = std::make_shared<FrozenOccupancyEpoch>();
  epoch->diagnostic_query = std::move(diagnostic_query);
  epoch->local_evidence_snapshot = state->local_evidence_snapshot;
  epoch->sparse_occupancy_derived_from_raw_centers = true;
  epoch->map_inflation_m = state->inflation;
  epoch->raw_occupied_voxel_centers = std::move(raw_centers);
  epoch->raw_occupied_voxel_keys = std::move(raw_keys);
  epoch->current_frame_occupied_voxel_centers = state->current_hits;
  epoch->active_window_obstacle_sources = state->active_obstacle_sources;
  epoch->environment_occupied_voxel_centers = state->environment_hits;
  epoch->lattice_origin = state->origin;
  epoch->voxel_dimensions = state->dimensions;
  epoch->extent_m = state->dimensions.cast<double>() * state->resolution;
  epoch->resolution_m = state->resolution;
  epoch->virtual_ceiling_height_m = state->virtual_ceiling;
  epoch->frame_id = state->frame_id;
  epoch->geometry_id = geometryIdentity(
      state->frame_id, state->origin, state->dimensions, state->resolution);
  epoch->cloud_stamp_s = state->cloud_stamp_s;
  epoch->generation = state->generation;
  epoch->active_window_generation = state->active_window_generation;
  epoch->current_frame_id = state->current_frame_id;
  epoch->current_frame_content_hash = state->current_frame
      ? state->current_frame->content_hash : std::string{};
  epoch->frame_contract_id = state->frame_contract_id;
  epoch->current_vehicle_position = state->current_vehicle_position;
  epoch->current_vehicle_clearance_radius_m =
      state->current_vehicle_clearance_radius_m;

  constexpr double kPi = 3.14159265358979323846;
  const auto &frame = *state->current_frame;
  auto support = std::make_shared<iap::TrustedLocalMapSupport>();
  support->T_map_sensor = frame.T_map_lidar;
  support->min_range_m = trusted_support_min_range_m_;
  support->max_range_m = trusted_support_max_range_m_;
  support->horizontal_fov_rad =
      trusted_support_horizontal_fov_deg_ * kPi / 180.0;
  support->vertical_min_rad =
      trusted_support_vertical_min_deg_ * kPi / 180.0;
  support->vertical_max_rad =
      trusted_support_vertical_max_deg_ * kPi / 180.0;
  support->stamp_s = frame.scan_end_stamp_s;
  support->valid_until_s = frame.scan_end_stamp_s +
      trusted_support_validity_s_;
  support->sensor_receipt_steady_s =
      static_cast<double>(frame.sensor_receipt_steady_ns) * 1.0e-9;
  support->frame_id = state->frame_id;
  support->model_version = trusted_support_model_version_ +
      ";coverage_history_v1";
  support->retained_min_map = frame.T_map_lidar.translation() -
      Eigen::Vector3d::Constant(trusted_support_max_range_m_);
  support->retained_max_map = frame.T_map_lidar.translation() +
      Eigen::Vector3d::Constant(trusted_support_max_range_m_);
  support->observations.reserve(state->support_history.size());
  for (const auto &historical : state->support_history)
  {
    iap::TrustedLocalMapSupportObservation observation;
    observation.T_map_sensor = historical.T_map_lidar;
    observation.stamp_s = historical.scan_end_stamp_s;
    observation.valid_until_s = historical.scan_end_stamp_s +
        trusted_support_validity_s_;
    observation.sensor_receipt_steady_s =
        static_cast<double>(historical.sensor_receipt_steady_ns) * 1.0e-9;
    observation.retained_min_map = historical.T_map_lidar.translation() -
        Eigen::Vector3d::Constant(trusted_support_max_range_m_);
    observation.retained_max_map = historical.T_map_lidar.translation() +
        Eigen::Vector3d::Constant(trusted_support_max_range_m_);
    support->observations.push_back(std::move(observation));
  }
  if (!support->valid())
    return nullptr;
  epoch->trusted_local_map_support = std::move(support);
  return epoch;
}

std::shared_ptr<const FrozenOccupancyEpoch>
GridMap::captureFrozenOccupancyEpoch() const
{
  // Export and planning share one preparation per generation. Mapping only
  // holds occupancy_epoch_mutex_ for the consistent copy, not index assembly.
  std::lock_guard<std::mutex> freeze_lock(physical_freeze_mutex_);
  struct FrozenBuffers
  {
    Eigen::Vector3d map_origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d map_size = Eigen::Vector3d::Zero();
    Eigen::Vector3i map_voxel_num = Eigen::Vector3i::Zero();
    double resolution = std::numeric_limits<double>::quiet_NaN();
    double resolution_inv = std::numeric_limits<double>::quiet_NaN();
    double inflation = std::numeric_limits<double>::quiet_NaN();
    double virtual_ceiling_height = -1.0;
    double min_occupancy_log = std::numeric_limits<double>::quiet_NaN();
    std::string frame_id;
    double cloud_stamp_s = std::numeric_limits<double>::quiet_NaN();
    uint64_t generation = 0;
    uint64_t active_window_generation = 0;
    int64_t current_frame_id = -1;
    std::string frame_contract_id;
    std::vector<double> fused;
    std::vector<char> inflated;
    std::vector<char> raw_cloud;
    std::vector<char> observed;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> environment_hits;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> raw_centers;
    std::shared_ptr<const std::vector<iap::VoxelKey>> raw_keys;
    std::shared_ptr<const std::vector<Eigen::Vector3d>> current_hits;
    std::shared_ptr<const std::vector<RegisteredLidarObstacleSource>>
        active_obstacle_sources;
    std::optional<RegisteredLidarFrameMetadata> current_registered_frame;
    std::shared_ptr<const LocalEvidenceSnapshot> local_evidence_snapshot;
    std::vector<RegisteredLidarFrameMetadata> support_history;
  };

  auto buffers = std::make_shared<FrozenBuffers>();
  {
    std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
    const uint64_t sequence = occupancy_update_sequence_.load(
        std::memory_order_acquire);
    const double cloud_stamp_s = occupancy_cloud_stamp_s_.load(
        std::memory_order_acquire);
    if ((sequence & 1u) != 0u || sequence == 0u ||
        (registered_lidar_window_enabled_ &&
         (!registered_active_window_healthy_ ||
          !registered_current_frame_healthy_)) ||
        !std::isfinite(cloud_stamp_s) || !mp_.map_origin_.allFinite() ||
        !std::isfinite(mp_.resolution_) || mp_.resolution_ <= 0.0 ||
        !std::isfinite(mp_.resolution_inv_) ||
        mp_.resolution_inv_ <= 0.0 || mp_.frame_id_.empty() ||
        (mp_.map_voxel_num_.array() <= 0).any())
      return nullptr;
    if (cached_physical_epoch_ && cached_physical_epoch_->generation == sequence/2u &&
        cached_physical_epoch_->cloud_stamp_s == cloud_stamp_s &&
        cached_physical_epoch_->frame_id == mp_.frame_id_ &&
        cached_physical_epoch_->lattice_origin == mp_.map_origin_ &&
        cached_physical_epoch_->voxel_dimensions == mp_.map_voxel_num_ &&
        cached_physical_epoch_->resolution_m == mp_.resolution_ &&
        cached_physical_epoch_->extent_m == mp_.map_size_)
      return cached_physical_epoch_;
    const std::size_t nx = static_cast<std::size_t>(mp_.map_voxel_num_(0));
    const std::size_t ny = static_cast<std::size_t>(mp_.map_voxel_num_(1));
    const std::size_t nz = static_cast<std::size_t>(mp_.map_voxel_num_(2));
    if (nx > std::numeric_limits<std::size_t>::max() / ny ||
        nx * ny > std::numeric_limits<std::size_t>::max() / nz)
      return nullptr;
    const std::size_t cell_count = nx * ny * nz;
    if (md_.occupancy_buffer_.size() != cell_count ||
        md_.occupancy_buffer_inflate_.size() != cell_count ||
        md_.occupancy_buffer_raw_cloud_.size() != cell_count ||
        md_.observed_buffer_.size() != cell_count)
      return nullptr;
    buffers->map_origin = mp_.map_origin_;
    buffers->map_size = mp_.map_size_;
    buffers->map_voxel_num = mp_.map_voxel_num_;
    buffers->resolution = mp_.resolution_;
    buffers->resolution_inv = mp_.resolution_inv_;
    buffers->inflation = mp_.obstacles_inflation_;
    buffers->virtual_ceiling_height = mp_.virtual_ceil_height_;
    buffers->min_occupancy_log = mp_.min_occupancy_log_;
    buffers->frame_id = mp_.frame_id_;
    // The legacy depth/cloud path is already expressed directly in the map
    // lattice.  Give that coordinate contract an explicit stable identity so
    // the strict P0 LOS adapter can distinguish it from missing provenance.
    // Registered-window mode replaces this with its externally declared
    // contract below.
    buffers->frame_contract_id =
        "grid_map_native_v1:" + mp_.frame_id_;
    buffers->cloud_stamp_s = cloud_stamp_s;
    buffers->generation = sequence / 2u;
    if (registered_lidar_window_)
    {
      buffers->active_window_generation =
          registered_lidar_window_->activeGeneration();
      buffers->current_frame_id =
          registered_lidar_window_->currentFrameId();
      buffers->frame_contract_id = registered_frame_contract_id_;
      buffers->environment_hits =
          registered_lidar_window_->environmentOccupiedVoxelCenters();
      buffers->current_hits =
          registered_lidar_window_->currentOccupiedVoxelCenters();
      buffers->active_obstacle_sources =
          registered_lidar_window_->activeObstacleSources();
      buffers->current_registered_frame =
          registered_lidar_window_->currentFrameMetadata();
      buffers->local_evidence_snapshot =
          registered_lidar_window_->captureLocalEvidenceSnapshot(
              buffers->generation);
      buffers->support_history.assign(
          registered_support_history_.begin(),
          registered_support_history_.end());
      auto centers = std::make_shared<std::vector<Eigen::Vector3d>>();
      auto keys = std::make_shared<std::vector<iap::VoxelKey>>();
      centers->reserve(registered_raw_occupied_addresses_.size());
      keys->reserve(registered_raw_occupied_addresses_.size());
      const int yz = mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
      for (const int address : registered_raw_occupied_addresses_)
      {
        const int x = address / yz;
        const int remainder = address % yz;
        const int y = remainder / mp_.map_voxel_num_(2);
        const int z = remainder % mp_.map_voxel_num_(2);
        keys->push_back(iap::VoxelKey{x, y, z});
        centers->push_back(
            (Eigen::Vector3i(x, y, z).cast<double>() +
             Eigen::Vector3d::Constant(0.5)) * mp_.resolution_ +
            mp_.map_origin_);
      }
      buffers->raw_centers = std::move(centers);
      buffers->raw_keys = std::move(keys);
    }
    // Registered-window mode has no depth/fused writer by contract. Avoid
    // copying the 59 MiB log-odds layer while holding the occupancy mutex;
    // that copy previously stalled the 10 Hz current-frame callback whenever
    // P0 captured a snapshot.
    if (!registered_lidar_window_enabled_)
      buffers->fused = md_.occupancy_buffer_;
    buffers->inflated = md_.occupancy_buffer_inflate_;
    buffers->raw_cloud = md_.occupancy_buffer_raw_cloud_;
    buffers->observed = md_.observed_buffer_;
  }

  auto centers = buffers->raw_centers
      ? std::make_shared<std::vector<Eigen::Vector3d>>(
            *buffers->raw_centers)
      : std::make_shared<std::vector<Eigen::Vector3d>>();
  auto current_centers = std::make_shared<std::vector<Eigen::Vector3d>>();
  std::ostringstream native_current_content;
  if (!buffers->raw_centers)
  {
    centers->reserve(buffers->raw_cloud.size());
    current_centers->reserve(buffers->raw_cloud.size());
    for (int x = 0; x < buffers->map_voxel_num(0); ++x)
      for (int y = 0; y < buffers->map_voxel_num(1); ++y)
        for (int z = 0; z < buffers->map_voxel_num(2); ++z)
        {
          const std::size_t address =
              static_cast<std::size_t>(x) *
                  static_cast<std::size_t>(buffers->map_voxel_num(1)) *
                  static_cast<std::size_t>(buffers->map_voxel_num(2)) +
              static_cast<std::size_t>(y) *
                  static_cast<std::size_t>(buffers->map_voxel_num(2)) +
              static_cast<std::size_t>(z);
          const bool raw_cloud = buffers->raw_cloud[address] != 0;
          const bool raw_fused = !buffers->fused.empty() &&
              buffers->fused[address] > buffers->min_occupancy_log;
          if (raw_cloud || raw_fused)
          {
            const Eigen::Vector3d center =
                (Eigen::Vector3i(x, y, z).cast<double>() +
                 Eigen::Vector3d::Constant(0.5)) *
                    buffers->resolution + buffers->map_origin;
            centers->push_back(center);
            // The independent PointCloud2 callback clears raw_cloud before
            // every accepted frame.  Those cells therefore have exact
            // current-frame provenance; fused depth cells remain outside
            // this vector and are conservatively uncertified downstream.
            if (raw_cloud)
            {
              current_centers->push_back(center);
              native_current_content << x << ',' << y << ',' << z << ';';
            }
          }
  }
  }

  const std::shared_ptr<const FrozenBuffers> frozen_buffers = buffers;
  auto cells = std::make_shared<FrozenOccupancyCells>();
  cells->flags.resize(buffers->raw_cloud.size());
  const size_t rows = static_cast<size_t>(buffers->map_voxel_num.x()) * buffers->map_voxel_num.y();
  cells->raw_row_offsets.resize(rows + 1, 0);
  for (size_t address = 0; address < cells->flags.size(); ++address) {
    const bool raw = buffers->raw_cloud[address] ||
        (address < buffers->fused.size() && buffers->fused[address] > buffers->min_occupancy_log);
    cells->flags[address] = (raw ? 1 : 0) | (raw && !buffers->raw_cloud[address] ? 8 : 0) | (buffers->inflated[address] ? 2 : 0) |
                           (buffers->observed[address] ? 4 : 0);
    if (raw) { cells->raw_addresses.push_back(address);
      ++cells->raw_row_offsets[address / buffers->map_voxel_num.z() + 1]; }
  }
  for (size_t row=1; row<cells->raw_row_offsets.size(); ++row)
    cells->raw_row_offsets[row] += cells->raw_row_offsets[row-1];
  // Keep one compact immutable physical representation after freezing.
  std::vector<double>().swap(buffers->fused);
  std::vector<char>().swap(buffers->raw_cloud);
  std::vector<char>().swap(buffers->inflated);
  std::vector<char>().swap(buffers->observed);
  auto epoch = std::make_shared<FrozenOccupancyEpoch>();
  epoch->cells = std::move(cells);
  epoch->local_evidence_snapshot = buffers->local_evidence_snapshot;
  epoch->map_inflation_m = frozen_buffers->inflation;
  epoch->raw_occupied_voxel_centers = std::move(centers);
  epoch->raw_occupied_voxel_keys = buffers->raw_keys;
  epoch->current_frame_occupied_voxel_centers =
      buffers->raw_centers ? buffers->current_hits : std::move(current_centers);
  epoch->active_window_obstacle_sources = buffers->active_obstacle_sources;
  epoch->environment_occupied_voxel_centers = buffers->environment_hits
      ? buffers->environment_hits : epoch->raw_occupied_voxel_centers;
  epoch->lattice_origin = frozen_buffers->map_origin;
  epoch->voxel_dimensions = frozen_buffers->map_voxel_num;
  epoch->extent_m = frozen_buffers->map_size;
  epoch->resolution_m = frozen_buffers->resolution;
  epoch->resolution_inv = frozen_buffers->resolution_inv;
  epoch->virtual_ceiling_height_m = buffers->virtual_ceiling_height;
  epoch->frame_id = frozen_buffers->frame_id;
  epoch->geometry_id = geometryIdentity(
      frozen_buffers->frame_id, frozen_buffers->map_origin,
      frozen_buffers->map_voxel_num, frozen_buffers->resolution,
      frozen_buffers->map_size == frozen_buffers->map_voxel_num.cast<double>() * frozen_buffers->resolution
          ? nullptr : &frozen_buffers->map_size);
  epoch->cloud_stamp_s = frozen_buffers->cloud_stamp_s;
  epoch->generation = frozen_buffers->generation;
  epoch->active_window_generation =
      frozen_buffers->active_window_generation;
  epoch->current_frame_id = buffers->raw_centers
      ? frozen_buffers->current_frame_id
      : static_cast<int64_t>(frozen_buffers->generation);
  if (buffers->raw_centers && buffers->current_registered_frame)
  {
    epoch->current_frame_content_hash =
        buffers->current_registered_frame->content_hash;
  }
  else if (!buffers->raw_centers)
  {
    std::uint64_t hash = 1469598103934665603ULL;
    for (const unsigned char byte : native_current_content.str())
    {
      hash ^= byte;
      hash *= 1099511628211ULL;
    }
    std::ostringstream identity;
    identity << "native_cloud_v1:" << std::hex << std::setw(16)
             << std::setfill('0') << hash;
    epoch->current_frame_content_hash = identity.str();
  }
  epoch->frame_contract_id = frozen_buffers->frame_contract_id;
  if (trusted_local_map_support_enabled_ &&
      frozen_buffers->current_registered_frame)
  {
    constexpr double kPi = 3.14159265358979323846;
    const auto &frame = *frozen_buffers->current_registered_frame;
    auto support = std::make_shared<iap::TrustedLocalMapSupport>();
    support->T_map_sensor = frame.T_map_lidar;
    support->min_range_m = trusted_support_min_range_m_;
    support->max_range_m = trusted_support_max_range_m_;
    support->horizontal_fov_rad =
        trusted_support_horizontal_fov_deg_ * kPi / 180.0;
    support->vertical_min_rad =
        trusted_support_vertical_min_deg_ * kPi / 180.0;
    support->vertical_max_rad =
        trusted_support_vertical_max_deg_ * kPi / 180.0;
    support->stamp_s = frame.scan_end_stamp_s;
    support->valid_until_s = frame.scan_end_stamp_s +
        trusted_support_validity_s_;
    support->sensor_receipt_steady_s =
        static_cast<double>(frame.sensor_receipt_steady_ns) * 1.0e-9;
    support->frame_id = frozen_buffers->frame_id;
    support->model_version = trusted_support_model_version_;
    support->retained_min_map = frame.T_map_lidar.translation() -
        Eigen::Vector3d::Constant(trusted_support_max_range_m_);
    support->retained_max_map = frame.T_map_lidar.translation() +
        Eigen::Vector3d::Constant(trusted_support_max_range_m_);
    support->observations.reserve(frozen_buffers->support_history.size());
    for (const auto &historical : frozen_buffers->support_history)
    {
      iap::TrustedLocalMapSupportObservation observation;
      observation.T_map_sensor = historical.T_map_lidar;
      observation.stamp_s = historical.scan_end_stamp_s;
      observation.valid_until_s = historical.scan_end_stamp_s +
          trusted_support_validity_s_;
      observation.sensor_receipt_steady_s =
          static_cast<double>(historical.sensor_receipt_steady_ns) * 1.0e-9;
      observation.retained_min_map =
          historical.T_map_lidar.translation() -
          Eigen::Vector3d::Constant(trusted_support_max_range_m_);
      observation.retained_max_map =
          historical.T_map_lidar.translation() +
          Eigen::Vector3d::Constant(trusted_support_max_range_m_);
      support->observations.push_back(std::move(observation));
    }
    support->model_version += ";coverage_history_v1";
    if (support->valid())
      epoch->trusted_local_map_support = std::move(support);
  }
  // The closure owns only immutable data, without an epoch->closure cycle.
  FrozenOccupancyEpoch query_epoch = *epoch;
  epoch->diagnostic_query = [query_epoch](const Eigen::Vector3d& position) {
    return GridMap::queryFrozenOccupancy(query_epoch, position, true);
  };
  {
    std::lock_guard<std::mutex> lock(occupancy_epoch_mutex_);
    if (occupancy_update_sequence_.load() == epoch->generation*2u)
      cached_physical_epoch_ = epoch;
  }
  return epoch;
}

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

void GridMap::extrinsicCallback(const nav_msgs::msg::Odometry::ConstPtr &odom)
{
  Eigen::Quaterniond cam2body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                     odom->pose.pose.orientation.x,
                                                     odom->pose.pose.orientation.y,
                                                     odom->pose.pose.orientation.z);
  Eigen::Matrix3d cam2body_r_m = cam2body_q.toRotationMatrix();
  md_.cam2body_.block<3, 3>(0, 0) = cam2body_r_m;
  md_.cam2body_(0, 3) = odom->pose.pose.position.x;
  md_.cam2body_(1, 3) = odom->pose.pose.position.y;
  md_.cam2body_(2, 3) = odom->pose.pose.position.z;
  md_.cam2body_(3, 3) = 1.0;
}

void GridMap::depthOdomCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const nav_msgs::msg::Odometry::ConstPtr &odom)
{
  double source_stamp_s = std::numeric_limits<double>::quiet_NaN();
  if (!sourceStampSeconds(img->header.stamp, &source_stamp_s))
    return;

  std::lock_guard<std::mutex> occupancy_lock(occupancy_epoch_mutex_);
  /* get pose */
  Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                 odom->pose.pose.orientation.x,
                                                 odom->pose.pose.orientation.y,
                                                 odom->pose.pose.orientation.z);
  Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();
  Eigen::Matrix4d body2world;
  body2world.block<3, 3>(0, 0) = body_r_m;
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;
  body2world(3, 3) = 1.0;

  Eigen::Matrix4d cam_T = body2world * md_.cam2body_;
  md_.camera_pos_(0) = cam_T(0, 3);
  md_.camera_pos_(1) = cam_T(1, 3);
  md_.camera_pos_(2) = cam_T(2, 3);
  md_.camera_r_m_ = cam_T.block<3, 3>(0, 0);

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  md_.occ_need_update_ = true;
  md_.pending_depth_source_stamp_s_ = source_stamp_s;
  md_.flag_use_depth_fusion = true;
}
