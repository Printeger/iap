#include "plan_env/grid_map.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_set>

#include <sensor_msgs/point_cloud2_iterator.hpp>

// #define current_img_ md_.depth_image_[image_cnt_ & 1]
// #define last_img_ md_.depth_image_[!(image_cnt_ & 1)]

namespace
{
constexpr uint32_t kRiskUnknown = 1u << 6;
constexpr uint32_t kRiskValidPi = 1u << 4;

const sensor_msgs::msg::PointField *findPointField(const sensor_msgs::msg::PointCloud2 &msg,
                                                   const std::string &name)
{
  for (const auto &field : msg.fields)
  {
    if (field.name == name)
    {
      return &field;
    }
  }
  return nullptr;
}

bool hasPointField(const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
{
  return findPointField(msg, name) != nullptr;
}

template <typename T>
bool readScalarAt(const sensor_msgs::msg::PointCloud2 &msg,
                  const sensor_msgs::msg::PointField &field,
                  const std::size_t point_index,
                  T *value)
{
  const std::size_t offset = point_index * msg.point_step + field.offset;
  if (!value || offset + sizeof(T) > msg.data.size())
  {
    return false;
  }
  std::memcpy(value, msg.data.data() + offset, sizeof(T));
  return true;
}

bool readNumericField(const sensor_msgs::msg::PointCloud2 &msg,
                      const sensor_msgs::msg::PointField &field,
                      const std::size_t point_index,
                      double *value)
{
  if (!value)
  {
    return false;
  }

  switch (field.datatype)
  {
  case sensor_msgs::msg::PointField::INT8:
  {
    int8_t v = 0;
    if (!readScalarAt(msg, field, point_index, &v)) return false;
    *value = static_cast<double>(v);
    return true;
  }
  case sensor_msgs::msg::PointField::UINT8:
  {
    uint8_t v = 0;
    if (!readScalarAt(msg, field, point_index, &v)) return false;
    *value = static_cast<double>(v);
    return true;
  }
  case sensor_msgs::msg::PointField::INT16:
  {
    int16_t v = 0;
    if (!readScalarAt(msg, field, point_index, &v)) return false;
    *value = static_cast<double>(v);
    return true;
  }
  case sensor_msgs::msg::PointField::UINT16:
  {
    uint16_t v = 0;
    if (!readScalarAt(msg, field, point_index, &v)) return false;
    *value = static_cast<double>(v);
    return true;
  }
  case sensor_msgs::msg::PointField::INT32:
  {
    int32_t v = 0;
    if (!readScalarAt(msg, field, point_index, &v)) return false;
    *value = static_cast<double>(v);
    return true;
  }
  case sensor_msgs::msg::PointField::UINT32:
  {
    uint32_t v = 0;
    if (!readScalarAt(msg, field, point_index, &v)) return false;
    *value = static_cast<double>(v);
    return true;
  }
  case sensor_msgs::msg::PointField::FLOAT32:
  {
    float v = 0.0f;
    if (!readScalarAt(msg, field, point_index, &v)) return false;
    *value = static_cast<double>(v);
    return true;
  }
  case sensor_msgs::msg::PointField::FLOAT64:
  {
    double v = 0.0;
    if (!readScalarAt(msg, field, point_index, &v)) return false;
    *value = v;
    return true;
  }
  default:
    return false;
  }
}

float nanf()
{
  return std::numeric_limits<float>::quiet_NaN();
}

int overlayAddress(const Eigen::Vector3i &id, const Eigen::Vector3i &dims)
{
  return id(0) * dims(1) * dims(2) + id(1) * dims(2) + id(2);
}

bool overlayContains(const Eigen::Vector3d &pos,
                     const Eigen::Vector3d &min_boundary,
                     const Eigen::Vector3d &max_boundary)
{
  constexpr double kEps = 1.0e-9;
  return pos.allFinite() &&
         (pos.array() >= (min_boundary.array() - kEps)).all() &&
         (pos.array() < (max_boundary.array() - kEps)).all();
}

double finiteOrZero(const double value)
{
  return std::isfinite(value) ? value : 0.0;
}

float packedRgbFloat(const uint8_t r, const uint8_t g, const uint8_t b)
{
  const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                       (static_cast<uint32_t>(g) << 8) |
                       static_cast<uint32_t>(b);
  float packed = 0.0f;
  std::memcpy(&packed, &rgb, sizeof(packed));
  return packed;
}

float heatmapRgbFloat(const double value, const double max_value)
{
  const double denom = std::max(1.0e-6, max_value);
  const double t = std::clamp(std::isfinite(value) ? value / denom : 0.0, 0.0, 1.0);
  const auto lerp = [](const int a, const int b, const double u)
  {
    return static_cast<uint8_t>(std::clamp(static_cast<int>(std::lround(a + (b - a) * u)), 0, 255));
  };
  if (t < 0.5)
  {
    const double u = t * 2.0;
    return packedRgbFloat(lerp(0, 255, u), lerp(140, 230, u), lerp(255, 40, u));
  }
  const double u = (t - 0.5) * 2.0;
  return packedRgbFloat(lerp(255, 255, u), lerp(230, 40, u), lerp(40, 40, u));
}
} // namespace

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
  node_->declare_parameter("grid_map/virtual_ceil_height", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yp", -0.1);
  node_->declare_parameter("grid_map/virtual_ceil_yn", -0.1);
  node_->declare_parameter("grid_map/show_occ_time", false);
  node_->declare_parameter("grid_map/pose_type", 1);
  node_->declare_parameter("grid_map/frame_id", "world");
  node_->declare_parameter("grid_map/local_map_margin", 1);
  node_->declare_parameter("grid_map/ground_height", 1.0);
  node_->declare_parameter("grid_map/odom_depth_timeout", 1.0);
  node_->declare_parameter("risk_overlay/enable", false);
  if (!node_->has_parameter("risk_overlay/use_for_astar"))
  {
    node_->declare_parameter("risk_overlay/use_for_astar", false);
  }
  if (!node_->has_parameter("risk_overlay/use_for_bspline"))
  {
    node_->declare_parameter("risk_overlay/use_for_bspline", false);
  }
  node_->declare_parameter("risk_overlay/topic", std::string("/iap/integrity_front_cost_field"));
  node_->declare_parameter("risk_overlay/stale_timeout_s", 1.0);
  node_->declare_parameter("risk_overlay/lambda_stale", 1.0);
  node_->declare_parameter("risk_overlay/stale_tau_s", 1.0);
  node_->declare_parameter("risk_overlay/lambda_unknown", 10.0);
  node_->declare_parameter("risk_overlay/r_soft", 0.75);
  node_->declare_parameter("risk_overlay/w_soft", 1.0);
  node_->declare_parameter("risk_overlay/w_hard", 10.0);
  node_->declare_parameter("risk_overlay/c_unsafe", 10.0);
  node_->declare_parameter("risk_overlay/eps_al_m", 1.0e-3);
  node_->declare_parameter("risk_overlay/gamma_h", 0.8);
  node_->declare_parameter("risk_overlay/gamma_v", 0.8);
  node_->declare_parameter("risk_overlay/drone_radius_m", 0.35);
  node_->declare_parameter("risk_overlay/safety_buffer_m", 0.20);
  node_->declare_parameter("risk_overlay/clearance_max_m", 5.0);
  node_->declare_parameter("risk_overlay/clearance_unknown_m", 1.0);
  node_->declare_parameter("risk_overlay/clearance_unknown_policy", std::string("conservative_default"));
  node_->declare_parameter("risk_overlay/edge_sample_alpha", 0.75);
  node_->declare_parameter("risk_overlay/debug_publish", true);
  node_->declare_parameter("risk_overlay/debug_topic", std::string("/grid_map/risk_overlay_debug"));
  node_->declare_parameter("risk_overlay/debug_publish_hz", 2.0);
  node_->declare_parameter("risk_overlay/debug_color_mode", std::string("cost"));
  node_->declare_parameter("risk_overlay/debug_cost_max", 50.0);

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
  node_->get_parameter("grid_map/virtual_ceil_height", mp_.virtual_ceil_height_);
  node_->get_parameter("grid_map/virtual_ceil_yp", mp_.virtual_ceil_yp_);
  node_->get_parameter("grid_map/virtual_ceil_yn", mp_.virtual_ceil_yn_);
  node_->get_parameter("grid_map/show_occ_time", mp_.show_occ_time_);
  node_->get_parameter("grid_map/pose_type", mp_.pose_type_);
  node_->get_parameter("grid_map/frame_id", mp_.frame_id_);
  node_->get_parameter("grid_map/local_map_margin", mp_.local_map_margin_);
  node_->get_parameter("grid_map/ground_height", mp_.ground_height_);
  node_->get_parameter("grid_map/odom_depth_timeout", mp_.odom_depth_timeout_);
  node_->get_parameter("risk_overlay/enable", risk_overlay_enabled_);
  node_->get_parameter("risk_overlay/use_for_astar", risk_overlay_use_for_astar_);
  node_->get_parameter("risk_overlay/use_for_bspline", risk_overlay_use_for_bspline_);
  node_->get_parameter("risk_overlay/topic", risk_overlay_topic_);
  node_->get_parameter("risk_overlay/stale_timeout_s", risk_overlay_stale_timeout_s_);
  node_->get_parameter("risk_overlay/lambda_stale", risk_overlay_lambda_stale_);
  node_->get_parameter("risk_overlay/stale_tau_s", risk_overlay_stale_tau_s_);
  node_->get_parameter("risk_overlay/lambda_unknown", risk_overlay_lambda_unknown_);
  node_->get_parameter("risk_overlay/r_soft", risk_overlay_r_soft_);
  node_->get_parameter("risk_overlay/w_soft", risk_overlay_w_soft_);
  node_->get_parameter("risk_overlay/w_hard", risk_overlay_w_hard_);
  node_->get_parameter("risk_overlay/c_unsafe", risk_overlay_c_unsafe_);
  node_->get_parameter("risk_overlay/eps_al_m", risk_overlay_eps_al_m_);
  node_->get_parameter("risk_overlay/gamma_h", risk_overlay_gamma_h_);
  node_->get_parameter("risk_overlay/gamma_v", risk_overlay_gamma_v_);
  node_->get_parameter("risk_overlay/drone_radius_m", risk_overlay_drone_radius_m_);
  node_->get_parameter("risk_overlay/safety_buffer_m", risk_overlay_safety_buffer_m_);
  node_->get_parameter("risk_overlay/clearance_max_m", risk_overlay_clearance_max_m_);
  node_->get_parameter("risk_overlay/clearance_unknown_m", risk_overlay_clearance_unknown_m_);
  node_->get_parameter("risk_overlay/clearance_unknown_policy", risk_overlay_clearance_unknown_policy_);
  node_->get_parameter("risk_overlay/edge_sample_alpha", risk_overlay_edge_sample_alpha_);
  node_->get_parameter("risk_overlay/debug_publish", risk_overlay_debug_publish_);
  node_->get_parameter("risk_overlay/debug_topic", risk_overlay_debug_topic_);
  node_->get_parameter("risk_overlay/debug_publish_hz", risk_overlay_debug_publish_hz_);
  node_->get_parameter("risk_overlay/debug_color_mode", risk_overlay_debug_color_mode_);
  node_->get_parameter("risk_overlay/debug_cost_max", risk_overlay_debug_cost_max_);

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > z_size)
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + z_size;
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
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

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  resetRiskOverlayBuffer();

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  md_.cam2body_ << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  /* init callback */

  // 初始化 message_filters::Subscriber
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

  // 使用独立的里程计和点云订阅
  indep_cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      "grid_map/cloud", 10, std::bind(&GridMap::cloudCallback, this, std::placeholders::_1));

  indep_odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "grid_map/odom", 10, std::bind(&GridMap::odomCallback, this, std::placeholders::_1));

  // 定时器
  occ_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.05),
      std::bind(&GridMap::updateOccupancyCallback, this));

  vis_timer_ = node_->create_wall_timer(
      std::chrono::duration<double>(0.11),
      std::bind(&GridMap::visCallback, this));

  // 发布者
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", 10);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", 10);
  if (risk_overlay_enabled_)
  {
    risk_overlay_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        risk_overlay_topic_, rclcpp::QoS(1).best_effort(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {
          ingestRiskOverlayCloud(*msg);
        });
    RCLCPP_INFO(node_->get_logger(),
                "EGO GridMap risk overlay enabled; topic=%s use_astar=%s use_bspline=%s",
                risk_overlay_topic_.c_str(),
                risk_overlay_use_for_astar_ ? "true" : "false",
                risk_overlay_use_for_bspline_ ? "true" : "false");
    if (risk_overlay_debug_publish_)
    {
      risk_overlay_debug_pub_ =
          node_->create_publisher<sensor_msgs::msg::PointCloud2>(risk_overlay_debug_topic_, rclcpp::QoS(1).best_effort());
      const double publish_hz = std::max(0.1, risk_overlay_debug_publish_hz_);
      risk_overlay_debug_timer_ = node_->create_wall_timer(
          std::chrono::duration<double>(1.0 / publish_hz),
          std::bind(&GridMap::publishRiskOverlayDebug, this));
      RCLCPP_INFO(node_->get_logger(),
                  "EGO GridMap risk overlay debug enabled; topic=%s rate=%.2fHz color_mode=%s",
                  risk_overlay_debug_topic_.c_str(),
                  publish_hz,
                  risk_overlay_debug_color_mode_.c_str());
    }
  }

  md_.occ_need_update_ = false;
  md_.local_updated_ = false;
  md_.has_first_depth_ = false;
  md_.has_odom_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.last_occ_update_time_ = rclcpp::Time(0, 0, RCL_SYSTEM_TIME);

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
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
  resetRiskOverlayBuffer(min_pos, max_pos);
}

void GridMap::resetRiskOverlayBuffer()
{
  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
  RiskOverlayVoxel unknown;
  unknown.flags = kRiskUnknown;
  risk_overlay_buffer_.assign(std::max(0, buffer_size), unknown);
  risk_overlay_stats_.enabled = risk_overlay_enabled_;
  ++risk_overlay_stats_.generation;
}

void GridMap::resetRiskOverlayBuffer(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos)
{
  if (risk_overlay_buffer_.empty())
  {
    return;
  }
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  RiskOverlayVoxel unknown;
  unknown.flags = kRiskUnknown;
  std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        risk_overlay_buffer_[toAddress(x, y, z)] = unknown;
      }
  ++risk_overlay_stats_.generation;
}

void GridMap::resetRiskOverlayIndexBox(Eigen::Vector3i min_id, Eigen::Vector3i max_id)
{
  if (risk_overlay_buffer_.empty())
  {
    return;
  }
  if ((min_id.array() > max_id.array()).any())
  {
    return;
  }
  boundIndex(min_id);
  boundIndex(max_id);

  RiskOverlayVoxel unknown;
  unknown.flags = kRiskUnknown;
  std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        risk_overlay_buffer_[toAddress(x, y, z)] = unknown;
      }
  ++risk_overlay_stats_.generation;
}

RiskOverlayStats GridMap::riskOverlayStats() const
{
  std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
  return risk_overlay_stats_;
}

std::shared_ptr<const RiskOverlaySnapshot> GridMap::riskOverlaySnapshot()
{
  auto snapshot = std::make_shared<RiskOverlaySnapshot>();
  std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
  snapshot->enabled = risk_overlay_enabled_;
  snapshot->map_origin = mp_.map_origin_;
  snapshot->map_min_boundary = mp_.map_min_boundary_;
  snapshot->map_max_boundary = mp_.map_max_boundary_;
  snapshot->map_voxel_num = mp_.map_voxel_num_;
  snapshot->resolution = mp_.resolution_;
  snapshot->resolution_inv = mp_.resolution_inv_;
  snapshot->stamp_now_s = node_ ? node_->now().seconds() : std::numeric_limits<double>::quiet_NaN();
  snapshot->clock_delta_s = risk_overlay_stats_.last_clock_delta_s;
  snapshot->generation = risk_overlay_stats_.generation;
  snapshot->voxels = risk_overlay_buffer_;
  return snapshot;
}

bool GridMap::ingestRiskOverlayCloud(const sensor_msgs::msg::PointCloud2 &msg)
{
  if (!risk_overlay_enabled_)
  {
    return false;
  }
  const std::vector<std::string> required = {"x", "y", "z", "hpl_adv", "vpl_adv", "flags"};
  for (const auto &name : required)
  {
    if (!hasPointField(msg, name))
    {
      std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
      ++risk_overlay_stats_.rejected_frames;
      RCLCPP_WARN(node_->get_logger(), "rejecting risk overlay frame: missing required field '%s'", name.c_str());
      return false;
    }
  }
  const auto *field_x = findPointField(msg, "x");
  const auto *field_y = findPointField(msg, "y");
  const auto *field_z = findPointField(msg, "z");
  const auto *field_hpl = findPointField(msg, "hpl_adv");
  const auto *field_vpl = findPointField(msg, "vpl_adv");
  const auto *field_flags = findPointField(msg, "flags");
  const auto *field_stamp = findPointField(msg, "stamp_s");
  const auto *field_source_age = findPointField(msg, "source_age_s");
  const bool has_stamp_s = field_stamp != nullptr;
  const bool has_source_age_s = field_source_age != nullptr;
  if (!has_stamp_s && !has_source_age_s)
  {
    std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
    ++risk_overlay_stats_.rejected_frames;
    RCLCPP_WARN(node_->get_logger(),
                "rejecting risk overlay frame: missing required field 'stamp_s' and compatible field 'source_age_s'");
    return false;
  }
  if (!msg.header.frame_id.empty() && msg.header.frame_id != mp_.frame_id_)
  {
    std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
    ++risk_overlay_stats_.rejected_frames;
    RCLCPP_WARN(node_->get_logger(), "rejecting risk overlay frame: frame_id '%s' does not match GridMap frame '%s'",
                msg.header.frame_id.c_str(), mp_.frame_id_.c_str());
    return false;
  }

  const double now_s = node_ ? node_->now().seconds() : rclcpp::Time(msg.header.stamp).seconds();
  int samples = 0;
  int written = 0;
  std::unordered_set<int> written_addresses;
  const std::size_t point_count = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);

  std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
  for (std::size_t i = 0; i < point_count; ++i)
  {
    ++samples;
    double x = std::numeric_limits<double>::quiet_NaN();
    double y = std::numeric_limits<double>::quiet_NaN();
    double z = std::numeric_limits<double>::quiet_NaN();
    double hpl = std::numeric_limits<double>::quiet_NaN();
    double vpl = std::numeric_limits<double>::quiet_NaN();
    double flags_value = 0.0;
    if (!readNumericField(msg, *field_x, i, &x) ||
        !readNumericField(msg, *field_y, i, &y) ||
        !readNumericField(msg, *field_z, i, &z) ||
        !readNumericField(msg, *field_hpl, i, &hpl) ||
        !readNumericField(msg, *field_vpl, i, &vpl) ||
        !readNumericField(msg, *field_flags, i, &flags_value))
    {
      continue;
    }

    const Eigen::Vector3d p(x, y, z);
    double sample_stamp_s = std::numeric_limits<double>::quiet_NaN();
    if (field_stamp)
    {
      readNumericField(msg, *field_stamp, i, &sample_stamp_s);
    }
    double source_age_s = std::numeric_limits<double>::quiet_NaN();
    if (field_source_age)
    {
      readNumericField(msg, *field_source_age, i, &source_age_s);
      if (std::isfinite(source_age_s))
      {
        source_age_s = std::max(0.0, source_age_s);
      }
    }
    if (std::isfinite(source_age_s) && std::isfinite(now_s))
    {
      const double source_age_stamp_s = now_s - source_age_s;
      const bool stamp_is_stale = std::isfinite(sample_stamp_s) &&
                                  now_s - sample_stamp_s > risk_overlay_stale_timeout_s_;
      const bool source_age_is_fresh = source_age_s <= risk_overlay_stale_timeout_s_;
      if (!std::isfinite(sample_stamp_s) || (stamp_is_stale && source_age_is_fresh))
      {
        sample_stamp_s = source_age_stamp_s;
      }
    }
    if (!p.allFinite() || !std::isfinite(hpl) || !std::isfinite(vpl) || !std::isfinite(sample_stamp_s))
    {
      continue;
    }
    if (!isInMap(p))
    {
      continue;
    }
    Eigen::Vector3i id;
    posToIndex(p, id);
    const int addr = toAddress(id);
    auto &voxel = risk_overlay_buffer_[addr];
    voxel.hpl_adv = std::isfinite(voxel.hpl_adv) ? std::max(voxel.hpl_adv, static_cast<float>(hpl))
                                                  : static_cast<float>(hpl);
    voxel.vpl_adv = std::isfinite(voxel.vpl_adv) ? std::max(voxel.vpl_adv, static_cast<float>(vpl))
                                                  : static_cast<float>(vpl);
    voxel.stamp_s = std::isfinite(voxel.stamp_s) ? std::max(voxel.stamp_s, sample_stamp_s) : sample_stamp_s;
    voxel.age_s = std::isfinite(now_s) && std::isfinite(voxel.stamp_s)
                      ? static_cast<float>(std::max(0.0, now_s - voxel.stamp_s))
                      : nanf();
    voxel.flags = (voxel.flags & ~kRiskUnknown) |
                  static_cast<uint32_t>(std::max(0.0, flags_value));
    bool stale = false;
    voxel.pi_cost = static_cast<float>(riskOverlayPiCost(p, voxel.hpl_adv, voxel.vpl_adv, 0.0, false, &stale));
    voxel.flags |= kRiskValidPi;
    if (written_addresses.insert(addr).second)
    {
      ++written;
    }
  }

  risk_overlay_stats_.samples_received += samples;
  risk_overlay_stats_.written_voxels += written;
  risk_overlay_stats_.enabled = risk_overlay_enabled_;
  risk_overlay_stats_.last_clock_delta_s =
      std::isfinite(now_s) ? now_s - rclcpp::Time(msg.header.stamp).seconds() : std::numeric_limits<double>::quiet_NaN();
  ++risk_overlay_stats_.generation;
  return written > 0;
}

double GridMap::rawOccupancyClearance(const Eigen::Vector3d &pos)
{
  if (!pos.allFinite() || !isInMap(pos))
  {
    return risk_overlay_clearance_unknown_m_;
  }
  Eigen::Vector3i center;
  posToIndex(pos, center);
  if (getOccupancy(center) == 1)
  {
    return 0.0;
  }
  const double max_clearance = std::max(0.0, risk_overlay_clearance_max_m_);
  const int max_step = std::max(1, static_cast<int>(std::ceil(max_clearance * mp_.resolution_inv_)));
  double best2 = max_clearance * max_clearance;
  for (int dx = -max_step; dx <= max_step; ++dx)
    for (int dy = -max_step; dy <= max_step; ++dy)
      for (int dz = -max_step; dz <= max_step; ++dz)
      {
        Eigen::Vector3i id = center + Eigen::Vector3i(dx, dy, dz);
        if (!isInMap(id) || getOccupancy(id) != 1)
        {
          continue;
        }
        Eigen::Vector3d p;
        indexToPos(id, p);
        best2 = std::min(best2, (p - pos).squaredNorm());
      }
  return std::min(max_clearance, std::sqrt(best2));
}

double GridMap::riskOverlayHal(const Eigen::Vector3d &pos)
{
  const double clearance = rawOccupancyClearance(pos);
  return risk_overlay_gamma_h_ *
         std::max(clearance - risk_overlay_drone_radius_m_ - risk_overlay_safety_buffer_m_, 0.0);
}

double GridMap::riskOverlayVal(const Eigen::Vector3d &pos)
{
  if (!pos.allFinite())
  {
    return 0.0;
  }
  const double lower = pos.z() - mp_.map_min_boundary_.z();
  const double upper = mp_.map_max_boundary_.z() - pos.z();
  return risk_overlay_gamma_v_ * std::max(std::min(lower, upper), 0.0);
}

double GridMap::riskOverlayPiCost(const Eigen::Vector3d &pos,
                                  const double hpl_adv,
                                  const double vpl_adv,
                                  const double age_s,
                                  const bool unknown,
                                  bool *stale)
{
  const bool is_stale = std::isfinite(age_s) && age_s > risk_overlay_stale_timeout_s_;
  if (stale)
  {
    *stale = is_stale;
  }
  double cost = 0.0;
  if (!unknown && std::isfinite(hpl_adv) && std::isfinite(vpl_adv))
  {
    const double hal = riskOverlayHal(pos);
    const double val = riskOverlayVal(pos);
    const double eps = std::max(1.0e-9, risk_overlay_eps_al_m_);
    const double r_h = hpl_adv / (hal + eps);
    const double r_v = vpl_adv / (val + eps);
    const double r = std::max(r_h, r_v);
    if (r < risk_overlay_r_soft_)
    {
      cost = 0.0;
    }
    else if (r < 1.0)
    {
      cost = risk_overlay_w_soft_ * (r - risk_overlay_r_soft_) * (r - risk_overlay_r_soft_);
    }
    else
    {
      cost = risk_overlay_w_hard_ * (r - 1.0) * (r - 1.0) + risk_overlay_c_unsafe_;
    }
  }
  if (is_stale)
  {
    const double tau = std::max(1.0e-6, risk_overlay_stale_tau_s_);
    cost += risk_overlay_lambda_stale_ * (1.0 - std::exp(-std::max(0.0, age_s) / tau));
  }
  if (unknown)
  {
    cost += risk_overlay_lambda_unknown_;
  }
  return std::max(0.0, cost);
}

RiskOverlayQuery GridMap::queryRiskInterpolatedLocked(const Eigen::Vector3d &pos,
                                                      const std::vector<RiskOverlayVoxel> &buffer,
                                                      const double now_s,
                                                      const int generation)
{
  RiskOverlaySnapshot snapshot;
  snapshot.enabled = risk_overlay_enabled_;
  snapshot.map_origin = mp_.map_origin_;
  snapshot.map_min_boundary = mp_.map_min_boundary_;
  snapshot.map_max_boundary = mp_.map_max_boundary_;
  snapshot.map_voxel_num = mp_.map_voxel_num_;
  snapshot.resolution = mp_.resolution_;
  snapshot.resolution_inv = mp_.resolution_inv_;
  snapshot.stamp_now_s = now_s;
  snapshot.clock_delta_s = risk_overlay_stats_.last_clock_delta_s;
  snapshot.generation = generation;
  snapshot.voxels = buffer;
  return queryRiskInterpolatedFromSnapshot(snapshot, pos);
}

RiskOverlayQuery GridMap::queryRiskInterpolatedFromSnapshot(const RiskOverlaySnapshot &snapshot,
                                                           const Eigen::Vector3d &pos)
{
  RiskOverlayQuery out;
  out.position = pos;
  out.generation = snapshot.generation;
  if (!snapshot.enabled || !overlayContains(pos, snapshot.map_min_boundary, snapshot.map_max_boundary) ||
      snapshot.voxels.empty() || (snapshot.map_voxel_num.array() <= 0).any())
  {
    out.cost = risk_overlay_lambda_unknown_;
    out.flags = kRiskUnknown;
    return out;
  }

  const Eigen::Array3d grid = (pos - snapshot.map_origin).array() * snapshot.resolution_inv - 0.5;
  Eigen::Vector3i base;
  Eigen::Vector3d frac;
  for (int axis = 0; axis < 3; ++axis)
  {
    const double clamped = std::clamp(grid(axis), 0.0, static_cast<double>(std::max(0, snapshot.map_voxel_num(axis) - 1)));
    const int lower = static_cast<int>(std::floor(clamped));
    base(axis) = std::clamp(lower, 0, std::max(0, snapshot.map_voxel_num(axis) - 2));
    frac(axis) = snapshot.map_voxel_num(axis) > 1 ? std::clamp(clamped - static_cast<double>(base(axis)), 0.0, 1.0) : 0.0;
  }

  bool unknown = false;
  uint32_t flags = 0u;
  double hpl = 0.0;
  double vpl = 0.0;
  double min_stamp_s = std::numeric_limits<double>::infinity();

  for (int dx = 0; dx <= 1; ++dx)
    for (int dy = 0; dy <= 1; ++dy)
      for (int dz = 0; dz <= 1; ++dz)
      {
        Eigen::Vector3i id = base + Eigen::Vector3i(dx, dy, dz);
        for (int axis = 0; axis < 3; ++axis)
        {
          id(axis) = std::clamp(id(axis), 0, snapshot.map_voxel_num(axis) - 1);
        }
        const double wx = dx ? frac.x() : 1.0 - frac.x();
        const double wy = dy ? frac.y() : 1.0 - frac.y();
        const double wz = dz ? frac.z() : 1.0 - frac.z();
        const double w = wx * wy * wz;
        const auto &voxel = snapshot.voxels[overlayAddress(id, snapshot.map_voxel_num)];
        if (w <= 1.0e-9)
        {
          continue;
        }
        flags |= voxel.flags;
        const bool corner_unknown = (voxel.flags & kRiskUnknown) != 0u ||
                                    !std::isfinite(voxel.hpl_adv) ||
                                    !std::isfinite(voxel.vpl_adv);
        unknown = unknown || corner_unknown;
        if (std::isfinite(voxel.stamp_s))
        {
          min_stamp_s = std::min(min_stamp_s, voxel.stamp_s);
        }
        hpl += w * finiteOrZero(voxel.hpl_adv);
        vpl += w * finiteOrZero(voxel.vpl_adv);
      }

  out.unknown = unknown;
  out.hpl_adv = unknown ? std::numeric_limits<double>::quiet_NaN() : hpl;
  out.vpl_adv = unknown ? std::numeric_limits<double>::quiet_NaN() : vpl;
  out.hal = riskOverlayHal(pos);
  out.val = riskOverlayVal(pos);
  out.age_s = std::isfinite(snapshot.stamp_now_s) && std::isfinite(min_stamp_s)
                  ? std::max(0.0, snapshot.stamp_now_s - min_stamp_s)
                  : std::numeric_limits<double>::quiet_NaN();
  out.sample_stamp_s = std::isfinite(min_stamp_s) ? min_stamp_s : std::numeric_limits<double>::quiet_NaN();
  out.clock_delta_s = snapshot.clock_delta_s;
  out.cost = riskOverlayPiCost(pos, out.hpl_adv, out.vpl_adv, out.age_s, out.unknown, &out.stale);
  out.valid = !out.unknown && std::isfinite(out.cost);
  out.flags = flags;
  if (out.unknown)
  {
    out.flags |= kRiskUnknown;
  }
  return out;
}

RiskOverlayQuery GridMap::queryRiskInterpolated(const std::shared_ptr<const RiskOverlaySnapshot> &snapshot,
                                                const Eigen::Vector3d &pos)
{
  if (!snapshot)
  {
    RiskOverlayQuery out;
    out.position = pos;
    out.cost = risk_overlay_lambda_unknown_;
    out.flags = kRiskUnknown;
    return out;
  }
  return queryRiskInterpolatedFromSnapshot(*snapshot, pos);
}

RiskOverlayQuery GridMap::queryRiskInterpolated(const Eigen::Vector3d &pos)
{
  const double now_s = node_ ? node_->now().seconds() : std::numeric_limits<double>::quiet_NaN();
  std::lock_guard<std::mutex> lock(risk_overlay_mutex_);
  auto out = queryRiskInterpolatedLocked(pos, risk_overlay_buffer_, now_s, risk_overlay_stats_.generation);
  ++risk_overlay_stats_.query_count;
  if (out.valid)
  {
    ++risk_overlay_stats_.query_hit_count;
  }
  if (out.unknown)
  {
    ++risk_overlay_stats_.query_unknown_count;
  }
  if (out.stale)
  {
    ++risk_overlay_stats_.query_stale_count;
  }
  return out;
}

Eigen::Vector3d GridMap::queryRiskGradient(const Eigen::Vector3d &pos)
{
  return queryRiskGradient(riskOverlaySnapshot(), pos);
}

Eigen::Vector3d GridMap::queryRiskGradient(const std::shared_ptr<const RiskOverlaySnapshot> &snapshot,
                                           const Eigen::Vector3d &pos)
{
  const double step = std::max(0.05, mp_.resolution_);
  Eigen::Vector3d grad = Eigen::Vector3d::Zero();
  for (int axis = 0; axis < 3; ++axis)
  {
    Eigen::Vector3d delta = Eigen::Vector3d::Zero();
    delta(axis) = step;
    const auto plus = queryRiskInterpolated(snapshot, pos + delta);
    const auto minus = queryRiskInterpolated(snapshot, pos - delta);
    if (!std::isfinite(plus.cost) || !std::isfinite(minus.cost))
    {
      return Eigen::Vector3d::Zero();
    }
    grad(axis) = (plus.cost - minus.cost) / (2.0 * step);
  }
  return grad;
}

bool GridMap::integrateRiskOnEdge(const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, double *mean_cost)
{
  return integrateRiskOnEdge(riskOverlaySnapshot(), p0, p1, mean_cost);
}

bool GridMap::integrateRiskOnEdge(const std::shared_ptr<const RiskOverlaySnapshot> &snapshot,
                                  const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, double *mean_cost)
{
  return integrateRiskOnEdge(snapshot, p0, p1, mean_cost, nullptr);
}

bool GridMap::integrateRiskOnEdge(const std::shared_ptr<const RiskOverlaySnapshot> &snapshot,
                                  const Eigen::Vector3d &p0, const Eigen::Vector3d &p1, double *mean_cost,
                                  RiskOverlayEdgeStats *edge_stats)
{
  if (!mean_cost)
  {
    return false;
  }
  *mean_cost = 0.0;
  if (!snapshot || !snapshot->enabled || !p0.allFinite() || !p1.allFinite())
  {
    return false;
  }
  if (edge_stats)
  {
    *edge_stats = RiskOverlayEdgeStats();
  }
  const double length = (p1 - p0).norm();
  const double alpha = std::clamp(risk_overlay_edge_sample_alpha_, 0.25, 2.0);
  const int n = std::max(2, static_cast<int>(std::ceil(length / (alpha * std::max(1.0e-6, snapshot->resolution)))));
  double sum = 0.0;
  int used = 0;
  for (int i = 0; i < n; ++i)
  {
    const double t = n == 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    const Eigen::Vector3d p = (1.0 - t) * p0 + t * p1;
    const auto q = queryRiskInterpolated(snapshot, p);
    if (edge_stats)
    {
      ++edge_stats->samples;
      if (q.valid)
      {
        ++edge_stats->hit_count;
      }
      if (q.unknown)
      {
        ++edge_stats->unknown_count;
      }
      if (q.stale)
      {
        ++edge_stats->stale_count;
      }
    }
    if (std::isfinite(q.cost))
    {
      sum += q.cost;
      ++used;
    }
  }
  if (used == 0)
  {
    return false;
  }
  *mean_cost = sum / static_cast<double>(used);
  return true;
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  int idx_ctns = toAddress(id);

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
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
    {

      for (int y = min_cut_m(1); y < min_cut(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
    {

      for (int x = min_cut_m(0); x < min_cut(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }

      for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x)
      {
        int idx = toAddress(x, y, z);
        md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
      }
    }

  resetRiskOverlayIndexBox(Eigen::Vector3i(min_cut_m(0), min_cut_m(1), min_cut_m(2)),
                           Eigen::Vector3i(max_cut_m(0), max_cut_m(1), min_cut(2) - 1));
  resetRiskOverlayIndexBox(Eigen::Vector3i(min_cut_m(0), min_cut_m(1), max_cut(2) + 1),
                           Eigen::Vector3i(max_cut_m(0), max_cut_m(1), max_cut_m(2)));
  resetRiskOverlayIndexBox(Eigen::Vector3i(min_cut_m(0), min_cut_m(1), min_cut_m(2)),
                           Eigen::Vector3i(max_cut_m(0), min_cut(1) - 1, max_cut_m(2)));
  resetRiskOverlayIndexBox(Eigen::Vector3i(min_cut_m(0), max_cut(1) + 1, min_cut_m(2)),
                           Eigen::Vector3i(max_cut_m(0), max_cut_m(1), max_cut_m(2)));
  resetRiskOverlayIndexBox(Eigen::Vector3i(min_cut_m(0), min_cut_m(1), min_cut_m(2)),
                           Eigen::Vector3i(min_cut(0) - 1, max_cut_m(1), max_cut_m(2)));
  resetRiskOverlayIndexBox(Eigen::Vector3i(max_cut(0) + 1, min_cut_m(1), min_cut_m(2)),
                           Eigen::Vector3i(max_cut_m(0), max_cut_m(1), max_cut_m(2)));

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
  if (md_.last_occ_update_time_.seconds() < 1.0)
    md_.last_occ_update_time_ = node_->now();

  if (!md_.occ_need_update_)
  {
    if (md_.flag_use_depth_fusion &&
        (node_->now() - md_.last_occ_update_time_).seconds() > mp_.odom_depth_timeout_)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "odom or depth lost! now=%f, last_occ_update_time=%f, odom_depth_timeout=%f",
                   node_->now().seconds(),
                   md_.last_occ_update_time_.seconds(),
                   mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return;
  }
  md_.last_occ_update_time_ = node_->now();

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  projectDepthImage();
  // t2 = ros::Time::now();
  raycastProcess();
  // t3 = ros::Time::now();

  if (md_.local_updated_)
    clearAndInflateLocalMap();

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
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstPtr &img,
                                const geometry_msgs::msg::PoseStamped::ConstPtr &pose)
{
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
  }
  else
  {
    md_.occ_need_update_ = false;
  }

  md_.flag_use_depth_fusion = true;
}

void GridMap::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  if (md_.has_first_depth_)
    return;

  md_.camera_pos_(0) = odom->pose.pose.position.x;
  md_.camera_pos_(1) = odom->pose.pose.position.y;
  md_.camera_pos_(2) = odom->pose.pose.position.z;

  md_.has_odom_ = true;
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &img)
{

  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);

  md_.has_cloud_ = true;

  if (!md_.has_odom_)
  {
    std::cout << "no odom!" << std::endl;
    return;
  }

  if (latest_cloud.points.size() == 0)
    return;

  if (isnan(md_.camera_pos_(0)) || isnan(md_.camera_pos_(1)) || isnan(md_.camera_pos_(2)))
    return;

  this->resetBuffer(md_.camera_pos_ - mp_.local_update_range_,
                    md_.camera_pos_ + mp_.local_update_range_);

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

      /* inflate the point */
      // 点云膨胀
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
        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.min_occupancy_log_)
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

void GridMap::publishRiskOverlayDebug()
{
  if (!risk_overlay_enabled_ || !risk_overlay_debug_publish_ || !risk_overlay_debug_pub_ ||
      risk_overlay_debug_pub_->get_subscription_count() <= 0)
  {
    return;
  }

  const auto snapshot = riskOverlaySnapshot();
  if (!snapshot || !snapshot->enabled || snapshot->voxels.empty())
  {
    return;
  }

  struct DebugPoint
  {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float rgb = 0.0f;
    float cost = 0.0f;
    float hpl_adv = nanf();
    float vpl_adv = nanf();
    float hal = nanf();
    float val = nanf();
    float age_s = nanf();
    float flags = 0.0f;
    float generation = 0.0f;
    float valid = 0.0f;
    float unknown = 0.0f;
    float stale = 0.0f;
  };

  std::vector<DebugPoint> points;
  points.reserve(std::min<std::size_t>(snapshot->voxels.size(), 50000));

  double sum_cost = 0.0;
  double max_cost = 0.0;
  int valid_count = 0;
  int unknown_count = 0;
  int stale_count = 0;

  for (int x = 0; x < snapshot->map_voxel_num.x(); ++x)
    for (int y = 0; y < snapshot->map_voxel_num.y(); ++y)
      for (int z = 0; z < snapshot->map_voxel_num.z(); ++z)
      {
        const Eigen::Vector3i id(x, y, z);
        const auto &voxel = snapshot->voxels[overlayAddress(id, snapshot->map_voxel_num)];
        if ((voxel.flags & kRiskUnknown) != 0u ||
            !std::isfinite(voxel.hpl_adv) ||
            !std::isfinite(voxel.vpl_adv))
        {
          continue;
        }

        Eigen::Vector3d pos;
        indexToPos(id, pos);
        const auto query = queryRiskInterpolated(snapshot, pos);

        DebugPoint point;
        point.x = static_cast<float>(pos.x());
        point.y = static_cast<float>(pos.y());
        point.z = static_cast<float>(pos.z());
        point.cost = static_cast<float>(std::isfinite(query.cost) ? query.cost : 0.0);
        point.hpl_adv = static_cast<float>(query.hpl_adv);
        point.vpl_adv = static_cast<float>(query.vpl_adv);
        point.hal = static_cast<float>(query.hal);
        point.val = static_cast<float>(query.val);
        point.age_s = static_cast<float>(query.age_s);
        point.flags = static_cast<float>(query.flags);
        point.generation = static_cast<float>(query.generation);
        point.valid = query.valid ? 1.0f : 0.0f;
        point.unknown = query.unknown ? 1.0f : 0.0f;
        point.stale = query.stale ? 1.0f : 0.0f;

        double color_value = query.cost;
        if (risk_overlay_debug_color_mode_ == "hpl_adv")
        {
          color_value = query.hpl_adv;
        }
        else if (risk_overlay_debug_color_mode_ == "vpl_adv")
        {
          color_value = query.vpl_adv;
        }
        point.rgb = heatmapRgbFloat(color_value, risk_overlay_debug_cost_max_);

        if (std::isfinite(query.cost))
        {
          sum_cost += query.cost;
          max_cost = std::max(max_cost, query.cost);
        }
        if (query.valid)
        {
          ++valid_count;
        }
        if (query.unknown)
        {
          ++unknown_count;
        }
        if (query.stale)
        {
          ++stale_count;
        }
        points.push_back(point);
      }

  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = mp_.frame_id_;
  const rclcpp::Time stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
  const int64_t stamp_ns = stamp.nanoseconds();
  msg.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000ll);
  msg.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000ll);
  msg.height = 1;
  msg.width = static_cast<uint32_t>(points.size());
  msg.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(
      15,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::msg::PointField::FLOAT32,
      "cost", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "vpl_adv", 1, sensor_msgs::msg::PointField::FLOAT32,
      "hal", 1, sensor_msgs::msg::PointField::FLOAT32,
      "val", 1, sensor_msgs::msg::PointField::FLOAT32,
      "age_s", 1, sensor_msgs::msg::PointField::FLOAT32,
      "flags", 1, sensor_msgs::msg::PointField::FLOAT32,
      "generation", 1, sensor_msgs::msg::PointField::FLOAT32,
      "valid", 1, sensor_msgs::msg::PointField::FLOAT32,
      "unknown", 1, sensor_msgs::msg::PointField::FLOAT32,
      "stale", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_rgb(msg, "rgb");
  sensor_msgs::PointCloud2Iterator<float> iter_cost(msg, "cost");
  sensor_msgs::PointCloud2Iterator<float> iter_hpl(msg, "hpl_adv");
  sensor_msgs::PointCloud2Iterator<float> iter_vpl(msg, "vpl_adv");
  sensor_msgs::PointCloud2Iterator<float> iter_hal(msg, "hal");
  sensor_msgs::PointCloud2Iterator<float> iter_val(msg, "val");
  sensor_msgs::PointCloud2Iterator<float> iter_age(msg, "age_s");
  sensor_msgs::PointCloud2Iterator<float> iter_flags(msg, "flags");
  sensor_msgs::PointCloud2Iterator<float> iter_generation(msg, "generation");
  sensor_msgs::PointCloud2Iterator<float> iter_valid(msg, "valid");
  sensor_msgs::PointCloud2Iterator<float> iter_unknown(msg, "unknown");
  sensor_msgs::PointCloud2Iterator<float> iter_stale(msg, "stale");

  for (const auto &point : points)
  {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    *iter_rgb = point.rgb;
    *iter_cost = point.cost;
    *iter_hpl = point.hpl_adv;
    *iter_vpl = point.vpl_adv;
    *iter_hal = point.hal;
    *iter_val = point.val;
    *iter_age = point.age_s;
    *iter_flags = point.flags;
    *iter_generation = point.generation;
    *iter_valid = point.valid;
    *iter_unknown = point.unknown;
    *iter_stale = point.stale;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_rgb;
    ++iter_cost;
    ++iter_hpl;
    ++iter_vpl;
    ++iter_hal;
    ++iter_val;
    ++iter_age;
    ++iter_flags;
    ++iter_generation;
    ++iter_valid;
    ++iter_unknown;
    ++iter_stale;
  }

  risk_overlay_debug_pub_->publish(msg);

  static int debug_publish_count = 0;
  if (++debug_publish_count % 10 == 0)
  {
    const double mean_cost = points.empty() ? 0.0 : sum_cost / static_cast<double>(points.size());
    RCLCPP_INFO(node_->get_logger(),
                "risk_overlay_debug generation=%d points=%zu valid=%d unknown=%d stale=%d mean_cost=%.3f max_cost=%.3f",
                snapshot->generation,
                points.size(),
                valid_count,
                unknown_count,
                stale_count,
                mean_cost,
                max_cost);
  }
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
  md_.flag_use_depth_fusion = true;
}
